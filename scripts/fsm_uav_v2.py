#!/usr/bin/env python

import rospy
import rosnode
import smach
import smach_ros
from smach import Concurrence
from sensor_msgs.msg import PointCloud2
from mrs_msgs.msg import UavStatus, UavManagerDiagnostics, HwApiRcChannels, ControlManagerDiagnostics
from std_msgs.msg import Bool, String, UInt8MultiArray
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger, TriggerRequest
from mrs_msgs.srv import Vec4, Vec4Request
import tf2_ros
import tf2_geometry_msgs
import utm
import numpy as np
import math

# --- Define FSM States ---

class Init(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['initialized', 'failed'])

        # Get swarm list from parameter server
        self.drone_list = rospy.get_param('~uav_names', [])
        self.drone_status = {name: False for name in self.drone_list}

        # Get computer name list from parameter server
        self.computer_name = rospy.get_param("~computer_name", "computer1")

    # Subscribe to GNSS
    def uav_status_callback(self, msg):
        if msg.hw_api_gnss_ok == True:
            if not self.logged_hw_api_gnss:
                self.hw_api_gnss_ok = True
                rospy.loginfo("[INIT]: GNSS data received.")
                self.logged_hw_api_gnss = True

    # Subscribe to 3D Lidar
    def lidar_3d_callback(self, msg):
        if msg.width > 0 and msg.height > 0:
            if not self.logged_lidar_3d:
                self.lidar_3d_ok = True
                rospy.loginfo("[INIT]: 3D LiDAR data received.")
                self.logged_lidar_3d = True

    # Subscribe to communication
    def make_callback(self, name):
        def callback(msg):
            if msg.uav_name in self.drone_list:
                if not self.drone_status[name]:
                    rospy.loginfo(f"{name}: OK")
                    self.drone_status[name] = True
        return callback

    # Subscribe to can take-off
    def can_take_callback(self, msg):
        if msg.data and not self.logged_can_take:
            self.can_take_ok = True
            self.logged_can_take = True
            # rospy.loginfo("State: PRE_FLIGHT - Can takeoff command received.")

    # Subscribe to radio controller
    def rc_callback(self, msg):
        if msg.channels[4] == 0.0:
            self.rc_arm = False
        else:
            self.rc_arm = True
        if msg.channels[10] == 0.0:
            self.rc_offboard = False
        else:
            self.rc_offboard = True

    # Subscriber to computer
    def swarm_init_cb(self, msg):
        if msg.data:
            rospy.loginfo("[INIT]: Received swarm init trigger.")
            self.swarm_init_received = True

    def execute(self, userdata):
        rospy.loginfo("[INIT]: Initializing drone systems.")

        # Flags
        self.hw_api_gnss_ok = False
        self.logged_hw_api_gnss = False
        self.lidar_3d_ok = False
        self.logged_lidar_3d = False
        self.swarm_communication = False
        self.logged_can_take = False
        self.can_take_ok = False
        self.rc_arm = False
        self.rc_offboard = False
        self.swarm_init_received = False

        # Publisher of status 
        self.pub = rospy.Publisher('fms/drone_status_report', UInt8MultiArray, queue_size=10)

        # Subscribe to base's trigger
        rospy.Subscriber('/' + self.computer_name + '/fms/init', Bool, self.swarm_init_cb)

        # Subscribe to sensors
        rospy.Subscriber('mrs_uav_status/uav_status', UavStatus, self.uav_status_callback)
        rospy.Subscriber('lslidar/pcl_filtered', PointCloud2, self.lidar_3d_callback)

        # Subscriber automatic start
        rospy.Subscriber('automatic_start/can_takeoff', Bool, self.can_take_callback)

        # Subscriber Radio Controller
        rospy.Subscriber('hw_api/rc_channels', HwApiRcChannels, self.rc_callback)

        # Dynamically subscribe to each drone's status
        self.subscribers = []
        for name in self.drone_list:
            # rospy.loginfo(f"UAV: : {name}")
            topic = f"/{name}/uav_manager/diagnostics"
            sub = rospy.Subscriber(topic, UavManagerDiagnostics, self.make_callback(name))
            self.subscribers.append(sub)

        # Send heartbeat every 1 second
        heartbeat_rate = rospy.Rate(1)

        while not rospy.is_shutdown() and not self.swarm_init_received:
            rospy.loginfo_throttle(5, "[INIT]: Sending INIT heartbeat to base.")
            rospy.logwarn("[INIT]: gnss=%s, lidar_3d=%s, swarm=%s, pre-flight=%s, armed=%s, offboard:%s",
                        self.hw_api_gnss_ok, self.lidar_3d_ok, self.swarm_communication, 
                        self.can_take_ok, self.rc_arm, self.rc_offboard)

            if all(self.drone_status.values()):
                self.swarm_communication = True

            status = UInt8MultiArray()
            # 1 for True, 0 for False
            status.data = [
                int(self.hw_api_gnss_ok),
                int(self.lidar_3d_ok),
                int(self.swarm_communication),
                int(self.can_take_ok),
                int(self.rc_arm),
                int(self.rc_offboard)
            ]

            self.pub.publish(status)

            heartbeat_rate.sleep()

        return 'initialized'


class CommLeader(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['initialized', 'failed'])
        self.drone_list = rospy.get_param('~uav_names', [])
        self.uav_name = rospy.get_param("~uav_name", "uav1")
        self.height_formation = rospy.get_param("~height_formation", 3.0)
        self.tolerance_distance = rospy.get_param("~tolerance_distance", 0.5)
        self.latitude_start = 41.2209807
        self.longitude_start = -8.5272058
        self.latitude_end = 41.2211611
        self.longitude_end = -8.5272058
        self.drone_flying = {name: False for name in self.drone_list}
        self.takeoff_execution = False

        # Wait for TFs
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)
        rospy.loginfo("[LEADER]: Waiting for TF to fill buffer...")
        rospy.sleep(2.0) 

    def make_callback(self, name):
        def callback(msg):
            if msg.uav_name in self.drone_list:
                self.drone_flying[name] = msg.flying_normally
        return callback

    def make_callback_odom(self, name):
        def callback(msg):
            self.odom = msg
        return callback

    def init_service_proxies(self):
        # TakeOff Services
        for name in self.drone_list:
            service_name = f'/{name}/uav_manager/takeoff'
            rospy.loginfo(f"[LEADER]: Waiting for take-off service: {service_name}")
            rospy.wait_for_service(service_name)
            self.uav_takeoff_services[name] = rospy.ServiceProxy(service_name, Trigger)
        # GoTo Services
        for name in self.drone_list:
            service_name = f'/{name}/control_manager/goto'
            rospy.loginfo(f"[LEADER]: Waiting for goto service: {service_name}")
            rospy.wait_for_service(service_name)
            self.uav_goto_services[name] = rospy.ServiceProxy(service_name, Vec4)
        # GoTo_fcu Services
        for name in self.drone_list:
            service_name = f'/{name}/control_manager/goto_fcu'
            rospy.loginfo(f"[LEADER]: Waiting for goto service: {service_name}")
            rospy.wait_for_service(service_name)
            self.uav_goto_fcu_services[name] = rospy.ServiceProxy(service_name, Vec4)

    def convert_utm_local(self, latitude, longitude, altitude, frame_id, target_frame):
        # Convert to UTM
        x, y, z = self.latlon_to_utm(latitude, longitude, altitude)
        
        # Create pose in utm_origin
        pose_in_utm_origin = PoseStamped()
        pose_in_utm_origin.header.stamp = rospy.Time.now()
        pose_in_utm_origin.header.frame_id = frame_id
        pose_in_utm_origin.pose.position.x = x
        pose_in_utm_origin.pose.position.y = y
        pose_in_utm_origin.pose.position.z = z
        pose_in_utm_origin.pose.orientation.w = 1.0  # Assuming no rotation

        return self.transform_pose(pose_in_utm_origin, target_frame)

    def latlon_to_utm(self, lat, lon, alt=0.0):
        # Convert latitude and longitude to UTM coordinates.
        u = utm.from_latlon(lat, lon)
        return u[0], u[1], alt  # x, y, z

    def transform_pose(self, pose_stamped, target_frame, timeout_sec=1.0):

        if not isinstance(pose_stamped, PoseStamped):
            raise TypeError("Expected PoseStamped, got: {}".format(type(pose_stamped)))

        tf_buffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tf_buffer)

        try:
            # Wait until the transform is available or timeout occurs
            rospy.logdebug("Waiting for transform from %s to %s...", pose_stamped.header.frame_id, target_frame)
            tf_buffer.can_transform(target_frame, pose_stamped.header.frame_id, rospy.Time(0), rospy.Duration(timeout_sec))
            transform = tf_buffer.lookup_transform(
                target_frame,
                pose_stamped.header.frame_id,
                rospy.Time(0),
                rospy.Duration(timeout_sec)
            )

            # Transform the pose
            transformed_pose = tf2_geometry_msgs.do_transform_pose(pose_stamped, transform)
            return transformed_pose

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr("Transform failed: %s", e)
            raise e
            return None

    def takeoff_service(self, uav):
        request = TriggerRequest()
        try:
            rospy.loginfo(f"[{uav}] Sending goal: {request}")
            response = self.uav_takeoff_services[uav](request)
            rospy.loginfo(f"[{uav}] Response: success={response.success}, message='{response.message}'")
        except rospy.ServiceException as e:
            rospy.logerr(f"[{uav}] Service call failed: {e}")

        return response.success

    def goto_service(self, uav, x, y, z, heading=0.0):
        request = Vec4Request()
        request.goal = [x, y, z, heading]
        try:
            rospy.loginfo(f"[{uav}] Sending goal: {request.goal}")
            response = self.uav_goto_services[uav](request)
            rospy.loginfo(f"[{uav}] Response: success={response.success}, message='{response.message}'")
        except rospy.ServiceException as e:
            rospy.logerr(f"[{uav}] Service call failed: {e}")

    def goto_fcu_service(self, uav, x, y, z, heading=0.0):
        request = Vec4Request()
        request.goal = [x, y, z, heading]
        try:
            rospy.loginfo(f"[{uav}] Sending goal: {request.goal}")
            response = self.uav_goto_fcu_services[uav](request)
            rospy.loginfo(f"[{uav}] Response: success={response.success}, message='{response.message}'")
        except rospy.ServiceException as e:
            rospy.logerr(f"[{uav}] Service call failed: {e}")

    def euclidean_distance(self, pose_stamped: PoseStamped, odom: Odometry, use_3d: bool = False) -> float:
        # Extract target pose
        target_x = pose_stamped.pose.position.x
        target_y = pose_stamped.pose.position.y
        target_z = pose_stamped.pose.position.z

        # Extract current odometry pose
        current_x = odom.pose.pose.position.x
        current_y = odom.pose.pose.position.y
        current_z = odom.pose.pose.position.z

        # Compute 2D or 3D distance
        if use_3d:
            distance = math.sqrt(
                (target_x - current_x) ** 2 +
                (target_y - current_y) ** 2 +
                (target_z - current_z) ** 2
            )
        else:
            distance = math.sqrt(
                (target_x - current_x) ** 2 +
                (target_y - current_y) ** 2
            )

        return distance

    def execute(self, userdata):
        if self.drone_list[0] == self.uav_name:
            rospy.logwarn("[LEADER]: I am the leader.")

            # Choose the type of format for the takeoff command
            if len(self.drone_list) == 1:
                rospy.loginfo("[LEADER]: Alone")
            elif len(self.drone_list) == 3:
                rospy.loginfo("[LEADER]: Leader + two drones in an equilateral triangle.")
            elif len(self.drone_list) == 6:
                rospy.loginfo("[LEADER]: Leader + five drones in a pentagon.")
            elif len(self.drone_list) == 9:
                rospy.loginfo("[LEADER]: Leader + 8 drones in an octagon.")
            else:
                rospy.loginfo("Unknown formation.") 
                return 'failed'

            # Dynamically subscribe to each drone's status
            self.subscribers = []
            for name in self.drone_list:
                # rospy.loginfo(f"UAV: : {name}")
                topic = f"/{name}/control_manager/diagnostics"
                sub = rospy.Subscriber(topic, ControlManagerDiagnostics, self.make_callback(name))
                self.subscribers.append(sub)

            # Dynamically subscribe to each drone's odometry
            self.subscribers_odom = []
            for name in self.drone_list:
                # rospy.loginfo(f"UAV: : {name}")
                topic = f"/{name}/estimation_manager/odom_main"
                sub_odom = rospy.Subscriber(topic, Odometry, self.make_callback_odom(name))
                self.subscribers_odom.append(sub_odom)

            # Initialize goto service proxies
            self.uav_takeoff_services = {}
            self.uav_goto_services = {}
            self.uav_goto_fcu_services = {}
            self.init_service_proxies()
            rospy.loginfo("[LEADER]: All UAV services connected.")

            # Calculation of Pose Start and Pose End
            pose_start = self.convert_utm_local(latitude = self.latitude_start,
                                                longitude = self.longitude_start,
                                                altitude = 0.0,
                                                frame_id = self.drone_list[0] + "/utm_origin",
                                                target_frame = self.drone_list[0] + "/liosam_origin")

            pose_end = self.convert_utm_local(latitude = self.latitude_end,
                                                longitude = self.longitude_end,
                                                altitude = 0.0,
                                                frame_id = self.drone_list[0] + "/utm_origin",
                                                target_frame = self.drone_list[0] + "/liosam_origin")

            if pose_start == None or pose_end == None:
                rospy.loginfo(f"Problems with poses")
                return 'failed'

            # Take-off
            if len(self.drone_list) == 1:
                rospy.loginfo("[LEADER]: Alone.")

                if not self.takeoff_service(self.drone_list[0]):
                    return 'failed'

                timeout = rospy.Time.now() + rospy.Duration(20)
                while rospy.Time.now() < timeout:
                    if self.drone_flying[self.drone_list[0]]:
                        rospy.loginfo(f"[LEADER]: {self.drone_list[0]} is flying.")
                        break
                    rospy.sleep(0.2)
                if self.drone_flying[self.drone_list[0]] == False:
                    rospy.loginfo(f"[LEADER]: {self.drone_list[0]}: not flying")
                    return 'failed'

                distance = self.euclidean_distance(pose_start, self.odom)
                # rospy.logwarn(f"[{self.drone_list[0]}] Pose start: {pose_start}")
                rospy.logwarn(f"[{self.drone_list[0]}] Distance to target: {distance}")

                if not self.goto_service(self.drone_list[0],
                                        pose_start.pose.position.x,
                                        pose_start.pose.position.y,
                                        self.height_formation):
                    return 'failed'

                while True:
                    distance = self.euclidean_distance(pose_start, self.odom)
                    rospy.logwarn_throttle(60, f"[{uav}] Distance to target: {distance}")
                    if self.euclidean_distance(pose_start, self.odom) < self.tolerance_distance:
                        if not self.goto_fcu_service(self.drone_list[0],
                                        0.0,
                                        0.0,
                                        0.0):
                            return 'failed'
                        break

            return 'initialized'
        else:
            rospy.logwarn("[LEADER]: I am not the leader.")
            rospy.sleep(2)
            return 'initialized'


class CommBase(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['finished','aborted'])
        self.task_execution = False

    def finish_callback(self, msg):
        if msg.data:
            rospy.logwarn("[TASK]: Finish trigger received!")
            self.task_execution = msg.data

    def execute(self, userdata):
        rospy.loginfo("[TASK]: Executing base communication task.")

        self.task_execution = False
        rospy.Subscriber('fms/finish', Bool, self.finish_callback)

        rate = rospy.Rate(10)
        while True:
            if self.preempt_requested():
                rospy.logwarn("[TASK]: Preempt requested! Aborting task.")
                self.service_preempt()
                return 'finished'  # Exit cleanly
            if self.task_execution:
                rospy.loginfo("[TASK]: Task execution completed normally.")
                return 'finished'
            # Simulate task operations here
            rate.sleep()


class Abort(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['shutdown'])

    def execute(self, userdata):
        rospy.logwarn("[ABORT]: Something went wrong. Landing...")
        rospy.sleep(2)
        return 'shutdown'


class Shutdown(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'])

    def execute(self, userdata):
        rospy.loginfo("[SHUTDOWN]: Powering down safely.")
        rospy.sleep(1)
        return 'done'

# --- Main FSM Container ---

def main():
    rospy.init_node('uav_fsm')

    sm = smach.StateMachine(outcomes=['fsm_complete'])

    with sm:
        smach.StateMachine.add('INIT', Init(), transitions={
            'initialized': 'LEADER',
            'failed': 'INIT'
        })

        smach.StateMachine.add('LEADER', CommLeader(), transitions={
            'initialized': 'TASK',
            'failed': 'ABORT'
        })

        smach.StateMachine.add('TASK', CommBase(), transitions={
            'finished': 'SHUTDOWN',
            'aborted': 'ABORT'
        })

        smach.StateMachine.add('ABORT', Abort(), transitions={
            'shutdown': 'SHUTDOWN'
        })

        smach.StateMachine.add('SHUTDOWN', Shutdown(), transitions={
            'done': 'fsm_complete'
        })

    # Start introspection server (for debugging in rqt_smach_viewer)
    sis = smach_ros.IntrospectionServer('drone_fsm_viewer', sm, '/DRONE_FSM')
    sis.start()

    outcome = sm.execute()
    sis.stop()


if __name__ == '__main__':
    main()