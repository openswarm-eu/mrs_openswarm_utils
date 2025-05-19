#!/usr/bin/env python

import rospy
import rosnode
import smach
import smach_ros
from smach import Concurrence
from sensor_msgs.msg import PointCloud2
from mrs_msgs.msg import UavStatus, UavManagerDiagnostics, HwApiRcChannels
from std_msgs.msg import Bool, String, UInt8MultiArray
import tf2_ros
import utm

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
        self.pub = rospy.Publisher('fms//drone_status_report', UInt8MultiArray, queue_size=10)

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
        self.drone_flying = {name: False for name in self.drone_list}
        self.takeoff_execution = False

    def make_callback(self, name):
        def callback(msg):
            if msg.uav_name in self.drone_list:
                self.drone_flying[name] = msg.flying_normally
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

    def convert_utm_local(self, latitude, longitude, altitude, frame_id, target_frame):
        # Convert to UTM
        x, y, z = self.latlon_to_utm(latitude, longitude, altitude)
        
        # Create pose in utm_origin
        pose_in_utm_origin = self.create_pose_stamped(x, y, z, frame_id)
        
        return self.transform_pose(pose_in_utm_origin, target_frame)

    def latlon_to_utm(self, lat, lon, alt=0.0):
        # Convert latitude and longitude to UTM coordinates.
        u = utm.from_latlon(lat, lon)
        return u[0], u[1], alt  # x, y, z

    def create_pose_stamped(self, x, y, z, frame_id="utm_origin"):
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = frame_id
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.w = 1.0  # no rotation
        return pose

    def transform_pose(self, pose, target_frame):
        try:
            # Wait until the transform becomes available
            self.tf_buffer.can_transform(
                target_frame, 
                pose.header.frame_id, 
                rospy.Time(0), 
                rospy.Duration(5.0)
            )
            transformed_pose = self.tf_buffer.transform(
                pose, target_frame, rospy.Duration(1.0)
            )
            return transformed_pose
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr(f"TF transform failed: {e}")
            return None

    def execute(self, userdata):
        if self.uav_name == self.drone_list[0]:
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

            # Initialize goto service proxies
            self.uav_takeoff_services = {}
            self.uav_goto_services = {}
            self.init_service_proxies()
            rospy.loginfo("[LEADER]: All UAV services connected.")

            # Wait for TFs
            self.tf_buffer = tf2_ros.Buffer()
            self.listener = tf2_ros.TransformListener(self.tf_buffer)
            rospy.loginfo("[LEADER]: Waiting for TF to fill buffer...")
            rospy.sleep(1.0) 

            # Calculation of Pose Start and Pose End
            pose_start = self.convert_utm_local(latitude = self.latitude_start,
                                                longitude = self.longitude_start,
                                                altitude = 0.0,
                                                frame_id = self.uav_name + "/utm_origin",
                                                target_frame = "common_origin")

            pose_end = self.convert_utm_local(latitude = self.latitude_end,
                                                longitude = self.longitude_end,
                                                altitude = 0.0,
                                                frame_id = self.uav_name + "/utm_origin",
                                                target_frame = "common_origin")

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

                if not self.goto_service(self.drone_list[0],
                                        pose_start.pose.position.x,
                                        pose_start.pose.position.y,
                                        self.height_formation):
                    return 'failed'

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
build_concurrent_check