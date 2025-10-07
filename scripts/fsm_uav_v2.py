#!/usr/bin/env python

import rospy
import rosnode
import smach
import smach_ros
from smach import Concurrence
from sensor_msgs.msg import PointCloud2, NavSatFix
from mrs_msgs.msg import UavStatus, UavManagerDiagnostics, HwApiRcChannels, ControlManagerDiagnostics
from std_msgs.msg import Bool, String, UInt8MultiArray
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger, TriggerRequest
from mrs_msgs.srv import Vec4, Vec4Request
from mrs_msgs.srv import TransformPoseSrv, TransformPoseSrvRequest
from mrs_msgs.srv import ReferenceStampedSrv, ReferenceStampedSrvRequest
import tf2_ros
import tf2_geometry_msgs
import utm
import numpy as np
import math
import time

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
            rospy.loginfo_throttle(10, "UAV is publishing normally.")
        else:
            rospy.loginfo_throttle(10, "UAV not received yet.")

    # Subscribe to GNSS
    def gnss_status_callback(self, msg):
        self.last_msg_time_gnss = time.time()
        if not self.hw_api_gnss_ok:
            rospy.loginfo("GNSS is now active.")
        self.hw_api_gnss_ok = True
        self.gnss_status = msg.status.status

    def check_gnss_status(self, event):
        if self.last_msg_time_gnss is None:
            rospy.logwarn_throttle(5, "GNSS not received yet.")
            return

        time_since_last = time.time() - self.last_msg_time_gnss
        if time_since_last > self.timeout:
            if self.hw_api_gnss_ok:
                rospy.logwarn(f"GNSS stopped publishing ({time_since_last:.1f}s since last message)")
            self.hw_api_gnss_ok = False
            self.gnss_fix_ok = False
        else:
            if self.gnss_status == 2:
                rospy.loginfo_throttle(10, "GNSS is publishing normally: STATUS_GBAS_FIX.")
                self.gnss_fix_ok = True
            else:
                rospy.loginfo_throttle(10, f"GNSS is publishing normally, but status is {self.gnss_status}")
                self.gnss_fix_ok = False

    # Subscribe to 3D Lidar
    def lidar_3d_callback(self, msg):
        self.last_msg_time = time.time()
        if not self.lidar_active:
            rospy.loginfo("LiDAR is now active.")
        self.lidar_active = True

    def check_lidar_status(self, event):
        if self.last_msg_time is None:
            rospy.logwarn_throttle(5, "LiDAR not received yet.")
            return

        time_since_last = time.time() - self.last_msg_time
        if time_since_last > self.timeout:
            if self.lidar_active:
                rospy.logwarn(f"LiDAR stopped publishing ({time_since_last:.1f}s since last message)")
            self.lidar_active = False
        else:
            rospy.loginfo_throttle(10, "LiDAR is publishing normally.")

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
        if msg.channels[8] < 0.5:
            self.rc_motor_off = False
        else:
            self.rc_motor_off = True

        if self.rc_motor_off == False:
            if msg.channels[4] < 0.5:
                self.rc_arm = False
            else:
                self.rc_arm = True
            if msg.channels[10] < 0.5:
                self.rc_offboard = False
            else:
                self.rc_offboard = True
        else:
            self.rc_arm = False
            self.rc_offboard = False

    # Subscriber to computer
    def swarm_init_cb(self, msg):
        if msg.data:
            rospy.loginfo("[INIT]: Received ckeck trigger.")
            self.swarm_init_received = True

    def execute(self, userdata):
        rospy.loginfo("[INIT]: Initializing drone systems.")

        # Flags
        self.hw_api_gnss_ok = False
        self.gnss_fix_ok = False
        self.swarm_communication = False
        self.logged_can_take = False
        self.can_take_ok = False
        self.rc_motor_off = False
        self.rc_arm = False
        self.rc_offboard = False
        self.swarm_init_received = False

        self.timeout = 2.0
        self.last_msg_time = None
        self.last_msg_time_gnss = None
        self.lidar_active = False

        # Value of gnss status
        self.gnss_status = 0

        rospy.Timer(rospy.Duration(1.0), self.check_lidar_status)
        rospy.Timer(rospy.Duration(1.0), self.check_gnss_status)

        # Publisher of status 
        self.pub = rospy.Publisher('fms/drone_status', UInt8MultiArray, queue_size=10)

        # Subscribe to base's trigger
        rospy.Subscriber('/' + self.computer_name + '/fms/init', Bool, self.swarm_init_cb)

        # Subscribe to sensors
        rospy.Subscriber('mrs_uav_status/uav_status', UavStatus, self.uav_status_callback)
        rospy.Subscriber('gnss_verifier/gnss_wstatus', NavSatFix, self.gnss_status_callback)
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
            rospy.logwarn("[INIT]: gnss=%s, lidar_3d=%s, swarm=%s, pre-flight=%s, motor_off=%s, armed=%s, offboard:%s",
                        self.gnss_fix_ok, self.lidar_active, self.swarm_communication, 
                        self.can_take_ok, self.rc_motor_off, self.rc_arm, self.rc_offboard)

            if all(self.drone_status.values()):
                self.swarm_communication = True

            status = UInt8MultiArray()
            # 1 for True, 0 for False
            status.data = [
                int(self.gnss_fix_ok),
                int(self.lidar_active),
                int(self.swarm_communication),
                int(self.can_take_ok),
                int(self.rc_arm),
                int(self.rc_offboard)
            ]

            self.pub.publish(status)

            heartbeat_rate.sleep()

        return 'initialized'

class SwarmInit(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['finished','aborted'])
        self.drone_list = rospy.get_param('~uav_names', [])
        self.uav_name = rospy.get_param("~uav_name", "uav1")
        self.computer_name = rospy.get_param("~computer_name", "computer1")
        self.takeoff_timeout = rospy.get_param("~takeoff_timeout", 30.0)
        self.height_formation = rospy.get_param("~height_formation", 3.0)
        self.tolerance_distance = rospy.get_param("~tolerance_distance", 0.5)
        self.swarm_init = False
        self.drone_flying = False

    def swarm_init_callback(self, msg):
        frame_id = msg.header.frame_id
        drone = frame_id.split('/')[0]
        if drone == self.uav_name:
            rospy.logwarn("[SWARM_INIT]: Swarm init trigger received!")
            # rospy.logwarn(f"[SWARM_INIT]: Target pose: {msg}")
            self.pose_start = msg
            self.swarm_init = True

    def drone_flying_callback(self, msg):
        self.drone_flying = msg.flying_normally

    def odom_callback(self, msg):
        self.odom = msg

    def init_service_proxies(self):
        # TakeOff Services
        for name in self.drone_list:
            service_name = f'/{name}/uav_manager/takeoff'
            rospy.loginfo(f"[SWARM_INIT]: Waiting for take-off service: {service_name}")
            rospy.wait_for_service(service_name)
            self.uav_takeoff_services[name] = rospy.ServiceProxy(service_name, Trigger)
        # GoTo Services
        for name in self.drone_list:
            service_name = f'/{name}/control_manager/goto'
            rospy.loginfo(f"[SWARM_INIT]: Waiting for goto service: {service_name}")
            rospy.wait_for_service(service_name)
            self.uav_goto_services[name] = rospy.ServiceProxy(service_name, Vec4)
        # GoTo_fcu Services
        for name in self.drone_list:
            service_name = f'/{name}/control_manager/goto_fcu'
            rospy.loginfo(f"[SWARM_INIT]: Waiting for goto service: {service_name}")
            rospy.wait_for_service(service_name)
            self.uav_goto_fcu_services[name] = rospy.ServiceProxy(service_name, Vec4)
        # Transform Services
        for name in self.drone_list:
            service_name = f'/{name}/control_manager/transform_pose'
            rospy.loginfo(f"[SWARM_INIT]: Waiting for transform service: {service_name}")
            rospy.wait_for_service(service_name)
            self.uav_transform_services[name] = rospy.ServiceProxy(service_name, TransformPoseSrv)
        # Reference Services
        for name in self.drone_list:
            service_name = f'/{name}/control_manager/reference'
            rospy.loginfo(f"[SWARM_INIT]: Waiting for reference service: {service_name}")
            rospy.wait_for_service(service_name)
            self.uav_reference_services[name] = rospy.ServiceProxy(service_name, ReferenceStampedSrv)

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
        return response.success

    def goto_fcu_service(self, uav, x, y, z, heading=0.0):
        request = Vec4Request()
        request.goal = [x, y, z, heading]
        try:
            rospy.loginfo(f"[{uav}] Sending goal: {request.goal}")
            response = self.uav_goto_fcu_services[uav](request)
            rospy.loginfo(f"[{uav}] Response: success={response.success}, message='{response.message}'")
        except rospy.ServiceException as e:
            rospy.logerr(f"[{uav}] Service call failed: {e}")
        return response.success

    def uav_reference_service(self, uav, x, y, z, heading=0.0, frame_id="fcu"):
        request = ReferenceStampedSrvRequest()
        request.header.stamp = rospy.Time.now()
        request.header.frame_id = f"{uav}/{frame_id}"
        request.reference.position.x = x
        request.reference.position.y = y
        request.reference.position.z = z
        request.reference.heading = heading
        try:
            rospy.loginfo(f"[{uav}] Sending goal: {request}")
            response = self.uav_reference_services[uav](request)
            rospy.loginfo(f"[{uav}] Response: success={response.success}, message='{response.message}'")
        except rospy.ServiceException as e:
            rospy.logerr(f"[{uav}] Service call failed: {e}")
        return response.success

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
        rospy.loginfo("[SWARM_INIT]: Executing base communication task.")
        self.swarm_init = False

        # Subscribe from base computer
        rospy.Subscriber('/' + self.computer_name + '/fms/swarm_check', PoseStamped, self.swarm_init_callback)

        # Subscribe diagnostics 
        rospy.Subscriber("control_manager/diagnostics", ControlManagerDiagnostics, self.drone_flying_callback)

        # Subscribe odometry
        rospy.Subscriber("estimation_manager/odom_main", Odometry, self.odom_callback)

        # Initialize goto service proxies
        self.uav_takeoff_services = {}
        self.uav_goto_services = {}
        self.uav_goto_fcu_services = {}
        self.uav_transform_services = {}
        self.uav_reference_services = {}
        self.init_service_proxies()

        while not self.swarm_init:
            rospy.logwarn_throttle(5, f"[SWARM_INIT]: Waiting base computer command.")

        # Initialize take-off
        rospy.loginfo(f"[SWARM_INIT]: {self.uav_name} initializes take-off.")

        if not self.takeoff_service(self.uav_name):
            return 'aborted'

        timeout = rospy.Time.now() + rospy.Duration(self.takeoff_timeout)
        while rospy.Time.now() < timeout:
            if self.drone_flying:
                rospy.loginfo(f"[SWARM_INIT]: {self.uav_name} is flying.")
                break
            rospy.sleep(0.2)

        if self.drone_flying == False:
            rospy.loginfo(f"[SWARM_INIT]: {self.uav_name} is not flying")
            return 'aborted'

        transform_pose_srv = TransformPoseSrvRequest()
        transform_pose_srv.frame_id = self.odom.header.frame_id
        transform_pose_srv.pose = self.pose_start
        response = self.uav_transform_services[self.uav_name](transform_pose_srv)

        self.pose_start_ = response.pose

        distance = self.euclidean_distance(self.pose_start_, self.odom, True)
        # rospy.logwarn(f"[SWARM_INIT]: Pose start: {self.pose_start_}")
        rospy.logwarn(f"[SWARM_INIT]: Distance to target: {distance}")

        # if not self.goto_service(self.uav_name,
        #                         self.pose_start_.pose.position.x,
        #                         self.pose_start_.pose.position.y,
        #                         self.pose_start_.pose.position.z):
        #     return 'aborted'

        if not self.uav_reference_service(self.uav_name,
                                self.pose_start.pose.position.x,
                                self.pose_start.pose.position.y,
                                self.pose_start.pose.position.z,
                                0.0, frame_id="utm_navsat"):
            return 'aborted'

        self.drone_swarm_init = rospy.Publisher('fms/drone_swarm_init', Bool, queue_size=10)

        while True:
            distance = self.euclidean_distance(self.pose_start_, self.odom, True)
            rospy.logwarn_throttle(5, f"[SWARM_INIT]: Distance to target: {distance}")
            if self.euclidean_distance(self.pose_start_, self.odom, True) < self.tolerance_distance:
                self.drone_swarm_init.publish(Bool(data=True))
                if not self.goto_fcu_service(self.uav_name, 0.0, 0.0, 0.0):
                    return 'aborted'
                break
            rospy.sleep(0.2)

        return 'finished'


class Monitor(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['finished','aborted'])
        self.task_execution = False
        # Get computer name list from parameter server
        self.computer_name = rospy.get_param("~computer_name", "computer1")

    def finish_callback(self, msg):
        if msg.data:
            rospy.logwarn("[MONITOR]: Finish trigger received!")
            self.task_execution = msg.data

    def execute(self, userdata):
        rospy.loginfo("[MONITOR]: Executing base communication task.")

        self.task_execution = False
        rospy.Subscriber('/' + self.computer_name + '/fms/finish', Bool, self.finish_callback)

        rate = rospy.Rate(0.5)
        while not self.task_execution:
            rospy.logwarn_throttle(5, f"[MONITOR]: Waiting to finish.")
            rate.sleep()

        return 'finished'


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
            'initialized': 'SWARM_INIT',
            'failed': 'INIT'
        })

        smach.StateMachine.add('SWARM_INIT', SwarmInit(), transitions={
            'finished': 'MONITOR',
            'aborted': 'ABORT'
        })
  
        smach.StateMachine.add('MONITOR', Monitor(), transitions={
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