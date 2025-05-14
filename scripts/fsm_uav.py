#!/usr/bin/env python

import rospy
import rosnode
import smach
import smach_ros
from smach import Concurrence
from sensor_msgs.msg import PointCloud2
from mrs_msgs.msg import UavStatus, UavManagerDiagnostics
from std_msgs.msg import Bool

# --- Define FSM States ---

class Init(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['initialized', 'failed'])

    def execute(self, userdata):
        rospy.loginfo("State: INIT - Initializing drone systems.")
        # Add init logic here
        rospy.sleep(1)
        return 'initialized'


class SystemCheck(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['passed', 'failed'])


    def uav_status_callback(self, msg):
        if msg.hw_api_gnss_ok == True:
            if not self.logged_hw_api_gnss:
                self.hw_api_gnss_ok = True
                rospy.loginfo("System check: GNSS data received.")
                self.logged_hw_api_gnss = True

    def lidar_3d_callback(self, msg):
        if msg.width > 0 and msg.height > 0:
            if not self.logged_lidar_3d:
                self.lidar_3d_ok = True
                rospy.loginfo("System check: 3D LiDAR data received.")
                self.logged_lidar_3d = True

    def execute(self, userdata):
        rospy.loginfo("State: SYSTEM CHECK")
        self.logged_lidar_3d = False
        self.logged_hw_api_gnss = False
        self.hw_api_gnss_ok = False
        self.lidar_3d_ok = False

        rospy.Subscriber('mrs_uav_status/uav_status', UavStatus, self.uav_status_callback)
        rospy.Subscriber('lslidar/pcl_filtered', PointCloud2, self.lidar_3d_callback)
        self.sensor_pub = rospy.Publisher('fms/sensor_check', Bool, queue_size=10)
        timeout = rospy.Time.now() + rospy.Duration(20)

        rospy.loginfo("System check: Waiting for GNSS data...")
        rospy.loginfo("System check: Waiting for LiDAR data...")

        while rospy.Time.now() < timeout:
            if self.hw_api_gnss_ok and self.lidar_3d_ok:
                rospy.loginfo("System check: Passed.")
                self.sensor_pub.publish(Bool(True))
                rospy.sleep(1)
                return 'passed'
            rospy.sleep(0.5)

        rospy.logerr("System check failed: gnss=%s, lidar_3d=%s",
                     self.hw_api_gnss_ok, self.lidar_3d_ok)
        self.sensor_pub.publish(Bool(False))
        rospy.sleep(1)
        return 'failed'


class SwarmCheck(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['passed', 'failed'])

        # Get swarm list from parameter server
        self.drone_list = rospy.get_param('~uav_names', [])

        # # Dictionary to track drone status
        # self.drone_status = {name: False for name in self.drone_list}

    def make_callback(self, name):
        def callback(msg):
            if msg.uav_name in self.drone_list:
                if not self.drone_status[name]:
                    rospy.loginfo(f"{name}: OK")
                    self.drone_status[name] = True
        return callback

    def execute(self, userdata):
        rospy.loginfo("SwarmCheck: Waiting for all drone statuses...")
        self.drone_status = {name: False for name in self.drone_list}
        self.swarm_pub = rospy.Publisher('fms/swarm_check', Bool, queue_size=10)

        # Dynamically subscribe to each drone's status
        self.subscribers = []
        for name in self.drone_list:
            # rospy.loginfo(f"UAV: : {name}")
            topic = f"/{name}/uav_manager/diagnostics"
            sub = rospy.Subscriber(topic, UavManagerDiagnostics, self.make_callback(name))
            self.subscribers.append(sub)

        timeout = rospy.Time.now() + rospy.Duration(20)

        while not all(self.drone_status.values()) and rospy.Time.now() < timeout:
            rospy.sleep(0.2)

        if all(self.drone_status.values()):
            rospy.loginfo("SwarmCheck: All the drones are communicating with me.")
            self.swarm_pub.publish(Bool(True))
            rospy.sleep(1)
            return 'passed'
        else:
            rospy.logwarn(f"SwarmCheck: Not all drones responded. Status: {self.drone_status}")
            self.swarm_pub.publish(Bool(False))
            rospy.sleep(1)
            return 'failed'


class PreFlight(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['passed', 'timeout'])
        self.logged_can_take = False
        self.can_take_ok = False
        self.automatic_start_node = True

    def can_take_callback(self, msg):
        if msg.data and not self.logged_can_take:
            self.can_take_ok = True
            self.logged_can_take = True
            # rospy.loginfo("State: PRE_FLIGHT - Can takeoff command received.")

    def is_node_alive(self, node_name):
        try:
            # This pings the node. If the node is alive, it returns True.
            return rosnode.rosnode_ping(node_name, max_count=1, verbose=False)
        except rosnode.ROSNodeIOException:
            return False

    def execute(self, userdata):
        rospy.loginfo("State: PRE_FLIGHT - Waiting for system.")
        self.uav_name = rospy.get_param("~uav_name", "uav1")

        self.logged_can_take = False
        self.can_take_ok = False
        rospy.Subscriber('automatic_start/can_takeoff', Bool, self.can_take_callback)
        self.ready_to_pub = rospy.Publisher('fms/pre_flight', Bool, queue_size=10)

        timeout = rospy.Time.now() + rospy.Duration(20)
        while rospy.Time.now() < timeout:
            if self.can_take_ok:
                rospy.loginfo("State: PRE_FLIGHT - Received Automatic Start Command.")
                break

        timeout = rospy.Time.now() + rospy.Duration(60)
        rospy.loginfo("State: PRE_FLIGHT - Waiting for RC command.")

        while rospy.Time.now() < timeout:
            self.ready_to_pub.publish(Bool(False))
            rospy.sleep(1)
            if not self.is_node_alive('/'+ self.uav_name + '/automatic_start'):
                rospy.loginfo("State: PRE_FLIGHT - Received RC command.")
                self.ready_to_pub.publish(Bool(True))
                rospy.sleep(1)
                return 'passed'

        rospy.loginfo("State: PRE_FLIGHT - Did not receive any command.")
        rospy.sleep(1)
        return 'timeout'

class CommBase(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['finished'])
        self.task_execution = False

    def finish_callback(self, msg):
        if msg.data:
            rospy.logwarn("COMM_BASE : Finish trigger received!")
            self.task_execution = msg.data

    def execute(self, userdata):
        rospy.loginfo("State: COMM_BASE - Executing base communication task.")

        self.task_execution = False
        rospy.Subscriber('fms/finish', Bool, self.finish_callback)

        rate = rospy.Rate(10)
        while True:
            if self.preempt_requested():
                rospy.logwarn("COMM_BASE: Preempt requested! Aborting task.")
                self.service_preempt()
                return 'finished'  # Exit cleanly
            if self.task_execution:
                rospy.loginfo("COMM_BASE: Task execution completed normally.")
                return 'finished'
            # Simulate task operations here
            rate.sleep()


class Monitor(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['aborted', 'succeeded'])
        self.abort_triggered = False

    def abort_callback(self, msg):
        if msg.data:
            rospy.logwarn("MONITOR: Abort trigger received!")
            self.abort_triggered = msg.data

    def execute(self, userdata):
        rospy.loginfo("State: MONITOR - Monitoring system status...")

        self.abort_triggered = False
        rospy.Subscriber('fms/stop', Bool, self.abort_callback)

        rate = rospy.Rate(10)  # 10 Hz monitoring
        while True:
            if self.preempt_requested():
                rospy.loginfo("MONITOR: Preempted by FSM.")
                self.service_preempt()
                return 'succeeded'

            if self.abort_triggered:
                return 'aborted'

            rate.sleep()


class Abort(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['shutdown'])

    def execute(self, userdata):
        rospy.logwarn("State: ABORT - Something went wrong. Landing...")
        rospy.sleep(2)
        return 'shutdown'


class Shutdown(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'])

    def execute(self, userdata):
        rospy.loginfo("State: SHUTDOWN - Powering down safely.")
        rospy.sleep(1)
        return 'done'

def build_concurrent_check():
    def child_termination_cb(outcome_map):
        return True  # terminate all children when any finishes

    def outcome_cb(outcome_map):
        if outcome_map.get('MONITOR') == 'aborted':
            return 'aborted'
        elif outcome_map.get('COMM_BASE') == 'finished':
            return 'finished'
        return 'aborted'  # Fallback

    concurrent = smach.Concurrence(
        outcomes=['aborted', 'finished'],
        default_outcome='aborted',
        child_termination_cb=child_termination_cb,
        outcome_cb=outcome_cb
    )

    with concurrent:
        smach.Concurrence.add('COMM_BASE', CommBase())
        smach.Concurrence.add('MONITOR', Monitor())

    return concurrent


# --- Main FSM Container ---

def main():
    rospy.init_node('uav_fsm')

    sm = smach.StateMachine(outcomes=['fsm_complete'])

    with sm:
        smach.StateMachine.add('INIT', Init(), transitions={
            'initialized': 'SYSTEM_CHECK',
            'failed': 'ABORT'
        })

        smach.StateMachine.add('SYSTEM_CHECK', SystemCheck(), transitions={
            'passed': 'SWARM_CHECK',
            'failed': 'INIT'
        })

        smach.StateMachine.add('SWARM_CHECK', SwarmCheck(), transitions={
            'passed': 'PRE_FLIGHT',
            'failed': 'SYSTEM_CHECK'
        })

        smach.StateMachine.add('PRE_FLIGHT', PreFlight(), transitions={
            'passed': 'WORKING',
            'timeout': 'SYSTEM_CHECK'
        })

        smach.StateMachine.add('WORKING', build_concurrent_check(), transitions={
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