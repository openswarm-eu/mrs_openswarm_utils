#!/usr/bin/env python

import rospy
import smach
import smach_ros
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
        self.hw_api_gnss_ok = True
        self.lidar_3d_ok = False
        self.logged_lidar_3d = True
        self.logged_hw_api_gnss = True

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
        rospy.Subscriber('mrs_uav_status_acquisition', UavStatus, self.lidar_3d_callback)
        rospy.Subscriber('lslidar/pcl_filtered', PointCloud2, self.lidar_3d_callback)

        timeout = rospy.Time.now() + rospy.Duration(20)
        self.logged_lidar_3d = False
        self.logged_hw_api_gnss = False
        rospy.loginfo("System check: Waiting for GNSS data...")
        rospy.loginfo("System check: Waiting for LiDAR data...")

        while rospy.Time.now() < timeout:
            if self.hw_api_gnss_ok and self.lidar_3d_ok:
                rospy.loginfo("System check: Passed.")
                return 'passed'
            rospy.sleep(0.5)

        rospy.logerr("System check failed: gnss=%s, lidar_3d=%s",
                     self.hw_api_gnss_ok, self.lidar_3d_ok)
        return 'failed'


class SwarmCheck(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['passed', 'failed'])

        # Get swarm list from parameter server
        self.drone_list = rospy.get_param('~uav_names', [])

        # Dictionary to track drone status
        self.drone_status = {name: False for name in self.drone_list}

    def make_callback(self, name):
        def callback(msg):
            if msg.uav_name in self.drone_list:
                if not self.drone_status[name]:
                    rospy.loginfo(f"{name}: OK")
                    self.drone_status[name] = True
        return callback

    def execute(self, userdata):
        rospy.loginfo("SwarmCheck: Waiting for all drone statuses...")

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
            return 'passed'
        else:
            rospy.logwarn(f"SwarmCheck: Not all drones responded. Status: {self.drone_status}")
            return 'failed'


class Ready(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['takeoff_commanded', 'abort'])

    def execute(self, userdata):
        rospy.loginfo("State: READY - Waiting for takeoff command.")
        # Simulate takeoff command check
        rospy.sleep(1)
        # Replace with actual command check (e.g. from topic or service)
        return 'takeoff_commanded'


class Idle(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['shutdown'])

    def execute(self, userdata):
        rospy.loginfo("State: IDLE - Hovering or holding position.")
        rospy.sleep(5)
        return 'shutdown'


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
            'failed': 'ABORT'
        })

        smach.StateMachine.add('SWARM_CHECK', SwarmCheck(), transitions={
            'passed': 'READY',
            'failed': 'ABORT'
        })

        smach.StateMachine.add('READY', Ready(), transitions={
            'takeoff_commanded': 'IDLE',
            'abort': 'ABORT'
        })

        smach.StateMachine.add('IDLE', Idle(), transitions={
            'shutdown': 'SHUTDOWN'
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
