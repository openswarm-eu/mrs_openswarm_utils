#!/usr/bin/env python

import rospy
import smach
import smach_ros
from std_msgs.msg import Bool, UInt8MultiArray
from visualization_msgs.msg import MarkerArray
from fsm_visualization import create_marker, create_text_marker
from fsm_functions import latlon_to_utm, create_pose_stamped, generate_swarm_poses, compute_formation_positions
from geometry_msgs.msg import PoseStamped, PoseArray, Pose
import tf2_ros
import tf2_geometry_msgs.tf2_geometry_msgs

# --- Define FSM States ---

class Init(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['initialized', 'timeout', 'failed'])

        # Get swarm list from parameter server
        self.drone_list = rospy.get_param('~uav_names', [])
        self.pass_init_takeoff = rospy.get_param('~pass_init_takeoff', False)
        self.mode = rospy.get_param('~mode', 'complete')  # Modes: complete, partial, minimal

        # Dictionary to track drone status
        self.drone_status_gnss = {name: False for name in self.drone_list}
        self.drone_status_lidar = {name: False for name in self.drone_list}
        self.drone_status_swarm = {name: False for name in self.drone_list}
        self.drone_status_pre_flight = {name: False for name in self.drone_list}
        self.drone_status_armed = {name: False for name in self.drone_list}
        self.drone_status_offboard = {name: False for name in self.drone_list}

    def make_callback(self, name):
        def callback(msg):
            self.drone_status_gnss[name] = bool(msg.data[0])
            self.drone_status_lidar[name] = bool(msg.data[1])
            self.drone_status_swarm[name] = bool(msg.data[2])
            self.drone_status_pre_flight[name] = bool(msg.data[3])
            self.drone_status_armed[name] = bool(msg.data[4])
            self.drone_status_offboard[name] = bool(msg.data[5])
        return callback

    def execute(self, userdata):
        rospy.loginfo("[INIT]: Initializing Base Station.")
        rospy.loginfo("[INIT]: Mode is %s.", self.mode)
        self.swarm_init_ok = False

       # Check formation type based on number of drones
        if len(self.drone_list) == 1:
            rospy.loginfo("One drone.")
        elif len(self.drone_list) == 3:
            rospy.loginfo("Three drones.")
        elif len(self.drone_list) == 6:
            rospy.loginfo("Six drones.")
        elif len(self.drone_list) == 9:
            rospy.loginfo("Nine drones.")
        else:
            rospy.logerr("Unknown formation.")
            return 'failed'

        # Dynamically subscribe to each drone's status
        self.subscribers_drones = []
        for name in self.drone_list:
            # rospy.loginfo(f"UAV: : {name}")
            topic = f"/{name}/fms/drone_status"
            sub = rospy.Subscriber(topic, UInt8MultiArray, self.make_callback(name))
            self.subscribers_drones.append(sub)

        self.swarm_pub = rospy.Publisher('fms/init', Bool, queue_size=10)
        rospy.sleep(2.0)

        if self.pass_init_takeoff:
            rospy.logwarn("[INIT]: Skipping initialization due to pass_init_takeoff flag.")
            return 'initialized'

        while not rospy.is_shutdown() and not self.swarm_init_ok:
            for name in self.drone_list:
                if not self.drone_status_gnss[name] or not self.drone_status_lidar[name]:
                    rospy.logerr("[%s]: gnss=%s, lidar_3d=%s, swarm=%s, pre-flight=%s, armed=%s, offboard:%s",
                            name, self.drone_status_gnss[name], self.drone_status_lidar[name], self.drone_status_swarm[name], 
                            self.drone_status_pre_flight[name], self.drone_status_armed[name], self.drone_status_offboard[name])
                else:
                    rospy.logwarn("[%s]: gnss=%s, lidar_3d=%s, swarm=%s, pre-flight=%s, armed=%s, offboard:%s",
                            name, self.drone_status_gnss[name], self.drone_status_lidar[name], self.drone_status_swarm[name], 
                            self.drone_status_pre_flight[name], self.drone_status_armed[name], self.drone_status_offboard[name])
                rospy.sleep(0.5)
            # Evaluate subsystem statuses for clarity
            gnss_ok       = all(self.drone_status_gnss.values())
            lidar_ok      = all(self.drone_status_lidar.values())
            swarm_ok      = all(self.drone_status_swarm.values())
            pre_flight_ok = all(self.drone_status_pre_flight.values())
            armed_ok      = all(self.drone_status_armed.values())
            offboard_ok   = all(self.drone_status_offboard.values())

            # Decide initialization success based on current mode
            if self.mode == "complete":
                # Requires all systems including offboard mode
                self.swarm_init_ok = all([gnss_ok, lidar_ok, swarm_ok, pre_flight_ok, armed_ok, offboard_ok])

            elif self.mode == "partial":
                # Requires all systems except offboard mode
                self.swarm_init_ok = all([gnss_ok, lidar_ok, swarm_ok, pre_flight_ok, armed_ok])

            elif self.mode == "minimal":
                # Requires only GNSS, LIDAR, and pre-flight checks
                self.swarm_init_ok = all([gnss_ok, lidar_ok, pre_flight_ok])

            elif self.mode == "none":
                # Requires no checks, always true
                self.swarm_init_ok = True

            else:
                # Unsupported mode or missing requirements
                self.swarm_init_ok = False

        self.swarm_pub.publish(Bool(data=True))
        self.swarm_init_ok = False
        rospy.sleep(2.0)       
        return 'initialized' 


class InitTakeOff(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['passed', 'failed'], output_keys=['formation_points_out'])

        # Get swarm list from parameter server
        self.drone_list = rospy.get_param('~uav_names', [])
        self.uav_name = rospy.get_param("~uav_name", "uav1")

        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(30.0))
        self.listener = tf2_ros.TransformListener(self.tf_buffer)
        rospy.sleep(1.0) 

        self.height_formation = rospy.get_param("~height_formation", 6.0)
        self.latitude_start = rospy.get_param("~latitude_start", 0.0)
        self.longitude_start = rospy.get_param("~longitude_start", 0.0) 
        self.latitude_end = rospy.get_param("~latitude_end", 0.0)
        self.longitude_end = rospy.get_param("~longitude_end", 0.0)
        self.six_formation = rospy.get_param("~six_formation", "regular")
        self.nine_formation = rospy.get_param("~nine_formation", "v_formation")
        self.distance = rospy.get_param("~distance", "0.0")

        self.publishers_drones = rospy.Publisher('fms/swarm_check', PoseStamped, queue_size=10)

        # Dictionary to track swarm init status
        self.drone_swarm_init = {name: False for name in self.drone_list}

    def make_callback(self, name):
        def callback(msg):
            self.drone_swarm_init[name] = bool(msg.data)
        return callback

    def transform_pose_frame(self, pose, target_frame, origin_frame):
        pose.header.frame_id = origin_frame
        try:
            # Wait until the transform becomes available
            self.tf_buffer.can_transform(
                target_frame, 
                pose.header.frame_id, 
                rospy.Time(0), 
                rospy.Duration(5.0)
            )
            transformed_pose = self.tf_buffer.transform(
                pose, target_frame, rospy.Duration(5.0)
            )
            return transformed_pose
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr(f"TF transform failed: {e}")
            return None

    def init_drones (self, latitude, longitude, altitude, frame_id, fomation_points):
        # Convert to UTM
        x, y, z = latlon_to_utm(latitude, longitude, altitude)

        # Create pose in utm_origin
        pose_in_utm_origin = create_pose_stamped(x, y, z, frame_id)

        # Create swarm poses in utm_origin
        swarm_poses = generate_swarm_poses(pose_in_utm_origin, self.drone_list, fomation_points)

        return swarm_poses

    def execute(self, userdata):
        rospy.loginfo("[INIT_TAKE_OFF]: Initializing Take-off State.")

        # Init Markers Publisher to visualize
        pub = rospy.Publisher("formation_markers", MarkerArray, queue_size=10)

        # Initialize the marker array
        marker_array = MarkerArray()

        # Mark start point
        x, y, z = latlon_to_utm(self.latitude_start, self.longitude_start, self.height_formation)
        pose_start_utm = create_pose_stamped(x, y, z, self.drone_list[0] + "/utm_navsat")
        self.pose_start_transformed = self.transform_pose_frame(pose_start_utm, \
                                            self.drone_list[0] + "/stable_origin", \
                                            self.drone_list[0] + "/utm_navsat")

        if self.pose_start_transformed == None:
            return 'failed'

        index = 30
        marker_array.markers.append(create_marker("common_origin", index, self.pose_start_transformed))
        marker_array.markers.append(create_text_marker("common_origin", 100 + index, "start", self.pose_start_transformed))
        pub.publish(marker_array)

        # Mark end point
        x, y, z = latlon_to_utm(self.latitude_end, self.longitude_end, self.height_formation)
        pose_end_utm = create_pose_stamped(x, y, z, self.drone_list[0] + "/utm_navsat")
        self.pose_end_transformed = self.transform_pose_frame(pose_end_utm, \
                                            self.drone_list[0] + "/stable_origin", \
                                            self.drone_list[0] + "/utm_navsat")

        if self.pose_end_transformed == None:
            return 'failed'

        index = 20
        marker_array.markers.append(create_marker("common_origin", index, self.pose_end_transformed))
        marker_array.markers.append(create_text_marker("common_origin", 100 + index, "end", self.pose_end_transformed))
        pub.publish(marker_array)

        # Swarm Poses
        pose_array = PoseArray()
        pose_array.header.stamp = rospy.Time.now()
        pose_array.header.frame_id = self.drone_list[0] + "/utm_navsat"

        # Choose the type of format for the takeoff command
        if len(self.drone_list) == 1:
            rospy.loginfo("One drone.")
            # Create a formation
            formation_points, sequence = compute_formation_positions(self.drone_list, pose_start_utm, pose_end_utm, 0.0, 0.0)
            rospy.loginfo(f"[INIT_TAKE_OFF]: Formation Points: {formation_points}")
        elif len(self.drone_list) == 3:
            rospy.loginfo("Three drones.")
            # Create a formation
            formation_points, sequence = compute_formation_positions(self.drone_list, pose_start_utm, pose_end_utm, self.distance, 0.0)
            rospy.loginfo(f"[INIT_TAKE_OFF]: Formation Points: {formation_points}")
        elif len(self.drone_list) == 6:
            rospy.loginfo("Six drones.")
            # Create a formation
            formation_points, sequence = compute_formation_positions(self.drone_list, pose_start_utm, pose_end_utm, self.distance, 0.0, self.six_formation)
            rospy.loginfo(f"[INIT_TAKE_OFF]: Formation Points: {formation_points}")
        elif len(self.drone_list) == 9:
            rospy.loginfo("Nine drones.")
            # Create a formation
            formation_points, sequence = compute_formation_positions(self.drone_list, pose_start_utm, pose_end_utm, self.distance, 0.0, self.nine_formation)
            rospy.loginfo(f"[INIT_TAKE_OFF]: Formation Points: {formation_points}")
        else:
            rospy.loginfo("Unknown formation.") 
            return 'failed'

        # Create poses
        swarm_poses = self.init_drones(self.latitude_start, self.longitude_start, self.height_formation, \
                                        self.drone_list[0] + "/utm_navsat", formation_points)
        # rospy.loginfo(f"[INIT_TAKE_OFF]: Swarm Poses: {swarm_poses}")

        marker_array_ = MarkerArray()
        # Mark each drone's initial position
        for index, name in enumerate(self.drone_list):
            swarm_poses[sequence[index]].header.stamp = rospy.Time.now()
            swarm_poses_transformed = self.transform_pose_frame(swarm_poses[sequence[index]], \
                                                            self.drone_list[0] + "/stable_origin", \
                                                            self.drone_list[0] + "/utm_navsat")
            swarm_poses_transformed.header.frame_id = self.drone_list[0] + "/stable_origin"
            # Mark pose
            marker_array_.markers.append(create_marker("common_origin", 50 + index, swarm_poses_transformed))
            marker_array_.markers.append(create_text_marker("common_origin", 200 + index, self.drone_list[sequence[index]], swarm_poses_transformed))
            # rospy.loginfo(f"[INIT_TAKE_OFF]: Swarm Poses: {swarm_poses_transformed}")
            pub.publish(marker_array_)

        # Dynamically subscribe to each swarm_init's status
        self.subscribers_drones = []
        for name in self.drone_list:
            # rospy.loginfo(f"UAV: : {name}")
            topic = f"/{name}/fms/drone_swarm_init"
            sub = rospy.Subscriber(topic, Bool, self.make_callback(name))
            self.subscribers_drones.append(sub)

        for index, name in enumerate(self.drone_list):
            name = self.drone_list[sequence[index]]
            rospy.loginfo(f"[INIT_TAKE_OFF]: Drone: {name}")
            swarm_poses[sequence[index]].header.stamp = rospy.Time.now()
            swarm_poses[sequence[index]].header.frame_id = name + "/utm_navsat"
            swarm_poses_transformed = self.transform_pose_frame(swarm_poses[sequence[index]], \
                                                name + "/stable_origin", \
                                                name + "/utm_navsat")
            swarm_poses_transformed.header.frame_id = name + "/stable_origin"
            # rospy.loginfo(f"[INIT_TAKE_OFF]: Pose {swarm_poses_transformed}.")

            # Publish the pose to the drone
            self.publishers_drones.publish(swarm_poses[sequence[index]])

            # Wait for the drone to arrive at the position
            timeout = rospy.Time.now() + rospy.Duration(60.0)
            while rospy.Time.now() < timeout:
                rospy.logwarn_throttle(5, f"[INIT_TAKE_OFF]: Waiting for the drone to arrive.")
                if self.drone_swarm_init[name]:
                    rospy.loginfo(f"[INIT_TAKE_OFF]: {self.uav_name} arrived at the initial position.")
                    break
                rospy.sleep(0.2)

        userdata.formation_points_out = formation_points

        return 'passed'


class InitSwarm(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['passed', 'failed'], input_keys=['formation_points_in'])
        self.latitude_start = rospy.get_param("~latitude_start", 0.0)
        self.longitude_start = rospy.get_param("~longitude_start", 0.0)
        self.latitude_end = rospy.get_param("~latitude_end", 0.0)
        self.longitude_end = rospy.get_param("~longitude_end", 0.0)
        self.height_formation = rospy.get_param("~height_formation", 0.0)
        self.drone_list = rospy.get_param('~uav_names', [])
        self.distance = rospy.get_param("~distance", "0.0")

        self.publish_pose_array = rospy.Publisher('swarm_formation/positions', PoseArray, queue_size=10)
        self.idx = 0

    def execute(self, userdata):
        rospy.logwarn("[INIT_SWARM]: Initializing Swarm State.")
        rospy.loginfo(f"[INIT_SWARM]: Current waypoint index: {self.idx}")

        waypoint_list = rospy.get_param('~waypoint_list', [])

        rospy.loginfo(f"[INIT_SWARM]: Publishing formation for waypoint #{self.idx + 1}")

        # Mark start point
        x, y, z = latlon_to_utm(waypoint_list[self.idx][0], waypoint_list[self.idx][1], waypoint_list[self.idx][2])
        pose_start_utm = create_pose_stamped(x, y, z, self.drone_list[0] + "/utm_navsat")

        # Mark end point
        x, y, z = latlon_to_utm(waypoint_list[self.idx + 1][0], waypoint_list[self.idx + 1][1], waypoint_list[self.idx + 1][2])
        pose_end_utm = create_pose_stamped(x, y, z, self.drone_list[0] + "/utm_navsat")

        formation_points, sequence = compute_formation_positions(self.drone_list, pose_start_utm, pose_end_utm, self.distance, 0.0)

        pose_array = PoseArray()
        pose_array.header.stamp = rospy.Time.now()
        pose_array.header.frame_id = self.drone_list[0] + "/utm_navsat"

        for i in range(len(formation_points)):
            drone_id = self.drone_list[i]
            pose_end_utm = create_pose_stamped(x, y, z, drone_id + "/utm_navsat")

            pose = Pose()
            pose.position.x = pose_end_utm.pose.position.x + formation_points[drone_id][0]
            pose.position.y = pose_end_utm.pose.position.y + formation_points[drone_id][1]
            pose.position.z = waypoint_list[self.idx + 1][2]

        pose_array.poses.append(pose)

        self.publish_pose_array.publish(pose_array)
        rospy.loginfo("Published %d poses for waypoint #%d", len(pose_array.poses), self.idx + 1)
        self.idx += 1

        rospy.sleep(2)
        return 'passed'


class Monitor(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['continue', 'aborted', 'shutdown'])
        self.idx = 0
        waypoint_list = rospy.get_param('~waypoint_list', [])
        self.waypoint_count = len(waypoint_list)

    def wait_for_enter(self):
        input("[MONITOR]: Press Enter to continue to next target...")

    def execute(self, userdata):
        rospy.logwarn("[MONITOR]: The task is running.")
        self.wait_for_enter()
        rospy.sleep(2)
        self.idx += 1
        if self.idx >= self.waypoint_count - 1:
            self.idx = 0
            return 'shutdown'
        else:
            return 'continue'


class Abort(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['shutdown'])

    def execute(self, userdata):
        rospy.logwarn("[ABORT]: Something went wrong.")
        rospy.sleep(2)
        return 'shutdown'


class Shutdown(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'])

    def execute(self, userdata):
        rospy.loginfo("[SHUTDOWN]: Powering down safely.")

        self.swarm_finish_pub = rospy.Publisher('fms/finish', Bool, queue_size=10)
        rospy.sleep(2.0)

        self.swarm_finish_pub.publish(Bool(data=True))

        rospy.sleep(1)
        return 'done'


# --- Main FSM Container ---

def main():
    rospy.init_node('cp_fsm')

    sm = smach.StateMachine(outcomes=['fsm_complete'])
    sm.userdata.formation_points = {}

    with sm:
        smach.StateMachine.add('INIT', Init(), transitions={
            'initialized': 'INIT_TAKE_OFF',
            'timeout': 'INIT',
            'failed': 'SHUTDOWN'
        })

        smach.StateMachine.add('INIT_TAKE_OFF', InitTakeOff(), transitions={
            'passed': 'INIT_SWARM',
            'failed': 'ABORT'
        }, remapping={'formation_points_out':'formation_points'})

        smach.StateMachine.add('INIT_SWARM', InitSwarm(), transitions={
            'passed': 'MONITOR',
            'failed': 'ABORT'
        }, remapping={'formation_points_in':'formation_points'})

        smach.StateMachine.add('MONITOR', Monitor(), transitions={
            'continue': 'INIT_SWARM',
            'aborted': 'ABORT',
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
