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
        smach.State.__init__(self, outcomes=['initialized', 'timeout'])

        # Get swarm list from parameter server
        self.drone_list = rospy.get_param('~uav_names', [])
        self.pass_init_takeoff = rospy.get_param('~pass_init_takeoff', False)

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
        self.swarm_init_ok = False

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
                    rospy.logerr_throttle(2,"[%s]: gnss=%s, lidar_3d=%s, swarm=%s, pre-flight=%s, armed=%s, offboard:%s",
                            name, self.drone_status_gnss[name], self.drone_status_lidar[name], self.drone_status_swarm[name], 
                            self.drone_status_pre_flight[name], self.drone_status_armed[name], self.drone_status_offboard[name])
                else:
                    rospy.logwarn_throttle(2,"[%s]: gnss=%s, lidar_3d=%s, swarm=%s, pre-flight=%s, armed=%s, offboard:%s",
                            name, self.drone_status_gnss[name], self.drone_status_lidar[name], self.drone_status_swarm[name], 
                            self.drone_status_pre_flight[name], self.drone_status_armed[name], self.drone_status_offboard[name])
            if all(self.drone_status_gnss.values()) and all(self.drone_status_lidar.values()) and \
                all(self.drone_status_swarm.values()) and all(self.drone_status_pre_flight.values()) and \
                    all(self.drone_status_armed.values()) and all(self.drone_status_offboard.values()):
            # if all(self.drone_status_gnss.values()) and all(self.drone_status_lidar.values()) and \
            #     all(self.drone_status_swarm.values()) and all(self.drone_status_pre_flight.values()):
            # if all(self.drone_status_gnss.values()) and all(self.drone_status_lidar.values()) and \
            #     all(self.drone_status_pre_flight.values()):
                self.swarm_init_ok = True

        self.swarm_pub.publish(Bool(data=True))
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
        rospy.loginfo(f"[INIT_TAKE_OFF]: Swarm Poses: {swarm_poses}")

        for index, name in enumerate(self.drone_list):
            name = self.drone_list[sequence[index]]
            rospy.loginfo(f"[INIT_TAKE_OFF]: Drone: {name}")
            swarm_poses[sequence[index]].header.stamp = rospy.Time.now()
            swarm_poses_transformed = self.transform_pose_frame(swarm_poses[sequence[index]], \
                                                name + "/stable_origin", \
                                                name + "/utm_navsat")
            swarm_poses_transformed.header.frame_id = name + "/stable_origin"
            # rospy.loginfo(f"[INIT_TAKE_OFF]: Pose {swarm_poses_transformed}.")

            # Mark pose
            marker_array.markers.append(create_marker("common_origin", index, swarm_poses[sequence[index]]))
            marker_array.markers.append(create_text_marker("common_origin", 100 + index, self.drone_list[sequence[index]], swarm_poses[sequence[index]]))
            pub.publish(marker_array)

            # Publish the pose to the drone
            self.publishers_drones.publish(swarm_poses[sequence[index]])

            rospy.sleep(10)
        
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

    def wait_for_enter(self):
        input("Press Enter to continue to next target...")

    def execute(self, userdata):
        rospy.logwarn("[INIT_SWARM]: Initializing Swarm State.")

        waypoint_list = [
            (self.latitude_end, self.longitude_end, self.height_formation)
        ]

        for idx, (latitude, longitude, height) in enumerate(waypoint_list):
            rospy.loginfo(f"[INIT_SWARM]: Publishing formation for waypoint #{idx + 1}")

            # Mark start point
            x, y, z = latlon_to_utm(self.latitude_start, self.longitude_start, self.height_formation)
            pose_start_utm = create_pose_stamped(x, y, z, self.drone_list[0] + "/utm_navsat")

            # Mark end point
            x, y, z = latlon_to_utm(latitude, longitude, self.height_formation)
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
                pose.position.z = height

                pose_array.poses.append(pose)

            self.publish_pose_array.publish(pose_array)
            rospy.loginfo("Published %d poses for waypoint #%d", len(pose_array.poses), idx + 1)

            self.latitude_start = latitude
            self.longitude_start = longitude

            self.wait_for_enter()

        rospy.sleep(2)
        return 'passed'


class Monitor(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['aborted', 'shutdown'])

    def execute(self, userdata):
        rospy.logwarn("[MONITOR]: The task will finish.")
        rospy.sleep(2)
        return 'shutdown'


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
            'timeout': 'INIT'
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
