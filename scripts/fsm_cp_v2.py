#!/usr/bin/env python

import rospy
import smach
import smach_ros
from std_msgs.msg import Bool, String, UInt8MultiArray
from std_srvs.srv import Trigger, TriggerRequest
from mrs_msgs.srv import Vec4, Vec4Request
from mrs_msgs.msg import ControlManagerDiagnostics
import numpy as np
import math
from itertools import combinations
from visualization_msgs.msg import MarkerArray
from fsm_visualization import create_marker, create_text_marker

from geographic_msgs.msg import GeoPoint
import tf2_ros
import tf2_geometry_msgs
import tf.transformations as tf_trans
from geometry_msgs.msg import PoseStamped, PoseArray, Pose, PointStamped
from geographiclib.geodesic import Geodesic

# If you know your UTM zone, you can use 'utm' package:
# pip install utm
import utm

# --- Define FSM States ---

class Init(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['initialized', 'timeout'])

        # Get swarm list from parameter server
        self.drone_list = rospy.get_param('~uav_names', [])

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
        smach.State.__init__(self, outcomes=['passed', 'failed'])

        # Get swarm list from parameter server
        self.drone_list = rospy.get_param('~uav_names', [])
        self.uav_name = rospy.get_param("~uav_name", "uav1")

        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(30.0))
        self.listener = tf2_ros.TransformListener(self.tf_buffer)
        rospy.loginfo("[INIT_TAKE_OFF]: Waiting for TF to fill buffer.")
        rospy.sleep(1.0) 

        self.height_formation = rospy.get_param("~height_formation", 6.0)
        self.latitude_start = rospy.get_param("~latitude_start", 0.0)
        self.longitude_start = rospy.get_param("~longitude_start", 0.0) 
        self.latitude_end = rospy.get_param("~latitude_end", 0.0)
        self.longitude_end = rospy.get_param("~longitude_end", 0.0)
        self.six_formation = rospy.get_param("~six_formation", "regular")
        self.nine_formation = rospy.get_param("~nine_formation", "v_formation")

        self.publish_pose_array = rospy.Publisher('swarm_formation/positions', PoseArray, queue_size=10)

    def compute_formation_positions(self, drone_list, distance, angle_rad):
        """
        Create a formation where all adjacent drones are `distance` meters apart.
        Leader is at (0, 0), others are placed in regular geometry.
        """
        num_drones = len(drone_list)
        rospy.loginfo(f"[INIT_TAKE_OFF]: Computing formation positions for {num_drones} drones with distance {distance} and angle {angle_rad} radians.")

        # angle_rad = np.deg2rad(angle_formation_deg)
        positions = {}

        def rotate(x, y, theta):
            return (
                x * np.cos(theta) - y * np.sin(theta),
                x * np.sin(theta) + y * np.cos(theta)
            )

        # Leader at origin
        positions[drone_list[0]] = (0.0, 0.0)

        # Calculate the direction angle based on the provided coordinates
        dx = self.pose_end_transformed.pose.position.x - self.pose_start_transformed.pose.position.x
        dy = self.pose_end_transformed.pose.position.y - self.pose_start_transformed.pose.position.y

        direction_angle = np.pi - np.arctan2(dx, dy)
        total_rotation = direction_angle + angle_rad

        if num_drones == 1:
            # Single drone at origin
            positions[drone_list[0]] = (0.0, 0.0)

        elif num_drones == 3:
            # Triangle: 2 followers behind leader at 120° separation
            r = - distance - 1.5
            angles = [np.deg2rad(150), np.deg2rad(210)]
            for i, angle in enumerate(angles):
                x = r * np.cos(angle)
                y = r * np.sin(angle)
                positions[drone_list[i + 1]] = rotate(x, y, np.pi / 2 + total_rotation)

        elif num_drones == 6:
            # Regular Pentagon + center
            if self.six_formation == "pentagon_center":
                self.sequence = [0, 1, 5, 2, 4, 3]
                sides = 5
                radius = -distance / (2 * np.sin(np.pi / sides))
                angles = np.linspace(0, 2 * np.pi, sides, endpoint=False)
                for i, a in enumerate(angles):
                    x = radius * np.cos(a)
                    y = radius * np.sin(a)
                    positions[drone_list[i + 1]] = rotate(x, y, np.pi / 2 + total_rotation)
                # Regular Hexagon (6 drones)
            elif self.six_formation == "regular":
                # self.sequence = [0, 1, 5, 2, 4, 3]
                self.sequence = [0, 1, 2, 3, 4, 5]
                sides = 6
                radius = -distance / (2 * np.sin(np.pi / sides))
                angles = np.linspace(0, 2 * np.pi, sides, endpoint=False)

                for i, a in enumerate(angles):
                    x = radius * np.cos(a)
                    y = radius * np.sin(a)
                    positions[drone_list[i]] = rotate(x, y, np.pi / 2 + total_rotation)
            elif self.six_formation == "v_formation":
                # V-formation: 6 drones in a V shape with leader at origin
                self.sequence = [0, 1, 5, 2, 4, 3]
                # self.sequence = [0, 1, 2, 3, 4, 5]
                r = -distance
                angles = [np.deg2rad(150), np.deg2rad(210)]
                for i, angle in enumerate(angles):
                    x = r * np.cos(angle)
                    y = r * np.sin(angle)
                    positions[drone_list[i + 1]] = rotate(x, y, np.pi / 2 + total_rotation)
                for i, angle in enumerate(angles):
                    x = 2 * r * np.cos(angle)
                    y = 2 * r * np.sin(angle)
                    positions[drone_list[i + 3]] = rotate(x, y, np.pi / 2 + total_rotation)
                x = 3 * r * np.cos(angles[0])
                y = 3 * r * np.sin(angles[0])
                positions[drone_list[5]] = rotate(x, y, np.pi / 2 + + total_rotation)

            else:
                raise ValueError(f"Unsupported six_formation: {self.six_formation}")
 
        elif num_drones == 9:
            rospy.loginfo(f"[INIT_TAKE_OFF]: Nine drones with formation {self.nine_formation}.")
            # Octagon (8 drones) + center
            if self.nine_formation == "octagon_center":
                sides = 8
                # Compute radius so adjacent drones are 'distance' apart
                radius = distance / (2 * np.sin(np.pi / sides))
                angles = np.linspace(np.pi / 8, 2 * np.pi + np.pi / 8, sides, endpoint=False)
                for i, theta in enumerate(angles):
                    x = radius * np.cos(theta)
                    y = radius * np.sin(theta)
                    x_rot, y_rot = rotate(x, y, np.pi / 2 + angle_rad)
                    positions[drone_list[i + 1]] = (x_rot, y_rot)
            elif self.nine_formation == "regular":
                sides = 6
                radius = distance / (2 * np.sin(np.pi / sides))
                angles = np.linspace(0, 2 * np.pi, sides, endpoint=False)

                for i, a in enumerate(angles):
                    x = radius * np.cos(a)
                    y = radius * np.sin(a)
                    positions[drone_list[i]] = rotate(x, y, np.pi / 2 + angle_rad)
            elif self.nine_formation == "v_formation":
                drone_index = 1  # Start from the first follower
                r = distance
                base_angle_deg = 60
                angles_deg = [180 - base_angle_deg, 180 + base_angle_deg]  # [150°, 210°]
                angles_rad = [np.deg2rad(a) for a in angles_deg]
                rospy.loginfo(f"[INIT_TAKE_OFF]: V-formation angles (degrees): {angles_deg}, (radians): {angles_rad}")
                for layer in range(1, 5):  # 4 layers (2 drones per layer = 8 followers)
                    rospy.loginfo(f"[INIT_TAKE_OFF]: Layer {layer} with distance {distance}")
                    for i, angle in enumerate(angles_rad):
                        x = layer * r * np.cos(angle)
                        y = layer * r * np.sin(angle)
                        positions[drone_list[drone_index]] = rotate(x, y, np.pi / 2 + angle_rad)
                        rospy.loginfo(f"[INIT_TAKE_OFF]: Drone positions: {positions[drone_list[drone_index]]}")
                        drone_index += 1
        else:
            raise ValueError(f"Unsupported formation for {num_drones} drones.")

        return positions

    def compute_inter_drone_distances(self, positions):
        """
        Compute pairwise distances between drones.

        :param positions: dict of {drone_id: (x, y)}
        :return: list of tuples [(drone1, drone2, distance), ...]
        """
        distances = []
        drone_pairs = combinations(positions.items(), 2)

        for (id1, pos1), (id2, pos2) in drone_pairs:
            dx = pos1[0] - pos2[0]
            dy = pos1[1] - pos2[1]
            distance = math.hypot(dx, dy)
            distances.append((id1, id2, distance))

        return distances

    def latlon_to_utm(self, lat, lon, alt=0.0):
        """Convert latitude and longitude to UTM coordinates."""
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
                pose, target_frame, rospy.Duration(5.0)
            )
            return transformed_pose
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr(f"TF transform failed: {e}")
            return None

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

    def compute_angle_with_x_axis(self, pose_start: PoseStamped, pose_end: PoseStamped) -> float:
        """
        Compute the angle in radians between the line formed by pose_start and pose_end,
        and the x-axis in the (x, y) plane.

        :param pose_start: geometry_msgs.msg.PoseStamped
        :param pose_end: geometry_msgs.msg.PoseStamped
        :return: Angle in radians between the line and the x-axis
        """
        dx = pose_end.pose.position.x - pose_start.pose.position.x
        dy = pose_end.pose.position.y - pose_start.pose.position.y

        angle = math.atan2(dy, dx)  # atan2 handles all quadrants correctly

        return angle

    def generate_swarm_poses(self, origin_pose: PoseStamped, drone_list: list, positions: dict) -> list:
        """
        Generate a list of PoseStamped for each drone in the swarm based on a formation origin.

        :param origin_pose: PoseStamped, the formation's origin pose (e.g. leader)
        :param drone_list: list of drone IDs or names
        :param positions: dict with relative (x, y) positions from formation center
        :return: list of PoseStamped
        """
        origin_x = origin_pose.pose.position.x
        origin_y = origin_pose.pose.position.y
        origin_z = origin_pose.pose.position.z

        # Extract yaw from quaternion
        q = origin_pose.pose.orientation
        quaternion = [q.x, q.y, q.z, q.w]
        _, _, yaw = tf_trans.euler_from_quaternion(quaternion)

        pose_list = []
        rospy.loginfo(f"[INIT_TAKE_OFF]: generate_swarm_poses: {positions}")

        for drone in drone_list:
            dx, dy = positions[drone]

            # Rotate relative offset by origin yaw
            rel_x = dx * math.cos(yaw) - dy * math.sin(yaw)
            rel_y = dx * math.sin(yaw) + dy * math.cos(yaw)

            drone_pose = PoseStamped()
            drone_pose.header.stamp = rospy.Time.now()
            drone_pose.header.frame_id = origin_pose.header.frame_id

            drone_pose.pose.position.x = origin_x + rel_x
            drone_pose.pose.position.y = origin_y + rel_y
            drone_pose.pose.position.z = origin_z

            # Optional: copy same orientation (or customize if needed)
            drone_pose.pose.orientation = origin_pose.pose.orientation

            pose_list.append(drone_pose)

        return pose_list

    def convert_utm_local(self, latitude, longitude, altitude, frame_id, target_frame):
        # Convert to UTM
        x, y, z = self.latlon_to_utm(latitude, longitude, altitude)
        
        # Create pose in utm_origin
        pose_in_utm_origin = self.create_pose_stamped(x, y, z, frame_id)
        
        return self.transform_pose(pose_in_utm_origin, target_frame)

    def init_drones (self, latitude, longitude, altitude, frame_id, fomation_points):
        # Convert to UTM
        x, y, z = self.latlon_to_utm(latitude, longitude, altitude)

        # Create pose in utm_origin
        pose_in_utm_origin = self.create_pose_stamped(x, y, z, frame_id)

        # Create swarm poses in utm_origin
        swarm_poses = self.generate_swarm_poses(pose_in_utm_origin, self.drone_list, fomation_points)

        return swarm_poses

    def execute(self, userdata):
        rospy.loginfo("[INIT_TAKE_OFF]: Initializing Take-off State.")

        # Init Markers Publisher to visualize
        pub = rospy.Publisher("formation_markers", MarkerArray, queue_size=10)
        self.publishers_drones = rospy.Publisher('fms/swarm_check', PoseStamped, queue_size=10)
        #self.publish_pose_array = rospy.Publisher('swarm_formation/positions', PoseArray, queue_size=10)
        rospy.sleep(2.0)

        # Initialize the marker array
        marker_array = MarkerArray()

        # Mark start point
        x, y, z = self.latlon_to_utm(self.latitude_start, self.longitude_start, self.height_formation)
        pose_start_utm = self.create_pose_stamped(x, y, z, self.drone_list[0] + "/utm_origin")
        self.pose_start_transformed = self.transform_pose_frame(pose_start_utm, \
                                            self.drone_list[0] + "/liosam_origin", \
                                            self.drone_list[0] + "/utm_origin")
        # rospy.loginfo(f"[INIT_TAKE_OFF]: Pose Start Transformed: {self.pose_start_transformed}")

        index = 30
        marker_array.markers.append(create_marker("common_origin", index, self.pose_start_transformed))
        marker_array.markers.append(create_text_marker("common_origin", 100 + index, "start", self.pose_start_transformed))
        pub.publish(marker_array)

        # Mark end point
        x, y, z = self.latlon_to_utm(self.latitude_end, self.longitude_end, self.height_formation)
        pose_end_utm = self.create_pose_stamped(x, y, z, self.drone_list[0] + "/utm_origin")
        self.pose_end_transformed = self.transform_pose_frame(pose_end_utm, \
                                            self.drone_list[0] + "/liosam_origin", \
                                            self.drone_list[0] + "/utm_origin")
        #rospy.loginfo(f"[INIT_TAKE_OFF]: Pose End Transformed: {self.pose_end_transformed}")

        index = 20
        marker_array.markers.append(create_marker("common_origin", index, self.pose_end_transformed))
        marker_array.markers.append(create_text_marker("common_origin", 100 + index, "end", self.pose_end_transformed))
        pub.publish(marker_array)
        
        # Choose the type of format for the takeoff command
        if len(self.drone_list) == 1:
            rospy.loginfo("One drone.")
            # Create a formation
            formation_points = self.compute_formation_positions(self.drone_list, 0.0, 0.0)
            # Create poses # UTM    
            swarm_poses = self.init_drones(self.latitude_start, self.longitude_start, self.height_formation, \
                                           self.drone_list[0] + "/utm_origin", formation_points)
            # rospy.loginfo(f"[INIT_TAKE_OFF]: Swarm Poses: {swarm_poses}")
            # Sequence of drones
            swarm_poses_list = []
            sequence = [0]
            for index, name in enumerate(self.drone_list):
                name = self.drone_list[sequence[index]]
                rospy.loginfo(f"[INIT_TAKE_OFF]: drone: {name}")
                swarm_poses[index].header.stamp = rospy.Time.now()
                swarm_poses_transformed = self.transform_pose_frame(swarm_poses[sequence[index]], \
                                                    name + "/liosam_origin", \
                                                    name + "/utm_origin")
                # rospy.loginfo(f"[INIT_TAKE_OFF]: Pose {swarm_poses_transformed}.")
                # if swarm_poses_transformed == None:
                #     # Try Leader TF
                #     swarm_poses_transformed = self.transform_pose_frame(swarm_poses[sequence[index]], \
                #                     self.drone_list[0] + "/liosam_origin", \
                #                     self.drone_list[0] + "/utm_origin")
                #     # rospy.loginfo(f"[INIT_TAKE_OFF]: Pose {swarm_poses_transformed}.")
                #     if swarm_poses_transformed == None:
                #         rospy.logerr(f"[INIT_TAKE_OFF]: TF transform failed for {name}.")
                #         return 'failed'

                # swarm_poses_transformed.header.frame_id = name + "/liosam_origin"
                
                pose_list = []
                pose_list.append(swarm_poses_transformed)
                marker_array.markers.append(create_marker("common_origin", index, pose_list[0]))
                marker_array.markers.append(create_text_marker("common_origin", 100 + index, self.drone_list[sequence[index]], pose_list[0]))
                pub.publish(marker_array)

                # rospy.loginfo(f"[INIT_TAKE_OFF]: Swarm Pose of {name}: {swarm_poses_transformed}")
                self.publishers_drones.publish(swarm_poses_transformed)
                swarm_poses_list.append(swarm_poses_transformed)
                rospy.sleep(10)

            # return 'passed'
        
        elif len(self.drone_list) == 3:
            rospy.loginfo("Leader + two drones in an equilateral triangle.")
            # Create a formation
            # formation_points = self.compute_formation_positions(self.drone_list, self.height_formation, -1.57)
            formation_points = self.compute_formation_positions(self.drone_list, self.height_formation, 0.0)
            # Create poses
            swarm_poses = self.init_drones(self.latitude_start, self.longitude_start, self.height_formation, \
                                           "utm_origin", formation_points)

            # Sequence of drones
            swarm_poses_list = []
            sequence = [0, 1, 2]
            for index, name in enumerate(self.drone_list):
                name = self.drone_list[sequence[index]]
                rospy.loginfo(f"[INIT_TAKE_OFF]: drone: {name}")
                swarm_poses[index].header.stamp = rospy.Time.now()
                swarm_poses_transformed = self.transform_pose_frame(swarm_poses[sequence[index]], \
                                                    name + "/liosam_origin", \
                                                    name + "/utm_origin")
                # # rospy.loginfo(f"[INIT_TAKE_OFF]: Pose {swarm_poses_transformed}.")
                # if swarm_poses_transformed == None:
                #     # Try Leader TF
                #     swarm_poses_transformed = self.transform_pose_frame(swarm_poses[sequence[index]], \
                #                     self.drone_list[0] + "/liosam_origin", \
                #                     self.drone_list[0] + "/utm_origin")
                #     # rospy.loginfo(f"[INIT_TAKE_OFF]: Pose {swarm_poses_transformed}.")
                #     if swarm_poses_transformed == None:
                #         rospy.logerr(f"[INIT_TAKE_OFF]: TF transform failed for {name}.")
                #         return 'failed'

                # swarm_poses_transformed.header.frame_id = name + "/liosam_origin"
                
                pose_list = []
                pose_list.append(swarm_poses_transformed)
                marker_array.markers.append(create_marker("common_origin", index, pose_list[0]))
                marker_array.markers.append(create_text_marker("common_origin", 100 + index, self.drone_list[sequence[index]], pose_list[0]))
                pub.publish(marker_array)

                rospy.loginfo(f"[INIT_TAKE_OFF]: Swarm Pose of {name}: {swarm_poses_transformed}")
                self.publishers_drones.publish(swarm_poses_transformed)
                swarm_poses_list.append(swarm_poses_transformed)
                rospy.sleep(10)

        elif len(self.drone_list) == 6:
            rospy.loginfo("Six drones.")

            # Create a formation
            # formation_points = self.compute_formation_positions(self.drone_list, self.height_formation, -1.57)
            formation_points = self.compute_formation_positions(self.drone_list, self.height_formation, 0.0)
            # Create poses
            swarm_poses = self.init_drones(self.latitude_start, self.longitude_start, self.height_formation, \
                                           "utm_origin", formation_points)

            # Sequence of drones
            swarm_poses_list = []
            for index, name in enumerate(self.drone_list):
                name = self.drone_list[self.sequence[index]]
                rospy.loginfo(f"[INIT_TAKE_OFF]: drone: {name}")
                swarm_poses[index].header.stamp = rospy.Time.now()
                swarm_poses_transformed = self.transform_pose_frame(swarm_poses[self.sequence[index]], \
                                                    name + "/utm_origin1", \
                                                    name + "/liosam_origin")
                # rospy.loginfo(f"[INIT_TAKE_OFF]: Pose {swarm_poses_transformed}.")
                if swarm_poses_transformed == None:
                    # Try Leader TF
                    swarm_poses_transformed = self.transform_pose_frame(swarm_poses[self.sequence[index]], \
                                    self.drone_list[0] + "/liosam_origin", \
                                    self.drone_list[0] + "/utm_origin")
                    # rospy.loginfo(f"[INIT_TAKE_OFF]: Pose {swarm_poses_transformed}.")
                    if swarm_poses_transformed == None:
                        rospy.logerr(f"[INIT_TAKE_OFF]: TF transform failed for {name}.")
                        return 'failed'

                swarm_poses_transformed.header.frame_id = name + "/liosam_origin"
                
                pose_list = []
                pose_list.append(swarm_poses_transformed)
                marker_array.markers.append(create_marker("common_origin", index, pose_list[0]))
                marker_array.markers.append(create_text_marker("common_origin", 100 + index, self.drone_list[self.sequence[index]], pose_list[0]))
                pub.publish(marker_array)

                # rospy.loginfo(f"[INIT_TAKE_OFF]: Swarm Pose of {name}: {swarm_poses_transformed}")
                self.publishers_drones.publish(swarm_poses_transformed)
                swarm_poses_list.append(swarm_poses_transformed)
                rospy.sleep(15)

        elif len(self.drone_list) == 9:
            rospy.loginfo("Nine drones.")

            # Create a formation
            # formation_points = self.compute_formation_positions(self.drone_list, self.height_formation, -1.57)
            formation_points = self.compute_formation_positions(self.drone_list, self.height_formation, 0.0)
            rospy.loginfo(f"[INIT_TAKE_OFF]: Formation points: {formation_points}")
            # Create poses
            swarm_poses = self.init_drones(self.latitude_start, self.longitude_start, self.height_formation, \
                                           "utm_origin", formation_points)

            # Sequence of drones
            swarm_poses_list = []
            # sequence = [0, 1, 5, 2, 4, 3]
            sequence = [0, 1, 2, 3, 4, 5, 6, 7, 8]
            for index, name in enumerate(self.drone_list):
                name = self.drone_list[sequence[index]]
                rospy.loginfo(f"[INIT_TAKE_OFF]: drone: {name}")
                swarm_poses[index].header.stamp = rospy.Time.now()
                swarm_poses_transformed = self.transform_pose_frame(swarm_poses[sequence[index]], \
                                                    name + "/utm_origin1", \
                                                    name + "/liosam_origin")
                # rospy.loginfo(f"[INIT_TAKE_OFF]: Pose {swarm_poses_transformed}.")
                if swarm_poses_transformed == None:
                    # Try Leader TF
                    swarm_poses_transformed = self.transform_pose_frame(swarm_poses[sequence[index]], \
                                    self.drone_list[0] + "/liosam_origin", \
                                    self.drone_list[0] + "/utm_origin")
                    # rospy.loginfo(f"[INIT_TAKE_OFF]: Pose {swarm_poses_transformed}.")
                    if swarm_poses_transformed == None:
                        rospy.logerr(f"[INIT_TAKE_OFF]: TF transform failed for {name}.")
                        return 'failed'

                rospy.loginfo(f"[INIT_TAKE_OFF]: Swarm Pose of {name}: {swarm_poses_transformed}")
                self.publishers_drones.publish(swarm_poses_transformed)
                swarm_poses_list.append(swarm_poses_transformed)
                rospy.sleep(20)
            
        else:
            rospy.loginfo("Unknown formation.") 
            return 'failed'

        # Swarm Poses
        pose_array = PoseArray()
        pose_array.header.stamp = rospy.Time.now()
        pose_array.header.frame_id = self.drone_list[0] + "/liosam_origin"

        x, y, z = self.latlon_to_utm(self.latitude_end, self.longitude_end, self.height_formation)
        for i in range(len(formation_points)):
            pose_end_utm = self.create_pose_stamped(x, y, z, self.drone_list[i] + "/utm_origin")
            # pose_end_transformed = self.transform_pose_frame(pose_end_utm, \
            #                                     self.drone_list[i] + "/liosam_origin", \
            #                                     self.drone_list[i] + "/utm_origin")
            pose = Pose()
            # pose.position.x = swarm_poses_list[i].pose.position.x
            # pose.position.y = swarm_poses_list[i].pose.position.y
            # pose.position.z = swarm_poses_list[i].pose.position.z
            # pose.orientation.x = swarm_poses_list[i].pose.orientation.x
            # pose.orientation.y = swarm_poses_list[i].pose.orientation.y
            # pose.orientation.z = swarm_poses_list[i].pose.orientation.z
            # pose.orientation.w = swarm_poses_list[i].pose.orientation.w
            pose.position.x = pose_end_utm.pose.position.x + formation_points[self.drone_list[i]][0]
            pose.position.y = pose_end_utm.pose.position.y + formation_points[self.drone_list[i]][1]
            pose.position.z = self.height_formation

            pose_array.poses.append(pose)

        self.publish_pose_array.publish(pose_array)
        rospy.loginfo("Published %d poses", len(formation_points))

        # marker_array = MarkerArray()
        # for i, (drone_id, pose) in enumerate(zip(self.drone_list, swarm_poses)):
        #     marker_array.markers.append(create_marker("world", i, pose))
        #     marker_array.markers.append(create_text_marker("world", 100 + i, drone_id, pose))
        # pub.publish(marker_array)

        # marker_array = MarkerArray()
        # for i, (drone_id, pose) in enumerate(zip(self.drone_list, swarm_poses_list)):
        #     marker_array.markers.append(create_marker("common_origin", i, pose))
        #     marker_array.markers.append(create_text_marker("common_origin", 100 + i, drone_id, pose))
        # pub.publish(marker_array)

        rospy.sleep(5)
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

    with sm:
        smach.StateMachine.add('INIT', Init(), transitions={
            'initialized': 'INIT_TAKE_OFF',
            'timeout': 'INIT'
        })

        smach.StateMachine.add('INIT_TAKE_OFF', InitTakeOff(), transitions={
            'passed': 'MONITOR',
            'failed': 'ABORT'
        })

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
