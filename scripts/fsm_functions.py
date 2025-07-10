#!/usr/bin/env python
import rospy
import utm
import math
import numpy as np
from geometry_msgs.msg import PoseStamped
import tf.transformations as tf_trans

def latlon_to_utm(lat, lon, alt=0.0):
    """Convert latitude and longitude to UTM coordinates."""
    u = utm.from_latlon(lat, lon)
    return u[0], u[1], alt  # x, y, z

def utm_to_latlon(x, y, zone_number, zone_letter):
    """Convert UTM coordinates to latitude and longitude."""
    latlon = utm.to_latlon(x, y, zone_number, zone_letter)
    return latlon[0], latlon[1]  # lat, lon

def create_pose_stamped(x, y, z, frame_id="utm_origin"):
    pose = PoseStamped()
    pose.header.stamp = rospy.Time.now()
    pose.header.frame_id = frame_id
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z
    pose.pose.orientation.w = 1.0  # no rotation
    return pose

def rotate(x, y, theta):
    return (
        x * np.cos(theta) - y * np.sin(theta),
        x * np.sin(theta) + y * np.cos(theta)
    )

def compute_formation_positions(drone_list, pose_start_transformed, pose_end_transformed, distance, angle_formation_deg, formation_type="regular"):
    """
    Create a formation where all adjacent drones are `distance` meters apart.
    Leader is at (0, 0), others are placed in regular geometry.
    """
    num_drones = len(drone_list)
    angle_rad = np.deg2rad(angle_formation_deg)
    
    # Initialize positions dictionary   
    positions = {}

    # Leader at origin
    positions[drone_list[0]] = (0.0, 0.0)

    # Calculate the direction angle based on the provided coordinates
    dx = pose_end_transformed.pose.position.x - pose_start_transformed.pose.position.x
    dy = pose_end_transformed.pose.position.y - pose_start_transformed.pose.position.y

    direction_angle = np.pi - np.arctan2(dx, dy)
    total_rotation = direction_angle + angle_rad

    if num_drones == 1:
        rospy.loginfo(f"[FORMATION_FUNCTION]: One drone with formation {formation_type}.")
        sequence = [0]
        # Single drone at origin
        positions[drone_list[0]] = (0.0, 0.0)

    elif num_drones == 3:
        rospy.loginfo(f"[FORMATION_FUNCTION]: Three drones with formation {formation_type}.")
        sequence = [0, 1, 2]
        # Triangle: 2 followers behind leader at 120° separation
        r = - distance - 1.5
        angles = [np.deg2rad(150), np.deg2rad(210)]
        for i, angle in enumerate(angles):
            x = r * np.cos(angle)
            y = r * np.sin(angle)
            positions[drone_list[i + 1]] = rotate(x, y, np.pi / 2 + total_rotation)

    elif num_drones == 6:
        rospy.loginfo(f"[FORMATION_FUNCTION]: Six drones with formation {formation_type}.")
        # Regular Pentagon + center
        if formation_type == "pentagon_center":
            sequence = [0, 1, 5, 2, 4, 3]
            sides = 5
            radius = -distance / (2 * np.sin(np.pi / sides))
            angles = np.linspace(0, 2 * np.pi, sides, endpoint=False)
            for i, a in enumerate(angles):
                x = radius * np.cos(a)
                y = radius * np.sin(a)
                positions[drone_list[i + 1]] = rotate(x, y, np.pi / 2 + total_rotation)
            # Regular Hexagon (6 drones)
        elif formation_type == "regular":
            sequence = [0, 1, 2, 3, 4, 5]
            sides = 6
            radius = -distance / (2 * np.sin(np.pi / sides))
            angles = np.linspace(0, 2 * np.pi, sides, endpoint=False)

            for i, a in enumerate(angles):
                x = radius * np.cos(a)
                y = radius * np.sin(a)
                positions[drone_list[i]] = rotate(x, y, np.pi / 2 + total_rotation)
        elif formation_type == "v_formation":
            # V-formation: 6 drones in a V shape with leader at origin
            sequence = [0, 1, 5, 2, 4, 3]
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
            raise ValueError(f"Unsupported six_formation: {formation_type}")

    elif num_drones == 9:
        rospy.loginfo(f"[FORMATION_FUNCTION]: Nine drones with formation {formation_type}.")
        sequence = [0, 1, 2, 3, 4, 5, 6, 7, 8]
        # Octagon (8 drones) + center
        if formation_type == "octagon_center":
            sides = 8
            # Compute radius so adjacent drones are 'distance' apart
            radius = distance / (2 * np.sin(np.pi / sides))
            angles = np.linspace(np.pi / 8, 2 * np.pi + np.pi / 8, sides, endpoint=False)
            for i, theta in enumerate(angles):
                x = radius * np.cos(theta)
                y = radius * np.sin(theta)
                x_rot, y_rot = rotate(x, y, np.pi / 2 + angle_rad)
                positions[drone_list[i + 1]] = (x_rot, y_rot)
        elif formation_type == "regular":
            sides = 6
            radius = distance / (2 * np.sin(np.pi / sides))
            angles = np.linspace(0, 2 * np.pi, sides, endpoint=False)

            for i, a in enumerate(angles):
                x = radius * np.cos(a)
                y = radius * np.sin(a)
                positions[drone_list[i]] = rotate(x, y, np.pi / 2 + angle_rad)
        elif formation_type == "v_formation":
            drone_index = 1  # Start from the first follower
            r = distance
            base_angle_deg = 60
            angles_deg = [180 - base_angle_deg, 180 + base_angle_deg]  # [150°, 210°]
            angles_rad = [np.deg2rad(a) for a in angles_deg]
            rospy.loginfo(f"[FORMATION]: V-formation angles (degrees): {angles_deg}, (radians): {angles_rad}")
            for layer in range(1, 5):  # 4 layers (2 drones per layer = 8 followers)
                rospy.loginfo(f"[FORMATION]: Layer {layer} with distance {distance}")
                for i, angle in enumerate(angles_rad):
                    x = layer * r * np.cos(angle)
                    y = layer * r * np.sin(angle)
                    positions[drone_list[drone_index]] = rotate(x, y, np.pi / 2 + angle_rad)
                    rospy.loginfo(f"[FORMATION]: Drone positions: {positions[drone_list[drone_index]]}")
                    drone_index += 1
    else:
        raise ValueError(f"Unsupported formation for {num_drones} drones.")

    return positions, sequence

def generate_swarm_poses(origin_pose: PoseStamped, drone_list: list, positions: dict) -> list:
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
    rospy.loginfo(f"[SWARM_POSE_FUNCTION]: Generate_swarm_poses: {positions}")

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