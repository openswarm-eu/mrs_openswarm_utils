#!/usr/bin/env python

import rospy
import smach
import smach_ros
from std_msgs.msg import Bool, String
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
from geometry_msgs.msg import PoseStamped, PointStamped
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
        self.drone_status = {name: False for name in self.drone_list}

    def make_callback(self, name):
        def callback(msg):
            if msg.data == "INIT":
                rospy.loginfo_throttle(1, f"{name} reported INIT")
                self.drone_status[name] = True
        return callback

    def execute(self, userdata):
        rospy.loginfo("State: INIT - Initializing Base Station.")

        # Dynamically subscribe to each drone's status
        self.subscribers_drones = []
        for name in self.drone_list:
            # rospy.loginfo(f"UAV: : {name}")
            topic = f"/{name}/fms/drone_status"
            sub = rospy.Subscriber(topic, String, self.make_callback(name))
            self.subscribers_drones.append(sub)

        self.swarm_pub = rospy.Publisher('fms/init', Bool, queue_size=10)

        timeout = rospy.Time.now() + rospy.Duration(10.0)

        while not all(self.drone_status.values()) and rospy.Time.now() < timeout:
            rospy.sleep(0.2)

        if all(self.drone_status.values()):
            rospy.loginfo("State: INIT - All drones are in INIT. Broadcasting swarm init trigger.")
            self.swarm_pub.publish(Bool(data=True))
            rospy.sleep(1.0)  # Give drones time to process
            return 'initialized'
        else:
            rospy.logwarn("State: INIT - Not all drones responded. Status: {self.drone_status}")
            return 'timeout'


class WaitDrones(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['passed', 'failed'])
        
        # Get swarm list from parameter server
        self.drone_list = rospy.get_param('~uav_names', [])

        # Dictionary to track drone sensor status
        self.drone_sensor = {name: False for name in self.drone_list}

        # Dictionary to track drone swarm status
        self.drone_swarm = {name: False for name in self.drone_list}

    def make_callback1(self, name):
        def callback(msg):
            if msg.data:
                rospy.loginfo(f"{name}: Sensor OK")
                self.drone_sensor[name] = True
        return callback

    def make_callback2(self, name):
        def callback(msg):
            if msg.data:
                rospy.loginfo(f"{name}: Swarm OK")
                self.drone_swarm[name] = True
        return callback

    def execute(self, userdata):
        rospy.loginfo("State: WAIT_DRONES - Waiting for messages from drones.")

        # Dynamically subscribe to each drone's status
        self.subscribers_sensors = []
        for name in self.drone_list:
            # rospy.loginfo(f"UAV: : {name}")
            topic = f"/{name}/fms/sensor_check"
            sub = rospy.Subscriber(topic, Bool, self.make_callback1(name))
            self.subscribers_sensors.append(sub)

        # Dynamically subscribe to each drone's status
        self.subscribers_swarm = []
        for name in self.drone_list:
            # rospy.loginfo(f"UAV: : {name}")
            topic = f"/{name}/fms/swarm_check"
            sub = rospy.Subscriber(topic, Bool, self.make_callback2(name))
            self.subscribers_swarm.append(sub)

        timeout = rospy.Time.now() + rospy.Duration(120)

        while not all(self.drone_swarm.values()) and rospy.Time.now() < timeout:
            rospy.sleep(0.2)

        if all(self.drone_swarm.values()):
            rospy.loginfo("WaitDrones: All the drones are communicating with me.")
            return 'passed'
        else:
            rospy.logwarn(f"WaitDrones: Not all drones responded. Status: {self.drone_swarm}")
            return 'failed'


class RcStart(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['manual_activation'])

    def execute(self, userdata):
        rospy.loginfo("State: RC_START.")
        rospy.sleep(1)
        return 'manual_activation'


class WaitArmed(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['passed', 'failed'])

        # Get swarm list from parameter server
        self.drone_list = rospy.get_param('~uav_names', [])

        # Dictionary to track drone status
        self.drone_received = {name: False for name in self.drone_list}
        self.drone_pre_flight = {name: False for name in self.drone_list}

    def make_callback(self, name):
        def callback(msg):
            self.drone_received[name] = True
            self.drone_pre_flight[name] = msg.data
        return callback

    def execute(self, userdata):
        rospy.loginfo("State: WAIT_ARMED.")

        # Dynamically subscribe to each drone's status
        self.subscribers_preflight = []
        for name in self.drone_list:
            # rospy.loginfo(f"UAV: : {name}")
            topic = f"/{name}/fms/pre_flight"
            sub = rospy.Subscriber(topic, Bool, self.make_callback(name))
            self.subscribers_preflight.append(sub)

        rospy.loginfo("State: WAIT_ARMED - Waiting for pre-flight check from drones.")

        while not all(self.drone_received.values()):
            rospy.sleep(0.2)

        rospy.loginfo("State: WAIT_ARMED - All drones have pre-flight checked.")
        rospy.loginfo("State: WAIT_ARMED - Waiting for RC start.")        
        
        while not all(self.drone_pre_flight.values()):
            rospy.sleep(0.2)

        rospy.loginfo("State: WAIT_ARMED - At least one drone is ready for flight.") 

        # Check if all drones are ready for flight
        timeout = rospy.Time.now() + rospy.Duration(10)
        while rospy.Time.now() < timeout:
            if all(self.drone_pre_flight.values()):
                rospy.loginfo("State: WAIT_ARMED - All drones are ready for flight.")
                return 'passed'
        
        for drone_name, status in self.drone_pre_flight.items():
            rospy.loginfo(f"Drone {drone_name} pre-flight status: {status}")
        return 'failed'


class InitTakeOff(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['passed', 'failed'])

        # Get swarm list from parameter server
        self.drone_list = rospy.get_param('~uav_names', [])
        self.drone_flying = {name: False for name in self.drone_list}

        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)
        rospy.loginfo("Waiting for TF to fill buffer...")
        rospy.sleep(1.0) 

        # Set origin (base station)
        self.base_lat = rospy.get_param("~latitude", 0.0)
        self.base_lon = rospy.get_param("~longitude", 0.0)
        self.height_formation = rospy.get_param("~height_formation", 3.0)
        # self.origin = GeoPoint(latitude=self.base_lat, longitude=self.base_lon, altitude=0.0)
        # self.origin_utm = geodesy.utm.fromMsg(self.origin)

        self.latitude_start = 41.2209807
        self.longitude_start = -8.5272058
        self.latitude_end = 41.2211611
        # self.longitude_end = -8.5271643
        self.longitude_end = -8.5272058

    def make_callback(self, name):
        def callback(msg):
            if msg.uav_name in self.drone_list:
                self.drone_flying[name] = msg.flying_normally
        return callback

    def init_service_proxies(self):
        # TakeOff Services
        for name in self.drone_list:
            service_name = f'/{name}/uav_manager/takeoff'
            rospy.loginfo(f"Waiting for take-off service: {service_name}")
            rospy.wait_for_service(service_name)
            self.uav_takeoff_services[name] = rospy.ServiceProxy(service_name, Trigger)
        # GoTo Services
        for name in self.drone_list:
            service_name = f'/{name}/control_manager/goto'
            rospy.loginfo(f"Waiting for goto service: {service_name}")
            rospy.wait_for_service(service_name)
            self.uav_goto_services[name] = rospy.ServiceProxy(service_name, Vec4)
        rospy.loginfo("All UAV services connected.")

    def compute_formation_positions(self, drone_list, distance, angle_rad):
        """
        Create a formation where all adjacent drones are `distance` meters apart.
        Leader is at (0, 0), others are placed in regular geometry.
        """
        num_drones = len(drone_list)
        # angle_rad = np.deg2rad(angle_formation_deg)
        positions = {}

        def rotate(x, y, theta):
            return (
                x * np.cos(theta) - y * np.sin(theta),
                x * np.sin(theta) + y * np.cos(theta)
            )

        # Leader at origin
        positions[drone_list[0]] = (0.0, 0.0)

        if num_drones == 3:
            # Triangle: 2 followers behind leader at 120° separation
            r = distance
            angles = [np.deg2rad(150), np.deg2rad(210)]
            for i, angle in enumerate(angles):
                x = r * np.cos(angle)
                y = r * np.sin(angle)
                positions[drone_list[i + 1]] = rotate(x, y, np.pi / 2 + angle_rad)

        elif num_drones == 6:
            # Regular Pentagon + center
            sides = 5
            radius = distance / (2 * np.sin(np.pi / sides))
            angles = np.linspace(0, 2 * np.pi, sides, endpoint=False)
            for i, a in enumerate(angles):
                x = radius * np.cos(a)
                y = radius * np.sin(a)
                positions[drone_list[i + 1]] = rotate(x, y, np.pi / 2 + angle_rad)

        elif num_drones == 9:
            # Octagon (8 drones) + center
            sides = 8
            # Compute radius so adjacent drones are 'distance' apart
            radius = distance / (2 * np.sin(np.pi / sides))
            angles = np.linspace(np.pi / 8, 2 * np.pi + np.pi / 8, sides, endpoint=False)
            for i, theta in enumerate(angles):
                x = radius * np.cos(theta)
                y = radius * np.sin(theta)
                x_rot, y_rot = rotate(x, y, np.pi / 2 + angle_rad)
                positions[drone_list[i + 1]] = (x_rot, y_rot)

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
                pose, target_frame, rospy.Duration(1.0)
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

    def call_goto_services(self, points):
        heading = 0.0

        for i, uav in enumerate(self.drone_list):
            x, y, z = points[i]
            request = Vec4Request()
            request.goal = [x, y, z, heading]

            try:
                rospy.loginfo(f"[{uav}] Sending goal: {request.goal}")
                response = self.uav_services[uav](request)
                rospy.loginfo(f"[{uav}] Response: success={response.success}, message='{response.message}'")
            except rospy.ServiceException as e:
                rospy.logerr(f"[{uav}] Service call failed: {e}")

            if i < len(self.drone_list) - 1:
                rospy.sleep(10.0)

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

    def execute(self, userdata):
        rospy.loginfo("State: INIT_TAKE_OFF.")
        pub = rospy.Publisher("formation_markers", MarkerArray, queue_size=10)

        # Choose the type of format for the takeoff command
        if len(self.drone_list) == 3:
            rospy.loginfo("Leader + two drones in an equilateral triangle.")
        elif len(self.drone_list) == 6:
            rospy.loginfo("Leader + five drones in a pentagon.")
        elif len(self.drone_list) == 9:
            rospy.loginfo("Leader + 8 drones in an octagon.")
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
        rospy.loginfo("State: INIT_TAKE_OFF - Initialize service proxies.")

        ######################################################################
        # Start
        # Convert to UTM
        x, y, z = self.latlon_to_utm(self.latitude_start, self.longitude_start, 0.0)

        # Create pose in utm_origin
        pose_in_utm_origin = self.create_pose_stamped(x, y, z, "uav11/utm_origin")

        # Transform to my_frame
        target_frame = "common_origin"

        pose_start = self.transform_pose(pose_in_utm_origin, target_frame)
        # if pose_start:
        #     rospy.loginfo(f"Transformed Pose in {target_frame}: {pose_start}")
        # else:
        #     rospy.logwarn("Could not transform pose.")

        # End
        # Convert to UTM
        x, y, z = self.latlon_to_utm(self.latitude_end, self.longitude_end, 0.0)

        # Create pose in utm_origin
        pose_in_utm_end = self.create_pose_stamped(x, y, z, "uav11/utm_origin")

        pose_end = self.transform_pose(pose_in_utm_end, target_frame)
        # if pose_end:
        #     rospy.loginfo(f"Transformed Pose in {target_frame}: {pose_end}")
        # else:
        #     rospy.logwarn("Could not transform pose.")

        angle = self.compute_angle_with_x_axis(pose_start, pose_end)
        rospy.loginfo(f"Angle: {angle}")

        # self.compute_yaw_formation()
        self.formation_points = self.compute_formation_positions(self.drone_list, 3.0, 0.0)

        # for drone, pos in self.formation_points.items():
        #     rospy.loginfo(f"{drone} -> Position: {pos}")

        # distances = self.compute_inter_drone_distances(self.formation_points)
        # for d1, d2, dist in distances:
        #     rospy.loginfo(f"Distance between {d1} and {d2}: {dist:.2f} meters")

        swarm_poses = self.generate_swarm_poses(pose_start, self.drone_list, self.formation_points)
        swarm_poses_end = self.generate_swarm_poses(pose_end, self.drone_list, self.formation_points)

        # for pose in swarm_poses:
        #     rospy.loginfo(pose)

        # rate = rospy.Rate(1)

        # while not rospy.is_shutdown():
        #     marker_array = MarkerArray()
        #     for i, (drone_id, (x, y)) in enumerate(self.formation_points.items()):
        #         marker_array.markers.append(create_marker("common_origin", i, x, y, z=0.0))
        #         marker_array.markers.append(create_text_marker("common_origin", 100 + i, drone_id, x, y, z=0.5))
        #     pub.publish(marker_array)
        #     rate.sleep()

        marker_array = MarkerArray()
        for i, (drone_id, pose) in enumerate(zip(self.drone_list, swarm_poses)):
            marker_array.markers.append(create_marker("world", i, pose))
            marker_array.markers.append(create_text_marker("world", 100 + i, drone_id, pose))
        pub.publish(marker_array)

        marker_array_end = MarkerArray()
        for i, (drone_id, pose) in enumerate(zip(self.drone_list, swarm_poses_end)):
            marker_array_end.markers.append(create_marker("world", i + 10, pose))
            marker_array_end.markers.append(create_text_marker("world", 100 + i + 10, drone_id, pose))
        pub.publish(marker_array_end)

        # Take-off
        if len(self.drone_list) == 3:
            rospy.loginfo("Leader + two drones in an equilateral triangle.")
            #################################################################
            # TIME 1
            #################################################################
            if not self.takeoff_service(self.drone_list[0]):
                return 'failed'

            timeout = rospy.Time.now() + rospy.Duration(20)
            while rospy.Time.now() < timeout:
                if self.drone_flying[self.drone_list[0]]:
                    rospy.loginfo(f"{self.drone_list[0]}: flying")
                    break
                rospy.sleep(0.2)
            if self.drone_flying[self.drone_list[0]] == False:
                rospy.loginfo(f"{self.drone_list[0]}: not flying")
                return 'failed'

            if not self.goto_service(self.drone_list[0],
                                     swarm_poses[0].pose.position.x,
                                     swarm_poses[0].pose.position.y,
                                     self.height_formation):
                return 'failed'

            #################################################################
            # TIME 2
            #################################################################
            if not self.takeoff_service(self.drone_list[1]):
                return 'failed'
            if not self.takeoff_service(self.drone_list[2]):
                return 'failed'

            timeout = rospy.Time.now() + rospy.Duration(20)
            while rospy.Time.now() < timeout:
                if self.drone_flying[self.drone_list[1]] and self.drone_flying[self.drone_list[2]]:
                    rospy.loginfo(f"{self.drone_list[1]}: flying")
                    rospy.loginfo(f"{self.drone_list[2]}: flying")
                    break
                rospy.sleep(0.2)
            if self.drone_flying[self.drone_list[1]] == False or self.drone_flying[self.drone_list[2]]:
                rospy.loginfo(f"{self.drone_list[1]} or {self.drone_list[2]}: not flying")
                return 'failed'

            if not self.goto_service(self.drone_list[1],
                                     swarm_poses[1].pose.position.x,
                                     swarm_poses[1].pose.position.y,
                                     self.height_formation):
                return 'failed'
            if not self.goto_service(self.drone_list[2],
                                     swarm_poses[2].pose.position.x,
                                     swarm_poses[2].pose.position.y,
                                     self.height_formation):
                return 'failed'
            #################################################################
        elif len(self.drone_list) == 6:
            rospy.loginfo("Leader + five drones in a pentagon.")
        elif len(self.drone_list) == 9:
            rospy.loginfo("Leader + 8 drones in an octagon.")
        else:
            rospy.loginfo("Unknown formation.") 
            return 'failed'


        rospy.sleep(5)
        return 'passed'


class Abort(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['shutdown'])

    def execute(self, userdata):
        rospy.logwarn("State: ABORT - Something went wrong.")
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
    rospy.init_node('cp_fsm')

    sm = smach.StateMachine(outcomes=['fsm_complete'])

    with sm:
        smach.StateMachine.add('INIT', Init(), transitions={
            'initialized': 'WAIT_DRONES',
            'timeout': 'INIT'
        })

        smach.StateMachine.add('WAIT_DRONES', WaitDrones(), transitions={
            'passed': 'RC_START',
            'failed': 'ABORT'
        })

        smach.StateMachine.add('RC_START', RcStart(), transitions={
            'manual_activation': 'WAIT_ARMED'
        })

        smach.StateMachine.add('WAIT_ARMED', WaitArmed(), transitions={
            'passed': 'INIT_TAKE_OFF',
            'failed': 'ABORT'
        })

        smach.StateMachine.add('INIT_TAKE_OFF', InitTakeOff(), transitions={
            'passed': 'SHUTDOWN',
            'failed': 'ABORT'
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
