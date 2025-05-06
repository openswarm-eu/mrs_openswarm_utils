#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker
from mrs_msgs.srv import Vec4, Vec4Request
import math

# Custom UAV namespace list
UAV_NAMES = ["uav8", "uav9", "uav10", "uav11", "uav12", "uav14"]

# Hexagon (rotated 30°, radius 3.0)
HEXAGON_RELATIVE_POS = [
    {'x': 2.598, 'y': 1.5, 'z': 0.0},
    {'x': 0.0,   'y': 3.0, 'z': 0.0},
    {'x': -2.598, 'y': 1.5, 'z': 0.0},
    {'x': -2.598, 'y': -1.5, 'z': 0.0},
    {'x': 0.0,   'y': -3.0, 'z': 0.0},
    {'x': 2.598, 'y': -1.5, 'z': 0.0},
]

class HexagonMission:
    def __init__(self):
        rospy.init_node('hexagon_marker_mission', anonymous=True)

        self.marker_pub = rospy.Publisher('/hexagon_markers', Marker, queue_size=10)
        self.pose_sub = rospy.Subscriber('/goal', PoseStamped, self.pose_callback)

        self.uav_services = {}
        self.init_service_proxies()

        rospy.loginfo("Hexagon node initialized and UAV service proxies ready.")

    def init_service_proxies(self):
        for name in UAV_NAMES:
            service_name = f'/{name}/control_manager/goto'
            rospy.loginfo(f"Waiting for service: {service_name}")
            rospy.wait_for_service(service_name)
            self.uav_services[name] = rospy.ServiceProxy(service_name, Vec4)
        rospy.loginfo("All UAV services connected.")

    def pose_callback(self, msg):
        rospy.loginfo("Received central pose. Publishing markers and preparing service calls.")
        center = msg.pose
        frame_id = msg.header.frame_id

        points = self.compute_absolute_points(center)

        self.publish_markers(points, frame_id)

        rospy.sleep(1.0)  # slight delay before service calls
        self.call_goto_services(points)

    def compute_absolute_points(self, center_pose):
        abs_points = []
        for rel in HEXAGON_RELATIVE_POS:
            x = center_pose.position.x + rel['x']
            y = center_pose.position.y + rel['y']
            z = center_pose.position.z + rel['z']
            abs_points.append((x, y, z))
        return abs_points

    # def publish_markers(self, points, frame_id):
    #     for i, (x, y, z) in enumerate(points):
    #         # Text marker
    #         text_marker = Marker()
    #         text_marker.header.frame_id = frame_id
    #         text_marker.header.stamp = rospy.Time.now()
    #         text_marker.ns = "hexagon_points"
    #         text_marker.id = i
    #         text_marker.type = Marker.TEXT_VIEW_FACING
    #         text_marker.action = Marker.ADD
    #         text_marker.pose.position = Point(x, y, z + 0.2)
    #         text_marker.pose.orientation.w = 1.0
    #         text_marker.scale.z = 0.4
    #         text_marker.color.a = 1.0
    #         text_marker.color.r = 0.1
    #         text_marker.color.g = 0.9
    #         text_marker.color.b = 0.2
    #         text_marker.text = str(UAV_NAMES[i])
    #         self.marker_pub.publish(text_marker)

    #         # Sphere marker
    #         sphere_marker = Marker()
    #         sphere_marker.header.frame_id = frame_id
    #         sphere_marker.header.stamp = rospy.Time.now()
    #         sphere_marker.ns = "hexagon_spheres"
    #         sphere_marker.id = i + 100  # Offset IDs to avoid collisions
    #         sphere_marker.type = Marker.SPHERE
    #         sphere_marker.action = Marker.ADD
    #         sphere_marker.pose.position = Point(x, y, z)
    #         sphere_marker.pose.orientation.w = 1.0
    #         sphere_marker.scale.x = 0.3
    #         sphere_marker.scale.y = 0.3
    #         sphere_marker.scale.z = 0.3
    #         sphere_marker.color.a = 1.0
    #         sphere_marker.color.r = 0.0
    #         sphere_marker.color.g = 0.6
    #         sphere_marker.color.b = 1.0
    #         self.marker_pub.publish(sphere_marker)

    def publish_markers(self, points, frame_id):
        for i, (x, y, z) in enumerate(points):
            marker = Marker()
            marker.header.frame_id = frame_id
            marker.header.stamp = rospy.Time.now()
            marker.ns = "hexagon_points"
            marker.id = i
            marker.type = Marker.TEXT_VIEW_FACING
            marker.action = Marker.ADD
            marker.pose.position = Point(x, y, z + 0.2)
            marker.pose.orientation.w = 1.0
            marker.scale.z = 0.4
            marker.color.a = 1.0
            marker.color.r = 0.1
            marker.color.g = 0.9
            marker.color.b = 0.2
            marker.text = str(UAV_NAMES[i])
            self.marker_pub.publish(marker)

    def call_goto_services(self, points):
        heading = 0.0

        for i, uav in enumerate(UAV_NAMES):
            x, y, z = points[i]
            request = Vec4Request()
            request.goal = [x, y, z, heading]

            try:
                rospy.loginfo(f"[{uav}] Sending goal: {request.goal}")
                response = self.uav_services[uav](request)
                rospy.loginfo(f"[{uav}] Response: success={response.success}, message='{response.message}'")
            except rospy.ServiceException as e:
                rospy.logerr(f"[{uav}] Service call failed: {e}")

            if i < len(UAV_NAMES) - 1:
                rospy.sleep(10.0)

if __name__ == '__main__':
    try:
        HexagonMission()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
