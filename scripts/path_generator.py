#!/usr/bin/env python3
import math
import yaml
import rospy
from mrs_msgs.srv import Vec4, Vec4Request
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from rospy.exceptions import ROSInterruptException
import os

class WaypointGotoCaller:
    def __init__(self):
        self.home_set = False
        self.home_position = None

        # Load parameters
        self.rate_hz = rospy.get_param("~rate", 0.2)  # default 0.2 Hz
        self.yaml_path = rospy.get_param("~waypoints_file", "waypoints.yaml")

        # Load waypoints from YAML
        self.waypoints = self.load_waypoints(self.yaml_path)

        # Prepare service client
        service_name = "octomap_planner/goto"
        rospy.loginfo(f"Waiting for service {service_name}...")
        rospy.wait_for_service(service_name)

        self.goto_srv = rospy.ServiceProxy(service_name, Vec4)
        rospy.loginfo(f"Connected to {service_name}.")

        # Publishers and Subscribers
        rospy.Subscriber('estimation_manager/odom_main', Odometry, self.odom_callback)
        self.distance_pub = rospy.Publisher("distance_from_home", Float64, queue_size=10)

    def load_waypoints(self, yaml_path):
        if not os.path.exists(yaml_path):
            rospy.logerr(f"YAML file not found: {yaml_path}")
            return []

        with open(yaml_path, "r") as f:
            data = yaml.safe_load(f)

        if "waypoints" not in data:
            rospy.logerr("YAML file must contain 'waypoints' key")
            return []

        rospy.loginfo(f"Loaded {len(data['waypoints'])} waypoints from {yaml_path}")
        return data["waypoints"]

    def odom_callback(self, msg: Odometry):
        if not self.home_set:
            self.home_position = msg.pose.pose.position
            self.home_position.z = 4.0
            self.home_set = True
            rospy.loginfo(
                "Home position set: [%.2f, %.2f, %.2f]",
                self.home_position.x,
                self.home_position.y,
                self.home_position.z,
            )
            return

        dx = msg.pose.pose.position.x - self.home_position.x
        dy = msg.pose.pose.position.y - self.home_position.y
        dz = msg.pose.pose.position.z - self.home_position.z

        distance = math.sqrt(dx * dx + dy * dy + dz * dz)

        dist_msg = Float64()
        dist_msg.data = distance
        self.distance_pub.publish(dist_msg)

    def call_goto(self, goal):
        request = Vec4Request()
        request.goal = goal
        try:
            rospy.loginfo(f"Calling goto with target {goal}")
            res = self.goto_srv(request)
            rospy.loginfo(f"Service response: {res}")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

    def run(self):

        while not self.home_set and not rospy.is_shutdown():
            rospy.loginfo("Waiting for home position to be set...")
            rospy.sleep(1.0)

        rate = rospy.Rate(self.rate_hz)

        # Execute waypoints in sequence
        for wp in self.waypoints:
            if rospy.is_shutdown():
                break
            # Expect waypoint as [x, y, z, yaw]
            self.call_goto(wp)
            rate.sleep()

        # Finally go back home
        home_goal = [
            self.home_position.x,
            self.home_position.y,
            self.home_position.z,
            0.0,
        ]
        rospy.loginfo("Returning home...")
        self.call_goto(home_goal)


if __name__ == "__main__":
    rospy.init_node("waypoint_goto_caller", anonymous=False)
    try:
        node = WaypointGotoCaller()
        node.run()
    except ROSInterruptException:
        pass
