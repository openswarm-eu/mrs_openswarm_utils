#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
import math

class EuclideanDistanceNode:
    def __init__(self):
        rospy.init_node('euclidean_distance_node', anonymous=False)

        # Load reference point from ROS parameters
        self.ref_x = rospy.get_param("~ref_x", 4.9)
        self.ref_y = rospy.get_param("~ref_y", -3.7)

        # Fixed publish rate (every 2 seconds)
        self.publish_interval = 2.0  # seconds

        # Placeholder for current position
        self.current_position = None

        # Publishers and Subscribers
        self.distance_pub = rospy.Publisher('/uav9/home_distance', Float64, queue_size=10)
        rospy.Subscriber('/uav9/estimation_manager/odom_main', Odometry, self.odom_callback)

        # Timer to trigger distance publishing
        rospy.Timer(rospy.Duration(self.publish_interval), self.publish_distance)

        rospy.loginfo("EuclideanDistanceNode initialized with reference point: x = %.2f, y = %.2f", self.ref_x, self.ref_y)

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.current_position = (x, y)
        rospy.logdebug("Odometry received: x = %.2f, y = %.2f", x, y)

    def publish_distance(self, event):
        if self.current_position is not None:
            dx = self.current_position[0] - self.ref_x
            dy = self.current_position[1] - self.ref_y
            distance = math.sqrt(dx**2 + dy**2)
            self.distance_pub.publish(Float64(distance))
            rospy.loginfo("Published Distance: %.3f", distance)

if __name__ == '__main__':
    try:
        EuclideanDistanceNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.logerr("EuclideanDistanceNode interrupted.")
