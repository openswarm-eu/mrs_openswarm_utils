#!/usr/bin/env python

import rospy
import math
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64

class PathDistanceCalculator:
    def __init__(self):
        rospy.init_node('path_distance_calculator_node', anonymous=True)

        rospy.Subscriber('distributedMapping/path', Path, self.path_callback)
        self.distance_pub = rospy.Publisher('distance_travelled', Float64, queue_size=10)

        rospy.loginfo("Path Distance Calculator Node Initialized")
        rospy.spin()

    def path_callback(self, msg):
        total_distance = 0.0
        poses = msg.poses

        if len(poses) < 2:
            rospy.logwarn("Path contains less than 2 poses. No distance to calculate.")
            return

        for i in range(len(poses) - 1):
            p1 = poses[i].pose.position
            p2 = poses[i + 1].pose.position

            dist = math.sqrt(
                (p2.x - p1.x)**2 +
                (p2.y - p1.y)**2 +
                (p2.z - p1.z)**2
            )
            total_distance += dist

        rospy.loginfo("Total Path Distance: %.3f meters", total_distance)
        self.distance_pub.publish(Float64(total_distance))

if __name__ == '__main__':
    try:
        PathDistanceCalculator()
    except rospy.ROSInterruptException:
        pass
