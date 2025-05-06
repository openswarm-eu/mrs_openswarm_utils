#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped

class OdomToPath:
    def __init__(self):
        rospy.init_node('odom_to_path_node', anonymous=True)

        # Parameters
        self.odom_topic = rospy.get_param("~odom_topic", "estimation_manager/odom_main")
        self.path_topic = rospy.get_param("~path_topic", "odom_path")

        # Path message initialization
        self.path = Path()
        self.path.header.frame_id = "world"  # Change to "map" if needed

        # Publisher
        self.path_pub = rospy.Publisher(self.path_topic, Path, queue_size=10)

        # Subscriber
        rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback)

        rospy.loginfo("Odom to Path node started. Subscribed to %s", self.odom_topic)

    def odom_callback(self, msg):
        pose_stamped = PoseStamped()
        pose_stamped.header = msg.header
        pose_stamped.pose = msg.pose.pose

        self.path.header.stamp = rospy.Time.now()
        self.path.poses.append(pose_stamped)

        self.path_pub.publish(self.path)

if __name__ == '__main__':
    try:
        OdomToPath()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
