#!/usr/bin/env python

import rospy
import yaml
import math
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler

class pathGeneratorClass():
    def __init__(self):

        # Publisher to publish the path
        path_publisher = rospy.Publisher("move_base_path/swarm", Path, queue_size=10)
        rate = rospy.Rate(1)

        self.yaml_file_path = rospy.get_param('~yaml_file', 'poses.yaml')
        self.frame = rospy.get_param('~frame', 'map')

        # Read the YAML file
        self.yaml_data = self.read_yaml_file(self.yaml_file_path)

        # Create a nav_msgs/Path from the YAML data
        self.path = self.create_path_from_yaml(self.yaml_data)

        count = 3
        while not rospy.is_shutdown():
            rospy.loginfo("Publishin in %d", count)
            if count == 0:
                path_publisher.publish(self.path)
                rospy.loginfo("Published")
                break
            count -= 1
            rate.sleep()


    def create_path_from_yaml(self, yaml_data):
        """Creates a nav_msgs/Path message from parameter file."""
        path = Path()
        path.header.stamp = rospy.Time.now()
        path.header.frame_id = self.frame

        for pose in yaml_data:
            pose_stamped = PoseStamped()
            pose_stamped.header.stamp = rospy.Time.now()
            pose_stamped.header.frame_id = self.frame
            pose_stamped.pose.position.x = pose['x']
            pose_stamped.pose.position.y = pose['y']
            pose_stamped.pose.position.z = 0.0  # Assume 2D for simplicity

            # Convert yaw to quaternion
            quaternion = quaternion_from_euler(0, 0, pose['yaw'])
            pose_stamped.pose.orientation.x = quaternion[0]
            pose_stamped.pose.orientation.y = quaternion[1]
            pose_stamped.pose.orientation.z = quaternion[2]
            pose_stamped.pose.orientation.w = quaternion[3]

            path.poses.append(pose_stamped)

        return path

    def read_yaml_file(self, file_path):
        """Reads a YAML file containing x, y, and yaw values."""
        with open(file_path, 'r') as file:
            data = yaml.safe_load(file)
        return data['poses']


if __name__ == '__main__':
    rospy.init_node('pose_generator', anonymous=True)
    try:
        path = pathGeneratorClass()
    except rospy.ROSInternalException:
        pass
