#!/usr/bin/env python
import rospy
from visualization_msgs.msg import Marker

def create_marker(frame_id, marker_id, pose, color=(0.0, 1.0, 0.0), scale=0.3):
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = rospy.Time.now()
    marker.id = marker_id
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    marker.pose = pose.pose
    marker.scale.x = scale
    marker.scale.y = scale
    marker.scale.z = scale
    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]
    marker.color.a = 1.0
    marker.lifetime = rospy.Duration(0)
    return marker

def create_text_marker(frame_id, marker_id, text, pose, z_offset=1.0):
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = rospy.Time.now()
    marker.id = marker_id
    marker.type = Marker.TEXT_VIEW_FACING
    marker.action = Marker.ADD

    marker.pose.position.x = pose.pose.position.x
    marker.pose.position.y = pose.pose.position.y
    marker.pose.position.z = pose.pose.position.z + z_offset  # Lift text above sphere

    marker.scale.z = 0.6  # Larger text size

    marker.color.r = 0.0
    marker.color.g = 0.0
    marker.color.b = 1.0
    marker.color.a = 1.0 

    marker.text = text
    marker.lifetime = rospy.Duration()

    return marker
