#!/usr/bin/env python

import rospy
import tf2_ros
import geometry_msgs.msg
from geodesy import utm
from geographic_msgs.msg import GeoPoint

def create_transform(parent_frame, child_frame, utm_point, altitude):
    tf = geometry_msgs.msg.TransformStamped()
    tf.header.stamp = rospy.Time.now()
    tf.header.frame_id = parent_frame
    tf.child_frame_id = child_frame

    tf.transform.translation.x = utm_point.easting
    tf.transform.translation.y = utm_point.northing
    tf.transform.translation.z = altitude

    tf.transform.rotation.x = 0.0
    tf.transform.rotation.y = 0.0
    tf.transform.rotation.z = 0.0
    tf.transform.rotation.w = 1.0

    return tf

def publish_static_transform():
    rospy.init_node('static_utm_gps_tf_broadcaster')

    drone_list = rospy.get_param('~uav_names', [])
    uav_name = rospy.get_param("~uav_name", "uav1")
    latitude = rospy.get_param("~latitude", 41.22061958)
    longitude = rospy.get_param("~longitude", -8.52732281)


    broadcaster = tf2_ros.StaticTransformBroadcaster()

    # Base station coordinates
    geo_point = GeoPoint()
    geo_point.latitude = latitude
    geo_point.longitude = longitude
    geo_point.altitude = 0.0

    # Convert to UTM
    utm_point = utm.fromMsg(geo_point)

    # Create transforms
    # tf1 = create_transform("uav12/utm_origin", "uav12/liosam_origin", utm_point, geo_point.altitude)
    # tf2 = create_transform("uav11/utm_origin", "uav11/liosam_origin", utm_point, geo_point.altitude)
    # tf3 = create_transform("uav13/utm_origin", "uav13/liosam_origin", utm_point, geo_point.altitude)
    # tf4 = create_transform("uav6/utm_origin", "uav6/liosam_origin", utm_point, geo_point.altitude)
    # tf5 = create_transform("uav7/utm_origin", "uav7/liosam_origin", utm_point, geo_point.altitude)
    # tf6 = create_transform("uav8/utm_origin", "uav8/liosam_origin", utm_point, geo_point.altitude)

    tf1 = create_transform("uav1/liosam_origin", "uav1/utm_origin1", utm_point, geo_point.altitude)
    tf2 = create_transform("uav7/liosam_origin", "uav7/utm_origin1", utm_point, geo_point.altitude)
    tf3 = create_transform("uav8/liosam_origin", "uav8/utm_origin1", utm_point, geo_point.altitude)
    tf4 = create_transform("uav9/liosam_origin", "uav9/utm_origin1", utm_point, geo_point.altitude)
    tf5 = create_transform("uav10/liosam_origin", "uav10/utm_origin1", utm_point, geo_point.altitude)
    tf6 = create_transform("uav11/liosam_origin", "uav11/utm_origin1", utm_point, geo_point.altitude)
    tf7 = create_transform("uav12/liosam_origin", "uav12/utm_origin1", utm_point, geo_point.altitude)
    tf8 = create_transform("uav13/liosam_origin", "uav13/utm_origin1", utm_point, geo_point.altitude)
    tf9 = create_transform("uav14/liosam_origin", "uav14/utm_origin1", utm_point, geo_point.altitude)

    # Send all transforms at once
    broadcaster.sendTransform([tf1, tf2, tf3, tf4, tf5, tf6, tf7, tf8, tf9])

    rospy.spin()

if __name__ == '__main__':
    publish_static_transform()
