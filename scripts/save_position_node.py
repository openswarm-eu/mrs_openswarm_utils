#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from visualization_msgs.msg import MarkerArray, Marker
import tf2_ros
import tf
import json
import os
import threading
import termios
import tty
import sys
import math
from datetime import datetime
from fsm_visualization import create_marker, create_text_marker
from fsm_functions import create_pose_stamped, latlon_to_utm

class PoseSaver:
    def __init__(self):
        rospy.init_node('pose_saver_node', anonymous=True)
        self.drone_name = rospy.get_param('~uav_name', [])
        self.odom_source = rospy.get_param('~odom_source', False)
        
        self.pose = None
        self.lock = threading.Lock()
        self.entries = []
        self.marker_id = 0

        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.listener = tf2_ros.TransformListener(self.tf_buffer)
        rospy.sleep(1.0) 

        # Get output path from parameter or default to home dir
        self.path = rospy.get_param("~path", os.path.expanduser('~'))
        if not os.path.exists(self.path):
            os.makedirs(self.path)

        time_str = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        self.filename = os.path.join(self.path, f"saved_positions_{time_str}.json")

        # Subscribers and publishers
        if self.odom_source == False:
            rospy.Subscriber('~gnss', NavSatFix, self.gnss_callback)
        else:
            rospy.Subscriber('~odom', Odometry, self.odom_callback)
        self.marker_pub = rospy.Publisher('~saved_poses', MarkerArray, queue_size=10)

        # Input thread
        self.input_thread = threading.Thread(target=self.wait_for_input)
        self.input_thread.daemon = True
        self.input_thread.start()

        print("\n--- Pose Saver With Marker Node Started ---")
        print("Press [Enter] to save pose and publish marker.")
        print("Press [ESC] to save all data to JSON and exit.\n")

    def gnss_callback(self, msg):
        with self.lock:
            if msg.status.status == 2:
                self.pose = {
                    'coordinates': {
                        'lat': round(msg.latitude, 10),
                        'lon': round(msg.longitude, 10)
                    }
                }

    def odom_callback(self, msg):
        with self.lock:
            self.frame_origin = msg.header.frame_id
            # Orientation to Euler
            q = msg.pose.pose.orientation
            euler = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
            self.pose = {
                'position': {
                    'x': round(msg.pose.pose.position.x, 3),
                    'y': round(msg.pose.pose.position.y, 3),
                    'z': round(msg.pose.pose.position.z, 3)
                },
                'orientation': {
                    'yaw': round(math.degrees(euler[2]), 2)
                }
            }

    def transform_pose_frame(self, pose, target_frame, origin_frame):
        pose.header.frame_id = origin_frame
        try:
            # Wait until the transform becomes available
            self.tf_buffer.can_transform(
                target_frame, 
                pose.header.frame_id, 
                rospy.Time(0), 
                rospy.Duration(5.0)
            )
            transformed_pose = self.tf_buffer.transform(
                pose, target_frame, rospy.Duration(5.0)
            )
            return transformed_pose
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr(f"TF transform failed: {e}")
            return None

    def wait_for_input(self):
        while not rospy.is_shutdown():
            key = self.getch()
            if key == '\x1b':  # ESC
                self.shutdown()
                break
            elif key == '\r' or key == '\n':  # Enter
                with self.lock:
                    if self.pose:
                        self.save_pose(self.pose)
                        self.publish_marker(self.pose)
                    else:
                        print("Pose not yet available.")

    def save_pose(self, pose_dict):
        self.entries.append(pose_dict)
        if self.odom_source == False:
            print(f"Saved GPS coordinates: {pose_dict['coordinates']['lat']}, {pose_dict['coordinates']['lon']}")
        else:
            print(f"Saved pose with yaw: {pose_dict['orientation']['yaw']}°")

    def publish_marker(self, pose_dict):
        if self.odom_source == False:
            pos = pose_dict['coordinates']
            x, y, z = latlon_to_utm(pos['lat'], pos['lon'])
            pose_in_utm = create_pose_stamped(x, y, z, self.drone_name + "/utm_navsat")

            # Initialize the marker array
            marker_array = MarkerArray()
            marker_array.markers.append(create_marker("common_origin", self.marker_id, pose_in_utm))
            marker_array.markers.append(create_text_marker("common_origin", 100 + self.marker_id, "P" + f"{self.marker_id}", pose_in_utm, z_offset=5.0, scale=0.2))
            self.marker_pub.publish(marker_array)

            self.marker_id += 1

    def shutdown(self):
        self.write_to_json()
        print(f"\nJSON written to: {self.filename}")
        rospy.signal_shutdown("User exited with ESC.")

    def write_to_json(self):
        if self.entries:
            with open(self.filename, 'w') as f:
                json.dump(self.entries, f, indent=2)
        else:
            print("No data to save.")

    def getch(self):
        """Read one character from terminal (non-blocking)."""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            ch = sys.stdin.read(1)
            return ch
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

if __name__ == '__main__':
    try:
        saver = PoseSaver()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
