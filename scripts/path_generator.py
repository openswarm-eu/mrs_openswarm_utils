#!/usr/bin/env python

import rospy
from mrs_msgs.srv import Vec4, Vec4Request
from std_srvs.srv import Trigger
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import math

class OctomapPlannerCaller:
    def __init__(self):
        rospy.init_node('octomap_planner_client', anonymous=True)

        # Home pose definition
        self.home_pose = Vec4Request()
        x = rospy.get_param('~x', 0.0)
        y = rospy.get_param('~y', 0.0)
        z = rospy.get_param('~z', 1.0)
        heading = rospy.get_param('~heading', 0.0)
        # Set home pose
        self.home_pose.goal = [x, y, z, heading]

        # # Path publisher
        # self.path_pub = rospy.Publisher('/planned_path', Path, queue_size=10)
        self.path_published = False

        # Wait for both services
        rospy.loginfo("Waiting for /octomap_planner/goto service...")
        rospy.wait_for_service('/uav9/octomap_planner/goto')
        self.goto_srv = rospy.ServiceProxy('/uav9/octomap_planner/goto', Vec4)
        rospy.loginfo("Connected to /octomap_planner/goto.")

        rospy.loginfo("Waiting for /octomap_planner/stop service...")
        rospy.wait_for_service('/uav9/octomap_planner/stop')
        self.stop_srv = rospy.ServiceProxy('/uav9/octomap_planner/stop', Trigger)
        rospy.loginfo("Connected to /octomap_planner/stop.")

        # Start timer
        rospy.Timer(rospy.Duration(2.0), self.timer_callback)

    def timer_callback(self, event):
        # Prevent repeated calls after first execution
        if self.path_published:
            return

        try:
            rospy.loginfo("Calling /octomap_planner/goto...")
            response = self.goto_srv(self.home_pose)
            rospy.loginfo("Service responded: %s", response)

            # # Create dummy path as the service doesn’t return one (adapt if it does)
            # path = Path()
            # path.header.frame_id = "map"
            # path.header.stamp = rospy.Time.now()

            # # Example path: three poses (simulate real planned path)
            # dummy_poses = [
            #     (0.0, 0.0, 1.0),
            #     (1.0, 1.0, 1.0),
            #     (2.0, 2.0, 1.0),
            # ]

            # total_distance = 0.0
            # prev_pose = None

            # for x, y, z in dummy_poses:
            #     pose = PoseStamped()
            #     pose.header.frame_id = "map"
            #     pose.header.stamp = rospy.Time.now()
            #     pose.pose.position.x = x
            #     pose.pose.position.y = y
            #     pose.pose.position.z = z
            #     pose.pose.orientation.w = 1.0  # Identity orientation
            #     path.poses.append(pose)

            #     if prev_pose:
            #         dx = x - prev_pose[0]
            #         dy = y - prev_pose[1]
            #         dz = z - prev_pose[2]
            #         total_distance += math.sqrt(dx*dx + dy*dy + dz*dz)

            #     prev_pose = (x, y, z)

            # # Publish path
            # self.path_pub.publish(path)
            # self.path_published = True
            # rospy.loginfo("Published planned path with total distance: %.2f meters", total_distance)

            # Call stop service
            rospy.loginfo("Calling /octomap_planner/stop...")
            stop_response = self.stop_srv()
            rospy.loginfo("Stop service responded: success=%s, message=%s", stop_response.success, stop_response.message)

        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)

if __name__ == '__main__':
    try:
        OctomapPlannerCaller()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
