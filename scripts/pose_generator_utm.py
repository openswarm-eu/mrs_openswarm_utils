#!/usr/bin/env python

import rospy
import yaml
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler
from fsm_functions import latlon_to_utm
from mrs_msgs.msg import ControlManagerDiagnostics
from mrs_msgs.srv import Vec4, Vec4Request

class pathGeneratorClass():
    def __init__(self):

        self.yaml_file_path = rospy.get_param('~yaml_file', 'poses.yaml')
        self.frame = rospy.get_param('~frame', 'map')
        self.timer_main_rate = rospy.get_param("~rate")
        self.drone_name = rospy.get_param('~uav_name', [])
        self.drone_list = rospy.get_param('~uav_names', [])

        rospy.loginfo(f"Drone: {self.drone_list[0]}")
        rospy.loginfo("Using timer rate: %f", self.timer_main_rate)

        # Read the YAML file
        self.yaml_data = self.read_yaml_file(self.yaml_file_path)

        # Create a nav_msgs/Path from the YAML data
        self.path = self.create_path_from_yaml(self.yaml_data)
        rospy.loginfo(f"Poses: {self.path.poses[0]}")

        # Topics
        self.sub_control_manager_diag = rospy.Subscriber("/" + self.drone_list[0] + "/control_manager/diagnostics", \
                                                         ControlManagerDiagnostics, self.callbackControlManagerDiagnostics)

        # Timer
        self.timer_main = rospy.Timer(rospy.Duration(1.0), self.timerMain)

        self.uav_goto_fcu_services = {}
        # GoTo_fcu Services
        for name in self.drone_list:
            service_name = f'/{name}/control_manager/goto_fcu'
            rospy.loginfo(f"Waiting for goto service: {service_name}")
            rospy.wait_for_service(service_name)
            self.uav_goto_fcu_services[name] = rospy.ServiceProxy(service_name, Vec4)

        self.current_pose = 0
        rospy.loginfo(f"Number of poses: {len(self.path.poses)}")

        self.is_initialized = True

        self.started = False
        self.finished = False
        self.called = False

        rospy.spin()

    def timerMain(self, event=None):

        if not self.is_initialized:
            return

        rospy.loginfo_once('Main timer spinning')

        if isinstance(self.sub_control_manager_diag, ControlManagerDiagnostics):

            # if self.sub_control_manager_diag.flying_normally and not self.sub_control_manager_diag.tracker_status.have_goal \
            #     and not self.finished and not self.called:
            if not self.finished and not self.called:
                rospy.loginfo('Start')
                if self.current_pose < len(self.path.poses):
                    try:
                        x, y, z = latlon_to_utm(self.path.poses[self.current_pose].pose.position.x,self.path.poses[self.current_pose].pose.position.y)
                        rospy.loginfo(f"Current Pose: {self.current_pose}")
                        rospy.loginfo(f"Poses: {x}, {y}, {z}")
                        request = Vec4Request()
                        request.goal = [x, y, z, 0.0]
                        try:
                            rospy.loginfo(f"[{self.drone_list[0]}] Sending goal: {request.goal}")
                            response = self.uav_goto_fcu_services[self.drone_list[0]](request)
                        except rospy.ServiceException as e:
                            rospy.logerr(f"[{self.drone_list[0]}] Service call failed: {e}")
                        rospy.sleep(1.0)
                    except:
                        rospy.logerr('GoTo service not callable')
                        pass

                    if response.success:
                        rospy.loginfo('GoTo set')
                        rospy.sleep(1.0)
                        self.called = True
                        self.current_pose += 1
                    else:
                        rospy.loginfo('GoTo setting failed, message: {}'.format(response.message))

                    return
                else:
                    self.finished = True
                    return

            if self.called:
                rospy.loginfo('Verification')
                self.called = False
                self.started = True
                if self.sub_control_manager_diag.tracker_status.have_goal:
                    self.called = False
                    self.started = True

            if self.started and not self.sub_control_manager_diag.tracker_status.have_goal and self.finished:
                rospy.loginfo('Land')
                # self.land()
                self.finished = True
                self.started = False
                return

    def goto_fcu_service(self, uav, x, y, z, heading=0.0):
        request = Vec4Request()
        request.goal = [x, y, z, heading]
        try:
            rospy.loginfo(f"[{uav}] Sending goal: {request.goal}")
            response = self.uav_goto_fcu_services[uav](request)
            rospy.loginfo(f"[{uav}] Response: success={response.success}, message='{response.message}'")
        except rospy.ServiceException as e:
            rospy.logerr(f"[{uav}] Service call failed: {e}")
        return response.success

    def callbackControlManagerDiagnostics(self, msg):
        if not self.is_initialized:
            return
        rospy.loginfo_once('Getting ControlManager diagnostics')
        self.sub_control_manager_diag = msg

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
