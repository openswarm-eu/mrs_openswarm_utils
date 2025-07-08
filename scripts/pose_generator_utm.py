#!/usr/bin/env python
import rospy
import yaml
import utm

from mrs_msgs.msg import ControlManagerDiagnostics
from mrs_msgs.msg import Reference
from mrs_msgs.srv import TrajectoryReferenceSrv,TrajectoryReferenceSrvRequest
from std_srvs.srv import Trigger,TriggerRequest

class Node:

    # #{ __init__(self)

    def __init__(self):

        rospy.init_node("sweeping_generator", anonymous=True)

        ## | --------------------- load parameters -------------------- |

        self.frame_id = rospy.get_param("~frame_id")
        self.timer_main_rate = rospy.get_param("~rate")
        self.yaml_file_path = rospy.get_param('~yaml_file', 'poses.yaml')

        rospy.loginfo("Using frame: %s", self.frame_id)
        rospy.loginfo("Using timer rate: %f", self.timer_main_rate)
        rospy.loginfo("Using YAML file: %s", self.yaml_file_path)

        # Read the YAML file
        self.yaml_data = self.read_yaml_file(self.yaml_file_path)

        rospy.loginfo('Initialized')

        ## | ----------------------- subscribers ---------------------- |

        self.sub_control_manager_diag = rospy.Subscriber("~control_manager_diag_in", ControlManagerDiagnostics, self.callbackControlManagerDiagnostics)

        ## | --------------------- service clients -------------------- |
        self.sc_trajectory = rospy.ServiceProxy('~trajectory_out', TrajectoryReferenceSrv)
        self.sc_land = rospy.ServiceProxy('~land_out', Trigger)

        ## | ------------------------- timers ------------------------- |
        #self.timer_main = rospy.Timer(rospy.Duration(1.0), self.timerMain)

        ## | -------------------- spin till the end ------------------- |
        self.is_initialized = True

        self.called = False
        self.started = False
        self.finished = False
  
        rospy.spin()

    # #} end of __init__()

    ## | ------------------------- methods ------------------------ |

    def read_yaml_file(self, file_path):
        """Reads a YAML file containing latitude, longitude, and yaw values."""
        with open(file_path, 'r') as file:
            data = yaml.safe_load(file)
        return data['poses']

    def planTrajectory(self, yaml_data):

        rospy.loginfo('Planning trajectory')

        trajectory_srv = TrajectoryReferenceSrvRequest()

        trajectory_srv.trajectory.header.frame_id = self.frame_id
        trajectory_srv.trajectory.header.stamp = rospy.Time.now()

        trajectory_srv.trajectory.fly_now = True

        trajectory_srv.trajectory.use_heading = True

        for pose in yaml_data:
            x, y = self.convert_to_utm(pose['lat'], pose['lon'])
            point = Reference()

            point.position.x = x
            point.position.y = y
            point.position.z = pose['alt']
            point.heading = pose['yaw']

            trajectory_srv.trajectory.points.append(point)

        return trajectory_srv

    def convert_to_utm(self, lat, lon):
        utm_coords = utm.from_latlon(lat, lon)
        easting, northing = utm_coords[:2]

        rospy.loginfo("Latitude: %f, Longitude: %f", lat, lon)
        rospy.loginfo("UTM Coordinates - Easting: %f, Northing: %f", easting, northing)

        return easting, northing

    def land(self):

        rospy.loginfo('Landing')
        land_srv = TriggerRequest()

        try:
            response = self.sc_land.call(land_srv)
        except:
            rospy.logerr('Land service not callable')
            pass


    ## | ------------------------ callbacks ----------------------- |

    # #{ callbackControlManagerDiagnostics():

    def callbackControlManagerDiagnostics(self, msg):

        if not self.is_initialized:
            return

        rospy.loginfo_once('Getting ControlManager diagnostics')

        self.sub_control_manager_diag = msg

    # #} end of

    ## | ------------------------- timers ------------------------- |

    # #{ timerMain()

    def timerMain(self, event=None):

        rospy.loginfo('Main timer')

        if not self.is_initialized:
            return

        rospy.loginfo_once('Main timer spinning')

        if isinstance(self.sub_control_manager_diag, ControlManagerDiagnostics):

            if not self.started and not self.finished and not self.called:
            #if self.sub_control_manager_diag.flying_normally and not self.started and not self.finished and not self.called:

                trajectory_srv = self.planTrajectory(self.yaml_data)

                try:
                    response = self.sc_trajectory.call(trajectory_srv)
                except:
                    rospy.logerr('Trajectory service not callable')
                    pass

                if response.success:
                    rospy.loginfo('Trajectory set')
                    self.called = True
                else:
                    rospy.loginfo('Trajectory setting failed, message: {}'.format(response.message))
                return

            if self.called:
                if self.sub_control_manager_diag.tracker_status.have_goal:
                    self.called = False
                    self.started = True

            if self.started and not self.sub_control_manager_diag.tracker_status.have_goal and not self.finished:
                self.land()
                self.finished = True
                return

    # #} end of timerMain()

if __name__ == '__main__':
    try:
        node = Node()
    except rospy.ROSInterruptException:
        pass