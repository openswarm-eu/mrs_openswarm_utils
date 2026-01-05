#!/usr/bin/env python3

import rospy

from mrs_msgs.msg import ControlManagerDiagnostics
from mrs_msgs.srv import String, StringRequest


class EstimatorSwitcher(object):

    def __init__(self):
        rospy.init_node('estimator_switcher', anonymous=False)

        self.drone_flying = False
        self.estimator_changed = False

        # Load parameters
        self.estimator_nav = rospy.get_param("~estimator_nav", "liosam")

        # Subscriber
        self.diagnostics_sub = rospy.Subscriber(
            "control_manager/diagnostics",
            ControlManagerDiagnostics,
            self.drone_flying_callback,
            queue_size=10
        )

        # Service
        rospy.loginfo("Waiting for estimation_manager/change_estimator service...")
        rospy.wait_for_service("estimation_manager/change_estimator")
        self.change_estimator_srv = rospy.ServiceProxy(
            "estimation_manager/change_estimator",
            String
        )

        rospy.loginfo("EstimatorSwitcher initialized.")

    def drone_flying_callback(self, msg):
        self.drone_flying = msg.flying_normally

        # Handle estimator change outside of logic pollution
        if self.drone_flying and not self.estimator_changed:
            self.change_estimator()

    def change_estimator(self):
        try:
            req = StringRequest()
            req.value = self.estimator_nav
            rospy.loginfo("Requesting estimator change to: %s", self.estimator_nav)

            response = self.change_estimator_srv(req)

            if response.success:
                rospy.loginfo("State estimator successfully changed.")
                self.estimator_changed = True
            else:
                rospy.logwarn("Estimator change failed.")

        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", str(e))


if __name__ == "__main__":
    try:
        EstimatorSwitcher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
