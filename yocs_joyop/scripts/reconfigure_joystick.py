#!/usr/bin/env python

import rospy
import dynamic_reconfigure.client

from yocs_msgs.cfg import JoystickConfig

##############################################################################
# ReconfigureJoystick
##############################################################################

class ReconfigureJoystick(object):

    def __init__(self):
        self.linear_scale = rospy.get_param("~linear_scale", 0.5)
        self.angular_scale = rospy.get_param("~angular_scale", 0.5 )
        self.joystick_reconfigure_namespace = rospy.get_param("~joystick_reconfigure_namespace", "yocs_joyop")
        self.client = None

        rospy.on_shutdown(self.shutdown)

        while not self.setup() and not rospy.is_shutdown():
            continue

        try:
            self.original_l_value = self.client.get_configuration()["linear_scale"]
            self.original_a_value = self.client.get_configuration()["angular_scale"]
        except dynamic_reconfigure.client.DynamicReconfigureParameterException:
            rospy.logwarn("Reconfigure Joy : failed to get original joystick configuration")

        try:
            self.client.update_configuration({"linear_scale": self.linear_scale, "angular_scale": self.angular_scale})
        except dynamic_reconfigure.client.DynamicReconfigureParameterException:
            rospy.logwarn("Reconfigure Joy : failed to update joystick configuration for bagging")

    def shutdown(self):
        try:
            self.client.update_configuration({"linear_scale":self.original_l_value, "angular_scale":self.original_a_value})
        except dynamic_reconfigure.client.DynamicReconfigureParameterException:
            rospy.logwarn("Reconfigure Joy : failed to reset joystick configuration to original")

    def setup(self):
        if not self.client:
            try:
                self.client = dynamic_reconfigure.client.Client(self.joystick_reconfigure_namespace, timeout=30)
            except rospy.ROSException:
                rospy.logwarn("Reconfigure Joy :Could not connect to dynamic reconfigure server. Try again....")
                return False
        return True

if __name__ == '__main__':
    rospy.init_node("reconfigure_joystick", log_level=rospy.INFO)
    bag_joystick = ReconfigureJoystick()
    rospy.spin()


