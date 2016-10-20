#!/usr/bin/env python

import rospy
import dynamic_reconfigure.client

from yocs_msgs.cfg import JoystickConfig

##############################################################################
# ReconfigureJoystick
##############################################################################

class ReconfigureJoystick(object):
  
    def shutdown(self):
      try:
	self.client.update_configuration({"linear_scale":self.original_l_value, "angular_scale":self.original_a_value})
      except self.client.DynamicReconfigureParameterException:
	rospy.logwarn("Reconfiture Joy : failed to reset joystick configuration to original")
	
    def setup(self):
      if not self.client:
	try:
	  self.client = dynamic_reconfigure.client.Client("yocs_joyop", timeout=30)
	except rospy.ROSException:
	  rospy.logwarn("Reconfiture Joy :Could not connect to dynamic reconfigure server. Try again....")
	  return False
      return True
	
    def __init__(self):
      _l_scale = rospy.get_param("~linear_scale")
      _a_scale = rospy.get_param("~angular_scale")
      
      _timeout = False
      self.client = None
      rospy.on_shutdown(self.shutdown)
      
      while not self.setup() and not rospy.is_shutdown():
	  None
	  
      try:
	self.original_l_value = self.client.get_configuration()["linear_scale"]
	self.original_a_value = self.client.get_configuration()["angular_scale"]
      except self.client.DynamicReconfigureParameterException:
	rospy.logwarn("Reconfiture Joy : failed to get original joystick configuration")
      
      try:
	self.client.update_configuration({"linear_scale":_l_scale, "angular_scale":_a_scale})
      except self.client.DynamicReconfigureParameterException:
	rospy.logwarn("Reconfiture Joy : failed to update joystick configuration for bagging")
	  
if __name__ == '__main__':
    rospy.init_node("reconfigure_joystick", log_level=rospy.INFO)
    bag_joystick = ReconfigureJoystick()
    rospy.spin()
    
    