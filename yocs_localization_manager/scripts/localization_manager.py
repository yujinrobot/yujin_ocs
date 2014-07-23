#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/yujinrobot/yujin_ocs/license/LICENSE
#

import rospy
import yocs_localization_manager 

if __name__ == '__main__':
    rospy.init_node('localization_manager')
    manager = yocs_localization_manager.LocalizationManager()
    manager.loginfo('Initialized')
    manager.spin()
    manager.loginfo('Bye Bye')
