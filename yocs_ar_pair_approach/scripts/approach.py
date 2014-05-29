#!/usr/bin/env python
#       
# License: BSD
#   https://raw.github.com/yujinrobot/yujin_ocs/license/LICENSE
#
import rospy
import yocs_ar_pair_approach

if __name__ == '__main__':

    rospy.init_node('ar_pair_approach')
    approach_behaviour = yocs_ar_pair_approach.Node()
    approach_behaviour.spin()
