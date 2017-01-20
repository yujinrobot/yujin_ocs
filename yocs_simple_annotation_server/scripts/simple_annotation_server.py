#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/yujinrobot/yujin_ocs/license/LICENSE
#

import rospy 
import yocs_simple_annotation_server

if __name__ == '__main__':

    rospy.init_node('simple_annotation_server')

    server = yocs_simple_annotation_server.SimpleAnnotationServer() 

    if not server.init():
        server.logerr("Failed to initialize")
    else:
        server.loginfo("Initialized")
        server.spin()
        server.loginfo("Bye Bye")
