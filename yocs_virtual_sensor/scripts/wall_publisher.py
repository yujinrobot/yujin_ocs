#!/usr/bin/env python

import rospy
import yaml
import tf
import copy

from geometry_msgs.msg import *
from rospy_message_converter import message_converter
from visualization_msgs.msg import *
from yocs_msgs.msg import *

def publish(filename):
    yaml_data = None 
    with open(filename) as f:
       yaml_data = yaml.load(f)

    wall_list = WallList()
    # Markers
    marker_list = MarkerArray()    

    marker_id = 1
    for t in yaml_data:
        object = Wall()
        object.name = t['name']
        object.length = float(t['length'])
        object.width  = float(t['width'])
        object.height = float(t['height'])
        object.pose.header.frame_id = t['frame_id']
        object.pose.header.stamp = rospy.Time.now()
        object.pose.pose.pose = message_converter.convert_dictionary_to_ros_message('geometry_msgs/Pose',t['pose'])
        wall_list.obstacles.append(object)

        marker = Marker()
        marker.id = marker_id
        marker.header = object.pose.header
        marker.type = Marker.CUBE
        marker.ns = "wall_obstacles"
        marker.action = Marker.ADD
        marker.lifetime = rospy.Duration.from_sec(0)
        marker.pose = copy.deepcopy(object.pose.pose.pose)
        marker.pose.position.z += object.height/2.0
        marker.scale.x = object.width
        marker.scale.y = object.length
        marker.scale.z = object.height
        marker.color.r = 0.2
        marker.color.g = 0.4
        marker.color.b = 0.4
        marker.color.a = 0.5
                                                                                                                                                    
        marker_list.markers.append(marker)
                                                                                                                                                    
        marker_id = marker_id + 1

    wall_pub.publish(wall_list)
    marker_pub.publish(marker_list)
    
    return

if __name__ == '__main__':
    rospy.init_node('wall_loader')
    filename = rospy.get_param('~filename')
    
    marker_pub = rospy.Publisher('wall_marker',    MarkerArray, latch=True, queue_size=1)
    wall_pub   = rospy.Publisher('wall_pose_list', WallList,    latch = True)

    rospy.loginfo('Publishing obstacles and visualization markers.')
    publish(filename)
    rospy.spin()
