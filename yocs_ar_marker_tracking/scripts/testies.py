#!/usr/bin/env python
#       
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/master/rocon_gateway/LICENSE 
#

##############################################################################
# Imports
##############################################################################

import math
import rospy
import rocon_utilities.console as console
import visualization_msgs.msg as visualization_msgs
from std_msgs.msg import Float32

# Can't turn it much - different error params?
# Sometimes jumps x-values near the edges -> due to bad z calculations on the fringe (out by 50cm!)
  # should be able to easily filter these out
# a and b disappear near the edges, but I still get readings

##############################################################################
# Functions
##############################################################################

class Marker:
    def __init__(self, id, x, y, z):
        self.id = id
        self.update(x, y, z)
    
    def update(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z
        self.distance = math.sqrt(x*x + z*z)

class Jagi(object):
    def __init__(self):
        self.baseline = 0.26
        self.left_marker = Marker(id=3, x=0, y=0, z=0)
        self.right_marker = Marker(id=0, x=0, y=0, z=0)
        self.left_last_timestamp = rospy.get_rostime()
        self.right_last_timestamp = rospy.get_rostime()
        self.x_publisher = rospy.Publisher('x', Float32)
        self.z_publisher = rospy.Publisher('z', Float32)
        self.heading_publisher = rospy.Publisher('heading', Float32)
        self.subscriber = rospy.Subscriber('/visualization_marker', visualization_msgs.Marker, self.callback)


    def print_pythags_results(self):
        b = self.baseline/2 + (self.left_marker.distance*self.left_marker.distance - self.right_marker.distance*self.right_marker.distance)/(2*self.baseline)
        try:
            a = math.sqrt(self.left_marker.distance*self.left_marker.distance - b*b)
        except ValueError:
            print("Value error: measured_distance=%s, b=%s" % (self.left_marker.distance, b))
            return False # self.left_marker not initialised fully yet, so negative sqrt
        print(console.cyan + "           a" + console.reset + " : " + console.yellow + "%s" % a + console.reset)
        print(console.cyan + "           b" + console.reset + " : " + console.yellow + "%s" % b + console.reset)
        return True

    def print_pose_results(self):
        # angle between the robot and the first marker
        alpha = math.atan2(self.left_marker.x, self.left_marker.z)
        alpha_degrees = alpha*180.0/math.pi
        # alpha + beta is angle between the robot and the second marker 
        beta = math.atan2(self.right_marker.x, self.right_marker.z)
        beta_degrees = beta*180.0/math.pi
        # theta is the angle between the wall and the perpendicular in front of the robot
        theta = math.atan2((self.left_marker.z - self.right_marker.z), (self.right_marker.x - self.left_marker.x))
        theta_degrees = theta*180.0/math.pi
    
        print(console.cyan + "       alpha" + console.reset + " : " + console.yellow + "%s" % alpha_degrees + " degrees" + console.reset)
        print(console.cyan + "        beta" + console.reset + " : " + console.yellow + "%s" % beta_degrees + " degrees" + console.reset)
        print(console.cyan + "       theta" + console.reset + " : " + console.yellow + "%s" % theta_degrees + " degrees" + console.reset)
        
        # M1 = (self.left_marker.x, self.left_marker.z)
        # M2 = (self.right_marker.x, self.right_marker.z)
        # M3 = M1 + (M2 - M1)/2   # midpoint of M1, M2
        # Target stop position relative to marker mid point (40cm perpendicularly out)
        # M4 = (-0.4*sin(theta), -0.4*cos(theta))
        # Target stop position
        # M5 = M3 + M4
        target_x = self.left_marker.x + (self.right_marker.x - self.left_marker.x)/2 - 0.4*math.sin(theta)
        target_z = self.left_marker.z + (self.right_marker.z - self.left_marker.z)/2 - 0.4*math.cos(theta)
        target_heading = math.atan2(target_x, target_z)
        target_heading_degrees = target_heading*180.0/math.pi
        
        print(console.cyan + "      target" + console.reset + " : " + console.yellow + "(x=%s, z=%s, heading=%s)" % (target_x, target_z, target_heading_degrees) + console.reset)
        self.x_publisher.publish(Float32(target_x))
        self.z_publisher.publish(Float32(target_z))
        self.heading_publisher.publish(Float32(target_heading_degrees))
    
    def callback(self, data):
        if data.id == 0:
            self.right_marker.update(data.pose.position.x, data.pose.position.y, data.pose.position.z)
            self.right_last_timestamp = data.header.stamp
            print(console.cyan + "r: [x, y, z]" + console.reset + " : " + console.yellow + "[%s, %s, %s]" % (self.right_marker.x, self.right_marker.y, self.right_marker.z) + console.reset)
            print(console.cyan + "r: d" + console.reset + "         : " + console.yellow + "%s" % self.right_marker.distance + console.reset)
        else:
            self.left_last_timestamp = data.header.stamp
            self.left_marker.update(data.pose.position.x, data.pose.position.y, data.pose.position.z)
            print(console.cyan + "l: [x, y, z]" + console.reset + " : " + console.yellow + "[%s, %s, %s]" % (self.left_marker.x, self.left_marker.y, self.left_marker.z) + console.reset)
            print(console.cyan + "l: d" + console.reset + "         : " + console.yellow + "%s" % self.left_marker.distance + console.reset)
        #if math.fabs(self.left_last_timestamp - self.right_last_timestamp) < 0.1:
        if self.left_last_timestamp == self.right_last_timestamp:
            if self.print_pythags_results():
                self.print_pose_results()

##############################################################################
# Main
##############################################################################

if __name__ == '__main__':
    #visualization_msgs/Marker
    rospy.init_node('roles_and_apps')
    jagi = Jagi()
    rospy.spin()
