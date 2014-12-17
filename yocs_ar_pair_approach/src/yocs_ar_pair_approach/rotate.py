#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_concert/license/LICENSE
#
##############################################################################
# Imports
##############################################################################

import rospy
import math
import std_msgs.msg as std_msgs
import geometry_msgs.msg as geometry_msgs
#import tf

##############################################################################
# Classes
##############################################################################


class Rotate(object):
    '''
        implements a rotating motion.
    '''
    __slots__ = [
            'yaw_absolute_rate',
            'yaw_direction',  # +1, or -1
            '_stop',
            '_cmd_vel_topic',
            '_cmd_vel_publisher',
            '_cmd_vel_frequency',
            '_rate',
            '_running',
            '_twist',
            '_listener',
            '_initialise_pose_trigger'  # tell the initial pose manager to grab an initialisation
        ]
    CLOCKWISE = -1
    COUNTER_CLOCKWISE = 1

    def __init__(self, cmd_vel_topic):
        self._cmd_vel_publisher = None
        self._cmd_vel_frequency = 5
        self._cmd_vel_topic = cmd_vel_topic
        self._rate = rospy.Rate(self._cmd_vel_frequency)
        self._stop = False
        self._running = False
        self._initialise_pose_trigger = rospy.Publisher('~initialise', std_msgs.Empty, queue_size=5)

        twist = geometry_msgs.Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        self._twist = twist
        #self._listener = tf.TransformListener()
        self.init()

    def init(self, yaw_absolute_rate=1.2, yaw_direction=CLOCKWISE):
        '''
          Initialise the direction and rate the robot should rotate at.
        '''
        self.yaw_absolute_rate = yaw_absolute_rate
        self.yaw_direction = yaw_direction

    def shutdown(self):
        self.stop()
        while self._running:
            rospy.sleep(0.01)

    def is_running(self):
        return self._running

    def change_direction(self):
        if self.yaw_direction == Rotate.CLOCKWISE:
            self.yaw_direction = Rotate.COUNTER_CLOCKWISE
        else:
            self.yaw_direction = Rotate.CLOCKWISE

    def is_stopped(self):
        return self._stop

    def stop(self):
        self._stop = True

    def execute(self):
        '''
          Drop this into threading.Thread or QThread for execution
        '''
        if self._running:
            rospy.logerr("AR Pair Search: already executing a motion, ignoring the request")
            return
        self._cmd_vel_publisher = rospy.Publisher(self._cmd_vel_topic, geometry_msgs.Twist, queue_size=10)
        self._stop = False
        self._running = True
        start = rospy.get_rostime()
        rospy.sleep(0.3)
        result = False
        twist = self._twist
        while not self._stop and not rospy.is_shutdown():
            update = self.yaw_direction * self.yaw_absolute_rate / 10.0
            if math.fabs(twist.angular.z) < self.yaw_absolute_rate:
                twist.angular.z = twist.angular.z + update
            else:
                # Make sure it is exact so the inequality in the while loop doesn't mess up next time around
                twist.angular.z = self.yaw_direction * self.yaw_absolute_rate
            now = rospy.get_rostime()
            rospy.logdebug("AR Pair Search: rotate: %s rad/s [%ss]" % (twist.angular.z, str(now.secs - start.secs)))
            try:
                self._cmd_vel_publisher.publish(twist)
            except rospy.ROSException:  # shutdown
                break
            self._rate.sleep()
        if not rospy.is_shutdown():
            cmd = geometry_msgs.Twist()
            cmd.angular.z = 0.0
            self._cmd_vel_publisher.publish(cmd)
            self._initialise_pose_trigger.publish(std_msgs.Empty())
            result = True
        self._cmd_vel_publisher.unregister()
        self._cmd_vel_publisher = None
        self._running = False
        return result
