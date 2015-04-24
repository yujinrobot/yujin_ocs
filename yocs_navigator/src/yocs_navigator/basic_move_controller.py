'''
 basic_move_controller.py
 LICENSE : BSD - https://raw.github.com/yujinrobot/yujin_ocs/license/LICENSE
'''
import math
import copy
import rospy
import tf
import nav_msgs.msg as nav_msgs
import geometry_msgs.msg as geometry_msgs

class BasicMoveController(object):
    def __init__(self, odom_topic='odom', cmd_vel_topic='cmd_vel'):
        self._sub_odom = rospy.Subscriber(odom_topic, nav_msgs.Odometry, self.process_odometry)
        self._pub_cmd_vel = rospy.Publisher(cmd_vel_topic, geometry_msgs.Twist, queue_size=5)
        
        self._odom = None

    def process_odometry(self, msg):
        self._odom = copy.deepcopy(msg)

    def move_at(self, v, w, t):
        vel = geometry_msgs.Twist()

        vel.linear.x = v
        vel.angular.z = w
        self._pub_cmd_vel.publish(vel)
        rospy.sleep(t)

    def turn_clockwise(self):
        self.move_at(0.0, -0.5, 0.1)

    def turn_counter_clockwise(self):
        self.move_at(0.0, 0.5, 0.1)

    def turn(self, angle):
        raise NotImplementedError()

    def spin_clockwise(self):
        yaw = self._get_odom_yaw()

        i = 0
        while (self._get_odom_yaw() <= yaw) or i < 5:
            self.turn_clockwise()
            i = i + 1

        i = 0
        while (self._get_odom_yaw() > yaw) or i < 5:
            self.turn_clockwise()
            i = i +1

    def _get_odom_yaw(self):
        quaternion = (self._odom.pose.pose.orientation.x, self._odom.pose.pose.orientation.y, self._odom.pose.pose.orientation.z, self._odom.pose.pose.orientation.w)
        roll, pitch, yaw = tf.transformations.euler_from_quaternion(quaternion)
        return yaw

    def slow_forward(self):
        self.move_at(0.1, 0.0, 0.1)

    def slow_backward(self):
        self.move_at(-0.1, 0.0, 0.1)

    def forward(self, distance):
        pos0 = copy.deepcopy(self._odom.pose.pose.position)

        while(self._distance2d(pos0, self._odom.pose.pose.position) < distance):
            self.slow_forward()

    def backward(self, distance):
        pos0 = copy.deepcopy(self._odom.pose.pose.position)

        while(self._distance2d(pos0, self._odom.pose.pose.position) < distance):
            self.slow_backward()

    def _distance2d(self, p1, p2):
        '''
        computes distance between two points
        '''
        return math.sqrt(math.pow((p2.x - p1.x), 2) + math.pow((p2.y -p1.y), 2))
