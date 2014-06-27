#
# License: BSD
#   https://raw.github.com/yujinrobot/yujin_ocs/license/LICENSE
#
##############################################################################
# Imports
##############################################################################

import rospy
import std_msgs.msg as std_msgs
import threading
import tf
import dynamic_reconfigure.client


# Local imports
from .rotate import Rotate


class Node(object):
    '''
      Manages connectivity information provided by services and provides this
      for human interactive agent (aka remocon) connections.
    '''
    __slots__ = [
#            'role_and_app_table',  # Dictionary of string : concert_msgs.RemoconApp[]
            '_publishers',
            '_subscribers',
            '_parameters',
            '_spotted_markers',
            '_thread',
            '_rotate',
            '_rate',
            '_listener',
            '_running',
            '_controller_finished',
            '_stop_requested',
            '_dynamic_reconfigure_client'
        ]
    SPOTTED_NONE = 'none'
    SPOTTED_LEFT = 'left'
    SPOTTED_RIGHT = 'right'
    SPOTTED_BOTH = 'both'

    ##########################################################################
    # Initialisation
    ##########################################################################

    def __init__(self):
        self._publishers, self._subscribers = self._setup_ros_api()
        self._parameters = self._setup_parameters()
        self._spotted_markers = Node.SPOTTED_NONE
        self._rotate = Rotate('~cmd_vel')
        self._thread = None
        self._running = False
        self._rate = 0.36  # this could be parameterised
        self._listener = tf.TransformListener()
        self._controller_finished = False
        self._stop_requested = False
        self._dynamic_reconfigure_client = dynamic_reconfigure.client.Client(rospy.get_param('~ar_tracker', 'ar_track_alvar'))

    def _setup_parameters(self):
        parameters = {}
        parameters['search_only'] = rospy.get_param('~search_only', False)
        return parameters

    def _setup_ros_api(self):
        '''
          These are all public topics. Typically that will drop them into the /concert
          namespace.
        '''
        publishers = {}
        publishers['result'] = rospy.Publisher('~result', std_msgs.Bool, queue_size=100)
        publishers['initial_pose_trigger'] = rospy.Publisher('~initialise', std_msgs.Empty, queue_size=100)
        publishers['enable_approach_controller'] = rospy.Publisher('~enable_approach_controller', std_msgs.Empty, queue_size=100)
        publishers['disable_approach_controller'] = rospy.Publisher('~disable_approach_controller', std_msgs.Empty, queue_size=100)
        subscribers = {}
        subscribers['enable'] = rospy.Subscriber('~enable', std_msgs.Bool, self._ros_enable_subscriber)
        subscribers['spotted_markers'] = rospy.Subscriber('~spotted_markers', std_msgs.String, self._ros_spotted_subscriber)
        subscribers['approach_controller_result'] = rospy.Subscriber('~approach_pose_reached', std_msgs.Bool, self._ros_controller_result_callback)
        return (publishers, subscribers)

    def _is_running(self):
        return self._running

    ##########################################################################
    # Ros Api Functions
    ##########################################################################

    def _ros_enable_subscriber(self, msg):
        if msg.data:
            if not self._is_running():
                self._running = True
                self._thread = threading.Thread(target=self.execute)
                self._thread.start()
        else:
            if self._is_running():
                self._publishers['result'].publish(std_msgs.Bool(False))
            self._stop()
            if self._thread is not None:
                self._thread.join()
                self._thread = None
            self._stop_requested = False

    def _ros_spotted_subscriber(self, msg):
        self._spotted_markers = msg.data
        if self._spotted_markers == Node.SPOTTED_BOTH and self._rotate.is_running():
            self._rotate.stop()

    def _ros_controller_result_callback(self, msg):
        rospy.loginfo("AR Pair Approach : received result from the approach controller.")
        if msg.data:
            rospy.loginfo("AR Pair Approach : Controller reached the goal.")
            self._controller_finished = True
        else:
            rospy.loginfo("AR Pair Approach : Controller failed to reach the goal.")
            # TODO: handle this situation

    ##########################################################################
    # Execute
    ##########################################################################

    def _stop(self):
        if not self._rotate.is_stopped():
            self._rotate.stop()
        self._stop_requested = True

    def _post_execute(self, result):
        self._stop_requested = False
        if not rospy.is_shutdown():
            self._dynamic_reconfigure_client.update_configuration({'enabled': 'False'})
            self._publishers['result'].publish(std_msgs.Bool(result))
        self._running = False

    def execute(self):
        self._dynamic_reconfigure_client.update_configuration({'enabled': 'True'})
        found_markers = self._initialise_rotation()
        if not found_markers:
            result = self._rotate.execute()
            if not result or self._stop_requested:
                self._post_execute(False)
                return
        rospy.loginfo("AR Pair Approach : found both ar pair markers.")
        if self._parameters['search_only']:
            rospy.loginfo("AR Pair Approach : aborting initialisation and approach as requested.")
            return
        rospy.loginfo("AR Pair Approach : setting an initial pose from the global ar pair reference.")
        self._publishers['initial_pose_trigger'].publish(std_msgs.Empty())
        rospy.loginfo("AR Pair Approach : enabling the approach controller")
        self._publishers['enable_approach_controller'].publish(std_msgs.Empty())
        while not rospy.is_shutdown() and not self._stop_requested:
            if self._controller_finished:
                self._controller_finished = False
                break
            rospy.sleep(0.1)
        rospy.loginfo("AR Pair Approach : disabling the approach controller")
        self._publishers['disable_approach_controller'].publish(std_msgs.Empty())
        if rospy.is_shutdown() or self._stop_requested:
            self._post_execute(False)
        else:
            self._post_execute(True)

    ##########################################################################
    # Runtime
    ##########################################################################

    def _initialise_rotation(self):
        '''
          Do not call this if already running, you will cause self._rotate to become volatile.

          @return : True or false depending on if we can skip this step or not.
        '''
        direction = Rotate.CLOCKWISE
        if self._spotted_markers == Node.SPOTTED_BOTH:
            rospy.loginfo("AR Pair Approach : received an enable command, both spotted markers already in view!")
            return True
        elif self._spotted_markers == Node.SPOTTED_LEFT:
            rospy.loginfo("AR Pair Approach : received an enable command, only left in view.")
        elif self._spotted_markers == Node.SPOTTED_RIGHT:
            rospy.loginfo("AR Pair Approach : received an enable command, only right in view.")
            direction = Rotate.COUNTER_CLOCKWISE
        else:  # self._spotted_markers == Node.SPOTTED_NONE
            try:
                # this is from global to base footprint
                (unused_t, orientation) = self._listener.lookupTransform('ar_global', 'base_footprint', rospy.Time(0))
                unused_roll, unused_pitch, yaw = tf.transformations.euler_from_quaternion(orientation)
                rospy.loginfo("AR Pair Search : current yaw = %s" % str(yaw))
                direction = Rotate.COUNTER_CLOCKWISE if yaw > 0 else Rotate.CLOCKWISE
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as unused_e:
                direction = Rotate.COUNTER_CLOCKWISE
            rospy.loginfo("AR Pair Search: received an enable command, none in view.")
        self._rotate.init(yaw_absolute_rate=self._rate, yaw_direction=direction)
        return False

    def spin(self):
        '''
          Parse the set of /remocons/<name>_<uuid> connections.
        '''
        rospy.spin()
        if self._thread is not None:
            self._thread.join()
