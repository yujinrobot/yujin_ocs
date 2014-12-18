#
# License: BSD
#   https://raw.github.com/yujinrobot/yujin_ocs/license/LICENSE
#
##############################################################################
# Imports
##############################################################################

import rospy
import std_msgs.msg as std_msgs
import geometry_msgs.msg as geometry_msgs
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
            '_publishers',
            '_subscribers',
            '_parameters',
            '_spotted_markers',
            '_target_frame',
            '_target_base_t',
            '_target_base_o',
            '_child_frame_id',
            '_parent_frame_id',
            '_thread',
            '_tf_thread',
            '_tf_broadcaster',
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
        self._rotate = Rotate('~cmd_vel')
        self._publishers, self._subscribers = self._setup_ros_api()
        self._parameters = self._setup_parameters()
        self._spotted_markers = Node.SPOTTED_NONE
        self._thread = None
        self._tf_thread = None
        self._running = False
        self._rate = 0.36  # this could be parameterised
        self._listener = tf.TransformListener()
        self._tf_broadcaster = tf.TransformBroadcaster()
        self._controller_finished = False
        self._stop_requested = False
        self._dynamic_reconfigure_client = dynamic_reconfigure.client.Client(rospy.get_param('~ar_tracker', 'ar_track_alvar'))

    def _setup_parameters(self):
        parameters = {}
        parameters['search_only'] = rospy.get_param('~search_only', False)
        parameters['base_postfix'] = rospy.get_param('~base_postfix', 'base')
        parameters['base_frame']  = rospy.get_param('~base_frame', 'base_footprint')
        parameters['odom_frame']  = rospy.get_param('~odom_frame', 'odom')
        return parameters

    def _setup_ros_api(self):
        '''
          These are all public topics. Typically that will drop them into the /concert
          namespace.
        '''
        publishers = {}
        publishers['result'] = rospy.Publisher('~result', std_msgs.Bool, queue_size=5)
        publishers['initial_pose_trigger'] = rospy.Publisher('~initialise', std_msgs.Empty, queue_size=5)
        publishers['enable_approach_controller'] = rospy.Publisher('~enable_approach_controller', std_msgs.String, queue_size=5)
        publishers['disable_approach_controller'] = rospy.Publisher('~disable_approach_controller', std_msgs.Empty, queue_size=5)
        subscribers = {}
        subscribers['enable'] = rospy.Subscriber('~enable', std_msgs.String, self._ros_enable_subscriber)
        subscribers['spotted_markers'] = rospy.Subscriber('~spotted_markers', std_msgs.String, self._ros_spotted_subscriber)
        subscribers['relative_target_pose'] = rospy.Subscriber('~relative_target_pose', geometry_msgs.PoseStamped, self._ros_relative_target_pose_subscriber)

        subscribers['approach_controller_result'] = rospy.Subscriber('~approach_pose_reached', std_msgs.Bool, self._ros_controller_result_callback)
        return (publishers, subscribers)

    def _is_running(self):
        return self._running

    ##########################################################################
    # Ros Api Functions
    ##########################################################################

    def _ros_enable_subscriber(self, msg):
        if msg.data:
            rospy.loginfo('enable ar pair approach')
            if not self._is_running():
                self._running = True
                self._set_target_base_transform(msg.data)
                self._tf_thread =threading.Thread(target=self._broadcast_tfs)
                self._tf_thread.start()
                self._thread = threading.Thread(target=self.execute)
                self._thread.start()
        else:
            rospy.loginfo('disable ar pair approach')
            if self._is_running():
                self._publishers['result'].publish(std_msgs.Bool(False))
            self._stop()
            if self._thread is not None:
                self._thread.join()
                self._thread = None
            self._stop_requested = False

    def _ros_relative_target_pose_subscriber(self, msg):
        self._relative_target_pose = msg

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
        self._tf_thread.join()

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

        #base_target_frame = self._target_frame + '_relative_' + self._parameters['base_postfix']
        base_target_frame = self._target_frame + '_' + self._parameters['base_postfix']
        self._publishers['enable_approach_controller'].publish(base_target_frame)
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

    def _set_target_base_transform(self, target_frame):
        self._target_frame = target_frame
        rospy.sleep(2.0)
        try:
            # this is from odom to target frame
            (t, o) = self._listener.lookupTransform(self._parameters['odom_frame'], target_frame, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as unused_e:
            self.logwarn("couldn't get transfrom between %s and %s"%(self._parameters['odom_frame'], target_frame))
            return 

        # Transform from target_frame to base target frame
        p = (0.0, 0.36, 0.0)
        q = tf.transformations.quaternion_from_euler(1.57, -1.57, 0.0)

        self._target_base_t = (t[0], t[1], 0.0)
        self._target_base_o = tf.transformations.quaternion_multiply(o, q)

        # Publish relative target frame
        base_postfix = self._parameters['base_postfix']
        self._parent_frame_id = self._parameters['odom_frame']
        self._child_frame_id = target_frame + '_' + base_postfix

    def _broadcast_tfs(self):
        r = rospy.Rate(5)

        while self._running and not rospy.is_shutdown():
            self._publish_tf() 
            r.sleep()   


    def _publish_tf(self):
        self._tf_broadcaster.sendTransform(self._target_base_t, self._target_base_o, rospy.Time.now(), self._child_frame_id, self._parent_frame_id)

    ##########################################################################
    # Runtime
    ##########################################################################

    def _initialise_rotation(self):
        '''
          Do not call this if already running, you will cause self._rotate to become volatile.

          @return : True or false depending on if we can skip this step or not.
        '''
        direction = Rotate.CLOCKWISE
        rospy.loginfo("Markers : %s"%self._spotted_markers)
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
                rospy.logerr("AR Pair Approach : this should not happen")
                # this is from global to base footprint
                (unused_t, orientation) = self._listener.lookupTransform(self._target_frame, self._parameters['base_frame'], rospy.Time(0))
                unused_roll, unused_pitch, yaw = tf.transformations.euler_from_quaternion(orientation)
                rospy.loginfo("AR Pair Search : current yaw = %s" % str(yaw))
                direction = Rotate.COUNTER_CLOCKWISE if yaw > 0 else Rotate.CLOCKWISE
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as unused_e:
                direction = Rotate.COUNTER_CLOCKWISE
            rospy.loginfo("AR Pair Search: received an enable command, none in view.")
        self._rotate.init(yaw_absolute_rate=self._rate, yaw_direction=direction)
        return False

    def logwarn(self, msg):
        rospy.logwarn('AR Pair Approach : %s'%msg)

    def spin(self):
        '''
          Parse the set of /remocons/<name>_<uuid> connections.
        '''


        rospy.spin()
        if self._thread is not None:
            self._thread.join()
