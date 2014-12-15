#
# License: BSD
#   https://raw.github.com/yujinrobot/yujin_ocs/license/LICENSE
#

import threading
import actionlib
import dynamic_reconfigure.client
import rospy
import tf
from yocs_navigator import BasicMoveController
import std_msgs.msg as std_msgs
import yocs_msgs.msg as yocs_msgs
import geometry_msgs.msg as geometry_msgs

class LocalizationManager(object):
    """
        Uses poses from a pose tracker to initialise the global robot pose
        using a pose initialisation node such as fake_localization
    """

    _localize_action_name = 'localize'

    def __init__(self):
        self._initialise = False
        self._thread =  None
        self._basic_move_controller = BasicMoveController()
        self._init_params()

        if self.param['simulation']:
            self.loginfo("Running in simulation mode.")
        else:
            self._client = dynamic_reconfigure.client.Client(rospy.get_param('~pose_tracker', 'ar_track_alvar'))
            self._sub_tracked_poses = rospy.Subscriber('pose_tracker/poses', geometry_msgs.PoseWithCovarianceStamped, self._tracked_poses_callback)

        self._pub_init_pose = rospy.Publisher('initialpose', geometry_msgs.PoseWithCovarianceStamped, latch=True, queue_size=3)

        self._as_localize = actionlib.SimpleActionServer(self._localize_action_name, yocs_msgs.LocalizeAction, auto_start=False)
        self._as_localize.register_goal_callback(self._process_localize_goal)
        self._as_localize.register_preempt_callback(self._process_localize_preempt)

    def _init_params(self):
        param = {}
        param['sleeptime'] = rospy.Duration(1 / rospy.get_param('~spin_freq', 10))
        param['simulation'] = rospy.get_param('~simulation', False)
        param['ar_pair_baseline'] = rospy.get_param('ar_pair/baseline', 0.28)
        param['ar_pair_target_offset'] = rospy.get_param('ar_pair/target_offset', 0.5)
        param['timeout'] = rospy.get_param('~timeout', 10.0)

        # configurations for simulation
        param['sim_global_frame'] = rospy.get_param('~simulation_global_frame','map')
        param['sim_x'] = rospy.get_param('~simulation_x', 0.0)
        param['sim_y'] = rospy.get_param('~simulation_y', 0.0)
        param['sim_a'] = rospy.get_param('~simulation_a', 0.0)

        self.param = param

    def _tracked_poses_callback(self, msg):
        if self._initialise:
            # send pose to pose initialisation node
            msg.header.stamp -= rospy.Duration(0.2) # TODO: get latest common time
            cov = list(msg.pose.covariance)
            cov[6 * 0 + 0] = self._distortion * self._distortion
            cov[6 * 1 + 1] = self._distortion * self._distortion
            msg.pose.covariance = tuple(cov)
            self._pub_init_pose.publish(msg)
            self.loginfo("localization done.")
            self._initialise = False

    def _process_localize_goal(self):
        goal = self._as_localize.accept_new_goal()
        self._distortion = goal.distortion
        self.loginfo("Received Localize goal %s"%str(goal))

        if self._initialise:
            message = 'robot is initialising already. Ignore the command'
            self._send_result(False, message)
        else:
            if self.param['simulation']:
                pose_msg = geometry_msgs.PoseWithCovarianceStamped()
                pose_msg.header.frame_id = self.param['sim_global_frame']
                pose_msg.header.stamp = rospy.Time.now() - rospy.Duration(0.2) # TODO: get latest common time
                pose_msg.pose.pose.position.x = self.param['sim_x']
                pose_msg.pose.pose.position.y = self.param['sim_y']
                pose_msg.pose.pose.position.z = 0.0
                quat = tf.transformations.quaternion_from_euler(0, 0, self.param['sim_a'])
                pose_msg.pose.pose.orientation = geometry_msgs.Quaternion(*quat)
                self._pub_init_pose.publish(pose_msg)
                # send success right away
                self._send_result(True,'Initialisation done in simulation.')
            elif goal.command == goal.STAND_AND_LOCALIZE:
                self._thread = threading.Thread(target=self._stand_and_localize)
                self._thread.start()
            elif goal.command == goal.SPIN_AND_LOCALIZE:
                self._thread = threading.Thread(target=self._spin_and_localize)
                self._thread.start()
            else:
                message = 'Invalid command %s'%str(goal.command)
                self._send_result(False, message)

    def _process_localize_preempt(self):
        self.logwarn('Received Preempt Request')
        pass

    def _stand_and_localize(self):
        self.loginfo("Stand and Localization started.")

        # enable pose tracker
        self._update_tracker(True)
        self._initialise = True
        
        timeout = self.param['timeout']
        sleeptime = self.param['sleeptime']
        start_time = rospy.Time.now()
        while not rospy.is_shutdown() and self._initialise:
            current_time = rospy.Time.now()
            dif = (current_time - start_time).to_sec()
            if dif > timeout: 
                break
            rospy.sleep(sleeptime)

        # disable the pose tracker
        self._update_tracker(False)

           
        if self._initialise:  # Timeout
            self._initialise = False
            self._send_result(False, "couldn't localize in time %s"%str(dif))
        else:  # localized
            self._send_result(True, "Localized")

    def _spin_and_localize(self):
        self.loginfo("Spin and localization started")

        self._update_tracker(True)
        self._initialise = True

        self._basic_move_controller.spin_clockwise()

        self._update_tracker(False)
        if self._initialise:
            self._initialise = False
            self._send_result(False, "couldn't localise after full spining")
        else:
            self._send_result(True, "Localised")
                
    def _send_result(self, success, message):
        if success:
            self.loginfo(str(message))
        else:
            self.logwarn(str(message))
        r = yocs_msgs.LocalizeResult()
        r.success = success
        r.message = message
        self._as_localize.set_succeeded(r)

    def _update_tracker(self, enabled):
        params = { 'enabled' : enabled}
        config = self._client.update_configuration(params)

    def loginfo(self, msg):
        rospy.loginfo('Localization Manager : ' + str(msg))

    def logwarn(self, msg):
        rospy.logwarn('Localization Manager : ' + str(msg))

    def spin(self):
        sleeptime = self.param['sleeptime']
        self._as_localize.start()
        while not rospy.is_shutdown():
            rospy.sleep(sleeptime)
        if self._thread:
            self._thread.join()
