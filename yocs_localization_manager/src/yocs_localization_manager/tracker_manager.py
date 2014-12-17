#
# License: BSD
#   https://raw.github.com/yujinrobot/yujin_ocs/license/LICENSE
#

import rospy
import tf
import ar_track_alvar_msgs.msg as ar_track_alvar_msgs
import yocs_msgs.msg as yocs_msgs
import copy
import math

def remove_leading_slash(frame_id):
    if frame_id.startswith('/'):
        return frame_id[1:]
    else:
        return frame_id

class TrackerManager(object):
    """
        Receives AR Pair Annotations from annotation server.
        Configure yocs_ar_pair_tracking and publish TFs
    """
    def __init__(self):
        self._init_params()
        self._init_variables()

        self._sub_global_pairs = rospy.Subscriber('global_pairs', ar_track_alvar_msgs.AlvarMarkers, self._process_global_pairs)
        self._pub_pair_list = rospy.Publisher('update_ar_pairs', yocs_msgs.ARPairList, latch=True, queue_size=1)
        

    def _init_params(self):
        param = {}
        param['tf_broadcast_freq'] = rospy.get_param('tf_broadcast_freq', 0.5)
        param['ar_pair_baseline'] = rospy.get_param('ar_pair/baseline', 0.28)
        param['ar_pair_target_offset'] = rospy.get_param('ar_pair/target_offset', 0.5)
        param['ar_pair_global_prefix'] = rospy.get_param('ar_pair/global_prefix', 'global_marker')
        param['ar_pair_target_postfix'] = rospy.get_param('ar_pair/global_postfix', 'target')

        self.loginfo("Target offset : %s"%param['ar_pair_target_offset'])

        self.param = param

    def _init_variables(self):
        self._tf_broadcaster = tf.TransformBroadcaster()
        self._global_pairs = []

    def _process_global_pairs(self, msg):
        # Register Global Marker
        self._global_pairs = copy.deepcopy(msg.markers)
        self.loginfo('Received ' + str(len(self._global_pairs)) + ' markers')

        # Notify AR Pair Tracker
        self._notify_ar_pair_tracker()

    def _notify_ar_pair_tracker(self):
        pair_list = yocs_msgs.ARPairList()

        baseline = self.param['ar_pair_baseline']
        target_offset = self.param['ar_pair_target_offset']
        global_prefix = self.param['ar_pair_global_prefix']
        target_postfix = self.param['ar_pair_target_postfix']

        for marker in self._global_pairs:
            p = yocs_msgs.ARPair()
            p.left_id = marker.id
            p.right_id = p.left_id - 3
            p.baseline = baseline
            p.target_offset = target_offset
            p.target_frame = global_prefix + '_' + str(marker.id) + '_' + target_postfix
            pair_list.pairs.append(p)

        self._pub_pair_list.publish(pair_list)
        


    def _publish_target_tfs(self):
        global_prefix = self.param['ar_pair_global_prefix']
        target_postfix = self.param['ar_pair_target_postfix']
        target_offset  = self.param['ar_pair_target_offset']

        for marker in self._global_pairs:
            parent_frame_id = global_prefix + '_' + str(marker.id)
            child_frame_id = parent_frame_id + '_' + target_postfix
            parent_frame_id = remove_leading_slash(parent_frame_id)
            child_frame_id = remove_leading_slash(child_frame_id)

            p = (0,0, target_offset)
            q = tf.transformations.quaternion_from_euler(math.pi, 0, 0)
            self._tf_broadcaster.sendTransform(p, q, rospy.Time.now(), child_frame_id , parent_frame_id)
            rospy.sleep(0.05)

    def _publish_marker_tfs(self):
        global_prefix = self.param['ar_pair_global_prefix']
        for marker in self._global_pairs:
            parent_frame_id = marker.pose.header.frame_id
            child_frame_id = global_prefix + '_' + str(marker.id)
            parent_frame_id = remove_leading_slash(parent_frame_id)
            child_frame_id = remove_leading_slash(child_frame_id)
            p = (marker.pose.pose.position.x,marker.pose.pose.position.y, marker.pose.pose.position.z)
            q = (marker.pose.pose.orientation.x, marker.pose.pose.orientation.y, marker.pose.pose.orientation.z,marker.pose.pose.orientation.w)
            self._tf_broadcaster.sendTransform(p, q, rospy.Time.now(), child_frame_id ,parent_frame_id)
            rospy.sleep(0.05)

    def loginfo(self, msg):
        rospy.loginfo('TrackerManager : ' + str(msg))

    def spin(self):
        if self.param['tf_broadcast_freq'] == 0:
            rospy.spin()
        else:
            hertz = 1 / self.param['tf_broadcast_freq']
            r = rospy.Rate(hertz)
            while not rospy.is_shutdown():
                self._publish_marker_tfs()
                r.sleep()
                self._publish_target_tfs()
                r.sleep()
