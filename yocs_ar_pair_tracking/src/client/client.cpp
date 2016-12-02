
#include "yocs_ar_pair_tracking/client.hpp"

namespace yocs {

ARPairTrackingClient::ARPairTrackingClient()
{
  ros::NodeHandle nh("");
  global_frame_ = std::string("map");
  global_marker_prefix_ = std::string("global_marker");
  target_frame_postfix_ = std::string("target");

  pub_update_ar_pair_ = nh.advertise<yocs_msgs::ARPairList>("update_ar_pairs", 1, true); 
  sub_global_marker_  = nh.subscribe<ar_track_alvar_msgs::AlvarMarkers>("global_markers", 1, &ARPairTrackingClient::globalMarkersCB, this);

  tf_broadcast_freq_ = 5.0;
  boost::thread(&ARPairTrackingClient::broadcastMarkersTF, this);
 
  baseline_ = 0.26;
  target_offset_ = 0.5;
}

ARPairTrackingClient::~ARPairTrackingClient()
{
}

void ARPairTrackingClient::globalMarkersCB(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg)
{
  global_markers_ = *msg;
  createMirrorMarkers();

  ROS_INFO("%lu global marker pose(s) received", global_markers_.markers.size());

  notifyARPairTracker();
}

void ARPairTrackingClient::createMirrorMarkers()
{
  for (unsigned int i = 0; i < global_markers_.markers.size(); i++)
  {
    // Compensate the vertical alignment of markers and put at ground level to adopt navistack goals format
    tf::Transform tf(tf::createQuaternionFromYaw(tf::getYaw(global_markers_.markers[i].pose.pose.orientation) - M_PI/2.0),
                     tf::Vector3(global_markers_.markers[i].pose.pose.position.x, global_markers_.markers[i].pose.pose.position.y, 0.0));
    tf::StampedTransform marker_gb(tf, ros::Time::now(), global_frame_, "KKKK");

    // Half turn and translate to put goal at some distance in front of the marker
    tf::Transform in_front(tf::createQuaternionFromYaw(M_PI),
    tf::Vector3(1.0, 0.0, 0.0));
    marker_gb *= in_front;

    ar_track_alvar_msgs::AlvarMarker kk; 
    mtk::tf2pose(marker_gb, kk.pose);
    global_markers_mirrors_.markers.push_back(kk);

    ROS_DEBUG("Marker %d: %s", global_markers_.markers[i].id, mtk::pose2str2D(global_markers_.markers[i].pose.pose).c_str());

    // TODO: In the current annotation design, it is impossible to record both left and right id of ar marker. 
  }
}

void ARPairTrackingClient::notifyARPairTracker()
{
  unsigned int i;
  yocs_msgs::ARPairList l;

  for(i = 0; i < global_markers_.markers.size(); i ++)
  {
    ar_track_alvar_msgs::AlvarMarker m = global_markers_.markers[i];
    yocs_msgs::ARPair p;
    char frame[32];
    sprintf(frame, "%s_%d_%s",global_marker_prefix_.c_str(), m.id, target_frame_postfix_.c_str());
    
    p.left_id = m.id;
    p.right_id = m.id - 3;
    p.baseline = baseline_;
    p.target_offset = target_offset_;
    p.target_frame = frame; 

    l.pairs.push_back(p);
  }
  
  pub_update_ar_pair_.publish(l);
}


void ARPairTrackingClient::broadcastMarkersTF()
{
  ros::Rate rate(tf_broadcast_freq_);

  // TODO semantic map is not broadcasting this already?  only docking base no
  while (ros::ok())
  {
    publishMarkerTFs(global_marker_prefix_,global_markers_);
    publishTargetTFs(global_marker_prefix_,global_markers_);
    publishMarkerTFs("mirror",global_markers_mirrors_);
    rate.sleep();
  }

}

void ARPairTrackingClient::publishTargetTFs(const std::string prefix, const ar_track_alvar_msgs::AlvarMarkers& markers)
{
  char parent_frame[32];
  char child_frame[32];
  tf::StampedTransform tf;
  tf.stamp_ = ros::Time::now();

  for (unsigned int i = 0; i <markers.markers.size(); i++)
  {
    sprintf(parent_frame, "%s_%d", prefix.c_str(), markers.markers[i].id);
    sprintf(child_frame, "%s_%s", parent_frame, target_frame_postfix_.c_str());

    tf::Transform tf(tf::createQuaternionFromRPY(M_PI,0,0),
                     tf::Vector3(0, 0, target_offset_));
    tf::StampedTransform target(tf, ros::Time::now(), parent_frame, child_frame);

    tf_brcaster_.sendTransform(target);
  }
}

void ARPairTrackingClient::publishMarkerTFs(const std::string prefix, const ar_track_alvar_msgs::AlvarMarkers& markers)
{
  char child_frame[32];
  tf::StampedTransform tf;
  tf.stamp_ = ros::Time::now();

  for (unsigned int i = 0; i <markers.markers.size(); i++)
  {
    sprintf(child_frame, "%s_%d", prefix.c_str(), markers.markers[i].id);
    mtk::pose2tf(markers.markers[i].pose, tf);
    tf.child_frame_id_ = child_frame;
    tf.stamp_ = ros::Time::now();
    tf_brcaster_.sendTransform(tf);
  }
}



}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ar_pair_client");
  yocs::ARPairTrackingClient client;

  ros::spin();
}
