/**
 * 
 * @brief Simple Annotation Server 
 *
 * License: BSD
 *  https://raw.githubusercontent.com/yujinrobot/yujin_ocs/license/LICENSE 
 **/

#include <yocs_simple_annotation_server/simple_annotation_server.hpp>

namespace yocs 
{

SimpleAnnotationServer::SimpleAnnotationServer(ros::NodeHandle& n) : nh_(n), priv_nh_("~")
{
}

SimpleAnnotationServer::~SimpleAnnotationServer()
{
}

bool SimpleAnnotationServer::init()
{
  if(!priv_nh_.getParam("filename", filename_)) {
    ROS_ERROR("Annotation Server : filename argument is not set");
    return false;
  }


  if(!loadAnnotationsFromFile())
  {
    ROS_ERROR("Annotation Server : Failed to parse yaml[%s]",filename_.c_str());
    return false;
  }

  initPublishers();
}

void SimpleAnnotationServer::initPublishers()
{
  pub_walls_      = nh_.advertise<yocs_msgs::WallList>        ("walls",      5, true);
  pub_tables_     = nh_.advertise<yocs_msgs::TableList>       ("tables",     5, true);
  pub_columns_    = nh_.advertise<yocs_msgs::ColumnList>      ("columns",    5, true);
  pub_ar_markers_ = nh_.advertise<ar_track_alvar::AlvarMarkers>("ar_markers", 5, true);
}

void SimpleAnnotationServer::publishAnnotations()
{
  pub_walls_.publish(walls_);
  pub_tables_.publish(tables_);
  pub_columns_.publish(columns_);
  pub_ar_markers_.publish(ar_markers_);
}

bool SimpleAnnotationServer::processSaveAnnotationService(const yocs_msgs::SaveAnnotations::Request& request, yocs_msgs::SaveAnnotations::Response& response)
{
  writeAnnotationsToFile(request);
  loadAnnotationsFromFile();
  publishAnnotations();
}

void SimpleAnnotationServer::spin()
{
  publishAnnotations();

  while(ros::ok())
  {
    ros::spinOnce();
    ros::Duration(1.0).sleep();
  }
}

}
