/**
 * 
 * @brief Simple Annotation Server 
 *
 * License: BSD
 *  https://raw.githubusercontent.com/yujinrobot/yujin_ocs/license/LICENSE 
 **/

#include <ros/ros.h>
#include <yocs_simple_annotation_server/simple_annotation_server.hpp>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "simple_annotation_server");
  ros::NodeHandle priv_n("~");
  ros::NodeHandle n;

  yocs::SimpleAnnotationServer sas(n);
  
  if(!sas.init())
    return -1;

  ROS_INFO("Annotation Server : Initialized");
  sas.spin();
  ROS_INFO("Annotation Server : Bye Bye");

  return 0;
}

