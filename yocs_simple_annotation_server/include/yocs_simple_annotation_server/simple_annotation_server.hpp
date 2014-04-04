/**
 * 
 * @brief Simple Annotation Server 
 *
 * License: BSD
 *  https://raw.githubusercontent.com/yujinrobot/yujin_ocs/license/LICENSE 
 **/

#ifndef YUJIN_OCS_SIMPLE_ANNOTATION_SERVER_HPP_
#define YUJIN_OCS_SIMPLE_ANNOTATION_SERVER_HPP_

/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <ros/ros.h>
#include <yocs_msgs/SaveAnnotations.h>
#include <yocs_msgs/WallList.h>
#include <yocs_msgs/ColumnList.h>
#include <yocs_msgs/TableList.h>
#include <ar_track_alvar/AlvarMarkers.h>

/*****************************************************************************
** Class 
*****************************************************************************/

/*
   Simply it parses annotations in yaml file and provides latch topic
   It updates the file and annotation topic when it receives save annotation call
 */

namespace yocs {
class SimpleAnnotationServer {
  public:
    SimpleAnnotationServer(ros::NodeHandle& n);
    ~SimpleAnnotationServer();

    bool init();
    void setAnnotationFile(const std::string filename);
    
    void spin();
  protected:
    bool processSaveAnnotationService(const yocs_msgs::SaveAnnotations::Request& request, yocs_msgs::SaveAnnotations::Response& response);
    void publishAnnotations();

    void writeAnnotationsToFile(const yocs_msgs::SaveAnnotations::Request& request);
    bool loadAnnotationsFromFile();
  private:
    void initPublishers();

    ros::NodeHandle nh_;
    ros::NodeHandle priv_nh_;
    std::string filename_;

    ros::Publisher pub_walls_;
    ros::Publisher pub_columns_;
    ros::Publisher pub_tables_;
    ros::Publisher pub_ar_markers_;

    yocs_msgs::WallList walls_;
    yocs_msgs::ColumnList columns_;
    yocs_msgs::TableList tables_;
    ar_track_alvar::AlvarMarkers ar_markers_;
}; // SimpleAnnotationServer
} // yocs

#endif // YUJIN_OCS_SIMPLE_ANNOTATION_SERVER_HPP_ 
