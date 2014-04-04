/**
 * 
 * @brief Simple Annotation Server 
 *
 * License: BSD
 *  https://raw.githubusercontent.com/yujinrobot/yujin_ocs/license/LICENSE 
 **/

#include <yocs_simple_annotation_server/simple_annotation_server.hpp>

namespace yocs {

void SimpleAnnotationServer::setAnnotationFile(const std::string filename)
{
  filename_ = filename;
}

bool SimpleAnnotationServer::loadAnnotationsFromFile()
{
  return false;
}

void SimpleAnnotationServer::writeAnnotationsToFile(const yocs_msgs::SaveAnnotations::Request& request)
{

}

} // yocs
