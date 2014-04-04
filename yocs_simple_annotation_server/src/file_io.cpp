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

void SimpleAnnotationServer::writeAnnotationsToFile(const yocs_msgs::SaveAnnotations::Request& request)
{

}

bool SimpleAnnotationServer::loadAnnotationsFromFile()
{
  YAML::Node doc;

//  if(!loadYAMLNode(filename, doc)
//    return false;

  loadWalls(doc.FindValue("walls"));
  loadColumns(doc.FindValue("columns"));
  loadTables(doc.FindValue("tables"));
  loadARMarkers(doc.FindValue("ar_markers"));

  return true;
}

void SimpleAnnotationServer::loadWalls(const YAML::Node* node)
{
}

void SimpleAnnotationServer::loadColumns(const YAML::Node* node)
{
}

void SimpleAnnotationServer::loadTables(const YAML::Node* node)
{
}

void SimpleAnnotationServer::loadARMarkers(const YAML::Node* node)
{
}

} // yocs
