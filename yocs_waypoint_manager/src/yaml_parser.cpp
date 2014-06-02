
#include "yocs_waypoint_manager/yaml_parser.hpp"

namespace yocs {
  bool loadWaypointListFromYaml(const std::string& filename,yocs_msgs::WaypointList& wps) {

    wps.waypoints.clear();

    // Yaml File Parsing
    try
    {
      YAML::Node doc;

      getYamlNode(filename, doc);
      parseWaypoints(doc, wps);
    }
    catch(YAML::ParserException& e)
    {
      ROS_ERROR("Parse waypoints file failed: %s", e.what());
      return false;
    }
    catch(YAML::RepresentationException& e)
    {
      ROS_ERROR("Parse waypoints file failed: %s", e.what());
      return false;
    }
    catch(std::string& e) {
      ROS_ERROR("Parse waypoints file failed: %s",e.c_str());
      return false;
    }
    return true;
  }

  void getYamlNode(const std::string& filename, YAML::Node& node) 
  {
    std::ifstream ifs(filename.c_str(), std::ifstream::in);
    if (ifs.good() == false)
    {                                                            
      throw std::string("Waypoints file not found");
    }

    #ifdef HAVE_NEW_YAMLCPP
      node = YAML::Load(ifs);
    #else
      YAML::Parser parser(ifs);
      parser.GetNextDocument(node);
    #endif
  }

  void parseWaypoints(const YAML::Node& node, yocs_msgs::WaypointList& wps) 
  {
    
    unsigned int i;

    #ifdef HAVE_NEW_YAMLCPP
    const YAML::Node& wp_node_tmp = node["waypoints"];   
    const YAML::Node* wp_node = wp_node_tmp ? &wp_node_tmp : NULL;   
    #else
    const YAML::Node* wp_node = node.FindValue("waypoints");   
    #endif
    if(wp_node != NULL)
    {
      for(i = 0; i < wp_node->size(); i++) 
      {
        // Parse waypoint entries on YAML
        yocs_msgs::Waypoint wp;

        (*wp_node)[i]["name"] >> wp.name;
        (*wp_node)[i]["frame_id"] >> wp.header.frame_id;
        (*wp_node)[i]["pose"]["position"]["x"] >> wp.pose.position.x;
        (*wp_node)[i]["pose"]["position"]["y"] >> wp.pose.position.y;
        (*wp_node)[i]["pose"]["position"]["z"] >> wp.pose.position.z;
        (*wp_node)[i]["pose"]["orientation"]["x"] >> wp.pose.orientation.x;
        (*wp_node)[i]["pose"]["orientation"]["y"] >> wp.pose.orientation.y;
        (*wp_node)[i]["pose"]["orientation"]["z"] >> wp.pose.orientation.z;
        (*wp_node)[i]["pose"]["orientation"]["w"] >> wp.pose.orientation.w;

        wps.waypoints.push_back(wp);
      }
    }
  }
}
