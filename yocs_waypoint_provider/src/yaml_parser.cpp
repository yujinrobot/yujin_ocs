#include <yocs_msgs/Trajectory.h>
#include <yocs_msgs/Waypoint.h>
#include "yocs_waypoint_provider/yaml_parser.hpp"

namespace yocs
{
  bool loadWaypointsAndTrajectoriesFromYaml(const std::string& filename,
                                            yocs_msgs::WaypointList& wps,
                                            yocs_msgs::TrajectoryList& trajs)
  {
    wps.waypoints.clear();
    trajs.trajectories.clear();

    // Yaml File Parsing
    try
    {
      YAML::Node doc;

      getYamlNode(filename, doc);
      parseWaypoints(doc, wps);
      parseTrajectories(doc, wps, trajs);
    }
    catch(YAML::ParserException& e)
    {
      ROS_ERROR("Parsing waypoints file failed: %s", e.what());
      return false;
    }
    catch(YAML::RepresentationException& e)
    {
      ROS_ERROR("Parsing waypoints file failed: %s", e.what());
      return false;
    }
    catch(std::string& e) {
      ROS_ERROR("Parsing waypoints file failed: %s",e.c_str());
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
    #ifdef HAVE_NEW_YAMLCPP
      const YAML::Node& wp_node_tmp = node["waypoints"];
      const YAML::Node* wp_node = wp_node_tmp ? &wp_node_tmp : NULL;
    #else
      const YAML::Node* wp_node = node.FindValue("waypoints");
    #endif

    if(wp_node != NULL)
    {
      for(unsigned int i = 0; i < wp_node->size(); ++i)
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
      ROS_INFO_STREAM("Parsed " << wps.waypoints.size() << " waypoints.");
    }
    else
    {
      ROS_WARN_STREAM("Couldn't find any waypoints in the provided yaml file.");
    }
  }

  void parseTrajectories(const YAML::Node& node,
                         const yocs_msgs::WaypointList& wps,
                         yocs_msgs::TrajectoryList& trajs)
  {
    unsigned int i;

    #ifdef HAVE_NEW_YAMLCPP
    const YAML::Node& wp_node_tmp = node["trajectories"];
    const YAML::Node* wp_node = wp_node_tmp ? &wp_node_tmp : NULL;
    #else
    const YAML::Node* wp_node = node.FindValue("trajectories");
    #endif
    if(wp_node != NULL)
    {
      for(i = 0; i < wp_node->size(); ++i)
      {
        // Parse trajectory entries on YAML
        yocs_msgs::Trajectory traj;

        // check if all specified waypoints are configured
        bool all_waypoints_found = true;

        for(unsigned int wp = 0; wp < (*wp_node)[i]["waypoints"].size(); ++wp)
        {
          bool wp_found = false;
          std::string wp_name;
          (*wp_node)[i]["waypoints"][wp] >> wp_name;
          for(unsigned int known_wp = 0; known_wp < wps.waypoints.size(); ++known_wp)
          {
            if (wp_name == wps.waypoints[known_wp].name)
            {
              traj.waypoints.push_back(wps.waypoints[known_wp]);
              wp_found = true;
              break;
            }
          }
          if (!wp_found)
          {
            all_waypoints_found = false;
            break;
          }
        }
        if (all_waypoints_found)
        {
          (*wp_node)[i]["name"] >> traj.name;
          trajs.trajectories.push_back(traj);
        }
      }
      ROS_INFO_STREAM("Parsed " << trajs.trajectories.size() << " trajectories.");
    }
    else
    {
      ROS_WARN_STREAM("Couldn't find any trajectories in the provided yaml file.");
    }
  }
}
