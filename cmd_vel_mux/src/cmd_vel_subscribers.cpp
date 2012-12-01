/*
 * cmd_vel_subscribers.cpp
 *
 *  Created on: Oct 31, 2012
 *      Author: jorge
 */

#include <fstream>
#include <iostream>

#include "cmd_vel_mux/cmd_vel_subscribers.h"


void CmdVelSubscribers::CmdVelSubs::operator << (const YAML::Node& node)
{
  node["name"]       >> name;
  node["topic"]      >> topic;
  node["timeout"]    >> timeout;
  node["priority"]   >> priority;   if (node.FindValue("short_desc") != NULL)
  node["short_desc"] >> short_desc;
}


bool CmdVelSubscribers::loadSubscribersCfg(std::string path)
{
  std::ifstream ifs(path.c_str(), std::ifstream::in);
  if (ifs.good() == false)
  {
    ROS_ERROR("Unable to read subscribers configuration file '%s'", path.c_str());
    return false;
  }

  bool result = true;

  try
  {
    YAML::Parser parser(ifs);

    YAML::Node doc;
    parser.GetNextDocument(doc);

    for (unsigned int i = 0; i < doc["subscribers"].size(); i++)
    {
      // Parse every entries on YAML
      CmdVelSubs subscriber(i);
      subscriber << doc["subscribers"][i];
      list.push_back(subscriber);
    }

    ROS_DEBUG("Subscribers configuration file %s successfully parsed", path.c_str());
  }
  catch(YAML::ParserException& e)
  {
    ROS_ERROR("Subscribers configuration file parse failed: %s", e.what());
    result = false;
  }
  catch(YAML::RepresentationException& e)
  {
    ROS_ERROR("Subscribers configuration file wrong format: %s", e.what());
    result = false;
  }

  ifs.close();
  return result;
}
