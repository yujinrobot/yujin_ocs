/**
 * @file /src/cmd_vel_subscribers.cpp
 *
 * @brief  Subscriber handlers for the cmd_vel_mux
 *
 * License: BSD
 *   https://raw.github.com/yujinrobot/yujin_ocs/master/cmd_vel_mux/LICENSE
 **/
/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <fstream>
#include "../include/cmd_vel_mux/cmd_vel_subscribers.hpp"
#include "../include/cmd_vel_mux/exceptions.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace cmd_vel_mux {

/*****************************************************************************
 ** Implementation
 *****************************************************************************/

void CmdVelSubscribers::CmdVelSubs::operator << (const YAML::Node& node)
{
  node["name"]       >> name;
  node["topic"]      >> topic;
  node["timeout"]    >> timeout;
  node["priority"]   >> priority;
  if (node.FindValue("short_desc") != NULL) {
    node["short_desc"] >> short_desc;
  }
}

void CmdVelSubscribers::configure(const std::string &yaml_configuration_file) {

  list.clear();

  /*********************
  ** Yaml File Parsing
  **********************/
  std::ifstream ifs(yaml_configuration_file.c_str(), std::ifstream::in);
  if (ifs.good() == false)
  {
    throw FileNotFoundException(yaml_configuration_file);
  }
  try
  {
    YAML::Parser parser(ifs);

    YAML::Node doc;
    parser.GetNextDocument(doc);

    if ( doc["subscribers"].size() == 0 ) {
      throw EmptyCfgException();
    }

    for (unsigned int i = 0; i < doc["subscribers"].size(); i++)
    {
      // Parse every entries on YAML
      CmdVelSubs subscriber(i);
      subscriber << doc["subscribers"][i];
      list.push_back(subscriber);
    }
  }
  catch(EmptyCfgException& e) {
    throw e;
  }
  catch(YAML::ParserException& e)
  {
    throw YamlException(e.what());
  }
  catch(YAML::RepresentationException& e)
  {
    throw YamlException(e.what());
  }
  ifs.close();
}


} // namespace cmd_vel_mux
