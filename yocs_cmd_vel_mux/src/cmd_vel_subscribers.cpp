/**
 * @file /src/cmd_vel_subscribers.cpp
 *
 * @brief  Subscriber handlers for the yocs_cmd_vel_mux
 *
 * License: BSD
 *   https://raw.github.com/yujinrobot/yujin_ocs/hydro/yocs_cmd_vel_mux/LICENSE
 **/
/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <fstream>

#include "yocs_cmd_vel_mux/cmd_vel_subscribers.hpp"
#include "yocs_cmd_vel_mux/exceptions.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace yocs_cmd_vel_mux {

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

void CmdVelSubscribers::configure(const YAML::Node& node) {

  list.clear();
  try
  {
    if ( node.size() == 0 ) {
      throw EmptyCfgException();
    }

    for (unsigned int i = 0; i < node.size(); i++)
    {
      // Parse every entries on YAML
      CmdVelSubs subscriber(i);
      subscriber << node[i];
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
}


} // namespace yocs_cmd_vel_mux
