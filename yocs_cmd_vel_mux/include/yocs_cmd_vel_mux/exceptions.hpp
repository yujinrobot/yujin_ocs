/**
 * @file /cmd_vel_mux/include/yocs_cmd_vel_mux/exceptions.hpp
 *
 * @brief Exception classes for cmd_vel_mux.
 *
 * License: BSD
 *   https://raw.github.com/yujinrobot/yujin_ocs/hydro/yocs_cmd_vel_mux/LICENSE
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef YUJIN_OCS_CMD_VEL_EXCEPTIONS_HPP_
#define YUJIN_OCS_CMD_VEL_EXCEPTIONS_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <exception>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace yocs_cmd_vel_mux {

/*****************************************************************************
** Exceptions
*****************************************************************************/

class FileNotFoundException: public std::runtime_error {
public:
  FileNotFoundException(const std::string& msg)
        : std::runtime_error(msg) {}
        virtual ~FileNotFoundException() throw() {}
};

class EmptyCfgException: public std::runtime_error {
public:
  EmptyCfgException()
        : std::runtime_error("") {}
        virtual ~EmptyCfgException() throw() {}
};

class YamlException: public std::runtime_error {
public:
  YamlException(const std::string& msg)
        : std::runtime_error(msg) {}
        virtual ~YamlException() throw() {}
};

} // namespace yocs_cmd_vel_mux

#endif /* YUJIN_OCS_CMD_VEL_EXCEPTIONS_HPP_ */
