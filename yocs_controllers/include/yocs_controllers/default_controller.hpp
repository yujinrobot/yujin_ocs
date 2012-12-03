/*
 * Copyright (c) 2012, Yujin Robot.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Yujin Robot nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file /yocs_controllers/include/yocs_controllers/default_controller.hpp
 *
 * @brief Definition of a simple controller
 *
 * @date Nov 30, 2012
 **/

/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef DEFAULT_CONTROLLER_HPP_
#define DEFAULT_CONTROLLER_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

namespace yocs
{

/**
 * @brief Controller template
 *
 * Use this class for implementing various controllers.
 */
class Controller
{
public:
  Controller() : controller_active_(false)
  {};
  virtual ~Controller(){};
  /**
   * @brief Initialise the controller
   *
   * Needs to be defined by any class inheriting from SafetyController
   *
   * @return true, if initialisation was succcessful
   */
  virtual bool init() = 0;

  /**
   * @brief Enables the controller
   *
   * @return Returns true, if controller was inactive, false otherwise
   */
  bool enable()
  {
    if (controller_active_)
    {
      return false;
    }
    else
    {
      controller_active_ = true;
      return true;
    }
  };

  /**
   * @brief Disables the controller
   *
   * @return Returns true, if controller was active, false otherwise
   */
  bool disable()
  {
    if (!controller_active_)
    {
      return false;
    }
    else
    {
      controller_active_ = false;
      return true;
    }
  };

  /**
   * @brief Returns the current state of the controller
   *
   * @return controller state variable
   */
  bool getState()
  {
    return controller_active_;
  }
  /**
   * @brief For complex controlling work
   *
   * If needed, implement your complex algorithm here.
   */
  virtual void spin()
  {
    // do nothing
  };

private:
  bool controller_active_;
};

} // namespace yocs

#endif /* DEFAULT_CONTROLLER_HPP_ */
