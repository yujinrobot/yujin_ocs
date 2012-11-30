/**
 * @file /safety_controller/include/safety_controller/safety_controller.hpp
 *
 * @brief Definition of a default controller
 *
 * A safety controller should ensure the safe operation of a robot.
 * This includes the robot does not harm itself or someone/something else.
 * One example applications is a controller, which reacts on bumper, cliff sensor
 * and wheel drop events.
 *
 * @date Nov 30, 2012
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef SAFETY_CONTROLLER_HPP_
#define SAFETY_CONTROLLER_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

/**
 * @brief Controller template
 *
 * Use this class for implementing various safety-related controllers
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
   * @brief Toggle the state of the controller between disabled and enabled
   */
  void changeState()
  {
    if (controller_active_)
    {
      controller_active_ = false;
    }
    else
    {
      controller_active_ = true;
    }
  };

  /**
   * @brief For complex controlling work
   *
   * In case the functionality of the controller can't be implemented with callbacks only, implement this method.
   */
  virtual void spin()
  {
    // do nothing
  };

private:
  bool controller_active_;
};


#endif /* SAFETY_CONTROLLER_HPP_ */
