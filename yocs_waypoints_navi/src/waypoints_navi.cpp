/*
 * Copyright (c) 2013, Jorge Santos, Yujin Robot.
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

#include <list>
#include <fstream>
#include <yaml-cpp/yaml.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include <yocs_math_toolkit/common.hpp>
#include <yocs_math_toolkit/geometry.hpp>


class WaypointsGoalNode
{
public:
  WaypointsGoalNode() : mode_(NONE), state_(IDLE), move_base_ac_("move_base", true)
  {
  }

  bool init()
  {
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    pnh.param("frequency",      frequency_,     1.0);
    pnh.param("close_enough",   close_enough_,  0.3);  // close enough to next waypoint
    pnh.param("goal_timeout",   goal_timeout_, 30.0);  // maximum time to reach a waypoint
    pnh.param("waypoints_file", waypoints_file_, std::string());

//    if (waypoints_file_.length() > 0)
//    {
//      if (readWaypointsFile(waypoints_file_) == true)
//      {
//        ROS_INFO("Waypoints file properly parsed: %s", waypoints_file_.c_str());
//        state_ = START;  // goal requested to start
//      }
//      else
//      {
//        ROS_ERROR("Waypoints file parse failed: %s", waypoints_file_.c_str());
//        return false;
//      }
//    }

    // Wait for the move_base action servers to come up
    ros::Time t0 = ros::Time::now();
    double timeout = 10.0;

    while ((move_base_ac_.waitForServer(ros::Duration(1.0)) == false) && (ros::ok() == true))
    {
      if ((ros::Time::now() - t0).toSec() > timeout/2.0)
        ROS_WARN_THROTTLE(3, "Waiting for move_base action server to come up...");

      if ((ros::Time::now() - t0).toSec() > timeout)
      {
        ROS_ERROR("Timeout while waiting for move_base action server to come up");
        return false;
      }
    }

//    waypoint_markers_pub_ = pnh.advertise<geometry_msgs::PoseStamped>("waypoint_markes", 1, true);
 // waypoint_markers_pub_ = pnh.advertise<geometry_msgs::PoseStamped>("waypoint_markes", 1, true);

    final_goal_sub_ = nh.subscribe("final_goal", 1, &WaypointsGoalNode::finalGoalCB, this);
    waypoints_sub_  = nh.subscribe("waypoints",  1, &WaypointsGoalNode::waypointCB, this);

    return true;
  }

  void finalGoalCB(const geometry_msgs::PoseStamped::ConstPtr& goal)
  {
    if ((state_ > IDLE) && (state_ < COMPLETED))
    {
      ROS_INFO("Already have a task; cancelling...");
      waypoints_.clear();
//      waypoints_it_.
      mode_  = NONE;
      state_ = IDLE;
      final_goal_ = geometry_msgs::PoseStamped();

      cancelAllGoals();
    }
    else
    {
      ROS_INFO("Goal received with %lu waypoints; lets' go!", waypoints_.size());

      mode_  = GOAL;
      state_ = START;
      final_goal_ = *goal;
    }
  }

  void waypointCB(const geometry_msgs::PointStamped::ConstPtr& point)
  {
    if ((state_ > IDLE) && (state_ < COMPLETED))
    {
      ROS_DEBUG("Already have a task; ignoring additional points");
    }
    else if ((waypoints_.size() > 0) && (mtk::distance2D(point->point, waypoints_.front().point) < 0.2))
    {
      // Waypoints loop closed; assume we are in loop mode and start moving
      ROS_INFO("Waypoints loop closed with %lu points; lets' go!", waypoints_.size());

      mode_  = LOOP;
      state_ = START;

      waypoints_it_ = waypoints_.begin();
    }
    else
    {
      ROS_DEBUG("Waypoint added: %s", mtk::point2str(point->point));
      waypoints_.push_back(*point);
    }
  }

  bool cancelAllGoals(double timeout = 2.0)
  {
    actionlib::SimpleClientGoalState goal_state = move_base_ac_.getState();
    if ((goal_state != actionlib::SimpleClientGoalState::ACTIVE) &&
        (goal_state != actionlib::SimpleClientGoalState::PENDING) &&
        (goal_state != actionlib::SimpleClientGoalState::RECALLED) &&
        (goal_state != actionlib::SimpleClientGoalState::PREEMPTED))
    {
      // We cannot cancel a REJECTED, ABORTED, SUCCEEDED or LOST goal
      ROS_WARN("Cannot cancel move base goal, as it has %s state!", goal_state.toString().c_str());
      return true;
    }

    ROS_INFO("Canceling move base goal with %s state...", goal_state.toString().c_str());
    move_base_ac_.cancelAllGoals();
    if (move_base_ac_.waitForResult(ros::Duration(timeout)) == false)
    {
      ROS_WARN("Cancel move base goal didn't finish after %.2f seconds: %s",
               timeout, goal_state.toString().c_str());
      return false;
    }

    ROS_INFO("Cancel move base goal succeed. New state is %s", goal_state.toString().c_str());
    return true;
  }

  void spin()
  {
    move_base_msgs::MoveBaseGoal mb_goal;

    ros::Rate rate(frequency_);

    while (ros::ok())
    {
      ros::spinOnce();
      rate.sleep();

      if ((state_ <= READY) || (state_ >= COMPLETED))
        continue;

      if (state_ == ACTIVE)
      {
        actionlib::SimpleClientGoalState goal_state = move_base_ac_.getState();
        if ((goal_state == actionlib::SimpleClientGoalState::ACTIVE) ||
            (goal_state == actionlib::SimpleClientGoalState::PENDING) ||
            (goal_state == actionlib::SimpleClientGoalState::RECALLED) ||
            (goal_state == actionlib::SimpleClientGoalState::PREEMPTED))
        {
          // We are still pursuing a goal...

          if ((ros::Time::now() - mb_goal.target_pose.header.stamp).toSec() >= goal_timeout_)
          {
            ROS_WARN("Cannot reach goal after %.2f seconds; request a new one (current state is %s)",
                      goal_timeout_, move_base_ac_.getState().toString().c_str());
          }
          else if (equals(mb_goal.target_pose, final_goal_) == false)
          {
            // When close enough to current goal (except for the final one!), go for the
            // next waypoint, so we avoid the final slow approach and subgoal obsession
            tf::StampedTransform robot_gb;
            try
            {
              tf_listener_.lookupTransform("/map", "/base_footprint", ros::Time(0.0), robot_gb); // TODO make params
            }
            catch (tf::TransformException& e)
            {
              ROS_WARN("Cannot get tf %s -> %s: %s", "/map", "/base_footprint", e.what());
              continue;
            }

            double distance =  std::sqrt(std::pow(robot_gb.getOrigin().x() - mb_goal.target_pose.pose.position.x, 2)
                                       + std::pow(robot_gb.getOrigin().y() - mb_goal.target_pose.pose.position.y, 2));
            if (distance > close_enough_)
            {
              continue;
            }
            else
            {
              ROS_DEBUG("Close enough to current goal (%.2f <= %.2f m); request a new one", distance, close_enough_);
            }
          }
          else
          {
            // keep going to except for the final goal
            continue;
          }
        }
        else if (equals(mb_goal.target_pose, final_goal_) == true)
        {
          mode_  = NONE;
          state_ = IDLE;

          final_goal_ = geometry_msgs::PoseStamped();

          // Final goal achieved or failed; check what happened
          if (move_base_ac_.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
          {
            ROS_ERROR("Go to goal failed: %s", move_base_ac_.getState().toString().c_str());
          }
          else
          {
            ROS_INFO("Go to goal successfully completed: %.2f, %.2f, %.2f",
                     mb_goal.target_pose.pose.position.x, mb_goal.target_pose.pose.position.y,
                     tf::getYaw(mb_goal.target_pose.pose.orientation));
          }

          continue;
        }
      } // if (state_ == ACTIVE)

      if (waypoints_.size() > 0)
      {
//        if (state_ == GOAL_ACTIVE)
//          waypoints_.pop_front();   // Not the first waypoint; discard already reached one

        geometry_msgs::PointStamped next;
        if (mode_ == GOAL)
        {
          next = waypoints_.front();
        }
        else      // LOOP
        {
          next = *waypoints_it_;
          waypoints_it_++;
          if (waypoints_it_ == waypoints_.end())
            waypoints_it_ = waypoints_.begin();
        }

        mb_goal.target_pose.header.stamp = ros::Time::now();
        mb_goal.target_pose.header.frame_id = "/map";
        mb_goal.target_pose.pose.position = next.point;
        mb_goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);  // TODO use the heading from robot loc to next (front)

        // If not in loop mode, remove current waypoint from the list so it's not showed as a marker but as a goal
        if (mode_ == GOAL)
          waypoints_.pop_front();
      }
      else
      {
        mb_goal.target_pose = final_goal_;
        mb_goal.target_pose.header.stamp = ros::Time::now();
      }

   //   exploration_goal_pub_.publish(mb_goal.target_pose);
      ROS_INFO("New goal: %.2f, %.2f, %.2f",
               mb_goal.target_pose.pose.position.x, mb_goal.target_pose.pose.position.y,
               tf::getYaw(mb_goal.target_pose.pose.orientation));

      // TODO This is a horrible workaround for a problem I cannot solve: send a new goal
      // when the previous one has been cancelled return immediately with succeeded state
      int times_sent = 0;
      do
      {
        move_base_ac_.sendGoal(mb_goal);
        times_sent++;
      } while ((move_base_ac_.waitForResult(ros::Duration(0.1)) == true) &&
               (move_base_ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED));

      if (times_sent > 1)
        ROS_WARN("Again the strange case of instantaneous goals... (goal sent %d times)", times_sent);

      state_ = ACTIVE;
    }
  }

private:
  enum { NONE = 0,
         GOAL,
         LOOP
       } mode_;

  enum { IDLE = 0,
         READY,
         START,
         ACTIVE,
//         GOAL_WAIPOINTS,
//         GOAL_LAST_LEG,
         COMPLETED
       } state_;

  double frequency_;
  double close_enough_;
  double goal_timeout_;

  std::string waypoints_file_;

  std::list<geometry_msgs::PointStamped>           waypoints_;
  std::list<geometry_msgs::PointStamped>::iterator waypoints_it_;
  geometry_msgs::PoseStamped                       final_goal_;

  tf::TransformListener   tf_listener_;
  ros::Publisher exploration_goal_pub_;
  ros::Subscriber final_goal_sub_;
  ros::Subscriber waypoints_sub_;

  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base_ac_;


  bool readWaypointsFile(std::string file)
  {
    waypoints_.clear();
    try
    {
      /*********************
      ** Yaml File Parsing
      **********************/
      std::ifstream ifs(file.c_str(), std::ifstream::in);
      if (ifs.good() == false)
      {
        ROS_ERROR("Waypoints file not found [%s]", file.c_str());
        return false;
      }

      YAML::Parser parser(ifs);
      YAML::Node doc;
      parser.GetNextDocument(doc);

      const YAML::Node *wp_node = doc.FindValue("waypoints");
      if (wp_node != NULL)
      {
        for (unsigned int i = 0; i < wp_node->size(); i++)
        {
          // Parse waypoint entries on YAML
          geometry_msgs::PointStamped point;

  //        (*wp_node)[i]["header"]["seq"]       >> point.header.seq;
  //        (*wp_node)[i]["header"]["stamp"]     >> point.header.stamp;
  //        (*wp_node)[i]["header"]["frame_id"]  >> point.header.frame_id;

          (*wp_node)[i]["point"]["x"]          >> point.point.x;
          (*wp_node)[i]["point"]["y"]          >> point.point.y;
          (*wp_node)[i]["point"]["z"]          >> point.point.z;

          waypoints_.push_back(point);
        }
      }
      else
      {
        ROS_DEBUG("No waypoints in file; assuming go to goal mode");  // still a valid file in goal mode
      }

      const YAML::Node *fg_node = doc.FindValue("final_goal");
      if ((wp_node == NULL) && (fg_node == NULL))
      {
        ROS_ERROR("Missing required key: final_goal");
        return false;
      }

      if (fg_node != NULL)
      {
        // Parse goal pose
        geometry_msgs::PoseStamped goal;

  //      (*fg_node)["header"]["seq"]            >> goal.header.seq;
  //      (*fg_node)["header"]["stamp"]          >> goal.header.stamp;
        (*fg_node)["header"]["frame_id"]       >> goal.header.frame_id;

        (*fg_node)["pose"]["position"]["x"]    >> goal.pose.position.x;
        (*fg_node)["pose"]["position"]["y"]    >> goal.pose.position.y;
        (*fg_node)["pose"]["position"]["z"]    >> goal.pose.position.z;

        (*fg_node)["pose"]["orientation"]["x"] >> goal.pose.orientation.x;
        (*fg_node)["pose"]["orientation"]["y"] >> goal.pose.orientation.y;
        (*fg_node)["pose"]["orientation"]["z"] >> goal.pose.orientation.z;
        (*fg_node)["pose"]["orientation"]["w"] >> goal.pose.orientation.w;

        final_goal_ = goal;
        mode_ = GOAL;
      }
      else
      {
        ROS_DEBUG("No final goal in file; assuming loop mode");  // still a valid file in loop mode
        waypoints_it_ = waypoints_.begin();
        mode_ = LOOP;
      }

      state_ = READY;

      ROS_DEBUG("Waypoints file successfully parsed: %s", waypoints_file_.c_str());
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

    return true;
  }

  bool equals(const geometry_msgs::PoseStamped& a, const geometry_msgs::PoseStamped& b)
  {
    return ((a.pose.position.x == b.pose.position.x) &&
            (a.pose.position.y == b.pose.position.y) &&
            (a.pose.position.z == b.pose.position.z));
    // TODO make decent, with rotation (tk::minAngle, I think) and frame_id and put in math toolkit
  }

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, ROS_PACKAGE_NAME);

  WaypointsGoalNode eg;
  if (eg.init() == false)
  {
    ROS_ERROR("%s initialization failed", ros::this_node::getName().c_str());
    return -1;
  }
  ROS_INFO("%s initialized", ros::this_node::getName().c_str());
  eg.spin();
  return 0;
}
