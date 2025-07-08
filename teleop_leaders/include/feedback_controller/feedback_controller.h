/*******************************************************************************
* Copyright 2020 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Ryan Shim */

#ifndef LEADERS_GRAVITY_COMPENSATION_CONTROLLER_H
#define LEADERS_GRAVITY_COMPENSATION_CONTROLLER_H

#include <boost/scoped_ptr.hpp>
#include <string>
#include <vector>

#include <controller_interface/controller.h>
#include <controller_interface/controller_base.h>
#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <kdl/tree.hpp>
#include <kdl/kdl.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>
#include <pluginlib/class_list_macros.h>
#include <ros/node_handle.h>
#include <ros/ros.h>
#include <ros/time.h>
#include <urdf/model.h>
#include <cmath>
#include <unordered_map>

namespace feedback_controller
{
class FeedbackController
    : public controller_interface::MultiInterfaceController<
          hardware_interface::EffortJointInterface>
{
 public:
  bool init(hardware_interface::RobotHW* robot_hardware,
            ros::NodeHandle& node_handle) override;
  void starting(const ros::Time& time) override;
  void update(const ros::Time& time, const ros::Duration& period) override;
  void stopping(const ros::Time& time) override;

 private:
  // Number of joints controlled
  int num_joints_;

  double max_torque_constant_ = 0.0;

  // Joint handle
  hardware_interface::EffortJointInterface* effort_joint_interface_;
  std::vector<hardware_interface::JointHandle> effort_joint_handles_;
  std::vector<std::string> joint_names_;
  std::unordered_map<std::string, double> init_pose_;
  std::unordered_map<std::string, double> spring_constant_;
  // KDL
  KDL::Tree kdl_tree_;
  KDL::Chain kdl_chain_;
  KDL::JntArray q_;
  KDL::JntArray tau_;

};


}  // namespace feedback_controller
#endif  // OPEN_MANIPULATOR_CONTROLLERS_GRAVITY_COMPENSATION_CONTROLLER_H