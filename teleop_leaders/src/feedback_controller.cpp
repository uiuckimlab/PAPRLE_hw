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

#include "feedback_controller/feedback_controller.h"

namespace feedback_controller
{
bool FeedbackController::init(
    hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle)
{
  // Joint interface
  effort_joint_interface_ =
      robot_hardware->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface_ == nullptr)
  {
    ROS_ERROR(
        "[FeedbackController] Could not get effort joint interface "
        "from hardware!");
    return false;
  }

  // Joint handle
  if (!node_handle.getParam("joint_names", joint_names_))
  {
    ROS_ERROR("[FeedbackController] Could not parse joint names");
    return false;
  }
  num_joints_ = joint_names_.size();
  for (size_t i = 0; i < joint_names_.size(); i++)
  {
    try
    {
      effort_joint_handles_.push_back(
          effort_joint_interface_->getHandle(joint_names_[i]));
    }
    catch (const hardware_interface::HardwareInterfaceException& e)
    {
      ROS_ERROR_STREAM(
          "[FeedbackController] Could not get joint handle: "
          << e.what());
      return false;
    }
  }

  for(size_t i = 0; i < joint_names_.size(); i++){
    double init_pose = 0.0;
    double spring_constant = 0.0;
    node_handle.param(std::string("init_pose/") + joint_names_[i],init_pose, 0.0);
    node_handle.param(std::string("spring_constant/") + joint_names_[i],spring_constant, 0.0);
    init_pose_[joint_names_[i]] = init_pose;
    spring_constant_[joint_names_[i]] = spring_constant;
  }

  node_handle.param(std::string("max_force"), max_torque_constant_, 0.45);

  // Resize the variables
  q_.resize(num_joints_);
  tau_.resize(num_joints_);

  return true;
}

void FeedbackController::starting(const ros::Time& time) {}

void FeedbackController::update(const ros::Time& time,
                                           const ros::Duration& period)
{
  
  // Get the current joint positions
  for (size_t i = 0; i < num_joints_; i++)
  {
    try{
        q_(i) = effort_joint_handles_[i].getPosition();
    }catch(const hardware_interface::HardwareInterfaceException& e){
        ROS_ERROR_STREAM("[FeedbackController] Could not get joint position: " << e.what());
        return;
    }
  }

  // Compute the gravity torque
  for (size_t i = 0; i < num_joints_; i++)
  {
    double des_max_radians = M_PI/2.0;

    double spring_constant = spring_constant_[joint_names_[i]];
    double spring_constK = spring_constant; 

    double desired_q = init_pose_[joint_names_[i]];
    tau_(i) = -1*(q_(i) - desired_q)*spring_constK;


    tau_(i) = std::max(-max_torque_constant_, tau_(i));
    tau_(i) = std::min(tau_(i), max_torque_constant_);

  }


  for (size_t i = 0; i < num_joints_; i++)
  {
    try{
        effort_joint_handles_[i].setCommand(tau_(i));
    }catch(const hardware_interface::HardwareInterfaceException& e){
        ROS_ERROR_STREAM("[FeedbackController] Could not set joint effort command: " << e.what());
        return;
    }
  }
}

void FeedbackController::stopping(const ros::Time& time) {}

}  // namespace feedback_controller

PLUGINLIB_EXPORT_CLASS(
    feedback_controller::FeedbackController,
    controller_interface::ControllerBase)