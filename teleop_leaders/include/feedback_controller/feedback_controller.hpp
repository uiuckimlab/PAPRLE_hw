// Copyright 2017 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/* This header must be included by all rclcpp headers which declare symbols
 * which are defined in the rclcpp library. When not building the rclcpp
 * library, i.e. when using the headers in other package's code, the contents
 * of this header change the visibility of certain symbols which the rclcpp
 * library cannot have, but the consuming code must have inorder to link.
 */

 #ifndef feedback_controller__feedback_controller_HPP_
 #define feedback_controller__feedback_controller_HPP_
 
 #include <memory>
 #include <string>
 #include <vector>
 #include <map>
 
 #include "controller_interface/controller_interface.hpp"
 #include "hardware_interface/types/hardware_interface_type_values.hpp"
 #include "rclcpp/rclcpp.hpp"
 #include "rclcpp_lifecycle/state.hpp"
 #include "sensor_msgs/msg/joint_state.hpp"
 #include "geometry_msgs/msg/transform.hpp"
 #include "std_msgs/msg/float64_multi_array.hpp"
 #include <kdl_parser/kdl_parser.hpp>
 #include <kdl/frames.hpp>
 #include <kdl/jntarray.hpp>
 #include <kdl/tree.hpp>
 #include <kdl/treeidsolver_recursive_newton_euler.hpp>

 #include <pinocchio/parsers/urdf.hpp>
 #include <pinocchio/multibody/model.hpp>
 #include <pinocchio/multibody/data.hpp>
 #include <pinocchio/algorithm/joint-configuration.hpp>
 #include <pinocchio/algorithm/kinematics.hpp>
 #include <pinocchio/algorithm/jacobian.hpp>
 #include <pinocchio/algorithm/frames.hpp>
 #include <pinocchio/algorithm/geometry.hpp>
 #include <Eigen/Dense>

 #include "feedback_controller/visibility_control.h"
 #include "yaml-cpp/yaml.h"
 #include "teleop_msgs/srv/set_base_pose.hpp"
 #include "teleop_msgs/srv/set_eef_names.hpp"
 #include "teleop_msgs/msg/eef_transforms.hpp"
 #include <std_msgs/msg/float64_multi_array.hpp>
 
 namespace feedback_controller
 {
 class FeedbackController : public controller_interface::ControllerInterface
 {
 public:
   feedback_controller_PUBLIC
   FeedbackController();
 
   feedback_controller_PUBLIC
   controller_interface::InterfaceConfiguration command_interface_configuration() const override;
 
   feedback_controller_PUBLIC
   controller_interface::InterfaceConfiguration state_interface_configuration() const override;
 
   feedback_controller_PUBLIC
   controller_interface::return_type update(
     const rclcpp::Time & time, const rclcpp::Duration & period) override;
 
   feedback_controller_PUBLIC
   controller_interface::CallbackReturn on_init() override;
 
   feedback_controller_PUBLIC
   controller_interface::CallbackReturn on_configure(
     const rclcpp_lifecycle::State & previous_state) override;
 
   feedback_controller_PUBLIC
   controller_interface::CallbackReturn on_activate(
     const rclcpp_lifecycle::State & previous_state) override;
 
   feedback_controller_PUBLIC
   controller_interface::CallbackReturn on_deactivate(
     const rclcpp_lifecycle::State & previous_state) override;
 
   feedback_controller_PUBLIC
   controller_interface::CallbackReturn on_cleanup(
     const rclcpp_lifecycle::State & previous_state) override;
 
   feedback_controller_PUBLIC
   controller_interface::CallbackReturn on_error(
     const rclcpp_lifecycle::State & previous_state) override;
 
   feedback_controller_PUBLIC
   controller_interface::CallbackReturn on_shutdown(
     const rclcpp_lifecycle::State & previous_state) override;

   feedback_controller_PUBLIC
   void ik_worker();

 protected:
   rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_feedback_subscription_;
   rclcpp::Subscription<teleop_msgs::msg::EEFTransforms>::SharedPtr eef_feedback_subscription_;
   rclcpp::Service<teleop_msgs::srv::SetBasePose>::SharedPtr set_base_pose_service_;
   rclcpp::Service<teleop_msgs::srv::SetEEFNames>::SharedPtr set_eef_name_service_;

   // Callback for joint feedback subscription
   void joint_feedback_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
   // Joint feedback subscription
   std::vector<std::string> joint_feedbacks_name;
   std::vector<double> joint_feedbacks_;
   std::vector<int> joint_feedbacks_index_;
   rclcpp::Time joint_feedbacks_time_;

    // Callback for end-effector feedback subscription
  void eef_feedback_callback(const teleop_msgs::msg::EEFTransforms::SharedPtr msg);
  // End-effector feedback subscription
  rclcpp::Time eef_feedbacks_time_;
  std::vector<std::string> eef_feedbacks_name_;
  std::vector<std_msgs::msg::Float64MultiArray> eef_transforms_;
  
  // Callback for set base position
  void set_joint_state_callback(
    const std::shared_ptr<teleop_msgs::srv::SetBasePose::Request> request,
    std::shared_ptr<teleop_msgs::srv::SetBasePose::Response> response);
  // Set base position
  std::vector<double> base_joint_positions_;
  std::vector<int> base_joint_name_index_;
  std::vector<int> base_joint_enabled_;

  void set_eef_name_callback(
    const std::shared_ptr<teleop_msgs::srv::SetEEFNames::Request> request,
    std::shared_ptr<teleop_msgs::srv::SetEEFNames::Response> response);


  std::string formatVector(const std::vector<double> & vec);

  std::thread ik_thread_;
  std::mutex ik_mutex_;
  bool running_ = false;
  std::atomic<bool> shutdown_{false};  // thread-safe flag
  std::optional<Eigen::Vector3d> eef_target_;
  Eigen::VectorXd latest_joint_result_;

  bool solve_ik(
    const pinocchio::Model &model,
    pinocchio::Data &data,
    const pinocchio::SE3 &target_pose,
    const std::string &eef_frame_name,
    Eigen::VectorXd &q_init,
    Eigen::VectorXd &q_out,
    int max_iter,
    double eps,
    double alpha);


   // Parameters
   struct Params {
    std::string robot_description = "robot_description";
    std::vector<std::string> joints = {};
    double Kp_gripper_homing = 0.0;
    double Kp_gripper_feedback = 0.0;
    double max_effort = 0.0;
  

    // Set end-effector name  
    std::vector<std::string> eef_links_;
    std::vector<std::string> limb_names_;
    
    std::vector<std::string> state_interfaces= {
      hardware_interface::HW_IF_POSITION,
      hardware_interface::HW_IF_VELOCITY,
    };
    std::vector<std::string> command_interfaces = {
      hardware_interface::HW_IF_EFFORT
    };
  
    std::vector<std::string> command_joints = {};

    std::vector<double> init_pose = {};

    // Only related to leader itself
    std::vector<double> Kp_nullspace_feedback = {};
    std::vector<double> Kp_base_pose_feedback = {};
    
    // Related to the follower robot 
    std::vector<double> Kp_follower_feedback = {};
    
    // for detecting if the parameter struct has been updated
    rclcpp::Time __stamp;
   };

   YAML::Node feedback_params;
   Params params_;
 
   // Pinocchio
   pinocchio::Model model_;
   pinocchio::Data data_; 

   std::vector<pinocchio::Model> reduced_models_;
   std::vector<pinocchio::Data> reduced_datas_;
   std::vector<pinocchio::JointIndex> unrelevant_joint_inds;
   std::vector<pinocchio::JointIndex> relevant_joint_inds;
   std::vector<int32_t> joint_mapping_;
   // create list of joint_mapping_
   std::vector<std::vector<int32_t>> joint_mapping_list_;
   pinocchio::Data::Matrix6x J;

   Eigen::Matrix<double, 6, 6> JJt;

  std::vector<pinocchio::JointIndex> get_relevant_joints_for_frame(const pinocchio::Model &model, const std::string &eef_frame_name);

   // State
   bool dither_switch_;
 
   // Interfaces
   std::vector<std::string> state_interface_types_ = {
     hardware_interface::HW_IF_POSITION,
     hardware_interface::HW_IF_VELOCITY,
   };
   std::vector<std::string> command_interface_types_ = {
     hardware_interface::HW_IF_EFFORT
   };
 
   // Degrees of freedom
   size_t dof_;
 
   // Storing command joint names for interfaces
   std::vector<std::string> command_joint_names_;
 
   // The interfaces are defined as the types in 'allowed_interface_types_' member.
   // For convenience, for each type the interfaces are ordered so that i-th position
   // matches i-th index in joint_names_
   template<typename T>
   using InterfaceReferences = std::vector<std::vector<std::reference_wrapper<T>>>;
 
   InterfaceReferences<hardware_interface::LoanedCommandInterface> joint_command_interface_;
   InterfaceReferences<hardware_interface::LoanedStateInterface> joint_state_interface_;
 
   std::vector<std::string> joint_names_;
   size_t n_joints_;
 
   std::vector<double> joint_positions_;
   std::vector<double> joint_velocities_;
 };
 
 }  // namespace feedback_controller
 
 #endif  // feedback_controller__feedback_controller_HPP_