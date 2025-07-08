#include "feedback_controller/feedback_controller.hpp"
#include <chrono>
#include <string>
#include <stdexcept>
#include "rclcpp/rclcpp.hpp"
#include "controller_interface/helpers.hpp"
#include <yaml-cpp/yaml.h>
#include <pinocchio/algorithm/model.hpp>

namespace feedback_controller
{
FeedbackController::FeedbackController()
: controller_interface::ControllerInterface(),
  dither_switch_(false)
{
}

controller_interface::InterfaceConfiguration
FeedbackController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (const auto & joint_name : joint_names_) {
    for (const auto & interface_type : command_interface_types_) {
      config.names.push_back(joint_name + "/" + interface_type);
    }
  }

  return config;
}

controller_interface::InterfaceConfiguration
FeedbackController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (const auto & joint_name : joint_names_) {
    for (const auto & interface_type : state_interface_types_) {
      config.names.push_back(joint_name + "/" + interface_type);
    }
  }

  return config;
}

void FeedbackController::ik_worker() {
  rclcpp::Rate rate(100);  // control how fast this loop runs

  Eigen::VectorXd q = Eigen::VectorXd::Zero(model_.nq);      // joint positions
  Eigen::VectorXd q_dot = Eigen::VectorXd::Zero(model_.nv);      // joint velocities
  Eigen::VectorXd torques = Eigen::VectorXd::Zero(model_.nv);    // joint torques
  
  while (!shutdown_) {
    std::vector<std_msgs::msg::Float64MultiArray> curr_eef_transforms;

    {
      std::lock_guard<std::mutex> lock(ik_mutex_);
      if (eef_transforms_.size() > 0) {
        curr_eef_transforms = eef_transforms_;
      }
    }

    if (curr_eef_transforms.size() == 0) {
      rate.sleep();
      continue;
    }
    // Populate joint positions and velocities from state interfaces
  
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      q(i) = joint_positions_[i];
      q_dot(i) = joint_velocities_[i];
  
    }

    for (size_t i=0; i < reduced_models_.size(); ++i) {
      auto reduced_model = reduced_models_[i];
      auto reduced_data = reduced_datas_[i];
      Eigen::VectorXd q_for_reduced = Eigen::VectorXd::Zero(reduced_model.nq);
      // generate q_for_reduced from q
      for (pinocchio::JointIndex ind = 1; ind < reduced_model.joints.size(); ++ind) {
        if (reduced_model.joints[ind].nv() > 0) {
          std::cout << "i" << ind << " name: " <<  reduced_model.names[ind]  << " joint_id: " << reduced_model.joints[ind].idx_q() << std::endl;
          q_for_reduced[reduced_model.joints[ind].idx_q()] = q(joint_mapping_list_[i][ind-1]); // quick fix, make it general to multiple arms
        }
      }
      // std::cout << "q_full: " << q.transpose() << std::endl;
      // std::cout << "q_for_reduced: " << q_for_reduced.transpose() << std::endl;

      pinocchio::forwardKinematics(reduced_model, reduced_data, q_for_reduced);
      pinocchio::updateFramePlacements(reduced_model, reduced_data);
      pinocchio::FrameIndex eef_id = reduced_model.getFrameId(params_.eef_links_[i]);
      pinocchio::Data::Matrix6x J(6, reduced_model.nv);
      pinocchio::computeFrameJacobian(reduced_model, reduced_data, q_for_reduced, eef_id, pinocchio::LOCAL, J);
      std::cout << "J: " << J << std::endl;
      Eigen::VectorXd err(6);
      for (size_t j = 0; j < 6; ++j) {
        err[j] = curr_eef_transforms[i].data[j];
      }
      auto tau_feedback = J.transpose() * err;

      std::cout << "EEF id: " << eef_id << std::endl;
      std::cout << "EEF name: " << params_.eef_links_[i] << " id: " << eef_id << std::endl;
      std::cout << "EEF transform: " << err.transpose() << std::endl;
      std::cout << "EEF feedback: " << tau_feedback.transpose() << std::endl;

    }


    rate.sleep();
  }
}

controller_interface::return_type FeedbackController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{


  // Calculate Time for whole update
  auto start_time = std::chrono::high_resolution_clock::now();

  auto assign_point_from_interface =
    [&](std::vector<double> & trajectory_point_interface, const auto & joint_interface) {
      for (size_t index = 0; index < dof_; ++index) {
        trajectory_point_interface[index] = joint_interface[index].get().get_value();
      }
  };
  assign_point_from_interface(joint_positions_, joint_state_interface_[0]);
  assign_point_from_interface(joint_velocities_, joint_state_interface_[1]);

  Eigen::VectorXd q = Eigen::VectorXd::Zero(model_.nq);      // joint positions
  Eigen::VectorXd q_dot = Eigen::VectorXd::Zero(model_.nv);      // joint velocities
  Eigen::VectorXd torques = Eigen::VectorXd::Zero(model_.nv);    // joint torques
  
  // Populate joint positions and velocities from state interfaces
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    q(i) = joint_positions_[i];
    q_dot(i) = joint_velocities_[i];

  }
  // Base Pose 
  // print base_joint_name_index_
  std::cout << "base_joint_name_index_: ";
  for (size_t i = 0; i < base_joint_name_index_.size(); ++i) {
    std::cout << base_joint_name_index_[i] << " ";
  }
  std::cout << std::endl;
  if (base_joint_name_index_.size() > 0) {
    for (size_t i = 0; i < reduced_models_.size(); ++i) {
      auto reduced_model = reduced_models_[i];
      auto reduced_data = reduced_datas_[i];

      pinocchio::Data::Matrix6x J_null(6, reduced_model.nv);
      Eigen::VectorXd q_for_null = Eigen::VectorXd::Zero(reduced_model.nq);
      for (pinocchio::JointIndex ind = 1; ind < reduced_model.joints.size(); ++ind) {
        if (reduced_model.joints[ind].nv() > 0) {
          q_for_null[joint_mapping_list_[i][ind-1]] = q(ind-1); // quick fix, make it general to multiple arms
        }
      }
      std::cout << "q_for_null: " << q_for_null.transpose() << std::endl;
  
      // declare num_arm_joints as unsigned int
      unsigned int num_arm_joints = reduced_model.nq;

      pinocchio::computeJointJacobian(reduced_model, reduced_data, q_for_null, num_arm_joints, J_null);  // or specific joint id
      
      std::cout << "J_null: " << J_null << std::endl;
  
      Eigen::MatrixXd J_dagger = J_null.completeOrthogonalDecomposition().pseudoInverse();
      Eigen::MatrixXd I = Eigen::MatrixXd::Identity(num_arm_joints, num_arm_joints);
      Eigen::MatrixXd null_space_projector = I - J_dagger * J_null;
      std::cout << "Null space projector: " << null_space_projector << std::endl;
  
      Eigen::VectorXd null_space_joint_target = Eigen::VectorXd::Zero(num_arm_joints);
      for (size_t i = 0; i < num_arm_joints; ++i) {
        null_space_joint_target(i) = base_joint_positions_[base_joint_name_index_[i]];
      }
      Eigen::VectorXd q_error = q_for_null - null_space_joint_target.head(num_arm_joints);
      Eigen::VectorXd tau_n = null_space_projector * (-0.1 * q_error);
      // std::cout << "Null space torque: " << tau_n.transpose() << std::endl;


    }

  };


  std::string torques_str = "Torques: ";
  std::string pose_str = "Base Pose Diff: ";
  for (size_t i = 0; i < model_.nq; ++i) {
    if (i >= joint_names_.size()) {
      continue;
    }
    torques(i) = 0.0;


    // Base Pose 
    if (base_joint_name_index_.size() > 0 && base_joint_enabled_[i] == 1) {
      double des_max_radians = M_PI/2.0;
      double spring_constK = 0.025 / des_max_radians; 
      double diff =  base_joint_positions_[base_joint_name_index_[i]] - q(i);
      // handling boundary 3.14
      if (diff > M_PI) {
        diff = diff - 2 * M_PI;
      } else if (diff < -M_PI) {
        diff = diff + 2 * M_PI;
      }
      torques(i) = diff * params_.Kp_base_pose_feedback[i]; // spring_constK;
      pose_str += std::to_string(diff) + " ";
    } else if (params_.init_pose.size() > 0) {
      // If no base pose is set, use the initial pose
      double diff = params_.init_pose[i] - q(i);
      // handling boundary 3.14
      if (diff > M_PI) {
        diff = diff - 2 * M_PI;
      } else if (diff < -M_PI) {
        diff = diff + 2 * M_PI;
      }
      torques(i) = diff * params_.Kp_base_pose_feedback[i]; // spring_constK;
      pose_str += std::to_string(diff) + " ";
    }
    
    // Feedback from follower side - read joint_feedbacks and apply torque if we get feedback msgs
    if (joint_feedbacks_index_.size() > 0){
      // Check when we get feedback msgs
      double time_diff = (get_node()->now()-joint_feedbacks_time_).seconds();
      //RCLCPP_INFO(get_node()->get_logger(), "Time difference: %f", time_diff);
      if (time_diff < 0.01) {
        torques(i) += joint_feedbacks_[joint_feedbacks_index_[i]] * params_.Kp_follower_feedback[i];
      }      
    }

    torques_str += std::to_string(torques(i)) + " ";

    joint_command_interface_[0][i].get().set_value(torques(i));
  }
// 
  // Print the torques
  // RCLCPP_INFO(get_node()->get_logger(), "%s", torques_str.c_str());
  // Print the friction torques
  //RCLCPP_INFO(get_node()->get_logger(), "%s", friction_str.c_str());

  // Print the base pose
  RCLCPP_INFO(get_node()->get_logger(), "%s", pose_str.c_str());

  dither_switch_ = !dither_switch_;  // Flip the dither switch

  // Calculate Time for whole update
  auto end_time = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
  // std::cout << "Update duration: " << duration.count() << " ms" << std::endl;
  return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn FeedbackController::on_init()
{
  // Create Service for setting base pose
  set_base_pose_service_ = get_node()->create_service<teleop_msgs::srv::SetBasePose>(
    "set_base_pos",
    std::bind(&FeedbackController::set_joint_state_callback, this, std::placeholders::_1, std::placeholders::_2));

  // Create joint feedback subscriber
  joint_feedback_subscription_ = get_node()->create_subscription<sensor_msgs::msg::JointState>(
    "followers_joint_feedback",
    rclcpp::SensorDataQoS(),
    std::bind(&FeedbackController::joint_feedback_callback, this, std::placeholders::_1)); 
  if (!joint_feedback_subscription_) {  // Check if the subscriber was created successfully
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to create joint feedback subscriber.");
    return CallbackReturn::ERROR;
  }

  eef_feedback_subscription_  = get_node()->create_subscription<teleop_msgs::msg::EEFTransforms>(
    "followers_eef_feedback",
    rclcpp::SensorDataQoS(),
    std::bind(&FeedbackController::eef_feedback_callback, this, std::placeholders::_1));
  if (!eef_feedback_subscription_) {  // Check if the subscriber was created successfully
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to create EEF feedback subscriber.");
    return CallbackReturn::ERROR;
  }
  

  return CallbackReturn::SUCCESS;
}

std::vector<pinocchio::JointIndex> FeedbackController::get_relevant_joints_for_frame(const pinocchio::Model &model,  const std::string &eef_frame_name)
{
pinocchio::FrameIndex eef_id = model.getFrameId(eef_frame_name);
if (eef_id == model.nframes) {
throw std::runtime_error("Frame '" + eef_frame_name + "' not found.");
}

pinocchio::JointIndex joint_id = model.frames[eef_id].parent;

// Climb the chain, include only joints with DOF (i.e., nv > 0)
while (joint_id != 0) {
  if (model.joints[joint_id].nv() > 0) {
  relevant_joint_inds.push_back(joint_id);
  std::cout << " - Joint [" << joint_id << "]: " << model.names[joint_id] << "\n";
  }
joint_id = model.parents[joint_id];
}

// check all joints, if they are not in the relevant_joint_inds, add them to unrelevant_joint_inds
for (pinocchio::JointIndex i = 1; i < model.njoints; ++i) {
  if (std::find(relevant_joint_inds.begin(), relevant_joint_inds.end(), i) == relevant_joint_inds.end()) {
    unrelevant_joint_inds.push_back(i);
    std::cout << " - Joint [" << i << "]: " << model.names[i] << " <- Not relevant \n";
  }
}

return std::vector<pinocchio::JointIndex>(relevant_joint_inds.begin(), relevant_joint_inds.end());
}

controller_interface::CallbackReturn FeedbackController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto logger = get_node()->get_logger();
  get_node()->get_parameter("robot_description", params_.robot_description);
  std::string feedback_param_yaml;
  get_node()->get_parameter("feedback_param_yaml", feedback_param_yaml);
  fprintf(stderr, "feedback_param_yaml: %s\n", feedback_param_yaml.c_str());
  feedback_params = YAML::LoadFile(feedback_param_yaml);

  // Update params_ from YAML file, reacing Node one by one
  for (const auto & param : feedback_params) {
    std::string param_name = param.first.as<std::string>();
    if (param_name == "joints") {
      for (const auto & joint : param.second) {
        params_.joints.push_back(joint.as<std::string>());
      }
    } else if (param_name == "init_pose"){
      std::string init_pose_str = "init_pose: ";
      for (const auto & val : param.second) {
        params_.init_pose.push_back(val.as<double>());
        init_pose_str += std::to_string(val.as<double>()) + " ";
      }
      // print the init pose
      RCLCPP_INFO(logger, "%s", init_pose_str.c_str());
    }else if (param_name == "max_effort") {
      params_.max_effort = param.second.as<double>();
    } else if (param_name == "Kp_follower_feedback"){
      for (const auto & joint : param.second) {
        params_.Kp_follower_feedback.push_back(joint.as<double>());
      }
    } else if (param_name == "Kp_nullspace_feedback"){
      for (const auto & joint : param.second) {
        params_.Kp_nullspace_feedback.push_back(joint.as<double>());
      }
    } else if (param_name == "Kp_base_pose_feedback"){
      for (const auto & joint : param.second) {
        params_.Kp_base_pose_feedback.push_back(joint.as<double>());
      }
    } else if (param_name == "limb_names"){
      for (const auto & joint : param.second) {
        params_.limb_names_.push_back(joint.as<std::string>());
        RCLCPP_INFO(logger, "Limb name: %s", joint.as<std::string>().c_str());
      }
    } else if (param_name == "eef_links"){
      for (const auto & joint : param.second) {
        params_.eef_links_.push_back(joint.as<std::string>());
        RCLCPP_INFO(logger, "EEF name: %s", joint.as<std::string>().c_str());
      }
    }   
  }
  
  dof_ = params_.joints.size();
  joint_positions_.resize(dof_);
  joint_velocities_.resize(dof_);

  if (params_.joints.empty()) {
    // TODO(destogl): is this correct? Can we really move-on if no joint names are not provided?
    RCLCPP_WARN(logger, "'joints' parameter is empty.");
  }


  joint_names_ = params_.joints;
  n_joints_ = joint_names_.size();

  command_joint_names_ = params_.command_joints;

  if (command_joint_names_.empty()) {
    command_joint_names_ = params_.joints;
    RCLCPP_INFO(
      logger, "No specific joint names are used for command interfaces. Using 'joints' parameter.");
  }

  joint_command_interface_.resize(command_interface_types_.size());
  joint_state_interface_.resize(state_interface_types_.size());

  const std::string & urdf = params_.robot_description; 

  try {
    pinocchio::urdf::buildModelFromXML(urdf, model_);

    for (size_t i=0; i<params_.eef_links_.size(); i++){
      unrelevant_joint_inds.clear();
      relevant_joint_inds.clear();
      
      std::string eef_name = params_.eef_links_[i];
      pinocchio::FrameIndex eef_id = model_.getFrameId(eef_name);
      std::cout << "EEF id: " << eef_id << std::endl;
      std::cout << "EEF name: " << eef_name << " id: " << eef_id << std::endl;
      get_relevant_joints_for_frame(model_, eef_name);

      auto reduced_model = pinocchio::buildReducedModel(model_, unrelevant_joint_inds, pinocchio::neutral(model_));
      std::cout << "Reduced Model: " << reduced_model.name << std::endl;
      for (pinocchio::JointIndex idx = 1; idx < reduced_model.joints.size(); ++idx) {
        std::cout << "Joint[" << idx << "] = " << reduced_model.names[idx] << std::endl;
      }
      auto reduced_data = pinocchio::Data(reduced_model);

      // find model joint id -> reduced model joint id mapping
      std::cout << "model_.nq: " << model_.nq << std::endl;
      std::cout << "reduced_model.nq: " << reduced_model.nq << std::endl;
      std::cout << "reduced_model.nv: " << reduced_model.nv << std::endl;
      std::vector<int> joint_mapping;
      for (pinocchio::JointIndex ind = 1; ind < reduced_model.joints.size(); ++ind) {
        auto it = std::find(model_.names.begin(), model_.names.end(), reduced_model.names[ind]);
        if ((it != model_.names.end()) && (reduced_model.joints[ind].nv() > 0)){
          std::cout << "Joint[" << ind << "] = " << reduced_model.names[ind] << " -> " << *it << std::endl;
          auto index = std::distance(model_.names.begin(), it);
          joint_mapping.push_back(static_cast<int>(model_.joints[index].idx_q()));
        } else {
          std::cout << "Joint " << reduced_model.names[ind] << " not found in model." << std::endl;
        }
      }
      std::cout << "Joint mapping: ";
      for (size_t i = 0; i < joint_mapping.size(); ++i) {
        std::cout << i << "->"  << joint_mapping[i] << " ";
      }
      std::cout << std::endl;
      joint_mapping_list_.push_back(joint_mapping);
      
      reduced_models_.push_back(reduced_model);
      reduced_datas_.push_back(reduced_data);
    }
  } catch (const std::exception& e) {
      std::cerr << "Failed to build model: " << e.what() << std::endl;
  }
  //auto logger = get_node()->get_logger();

  RCLCPP_INFO(get_node()->get_logger(), "FeedbackController configured successfully.");
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn FeedbackController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto logger = get_node()->get_logger();

  // order all joints in the storage
  for (const auto & interface : params_.command_interfaces) {
    auto it =
      std::find(command_interface_types_.begin(), command_interface_types_.end(), interface);
    auto index = static_cast<size_t>(std::distance(command_interface_types_.begin(), it));
    if (!controller_interface::get_ordered_interfaces(
        command_interfaces_, command_joint_names_, interface, joint_command_interface_[index]))
    {
      RCLCPP_ERROR(
        logger, "Expected %zu '%s' command interfaces, got %zu.", dof_, interface.c_str(),
        joint_command_interface_[index].size());
      return CallbackReturn::ERROR;
    }
  }
  for (const auto & interface : params_.state_interfaces) {
    auto it =
      std::find(state_interface_types_.begin(), state_interface_types_.end(), interface);
    auto index = static_cast<size_t>(std::distance(state_interface_types_.begin(), it));
    if (!controller_interface::get_ordered_interfaces(
        state_interfaces_, params_.joints, interface, joint_state_interface_[index]))
    {
      RCLCPP_ERROR(
        logger, "Expected %zu '%s' state interfaces, got %zu.", dof_, interface.c_str(),
        joint_state_interface_[index].size());
      return CallbackReturn::ERROR;
    }
  }
  RCLCPP_INFO(get_node()->get_logger(), "FeedbackController activated successfully.");

  running_ = true;
  shutdown_ = false;
  ik_thread_ = std::thread(&FeedbackController::ik_worker, this);


  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn FeedbackController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  for (size_t i = 0; i < n_joints_; ++i) {
    for (size_t j = 0; j < command_interface_types_.size(); ++j) {
      command_interfaces_[i * command_interface_types_.size() + j].set_value(0.0);
    }
  }
  RCLCPP_INFO(get_node()->get_logger(), "FeedbackController deactivated successfully.");

  shutdown_ = true;
  if (ik_thread_.joinable()) {
    ik_thread_.join();
  }
  running_ = false;

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn FeedbackController::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Reset flags and parameters
  dither_switch_ = false;

  // Load Pinocchio Model
  pinocchio::Model model;

  RCLCPP_INFO(get_node()->get_logger(), "FeedbackController cleaned up successfully.");
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn FeedbackController::on_error(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_ERROR(get_node()->get_logger(), "Error occurred in FeedbackController.");
  return CallbackReturn::ERROR;
}

controller_interface::CallbackReturn FeedbackController::on_shutdown(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_node()->get_logger(), "Shutting down FeedbackController.");

  return CallbackReturn::SUCCESS;
}

std::string FeedbackController::formatVector(const std::vector<double> & vec)
{
  std::ostringstream oss;
  for (size_t i = 0; i < vec.size(); ++i) {
    oss << vec[i];
    if (i != vec.size() - 1) {
      oss << ", ";
    }
  }
  return oss.str();
}

// Define callbacks for joint feedback subscription
void FeedbackController::joint_feedback_callback(
  const sensor_msgs::msg::JointState::SharedPtr msg)
{
  joint_feedbacks_name = msg->name;
  // index mapping
  if (joint_feedbacks_index_.size() == 0) {
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      auto it = std::find(joint_feedbacks_name.begin(), joint_feedbacks_name.end(), joint_names_[i]);
      if (it != joint_feedbacks_name.end()) {
        joint_feedbacks_index_.push_back(static_cast<int>(std::distance(joint_feedbacks_name.begin(), it)));
      } else {
        RCLCPP_ERROR(get_node()->get_logger(), "Joint name %s not found in feedback message.", joint_names_[i].c_str());
      }
    }
  }

  joint_feedbacks_ = msg->position;
  joint_feedbacks_time_ = msg->header.stamp;
  // RCLCPP_INFO(
  //   get_node()->get_logger(), "Joint feedbacks: %s", formatVector(joint_feedbacks_).c_str());
  // // Compare time stamp with current time
  // auto current_time = get_node()->now();
  // // calculate the time difference
  // auto time_diff = current_time - msg->header.stamp;
  // RCLCPP_INFO(
  //   get_node()->get_logger(), "Time difference: %f", time_diff.seconds());
  
}
void FeedbackController::eef_feedback_callback(const teleop_msgs::msg::EEFTransforms::SharedPtr msg)
{

  {
    std::lock_guard<std::mutex> lock(ik_mutex_);
    eef_transforms_.clear();
    eef_transforms_.resize(msg->eef_transforms.size());
    for (size_t i = 0; i < msg->eef_transforms.size(); ++i) {
      eef_transforms_[i] = msg->eef_transforms[i];
      //eef_tranfsfomrs_[i] is Float64MultiArray
      //std::cout << "EEF translation " << i << ": " << eef_transforms_[i].data[0] << " " << eef_transforms_[i].data[1] << " " << eef_transforms_[i].data[2] << std::endl;
      //std::cout << "EEF orientation " << i << ": " << eef_transforms_[i].data[3] << " " << eef_transforms_[i].data[4] << " " << eef_transforms_[i].data[5] << std::endl;
    }
  }


}

void FeedbackController::set_joint_state_callback(
  const std::shared_ptr<teleop_msgs::srv::SetBasePose::Request> request,
  std::shared_ptr<teleop_msgs::srv::SetBasePose::Response> response)
{
  base_joint_positions_ = request->joint_state.position;
  RCLCPP_INFO(
    get_node()->get_logger(), "Base joint positions: %s", formatVector(base_joint_positions_).c_str());
  if (base_joint_name_index_.size() == 0) {
    RCLCPP_INFO(get_node()->get_logger(), "Base joint name index is empty. Initializing...");
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      auto it = std::find(request->joint_state.name.begin(), request->joint_state.name.end(), joint_names_[i]);
      if (it != request->joint_state.name.end()) {
        base_joint_name_index_.push_back(static_cast<int>(std::distance(request->joint_state.name.begin(), it)));
        base_joint_enabled_.push_back(1);
      } else {
        RCLCPP_ERROR(get_node()->get_logger(), "Joint name %s not found in feedback message.", joint_names_[i].c_str());
        base_joint_name_index_.push_back(-1);
        base_joint_enabled_.push_back(0);
      }
    }

  }

  response->success = true;
}
} // namespace feedback_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  feedback_controller::FeedbackController,
  controller_interface::ControllerInterface)
