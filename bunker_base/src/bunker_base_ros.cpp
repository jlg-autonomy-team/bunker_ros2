/*
 * bunker_base_ros.cpp
 *
 * Created on: 3 2, 2022 16:41
 * Description:
 *
 * Copyright (c) 2022 Agilex Robot Pte. Ltd.
 */

#include "bunker_base/bunker_base_ros.hpp"

#include <cmath>
#include <stdexcept>
#include <vector>

#include "bunker_base/bunker_messenger.hpp"
#include "ugv_sdk/utilities/protocol_detector.hpp"

namespace westonrobot {
BunkerBaseRos::BunkerBaseRos(std::string node_name)
    : rclcpp::Node(node_name), keep_running_(false) {
  this->declare_parameter("port_name", rclcpp::ParameterValue("can0"));   

  this->declare_parameter("odom_frame", rclcpp::ParameterValue("odom"));
  this->declare_parameter("base_frame", rclcpp::ParameterValue("base_link"));
  this->declare_parameter("odom_topic_name", rclcpp::ParameterValue("odom"));
  this->declare_parameter("angular_error_scalar", rclcpp::ParameterValue(1.0));
  // Command (multiplies cmd_vel angular) and feedback (divides the reported angular velocity) scalars. Negative means
  // "use angular_error_scalar", which is what the single parameter did before. The fitted covariances were measured with
  // the feedback scalar the robot was running (1.125), so change it only together with a new characterization.
  this->declare_parameter("command_angular_scalar", rclcpp::ParameterValue(-1.0));
  this->declare_parameter("feedback_angular_scalar", rclcpp::ParameterValue(-1.0));
  // Report zero velocity with unknown covariance when no robot state arrived for this long.
  this->declare_parameter("stale_state_timeout_s", rclcpp::ParameterValue(0.5));
  // Twist covariance model of the wheel odometry (see wheel_odom_covariance.hpp). Defaults are the outdoor fit.
  const WheelTwistCovarianceModel model;
  this->declare_parameter("twist_covariance.vx_base", rclcpp::ParameterValue(model.vx_base));
  this->declare_parameter("twist_covariance.vx_per_v2", rclcpp::ParameterValue(model.vx_per_v2));
  this->declare_parameter("twist_covariance.vy_base", rclcpp::ParameterValue(model.vy_base));
  this->declare_parameter("twist_covariance.vy_per_vwz2", rclcpp::ParameterValue(model.vy_per_vwz2));
  this->declare_parameter("twist_covariance.wz_base", rclcpp::ParameterValue(model.wz_base));
  this->declare_parameter("twist_covariance.wz_per_v2", rclcpp::ParameterValue(model.wz_per_v2));
  this->declare_parameter("twist_covariance.wz_drive_per_wz2", rclcpp::ParameterValue(model.wz_drive_per_wz2));
  this->declare_parameter("twist_covariance.wz_pivot_per_wz2", rclcpp::ParameterValue(model.wz_pivot_per_wz2));
  this->declare_parameter("twist_covariance.pivot_speed_threshold", rclcpp::ParameterValue(model.pivot_speed_threshold));
  this->declare_parameter("twist_covariance.unused_variance", rclcpp::ParameterValue(model.unused_variance));
  this->declare_parameter("pose_covariance_diagonal",
                          rclcpp::ParameterValue(std::vector<double>{1.0, 1.0, 1e6, 1e6, 1e6, 1.0}));
  this->declare_parameter("is_bunker_mini", rclcpp::ParameterValue(false));
  this->declare_parameter("simulated_robot", rclcpp::ParameterValue(false));
  this->declare_parameter("control_rate", rclcpp::ParameterValue(50));

  LoadParameters();
}

void BunkerBaseRos::LoadParameters() {
  this->get_parameter_or<std::string>("port_name", port_name_, "can0");

  this->get_parameter_or<std::string>("odom_frame", odom_frame_, "odom");
  this->get_parameter_or<std::string>("base_frame", base_frame_, "base_link");
  this->get_parameter_or<std::string>("odom_topic_name", odom_topic_name_,
                                      "odom");
  this->get_parameter_or<double>("angular_error_scalar", angular_error_scalar_, 1.0);
  this->get_parameter_or<double>("command_angular_scalar", command_angular_scalar_, -1.0);
  this->get_parameter_or<double>("feedback_angular_scalar", feedback_angular_scalar_, -1.0);
  this->get_parameter_or<double>("stale_state_timeout_s", stale_state_timeout_s_, 0.5);
  if (angular_error_scalar_ <= 0.0 || command_angular_scalar_ == 0.0 || feedback_angular_scalar_ == 0.0 ||
      !(stale_state_timeout_s_ > 0.0)) {
    throw std::invalid_argument(
        "angular scalars must be positive (or negative to inherit angular_error_scalar) and "
        "stale_state_timeout_s must be positive");
  }
  this->get_parameter_or<double>("twist_covariance.vx_base", covariance_model_.vx_base, covariance_model_.vx_base);
  this->get_parameter_or<double>("twist_covariance.vx_per_v2", covariance_model_.vx_per_v2, covariance_model_.vx_per_v2);
  this->get_parameter_or<double>("twist_covariance.vy_base", covariance_model_.vy_base, covariance_model_.vy_base);
  this->get_parameter_or<double>("twist_covariance.vy_per_vwz2", covariance_model_.vy_per_vwz2, covariance_model_.vy_per_vwz2);
  this->get_parameter_or<double>("twist_covariance.wz_base", covariance_model_.wz_base, covariance_model_.wz_base);
  this->get_parameter_or<double>("twist_covariance.wz_per_v2", covariance_model_.wz_per_v2, covariance_model_.wz_per_v2);
  this->get_parameter_or<double>("twist_covariance.wz_drive_per_wz2", covariance_model_.wz_drive_per_wz2,
                                 covariance_model_.wz_drive_per_wz2);
  this->get_parameter_or<double>("twist_covariance.wz_pivot_per_wz2", covariance_model_.wz_pivot_per_wz2,
                                 covariance_model_.wz_pivot_per_wz2);
  this->get_parameter_or<double>("twist_covariance.pivot_speed_threshold", covariance_model_.pivot_speed_threshold,
                                 covariance_model_.pivot_speed_threshold);
  this->get_parameter_or<double>("twist_covariance.unused_variance", covariance_model_.unused_variance,
                                 covariance_model_.unused_variance);
  covariance_model_.Validate();
  const std::vector<double> pose_diagonal = this->get_parameter("pose_covariance_diagonal").as_double_array();
  if (pose_diagonal.size() != pose_covariance_diagonal_.size()) {
    throw std::invalid_argument("pose_covariance_diagonal must have 6 entries");
  }
  for (std::size_t i = 0; i < pose_diagonal.size(); ++i) {
    if (!std::isfinite(pose_diagonal[i]) || pose_diagonal[i] < 0.0) {
      throw std::invalid_argument("pose_covariance_diagonal entries must be finite and not negative");
    }
    pose_covariance_diagonal_[i] = pose_diagonal[i];
  }
  this->get_parameter_or<bool>("is_bunker_mini", is_bunker_mini_, false);
  this->get_parameter_or<bool>("simulated_robot", simulated_robot_, false);
  this->get_parameter_or<int>("control_rate", sim_control_rate_, 50);

  std::cout << "Loading parameters: " << std::endl;
  std::cout << "- port name: " << port_name_ << std::endl;
  std::cout << "- odom frame name: " << odom_frame_ << std::endl;
  std::cout << "- base frame name: " << base_frame_ << std::endl;
  std::cout << "- odom topic name: " << odom_topic_name_ << std::endl;
  std::cout << "- angular scalars: error " << angular_error_scalar_ << ", command "
            << command_angular_scalar_ << ", feedback " << feedback_angular_scalar_
            << " (negative = use error scalar)" << std::endl;
  std::cout << "- twist variance: vx " << covariance_model_.vx_base << ", vy " << covariance_model_.vy_base
            << " + " << covariance_model_.vy_per_vwz2 << "*(v*wz)^2, wz " << covariance_model_.wz_base << " + "
            << covariance_model_.wz_drive_per_wz2 << "*wz^2 driving / " << covariance_model_.wz_pivot_per_wz2
            << "*wz^2 pivoting" << std::endl;

  std::cout << "- is bunker mini: " << std::boolalpha << is_bunker_mini_
            << std::endl;


  std::cout << "- simulated robot: " << std::boolalpha << simulated_robot_
            << std::endl;
  std::cout << "- sim control rate: " << sim_control_rate_ << std::endl;
  std::cout << "----------------------------" << std::endl;
}

bool BunkerBaseRos::Initialize() {
  if (is_bunker_mini_) {
    std::cout << "Robot base: Bunker Mini" << std::endl;
  } else {
    std::cout << "Robot base: Bunker" << std::endl;
  }

  ProtocolDetector detector;
  if (detector.Connect(port_name_)) {
    auto proto = detector.DetectProtocolVersion(5);
      if (proto == ProtocolVersion::AGX_V1) {
        std::cout << "Detected protocol: AGX_V1" << std::endl;
        robot_ = std::unique_ptr<BunkerRobot>(
        new BunkerRobot(ProtocolVersion::AGX_V1));
      }
      else if (proto == ProtocolVersion::AGX_V2)
      {
        std::cout << "Detected protocol: AGX_V2" << std::endl;
        robot_ = std::unique_ptr<BunkerRobot>(
        new BunkerRobot(ProtocolVersion::AGX_V2));
      }else {
      std::cout << "Detected protocol: UNKONWN" << std::endl;
      return false;
    }
  } else {
    return false;
  }

  return true;
}

void BunkerBaseRos::Stop() { keep_running_ = false; }

void BunkerBaseRos::Run() {

  std::unique_ptr<BunkerMessenger<BunkerRobot>> messenger =
      std::unique_ptr<BunkerMessenger<BunkerRobot>>(new BunkerMessenger<BunkerRobot>(robot_,this));

  messenger->SetOdometryFrame(odom_frame_);
  messenger->SetBaseFrame(base_frame_);
  messenger->SetOdometryTopicName(odom_topic_name_);
  messenger->SetAngularErrorScalar(angular_error_scalar_);
  if (command_angular_scalar_ > 0.0) messenger->SetCommandAngularScalar(command_angular_scalar_);
  if (feedback_angular_scalar_ > 0.0) messenger->SetFeedbackAngularScalar(feedback_angular_scalar_);
  messenger->SetStaleStateTimeout(stale_state_timeout_s_);
  messenger->SetTwistCovarianceModel(covariance_model_);
  messenger->SetPoseCovarianceDiagonal(pose_covariance_diagonal_);

  if (simulated_robot_) messenger->SetSimulationMode(sim_control_rate_);

  // connect to robot and setup ROS subscription
  if (port_name_.find("can") != std::string::npos) {
    if (robot_->Connect(port_name_)) {
      robot_->EnableCommandedMode();
      std::cout << "Using CAN bus to talk with the robot" << std::endl;
    } else {
      std::cout << "Failed to connect to the robot CAN bus" << std::endl;
      return;
    }
  } else {
    std::cout << "Please check the specified port name is a CAN port"
              << std::endl;
    return;
  }

  // publish robot state at 50Hz while listening to twist commands
  messenger->SetupSubscription();
  keep_running_ = true;
  rclcpp::Rate rate(50);
  while (keep_running_) {
    messenger->PublishStateToROS();
    rclcpp::spin_some(shared_from_this());
    rate.sleep();
  }
  
}
}  // namespace westonrobot
