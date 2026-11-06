/*
 * bunker_base_ros.hpp
 *
 * Created on: 3 2, 2022 16:41
 * Description:
 *
 * Copyright (c) 2022 Agilex Robot Pte. Ltd.
 */

#ifndef BUNKER_BASE_ROS_HPP
#define BUNKER_BASE_ROS_HPP

#include <array>
#include <atomic>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "bunker_base/wheel_odom_covariance.hpp"
#include "ugv_sdk/mobile_robot/bunker_robot.hpp"

namespace westonrobot {
class BunkerBaseRos : public rclcpp::Node {
 public:
  BunkerBaseRos(std::string node_name);

  bool Initialize();
  void Run();
  void Stop();

 private:
  std::string port_name_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string odom_topic_name_;
  double angular_error_scalar_;
  // Negative: use angular_error_scalar_ for that direction.
  double command_angular_scalar_ = -1.0;
  double feedback_angular_scalar_ = -1.0;
  double stale_state_timeout_s_ = 0.5;
  WheelTwistCovarianceModel covariance_model_;
  std::array<double, 6> pose_covariance_diagonal_{{1.0, 1.0, 1e6, 1e6, 1e6, 1.0}};
  bool is_bunker_mini_ = false;
  // bool is_omni_wheel_ = false;

  bool simulated_robot_ = false;
  int sim_control_rate_ = 50;


  std::shared_ptr<BunkerRobot> robot_;
  // std::shared_ptr<BunkerMiniOmniRobot> omni_robot_;

  std::atomic<bool> keep_running_;

  void LoadParameters();
};
}  // namespace westonrobot

#endif /* SCOUT_BASE_ROS_HPP */
