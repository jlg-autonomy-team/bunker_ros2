/*
 * wheel_odom_covariance.hpp
 *
 * Twist covariance model for the Bunker wheel odometry.
 *
 * The coefficients are fitted from bags of the robot driven by RC, with SLAM and an RTK INS as the reference
 * (tools/odom_covariance/RECORDING.md in the integration repo). Each variance is the mean squared residual of the wheel
 * twist against the reference over 1 s windows, bias (track slip, scale error) included, so it is not a noise figure.
 * Residuals depend on the speed v and the yaw rate wz that the odometry itself reports:
 *
 *   var_vx = vx_base + vx_per_v2 * v^2
 *   var_vy = vy_base + vy_per_vwz2 * (v * wz)^2          lateral slip while turning
 *   var_wz = wz_base + wz_per_v2 * v^2
 *            + (pivoting ? wz_pivot_per_wz2 : wz_drive_per_wz2) * wz^2
 *
 * Pivoting means |v| below pivot_speed_threshold. The defaults are the outdoor (RTK reference) fit; the indoor floor is
 * several times smaller.
 */

#ifndef BUNKER_WHEEL_ODOM_COVARIANCE_HPP
#define BUNKER_WHEEL_ODOM_COVARIANCE_HPP

#include <array>
#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>

namespace westonrobot {

struct WheelTwistCovarianceModel {
  double vx_base = 9.9e-4;
  double vx_per_v2 = 0.0;
  double vy_base = 7.7e-4;
  double vy_per_vwz2 = 0.0177;
  double wz_base = 2.9e-4;
  double wz_per_v2 = 0.0;
  double wz_drive_per_wz2 = 0.045;
  double wz_pivot_per_wz2 = 0.0079;
  double pivot_speed_threshold = 0.1;
  // Variance reported for what a ground vehicle cannot measure (vz, roll and pitch rates), and for every axis while the
  // controller state is stale.
  double unused_variance = 1e3;

  void Validate() const {
    const std::pair<const char *, double> values[] = {
        {"vx_base", vx_base},
        {"vx_per_v2", vx_per_v2},
        {"vy_base", vy_base},
        {"vy_per_vwz2", vy_per_vwz2},
        {"wz_base", wz_base},
        {"wz_per_v2", wz_per_v2},
        {"wz_drive_per_wz2", wz_drive_per_wz2},
        {"wz_pivot_per_wz2", wz_pivot_per_wz2},
        {"pivot_speed_threshold", pivot_speed_threshold},
        {"unused_variance", unused_variance}};
    for (const auto &entry : values) {
      if (!std::isfinite(entry.second) || entry.second < 0.0) {
        throw std::invalid_argument(std::string("twist covariance parameter ") +
                                    entry.first + " must be finite and not negative");
      }
    }
  }

  // Row-major 6x6 covariance of [vx vy vz wx wy wz] for the reported speed and yaw rate.
  std::array<double, 36> Twist(double v, double wz) const {
    std::array<double, 36> covariance{};
    const double vwz = v * wz;
    const bool pivoting = std::fabs(v) < pivot_speed_threshold;
    covariance[0] = vx_base + vx_per_v2 * v * v;
    covariance[7] = vy_base + vy_per_vwz2 * vwz * vwz;
    covariance[14] = unused_variance;
    covariance[21] = unused_variance;
    covariance[28] = unused_variance;
    covariance[35] = wz_base + wz_per_v2 * v * v +
                     (pivoting ? wz_pivot_per_wz2 : wz_drive_per_wz2) * wz * wz;
    return covariance;
  }

  // Nothing is known: stale controller state.
  std::array<double, 36> Unknown() const {
    std::array<double, 36> covariance{};
    for (int i = 0; i < 6; ++i) {
      covariance[i * 7] = unused_variance;
    }
    return covariance;
  }
};

}  // namespace westonrobot

#endif /* BUNKER_WHEEL_ODOM_COVARIANCE_HPP */
