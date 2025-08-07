/******************************************************************************
 * Copyright 2023 The zpilot Authors Fuhuanhuan. All Rights Reserved.
 *****************************************************************************/
#pragma once
#include <string>
#include <vector>

#include "apps/planning/src/planning_msgs/math/cos_theta_smoother_config.h"
#include "apps/planning/src/planning_msgs/math/fem_pos_deviation_smoother_config.h"

namespace zark {
namespace planning {
// Struct
class QpSplineSmootherConfig {
 public:
  // Default constructor
  QpSplineSmootherConfig() = default;

  // Getter functions
  uint32_t spline_order() const { return spline_order_; }
  double max_spline_length() const { return max_spline_length_; }
  double regularization_weight() const { return regularization_weight_; }
  double second_derivative_weight() const { return second_derivative_weight_; }
  double third_derivative_weight() const { return third_derivative_weight_; }

  // Setter functions
  void set_spline_order(uint32_t spline_order) { spline_order_ = spline_order; }
  void set_max_spline_length(double max_spline_length) {
    max_spline_length_ = max_spline_length;
  }
  void set_regularization_weight(double regularization_weight) {
    regularization_weight_ = regularization_weight;
  }
  void set_second_derivative_weight(double second_derivative_weight) {
    second_derivative_weight_ = second_derivative_weight;
  }
  void set_third_derivative_weight(double third_derivative_weight) {
    third_derivative_weight_ = third_derivative_weight;
  }

 private:
  uint32_t spline_order_ = 5;
  double max_spline_length_ = 10.0;  // 25.0
  double regularization_weight_ = 0.1;
  double second_derivative_weight_ = 0.0;
  double third_derivative_weight_ = 100.0;
};

class SpiralSmootherConfig {
 public:
  // Default constructor
  SpiralSmootherConfig() = default;

  // Getter functions
  double max_deviation() const { return max_deviation_; }
  double piecewise_length() const { return piecewise_length_; }
  uint32_t max_iteration() const { return max_iteration_; }
  double opt_tol() const { return opt_tol_; }
  double opt_acceptable_tol() const { return opt_acceptable_tol_; }
  uint32_t opt_acceptable_iteration() const {
    return opt_acceptable_iteration_;
  }
  double weight_curve_length() const { return weight_curve_length_; }
  double weight_kappa() const { return weight_kappa_; }
  double weight_dkappa() const { return weight_dkappa_; }

  // Setter functions
  void set_max_deviation(double max_deviation) {
    max_deviation_ = max_deviation;
  }
  void set_piecewise_length(double piecewise_length) {
    piecewise_length_ = piecewise_length;
  }
  void set_max_iteration(uint32_t max_iteration) {
    max_iteration_ = max_iteration;
  }
  void set_opt_tol(double opt_tol) { opt_tol_ = opt_tol; }
  void set_opt_acceptable_tol(double opt_acceptable_tol) {
    opt_acceptable_tol_ = opt_acceptable_tol;
  }
  void set_opt_acceptable_iteration(uint32_t opt_acceptable_iteration) {
    opt_acceptable_iteration_ = opt_acceptable_iteration;
  }
  void set_weight_curve_length(double weight_curve_length) {
    weight_curve_length_ = weight_curve_length;
  }
  void set_weight_kappa(double weight_kappa) { weight_kappa_ = weight_kappa; }
  void set_weight_dkappa(double weight_dkappa) {
    weight_dkappa_ = weight_dkappa;
  }

 private:
  double max_deviation_ = 0.1;
  double piecewise_length_ = 10.0;
  uint32_t max_iteration_ = 1000;
  double opt_tol_ = 1.0e-8;
  double opt_acceptable_tol_ = 1e-6;
  uint32_t opt_acceptable_iteration_ = 15;
  double weight_curve_length_ = 1.0;
  double weight_kappa_ = 1.0;
  double weight_dkappa_ = 100.0;
};

enum SmoothingMethod {
  NOT_DEFINED = 0,
  COS_THETA_SMOOTHING = 1,
  FEM_POS_DEVIATION_SMOOTHING = 2
};

class DiscretePointsSmootherConfig {
 public:
  // Default constructor
  DiscretePointsSmootherConfig() = default;

  // Getter functions
  SmoothingMethod smoothing_method() const { return smoothing_method_; }
  const std::vector<CosThetaSmootherConfig>& cos_theta_smoothing() const {
    return cos_theta_smoothing_;
  }
  const FemPosDeviationSmootherConfig& fem_pos_deviation_smoothing() const {
    return fem_pos_deviation_smoothing_;
  }

  // Setter functions
  void set_smoothing_method(SmoothingMethod smoothing_method) {
    smoothing_method_ = smoothing_method;
  }
  void set_cos_theta_smoothing(
      const std::vector<CosThetaSmootherConfig>& cos_theta_smoothing) {
    cos_theta_smoothing_ = cos_theta_smoothing;
  }
  void set_fem_pos_deviation_smoothing(
      const FemPosDeviationSmootherConfig& fem_pos_deviation_smoothing) {
    fem_pos_deviation_smoothing_ = fem_pos_deviation_smoothing;
  }

 private:
  SmoothingMethod smoothing_method_ = FEM_POS_DEVIATION_SMOOTHING;
  std::vector<CosThetaSmootherConfig> cos_theta_smoothing_;
  FemPosDeviationSmootherConfig fem_pos_deviation_smoothing_;
};

class LocalRouteSmootherConfig {
 public:
  // Default constructor
  LocalRouteSmootherConfig() = default;

  // Getter functions
  double max_constraint_interval() const { return max_constraint_interval_; }
  double longitudinal_boundary_bound() const {
    return longitudinal_boundary_bound_;
  }
  double max_lateral_boundary_bound() const {
    return max_lateral_boundary_bound_;
  }
  double min_lateral_boundary_bound() const {
    return min_lateral_boundary_bound_;
  }
  uint32_t num_of_total_points() const { return num_of_total_points_; }
  double curb_shift() const { return curb_shift_; }
  double lateral_buffer() const { return lateral_buffer_; }
  double resolution() const { return resolution_; }
  const QpSplineSmootherConfig& qp_spline() const { return qp_spline_; }
  const SpiralSmootherConfig& spiral() const { return spiral_; }
  const DiscretePointsSmootherConfig& discrete_points() const {
    return discrete_points_;
  }

  // Setter functions
  void set_max_constraint_interval(double max_constraint_interval) {
    max_constraint_interval_ = max_constraint_interval;
  }
  void set_longitudinal_boundary_bound(double longitudinal_boundary_bound) {
    longitudinal_boundary_bound_ = longitudinal_boundary_bound;
  }
  void set_max_lateral_boundary_bound(double max_lateral_boundary_bound) {
    max_lateral_boundary_bound_ = max_lateral_boundary_bound;
  }
  void set_min_lateral_boundary_bound(double min_lateral_boundary_bound) {
    min_lateral_boundary_bound_ = min_lateral_boundary_bound;
  }
  void set_num_of_total_points(uint32_t num_of_total_points) {
    num_of_total_points_ = num_of_total_points;
  }
  void set_curb_shift(double curb_shift) { curb_shift_ = curb_shift; }
  void set_lateral_buffer(double lateral_buffer) {
    lateral_buffer_ = lateral_buffer;
  }
  void set_resolution(double resolution) { resolution_ = resolution; }
  void set_qp_spline(const QpSplineSmootherConfig& qp_spline) {
    qp_spline_ = qp_spline;
  }
  void set_spiral(const SpiralSmootherConfig& spiral) { spiral_ = spiral; }
  void set_discrete_points(
      const DiscretePointsSmootherConfig& discrete_points) {
    discrete_points_ = discrete_points;
  }

 private:
  double max_constraint_interval_ = 0.4;      // 5.0
  double longitudinal_boundary_bound_ = 0.3;  // 1.0
  double max_lateral_boundary_bound_ = 0.2;   // 0.5
  double min_lateral_boundary_bound_ = 0.1;   // 0.25
  uint32_t num_of_total_points_ = 500;
  double curb_shift_ = 0.2;
  double lateral_buffer_ = 0.2;
  double resolution_ = 0.02;
  QpSplineSmootherConfig qp_spline_;
  SpiralSmootherConfig spiral_;
  DiscretePointsSmootherConfig discrete_points_;
};

}  // namespace planning
}  // namespace zark
