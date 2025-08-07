/******************************************************************************
 * Copyright 2023 The zpilot Authors Fuhuanhuan. All Rights Reserved.
 *****************************************************************************/
#pragma once

namespace zark {
namespace planning {
class CosThetaSmootherConfig {
 public:
  // Default constructor
  CosThetaSmootherConfig()
      : weight_cos_included_angle_(10000.0),
        weight_anchor_points_(1.0),
        weight_length_(1.0),
        print_level_(0),
        max_num_of_iterations_(500),
        acceptable_num_of_iterations_(15),
        tol_(1e-8),
        acceptable_tol_(1e-1),
        ipopt_use_automatic_differentiation_(false) {}

  // Getter functions
  double weight_cos_included_angle() const {
    return weight_cos_included_angle_;
  }
  double weight_anchor_points() const { return weight_anchor_points_; }
  double weight_length() const { return weight_length_; }
  int print_level() const { return print_level_; }
  int max_num_of_iterations() const { return max_num_of_iterations_; }
  int acceptable_num_of_iterations() const {
    return acceptable_num_of_iterations_;
  }
  double tol() const { return tol_; }
  double acceptable_tol() const { return acceptable_tol_; }
  bool ipopt_use_automatic_differentiation() const {
    return ipopt_use_automatic_differentiation_;
  }

  // Setter functions
  void set_weight_cos_included_angle(double weight_cos_included_angle) {
    weight_cos_included_angle_ = weight_cos_included_angle;
  }
  void set_weight_anchor_points(double weight_anchor_points) {
    weight_anchor_points_ = weight_anchor_points;
  }
  void set_weight_length(double weight_length) {
    weight_length_ = weight_length;
  }
  void set_print_level(int print_level) { print_level_ = print_level; }
  void set_max_num_of_iterations(int max_num_of_iterations) {
    max_num_of_iterations_ = max_num_of_iterations;
  }
  void set_acceptable_num_of_iterations(int acceptable_num_of_iterations) {
    acceptable_num_of_iterations_ = acceptable_num_of_iterations;
  }
  void set_tol(double tol) { tol_ = tol; }
  void set_acceptable_tol(double acceptable_tol) {
    acceptable_tol_ = acceptable_tol;
  }
  void set_ipopt_use_automatic_differentiation(
      bool ipopt_use_automatic_differentiation) {
    ipopt_use_automatic_differentiation_ = ipopt_use_automatic_differentiation;
  }

 private:
  double weight_cos_included_angle_;
  double weight_anchor_points_;
  double weight_length_;
  int print_level_;
  int max_num_of_iterations_;
  int acceptable_num_of_iterations_;
  double tol_;
  double acceptable_tol_;
  bool ipopt_use_automatic_differentiation_;
};

}  // namespace planning
}  // namespace zark
