/******************************************************************************
 * Copyright 2023 The zpilot Authors Fuhuanhuan. All Rights Reserved.
 *****************************************************************************/
#pragma once

namespace zark {
namespace planning {
class FemPosDeviationSmootherConfig {
 public:
  // Default constructor
  FemPosDeviationSmootherConfig()
      : weight_fem_pos_deviation_(1.0e9),
        weight_ref_deviation_(1.0e6),
        weight_path_length_(0.3),
        apply_curvature_constraint_(false),
        weight_curvature_constraint_slack_var_(1.0e2),
        curvature_constraint_(0.2),
        use_sqp_(false),
        sqp_ftol_(0.05),
        sqp_ctol_(0.08),
        sqp_pen_max_iter_(10),
        sqp_sub_max_iter_(20),
        max_iter_(500),
        time_limit_(0.0),
        verbose_(false),
        scaled_termination_(true),
        warm_start_(true),
        print_level_(0),
        max_num_of_iterations_(500),
        acceptable_num_of_iterations_(15),
        tol_(0.0),
        acceptable_tol_(1e-1) {}

  // Getter functions
  double weight_fem_pos_deviation() const { return weight_fem_pos_deviation_; }
  double weight_ref_deviation() const { return weight_ref_deviation_; }
  double weight_last_ref_deviation() const {
    return weight_last_ref_deviation_;
  }
  double weight_path_length() const { return weight_path_length_; }
  bool apply_curvature_constraint() const {
    return apply_curvature_constraint_;
  }
  double weight_curvature_constraint_slack_var() const {
    return weight_curvature_constraint_slack_var_;
  }
  double curvature_constraint() const { return curvature_constraint_; }
  bool use_sqp() const { return use_sqp_; }
  double sqp_ftol() const { return sqp_ftol_; }
  double sqp_ctol() const { return sqp_ctol_; }
  int sqp_pen_max_iter() const { return sqp_pen_max_iter_; }
  int sqp_sub_max_iter() const { return sqp_sub_max_iter_; }
  int max_iter() const { return max_iter_; }
  double time_limit() const { return time_limit_; }
  bool verbose() const { return verbose_; }
  bool scaled_termination() const { return scaled_termination_; }
  bool warm_start() const { return warm_start_; }
  int print_level() const { return print_level_; }
  int max_num_of_iterations() const { return max_num_of_iterations_; }
  int acceptable_num_of_iterations() const {
    return acceptable_num_of_iterations_;
  }
  double tol() const { return tol_; }
  double acceptable_tol() const { return acceptable_tol_; }

  // Setter functions
  void set_weight_fem_pos_deviation(double weight_fem_pos_deviation) {
    weight_fem_pos_deviation_ = weight_fem_pos_deviation;
  }
  void set_weight_ref_deviation(double weight_ref_deviation) {
    weight_ref_deviation_ = weight_ref_deviation;
  }
  void set_last_weight_ref_deviation(double weight_last_ref_deviation) {
    weight_last_ref_deviation_ = weight_last_ref_deviation;
  }
  void set_weight_path_length(double weight_path_length) {
    weight_path_length_ = weight_path_length;
  }
  void set_apply_curvature_constraint(bool apply_curvature_constraint) {
    apply_curvature_constraint_ = apply_curvature_constraint;
  }
  void set_weight_curvature_constraint_slack_var(
      double weight_curvature_constraint_slack_var) {
    weight_curvature_constraint_slack_var_ =
        weight_curvature_constraint_slack_var;
  }
  void set_curvature_constraint(double curvature_constraint) {
    curvature_constraint_ = curvature_constraint;
  }
  void set_use_sqp(bool use_sqp) { use_sqp_ = use_sqp; }
  void set_sqp_ftol(double sqp_ftol) { sqp_ftol_ = sqp_ftol; }
  void set_sqp_ctol(double sqp_ctol) { sqp_ctol_ = sqp_ctol; }
  void set_sqp_pen_max_iter(int sqp_pen_max_iter) {
    sqp_pen_max_iter_ = sqp_pen_max_iter;
  }
  void set_sqp_sub_max_iter(int sqp_sub_max_iter) {
    sqp_sub_max_iter_ = sqp_sub_max_iter;
  }
  void set_max_iter(int max_iter) { max_iter_ = max_iter; }
  void set_time_limit(double time_limit) { time_limit_ = time_limit; }
  void set_verbose(bool verbose) { verbose_ = verbose; }
  void set_scaled_termination(bool scaled_termination) {
    scaled_termination_ = scaled_termination;
  }
  void set_warm_start(bool warm_start) { warm_start_ = warm_start; }
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

 private:
  double weight_fem_pos_deviation_;
  double weight_ref_deviation_;
  double weight_last_ref_deviation_;
  double weight_path_length_;
  bool apply_curvature_constraint_;
  double weight_curvature_constraint_slack_var_;
  double curvature_constraint_;
  bool use_sqp_;
  double sqp_ftol_;
  double sqp_ctol_;
  int sqp_pen_max_iter_;
  int sqp_sub_max_iter_;
  int max_iter_;
  double time_limit_;
  bool verbose_;
  bool scaled_termination_;
  bool warm_start_;
  int print_level_;
  int max_num_of_iterations_;
  int acceptable_num_of_iterations_;
  double tol_;
  double acceptable_tol_;
};

}  // namespace planning
}  // namespace zark
