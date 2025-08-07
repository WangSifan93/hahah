#ifndef PLANNER_MATH_PIECEWISE_JERK_QP_SOLVER_PIECEWISE_JERK_QP_SOLVER_H_
#define PLANNER_MATH_PIECEWISE_JERK_QP_SOLVER_PIECEWISE_JERK_QP_SOLVER_H_

#include <glog/logging.h>

#include <array>
#include <vector>

#include "absl/status/status.h"
#include "absl/types/span.h"
#include "osqp.h"

namespace e2e_noa {

enum class BoundType {
  LOWER_BOUND = 0,
  UPPER_BOUND = 1,
};

class PiecewiseJerkQpSolver {
 public:
  PiecewiseJerkQpSolver(int num_knots, double delta_t,
                        int x_lower_soft_constraint_type_num,
                        int x_upper_soft_constraint_type_num,
                        int dx_lower_soft_constraint_type_num,
                        int dx_upper_soft_constraint_type_num,
                        int ddx_lower_soft_constraint_type_num,
                        int ddx_upper_soft_constraint_type_num,
                        int dddx_lower_soft_constraint_type_num,
                        int dddx_upper_soft_constraint_type_num);

  PiecewiseJerkQpSolver(int num_knots, double delta_t);

  virtual ~PiecewiseJerkQpSolver() = default;

  void ReInit();

  template <int Order,
            typename std::enable_if_t<(Order >= 0 && Order <= 3), bool> = 0>
  void AddNthOrderReferencePointKernel(int index, double ref_val,
                                       double weight) {
    CHECK_LT(index, num_knots_);

    if constexpr (Order == 0) {
      weight_.x_w[index] += 2.0 * weight;
    } else if constexpr (Order == 1) {
      weight_.dx_w[index] += 2.0 * weight;
    } else if constexpr (Order == 2) {
      weight_.ddx_w[index] += 2.0 * weight;
    } else {
      weight_.dddx_w[index] += 2.0 * weight;
    }

    offset_[index + num_knots_ * Order] -= 2.0 * weight * ref_val;
  }

  template <int Order,
            typename std::enable_if_t<(Order >= 0 && Order <= 3), bool> = 0>
  void AddNthOrderWeight(int index, double weight) {
    CHECK_LT(index, num_knots_);

    if constexpr (Order == 0) {
      weight_.x_w[index] += 2.0 * weight;
    } else if constexpr (Order == 1) {
      weight_.dx_w[index] += 2.0 * weight;
    } else if constexpr (Order == 2) {
      weight_.ddx_w[index] += 2.0 * weight;
    } else {
      weight_.dddx_w[index] += 2.0 * weight;
    }
  }

  void AddFirstOrderDerivativeKernel(double weight);

  void AddSecondOrderDerivativeKernel(double weight);

  void AddThirdOrderDerivativeKernel(double weight);

  void AddRegularization(double delta);

  void AddZeroOrderSlackVarKernel(BoundType bound_type,
                                  absl::Span<const int> slack_indices,
                                  absl::Span<const double> weights);

  void AddFirstOrderSlackVarKernel(BoundType bound_type,
                                   absl::Span<const int> slack_indices,
                                   absl::Span<const double> weights);

  void AddSecondOrderSlackVarKernel(BoundType bound_type,
                                    absl::Span<const int> slack_indices,
                                    absl::Span<const double> weights);

  void AddThirdOrderSlackVarKernel(BoundType bound_type,
                                   absl::Span<const int> slack_indices,
                                   absl::Span<const double> weights);

  void AddNthOrderEqualityConstraint(int order,
                                     absl::Span<const int> index_list,
                                     absl::Span<const double> coeff,
                                     double value);

  void AddNthOrderInequalityConstraint(int order,
                                       absl::Span<const int> index_list,
                                       absl::Span<const double> coeff,
                                       double lower_bound, double upper_bound);

  void AddInequalityConstraint(absl::Span<const int> index_list,
                               absl::Span<const double> coeff,
                               double lower_bound, double upper_bound);

  void AddZeroOrderInequalityConstraintWithOneSideSlack(
      absl::Span<const int> index_list, absl::Span<const double> coeff,
      int slack_term_index, BoundType bound_type, double bound);

  void AddFirstOrderInequalityConstraintWithOneSideSlack(
      absl::Span<const int> index_list, absl::Span<const double> coeff,
      int slack_term_index, BoundType bound_type, double bound);

  void AddSecondOrderInequalityConstraintWithOneSideSlack(
      absl::Span<const int> index_list, absl::Span<const double> coeff,
      int slack_term_index, BoundType bound_type, double bound);

  void AddThirdOrderInequalityConstraintWithOneSideSlack(
      absl::Span<const int> index_list, absl::Span<const double> coeff,
      int slack_term_index, BoundType bound_type, double bound);

  void SetInitConditions(const std::array<double, 3>& x_init);

  absl::Status Optimize();

  std::array<double, 4> Evaluate(double t);

  int osqp_iter() const { return osqp_iter_; }

  int status_val() const { return status_val_; }

 private:
  void CalculateKernel(std::vector<c_float>* P_data,
                       std::vector<c_int>* P_indices,
                       std::vector<c_int>* P_indptr);

  void CalculateOffset(std::vector<c_float>* q);

  void CalculateAffineConstraint(std::vector<c_float>* A_data,
                                 std::vector<c_int>* A_indices,
                                 std::vector<c_int>* A_indptr,
                                 std::vector<c_float>* lower_bounds,
                                 std::vector<c_float>* upper_bounds);

  absl::Status OptimizeWithOsqp(
      int kernel_dim, int num_affine_constraint, std::vector<c_float>& P_data,
      std::vector<c_int>& P_indices, std::vector<c_int>& P_indptr,
      std::vector<c_float>& A_data, std::vector<c_int>& A_indices,
      std::vector<c_int>& A_indptr, std::vector<c_float>& lower_bounds,
      std::vector<c_float>& upper_bounds, std::vector<c_float>& q,
      OSQPData* data, OSQPWorkspace** work, OSQPSettings* settings);

  void GetOptimum(const OSQPWorkspace* work);

  void AddIntrinsicConstraint();

  void AddSlackPositiveConstraint();

 private:
  class Segment {
   public:
    Segment(double x, double dx, double ddx, double dddx, double t_range)
        : x0_(x), dx0_(dx), ddx0_(ddx), dddx_(dddx), t_range_(t_range) {}

    double s_range() const { return t_range_; }

    std::array<double, 4> Evaluate(double t) const {
      const double ddx = ddx0_ + dddx_ * t;
      const double dx = dx0_ + ddx0_ * t + 0.5 * dddx_ * t * t;
      const double x =
          x0_ + dx0_ * t + 0.5 * ddx0_ * t * t + 0.166667 * dddx_ * t * t * t;
      return std::array<double, 4>{x, dx, ddx, dddx_};
    }

   private:
    double x0_ = 0.0;
    double dx0_ = 0.0;
    double ddx0_ = 0.0;
    double dddx_ = 0.0;
    double t_range_ = 0.0;
  };

  struct {
    std::vector<double> x_w;
    std::vector<double> dx_w;
    std::vector<double> ddx_w;
    std::vector<double> dddx_w;
    std::vector<double> x_lower_slack_w;
    std::vector<double> x_upper_slack_w;
    std::vector<double> dx_lower_slack_w;
    std::vector<double> dx_upper_slack_w;
    std::vector<double> ddx_lower_slack_w;
    std::vector<double> ddx_upper_slack_w;
    std::vector<double> dddx_lower_slack_w;
    std::vector<double> dddx_upper_slack_w;
  } weight_;

 private:
  int num_knots_ = 0;

  int x_lower_soft_constraint_type_num_ = 0;
  int x_upper_soft_constraint_type_num_ = 0;
  int dx_lower_soft_constraint_type_num_ = 0;
  int dx_upper_soft_constraint_type_num_ = 0;
  int ddx_lower_soft_constraint_type_num_ = 0;
  int ddx_upper_soft_constraint_type_num_ = 0;
  int dddx_lower_soft_constraint_type_num_ = 0;
  int dddx_upper_soft_constraint_type_num_ = 0;

  int total_num_of_slack_ = 0;
  int total_num_of_var_ = 0;

  int x_lower_slack_start_idx = 0;
  int x_upper_slack_start_idx = 0;
  int dx_lower_slack_start_idx = 0;
  int dx_upper_slack_start_idx = 0;
  int ddx_lower_slack_start_idx = 0;
  int ddx_upper_slack_start_idx = 0;
  int dddx_lower_slack_start_idx = 0;
  int dddx_upper_slack_start_idx = 0;

  std::vector<Segment> segments_;

  std::array<double, 3> x_init_ = {0.0};

  std::vector<std::vector<c_float>> constraints_data_cols_;
  std::vector<std::vector<c_float>> constraints_indice_cols_;
  std::vector<c_float> lower_bounds_;
  std::vector<c_float> upper_bounds_;
  int curr_num_constraints_ = 0;

  std::vector<c_float> offset_;

  double delta_t_ = 1.0;
  double delta_t_sq_ = 1.0;
  double delta_t_tri_ = 1.0;

  OSQPWorkspace* work_ = nullptr;

  int osqp_iter_ = -1;
  int status_val_ = 0;
};

}  // namespace e2e_noa

#endif
