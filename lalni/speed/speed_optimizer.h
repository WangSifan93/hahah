#ifndef E2E_NOA_PLANNING_SPEED_SPEED_OPTIMIZER
#define E2E_NOA_PLANNING_SPEED_SPEED_OPTIMIZER

#include <map>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "absl/status/status.h"
#include "absl/strings/string_view.h"
#include "math/piecewise_linear_function.h"
#include "planner_params.pb.h"
#include "speed/solver/piecewise_jerk_qp_solver/piecewise_jerk_qp_solver.h"
#include "speed/speed_bound.h"
#include "speed/speed_optimizer_constraint_manager.h"
#include "speed/speed_optimizer_object.h"
#include "speed/speed_optimizer_object_manager.h"
#include "speed/speed_vector.h"
#include "speed_planning.pb.h"
#include "speed_planning_params.pb.h"

namespace e2e_noa::planning {

class SpeedOptimizer {
 public:
  SpeedOptimizer(
      std::string_view base_name, double init_v, double init_a,
      const SpacetimeConstraintParamsProto* spacetime_constraint_params,
      const SpeedPlanningParamsProto* speed_planning_params, double path_length,
      double default_speed_limit, double delta_t);

  absl::Status Optimize(
      const SpeedOptimizerObjectManager& opt_obj_mgr,
      const SpeedBoundMapType& speed_bound_map,
      const SpeedVector& reference_speed,
      const std::vector<std::pair<double, double>>& accel_bound,
      SpeedVector* optimized_speed,
      SpeedPlanningDebugProto* speed_planning_debug_proto);

 private:
  bool AddConstarints(double init_v, double init_a,
                      const SpeedOptimizerConstraintManager& constraint_mgr);

  bool AddKernel(const SpeedOptimizerConstraintManager& constraint_mgr,
                 const SpeedVector& reference_speed);

  absl::Status Solve();

  void MakeSConstraint(int knot_idx,
                       const SpeedOptimizerObjectManager& opt_obj_mgr,
                       const std::optional<double>& min_stationary_upper_bound,
                       SpeedOptimizerConstraintManager* constraint_mgr,
                       SpeedPlanningDebugProto* speed_planning_debug_proto);

  void MakeMovingObjectFollowConstraint(
      int knot_idx, const ObjectOverlapState& overlap_state,
      absl::string_view id, double follow_standstill,
      const std::optional<double>& min_stationary_upper_bound,
      double time_att_gain,
      const PiecewiseLinearFunction<double>&
          comfortable_brake_bound_violation_rel_speed_plf,
      SpeedOptimizerConstraintManager* constraint_mgr,
      SpeedPlanningDebugProto* speed_planning_debug_proto);

  void MakeStationaryObjectFollowConstraint(
      int knot_idx, const ObjectOverlapState& overlap_state,
      absl::string_view id, double time_gain,
      SpeedOptimizerConstraintManager* constraint_mgr,
      SpeedPlanningDebugProto* speed_planning_debug_proto);

  void MakeMovingObjectLeadConstraint(
      int knot_idx, const ObjectOverlapState& overlap_state,
      absl::string_view id, double time_att_gain,
      SpeedOptimizerConstraintManager* constraint_mgr,
      SpeedPlanningDebugProto* speed_planning_debug_proto);

  void MakeSpeedConstraint(
      int knot_idx,
      const std::map<SpeedLimitTypeProto::Type,
                     std::vector<SpeedBoundWithInfo>>& speed_limit_map,
      SpeedOptimizerConstraintManager* constraint_mgr,
      SpeedPlanningDebugProto* speed_planning_debug_proto) const;

  void MakeAccelConstraint(
      int knot_idx, double reference_speed,
      const std::vector<std::pair<double, double>>& accel_bound,
      SpeedOptimizerConstraintManager* constraint_mgr,
      SpeedPlanningDebugProto* speed_planning_debug) const;

  void MakeJerkConstraint(
      SpeedOptimizerConstraintManager* constraint_mgr) const;

  std::optional<double> ComputeMinStationaryUpperBound(
      const SpeedOptimizerObjectManager& opt_obj_mgr);

  void FilterQpSpeedPoints(std::vector<SpeedPoint>& qp_speed_points);

 private:
  std::string base_name_;
  double init_v_;
  double init_a_;

  const SpacetimeConstraintParamsProto* spacetime_constraint_params_;
  const SpeedPlanningParamsProto* speed_planning_params_;
  const SpeedPlanningParamsProto::SpeedOptimizerParamsProto*
      speed_optimizer_params_;

  double delta_t_ = 0.0;
  int knot_num_ = 0;
  double total_time_ = 0.0;
  double allowed_max_speed_ = 0.0;
  double max_path_length_ = 0.0;
  PiecewiseLinearFunction<double> probability_gain_plf_;
  PiecewiseLinearFunction<double> accel_weight_gain_plf_;
  std::vector<double> piecewise_time_range_;
  std::vector<double> moving_obj_time_gain_;
  std::vector<double> static_obj_time_gain_;
  PiecewiseLinearFunction<double> accel_lower_bound_plf_;
  PiecewiseLinearFunction<double> ref_speed_time_gain_;

  SpeedVector comfortable_brake_speed_;
  SpeedVector max_brake_speed_;

  std::unique_ptr<PiecewiseJerkQpSolver> solver_;

  bool use_soft_jerk_constraint_ = false;
};

}  // namespace e2e_noa::planning

#endif
