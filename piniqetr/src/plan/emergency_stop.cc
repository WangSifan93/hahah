#include "plan/emergency_stop.h"

#include <algorithm>
#include <cmath>
#include <ostream>
#include <utility>

#include "absl/status/statusor.h"
#include "affine_transformation.pb.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "math/frenet_common.h"
#include "math/frenet_frame.h"
#include "math/geometry/box2d.h"
#include "math/geometry/util.h"
#include "math/util.h"
#include "math/vec.h"
#include "plan/planner_defs.h"
#include "plan/trajectory_util.h"
#include "planner_params.pb.h"
#include "util/path_util.h"
#include "util/status_macros.h"

DEFINE_bool(emergency_stop_canvas_render_collision_area, false,
            "Emergency stop render collision area");

namespace e2e_noa {
namespace planning {
namespace aeb {
namespace {

void InsertLineSideWaypoints(double potential_risk_area_half_width,
                             const TrajectoryPoint& center_point,
                             double steer_angle,
                             std::vector<Vec2d>* right_side_points,
                             std::vector<Vec2d>* left_side_points) {
  CHECK_NOTNULL(right_side_points);
  CHECK_NOTNULL(left_side_points);
  Vec2d tangent = Vec2d::FastUnitFromAngle(
      NormalizeAngle(center_point.theta() - steer_angle));

  const Vec2d right_edge_point =
      center_point.pos() + tangent.Perp() * potential_risk_area_half_width;
  const Vec2d left_edge_point =
      center_point.pos() - tangent.Perp() * potential_risk_area_half_width;
  right_side_points->push_back(right_edge_point);
  left_side_points->push_back(left_edge_point);
}

}  // namespace

std::vector<ApolloTrajectoryPointProto> PlanEmergencyStopTrajectory(
    const ApolloTrajectoryPointProto& plan_start_point,
    double path_s_inc_from_prev, bool reset,
    const std::vector<ApolloTrajectoryPointProto>& prev_traj_points,
    const EmergencyStopParamsProto& emergency_stop_params,
    const SpacetimeConstraintParamsProto& spacetime_constraint_params) {
  VLOG(3) << "Generate emergency stop trajectory.";
  TrajectoryPoint plan_start_traj_point;
  plan_start_traj_point.FromProto(plan_start_point);

  return aeb::GenerateStopTrajectory(
      path_s_inc_from_prev, reset, true,
      spacetime_constraint_params.max_deceleration(),
      spacetime_constraint_params, plan_start_traj_point, prev_traj_points);
}

std::vector<ApolloTrajectoryPointProto> GenerateStopTrajectory(
    double init_s, bool reset, bool forward, double max_deceleration,
    const SpacetimeConstraintParamsProto& spacetime_constraint_params,
    const TrajectoryPoint& plan_start_traj_point,
    const std::vector<ApolloTrajectoryPointProto>& prev_trajectory) {
  ApolloTrajectoryPointProto curr_point;
  plan_start_traj_point.ToProto(&curr_point);
  auto prev_traj = prev_trajectory;

  if (!forward) {
    curr_point.mutable_path_point()->set_theta(
        NormalizeAngle(curr_point.path_point().theta() + M_PI));
    curr_point.mutable_path_point()->set_kappa(
        -curr_point.path_point().kappa());
    curr_point.set_v(-curr_point.v());
    curr_point.set_a(-curr_point.a());
    curr_point.set_j(-curr_point.j());
    curr_point.mutable_path_point()->set_s(-curr_point.path_point().s());
    for (auto& pt : prev_traj) {
      pt.mutable_path_point()->set_theta(
          NormalizeAngle(pt.path_point().theta() + M_PI));
      pt.mutable_path_point()->set_kappa(-pt.path_point().kappa());
      pt.set_v(-pt.v());
      pt.set_a(-pt.a());
      pt.set_j(-pt.j());
      pt.mutable_path_point()->set_s(-pt.path_point().s());
    }
  }

  std::vector<ApolloTrajectoryPointProto> output_trajectory;
  double accumulate_s = 0.0;
  constexpr double kMinSpeed = 1e-6;
  constexpr double kMinDist = 1e-6;
  for (int i = 0; i < kTrajectorySteps; ++i) {
    const double dist =
        std::max(kMinDist, curr_point.v() * kTrajectoryTimeStep +
                               0.5 * max_deceleration * kTrajectoryTimeStep *
                                   kTrajectoryTimeStep);
    accumulate_s += dist;
    const double accumulate_s_at_prev_traj = accumulate_s + init_s;
    ApolloTrajectoryPointProto next_point;
    const auto& curr_path_point = curr_point.path_point();
    if (prev_traj.empty() || reset ||
        accumulate_s_at_prev_traj >= prev_traj.rbegin()->path_point().s()) {
      *next_point.mutable_path_point() =
          GetPathPointAlongCircle(curr_path_point, dist);
    } else {
      next_point =
          QueryApolloTrajectoryPointByS(prev_traj, accumulate_s_at_prev_traj);
    }
    next_point.set_relative_time(curr_point.relative_time() +
                                 kTrajectoryTimeStep);
    next_point.set_v(std::max(
        kMinSpeed, curr_point.v() + kTrajectoryTimeStep * max_deceleration));
    next_point.set_a(std::max(max_deceleration,
                              -1.0 * next_point.v() / kTrajectoryTimeStep));
    curr_point.set_j(
        std::clamp((next_point.a() - curr_point.a()) / kTrajectoryTimeStep,
                   spacetime_constraint_params.max_decel_jerk(),
                   spacetime_constraint_params.max_accel_jerk()));
    output_trajectory.push_back(curr_point);
    curr_point = next_point;
  }

  if (!forward) {
    for (auto& pt : output_trajectory) {
      pt.mutable_path_point()->set_theta(
          NormalizeAngle(pt.path_point().theta() + M_PI));
      pt.mutable_path_point()->set_kappa(-pt.path_point().kappa());
      pt.set_v(-pt.v());
      pt.set_a(-pt.a());
      pt.set_j(-pt.j());
      pt.mutable_path_point()->set_s(-pt.path_point().s());
    }
  }

  return output_trajectory;
}

}  // namespace aeb
}  // namespace planning
}  // namespace e2e_noa
