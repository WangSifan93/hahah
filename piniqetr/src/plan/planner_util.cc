#include "plan/planner_util.h"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
#include <map>
#include <memory>
#include <ostream>
#include <string>
#include <utility>
#include <vector>

#include "affine_transformation.pb.h"
#include "container/strong_int.h"
#include "maps/semantic_map_defs.h"
#include "maps/semantic_map_util.h"
#include "math/frenet_common.h"
#include "math/geometry/polygon2d.h"
#include "math/geometry/util.h"
#include "math/util.h"
#include "math/vec.h"
#include "plan/discretized_path.h"
#include "plan/planner_defs.h"
#include "util/file_util.h"
#include "util/time_util.h"

namespace e2e_noa {
namespace planning {

namespace {

void UpdateToMinSpeedLimit(std::map<mapping::ElementId, double>* map,
                           mapping::ElementId lane_id, double speed_limit) {
  auto pair_it = map->insert({lane_id, speed_limit});
  if (pair_it.second == false) {
    pair_it.first->second = std::min(pair_it.first->second, speed_limit);
  }
}

}  // namespace

double ComputeLongitudinalJerk(const TrajectoryPoint& traj_point) {
  return traj_point.j() - Cube(traj_point.v()) * Sqr(traj_point.kappa());
}

double ComputeLateralAcceleration(const TrajectoryPoint& traj_point) {
  return Sqr(traj_point.v()) * traj_point.kappa();
}

double ComputeLateralJerk(const TrajectoryPoint& traj_point) {
  return 3.0 * traj_point.v() * traj_point.a() * traj_point.kappa() +
         Sqr(traj_point.v()) * traj_point.psi();
}

bool IsVulnerableRoadUserCategory(ObjectType type) {
  return type == OT_PEDESTRIAN || type == OT_CYCLIST || type == OT_TRICYCLIST ||
         type == OT_MOTORCYCLIST;
}

bool IsStaticObjectType(ObjectType type) {
  return type == OT_UNKNOWN_STATIC || type == OT_VEGETATION || type == OT_FOD ||
         type == OT_BARRIER || type == OT_CONE;
}

std::vector<ApolloTrajectoryPointProto> CreatePastPointsList(
    absl::Time plan_time, const TrajectoryProto& prev_traj, bool reset,
    int max_past_point_num) {
  std::vector<ApolloTrajectoryPointProto> past_points;
  const double curr_plan_time = ToUnixDoubleSeconds(plan_time);
  const double prev_traj_start_time = prev_traj.trajectory_start_timestamp();
  if (prev_traj.trajectory_point().empty() ||
      curr_plan_time >
          prev_traj_start_time +
              prev_traj.trajectory_point().rbegin()->relative_time() ||
      curr_plan_time < prev_traj_start_time || reset) {
    return past_points;
  }
  past_points.reserve(max_past_point_num);
  const int relative_time_index =
      RoundToInt((curr_plan_time - prev_traj_start_time) / kTrajectoryTimeStep);
  CHECK_LT(relative_time_index, prev_traj.trajectory_point_size());
  const double relative_s =
      -prev_traj.trajectory_point(relative_time_index).path_point().s();
  for (int i = max_past_point_num; i > 0; --i) {
    const int index = relative_time_index - i;
    if (index + prev_traj.past_points_size() < 0) {
      continue;
    }
    auto point =
        index < 0 ? prev_traj.past_points(index + prev_traj.past_points_size())
                  : prev_traj.trajectory_point(index);
    point.set_relative_time(-i * kTrajectoryTimeStep);
    point.mutable_path_point()->set_s(point.path_point().s() + relative_s);
    past_points.push_back(point);
  }
  past_points.shrink_to_fit();

  return past_points;
}

ApolloTrajectoryPointProto ComputePlanStartPointAfterReset(
    const std::optional<ApolloTrajectoryPointProto>& prev_reset_planned_point,
    const PoseProto& pose, double front_wheel_angle,
    const SpacetimeConstraintParamsProto& spacetime_constraint_params,
    const VehicleGeometryParamsProto& vehicle_geom_params,
    const VehicleDriveParamsProto& vehicle_drive_params, bool is_forward_task) {
  ApolloTrajectoryPointProto plan_start_point;
  const Vec2d pose_pos(pose.pos_smooth().x(), pose.pos_smooth().y());
  plan_start_point.mutable_path_point()->set_x(pose.pos_smooth().x());
  plan_start_point.mutable_path_point()->set_y(pose.pos_smooth().y());
  plan_start_point.mutable_path_point()->set_s(0.0);
  plan_start_point.mutable_path_point()->set_theta(pose.yaw());
  const double pose_v = pose.vel_body().x();
  const double abs_pose_v = std::abs(pose_v);
  if (is_forward_task) {
    plan_start_point.set_v(std::max(0.0, pose_v));
  } else {
    plan_start_point.set_v(std::min(0.0, pose_v));
  }

  if (prev_reset_planned_point.has_value()) {
    constexpr double kFullStopSpeedThreshold = 0.05;
    const bool full_stop = prev_reset_planned_point->v() == 0.0 &&
                           abs_pose_v < kFullStopSpeedThreshold;
    if (full_stop) {
      plan_start_point.mutable_path_point()->set_kappa(
          prev_reset_planned_point->path_point().kappa());
      plan_start_point.mutable_path_point()->set_lambda(
          prev_reset_planned_point->path_point().lambda());
      plan_start_point.set_a(0.0);
      plan_start_point.set_j(0.0);
      return plan_start_point;
    }
  }

  constexpr double kLowSpeedThreshold = 1.0;
  if (abs_pose_v < kLowSpeedThreshold) {
    const double kappa =
        std::tan(front_wheel_angle) / vehicle_geom_params.wheel_base();
    plan_start_point.mutable_path_point()->set_kappa(kappa);
    plan_start_point.mutable_path_point()->set_lambda(0.0);
    plan_start_point.set_a(0.0);
    plan_start_point.set_j(0.0);
  } else {
    plan_start_point.mutable_path_point()->set_kappa(pose.ar_smooth().z() /
                                                     pose_v);
    plan_start_point.mutable_path_point()->set_lambda(0.0);
    plan_start_point.set_a(std::clamp(
        pose.accel_body().x(), spacetime_constraint_params.max_deceleration(),
        spacetime_constraint_params.max_acceleration()));
    plan_start_point.set_j(0.0);
  }
  return plan_start_point;
}

ApolloTrajectoryPointProto ComputePlanStartPointAfterLateralReset(
    const std::optional<ApolloTrajectoryPointProto>& prev_reset_planned_point,
    const PoseProto& pose, double front_wheel_angle,
    const VehicleGeometryParamsProto& vehicle_geom_params,
    const VehicleDriveParamsProto& vehicle_drive_params) {
  ApolloTrajectoryPointProto plan_start_point;
  const Vec2d pose_pos(pose.pos_smooth().x(), pose.pos_smooth().y());
  plan_start_point.mutable_path_point()->set_x(pose.pos_smooth().x());
  plan_start_point.mutable_path_point()->set_y(pose.pos_smooth().y());
  plan_start_point.mutable_path_point()->set_s(0.0);
  plan_start_point.mutable_path_point()->set_theta(pose.yaw());

  CHECK(prev_reset_planned_point.has_value());
  plan_start_point.set_v(prev_reset_planned_point->v());
  plan_start_point.set_a(prev_reset_planned_point->a());
  plan_start_point.set_j(prev_reset_planned_point->j());

  const double pose_v = pose.vel_body().x();
  const double abs_pose_v = std::abs(pose_v);
  constexpr double kFullStopSpeedThreshold = 0.05;
  const bool full_stop = prev_reset_planned_point->v() == 0.0 &&
                         abs_pose_v < kFullStopSpeedThreshold;
  if (full_stop) {
    plan_start_point.mutable_path_point()->set_kappa(
        prev_reset_planned_point->path_point().kappa());
    plan_start_point.mutable_path_point()->set_lambda(
        prev_reset_planned_point->path_point().lambda());
    return plan_start_point;
  }

  constexpr double kLowSpeedThreshold = 1.0;
  if (abs_pose_v < kLowSpeedThreshold) {
    const double kappa =
        std::tan(front_wheel_angle) / vehicle_geom_params.wheel_base();
    plan_start_point.mutable_path_point()->set_kappa(kappa);
    plan_start_point.mutable_path_point()->set_lambda(0.0);
  } else {
    plan_start_point.mutable_path_point()->set_kappa(pose.ar_smooth().z() /
                                                     pose_v);
    plan_start_point.mutable_path_point()->set_lambda(0.0);
  }
  return plan_start_point;
}

ApolloTrajectoryPointProto
ComputePlanStartPointAfterLongitudinalResetFromPrevTrajectory(
    const TrajectoryProto& prev_traj, const PoseProto& pose,
    double front_wheel_angle,
    const VehicleGeometryParamsProto& vehicle_geom_params,
    const VehicleDriveParamsProto& vehicle_drive_params) {
  ApolloTrajectoryPointProto plan_start_point;

  plan_start_point.set_v(std::max(0.0, pose.vel_body().x()));
  plan_start_point.set_a(pose.accel_body().x());
  plan_start_point.set_j(0.0);

  if (prev_traj.trajectory_point_size() < 2) {
    plan_start_point.mutable_path_point()->set_s(0.0);
    plan_start_point.mutable_path_point()->set_x(pose.pos_smooth().x());
    plan_start_point.mutable_path_point()->set_y(pose.pos_smooth().y());
    plan_start_point.mutable_path_point()->set_theta(pose.yaw());
    constexpr double kLowSpeedThreshold = 1.0;
    if (std::abs(pose.vel_body().x()) < kLowSpeedThreshold) {
      const double kappa =
          std::tan(front_wheel_angle) / vehicle_geom_params.wheel_base();
      plan_start_point.mutable_path_point()->set_kappa(kappa);
      plan_start_point.mutable_path_point()->set_lambda(0.0);
    } else {
      plan_start_point.mutable_path_point()->set_kappa(pose.ar_smooth().z() /
                                                       pose.vel_body().x());
      plan_start_point.mutable_path_point()->set_lambda(0.0);
    }
    return plan_start_point;
  }

  std::vector<PathPoint> prev_traj_path_points;
  prev_traj_path_points.reserve(prev_traj.trajectory_point_size());
  for (int i = 0; i < prev_traj.trajectory_point_size(); ++i) {
    prev_traj_path_points.push_back(prev_traj.trajectory_point(i).path_point());

    prev_traj_path_points.back().set_s(
        prev_traj.trajectory_point(i).path_point().s() -
        prev_traj.trajectory_point(0).path_point().s());
  }
  DiscretizedPath prev_traj_path(std::move(prev_traj_path_points));
  const auto pose_sl = prev_traj_path.XYToSL(
      Vec2d(pose.pos_smooth().x(), pose.pos_smooth().y()));
  *plan_start_point.mutable_path_point() = prev_traj_path.Evaluate(pose_sl.s);
  plan_start_point.mutable_path_point()->set_s(0.0);
  return plan_start_point;
}

SelectorParamsProto LoadSelectorConfigurationFromFile(
    const std::string& file_address) {
  SelectorParamsProto selector_params_proto;
  if (!file_util::TextFileToProto(file_address, &selector_params_proto)) {
    CHECK(false) << "Read auto tuned selector params as text file failed!!!!";
  }
  LOG(INFO) << "New auto tuned selector params are used.";
  return selector_params_proto;
}

}  // namespace planning
}  // namespace e2e_noa
