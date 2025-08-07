#ifndef ONBOARD_PLANNER_EMERGENCY_STOP_H_
#define ONBOARD_PLANNER_EMERGENCY_STOP_H_

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "chassis.pb.h"
#include "math/geometry/polygon2d.h"
#include "perception.pb.h"
#include "plan/trajectory_point.h"
#include "planner_params.pb.h"
#include "positioning.pb.h"
#include "trajectory_point.pb.h"
#include "vehicle.pb.h"

namespace e2e_noa {
namespace planning {
namespace aeb {

struct EmergencyStopInfo {
  bool emergency_stop = false;
  Polygon2d risk_area;
  std::string object_id;
  e2e_noa::ObjectType object_type;
};

std::vector<ApolloTrajectoryPointProto> PlanEmergencyStopTrajectory(
    const ApolloTrajectoryPointProto& plan_start_point,
    double path_s_inc_from_prev, bool reset,
    const std::vector<ApolloTrajectoryPointProto>& prev_traj_points,
    const EmergencyStopParamsProto& emergency_stop_params,
    const SpacetimeConstraintParamsProto& spacetime_constraint_params);

std::vector<ApolloTrajectoryPointProto> GenerateStopTrajectory(
    double init_s, bool reset, bool forward, double max_deceleration,
    const SpacetimeConstraintParamsProto& spacetime_constraint_params,
    const TrajectoryPoint& plan_start_traj_point,
    const std::vector<ApolloTrajectoryPointProto>& prev_trajectory);

}  // namespace aeb
}  // namespace planning
}  // namespace e2e_noa

#endif
