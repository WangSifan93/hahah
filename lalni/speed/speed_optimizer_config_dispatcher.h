#ifndef PLANNER_SPEED_SPEED_OPTIMIZER_CONFIG_DISPATCHER_H_
#define PLANNER_SPEED_SPEED_OPTIMIZER_CONFIG_DISPATCHER_H_

#include "absl/types/span.h"
#include "common/path_approx.h"
#include "math/segment_matcher/segment_matcher_kdtree.h"
#include "object/spacetime_trajectory_manager.h"
#include "speed/st_boundary_with_decision.h"
#include "speed_planning_params.pb.h"
#include "trajectory_point.pb.h"
#include "vehicle.pb.h"

namespace e2e_noa::planning {

std::optional<SpeedPlanningParamsProto::SpeedOptimizerParamsProto>
DispatchSpeedOptimizerConfig(
    absl::Span<const StBoundaryWithDecision> st_boundaries_with_decision,
    const SpacetimeTrajectoryManager& traj_mgr, const PathApprox& path_approx,
    const KdtreeSegmentMatcher& path_kd_tree, double av_radius,
    double path_step_length, int path_last_index, double current_v,
    const PathPoint& current_path_point,
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    const SpeedPlanningParamsProto::SpeedOptimizerParamsProto&
        raw_speed_optimizer_params,
    const SpeedPlanningParamsProto::SpeedOptimizerConfigDispatcherParams&
        config_dispatcher_params);

}

#endif
