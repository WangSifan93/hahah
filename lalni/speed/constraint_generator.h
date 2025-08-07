#ifndef PLANNER_SPEED_CONSTRAINT_GENERATOR_H_
#define PLANNER_SPEED_CONSTRAINT_GENERATOR_H_

#include <vector>

#include "absl/types/span.h"
#include "common/path_sl_boundary.h"
#include "constraint.pb.h"
#include "object/spacetime_trajectory_manager.h"
#include "plan/discretized_path.h"
#include "router/plan_passage.h"
#include "speed/path_semantic_analyzer.h"
#include "speed/st_boundary_with_decision.h"
#include "speed/st_graph.h"
#include "vehicle.pb.h"

namespace e2e_noa {
namespace planning {

struct SpeedConstraintGeneratorOutput {
  std::vector<ConstraintProto::PathSpeedRegionProto> path_speed_regions;
  std::vector<ConstraintProto::PathStopLineProto> path_stop_lines;
};

SpeedConstraintGeneratorOutput GenerateStationaryCloseObjectConstraints(
    absl::Span<const StBoundaryWithDecision> st_boundaries_with_decision,
    const StGraph& st_graph, const SpacetimeTrajectoryManager& traj_mgr,
    const DiscretizedPath& path, const PlanPassage& plan_passage,
    const PathSlBoundary& path_sl_boundary, double av_speed,
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    const bool is_on_highway,
    const bool is_near_consecutive_stationary_objects);

SpeedConstraintGeneratorOutput GenerateDenseTrafficFlowConstraint(
    absl::Span<const StBoundaryWithDecision> st_boundaries_with_decision,
    const SpacetimeTrajectoryManager& traj_mgr,
    const std::vector<PathPointSemantic>& path_semantics,
    const DiscretizedPath& path, double plan_start_v,
    const VehicleGeometryParamsProto& vehicle_geometry_params);

}  // namespace planning

}  // namespace e2e_noa

#endif
