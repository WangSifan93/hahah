#ifndef PLANNER_SPEED_ST_OVERLAP_ANALYZER_H_
#define PLANNER_SPEED_ST_OVERLAP_ANALYZER_H_

#include <vector>

#include "absl/types/span.h"
#include "common/vehicle_shape.h"
#include "lane_change.pb.h"
#include "object/spacetime_trajectory_manager.h"
#include "plan/discretized_path.h"
#include "plan/planner_semantic_map_manager.h"
#include "router/plan_passage.h"
#include "speed/path_semantic_analyzer.h"
#include "speed/st_boundary.h"
#include "vehicle.pb.h"

namespace e2e_noa {
namespace planning {

struct OverlapSourcePriority {
  StOverlapMetaProto::OverlapSource source = StOverlapMetaProto::UNKNOWN_SOURCE;
  StOverlapMetaProto::OverlapPriority priority =
      StOverlapMetaProto::UNKNOWN_PRIORITY;
  std::string priority_reason;
  std::optional<double> time_to_lc_complete;
  std::optional<bool> is_making_u_turn;
  std::optional<bool> is_merging_straight_lane;
  std::optional<bool> is_crossing_straight_lane;
  std::optional<bool> is_unprotected_left_turn;
  std::optional<e2e_noa::planning::TurnType> obj_lane_direction;
};

bool IsAnalyzableStBoundary(const StBoundaryRef& st_boundary);

void AnalyzeStOverlaps(
    const DiscretizedPath& path,
    absl::Span<const PathPointSemantic> path_semantics,
    const PlannerSemanticMapManager& psmm,
    const SpacetimeTrajectoryManager& st_traj_mgr,
    const PlanPassage& plan_passage,
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    const LaneChangeStage& lc_stage,
    const std::vector<VehicleShapeBasePtr>& av_shapes, double init_v,
    std::vector<StBoundaryRef>* st_boundaries);

StOverlapMetaProto::OverlapPattern AnalyzeOverlapPattern(
    const StBoundaryRef& st_boundary, const SpacetimeObjectTrajectory& st_traj,
    const DiscretizedPath& path,
    const VehicleGeometryParamsProto& vehicle_geometry_params);
bool handle_intersection(
    const StBoundaryRef& st_boundary,
    absl::Span<const PathPointSemantic> path_semantics,
    const PlannerSemanticMapManager& psmm,
    const SpacetimeObjectTrajectory& st_traj, const DiscretizedPath& path,
    const std::vector<mapping::ElementId>& fo_av_lane_id_list,
    const OverlapInfo& fo_info, const int& av_mean_idx,
    OverlapSourcePriority& result, const LaneChangeStage& lc_stage);

}  // namespace planning
}  // namespace e2e_noa

#endif
