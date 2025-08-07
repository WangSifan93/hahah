#ifndef PLANNER_SPEED_IGNORE_DECIDER_H_
#define PLANNER_SPEED_IGNORE_DECIDER_H_

#include <optional>
#include <vector>

#include "common/vehicle_shape.h"
#include "lane_change.pb.h"
#include "object/spacetime_trajectory_manager.h"
#include "plan/discretized_path.h"
#include "plan/planner_semantic_map_manager.h"
#include "router/plan_passage.h"
#include "speed/object_scene_recognition.h"
#include "speed/path_semantic_analyzer.h"
#include "speed/st_boundary_with_decision.h"
#include "speed/vt_speed_limit.h"
#include "speed_planning_params.pb.h"
#include "vehicle.pb.h"

namespace e2e_noa {
namespace planning {

struct IgnoreDescriptorInput {
  const SpeedPlanningParamsProto::IgnoreDeciderParamsProto* params = nullptr;
  const DiscretizedPath* path = nullptr;

  const std::vector<PathPointSemantic>* path_semantics = nullptr;
  const PlannerSemanticMapManager* psmm = nullptr;
  const SpacetimeTrajectoryManager* st_traj_mgr = nullptr;
  const PlanPassage* plan_passage = nullptr;
  const VehicleGeometryParamsProto* vehicle_geometry_params = nullptr;
  const std::vector<VehicleShapeBasePtr>* av_shapes = nullptr;
  double current_v = 0.0;
  double max_v = 0.0;
  double time_step = 0.0;
  int trajectory_steps = 0;
  LaneChangeStage lc_stage;
  std::vector<DrivingProcess> driving_process_seq;
  double current_acc = 0.0;

  bool is_narrow_near_large_vehicle = false;
};

enum Merge_Direction { MERGE_NONE = 0, MERGE_LEFT = 1, MERGE_RIGHT = 2 };

void MakeIgnoreAndPreBrakeDecisionForStBoundaries(
    const IgnoreDescriptorInput& input,
    std::vector<StBoundaryWithDecision>* st_boundaries_wd,
    std::optional<VtSpeedLimit>* speed_limit, bool* const is_left_turn_ignore);
}  // namespace planning
}  // namespace e2e_noa

#endif
