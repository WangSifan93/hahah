#ifndef ONBOARD_PLANNER_PLAN_FALLBACK_PLANNER_H_
#define ONBOARD_PLANNER_PLAN_FALLBACK_PLANNER_H_

#include <optional>
#include <string>
#include <vector>

#include "absl/container/flat_hash_set.h"
#include "absl/time/time.h"
#include "async/thread_pool.h"
#include "behavior.pb.h"
#include "common/plan_start_point_info.h"
#include "common/planner_status.h"
#include "constraint.pb.h"
#include "decision_exploration/decision_exploration_input.h"
#include "decision_exploration/smooth_reference_line_result.h"
#include "ego_history.h"
#include "lane_change.pb.h"
#include "maps/lane_path.h"
#include "maps/lane_point.h"
#include "maps/map_def.h"
#include "object/partial_spacetime_object_trajectory.h"
#include "object/planner_object_manager.h"
#include "object/spacetime_trajectory_manager.h"
#include "perception.pb.h"
#include "plan/discretized_path.h"
#include "plan/planner_semantic_map_manager.h"
#include "plan/st_planner_output.h"
#include "planner_params.pb.h"
#include "router/route_sections.h"
#include "router/route_sections_info.h"
#include "scene_understanding.pb.h"
#include "speed_planning.pb.h"
#include "trajectory_point.pb.h"

namespace e2e_noa::planning {

class StBoundaryWithDecision;
struct FallbackPlannerInput {
  const PlannerSemanticMapManager* psmm;
  const PlanStartPointInfo* start_point_info;

  const std::vector<ApolloTrajectoryPointProto>* time_aligned_prev_trajectory;

  const mapping::LanePath* prev_target_lane_path_from_start;
  double prev_length_along_route;
  double prev_max_reach_length;
  const mapping::LanePoint* station_anchor;
  bool prev_smooth_state = false;
  const mapping::LanePath* prev_lane_path_before_lc;

  const PlannerObjectController* obj_mgr;
  const SpacetimeTrajectoryManager* st_traj_mgr;
  const absl::flat_hash_set<std::string>* stalled_objects;
  const SceneOutputProto* scene_reasoning;
  const LaneChangeStateProto* prev_lc_state;

  const DeciderStateProto* pre_decision_state;

  const ad_e2e::planning::TrafficLightStatusMap* traffic_light_status_map =
      nullptr;
  const SmoothedReferenceLineResultMap* smooth_result_map;

  std::optional<double> cruising_speed_limit = std::nullopt;
  const Behavior* behavior = nullptr;
  const ad_e2e::planning::SpeedState* speed_state = nullptr;
  int plan_id = 0;
  const EgoHistory* ego_history;
};

struct FallbackPlannerOutput {
  DeciderStateProto decision_state;
  std::vector<ApolloTrajectoryPointProto> trajectory_points;
  SpacetimeTrajectoryManager filtered_traj_mgr;
  std::vector<PartialSpacetimeObjectTrajectory> considered_st_objects;
  std::optional<TrajectoryEndInfoProto> trajectory_end_info;

  DecisionExplorationOutput decision_exploration;
  DiscretizedPath path;
  std::vector<PathPoint> st_path_points;
  std::vector<StBoundaryWithDecision> st_boundaries_with_decision;
  int tl_stop_interface = 0;
  ad_e2e::planning::SpeedState speed_state;
  EgoFrame curr_ego_frame;
  int plan_id = -2;
};

PlannerStatus RunFallbackPlanner(
    const FallbackPlannerInput& input, const VehicleParamsProto& vehicle_params,
    const SpacetimeConstraintParamsProto& spacetime_constraint_params,
    const DecisionConstraintConfigProto& decision_constraint_config,
    const FallbackPlannerParamsProto& fallback_planner_params,
    FallbackPlannerOutput* output, StPlannerDebug* debug,
    WorkerThreadManager* thread_pool);

}  // namespace e2e_noa::planning

#endif
