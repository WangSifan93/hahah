#ifndef ST_PLANNING_ST_PLANNER_OUTPUT
#define ST_PLANNING_ST_PLANNER_OUTPUT

#include <optional>
#include <string>
#include <vector>

#include "absl/container/flat_hash_set.h"
#include "common/type_def.h"
#include "constraint.pb.h"
#include "decision_exploration/decision_exploration_output.h"
#include "descriptor/leading_groups_builder.h"
#include "descriptor/traffic_gap_finder.h"
#include "ego_history.h"
#include "object/partial_spacetime_object_trajectory.h"
#include "object/spacetime_object_trajectory.h"
#include "object/spacetime_planner_object_trajectories.h"
#include "plan/discretized_path.h"
#include "plan/trajectory_point.h"
#include "planner.pb.h"
#include "speed/st_boundary_with_decision.h"
#include "speed_planning.pb.h"
#include "trajectory_initialization.pb.h"
#include "turn_signal.pb.h"
#include "util/hmi_content_util.h"

namespace e2e_noa {
namespace planning {

struct StPlannerDebug {
  SpacetimePlannerObjectTrajectoriesProto st_planner_object_trajectories;
  FilteredTrajectories filtered_prediction_trajectories;
  ConstraintProto decision_constraints;
  InitializationDebugProto initialization_debug_proto;
  TrajectoryOptimizerDebugProto optimizer_debug_proto;

  SpeedPlanningDebugProto speed_planning_debug;
  TrajectoryValidationResultProto traj_validation_result;
};

struct StPlannerOutput {
  DecisionExplorationOutput decision_exploration;
  DiscretizedPath path;
  LeadingGroup leading_trajs;
  PlannerStatusProto::PlannerStatusCode lc_status_code;
  bool is_init_follow_scene = false;
  std::string lc_lead_obj_id = "none";
  absl::flat_hash_set<std::string> follower_set;
  absl::flat_hash_set<std::string> leader_set;
  absl::flat_hash_set<std::string> unsafe_object_ids;
  double follower_max_decel = 0.0;

  std::vector<ApolloTrajectoryPointProto> traj_points;
  std::vector<PathPoint> st_path_points;

  std::optional<double> first_stop_s = std::nullopt;
  std::optional<mapping::ElementId> redlight_lane_id = std::nullopt;

  DeciderStateProto decision_state;

  InitializationStateProto initialization_state;

  TrajectoryOptimizerStateProto trajectory_optimizer_state_proto;

  SpacetimePlannerObjectTrajectoriesProto st_planner_object_trajectories;

  std::vector<PartialSpacetimeObjectTrajectory> considered_st_objects;
  std::optional<TrajectoryEndInfoProto> trajectory_end_info;

  std::optional<std::string> alerted_front_vehicle = std::nullopt;
  std::optional<double> distance_to_traffic_light_stop_line = std::nullopt;
  std::optional<NudgeObjectInfo> nudge_object_info;

  int tl_stop_interface = 0;

  std::vector<StBoundaryWithDecision> st_boundaries_with_decision;
  std::map<std::string, ObjectSlInfo> obj_sl_map;
  std::map<std::string, bool> obj_lead;

  TrafficGapResult traffic_gap;

  ad_e2e::planning::SpeedState speed_state;
  EgoFrame curr_ego_frame;
  int plan_id = -1;
};

}  // namespace planning
}  // namespace e2e_noa

#endif
