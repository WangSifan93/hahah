#ifndef E2E_NOA_PLANNING_SPEED_SPEED_FINDER_INPUT
#define E2E_NOA_PLANNING_SPEED_SPEED_FINDER_INPUT

#include <map>
#include <string>
#include <vector>

#include "absl/container/flat_hash_set.h"
#include "behavior.pb.h"
#include "common/path_sl_boundary.h"
#include "common/type_def.h"
#include "descriptor/constraint_manager.h"
#include "object/spacetime_trajectory_manager.h"
#include "perception.pb.h"
#include "plan/discretized_path.h"
#include "plan/ego_history.h"
#include "plan/planner_semantic_map_manager.h"
#include "planner.pb.h"
#include "router/plan_passage.h"
#include "router/route_sections.h"
#include "util/hmi_content_util.h"

namespace e2e_noa::planning {
struct SpeedPlanningInput {
  std::string base_name;
  const Behavior* behavior = nullptr;
  const PlannerSemanticMapManager* psmm = nullptr;
  const SpacetimeTrajectoryManager* traj_mgr = nullptr;
  const ConstraintManager* constraint_mgr = nullptr;
  const std::map<std::string, ConstraintProto::LeadingObjectProto>*
      leading_trajs = nullptr;
  const absl::flat_hash_set<std::string>* follower_set = nullptr;
  const absl::flat_hash_set<std::string>* leader_set = nullptr;
  bool consider_lane_change_gap = true;
  const PlanPassage* plan_passage = nullptr;
  const PathSlBoundary* path_sl_boundary = nullptr;
  const absl::flat_hash_set<std::string>* stalled_objects = nullptr;

  const DiscretizedPath* path = nullptr;

  const std::vector<PathPoint>* st_path_points = nullptr;

  double plan_start_v = 0.0;
  double plan_start_a = 0.0;
  double plan_start_j = 0.0;
  absl::Time plan_time;
  const ObjectsProto* objects_proto = nullptr;
  bool run_act_net_speed_decision = false;
  int plan_id = 0;
  LaneChangeStage lc_stage;
  const LaneChangeStateProto lane_change_state;
  std::string attention_obj_id = "";
  const NudgeObjectInfo* nudge_object_info = nullptr;
  bool is_open_gap = false;
  const EgoHistory* ego_history;
  const uint64_t seq_num = 0;
  std::unordered_map<std::string, std::vector<ad_e2e::planning::AgentState>>
      agent_status_history;
};

}  // namespace e2e_noa::planning

#endif
