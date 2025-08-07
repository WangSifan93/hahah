#ifndef ONBOARD_PLANNER_DECISION_LEADING_OBJECT_H_
#define ONBOARD_PLANNER_DECISION_LEADING_OBJECT_H_

#include <string>
#include <vector>

#include "absl/container/flat_hash_set.h"
#include "common/path_sl_boundary.h"
#include "constraint.pb.h"
#include "lane_change.pb.h"
#include "maps/map.h"
#include "math/frenet_common.h"
#include "object/object_history.h"
#include "object/spacetime_trajectory_manager.h"
#include "plan/planner_semantic_map_manager.h"
#include "router/plan_passage.h"
#include "scene_understanding.pb.h"
#include "trajectory_point.pb.h"
#include "vehicle.pb.h"

namespace e2e_noa {
namespace planning {

std::vector<ConstraintProto::LeadingObjectProto> FindLeadingObjects(
    const ad_e2e::planning::TrafficLightStatusMap& tl_status_map,
    const PlannerSemanticMapManager& psmm, const PlanPassage& passage,
    const PathSlBoundary& sl_boundary, LaneChangeStage lc_stage,
    const SceneOutputProto& scene_reasoning,
    const SpacetimeTrajectoryManager& st_traj_mgr,
    const absl::flat_hash_set<std::string>& stalled_objects,
    const ApolloTrajectoryPointProto& plan_start_point,
    const e2e_noa::VehicleGeometryParamsProto& vehicle_geometry_params,
    const FrenetBox& ego_frenet_box, bool borrow_lane_boundary,
    const ObjectHistoryController* obs_his_manager,
    std::map<std::string, bool>& obj_lead,
    const NudgeObjectInfo* nudge_object_info, const bool& is_lane_change,
    const double& nearest_stop_s, bool* is_first_lead, bool& must_borrow,
    int plan_id);
}
}  // namespace e2e_noa

#endif
