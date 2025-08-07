#ifndef PLANNER_SCHEDULER_TARGET_LANE_PATH_FILTER_H_
#define PLANNER_SCHEDULER_TARGET_LANE_PATH_FILTER_H_

#include <vector>

#include "common/lane_path_info.h"
#include "maps/lane_path.h"
#include "plan/planner_semantic_map_manager.h"
#include "planner.pb.h"
#include "pnp_info.pb.h"
#include "router/navi/route_navi_info.h"
#include "router/route_sections_info.h"
#include "trajectory_point.pb.h"

namespace e2e_noa::planning {

struct PnpTop1History {
  std::deque<double> probabilities;
  std::deque<bool> direction_alignment;
};

std::vector<LanePathInfo> FilterMultipleTargetLanePath(
    const PlannerSemanticMapManager& psmm,
    const ad_e2e::planning::LaneConstPtr& nearest_lane,
    const RouteSectionsInfo& route_sections_info,
    const RouteNaviInfo& route_navi_info,
    const mapping::LanePath& last_target_lane_path,
    const ApolloTrajectoryPointProto& plan_start_point,
    const mapping::LanePath& preferred_lane_path,
    std::vector<LanePathInfo>* mutable_lp_infos);

std::vector<LanePathInfo> FilterMultipleTargetLanePath(
    const PlannerSemanticMapManager& psmm,
    const RouteSectionsInfo& route_sections_info,
    const RouteNaviInfo& route_navi_info,
    const mapping::LanePath& last_target_lane_path,
    const ApolloTrajectoryPointProto& plan_start_point,
    const mapping::LanePath& preferred_lane_path,
    const std::shared_ptr<PNPInfos>& pnp_infos,

    std::vector<LanePathInfo>* mutable_lp_infos,
    const LaneChangeStage& lane_change_stage, bool pre_lc_direction_left,
    PnpTop1History* top1_history);

bool GetMatchLanePathByPnpInfos(
    const absl::flat_hash_set<mapping::ElementId>& pnp_lanes,
    const std::vector<LanePathInfo>& lp_infos, LanePathInfo* match_lp);
}  

#endif
