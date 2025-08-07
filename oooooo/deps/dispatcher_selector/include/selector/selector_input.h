#ifndef PLANNER_SELECTOR_SELECTOR_INPUT_H_
#define PLANNER_SELECTOR_SELECTOR_INPUT_H_

#include <string>
#include <vector>

#include "absl/container/flat_hash_set.h"
#include "behavior.pb.h"
#include "driving_style.pb.h"
#include "maps/lane_sequence.h"
#include "plan/planner_semantic_map_manager.h"
#include "planner_params.pb.h"
#include "prediction.pb.h"
#include "router/navi/route_navi_info.h"
#include "router/route_sections_info.h"
#include "selector/selector_state.h"
#include "vehicle.pb.h"

namespace e2e_noa::planning {
using LaneSequencePtr = ad_e2e::planning::LaneSequencePtr;
struct SelectorFlags {
  bool planner_enable_selector_scoring_net = false;
  bool planner_dumping_selector_features = false;
  int planner_begin_lane_change_frame = 1;
  int planner_begin_signal_frame = 1;
  bool planner_enable_lane_change_in_intersection = true;
  bool planner_enable_cross_solid_boundary = true;
  LaneChangeStyle planner_lane_change_style = LaneChangeStyle::LC_STYLE_NORMAL;
  bool planner_need_to_lane_change_confirmation = false;
  bool planner_is_bus_model = false;
};

struct SelectorInput {
  const PlannerSemanticMapManager* psmm;
  const RouteSectionsInfo* sections_info;
  const mapping::LanePath* prev_lane_path_from_current;
  const e2e_noa::LaneChangeStage& pre_lc_stage;
  const std::vector<ApolloTrajectoryPointProto>* prev_traj;
  const SpacetimeConstraintParamsProto* spacetime_constraints;
  const VehicleGeometryParamsProto* vehicle_geom;
  const ApolloTrajectoryPointProto* plan_start_point;
  const absl::flat_hash_set<std::string>* stalled_objects;
  const RouteNaviInfo* route_navi_info;
  const absl::flat_hash_set<mapping::ElementId>* avoid_lanes;
  const absl::Time plan_time;
  const std::optional<bool> alc_confirmation;
  const SelectorState* selector_state;
  const SelectorFlags* selector_flags;
  const SelectorParamsProto* config;
};

struct DeviateNaviInput {
  const PlannerSemanticMapManager* psmm;
  const e2e_noa::Behavior_FunctionId func_id;
  const e2e_noa::LaneChangeStage& pre_lc_stage;
  const VehicleGeometryParamsProto* vehicle_geom;
  const ApolloTrajectoryPointProto* plan_start_point;
  const LaneSequencePtr& pre_target_laneseq;
  const LaneSequencePtr& pre_target_laneseq_before_lc;
};

}  

#endif
