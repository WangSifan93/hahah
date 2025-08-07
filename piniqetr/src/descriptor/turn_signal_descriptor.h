
#ifndef ONBOARD_PLANNER_DECISION_TURN_SIGNAL_DECIDER_H_
#define ONBOARD_PLANNER_DECISION_TURN_SIGNAL_DECIDER_H_

#include "lane_change.pb.h"
#include "maps/lane_path.h"
#include "math/frenet_common.h"
#include "plan/planner_semantic_map_manager.h"
#include "pnp_info.pb.h"
#include "positioning.pb.h"
#include "router/plan_passage.h"
#include "router/route_sections.h"
#include "turn_signal.pb.h"

namespace e2e_noa {
namespace planning {

struct TurnSignalResult {
  TurnSignal signal = TurnSignal::TURN_SIGNAL_NONE;
  TurnSignalReason reason = TurnSignalReason::TURN_SIGNAL_OFF;
};

TurnSignalResult DecideTurnSignal(
    const PlannerSemanticMapManager& psmm, TurnSignal pre_lane_change_signal,
    const mapping::LanePath& current_lane_path,
    const std::optional<mapping::ElementId>& redlight_lane_id,
    const LaneChangeStateProto& lc_state, const PlanPassage& plan_passage,
    const FrenetBox& ego_sl_box, const std::optional<PNPInfos>& pnp_infos,
    const TurnSignalResult& planned_result, const PoseProto& ego_pose,
    bool if_continue_lc, const ad_e2e::planning::LaneSeqInfoPtr& lane_seq_info,
    TurnSignal turn_type_signal);

}  // namespace planning
}  // namespace e2e_noa

#endif
