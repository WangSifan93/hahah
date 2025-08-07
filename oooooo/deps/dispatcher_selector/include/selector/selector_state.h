#ifndef PLANNER_SELECTOR_SELECTOR_STATE_H_
#define PLANNER_SELECTOR_SELECTOR_STATE_H_

#include <deque>
#include <optional>

#include "absl/time/time.h"
#include "lane_change.pb.h"
#include "selector_state.pb.h"
#include "turn_signal.pb.h"

namespace e2e_noa {
namespace planning {

struct SelectorState {
  void FromProto(const SelectorStateProto& proto);
  void ToProto(SelectorStateProto* proto) const;

  std::deque<TargetLaneStateProto> history_best_target_lane_states;
  TargetLaneStateProto selected_target_lane_state;
  TurnSignal pre_turn_signal = TurnSignal::TURN_SIGNAL_NONE;
  std::optional<absl::Time> last_redlight_stop_time = std::nullopt;
  LaneChangeReason lane_change_reason = LaneChangeReason::NO_CHANGE;
  LastLcInfoProto last_lc_info;
  SelectorLaneChangeRequestProto selector_lane_change_request;
  std::optional<absl::Time> last_user_reject_alc_time = std::nullopt;
  LaneChangeReason last_user_reject_alc_reason = LaneChangeReason::NO_CHANGE;
  LaneChangePrepareState lane_change_prepare_state =
      LaneChangePrepareState::Lane_Keeping;
  LcFeasibility lc_unable_reason = LcFeasibility::FEASIBILITY_OK;
  LaneChangeReason last_lane_change_reason = LaneChangeReason::NO_CHANGE;
  LaneChangeState pre_lane_change_state = LaneChangeState::Lc_Keeping;
  e2e_noa::LaneChangeStage prev_lc_stage = e2e_noa::LaneChangeStage::LCS_NONE;
  std::string gap_front_id = "";
  std::string gap_back_id = "";
  bool is_deviate_navi = false;
};

}  
}  

#endif
