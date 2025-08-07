#ifndef AD_E2E_SCHEDULER_CANDIDATE_LANE_SEQUENCES_H
#define AD_E2E_SCHEDULER_CANDIDATE_LANE_SEQUENCES_H
#include "autonomy_state.pb.h"
#include "behavior.pb.h"
#include "lane_change.pb.h"
#include "maps/lane_path.h"
#include "maps/map.h"
#include "pnp_info.pb.h"
#include <unordered_map>

namespace ad_e2e {
namespace planning {
bool DecisionExplorationCandidateLaneSequences(
    const Vec2d& ego_pos, const MapPtr& map, const LaneConstPtr& start_lane,
    const LaneConstPtr& nearest_lane,
    const std::optional<e2e_noa::PNPInfos>& pnp_infos,
    const e2e_noa::DriverAction::LaneChangeCommand mlc_cmd,
    e2e_noa::mapping::LanePath* preferred_lane_path,
    std::vector<std::vector<LaneConstPtr>>* candidate_lane_seqs,
    std::vector<int>* candidate_lane_seqs_proprobabilty,
    const e2e_noa::LaneChangeStage& lane_change_stage,
    const ad_e2e::planning::LaneSequencePtr& pre_target_lane_seq,
    const ad_e2e::planning::LaneSequencePtr& scene_target_lane_seq,
    const bool is_navi, const bool is_lka,
    const e2e_noa::Behavior_FunctionId& map_func_id, const double ego_heading,
    const double ego_l_offset, bool& if_continue_lc,
    const e2e_noa::LaneChangeStateProto lc_state_proto, int auto_drive_counter);

bool GetMatchLanePathByIds(
    const MapConstPtr& map, const std::vector<std::string> lane_ids,
    const std::vector<std::vector<LaneConstPtr>>& all_lane_seqs,
    std::vector<LaneConstPtr>* match_lane_seq);

bool GetLaneSeqInfo(LaneConstPtr current_lane, const Vec2d start_point,
                    const LaneSequencePtr& tgt_lane_seq, const MapPtr& map,
                    const std::optional<e2e_noa::PNPInfos>& pnp_infos,
                    LaneSeqInfo* tgt_seq_info);

double GetDistanceToSolidLine(const Vec2d& start_point,
                              const LaneSequencePtr& tgt_lane_seq,
                              const LaneConstPtr& nearest_lane, bool is_left,
                              double check_range);
}  // namespace planning
}  // namespace ad_e2e
#endif
