#ifndef LANE_MANAGER_H_
#define LANE_MANAGER_H_

#include "behavior.pb.h"
#include "common/planner_status.h"
#include "common/vector/polyline.h"
#include "context/e2e_planner_context.h"
#include "context/e2e_planner_utils.h"
#include "maps/lane_path.h"
#include "maps/map.h"
#include "math/vec.h"
#include "plan/planner_semantic_map_manager.h"
#include "plan/planner_state.h"
#include "plan/planner_world.h"
#include "router/lane_graph/v2/route_lane_graph_builder.h"
#include "router/navi/route_navi_info.h"
#include "router/route_sections.h"
#include "router/route_sections_util.h"
#include "scene/construction_scene_identification.h"

namespace e2e_noa::planning {
class LaneManager {
 public:
  explicit LaneManager(std::shared_ptr<E2EPlannerContext> e2e_planner_context)
      : e2e_planner_context_(e2e_planner_context){};

  ~LaneManager() = default;

  void update(const PlannerWorldInput &input, const PlannerState &planner_state,
              PlannerWorldOutput *output, WorkerThreadManager *thread_pool,
              std::optional<double> &cruising_speed_limit);

  e2e_noa::Behavior_FunctionId function_id() const { return function_id_; }

  bool is_navi() const { return is_navi_; }

  bool is_lka() const { return is_lka_; }

  PlannerStatus planner_status() const { return planner_status_; }

  ad_e2e::planning::LaneSequencePtr pre_target_lane_seq() const {
    return pre_target_lane_seq_;
  }
  std::vector<double> last_angles() const { return last_angles_; }

  mapping::LanePath &prev_target_lane_path_from_start() {
    return prev_target_lane_path_from_start_;
  }

  mapping::LanePath &prev_lane_path_before_lc_from_start() {
    return prev_lane_path_before_lc_from_start_;
  }

  ad_e2e::planning::LaneSequencePtr pre_lane_seq_before_lc() const {
    return pre_lane_seq_before_lc_;
  }

  ad_e2e::planning::LaneSequencePtr scene_target_lane_seq() const {
    return scene_target_lane_seq_;
  }

  LaneChangeReason new_lane_change_reason() const {
    return new_lane_change_reason_;
  }

  bool if_continuous_lc_secnario() const { return if_continuous_lc_secnario_; }

  bool is_open_gap() const { return is_open_gap_; }

  std::vector<LanePathInfo> target_lp_infos() const { return target_lp_infos_; }

  bool if_miss_navi_secnario() const { return if_miss_navi_secnario_; }

  int cur_navi_lc_num() const { return cur_navi_lc_num_; }

  double left_navi_dist_v2() const { return left_navi_dist_v2_; }

  double left_navi_dist() const { return left_navi_dist_; }

  double cur_dist_to_junction() const { return cur_dist_to_junction_; }

  ad_e2e::planning::LaneConstPtr nearest_lane() const { return nearest_lane_; }

  ad_e2e::planning::V2TurnInfo::V2DetailTurnType cur_nearest_turn_type() const {
    return cur_nearest_turn_type_;
  }

  ad_e2e::planning::V2TurnInfo::V2DetailTurnType last_turn_type_v2() const {
    return last_turn_type_v2_;
  }

  std::optional<double> UpdateCruisingSpeedLimitByLabel(
      const ad_e2e::planning::MapPtr &map, const double &start_v,
      const double &raw_cruising_speed_limit,
      const ad_e2e::planning::LaneSequencePtr &prev_target_lane_seq);

  void HandleSceneLcIntent(
      const ApolloTrajectoryPointProto *plan_start_point,
      const VehicleParamsProto *vehicle_param,

      const std::optional<double> lcc_cruising_speed_limit,
      const std::optional<Behavior_FunctionId> function_id,
      const std::optional<int> tunnel_status,
      const DriverAction::LaneChangeCommand &lane_change_command,
      const LaneChangeStateProto *lane_change_state,
      const PlannerSemanticMapManager *psmm, const Vec2d &ego_pos,
      const LaneConstPtr &nearest_lane, const PlannerObjectController &obj_mgr,
      const LaneSequencePtr &pre_target_laneseq,
      const std::optional<PNPInfos> pnp_infos, const bool lka_allow_navi_lc,
      const LaneChangeReason &lane_change_reason,
      LaneChangeReason &new_lane_change_reason,
      LaneSequencePtr &target_lane_seq);

  void CountNaviAndEfficiencyInfo(const Vec2d &ego_pos,
                                  const LaneSequencePtr &laneseq,
                                  const PlannerObjectController &obj_mgr,
                                  int *navi_section_cnt, double *average_speed);

  RouteNaviInfo AlignRouteNaviInfoWithCurrentSections(
      const PlannerSemanticMapManager &psmm,
      const RouteSections &route_sections,
      const RouteSections &current_sections,
      const RouteNaviInfo &route_navi_info);

 private:
  std::weak_ptr<E2EPlannerContext> e2e_planner_context_;
  e2e_noa::Behavior_FunctionId function_id_;
  bool is_navi_ = false;
  bool is_lka_ = false;
  PlannerStatus planner_status_;
  ad_e2e::planning::LaneSequencePtr pre_target_lane_seq_{nullptr};
  std::vector<double> last_angles_;
  mapping::LanePath prev_target_lane_path_from_start_;
  mapping::LanePath prev_lane_path_before_lc_from_start_;
  ad_e2e::planning::LaneSequencePtr pre_lane_seq_before_lc_{nullptr};
  ad_e2e::planning::LaneSequencePtr scene_target_lane_seq_{nullptr};
  LaneChangeReason new_lane_change_reason_ = LaneChangeReason::NO_CHANGE;
  bool if_continuous_lc_secnario_ = false;
  bool is_open_gap_ = true;
  std::vector<LanePathInfo> target_lp_infos_;
  bool if_miss_navi_secnario_ = false;
  int cur_navi_lc_num_ = 0;
  double left_navi_dist_v2_ = 999.0;
  double left_navi_dist_ = 999.0;
  double cur_dist_to_junction_ = DBL_MAX;
  ad_e2e::planning::V2TurnInfo::V2DetailTurnType cur_nearest_turn_type_ =
      ad_e2e::planning::V2TurnInfo::V2DetailTurnType::NONE;

  ad_e2e::planning::V2TurnInfo::V2DetailTurnType last_turn_type_v2_ =
      ad_e2e::planning::V2TurnInfo::V2DetailTurnType::NONE;

  ad_e2e::planning::LaneConstPtr nearest_lane_{nullptr};
};
}  // namespace e2e_noa::planning
#endif