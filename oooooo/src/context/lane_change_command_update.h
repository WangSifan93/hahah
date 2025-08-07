#ifndef LANE_CHANGE_COMMAND_UPDATE_H_
#define LANE_CHANGE_COMMAND_UPDATE_H_

#include "alc.pb.h"
#include "autonomy_state.pb.h"
#include "common/planner_status.h"
#include "context/e2e_planner_context.h"
#include "driving_style.pb.h"
#include "maps/lane_path.h"
#include "plan/planner_state.h"
#include "plan/planner_world.h"

namespace e2e_noa::planning {
using LcReason = ad_e2e::planning::LcReason;
class LaneChangeCommandUpdate {
 public:
  explicit LaneChangeCommandUpdate(
      std::shared_ptr<E2EPlannerContext> e2e_planner_context);
  ~LaneChangeCommandUpdate() = default;

  void update(const PlannerWorldInput& input, const PlannerState& planner_state,
              PlannerWorldOutput* output);

  mapping::LanePath& preferred_lane_path() { return preferred_lane_path_; }

  // void set_preferred_lane_path(const mapping::LanePath& preferred_lane_path)
  // { preferred_lane_path_ = preferred_lane_path;}

  ALCState& new_alc_state() { return new_alc_state_; }

  DriverAction::LaneChangeCommand& new_lc_cmd_state() {
    return new_lc_cmd_state_;
  }

  LaneChangeStyle& lane_change_style() { return lane_change_style_; }

  int pnp_top1_reason() const { return pnp_top1_reason_; }

 private:
  std::shared_ptr<E2EPlannerContext> e2e_planner_context_{nullptr};
  ALCState new_alc_state_;
  DriverAction::LaneChangeCommand new_lc_cmd_state_;
  LaneChangeStyle lane_change_style_ = LC_STYLE_NORMAL;
  int pnp_top1_reason_ = ad_e2e::planning::LC_REASON_NONE;
  mapping::LanePath preferred_lane_path_;
};
}  // namespace e2e_noa::planning
#endif