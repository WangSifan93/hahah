#ifndef E2E_PLANNER_CONTEXT_H_
#define E2E_PLANNER_CONTEXT_H_

#include <memory>

namespace e2e_noa::planning {
class EgoState;
class LaneChangeCommandUpdate;
class LaneManager;

class E2EPlannerContext {
 public:
  E2EPlannerContext() = default;
  ~E2EPlannerContext() = default;

  void set_ego_state(const std::shared_ptr<EgoState>& ego_state_ptr) {
    ego_state_ptr_ = ego_state_ptr;
  }
  const std::shared_ptr<EgoState>& ego_state() const { return ego_state_ptr_; }

  void set_lane_change_command_update(
      const std::shared_ptr<LaneChangeCommandUpdate>&
          lane_change_command_update_ptr) {
    lane_change_command_update_ptr_ = lane_change_command_update_ptr;
  }
  const std::shared_ptr<LaneChangeCommandUpdate>& lane_change_command_update()
      const {
    return lane_change_command_update_ptr_;
  }

  void set_lane_manager(const std::shared_ptr<LaneManager>& lane_manager_ptr) {
    lane_manager_ptr_ = lane_manager_ptr;
  }
  const std::shared_ptr<LaneManager>& lane_manager_ptr() const {
    return lane_manager_ptr_;
  }

 private:
  std::shared_ptr<EgoState> ego_state_ptr_{nullptr};
  std::shared_ptr<LaneChangeCommandUpdate> lane_change_command_update_ptr_{
      nullptr};
  std::shared_ptr<LaneManager> lane_manager_ptr_{nullptr};
};

}  // namespace e2e_noa::planning
#endif