#ifndef ONBOARD_PLANNER_PLAN_MULTI_TASKS_CRUISE_PLANNER_H_
#define ONBOARD_PLANNER_PLAN_MULTI_TASKS_CRUISE_PLANNER_H_

#include "async/thread_pool.h"
#include "common/planner_status.h"
#include "context/e2e_planner_context.h"
#include "context/ego_state.h"
#include "context/lane_change_command_update.h"
#include "context/lane_manager.h"
#include "plan/planner_state.h"
#include "plan/planner_world.h"

namespace e2e_noa::planning {
class E2EPlanner {
 public:
  using PushDirection = ad_e2e::planning::PushDirection;

  E2EPlanner() = default;
  ~E2EPlanner() = default;
  void init();
  PlannerStatus run(const PlannerWorldInput &input,
                    const PlannerState &planner_state,
                    PlannerWorldOutput *output,
                    WorkerThreadManager *thread_pool);
  void ParseDecisionExplorationOutputToPlannerState(
      const DecisionExplorationOutput &decision_exploration,
      PlannerState *planner_state, const ad_e2e::planning::MapPtr &map);
  void ParseStPlannerOutputToPlannerState(
      const StPlannerOutput &st_planner_output, PlannerState *planner_state);

  PushDirection PushIntention(
      const ApolloTrajectoryPointProto plan_start_point,
      const PlannerObjectController *obj_mgr,
      const std::vector<DecisionExplorationOutput> &tasks,
      const bool &last_lc_push_state, const bool &if_miss_navi_secnario);
  void ProcessTrajectorySelection();

 private:
  std::shared_ptr<E2EPlannerContext> e2e_planner_context_{nullptr};
  std::shared_ptr<EgoState> ego_state_{nullptr};
  std::shared_ptr<LaneChangeCommandUpdate> lane_change_command_update_{nullptr};
  std::shared_ptr<LaneManager> lane_manager_{nullptr};
};

}  // namespace e2e_noa::planning

#endif
