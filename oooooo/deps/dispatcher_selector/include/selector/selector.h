#ifndef PLANNER_SELECTOR_SELECTOR_H_
#define PLANNER_SELECTOR_SELECTOR_H_

#include <vector>

#include "absl/status/statusor.h"
#include "common/planner_status.h"
#include "object/spacetime_trajectory_manager.h"
#include "plan/st_planner_output.h"
#include "pnp_info.pb.h"
#include "selector/selector_input.h"
#include "selector/selector_state.h"
#include "selector_debug.pb.h"
namespace e2e_noa {
namespace planning {
using LcReason = ad_e2e::planning::LcReason;

absl::StatusOr<int> SelectTrajectory(
    const SelectorInput& input, const std::vector<PlannerStatus>& st_status,
    const std::vector<SpacetimeTrajectoryManager>& st_traj_mgr_list,
    const std::vector<StPlannerOutput>& results,
    const std::optional<PNPInfos>& pnp_infos,
    const DeviateNaviInput& deviate_navi_input,
    SelectorDebugProto* selector_debug, SelectorState* selector_state);

}  
}  

#endif
