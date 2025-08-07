#ifndef ST_PLANNING_PNP_UTIL
#define ST_PLANNING_PNP_UTIL

#include "autonomy_state.pb.h"
#include "lane_path.pb.h"
#include "plan/planner_semantic_map_manager.h"
#include "pnp_info.pb.h"
#include "router/route_sections.h"

namespace e2e_noa {
namespace planning {

enum class PNP_LC_STATE {
  NONE = 0,
  KEEP = 1,
  LEFT = 2,
  RIGHT = 3,
};

DriverAction::LaneChangeCommand PNPIntention(
    const PlannerSemanticMapManager& psmm,
    const RouteSections& route_section_in_horizon,
    const std::shared_ptr<PNPInfo> pnp_info,
    const mapping::LanePath& prev_target_lane_path);

std::vector<mapping::ElementId> PnpIds(const std::shared_ptr<PNPInfo> pnp_info);

}  // namespace planning
}  // namespace e2e_noa

#endif
