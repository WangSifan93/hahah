#include "plan/pnp_util.h"

namespace e2e_noa {
namespace planning {

DriverAction::LaneChangeCommand PNPIntention(
    const PlannerSemanticMapManager& psmm,
    const RouteSections& route_section_in_horizon,
    const std::shared_ptr<PNPInfo> pnp_info,
    const mapping::LanePath& prev_target_lane_path) {
  DriverAction::LaneChangeCommand pnp_lc_state = DriverAction::LC_CMD_NONE;
  if (pnp_info == nullptr) return pnp_lc_state;

  auto front_section_info =
      psmm.FindSectionByIdOrNull(route_section_in_horizon.front().id);
  std::vector<std::string> lane_ids;
  lane_ids.assign(front_section_info->lanes().begin(),
                  front_section_info->lanes().end());

  std::vector<mapping::ElementId> pnp_lane_ids;
  for (auto id : pnp_info->target_lane_sequence_ids()) {
    pnp_lane_ids.push_back(static_cast<mapping::ElementId>(id));
  }

  mapping::ElementId pnp_id;
  if (!pnp_lane_ids.empty()) {
    pnp_id = pnp_lane_ids.front();
  }
  const auto pnp_it = std::find(lane_ids.begin(), lane_ids.end(), pnp_id);
  if (pnp_it == lane_ids.end()) {
    return DriverAction::LC_CMD_NONE;
  }
  const int pnp_index = pnp_it - lane_ids.begin();

  mapping::ElementId prev_id;
  if (!prev_target_lane_path.IsEmpty() &&
      !prev_target_lane_path.lane_path_data().lane_ids().empty()) {
    prev_id = prev_target_lane_path.lane_path_data().lane_ids().front();
  }
  const auto prev_it = std::find(lane_ids.begin(), lane_ids.end(), prev_id);
  if (prev_it == lane_ids.end()) {
    return DriverAction::LC_CMD_NONE;
  }
  int prev_index = prev_it - lane_ids.begin();

  if (prev_index < pnp_index) {
    return DriverAction::LC_CMD_RIGHT;
  } else if (prev_index > pnp_index) {
    return DriverAction::LC_CMD_LEFT;
  } else {
    return DriverAction::LC_CMD_NONE;
  }
}

std::vector<mapping::ElementId> PnpIds(
    const std::shared_ptr<PNPInfo> pnp_info) {
  std::vector<mapping::ElementId> pnp_lane_ids;
  if (pnp_info == nullptr) {
    return pnp_lane_ids;
  } else {
  }
  pnp_lane_ids.clear();
  for (auto id : pnp_info->target_lane_sequence_ids()) {
    pnp_lane_ids.push_back(static_cast<mapping::ElementId>(id));
  }
  return pnp_lane_ids;
}

}  // namespace planning
}  // namespace e2e_noa
