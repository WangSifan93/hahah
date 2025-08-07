#ifndef ONBOARD_PLANNER_ASSIST_RLC_INTERNAL_RESULT_H_
#define ONBOARD_PLANNER_ASSIST_RLC_INTERNAL_RESULT_H_

#include <memory>
#include <string>
#include <vector>

#include "autonomy_state.pb.h"
#include "maps/lane_path.h"

namespace e2e_noa::planning {

struct PlcInternalResult {
  enum PlcInternalStatus {
    kOk = 0,
    kBranchNotFound = 1,
    kBranchFailedInternal = 2,
    kSolidBoundary = 3,
    kUnsafeObject = 4,
  };
  PlcInternalStatus status = kOk;

  std::vector<std::string> unsafe_object_ids;
  std::optional<bool> left_solid_boundary = std::nullopt;

  mapping::LanePath preferred_lane_path;
  DriverAction::LaneChangeCommand lane_change_command =
      DriverAction::LC_CMD_NONE;
};

}  // namespace e2e_noa::planning

#endif
