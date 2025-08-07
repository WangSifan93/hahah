#ifndef ONBOARD_PLANNER_COMMON_PLAN_START_POINT_INFO_H_
#define ONBOARD_PLANNER_COMMON_PLAN_START_POINT_INFO_H_

#include <optional>
#include <string>
#include <vector>

#include "absl/strings/str_format.h"
#include "absl/strings/str_join.h"
#include "absl/time/time.h"
#include "planner.pb.h"
#include "trajectory_point.pb.h"
#include "util/time_util.h"

namespace e2e_noa {
namespace planning {

struct PlanStartPointInfo {
  bool reset;
  std::optional<int> start_index_on_prev_traj;
  ApolloTrajectoryPointProto start_point;
  double path_s_increment_from_previous_frame;
  absl::Time plan_time;
  bool full_stop;
  ResetReasonProto::Reason reset_reason;
  std::string DebugString() const {
    return absl::StrFormat(
        "reset: %d, reset_reason: %s, plan_time: %f, full_stop: %d, "
        "path_s_increment_from_previous_frame: %f, start_point: %s, ",
        reset, ResetReasonProto::Reason_Name(reset_reason),
        ToUnixDoubleSeconds(plan_time), full_stop,
        path_s_increment_from_previous_frame, start_point.ShortDebugString());
  }
};

struct StPathPlanStartPointInfo {
  bool reset;
  int relative_index_from_plan_start_point;
  ApolloTrajectoryPointProto start_point;
  absl::Time plan_time;
};

}  // namespace planning
}  // namespace e2e_noa

#endif
