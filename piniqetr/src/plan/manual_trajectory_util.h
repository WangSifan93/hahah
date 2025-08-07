#ifndef ONBOARD_PLANNER_MANUAL_TRAJECTORY_UTIL_H_
#define ONBOARD_PLANNER_MANUAL_TRAJECTORY_UTIL_H_

#include "absl/time/time.h"
#include "trajectory.pb.h"

namespace e2e_noa {
namespace planning {

void CompleteTrajectoryPastPoints(double trajectory_time_step,
                                  TrajectoryProto* trajectory);

void ShiftPreviousTrajectory(const absl::Duration shift_duration,
                             TrajectoryProto* trajectory);

void CompleteTrajectoryPastPoints(TrajectoryProto* trajectory);

void UpdateTrajectoryPointAccel(TrajectoryProto* trajectory);

}  // namespace planning
}  // namespace e2e_noa

#endif
