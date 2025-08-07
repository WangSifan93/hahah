#ifndef ONBOARD_PLANNER_OPTIMIZATION_DDP_TRAJECTORY_OPTIMIZER_OUTPUT_H_
#define ONBOARD_PLANNER_OPTIMIZATION_DDP_TRAJECTORY_OPTIMIZER_OUTPUT_H_

#include <string>
#include <vector>

#include "optimization/ddp/trajectory_optimizer_state.h"
#include "plan/trajectory_point.h"
#include "planner.pb.h"
#include "util/hmi_content_util.h"

namespace e2e_noa::planning {

struct TrajectoryOptimizerOutput {
  std::vector<TrajectoryPoint> trajectory;
  std::vector<ApolloTrajectoryPointProto> trajectory_proto;

  std::optional<NudgeObjectInfo> nudge_object_info;

  TrajectoryOptimizerState trajectory_optimizer_state;
};

}  

#endif
