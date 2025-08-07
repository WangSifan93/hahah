#ifndef PLANNER_SPEED_DECIDER_SPEED_LIMIT_DECIDER_H_
#define PLANNER_SPEED_DECIDER_SPEED_LIMIT_DECIDER_H_

#include <vector>

#include "common/path_sl_boundary.h"
#include "constraint.pb.h"
#include "plan/discretized_path.h"
#include "router/plan_passage.h"
#include "speed/open_loop_speed_limit.h"
#include "speed/st_graph_defs.h"
#include "vehicle.pb.h"

namespace e2e_noa {
namespace planning {

void LaneChangeSpeedDecider(const double& av_speed, const double& max_acc,
                            OpenLoopSpeedLimit* open_loop_speed_limit);

}
}  // namespace e2e_noa

#endif
