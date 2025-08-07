#ifndef PLANNER_SPEED_EMPTY_ROAD_SPEED_H_
#define PLANNER_SPEED_EMPTY_ROAD_SPEED_H_

#include "plan/discretized_path.h"
#include "plan/speed_profile.h"
#include "planner_params.pb.h"
#include "speed/speed_limit.h"

namespace e2e_noa {
namespace planning {

SpeedProfile CreateEmptyRoadSpeedProfile(
    const SpacetimeConstraintParamsProto& spacetime_constraint_params,
    const DiscretizedPath& path, const SpeedLimit& speed_limit, double v_now,
    int traj_steps);

}
}  // namespace e2e_noa

#endif
