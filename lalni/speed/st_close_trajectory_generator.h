#ifndef PLANNER_SPEED_ST_CLOSE_TRAJECTORY_GENERATOR_H_
#define PLANNER_SPEED_ST_CLOSE_TRAJECTORY_GENERATOR_H_

#include <vector>

#include "object/spacetime_object_trajectory.h"
#include "speed/st_close_trajectory.h"

namespace e2e_noa::planning {
std::vector<StCloseTrajectory> GenerateMovingStCloseTrajectories(
    const SpacetimeObjectTrajectory& st_traj,
    std::vector<std::vector<StCloseTrajectory::StNearestPoint>>
        close_traj_points);

}

#endif
