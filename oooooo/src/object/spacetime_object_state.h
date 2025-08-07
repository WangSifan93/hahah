#ifndef ONBOARD_PLANNER_OBJECT_SPACETIME_OBJECT_STATE_H_
#define ONBOARD_PLANNER_OBJECT_SPACETIME_OBJECT_STATE_H_

#include <vector>

#include "math/geometry/box2d.h"
#include "math/geometry/polygon2d.h"
#include "math/vec.h"
#include "prediction/predicted_trajectory.h"

namespace e2e_noa::planning {

struct SpacetimeObjectState {
  const prediction::PredictedTrajectoryPoint* traj_point;

  Box2d box;
  Polygon2d contour;
};

std::vector<SpacetimeObjectState> ExtractTrajectoryStatePoints(
    const prediction::PredictedTrajectory& pred_traj, const Vec2d& init_pos,
    const Polygon2d& init_contour, const Box2d& init_box);
}  // namespace e2e_noa::planning

#endif
