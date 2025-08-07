#ifndef AD_E2E_PLANNING_COMMON_OBSTACLE_PREDICTION_CONSTRAIN_H
#define AD_E2E_PLANNING_COMMON_OBSTACLE_PREDICTION_CONSTRAIN_H

#include "math/vec2d.h"

namespace ad_e2e {
namespace planning {
namespace obstacle {
using Vec2d = math::Vec2d;

struct PredictionConstrain {
  PredictionConstrain(const Vec2d& point, const double time, const Vec2d& v) {
    limit_point = point;
    relative_time = time;
    velocity = v;
    heading = velocity.Angle();
  }
  PredictionConstrain() {}

  Vec2d limit_point;
  Vec2d velocity;
  Vec2d acceleration;
  double heading = 0.0;
  double relative_time = 0.0;
};
}  // namespace obstacle
}  // namespace planning
}  // namespace ad_e2e

#endif
