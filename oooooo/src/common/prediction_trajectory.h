#ifndef AD_E2E_PLANNING_COMMON_PREDICTION_TRAJECTOR_H
#define AD_E2E_PLANNING_COMMON_PREDICTION_TRAJECTOR_H
#include <vector>

#include "common/type_def.h"
#include "math/vec2d.h"

namespace ad_e2e {
namespace planning {

using Vec2d = math::Vec2d;

struct PredictionState {
  Vec2d position;
  double heading;
  Vec2d velocity;
  Vec2d acceleration;
  double time_sec;
};

struct PredictionTrajectory {
  int32_t id = 0;
  int32_t behavior_id = -1;
  double probability = 0.0;
  std::vector<PredictionState> trajectory;
  PredictorType predictor_type = PredictorType::UNKNOWN;
  ObstacleIntention intention = ObstacleIntention::INTENTION_UNKNOWN;
};

}  // namespace planning
}  // namespace ad_e2e

#endif
