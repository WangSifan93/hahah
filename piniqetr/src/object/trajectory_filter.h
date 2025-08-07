#ifndef ONBOARD_PLANNER_OBJECT_TRAJECTORY_FILTER_H_
#define ONBOARD_PLANNER_OBJECT_TRAJECTORY_FILTER_H_

#include <memory>
#include <vector>

#include "object/planner_object.h"
#include "planner.pb.h"
#include "prediction/predicted_trajectory.h"

namespace e2e_noa {
namespace planning {

class TrajectoryFilter {
 public:
  virtual FilterReason::Type Filter(
      const PlannerObject& object,
      const prediction::PredictedTrajectory& traj) const = 0;

  virtual ~TrajectoryFilter() {}
};

}  // namespace planning
}  // namespace e2e_noa
#endif
