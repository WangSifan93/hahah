#ifndef ONBOARD_PLANNER_OBJECT_PREDICTED_MOTION_FILTER_H_
#define ONBOARD_PLANNER_OBJECT_PREDICTED_MOTION_FILTER_H_

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "absl/types/span.h"
#include "math/geometry/aabox2d.h"
#include "math/geometry/aabox_kdtree2d.h"
#include "math/geometry/box2d.h"
#include "math/vec.h"
#include "object/planner_object.h"
#include "object/trajectory_filter.h"
#include "planner_object.pb.h"
#include "prediction/predicted_trajectory.h"

namespace e2e_noa {
namespace planning {

class PredictedSpacetimeFilter : public TrajectoryFilter {
 public:
  struct ObjectCollisionCheckingState {
    std::string id;

    Box2d box;
    AABox2d ComputeAABox() const { return box.GetAABox(); }

    double DistanceSquareTo(const Vec2d& point) const {
      return box.DistanceSquareTo(point);
    }
  };

  explicit PredictedSpacetimeFilter(
      absl::Span<const PlannerObject> planner_objects);

  FilterReason::Type Filter(
      const PlannerObject& object,
      const prediction::PredictedTrajectory& traj) const override;

 private:
  std::vector<ObjectCollisionCheckingState> confident_object_states_;
  std::unique_ptr<AABoxKDTree2d<ObjectCollisionCheckingState>> kdtree_;
};

}  // namespace planning
}  // namespace e2e_noa

#endif
