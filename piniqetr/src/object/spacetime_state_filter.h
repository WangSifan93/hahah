#ifndef ONBOARD_PLANNER_OBJECT_MOTION_STATE_FILTER_H_
#define ONBOARD_PLANNER_OBJECT_MOTION_STATE_FILTER_H_

#include "math/vec.h"
#include "object/planner_object.h"
#include "object/trajectory_filter.h"
#include "planner_object.pb.h"
#include "positioning.pb.h"
#include "prediction/predicted_trajectory.h"
#include "vehicle.pb.h"

namespace e2e_noa {
namespace planning {

class SpacetimeStateFilter : public TrajectoryFilter {
 public:
  SpacetimeStateFilter(const PoseProto& pose,
                       const VehicleGeometryParamsProto& vehicle_geom);

  FilterReason::Type Filter(
      const PlannerObject& object,
      const prediction::PredictedTrajectory& traj) const override;

 private:
  double yaw_;
  Vec2d tangent_;
  Vec2d pos_;
  Vec2d velocity_;
  double speed_;
  Vec2d spacetime_backoff_pos_;
  Vec2d stationary_backoff_pos_;
};

}  // namespace planning
}  // namespace e2e_noa

#endif
