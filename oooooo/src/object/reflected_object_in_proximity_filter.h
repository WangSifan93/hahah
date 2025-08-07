#ifndef ONBOARD_PLANNER_OBJECT_REFLECTED_OBJECT_IN_PROXIMITY_FILTER_H_
#define ONBOARD_PLANNER_OBJECT_REFLECTED_OBJECT_IN_PROXIMITY_FILTER_H_

#include "math/geometry/box2d.h"
#include "object/planner_object.h"
#include "object/trajectory_filter.h"
#include "planner_object.pb.h"
#include "positioning.pb.h"
#include "prediction/predicted_trajectory.h"
#include "vehicle.pb.h"

namespace e2e_noa {
namespace planning {

class ReflectedObjectInProximityFilter : public TrajectoryFilter {
 public:
  ReflectedObjectInProximityFilter(
      const PoseProto& pose, const VehicleGeometryParamsProto& vehicle_geom,
      double padding);

  FilterReason::Type Filter(
      const PlannerObject& object,
      const prediction::PredictedTrajectory& traj) const override;

 private:
  Box2d padded_box_;
};

}  // namespace planning
}  // namespace e2e_noa

#endif
