#include "object/spacetime_state_filter.h"

#include <cmath>

#include "affine_transformation.pb.h"
#include "math/geometry/polygon2d.h"
#include "math/util.h"
#include "plan/trajectorypoint_with_acceleration.h"
#include "prediction/prediction_util.h"

namespace e2e_noa {
namespace planning {

SpacetimeStateFilter::SpacetimeStateFilter(
    const PoseProto& pose, const VehicleGeometryParamsProto& vehicle_geom) {
  yaw_ = pose.yaw();
  pos_ = Vec2d(pose.pos_smooth().x(), pose.pos_smooth().y());
  tangent_ = Vec2d::FastUnitFromAngle(pose.yaw());
  speed_ = pose.speed();
  velocity_ = Vec2d(pose.vel_smooth().x(), pose.vel_smooth().y());

  constexpr double kBackOffDistance = 2.0;
  spacetime_backoff_pos_ =
      pos_ - tangent_ * (kBackOffDistance + vehicle_geom.back_edge_to_center());
  stationary_backoff_pos_ = pos_;
}

FilterReason::Type SpacetimeStateFilter::Filter(
    const PlannerObject& object,
    const prediction::PredictedTrajectory& traj) const {
  if (tangent_.Dot(velocity_) < 0.0) {
    return FilterReason::NONE;
  }

  const auto& contour = object.contour();
  const Vec2d obj_center = contour.CircleCenter();

  if (prediction::IsStationaryTrajectory(traj)) {
    const Vec2d av_to_obj = obj_center - stationary_backoff_pos_;
    const double proj_lon = tangent_.Dot(av_to_obj);
    const double padding = contour.CircleRadius();
    if (proj_lon < -padding) {
      return FilterReason::STATIONARY_OBJECT_BEHIND_AV;
    }
  } else {
    const Vec2d av_to_obj = obj_center - spacetime_backoff_pos_;
    const double proj_lon = tangent_.Dot(av_to_obj);
    const double padding = contour.CircleRadius();

    constexpr double kOppositeDirectionThreshold = M_PI_4;
    if (proj_lon < -padding &&
        std::abs(AngleDifference(object.pose().theta(), OppositeAngle(yaw_))) <
            kOppositeDirectionThreshold) {
      return FilterReason::OBJECT_BEHIND_MOVING_AWAY_FROM_AV;
    }
  }
  return FilterReason::NONE;
}

}  // namespace planning
}  // namespace e2e_noa
