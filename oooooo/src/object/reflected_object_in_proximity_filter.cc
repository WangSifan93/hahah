#include "object/reflected_object_in_proximity_filter.h"

#include <algorithm>

#include "affine_transformation.pb.h"
#include "math/geometry/polygon2d.h"
#include "math/vec.h"
#include "perception.pb.h"

namespace e2e_noa {
namespace planning {

namespace {
bool MirroredEntityCategory(ObjectType type) {
  switch (type) {
    case ObjectType::OT_UNKNOWN_STATIC:
    case ObjectType::OT_FOD:
    case ObjectType::OT_BARRIER:
    case ObjectType::OT_CONE:
    case ObjectType::OT_WARNING_TRIANGLE:
    case ObjectType::OT_VEGETATION:
      return true;
    case ObjectType::OT_VEHICLE:
    case ObjectType::OT_LARGE_VEHICLE:
    case ObjectType::OT_MOTORCYCLIST:
    case ObjectType::OT_PEDESTRIAN:
    case ObjectType::OT_CYCLIST:
    case ObjectType::OT_TRICYCLIST:
    case ObjectType::OT_UNKNOWN_MOVABLE:
      return false;
    default:
      throw std::runtime_error("switch case on enum unexpected");
  }
}
}  // namespace

ReflectedObjectInProximityFilter::ReflectedObjectInProximityFilter(
    const PoseProto& pose, const VehicleGeometryParamsProto& vehicle_geom,
    double padding) {
  const Vec2d pos(pose.pos_smooth().x(), pose.pos_smooth().y());

  const Vec2d tangent = Vec2d::FastUnitFromAngle(pose.yaw());
  const double half_length = vehicle_geom.length() * 0.5;
  const double half_width = vehicle_geom.width() * 0.5;
  const double rac_to_center =
      half_length - vehicle_geom.back_edge_to_center() - 0.5 * padding;
  const Vec2d center = pos + tangent * rac_to_center;
  padded_box_ = Box2d(half_length + 0.5 * padding, half_width + padding, center,
                      pose.yaw(), tangent);
}

FilterReason::Type ReflectedObjectInProximityFilter::Filter(
    const PlannerObject& object,
    const prediction::PredictedTrajectory& traj) const {
  if (!MirroredEntityCategory(object.type())) {
    return FilterReason::NONE;
  }

  if (std::max(object.bounding_box().length(), object.bounding_box().width()) >
      padded_box_.length()) {
    return FilterReason::NONE;
  }

  const auto& contour = object.contour();

  if (contour.HasOverlap(padded_box_)) {
    return FilterReason::REFLECTED_OBJECT_IN_PROXIMITY;
  }
  return FilterReason::NONE;
}

}  // namespace planning
}  // namespace e2e_noa
