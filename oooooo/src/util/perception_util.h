#ifndef ONBOARD_PLANNER_UTIL_PERCEPTION_UTIL_H_
#define ONBOARD_PLANNER_UTIL_PERCEPTION_UTIL_H_

#include <string>

#include "math/geometry/polygon2d.h"
#include "perception.pb.h"
#include "positioning.pb.h"
#include "vehicle.pb.h"

namespace e2e_noa {
namespace planning {

inline bool IsConsiderMirrorObject(const ObjectProto& object_proto,
                                   double min_mirror_height_avg,
                                   double max_mirror_height_avg) {
  if (!object_proto.has_min_z() || !object_proto.has_max_z() ||
      !object_proto.has_ground_z()) {
    return true;
  }
  const double object_max_height =
      object_proto.max_z() - object_proto.ground_z();
  const double object_min_height =
      object_proto.min_z() - object_proto.ground_z();
  return object_max_height > min_mirror_height_avg &&
         object_min_height < max_mirror_height_avg;
}

inline bool IsVehicle(ObjectType type) {
  return (type == ObjectType::OT_VEHICLE ||
          type == ObjectType::OT_LARGE_VEHICLE);
}
bool IsLargeVehicle(const ObjectProto& object);

Polygon2d ComputeObjectContour(const ObjectProto& object);

ObjectProto AvPositionProtocolToObjectProtocol(
    const std::string& object_id,
    const VehicleGeometryParamsProto& vehicle_geom, const PoseProto& pose,
    bool offroad);

bool IsCameraObject(const ObjectProto& object);

}  // namespace planning
}  // namespace e2e_noa

#endif
