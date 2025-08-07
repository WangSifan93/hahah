#ifndef ONBOARD_PLANNER_UTIL_VEHICLE_GEOMETRY_UTIL_H_
#define ONBOARD_PLANNER_UTIL_VEHICLE_GEOMETRY_UTIL_H_

#include <limits>
#include <utility>

#include "aabox3d.pb.h"
#include "math/geometry/box2d.h"
#include "math/geometry/offset_rect.h"
#include "math/vec.h"
#include "vehicle.pb.h"

namespace e2e_noa {
namespace planning {

inline OffsetRect CreateOffsetRectFromVehicleGeometry(
    const VehicleGeometryParamsProto& vehicle_geom) {
  return OffsetRect(
      0.5 * vehicle_geom.length(), 0.5 * vehicle_geom.width(),
      0.5 * vehicle_geom.length() - vehicle_geom.front_edge_to_center());
}

inline std::pair<double, double> ComputeMinMaxMirrorAverageHeight(
    const VehicleGeometryParamsProto& vehicle_geometry_params) {
  if (vehicle_geometry_params.has_left_mirror() &&
      vehicle_geometry_params.has_right_mirror()) {
    const auto& left_mirror = vehicle_geometry_params.left_mirror();
    const auto& right_mirror = vehicle_geometry_params.right_mirror();
    const double min_mirror_height_avg =
        ((left_mirror.z() - left_mirror.height() * 0.5) +
         (right_mirror.z() - right_mirror.height() * 0.5)) *
        0.5;
    const double max_mirror_height_avg =
        ((left_mirror.z() + left_mirror.height() * 0.5) +
         (right_mirror.z() + right_mirror.height() * 0.5)) *
        0.5;
    return std::make_pair(min_mirror_height_avg, max_mirror_height_avg);
  } else {
    return std::make_pair(std::numeric_limits<double>::infinity(),
                          -std::numeric_limits<double>::infinity());
  }
}

double ComputeCenterMaxCurvature(
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    const VehicleDriveParamsProto& vehicle_drive_params);
double ComputeRelaxedCenterMaxCurvature(
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    const VehicleDriveParamsProto& vehicle_drive_params);
Vec2d ComputeAvFrontLeftCorner(
    const Vec2d& av_pos, double av_theta,
    const VehicleGeometryParamsProto& vehicle_geometry_params);
Vec2d ComputeAvFrontRightCorner(
    const Vec2d& av_pos, double av_theta,
    const VehicleGeometryParamsProto& vehicle_geometry_params);
Vec2d ComputeAvFrontCenter(
    const Vec2d& av_pos, double av_theta,
    const VehicleGeometryParamsProto& vehicle_geometry_params);
Vec2d ComputeAvGeometryCenter(
    const Vec2d& av_pos, double av_theta,
    const VehicleGeometryParamsProto& vehicle_geometry_params);
Vec2d ComputeAvGeometryCenter(
    const Vec2d& av_pos, const Vec2d& av_dir,
    const VehicleGeometryParamsProto& vehicle_geometry_params);

Box2d ComputeAvBox(const Vec2d& av_pos, double av_theta,
                   const VehicleGeometryParamsProto& vehicle_geometry_params);
Box2d ComputeAvBoxWithBuffer(
    const Vec2d& av_xy, double av_theta,
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    double length_buffer, double width_buffer);
Box2d ComputeAvWheelBox(
    const Vec2d& av_pos, double av_theta,
    const VehicleGeometryParamsProto& vehicle_geometry_params);
Box2d ComputeAvWheelBoundingBoxWithBuffer(
    const Vec2d& av_pos, double av_theta,
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    double length_buffer, double width_buffer);
}  // namespace planning
}  // namespace e2e_noa

#endif
