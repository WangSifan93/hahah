/******************************************************************************
 * Copyright 2024 The zpilot. All Rights Reserved.
 *****************************************************************************/

/**
 * @file conversion.h
 **/

#pragma once

#include <cmath>
#include <utility>

#include "box2d.h"
#include "apps/planning/src/common/vehicle_config.h"
#include "pnc_point.h"

namespace zark {
namespace planning {
namespace common {

/**
 * @brief convert ego position from rear axle to center gravity
 * @param x rear axle center position x
 * @param y rear axle center position y
 * @param theta ego heading
 * @param d_rear_axle_to_cg distance from rear axle to center gravity
 * @return center gravity position
 */
inline std::pair<double, double> ConvertFromRearAxleToCG(
    const double x, const double y, const double theta,
    const double d_rear_axle_to_cg) {
  return std::make_pair(x + d_rear_axle_to_cg * cos(theta),
                        y + d_rear_axle_to_cg * sin(theta));
}

/**
 * @brief convert a trajectory point from rear axle to center of gravity
 * @param d_rear_axle_to_cg distance from rear axle to center gravity
 * @param point the trajectory point to be updated
 * @return
 */
inline void ConvertFromRearAxleToCG(const double d_rear_axle_to_cg,
                                    ::common::TrajectoryPoint& point) {
  ::common::PathPoint* path_point = point.mutable_path_point();
  double x;
  double y;
  std::tie(x, y) = ConvertFromRearAxleToCG(
      path_point->x(), path_point->y(), path_point->theta(), d_rear_axle_to_cg);
  path_point->set_x(x);
  path_point->set_y(y);
}

/**
 * @brief convert ego position from center gravity to rear axle
 * @param x center gravity position x
 * @param y center gravity position y
 * @param theta ego heading
 * @param d_rear_axle_to_cg distance from rear axle to center gravity
 * @return rear axle position
 */
inline std::pair<double, double> ConvertFromCGToRearAxle(
    const double x, const double y, const double theta,
    const double d_rear_axle_to_cg) {
  return std::make_pair(x - d_rear_axle_to_cg * cos(theta),
                        y - d_rear_axle_to_cg * sin(theta));
}

/**
 * @brief convert a trajectory point from center gravity to rear axle
 * @param d_rear_axle_to_cg distance from rear axle to center gravity
 * @param point the trajectory point to be updated
 * @return
 */
inline void ConvertFromCGToRearAxle(const double d_rear_axle_to_cg,
                                    ::common::TrajectoryPoint& point) {
  ::common::PathPoint* path_point = point.mutable_path_point();
  double x;
  double y;
  std::tie(x, y) = ConvertFromCGToRearAxle(
      path_point->x(), path_point->y(), path_point->theta(), d_rear_axle_to_cg);
  path_point->set_x(x);
  path_point->set_y(y);
}

/**
 * @brief convert ego position from centroid to center gravity
 * @param x centroid position x
 * @param y centroid position y
 * @param theta ego heading
 * @param d_centroid_to_cg distance from centroid to center gravity
 * @return center gravity position
 */
inline std::pair<double, double> ConvertFromCentroidToCG(
    const double x, const double y, const double theta,
    const double d_centroid_to_cg) {
  return std::make_pair(x + d_centroid_to_cg * cos(theta),
                        y + d_centroid_to_cg * sin(theta));
}

/**
 * @brief convert ego position from center gravity to centroid
 * @param x center gravity position x
 * @param y center gravity position y
 * @param theta ego heading
 * @param d_centroid_to_cg distance from centroid to center gravity
 * @return centroid position
 */
inline std::pair<double, double> ConvertFromCGToCentroid(
    const double x, const double y, const double theta,
    const double d_centroid_to_cg) {
  return std::make_pair(x - d_centroid_to_cg * cos(theta),
                        y - d_centroid_to_cg * sin(theta));
}

/**
 * @brief calculate corner points with ego center
 * @param x_cg ego CG position x
 * @param y_cg ego CG position y
 * @param heading_cg ego CG heading
 * @param ego_param ego body param
 * @return return ego Centroid box
 */
inline ::math::Box2d CalculateCenterBoxFromCGPose(const double x_cg,
                                                  const double y_cg,
                                                  const double heading_cg,
                                                  const VehicleParam& param) {
  double distance_CG_centroid =
      param.rear_axle_to_cg() -
      (param.length() / 2.0 - param.back_edge_to_center());
  const auto& convert_point =
      ConvertFromCGToCentroid(x_cg, y_cg, heading_cg, distance_CG_centroid);
  ::math::Vec2d center(convert_point.first, convert_point.second);
  return ::math::Box2d(center, heading_cg, param.length(), param.width());
}

}  // namespace common
}  // namespace planning
}  // namespace zark
