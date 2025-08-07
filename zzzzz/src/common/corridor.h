/******************************************************************************
 * Copyright 2024 The zpilot. All Rights Reserved.
 *****************************************************************************/

/**
 * @file corridor.h
 **/

#pragma once

#include <vector>

#include "pnc_point.h"
#include "cartesian_frenet_conversion.h"
#include "vec2d.h"

namespace zark {
namespace planning {

struct CorridorPoint {
  enum class Type {
    UNKNOWN = 0,
    DASHED_LINE = 1,
    SOLID_LINE = 2,
    CURB = 3,
    OTHERS = 4
  };
  double s_ref;            // s in the Frenet frame of local route [m]
  double l_ref;            // l in the Frenet frame of local route [m]
  double s;                // s in the Frenet frame of corridor [m]
  double l;                // l in the Frenet frame of corridor [m]
  double l_left;           // l_left in the Frenet frame of corridor [m]
  double l_right;          // l_right in the Frenet frame of corridor [m]
  ::math::Vec2d xy_ref;    // [m]
  ::math::Vec2d xy_left;   // [m]
  ::math::Vec2d xy_right;  // [m]
  double theta;            // [rad]
  double kappa;            // [m^-1]
  double v;                // v in Cartesian frame [m/s]
  double a;                // a in Cartesian frame [m/s^2]
  double t;                // [s]
  Type type_left;
  Type type_right;
};

/**
 * class Corridor
 * @brief Corridor built based on CorridorPoint.
 */
class Corridor : public std::vector<CorridorPoint> {
 public:
  Corridor() = default;

  explicit Corridor(const std::vector<CorridorPoint>& points);

  /**
   * @brief Convert a point from Cartesian to Frenet.
   *
   * @param xy_point Point in Cartesian.
   * @param sl_point Result of conversion in Frenet.
   * @return bool Validation of Conversion.
   */
  bool XYToSL(const ::math::Vec2d& xy_point, ::common::SLPoint& sl_point) const;

  /**
   * @brief Convert a point from Frenet to Cartesian.
   *
   * @param sl_point Point in Frenet.
   * @param xy_point Result of conversion in Cartesian.
   * @return bool Validation of Conversion.
   */

  bool SLToXY(const ::common::SLPoint& sl_point,
              ::math::Vec2d* const xy_point) const;
  /**
   * @brief Evaluate the corridor point at a given length 's'.
   *
   * @param s Corridorpoint in s.
   * @return CorridorPoint at the given length 's'.
   */
  CorridorPoint EvaluateByS(const double s) const;

  /**
   * @brief Convert a trajectory point to frenet frame.
   *
   * @param traj_point Trajectory point.
   * @return FrenetPoint Frenet point.
   */
  ::common::FrenetPoint ToFrenetFrame(
      const ::common::TrajectoryPoint& traj_point) const;

 private:
  /**
   * @brief Initialization.
   */
  void Init();

  /**
   * @brief Get index of point with input s.
   *
   * @param s Position s.
   * @return std::pair<int, double> Index of point and offset of s.
   */
  std::pair<int, double> GetIndexFromS(double s) const;

  /**
   * @brief Get reference point with input s.
   *
   * @param s Position s.
   * @return CorridorPoint Point at s.
   */
  CorridorPoint GetReferencePoint(const double s) const;

 private:
  int num_points_ = 0;
  int num_sample_points_ = 0;
  double length_ = 0.0;
  std::vector<int> last_point_index_;
};

}  // namespace planning
}  // namespace zark
