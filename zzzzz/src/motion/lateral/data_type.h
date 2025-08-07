/******************************************************************************
 * Copyright 2024 The zpilot. All Rights Reserved.
 *****************************************************************************/

/**
 * @file data_type.h
 **/

#pragma once

#include <vector>

#include "vec2d.h"
#include "apps/planning/src/common/obstacle.h"

namespace zark {
namespace planning {

constexpr int kIdxL = 0;
constexpr int kIdxLDot = 1;
constexpr int kIdxPsiS = 2;
constexpr int kIdxPsiSDot = 3;
constexpr int kIdxDelta = 0;
constexpr int kIdxDeltaDot = 0;

/**
 * struct Tube
 * @brief The struct to store tube information.
 */
struct Tube {
  struct TubePoint {
    enum class TubeType { NORMAL = 0, LARGE = 1, CURB = 2 };
    double t;                      // [s]
    double s;                      // w.r.t. the Frenet frame of corridor [m]
    double l_left_ref;             // w.r.t. the Frenet frame of corridor [m]
    double l_right_ref;            // w.r.t. the Frenet frame of corridor [m]
    double l_left_soft;            // w.r.t. the Frenet frame of corridor [m]
    double l_right_soft;           // w.r.t. the Frenet frame of corridor [m]
    double l_left_stiff;           // w.r.t. the Frenet frame of corridor [m]
    double l_right_stiff;          // w.r.t. the Frenet frame of corridor [m]
    double l_left_hard;            // w.r.t. the Frenet frame of corridor [m]
    double l_right_hard;           // w.r.t. the Frenet frame of corridor [m]
    ::math::Vec2d xy_left_ref;     // [m]
    ::math::Vec2d xy_right_ref;    // [m]
    ::math::Vec2d xy_left_soft;    // [m]
    ::math::Vec2d xy_right_soft;   // [m]
    ::math::Vec2d xy_left_stiff;   // [m]
    ::math::Vec2d xy_right_stiff;  // [m]
    ::math::Vec2d xy_left_hard;    // [m]
    ::math::Vec2d xy_right_hard;   // [m]
  };
  std::vector<TubePoint> pts;
};

struct Nudger {
  Nudger(const int n_nodes) {
    l = std::vector<double>(n_nodes, std::numeric_limits<double>::quiet_NaN());
    v = std::vector<double>(n_nodes, std::numeric_limits<double>::quiet_NaN());
  }
  std::vector<double> l;
  std::vector<double> v;
  bool is_left;
  const Obstacle* obs;
};

}  // namespace planning
}  // namespace zark
