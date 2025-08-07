/******************************************************************************
 * Copyright 2024 The zpilot. All Rights Reserved.
 *****************************************************************************/

/**
 * @file sl_boundary.cc
 **/

#include <algorithm>

#include "sl_boundary.h"
#include "linear_interpolation.h"

namespace zark {
namespace planning {

SLBoundary SLTrajectory::EvaluateByT(const double t) const {
  CHECK_GT(size(), 1U);
  auto it_lower = std::lower_bound(
      begin(), end(), t, [](const SLBoundary& sl_boundary, const double t) {
        return sl_boundary.t() < t;
      });
  if (it_lower == begin()) {
    return front();
  } else if (it_lower == end()) {
    return back();
  }
  const auto& sl_boundary_0 = *(it_lower - 1);
  const double t0 = sl_boundary_0.t();
  const auto& sl_boundary_1 = *it_lower;
  const double t1 = sl_boundary_1.t();

  SLBoundary sl_boundary;
  sl_boundary.set_start_s(::math::lerp(sl_boundary_0.start_s(), t0,
                                       sl_boundary_1.start_s(), t1, t));
  sl_boundary.set_end_s(
      ::math::lerp(sl_boundary_0.end_s(), t0, sl_boundary_1.end_s(), t1, t));
  sl_boundary.set_start_l(::math::lerp(sl_boundary_0.start_l(), t0,
                                       sl_boundary_1.start_l(), t1, t));
  sl_boundary.set_end_l(
      ::math::lerp(sl_boundary_0.end_l(), t0, sl_boundary_1.end_l(), t1, t));
  sl_boundary.set_v(
      ::math::lerp(sl_boundary_0.v(), t0, sl_boundary_1.v(), t1, t));

  return sl_boundary;
}

}  // namespace planning
}  // namespace zark
