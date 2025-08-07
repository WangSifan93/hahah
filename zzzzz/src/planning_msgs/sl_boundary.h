/******************************************************************************
 * Copyright 2023 The zpilot Authors Fuhuanhuan. All Rights Reserved.
 *****************************************************************************/
#pragma once
#include <sstream>
#include <string>
#include <vector>

#include "pnc_point.h"
namespace zark {
namespace planning {
// Struct
/////////////////////////////////////////////////////////////////
// The start_s and end_s are longitudinal values.
// start_s <= end_s.
//
//              end_s
//                ^
//                |
//          S  direction
//                |
//            start_s
//
// The start_l and end_l are lateral values.
// start_l <= end_l. Left side of the reference line is positive,
// and right side of the reference line is negative.
//  end_l  <-----L direction---- start_l
/////////////////////////////////////////////////////////////////
class SLBoundary {
 public:
  // Default constructor
  SLBoundary() = default;

  // Getter functions
  double start_s() const { return start_s_; }
  double end_s() const { return end_s_; }
  double start_l() const { return start_l_; }
  double end_l() const { return end_l_; }
  double t() const { return t_; }
  double v() const { return v_; }
  const std::vector<::common::SLPoint>& boundary_point() const {
    return boundary_point_;
  }

  // Setter functions
  void set_start_s(double start_s) { start_s_ = start_s; }
  void set_end_s(double end_s) { end_s_ = end_s; }
  void set_start_l(double start_l) { start_l_ = start_l; }
  void set_end_l(double end_l) { end_l_ = end_l; }
  void set_t(double t) { t_ = t; }
  void set_v(double v) { v_ = v; }
  void set_boundary_point(
      const std::vector<::common::SLPoint>& boundary_point) {
    boundary_point_ = boundary_point;
  }

  std::string ShortDebugString() {
    std::stringstream ss;
    ss << "start_s=" << start_s_ << " end_s= " << end_s_
       << " start_l=" << start_l_ << " end_l=" << end_l_ << " t=" << t_
       << " v=" << v_;
    return ss.str();
  }

  // CopyFrom
  void CopyFrom(const SLBoundary& other) { *this = other; }

  ::common::SLPoint* add_boundary_point() {
    ::common::SLPoint other;
    boundary_point_.emplace_back(other);
    return &boundary_point_.back();
  }

 private:
  double start_s_;
  double end_s_;
  double start_l_;
  double end_l_;
  double t_;
  double v_;
  std::vector<::common::SLPoint> boundary_point_;
};

class SLTrajectory : public std::vector<SLBoundary> {
 public:
  /**
   * @brief Evaluate the sl boundary at a given time 't'.
   *
   * @param t evaluate time t.
   * @return SLBoundary at the given length 't'.
   */
  SLBoundary EvaluateByT(const double t) const;
};

}  // namespace planning
}  // namespace zark
