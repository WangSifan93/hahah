#ifndef AD_E2E_PLANNING_MATH_LINE_CURVE_H
#define AD_E2E_PLANNING_MATH_LINE_CURVE_H

#include <iostream>

#include "common/type_def.h"
#include "math/line_segment2d.h"

namespace ad_e2e {
namespace planning {
namespace math {

class LineCurve2d {
 public:
  LineCurve2d() = default;
  explicit LineCurve2d(const std::vector<Vec2d> &points);

  void InitializePoints(const std::vector<Vec2d> &points);
  Vec2d begin_point() const;
  Vec2d end_point() const;
  const std::vector<Vec2d> &points() const { return points_; }
  double length() const {
    return accumulated_s_.empty() ? 0.0 : accumulated_s_.back();
  }

  const std::vector<double> &GetAccuLength() const { return accumulated_s_; }

  bool IsValid() const { return points_.size() >= 2; }

  void Clear();

  Vec2d GetPointAtS(const double s) const;

  int GetIndexByS(const double s) const;

  bool SamplePoints(const double start_s, const double length,
                    std::vector<Vec2d> *const points,
                    const double interval = 3.0) const;

  double GetDistance(const Vec2d &point, Vec2d *nearest_point,
                     double *s_offset) const;

  double GetDistance(const double x, const double y) const;

  double GetDistance(const Vec2d &point) const;

  bool GetProjection(const Vec2d &point, double *accumulate_s,
                     double *lateral) const;

  bool GetPoint(const double s, const double l, Vec2d *const point) const;

  bool GetHeadingFromS(const double s, double *const heading) const;

 private:
  std::vector<double> accumulated_s_;
  std::vector<double> headings_;
  std::vector<Vec2d> points_;
};

}  // namespace math
}  // namespace planning
}  // namespace ad_e2e

#endif
