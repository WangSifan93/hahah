#ifndef PILOT_PLANNING_COMMON_CURVE_BOUNDARY_H_
#define PILOT_PLANNING_COMMON_CURVE_BOUNDARY_H_
#include <map>
#include <vector>

#include "common/type_def.h"
#include "math/line_curve1d.h"

namespace ad_e2e {
namespace planning {
class CurveBoundary1d {
 public:
  CurveBoundary1d() = default;
  ~CurveBoundary1d() = default;

  bool Create(const LineCurve1d &lower_boundary,
              const LineCurve1d &upper_boundary, double s_interval = 0.5);

  void Clear() { boundary_map_.clear(); };
  bool IsValid() const { return !boundary_map_.empty(); }

  void SetBoundary(const double x, const double lower, const double upper);

  bool GetBoundary(const double x, double *lower, double *upper) const;
  const std::map<double, std::pair<double, double>> &GetBoundary() const {
    return boundary_map_;
  }
  bool GetBoundary(std::vector<Point2d> &lower,
                   std::vector<Point2d> &upper) const;

  bool GetRange(double *begin, double *end) const;

 private:
  std::map<double, std::pair<double, double>> boundary_map_;
};
using CurveBoundary1dPtr = std::shared_ptr<CurveBoundary1d>;
}  // namespace planning
}  // namespace ad_e2e
#endif
