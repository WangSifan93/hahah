#ifndef AD_E2E_PLANNING_MATH_CURVE_BOUNDARY_1D_H_
#define AD_E2E_PLANNING_MATH_CURVE_BOUNDARY_1D_H_
#include <map>
#include <vector>

#include "common/type_def.h"
#include "math/line_curve1d.h"

namespace ad_e2e {
namespace planning {
class CurveLimit {
 public:
  CurveLimit() = default;
  ~CurveLimit() = default;

  bool Create(const LineCurve1d &lower_boundary,
              const LineCurve1d &upper_boundary, double s_interval = 0.5);

  void Clear() { boundary_map_.clear(); };
  bool IsValid() const { return !boundary_map_.empty(); }

  void SetBoundary(const double &x, const double &lower, const double &upper);

  bool GetBoundary(const double &x, double *lower, double *upper) const;
  const std::map<double, std::pair<double, double>> &GetBoundary() const {
    return boundary_map_;
  }
  bool GetBoundary(std::vector<double> &x, std::vector<double> &lower,
                   std::vector<double> &upper) const;
  bool GetBoundary(std::vector<ad_e2e::planning::Point2d> &lower,
                   std::vector<ad_e2e::planning::Point2d> &upper) const;
  bool GetBoundary(std::map<double, double> &left_limit,
                   std::map<double, double> &right_limit) const;

  bool GetRange(double *begin, double *end) const;

 private:
  std::map<double, std::pair<double, double>> boundary_map_;
};
using CurveLimitPtr = std::shared_ptr<CurveLimit>;
}  // namespace planning
}  // namespace ad_e2e
#endif
