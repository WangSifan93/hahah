#ifndef AD_E2E_PLANNING_MATH_LINE_CURVE1D_H_
#define AD_E2E_PLANNING_MATH_LINE_CURVE1D_H_
#include <map>

namespace ad_e2e {
namespace planning {
class LineCurve1d {
 public:
  LineCurve1d() = default;
  ~LineCurve1d() = default;

  void Clear() { curve_map_.clear(); };

  bool IsValid() const { return !curve_map_.empty(); }

  void SetValue(const double x, const double value);

  double GetValue(const double x) const;

  bool GetRange(double *begin, double *end) const;

  std::map<double, double> GetCurveMap() const { return curve_map_; }

 private:
  std::map<double, double> curve_map_;
};
}  // namespace planning
}  // namespace ad_e2e
#endif
