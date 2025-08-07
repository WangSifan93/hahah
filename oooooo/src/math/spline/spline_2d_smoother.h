#ifndef MATH_SPLINE_2D_SMOOTHER_H_
#define MATH_SPLINE_2D_SMOOTHER_H_
#include <vector>

#include "math/vec2d.h"

namespace ad_e2e {
namespace planning {
class Spline2dSmoother {
 public:
  explicit Spline2dSmoother(const std::vector<double> &s_knots,
                            const std::vector<double> &x_points,
                            const std::vector<double> &y_points)
      : s_knots_(s_knots), x_points_(x_points), y_points_(y_points) {}
  ~Spline2dSmoother() = default;

  bool Smooth(const double &s_revolution, std::vector<math::Vec2d> *result);

 private:
  std::vector<double> s_knots_;
  std::vector<double> x_points_;
  std::vector<double> y_points_;
};
}  // namespace planning
}  // namespace ad_e2e
#endif
