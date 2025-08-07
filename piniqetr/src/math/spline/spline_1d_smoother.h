#ifndef MATH_SPLINE_1D_SMOOTHER_H_
#define MATH_SPLINE_1D_SMOOTHER_H_
#include <vector>

#include "common/type_def.h"
#include "math/vec2d.h"

namespace ad_e2e {
namespace planning {
class Spline1dSmoother {
 public:
  explicit Spline1dSmoother(const std::vector<double> &x_points,
                            const std::vector<double> &y_points)
      : x_points_(x_points), y_points_(y_points) {}
  ~Spline1dSmoother() = default;

  bool Smooth(const double &s_revolution, std::vector<math::Vec2d> *result);

 private:
  std::vector<double> x_points_;
  std::vector<double> y_points_;
};
}  // namespace planning
}  // namespace ad_e2e
#endif
