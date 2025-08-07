#include "spline_1d_smoother.h"

#include <cmath>
#include <iostream>

#include "math/spline/spline_1d.h"
namespace ad_e2e {
namespace planning {
bool Spline1dSmoother::Smooth(const double &s_revolution,
                              std::vector<math::Vec2d> *result) {
  result->clear();

  if (x_points_.empty()) {
    std::cout << "no points to smooth !" << std::endl;
    return false;
  }
  if (x_points_.size() != y_points_.size()) {
    std::cout << "Err size of points!" << std::endl;
    return false;
  }
  if (x_points_.size() == 1) {
    result->push_back(math::Vec2d(x_points_.back(), y_points_.back()));
    return true;
  }

  tk::spline spline;
  try {
    spline.set_points(x_points_, y_points_);
  } catch (const std::exception &e) {
    std::cout << "Generate spline y = f(x) Fail!" << std::endl;
    std::cerr << e.what() << '\n';
    return false;
  } catch (...) {
    std::cout << "Generate spline y = f(x) Fail!" << std::endl;
    std::cerr << "e.what()" << '\n';
    return false;
  }

  for (double x = x_points_[0]; x < x_points_.back();) {
    double dy = spline.deriv(1, x);
    double ds = sqrt(1.0 + dy * dy);
    x += s_revolution / ds;
    result->push_back(math::Vec2d(x, spline(x)));
  }
  return true;
}
}  // namespace planning
}  // namespace ad_e2e
