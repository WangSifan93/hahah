#ifndef ONBOARD_PLANNER_UTIL_SPEED_UTIL_H_
#define ONBOARD_PLANNER_UTIL_SPEED_UTIL_H_

#include "math/piecewise_linear_function.h"

namespace e2e_noa {
namespace planning {
double VelocityCoordinateTransform(
    const PiecewiseLinearFunction<double>& set_speed_bias_plf,
    double set_speed_kph);
}
}  // namespace e2e_noa

#endif
