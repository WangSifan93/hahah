#include "util/speed_util.h"

namespace e2e_noa {
namespace planning {
double VelocityCoordinateTransform(
    const PiecewiseLinearFunction<double>& set_speed_bias_plf,
    double set_speed_kph) {
  const double speed_offset = set_speed_bias_plf(set_speed_kph);
  set_speed_kph += speed_offset;
  double act_speed_mps = (set_speed_kph < 26.0)
                             ? (set_speed_kph / 1.04)
                             : ((set_speed_kph - 0.5) / 1.02);
  act_speed_mps /= 3.6;
  return std::max(0.0, std::min(40.0, act_speed_mps));
}
}  // namespace planning
}  // namespace e2e_noa
