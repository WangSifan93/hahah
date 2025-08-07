#ifndef ONBOARD_PLANNER_MATH_INTELLIGENT_DRIVER_MODEL_H_
#define ONBOARD_PLANNER_MATH_INTELLIGENT_DRIVER_MODEL_H_

namespace e2e_noa {
namespace planning {

namespace idm {
struct Parameters {
  double v_desire;
  double s_min;
  double t_desire;
  double acc_max;
  double comfortable_brake;
  double brake_max;
  double delta;
};
double ComputeIDMAcceleration(double v, double ds, double dv,
                              const Parameters& params);
}  // namespace idm
}  // namespace planning
}  // namespace e2e_noa

#endif
