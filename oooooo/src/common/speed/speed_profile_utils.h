#ifndef PILOT_PLANNING_COMMON_SPEED_SPEED_PROFILE_UTILS_H_
#define PILOT_PLANNING_COMMON_SPEED_SPEED_PROFILE_UTILS_H_
#include "common/type_def.h"

namespace ad_e2e {
namespace planning {
namespace speed_profile_utils {

SpeedPoint CalConstAccSpeedPoint(const SpeedPoint &start, const double a,
                                 const double t);

SpeedPoint CalConstJerkSpeedPoint(const SpeedPoint &start, const double jerk,
                                  const double t);

}  // namespace speed_profile_utils

}  // namespace planning
}  // namespace ad_e2e

#endif
