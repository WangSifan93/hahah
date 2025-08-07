#ifndef PLANNER_SPEED_DECIDER_PRE_BRAKE_UTIL_H_
#define PLANNER_SPEED_DECIDER_PRE_BRAKE_UTIL_H_

#include <string>

#include "speed/vt_speed_limit.h"

namespace e2e_noa {
namespace planning {

VtSpeedLimit GenerateConstAccSpeedLimit(double start_t, double end_t,
                                        double start_v, double min_v,
                                        double max_v, double acc,
                                        double time_step, int step_num,
                                        const std::string& info);

}
}  // namespace e2e_noa
#endif
