#ifndef ONBOARD_PLANNER_OPTIMIZATION_DDP_TRAJECTORY_OPTIMIZER_DEFS_H_
#define ONBOARD_PLANNER_OPTIMIZATION_DDP_TRAJECTORY_OPTIMIZER_DEFS_H_

#include <limits>

#include "optimization/problem/mixed_fourth_order_bicycle.h"
#include "plan/planner_defs.h"

namespace e2e_noa {
namespace planning {
namespace optimizer {

using Mfob = MixedFourthOrderBicycle;

struct LeadingInfo {
  double s = std::numeric_limits<double>::infinity();
  double v = std::numeric_limits<double>::infinity();
};

}  
}  
}  

#endif
