#ifndef SPEED_VT_SPEED_LIMIT_H_
#define SPEED_VT_SPEED_LIMIT_H_

#include <vector>

#include "speed/speed_limit.h"

namespace e2e_noa::planning {
using VtSpeedLimit = std::vector<SpeedLimit::SpeedLimitInfo>;

void MergeVtSpeedLimit(const VtSpeedLimit& source, VtSpeedLimit* target);

}  // namespace e2e_noa::planning

#endif
