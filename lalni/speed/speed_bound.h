#ifndef PLANNER_SPEED_SPEED_BOUND_H_
#define PLANNER_SPEED_SPEED_BOUND_H_

#include <map>
#include <string>
#include <vector>

#include "speed_planning.pb.h"

namespace e2e_noa::planning {

struct SpeedBoundWithInfo {
  double bound = 0.0;
  std::string info;
};
using SpeedBoundMapType =
    std::map<SpeedLimitTypeProto::Type, std::vector<SpeedBoundWithInfo>>;

}  // namespace e2e_noa::planning

#endif
