#ifndef ONBOARD_PLANNER_DECISION_DECIDER_OUTPUT_H_
#define ONBOARD_PLANNER_DECISION_DECIDER_OUTPUT_H_

#include <optional>
#include <vector>

#include "common/type_def.h"
#include "constraint.pb.h"
#include "descriptor/constraint_manager.h"

namespace e2e_noa::planning {

struct Descriptor {
  ConstraintManager constraint_manager;
  DeciderStateProto decision_state;
  std::optional<double> distance_to_traffic_light_stop_line = std::nullopt;
  int tl_stop_interface = 0;
  ad_e2e::planning::SpeedState speed_state;
};

}  // namespace e2e_noa::planning

#endif
