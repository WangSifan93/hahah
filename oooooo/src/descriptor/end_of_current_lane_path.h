#ifndef ONBOARD_PLANNER_DECISION_END_OF_CURRENT_LANE_PATH_H_
#define ONBOARD_PLANNER_DECISION_END_OF_CURRENT_LANE_PATH_H_

#include "absl/status/statusor.h"
#include "constraint.pb.h"
#include "router/plan_passage.h"

namespace e2e_noa {
namespace planning {

absl::StatusOr<ConstraintProto::StopLineProto>
BuildEndOfCurrentLanePathConstraint(const PlanPassage& passage);

}
}  // namespace e2e_noa

#endif
