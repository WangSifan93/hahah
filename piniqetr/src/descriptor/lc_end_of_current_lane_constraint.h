#ifndef ONBOARD_PLANNER_DECISION_LC_END_OF_CURRENT_LANE_CONSTRAINT_H_
#define ONBOARD_PLANNER_DECISION_LC_END_OF_CURRENT_LANE_CONSTRAINT_H_

#include "absl/status/statusor.h"
#include "constraint.pb.h"
#include "maps/lane_path.h"
#include "router/plan_passage.h"

namespace e2e_noa {
namespace planning {

absl::StatusOr<ConstraintProto::SpeedRegionProto>
BuildLcEndOfCurrentLaneConstraints(const PlanPassage& dp,
                                   const mapping::LanePath& lane_path_before_lc,
                                   double ego_v);

}
}  // namespace e2e_noa

#endif
