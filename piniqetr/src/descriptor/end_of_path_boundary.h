#ifndef ONBOARD_PLANNER_DECISION_END_OF_PATH_BOUNDARY_H_
#define ONBOARD_PLANNER_DECISION_END_OF_PATH_BOUNDARY_H_

#include "absl/status/statusor.h"
#include "common/path_sl_boundary.h"
#include "constraint.pb.h"
#include "router/plan_passage.h"

namespace e2e_noa {
namespace planning {

absl::StatusOr<ConstraintProto::StopLineProto> BuildEndOfPathBoundaryConstraint(
    const PlanPassage& passage, const PathSlBoundary& path_boundary);

}
}  // namespace e2e_noa

#endif
