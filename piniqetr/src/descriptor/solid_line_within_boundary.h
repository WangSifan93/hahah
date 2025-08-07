#ifndef ONBOARD_PLANNER_DECISION_SOLID_LINE_WITHIN_BOUNDARY_H_
#define ONBOARD_PLANNER_DECISION_SOLID_LINE_WITHIN_BOUNDARY_H_

#include <vector>

#include "absl/status/statusor.h"
#include "common/path_sl_boundary.h"
#include "constraint.pb.h"
#include "router/plan_passage.h"
#include "trajectory_point.pb.h"

namespace e2e_noa {
namespace planning {

absl::StatusOr<std::vector<ConstraintProto::AvoidLineProto>>
BuildSolidLineWithinBoundaryConstraint(
    const PlanPassage& plan_passage, const PathSlBoundary& path_boundary,
    const ApolloTrajectoryPointProto& plan_start_point);

}
}  // namespace e2e_noa

#endif
