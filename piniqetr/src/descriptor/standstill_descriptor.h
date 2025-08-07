#ifndef ONBOARD_PLANNER_DECISION_STANDSTILL_DECIDER_H_
#define ONBOARD_PLANNER_DECISION_STANDSTILL_DECIDER_H_

#include <vector>

#include "absl/status/statusor.h"
#include "absl/types/span.h"
#include "constraint.pb.h"
#include "router/plan_passage.h"
#include "trajectory_point.pb.h"
#include "vehicle.pb.h"

namespace e2e_noa {
namespace planning {
absl::StatusOr<std::vector<ConstraintProto::StopLineProto>>
BuildStandstillConstraints(
    const e2e_noa::VehicleGeometryParamsProto& vehicle_geometry_params,
    const ApolloTrajectoryPointProto& plan_start_point,
    const PlanPassage& passage,
    absl::Span<const ConstraintProto::StopLineProto> stop_lines);
}
}  // namespace e2e_noa
#endif
