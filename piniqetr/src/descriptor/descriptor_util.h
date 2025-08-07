#ifndef ONBOARD_PLANNER_DECISION_DECISION_UTIL_H_
#define ONBOARD_PLANNER_DECISION_DECISION_UTIL_H_

#include <utility>

#include "absl/types/span.h"
#include "common/path_sl_boundary.h"
#include "constraint.pb.h"
#include "descriptor/constraint_manager.h"
#include "math/frenet_common.h"
#include "object/spacetime_object_trajectory.h"
#include "perception.pb.h"
#include "plan/planner_semantic_map_manager.h"
#include "plan/speed_profile.h"
#include "router/plan_passage.h"

namespace e2e_noa {
namespace planning {

SpeedProfile CreateSpeedProfile(
    double v_now, const PlanPassage& passage,
    const PlannerSemanticMapManager& psmm,
    const absl::Span<const ConstraintProto::SpeedRegionProto>& speed_zones,
    const absl::Span<const ConstraintProto::StopLineProto>& stop_points);

ConstraintProto::SpeedRegionProto MergeSameElement(
    absl::Span<const ConstraintProto::SpeedRegionProto> elements);

void FillDecisionConstraintDebugInfo(const ConstraintManager& constraint_mgr,
                                     ConstraintProto* constraint);

bool IsLeadingObjectType(ObjectType type);

std::pair<double, double> CalcSlBoundaries(const PathSlBoundary& sl_boundary,
                                           const FrenetBox& frenet_box);

ConstraintProto::LeadingObjectProto CreateLeadingObject(
    const SpacetimeObjectTrajectory& traj, const PlanPassage& passage,
    ConstraintProto::LeadingObjectProto::Reason reason,
    bool is_group_tail = false);

bool IsTrafficLightControlledLane(const ad_e2e::planning::Lane& lane);

}  // namespace planning
}  // namespace e2e_noa

#endif
