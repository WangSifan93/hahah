#ifndef ONBOARD_PLANNER_DECISION_SPEED_BUMP_H_
#define ONBOARD_PLANNER_DECISION_SPEED_BUMP_H_

#include <vector>

#include "constraint.pb.h"
#include "plan/planner_semantic_map_manager.h"
#include "router/plan_passage.h"

namespace e2e_noa {
namespace planning {

std::vector<ConstraintProto::SpeedRegionProto> BuildSpeedBumpConstraints(
    const PlannerSemanticMapManager& psmm, const PlanPassage& passage);

}
}  // namespace e2e_noa

#endif
