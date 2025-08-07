#ifndef DECISION_CAUTIOUS_BRAKE_H_
#define DECISION_CAUTIOUS_BRAKE_H_

#include <vector>

#include "constraint.pb.h"
#include "maps/lane_path.h"
#include "object/spacetime_trajectory_manager.h"
#include "plan/planner_semantic_map_manager.h"
#include "router/plan_passage.h"

namespace e2e_noa {
namespace planning {

std::vector<ConstraintProto::SpeedRegionProto> BuildCautiousBrakeConstraints(
    const PlannerSemanticMapManager& planner_semantic_map_manager,
    const PlanPassage& passage, const mapping::LanePath& lane_path_from_start,
    double s_offset, const SpacetimeTrajectoryManager& st_traj_mgr);

}
}  // namespace e2e_noa

#endif
