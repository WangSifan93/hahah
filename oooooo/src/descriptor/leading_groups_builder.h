#ifndef ONBOARD_PLANNER_DECISION_LEADING_GROUPS_BUILDER_H_
#define ONBOARD_PLANNER_DECISION_LEADING_GROUPS_BUILDER_H_

#include <map>
#include <string>
#include <vector>

#include "absl/container/flat_hash_set.h"
#include "common/path_sl_boundary.h"
#include "constraint.pb.h"
#include "math/frenet_common.h"
#include "object/spacetime_trajectory_manager.h"
#include "router/plan_passage.h"
#include "vehicle.pb.h"

namespace e2e_noa::planning {

using LeadingGroup = std::map<std::string, ConstraintProto::LeadingObjectProto>;

std::vector<LeadingGroup> FindMultipleLeadingGroups(
    const PlanPassage& plan_passage, const PathSlBoundary& path_boundary,
    bool lc_left, const SpacetimeTrajectoryManager& st_traj_mgr,
    const absl::flat_hash_set<std::string>& stalled_objects, double ego_heading,
    const FrenetBox& ego_frenet_box,
    const VehicleGeometryParamsProto& vehicle_geom);

}  

#endif
