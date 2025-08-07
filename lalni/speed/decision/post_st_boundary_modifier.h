#ifndef E2E_NOA_PLANNING_SPEED_DECIDER_POST_ST_BOUNDARY_MODIFIER
#define E2E_NOA_PLANNING_SPEED_DECIDER_POST_ST_BOUNDARY_MODIFIER

#include <string>
#include <unordered_map>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "object/spacetime_object_trajectory.h"
#include "object/spacetime_trajectory_manager.h"
#include "speed/decision/st_boundary_modifier_util.h"
#include "speed/st_boundary_with_decision.h"
#include "speed/st_graph.h"
#include "speed_planning.pb.h"

namespace e2e_noa::planning {

struct StBoundaryModificationInfo {
  StBoundaryModifierProto::ModifierType modifier_type =
      StBoundaryModifierProto::UNKNOWN;
  StBoundaryProto::DecisionType decision = StBoundaryProto::UNKNOWN;
  bool is_decision_changed = false;

  std::vector<AccelPoint> accel_point_list;
};

struct PostStboundaryModifierInput {
  const StGraph* st_graph = nullptr;
  const SpacetimeTrajectoryManager* st_traj_mgr = nullptr;
  const absl::flat_hash_map<std::string, StBoundaryModificationInfo>*
      modification_info_map = nullptr;
  const DiscretizedPath* path = nullptr;
};

SpacetimeObjectTrajectory CreateSpacetimeTrajectoryWithAccelPointList(
    absl::Span<const AccelPoint> accel_point_list,
    const SpacetimeObjectTrajectory& st_object);

void PostModifyStBoundaries(
    const PostStboundaryModifierInput& input,
    std::vector<StBoundaryWithDecision>* st_boundaries_wd,
    std::unordered_map<std::string, SpacetimeObjectTrajectory>*
        processed_st_objects);

}  // namespace e2e_noa::planning

#endif
