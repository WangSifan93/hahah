#ifndef AD_E2E_PLANNING_UTIL_LANE_POINT_UTIL_H
#define AD_E2E_PLANNING_UTIL_LANE_POINT_UTIL_H

#include "absl/status/statusor.h"
#include "maps/lane_point.h"
#include "maps/semantic_map_defs.h"
#include "math/vec.h"
#include "plan/planner_semantic_map_manager.h"

namespace e2e_noa::planning {

Vec2d ComputeLanePointPos(const PlannerSemanticMapManager& psmm,
                          const mapping::LanePoint& lane_point);

Vec2d ComputeLanePointTangent(const PlannerSemanticMapManager& psmm,
                              const mapping::LanePoint& lane_point);

double ComputeLanePointLerpTheta(const PlannerSemanticMapManager& psmm,
                                 const mapping::LanePoint& lane_point);

double ComputeLanePointLerpThetaWithSuccessorLane(
    const PlannerSemanticMapManager& psmm, mapping::ElementId succ_lane_id,
    const mapping::LanePoint& lane_point);

}  // namespace e2e_noa::planning

#endif
