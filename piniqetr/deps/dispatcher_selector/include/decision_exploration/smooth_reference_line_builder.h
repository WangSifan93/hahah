#ifndef PLANNER_SCHEDULER_SMOOTH_REFERENCE_LINE_BUILDER_H_
#define PLANNER_SCHEDULER_SMOOTH_REFERENCE_LINE_BUILDER_H_

#include <vector>

#include "absl/status/statusor.h"
#include "common/path_sl_boundary.h"
#include "maps/semantic_map_defs.h"
#include "plan/planner_semantic_map_manager.h"
#include "router/plan_passage.h"
#include "router/route_sections.h"
#include "decision_exploration/smooth_reference_line_result.h"

namespace e2e_noa::planning {

absl::StatusOr<SmoothedReferenceCenterResult> SmoothLanePathByLaneIds(
    const PlannerSemanticMapManager& psmm,
    const std::vector<mapping::ElementId>& lane_ids, double half_av_width);

absl::StatusOr<SmoothedReferenceLineResultMap>
BuildSmoothedResultMapFromRouteSections(const PlannerSemanticMapManager& psmm,
                                        double half_av_width,
                                        SmoothedReferenceLineResultMap results);

absl::StatusOr<std::vector<std::vector<mapping::ElementId>>>
FindLanesToSmoothFromRoute(const PlannerSemanticMapManager& psmm);

absl::StatusOr<SmoothedReferenceCenterResult>
SmoothLanePathBoundedByPathBoundary(
    const PlannerSemanticMapManager& psmm, const PlanPassage& plan_passage,
    const PathSlBoundary& boundary,
    const std::vector<mapping::ElementId>& lane_ids, double half_av_width);

}  

#endif
