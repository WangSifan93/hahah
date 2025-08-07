#ifndef ONBOARD_PLANNER_UTIL_SCENE_UTIL_H_
#define ONBOARD_PLANNER_UTIL_SCENE_UTIL_H_

#include "plan/planner_semantic_map_manager.h"
#include "router/route_sections.h"

namespace e2e_noa {
namespace planning {

bool IsInHighWay(const PlannerSemanticMapManager& psmm,
                 const RouteSections& sections, const double preview_distance);

}
}  // namespace e2e_noa

#endif
