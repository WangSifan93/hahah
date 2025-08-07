

#include <memory>
#include "apps/planning/src/planner/on_lane_planner_dispatcher.h"
#include "apps/planning/src/planning_msgs/planning_config.h"

namespace zark {
namespace planning {

std::unique_ptr<Planner> OnLanePlannerDispatcher::DispatchPlanner(
    const PlanningConfig& planning_config,
    const std::shared_ptr<DependencyInjector>& injector) {
  return planner_factory_.CreateObject(
      planning_config.standard_planning_config().planner_type().at(0),
      injector);
}

}  // namespace planning
}  // namespace zark
