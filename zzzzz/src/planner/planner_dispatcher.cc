

#include <memory>
#include "apps/planning/src/planner/planner_dispatcher.h"
#include "apps/planning/src/planner/public_road/public_road_planner.h"
#include "apps/planning/src/planning_msgs/planning_config.h"

namespace zark {
namespace planning {

void PlannerDispatcher::RegisterPlanners() {
  planner_factory_.Register(
      PlannerType::PUBLIC_ROAD,
      [](const std::shared_ptr<DependencyInjector>& injector) -> Planner* {
        return new PublicRoadPlanner(injector);
      });
}

}  // namespace planning
}  // namespace zark
