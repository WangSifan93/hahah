#pragma once

#include <memory>

#include "factory.h"
#include "apps/planning/src/planner/planner_dispatcher.h"

/**
 * @namespace zark::planning
 * @brief zark::planning
 */
namespace zark {
namespace planning {

/**
 * @class planning
 *
 * @brief PlannerDispatcher module main class.
 */
class OnLanePlannerDispatcher final : public PlannerDispatcher {
 public:
  OnLanePlannerDispatcher() = default;
  virtual ~OnLanePlannerDispatcher() = default;

  std::unique_ptr<Planner> DispatchPlanner(
      const PlanningConfig& planning_config,
      const std::shared_ptr<DependencyInjector>& injector) override;
};

}  // namespace planning
}  // namespace zark
