#pragma once

#include <memory>

#include "status.h"
#include "factory.h"
#include "apps/planning/src/planner/planner.h"

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
class PlannerDispatcher {
 public:
  PlannerDispatcher() = default;
  virtual ~PlannerDispatcher() = default;

  virtual Status Init() {
    RegisterPlanners();
    return Status::OK();
  }

  virtual std::unique_ptr<Planner> DispatchPlanner(
      const PlanningConfig& planning_config,
      const std::shared_ptr<DependencyInjector>& injector) = 0;

 protected:
  void RegisterPlanners();

  ::util::Factory<
      PlannerType, Planner,
      Planner* (*)(const std::shared_ptr<DependencyInjector>& injector)>
      planner_factory_;
};

}  // namespace planning
}  // namespace zark
