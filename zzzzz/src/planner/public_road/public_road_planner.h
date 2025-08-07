#pragma once

#include <memory>
#include <string>

#include "pnc_point.h"
#include "status.h"
#include "factory.h"
#include "apps/planning/src/common/math/curve1d/quintic_polynomial_curve1d.h"
#include "apps/planning/src/planner/planner.h"
#include "apps/planning/src/planning_msgs/planning.h"
#include "apps/planning/src/planning_msgs/planning_config.h"
#include "apps/planning/src/tasks/task.h"

/**
 * @namespace zark::planning
 * @brief zark::planning
 */
namespace zark {
namespace planning {

/**
 * @class PublicRoadPlanner
 * @brief PublicRoadPlanner is an expectation maximization planner.
 */

class PublicRoadPlanner : public Planner {
 public:
  /**
   * @brief Constructor
   */
  PublicRoadPlanner() = delete;

  explicit PublicRoadPlanner(
      const std::shared_ptr<DependencyInjector>& injector)
      : Planner(injector) {}

  /**
   * @brief Destructor
   */
  virtual ~PublicRoadPlanner() = default;

  void Stop() override {}

  std::string Name() override { return "PUBLIC_ROAD"; }

  Status Init(const PlanningConfig& config) override;

  /**
   * @brief Override function Plan in parent class Planner.
   * @param planning_init_point The trajectory point where planning starts.
   * @param frame Current planning frame.
   * @return OK if planning succeeds; error otherwise.
   */
  Status Plan(const ::common::TrajectoryPoint& planning_init_point,
              Frame* frame, scenario::Scenario* scenario,
              ADCTrajectory& computed_trajectory) override;
};

}  // namespace planning
}  // namespace zark
