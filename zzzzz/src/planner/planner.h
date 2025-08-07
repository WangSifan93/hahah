#pragma once

#include <memory>
#include <string>

#include "pnc_point.h"
#include "apps/planning/src/common/frame.h"
#include "status.h"
#include "apps/planning/src/planning_msgs/planning_config.h"
#include "apps/planning/src/scenarios/scenario.h"
#include "apps/planning/src/scenarios/scenario_manager.h"

/**
 * @namespace zark::planning
 * @brief zark::planning
 */
namespace zark {
namespace planning {

/**
 * @class Planner
 * @brief Planner is a base class for specific planners.
 *        It contains a pure virtual function Plan which must be implemented in
 * derived class.
 */
class Planner {
 public:
  /**
   * @brief Constructor
   */
  Planner() = delete;

  explicit Planner(const std::shared_ptr<DependencyInjector>& injector)
      : scenario_manager_(injector) {}

  /**
   * @brief Destructor
   */
  virtual ~Planner() = default;

  virtual std::string Name() = 0;
  virtual ::common::Status Init(const PlanningConfig& config) = 0;

  /**
   * @brief Compute trajectories for execution.
   * @param planning_init_point The trajectory point where planning starts.
   * @param frame Current planning frame.
   * @return OK if planning succeeds; error otherwise.
   */
  virtual ::common::Status Plan(
      const ::common::TrajectoryPoint& planning_init_point, Frame* frame,
      scenario::Scenario* scenario, ADCTrajectory& computed_trajectory) = 0;

  virtual void Stop() = 0;

 private:
  // std::unordered_map<std::string, Lattice_Debug> lattice_debug_;

 protected:
  PlanningConfig config_{};
  scenario::ScenarioManager scenario_manager_;
  scenario::Scenario* scenario_ = nullptr;
};

}  // namespace planning
}  // namespace zark
