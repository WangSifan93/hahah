/******************************************************************************
 * Copyright 2023 The zpilot Authors Fuhuanhuan. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "apps/planning/src/common/dependency_injector.h"
#include "apps/planning/src/common/frame.h"
#include "apps/planning/src/common/local_view.h"
#include "apps/planning/src/common/trajectory/publishable_trajectory.h"
#include "apps/planning/src/common/vehicle_state/vehicle_state_provider.h"
#include "apps/planning/src/planner/planner.h"
#include "apps/planning/src/planner/planner_dispatcher.h"
#include "apps/planning/src/planning_msgs/planning.h"
#include "apps/planning/src/planning_msgs/planning_config.h"
#include "pnc_point.h"
#include "status.h"

using namespace ::common;

/**
 * @namespace zark::planning
 * @brief zark::planning
 */
namespace zark {
namespace planning {
/**
 * @class planning
 *
 * @brief PlanningBase module main class.
 */
class PlanningBase {
 public:
  PlanningBase() = delete;

  explicit PlanningBase(const std::shared_ptr<DependencyInjector>& injector);

  virtual ~PlanningBase();

  virtual Status Init(const PlanningConfig& config);

  virtual std::string Name() const = 0;

  virtual void RunOnce(const LocalView& local_view,
                       ADCTrajectory& adc_trajectory) = 0;

  /**
   * @brief Plan the trajectory given current vehicle state
   */
  virtual Status Plan(const double current_time_stamp,
                      const TrajectoryPoint& planning_start_point,
                      ADCTrajectory& trajectory, const bool need_replan) = 0;

  virtual int64_t getPlanTimeStamp() {
    std::lock_guard<std::mutex> lock(timestamp_lock_);
    return localitzation_timestamp_;
  }

  virtual void GetLocalRoutes(
      const ::common::TrajectoryPoint& planning_start_point,
      const bool is_acc_mode,
      std::list<zark::planning::LocalRoute>& local_routes) const = 0;

 protected:
  LocalView local_view_;

  double start_time_ = 0.0;
  size_t seq_num_ = 0;
  int64_t localitzation_timestamp_ = 0;
  std::mutex timestamp_lock_;

  PlanningConfig config_{};
  std::unique_ptr<Frame> frame_;
  std::unique_ptr<Planner> planner_;
  std::unique_ptr<PublishableTrajectory> last_publishable_trajectory_;
  std::unique_ptr<PlannerDispatcher> planner_dispatcher_;
  std::shared_ptr<DependencyInjector> injector_;
};

}  // namespace planning
}  // namespace zark
