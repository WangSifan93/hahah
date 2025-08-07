/******************************************************************************
 * Copyright 2023 The zpilot Author Fuhuanhuan. All Rights Reserved.
 *****************************************************************************/

/**
 * @file
 **/

#pragma once

#include <memory>
#include <string>
#include <unordered_map>

#include "apps/planning/src/common/frame.h"
#include "status.h"
#include "factory.h"
#include "apps/planning/src/planning_msgs/planning_config.h"
#include "apps/planning/src/scenarios/stage.h"
#include "apps/planning/src/tasks/task.h"

namespace zark {
namespace planning {
namespace scenario {

struct ScenarioContext {};

class Scenario {
 public:
  enum ScenarioStatus {
    STATUS_UNKNOWN = 0,
    STATUS_PROCESSING = 1,
    STATUS_DONE = 2,
  };

  Scenario(const ScenarioConfig& config, const ScenarioContext* context,
           const std::shared_ptr<DependencyInjector>& injector);

  static bool LoadConfig(const std::string& config_file, ScenarioConfig* config,
                         planning::PlanningConfig planning_config);

  virtual ~Scenario() = default;

  ScenarioType scenario_type() const { return config_.scenario_type(); }

  /**
   * Each scenario should define its own stages object's creation
   * scenario will call stage's Stage::Process function following a configured
   * order, The return value of Stage::Process function determines the
   * transition from one stage to another.
   */
  virtual std::unique_ptr<Stage> CreateStage(
      const ScenarioConfig::StageConfig& stage_config,
      const std::shared_ptr<DependencyInjector>& injector) = 0;

  // Each scenario should define its own transfer condition, i.e., when it
  // should allow to transfer from other scenario to itself.
  virtual bool IsTransferable(const Scenario& other_scenario,
                              const Frame& frame) {
    return true;
  }

  virtual ScenarioStatus Process(
      const ::common::TrajectoryPoint& planning_init_point, Frame* frame);

  const ScenarioStatus& GetStatus() const { return scenario_status_; }

  const StageType GetStage() const {
    return current_stage_ ? current_stage_->stage_type() : StageType::NO_STAGE;
  }

  virtual void Init();

  const std::string& Name() const;
  const std::string& GetMsg() const { return msg_; }
  // static bool LoadConfigFromFile(const std::string& config_file,
  //                                ScenarioConfig* config);

 protected:
  ScenarioStatus scenario_status_ = STATUS_UNKNOWN;
  std::unique_ptr<Stage> current_stage_;
  ScenarioConfig config_;
  std::unordered_map<StageType, const ScenarioConfig::StageConfig*,
                     std::hash<int>>
      stage_config_map_;
  const ScenarioContext* scenario_context_ = nullptr;
  std::string name_;
  std::string msg_;  // debug msg
  std::shared_ptr<DependencyInjector> injector_;
};

}  // namespace scenario
}  // namespace planning
}  // namespace zark
