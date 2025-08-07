/******************************************************************************
 * Copyright 2023 The zpilot Authors Fuhuanhuan. All Rights Reserved.
 *****************************************************************************/
#pragma once

#include <malloc.h>

#include <string>
#include <vector>

#include "geometry.h"
#include "apps/planning/src/planning_msgs/scenario_type.h"
#include "apps/planning/src/planning_msgs/task_config.h"

namespace zark {
namespace planning {

class TaskConfig {
 public:
  TaskConfig() = default;
  enum TaskType {
    TRAFFIC_RULE_DECIDER = 39,
    MISSION_DECIDER = 40,
    LATERAL_DECIDER = 41,
    LONGITUDINAL_DECIDER = 42,
    LONGITUDINAL_OPTIMIZER = 43,
    LATERAL_OPTIMIZER = 44,
    EVALUATION_DECIDER = 45,
  };

  static std::string TaskType_Name(TaskType type) {
    std::string res = "";  // TODO need add
    switch (type) {
      case TRAFFIC_RULE_DECIDER:
        res = "TRAFFIC_RULE_DECIDER";
        break;
      case MISSION_DECIDER:
        res = "MISSION_DECIDER";
        break;
      case LATERAL_DECIDER:
        res = "LATERAL_DECIDER";
        break;
      case LONGITUDINAL_DECIDER:
        res = "LONGITUDINAL_DECIDER";
        break;
      case LONGITUDINAL_OPTIMIZER:
        res = "LONGITUDINAL_OPTIMIZER";
        break;
      case LATERAL_OPTIMIZER:
        res = "LATERAL_OPTIMIZER";
        break;
      case EVALUATION_DECIDER:
        res = "EVALUATION_DECIDER";
        break;
      default:
        break;
    }
    return res;
  }

  // oneof
  struct task_config_union {
    TrafficRuleDeciderConfig traffic_rule_decider_config;
    MissionDeciderConfig mission_decider_config;
    LateralDeciderConfig lateral_decider_config;
    LongitudinalDeciderConfig longitudinal_decider_config;
    LongitudinalOptimizerConfig longitudinal_optimizer_config;
    LateralOptimizerConfig lateral_optimizer_config;
    EvaluationDeciderConfig evaluation_decider_config;
  };
  // oneof

  TaskType task_type() const { return task_type_; }
  void set_task_type(TaskType type) { task_type_ = type; }

  task_config_union task_config() const { return task_config_; }

  void set_task_config(task_config_union conf) { task_config_ = conf; }

  task_config_union get_task_config() { return task_config_; }

  void MergeFrom(const TaskConfig &config) {  // TODO check is right?
    task_type_ = config.task_type();
    task_config_ = config.task_config();
  }

  std::string DebugString() const { return "DebugString!"; }

 private:
  TaskType task_type_;

  task_config_union task_config_;
};

// scenario configs
class ScenarioConfig {
 public:
  class StageConfig {
   public:
    StageConfig() : enabled_(true) {}

    StageType stage_type() const { return stage_type_; }
    void set_stage_type(StageType type) { stage_type_ = type; }

    bool enabled() const { return enabled_; }
    void set_enabled(bool value) { enabled_ = value; }

    const std::vector<TaskConfig::TaskType> &task_type() const {
      return task_type_;
    }
    void set_task_type(const std::vector<TaskConfig::TaskType> &types) {
      task_type_ = types;
    }
    void add_task_type(const TaskConfig::TaskType type) {
      task_type_.emplace_back(type);
    }

    const std::vector<TaskConfig> &task_config() const { return task_config_; }
    void set_task_config(const std::vector<TaskConfig> &config) {
      task_config_ = config;
    }
    void add_task_config(const TaskConfig config) {
      task_config_.emplace_back(config);
    }

   private:
    StageType stage_type_{};
    bool enabled_;  // [default = true]
    std::vector<TaskConfig::TaskType> task_type_;
    std::vector<TaskConfig> task_config_;
  };

  ScenarioType scenario_type() const { return scenario_type_; }
  void set_scenario_type(ScenarioType type) { scenario_type_ = type; }

  // Add setter and getter functions for other scenario configs
  const std::vector<StageType> &stage_type() const { return stage_type_; }

  void set_stage_type(const std::vector<StageType> &types) {
    stage_type_ = types;
  }

  void add_stage_type(const StageType type) { stage_type_.emplace_back(type); }

  void clear_stage_type() { stage_type_.clear(); }

  const std::vector<StageConfig> &stage_config() const { return stage_config_; }

  void set_stage_config(const std::vector<StageConfig> &config) {
    stage_config_ = config;
  }
  void add_stage_config(const StageConfig &config) {
    stage_config_.emplace_back(config);
  }

  void clear_stage_config() { stage_config_.clear(); }

  // oneof
  struct scenario_config_union {};
  // oneof

  scenario_config_union scenario_config() const { return scenario_config_; }
  void set_scenario_config(scenario_config_union &conf) {
    scenario_config_ = conf;
  }

 private:
  ScenarioType scenario_type_;

  // Add member variables for other scenario configs
  std::vector<StageType> stage_type_;
  std::vector<StageConfig> stage_config_;
  scenario_config_union scenario_config_;
};

class PlannerPublicRoadConfig {
 public:
  PlannerPublicRoadConfig() = default;
};

enum PlannerType {
  PUBLIC_ROAD = 0,
};

class StandardPlanningConfig {
 public:
  StandardPlanningConfig() {}

  const std::vector<PlannerType> &planner_type() const { return planner_type_; }
  void set_planner_type(const std::vector<PlannerType> &types) {
    planner_type_ = types;
  }

  void add_planner_type(PlannerType planner_type) {
    planner_type_.emplace_back(planner_type);
  }

  const PlannerPublicRoadConfig &planner_public_road_config() const {
    return planner_public_road_config_;
  }
  void set_planner_public_road_config(const PlannerPublicRoadConfig &config) {
    planner_public_road_config_ = config;
  }
  bool has_planner_public_road_config() const {
    return planner_type_.size() > 0;
  }

 private:
  std::vector<PlannerType> planner_type_;
  PlannerPublicRoadConfig planner_public_road_config_;
};

class PlanningConfig {
 public:
  const StandardPlanningConfig &standard_planning_config() const {
    return standard_planning_config_;
  }
  void set_standard_planning_config(const StandardPlanningConfig &config) {
    standard_planning_config_ = config;
  }

  const bool &tranform_traj_to_body_frame() const {
    return tranform_traj_to_body_frame_;
  }
  void set_tranform_traj_to_body_frame(
      const bool &tranform_traj_to_body_frame) {
    tranform_traj_to_body_frame_ = tranform_traj_to_body_frame;
  }

  const std::vector<TaskConfig> &default_task_config() const {
    return default_task_config_;
  }

  void add_default_task_config(TaskConfig task_config) {
    default_task_config_.emplace_back(task_config);
  }

  void set_default_task_config(const std::vector<TaskConfig> &config) {
    default_task_config_ = config;
  }

  const bool has_standard_planning_config() const {
    if (standard_planning_config_.planner_type().size() > 0) {
      return true;
    }
    return false;
  }

  // oneof start
  struct planning_config_union {
    StandardPlanningConfig standard_planning_config;
  };
  // one of end

  planning_config_union planning_config() const { return planning_config_; }
  void set_planning_config(planning_config_union &conf) {
    planning_config_ = conf;
  }

  std::string DebugString() const { return "debug string"; }

 private:
  StandardPlanningConfig standard_planning_config_;
  std::vector<TaskConfig> default_task_config_;
  planning_config_union planning_config_;
  bool tranform_traj_to_body_frame_;
};

}  // namespace planning
}  // namespace zark
