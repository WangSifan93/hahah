/******************************************************************************
 * Copyright 2023 The Author Fuhuanhuan. All Rights Reserved.
 *****************************************************************************/

/**
 * @file
 **/

#include "apps/planning/src/scenarios/scenario.h"
#include "apps/planning/src/common/frame.h"
#include "apps/planning/src/config/config_main.h"

namespace zark {
namespace planning {
namespace scenario {
using namespace common;

Scenario::Scenario(const ScenarioConfig& config, const ScenarioContext* context,
                   const std::shared_ptr<DependencyInjector>& injector)
    : config_(config), scenario_context_(context), injector_(injector) {
  // name_ = ScenarioType_Name(config.scenario_type()); //todo fix
  name_ = "LANE_FOLLOW";
}

bool Scenario::LoadConfig(const std::string& config_file,
                          ScenarioConfig* config,
                          planning::PlanningConfig planning_config) {
  // return LoadConfigFromFile(config_file,config,planning_config);
  if (!config_file.empty()) {
    zark::planning::Config scenario_config{config_file};
    scenario_config.SetScenarioConfig(*config, planning_config);
    return true;
  }
  return false;
}

void Scenario::Init() {
  ACHECK(!config_.stage_type().empty());

  for (const auto& stage_config : config_.stage_config()) {
    stage_config_map_[stage_config.stage_type()] = &stage_config;
  }
  for (std::size_t i = 0; i < config_.stage_type().size(); ++i) {
    auto stage_type = config_.stage_type().at(i);
    // bool find_stage = false;
    if (stage_config_map_.find(stage_type) != stage_config_map_.end()) {
      // find_stage = true;
    } else {
      // find_stage = false;
      AINFO << "stage type : " << (stage_type) << " has no config";
    }
  }
  ADEBUG << "init stage " << config_.stage_type().at(0);
  current_stage_ =
      CreateStage(*stage_config_map_[config_.stage_type().at(0)], injector_);
}

Scenario::ScenarioStatus Scenario::Process(
    const ::common::TrajectoryPoint& planning_init_point, Frame* frame) {
  if (current_stage_ == nullptr) {
    AWARN << "Current stage is a null pointer.";
    return STATUS_UNKNOWN;
  }
  if (current_stage_->stage_type() == StageType::NO_STAGE) {
    scenario_status_ = STATUS_DONE;
    return scenario_status_;
  }
  auto ret = current_stage_->Process(planning_init_point, frame);
  switch (ret) {
    case Stage::ERROR: {
      AERROR << "Stage '" << current_stage_->Name() << "' returns error";
      scenario_status_ = STATUS_UNKNOWN;
      break;
    }
    case Stage::RUNNING: {
      scenario_status_ = STATUS_PROCESSING;
      break;
    }
    case Stage::FINISHED: {
      auto next_stage = current_stage_->NextStage();
      if (next_stage != current_stage_->stage_type()) {
        AINFO << "switch stage from " << current_stage_->Name() << " to "
              << (next_stage);
        if (next_stage == StageType::NO_STAGE) {
          scenario_status_ = STATUS_DONE;
          return scenario_status_;
        }
        if (stage_config_map_.find(next_stage) == stage_config_map_.end()) {
          AERROR << "Failed to find config for stage: " << next_stage;
          scenario_status_ = STATUS_UNKNOWN;
          return scenario_status_;
        }
        current_stage_ = CreateStage(*stage_config_map_[next_stage], injector_);
        if (current_stage_ == nullptr) {
          AWARN << "Current stage is a null pointer.";
          return STATUS_UNKNOWN;
        }
      }
      if (current_stage_ != nullptr &&
          current_stage_->stage_type() != StageType::NO_STAGE) {
        scenario_status_ = STATUS_PROCESSING;
      } else {
        scenario_status_ = STATUS_DONE;
      }
      break;
    }
    default: {
      AWARN << "Unexpected Stage return value: " << ret;
      scenario_status_ = STATUS_UNKNOWN;
    }
  }
  return scenario_status_;
}

const std::string& Scenario::Name() const { return name_; }

// bool Scenario::LoadConfigFromFile(const std::string& config_file,
//                           ScenarioConfig* config) {

// }

}  // namespace scenario
}  // namespace planning
}  // namespace zark
