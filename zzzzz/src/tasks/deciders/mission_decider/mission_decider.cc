/******************************************************************************
 * Copyright 2024 The zark . All Rights Reserved.
 *****************************************************************************/

/**
 * @file msission_decider.cc
 **/

#include "apps/planning/src/tasks/deciders/mission_decider/mission_decider.h"

#include "apps/planning/src/common/clock.h"

namespace zark {
namespace planning {

MissionDecider::MissionDecider(
    const TaskConfig& config,
    const std::shared_ptr<DependencyInjector>& injector)
    : Decider(config, injector) {
  env_info_builder_ = std::make_unique<EnvInfoBuilder>(
      config.task_config().mission_decider_config);

  lc_request_ =
      std::make_unique<LCRequest>(config.task_config().mission_decider_config);
}

::common::Status MissionDecider::Execute(Frame* frame) {
  if (frame == nullptr) {
    return ::common::Status(::common::ErrorCode::PLANNING_ERROR,
                            "Frame is nullptr.");
  }

  if (frame->FindCurrentLocalRoute() == nullptr) {
    return ::common::Status(::common::ErrorCode::PLANNING_ERROR,
                            "Current LocalRoute is nullptr.");
  }

  Mission mission;
  mission = lc_request_->Process(frame);
  frame->SetMission(mission);
  frame->SetIsNearIntersection(lc_request_->GetIsNearRamp());

  SetTargetLocalRoute(frame);

  return ::common::Status::OK();
}

void MissionDecider::SetTargetLocalRoute(Frame* frame) {
  auto cur_mission = frame->GetMission();

  switch (cur_mission.target_reference_line) {
    case Mission::TargetReflineType::REF_CURRENT_LANE:
      frame->SetTargetLocalRoute(frame->FindCurrentLocalRoute());
      ADEBUG << "Set Current LocalRoute Target in Frame.";
      break;
    case Mission::TargetReflineType::REF_LEFT_LANE:
      if (frame->FindCurrentLocalRoute()->LeftRoute()) {
        frame->SetTargetLocalRoute(frame->FindCurrentLocalRoute()->LeftRoute());
        ADEBUG << "Set Left LocalRoute Target in Frame.";
      } else {
        frame->SetTargetLocalRoute(frame->FindCurrentLocalRoute());
        ADEBUG
            << "LeftRoute = Nullptr, Set Current LocalRoute Target in Frame.";
      }
      break;
    case Mission::TargetReflineType::REF_RIGHT_LANE:
      if (frame->FindCurrentLocalRoute()->RightRoute()) {
        frame->SetTargetLocalRoute(
            frame->FindCurrentLocalRoute()->RightRoute());
        ADEBUG << "Set Left LocalRoute Target in Frame.";
      } else {
        frame->SetTargetLocalRoute(frame->FindCurrentLocalRoute());
        ADEBUG
            << "RightRoute = Nullptr, Set Current LocalRoute Target in Frame.";
      }
      break;
    default:
      frame->SetTargetLocalRoute(frame->FindCurrentLocalRoute());
      break;
  }
}

}  // namespace planning
}  // namespace zark
