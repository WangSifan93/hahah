/******************************************************************************
 * Copyright 2023 The zpilot . All Rights Reserved.
 *****************************************************************************/
/**
 * @file
 **/

#include <utility>

#include "apps/planning/src/common/clock.h"
#include "apps/planning/src/common/frame.h"
#include "apps/planning/src/common/log.h"
#include "apps/planning/src/scenarios/lane_follow/lane_follow_stage_v2.h"

namespace zark {
namespace planning {
namespace scenario {
namespace lane_follow {

using zark::common::Clock;

LaneFollowStageV2::LaneFollowStageV2(
    const ScenarioConfig::StageConfig &config,
    const std::shared_ptr<DependencyInjector> &injector)
    : Stage(config, injector) {}

Stage::StageStatus LaneFollowStageV2::Process(
    const ::common::TrajectoryPoint &planning_start_point, Frame *frame) {
  bool has_drivable_local_route = false;

  ADEBUG << "Number of local routes:\t" << frame->MutableLocalRoutes().size();

  auto cur_status = PlanOnLocalRoute(planning_start_point, frame);

  if (cur_status.ok()) {
    has_drivable_local_route = true;
  }

  return has_drivable_local_route ? StageStatus::RUNNING : StageStatus::ERROR;
}

Status LaneFollowStageV2::PlanOnLocalRoute(
    const TrajectoryPoint &planning_start_point, Frame *frame) {
  auto ret = Status::OK();
  std::map<std::string, double> status_map;

  double task_latency_sum = 0.0;
  for (auto *task : task_list_) {
    const double start_timestamp = Clock::NowInSeconds();
    ret = task->Execute(frame);
    const double end_timestamp = Clock::NowInSeconds();
    const double time_diff_ms = (end_timestamp - start_timestamp) * 1000;
    TaskStats *task_stats = frame->MutableCurrentFramePlannedTrajectory()
                                ->mutable_latency_stats()
                                ->add_task_stats();
    task_stats->set_name(task->Name());
    task_stats->set_time_ms(time_diff_ms);
    task_latency_sum += time_diff_ms;
    AINFO << task->Name() << " time spend: " << time_diff_ms << " ms.";
    status_map[task->Name()] = time_diff_ms;
    if (!ret.ok()) {
      AERROR << "Failed to run tasks["
             << task->Config().TaskType_Name(task->Config().task_type())
             << "], Error message: " << ret.error_message();
      break;
    }
  }
  TaskStats *task_stats = frame->MutableCurrentFramePlannedTrajectory()
                              ->mutable_latency_stats()
                              ->add_task_stats();
  task_stats->set_name("TASKS_SUM");
  task_stats->set_time_ms(task_latency_sum);

  for (const auto &s : status_map) {
    ADEBUG << s.first << " consume " << s.second << " ms. ";
  }
  return Status::OK();
}

}  // namespace lane_follow
}  // namespace scenario
}  // namespace planning
}  // namespace zark
