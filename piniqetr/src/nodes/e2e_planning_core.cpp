#include "nodes/e2e_planning_core.h"

#include <filesystem>

#include "common/gflags.h"
#include "common/log_data.h"
#include "common/timer.h"
#include "glog/logging.h"
#include "util/hr_timer.h"
#include "util/utility.h"
#include "zlog.h"

namespace ad_e2e {
namespace planning {
bool E2EPlanningCore::Init() {
  FLAGS_ad_e2e_platform_config =
      FLAGS_ad_e2e_city_config + FLAGS_ad_e2e_platform_config;
  CHECK(Utility::GetProtoFromFile(FLAGS_ad_e2e_platform_config,
                                  &planning_config_));

  CHECK(InitGflags());

  set_planning_enabled(true);

  if (planning_enabled()) {
    city_planner_ = std::make_unique<ad_e2e::planning::CityPlanner>(
        FLAGS_ad_e2e_city_planner_pool_size);

    CHECK(city_planner_->Init(FLAGS_ad_e2e_city_config));
  }

  return true;
}

bool E2EPlanningCore::InitGflags() const {
  std::string platform;
  char* platform_ptr = getenv("PLATFORM");
  if (platform_ptr == nullptr) {
    LOG(ERROR) << "E2EPlanning got envariable[PLATFORM] failed!";
    return false;
  }
  platform = std::string(platform_ptr);

  bool find_config = false;
  e2e_noa::planning::planning_config::PlatformConfig platform_config;
  for (const auto& config : planning_config_.platform_config()) {
    if (config.platform() == platform) {
      find_config = true;
      platform_config = config;
      FLAGS_ad_e2e_city_planner_pool_size = config.num_threads_plan();
      break;
    }
  }
  if (!find_config) {
    LOG(ERROR) << "E2EPlanning can not find [" << platform
               << "] platform config";
    return false;
  } else {
    FLAGS_ad_e2e_planning_platform = platform_config.platform();
    LOG(INFO) << "E2EPlanning platform: " << FLAGS_ad_e2e_planning_platform
              << ", [num_threads param] planner: "
              << FLAGS_ad_e2e_city_planner_pool_size;
    return true;
  }
  return false;
}

bool E2EPlanningCore::Exit() { return true; }

std::shared_ptr<planning_result_type> E2EPlanningCore::PlanningCallback(
    const std::pair<PlanResult, PlanningInputFrame::Ptr>& input_frame) {
  static uint8_t sequence = 0;
  sequence++;
  if (sequence < 0 || sequence > 255) sequence = 0;

  std::shared_ptr<planning_result_type> res{nullptr};
  if (input_frame.first != ad_e2e::planning::PlanResult::PLAN_OK) {
    auto fail_reason = input_frame.first;
    LOG(ERROR) << "planning input frame invalid: " << fail_reason;

    res = std::make_shared<planning_result_type>();
    res->first.proto.mutable_state()->set_result(
        zark::e2e_noa::Result::RESULT_FAIL);
    res->first.proto.mutable_state()->set_fail_reason(fail_reason);

    res->second.result = Result::RESULT_FAIL;
    res->second.fail_reason = static_cast<uint8_t>(fail_reason);

    DFHLOG_E("Plan result not ok, city planner reset and return !");
    city_planner_->Reset();

    res->first.proto.mutable_trajectory()->set_rolling_counter(sequence);

    return res;
  }

  Planner* planner{nullptr};
  const PlanningInputFrame* frame = std::get<1>(input_frame).get();
  bool func_id_valid = true;

  planner = city_planner_.get();

  planner->Run(frame);

  res = planner->GetPlanningResult();

  if (!func_id_valid || res->second.result != Result::RESULT_OK) {
    planner->Reset();
  }

  res->first.proto.mutable_trajectory()->set_rolling_counter(sequence);

  return res;
}

}  // namespace planning
}  // namespace ad_e2e
