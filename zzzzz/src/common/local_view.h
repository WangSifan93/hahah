/******************************************************************************
 * Copyright 2018 The zpilot Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#pragma once

#include <memory>

#include "messages/canbus/chassis.pb.h"
#include "messages/common/heartbeat.pb.h"
#include "messages/localization/localization.pb.h"
#include "messages/map_service/all_map_new.pb.h"
#include "messages/mapfusion/all_map.pb.h"
#include "messages/mapfusion/map_fusion.pb.h"
#include "messages/perception/obstacle.pb.h"
#include "messages/planning/ads_adptrin.pb.h"
#include "messages/planning/ads_com.pb.h"
#include "messages/planning/ads_fct.pb.h"
#include "messages/planning/ads_decision.pb.h"
#include "messages/planning/planning.pb.h"
#include "messages/planning/planning_debug.pb.h"
#include "messages/planning/reference_line.pb.h"
#include "messages/prediction/prediction.pb.h"
#include "messages/sensors/vehicle_upstream.pb.h"

namespace zark {
namespace planning {

/**
 * @struct local_view
 * @brief LocalView contains all necessary data as planning input
 */

struct LocalView {
  std::shared_ptr<zark::prediction::proto::PredictionObjects>
      prediction_obstacles{nullptr};
  std::shared_ptr<zark::localization::LocalizationInfo> localization_estimate{
      nullptr};
  std::shared_ptr<zark::ads_adptrin::ADAS_Inputs> can_data{nullptr};
  std::shared_ptr<zark::ads_fct::FCT_Outputs> fct_output{nullptr};
  std::shared_ptr<zark::ads_decision::DEC_Outputs> dec_output{nullptr};
  // perception
  // stories
};

}  // namespace planning
}  // namespace zark
