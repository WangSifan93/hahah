/******************************************************************************
 * Copyright 2019 The zpilot Authors. All Rights Reserved.
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

#include <boost/filesystem.hpp>
#include <boost/range/iterator_range.hpp>
#include <string>
#include <vector>

#include "apps/planning/src/common/vehicle_state/proto/vehicle_state.h"
#include "apps/planning/src/common/vehicle_state/vehicle_state_provider.h"

namespace zark {
namespace planning {
namespace util {

bool IsVehicleStateValid(const common::VehicleState& vehicle_state);

}  // namespace util
}  // namespace planning
}  // namespace zark
