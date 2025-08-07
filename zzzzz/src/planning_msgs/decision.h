/******************************************************************************
 * Copyright 2023 The zpilot Authors Fuhuanhuan. All Rights Reserved.
 *****************************************************************************/
#pragma once
#include <string>
#include <vector>

#include "geometry.h"
#include "vehicle_signal.h"
#include "messages/mapfusion/routing.pb.h"

namespace zark {
namespace planning {

enum class RelativeRegionType {
  IDLE_TYPE = 0,
  FRONT = 1,
  LEFT = 2,
  RIGHT = 3,
  REAR = 4,
  LEFT_FRONT = 5,
  LEFT_REAR = 6,
  RIGHT_FRONT = 7,
  RIGHT_REAR = 8,
  LEFT_LEFT_FRONT = 9,
  RIGHT_RIGHT_FRONT = 10,
};

}  // namespace planning
}  // namespace zark
