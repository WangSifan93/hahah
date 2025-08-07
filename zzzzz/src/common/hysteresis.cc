/******************************************************************************
 * Copyright 2024 The zark Authors fuhh. All Rights Reserved.
 *
 *****************************************************************************/

/**
 * @file hysteresis.cc
 **/

#include "apps/planning/src/common/hysteresis.h"

namespace zark {
namespace planning {
namespace hysteresis {

Hysteresis::Hysteresis(int hysteresis_cycle_number)
    : hysteresis_cycle_number_(hysteresis_cycle_number) {}

void Hysteresis::InitState(const bool state) { current_state_ = state; }

const bool Hysteresis::UpdateState(const bool expected_state) {
  static int state_change_count = 0;
  if (expected_state != current_state_) {
    state_change_count++;
  } else {
    state_change_count = 0;
  }
  if (state_change_count >= hysteresis_cycle_number_) {
    current_state_ = expected_state;
  }
  return current_state_;
}

}  // namespace hysteresis
}  // namespace planning
}  // namespace zark
