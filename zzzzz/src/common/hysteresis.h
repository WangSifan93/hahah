/******************************************************************************
 * Copyright 2024 The zark Authors fuhh. All Rights Reserved.
 *
 *****************************************************************************/

/**
 * @file hysteresis.h
 **/

#pragma once

namespace zark {
namespace planning {
namespace hysteresis {

class Hysteresis {
 public:
  explicit Hysteresis(int hysteresis_cycle_number);

  /**
   * @brief init state
   * @param start_state start state
   */
  void InitState(const bool start_state);

  /**
   * @brief update expect state
   * @param expect_state expect state
   * @return current state
   */
  const bool UpdateState(const bool expect_state);

  /**
   * @brief get current state
   * @return current state
   */
  inline const bool GetCurrentState() { return current_state_; }

 private:
  int hysteresis_cycle_number_;
  bool current_state_;
};

}  // namespace hysteresis
}  // namespace planning
}  // namespace zark
