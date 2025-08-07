/******************************************************************************
 * Copyright 2023 The zpilot Fuhuanhuan. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include <chrono>

namespace zark {
namespace common {
class Clock {
 private:
  /* data */
 public:
  // Clock() {};
  Clock() = delete;

  static double calculateTimeDifference(
      std::chrono::time_point<std::chrono::system_clock> start,
      std::chrono::time_point<std::chrono::system_clock> end) {
    // 计算时间差
    std::chrono::duration<double, std::milli> duration = end - start;
    return duration.count();
  }

  static double NowInSeconds() {
    auto now = std::chrono::system_clock::now();
    auto duration = now.time_since_epoch();
    double seconds =
        std::chrono::duration_cast<std::chrono::seconds>(duration).count();
    double subseconds =
        std::chrono::duration<double, std::milli>(duration).count() -
        seconds * 1000.0;
    return seconds + subseconds / 1000.0;
  }
};
}  // namespace common
}  // namespace zark
