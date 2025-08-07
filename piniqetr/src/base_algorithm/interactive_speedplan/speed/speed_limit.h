#ifndef ONBOARD_PLANNER_SPEED_SPEED_LIMIT_H_
#define ONBOARD_PLANNER_SPEED_SPEED_LIMIT_H_

#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "absl/types/span.h"

namespace e2e_noa::planning {

class SpeedLimit {
 public:
  struct SpeedLimitRange {
    double start_s = 0.0;
    double end_s = 0.0;
    double speed_limit = 0.0;
    std::string info;
  };

  struct SpeedLimitInfo {
    SpeedLimitInfo() = delete;
    explicit SpeedLimitInfo(const SpeedLimitRange& speed_limit_range)
        : speed_limit(speed_limit_range.speed_limit),
          info(speed_limit_range.info) {}

    SpeedLimitInfo(double speed, std::string info)
        : speed_limit(speed), info(std::move(info)) {}

    SpeedLimitInfo& operator=(const SpeedLimitInfo& other) {
      if (&other == this) return *this;
      speed_limit = other.speed_limit;
      info = other.info;
      return *this;
    }
    double speed_limit = 0.0;
    std::string info;
  };

  explicit SpeedLimit(const std::vector<SpeedLimitRange>& speed_limit_ranges);

  std::optional<SpeedLimitInfo> GetSpeedLimitInfoByS(double s) const;

  std::optional<double> GetSpeedLimitByS(double s) const;

  absl::Span<const SpeedLimitRange> speed_limit_ranges() const {
    return speed_limit_ranges_;
  }

 private:
  std::vector<SpeedLimitRange> speed_limit_ranges_;
};

}  // namespace e2e_noa::planning

#endif
