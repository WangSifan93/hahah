#ifndef ST_PLANNING_OPEN_LOOP_SPEED_LIMIT
#define ST_PLANNING_OPEN_LOOP_SPEED_LIMIT

#include <algorithm>
#include <cstddef>
#include <iterator>
#include <limits>
#include <memory>
#include <optional>
#include <ostream>
#include <string>
#include <utility>
#include <vector>

#include "glog/logging.h"
#include "math/util.h"
#include "speed/speed_limit.h"
#include "speed/vt_speed_limit.h"

namespace e2e_noa::planning {

struct ConstantAccLimit {
  double acc = 4.0;
  std::optional<double> speed = std::nullopt;
  std::string source = "";
};

struct ConstantSpeedLimit {
  double speed = 40.0;
  double time = 20.0;
  std::string source = "";
};

class OpenLoopSpeedLimit {
 public:
  OpenLoopSpeedLimit() {}

  OpenLoopSpeedLimit(const OpenLoopSpeedLimit& other) { *this = other; }
  OpenLoopSpeedLimit& operator=(const OpenLoopSpeedLimit& other) {
    if (this != &other) {
      constant_acc_limits_.clear();
      constant_acc_limits_ = other.GetALimits();
      constant_speed_limits_.clear();
      constant_speed_limits_ = other.GetVLimits();
    }
    return *this;
  }

  void AddALimit(const double& acc, const std::optional<double>& speed,
                 std::string source = "") {
    ConstantAccLimit const_acc_limit{
        .acc = acc, .speed = speed, .source = source};
    constant_acc_limits_.push_back(const_acc_limit);
  };

  void AddVLimit(const double& speed, const double& time,
                 std::string source = "") {
    ConstantSpeedLimit const_speed_limit{
        .speed = speed, .time = time, .source = source};
    constant_speed_limits_.push_back(const_speed_limit);
  };

  const std::vector<ConstantAccLimit>& GetALimits() const {
    return constant_acc_limits_;
  }

  const std::vector<ConstantSpeedLimit>& GetVLimits() const {
    return constant_speed_limits_;
  }

  std::optional<VtSpeedLimit> GenerateOpenLoopSpeedLimit(
      const double& av_speed, const double& step_num,
      const double& t_interval) {
    constexpr double kMaxSpeed = 35.0;

    if (constant_acc_limits_.empty() && constant_speed_limits_.empty()) {
      return std::nullopt;
    }

    VtSpeedLimit vt_limit;
    vt_limit.reserve(step_num + 1);
    for (int i = 0; i < step_num + 1; i++) {
      double min_v = kMaxSpeed;
      std::string source = "";
      double time = static_cast<double>(i) * t_interval;
      for (const auto& a_limit : constant_acc_limits_) {
        double v_tmp = kMaxSpeed;
        if (a_limit.speed.has_value()) {
          v_tmp = a_limit.acc > 0.0 ? std::min(av_speed + a_limit.acc * time,
                                               a_limit.speed.value())
                                    : std::max(av_speed + a_limit.acc * time,
                                               a_limit.speed.value());
        } else {
          v_tmp = av_speed + a_limit.acc * time;
        }
        if (v_tmp < min_v) {
          min_v = v_tmp;
          source = a_limit.source;
        }
      }
      for (const auto& v_limit : constant_speed_limits_) {
        double v_tmp = kMaxSpeed;
        if (time > v_limit.time || v_limit.speed > av_speed) {
          v_tmp = v_limit.speed;
        }
        if (v_tmp < min_v) {
          min_v = v_tmp;
          source = v_limit.source;
        }
      }
      vt_limit.emplace_back(min_v, source);
    }
    return vt_limit;
  }

 private:
  std::vector<ConstantAccLimit> constant_acc_limits_;
  std::vector<ConstantSpeedLimit> constant_speed_limits_;
};

}  // namespace e2e_noa::planning

#endif
