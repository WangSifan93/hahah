
#pragma once
#include <map>
#include <string>
#include <vector>
#include <utility>
#include "signal_base.h"  // Assuming this header defines SignalType and Signal

namespace e2e_noa {
namespace planning {

// 支持配置驱动的信号创建
struct SignalConfig {
  SignalType type;
  std::map<std::string, double> parameters;
  std::vector<std::pair<double, double>> geometry;
};

// 便于构建SignalConfig的构建器模式
class SignalConfigBuilder {
 public:
  SignalConfigBuilder(SignalType type) { config_.type = type; }

  SignalConfigBuilder& SetParameter(const std::string& key, double value) {
    config_.parameters[key] = value;
    return *this;
  }

  SignalConfigBuilder& SetGeometry(
      const std::vector<std::pair<double, double>>& geometry) {
    config_.geometry = geometry;
    return *this;
  }

  SignalConfigBuilder& AddVertex(double x, double y) {
    config_.geometry.emplace_back(x, y);
    return *this;
  }

  SignalConfig Build() const { return config_; }

  // 交通灯配置
  static SignalConfigBuilder TrafficLight(double x, double y, int state) {
    return SignalConfigBuilder(SignalType::kTrafficLight)
        .SetParameter("x", x)
        .SetParameter("y", y)
        .SetParameter("state", static_cast<double>(state));
  }

  // 速度限制配置
  static SignalConfigBuilder SpeedLimit(double speed_limit) {
    return SignalConfigBuilder(SignalType::kSpeedLimit)
        .SetParameter("speed_limit", speed_limit);
  }

  // 停车标志配置
  static SignalConfigBuilder StopSign(double x, double y) {
    return SignalConfigBuilder(SignalType::kStopSign)
        .SetParameter("x", x)
        .SetParameter("y", y);
  }

 private:
  SignalConfig config_;
};

}  // namespace planning
}  // namespace e2e_noa