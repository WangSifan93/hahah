
#pragma once
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "half_plane_signal.h"
#include "polygon_signal.h"
#include "signal_base.h"
#include "signal_config.h"
#include "speed_limit_signal.h"
#include "stop_sign_signal.h"
#include "traffic_light_signal.h"

namespace e2e_noa {
namespace planning {

// 完整的SignalFactory实现
class SignalFactory {
 public:
  // 主工厂方法 - 根据配置创建信号
  static std::unique_ptr<Signal> CreateSignal(const SignalConfig& config) {
    switch (config.type) {
      case SignalType::kTrafficLight:
        return CreateTrafficLight(config);
      case SignalType::kSpeedLimit:
        return CreateSpeedLimit(config);
      case SignalType::kStopSign:
        return CreateStopSign(config);
      case SignalType::kHalfPlane:
        return CreateHalfPlane(config);
      case SignalType::kPolygon:
        return CreatePolygon(config);
      default:
        return nullptr;
    }
  }

  // 便利方法 - 直接创建交通灯信号
  static std::unique_ptr<TrafficLightSignal> CreateTrafficLightDirect(
      double x, double y, TrafficLightSignal::State state) {
    return std::make_unique<TrafficLightSignal>(x, y, state);
  }

  // 便利方法 - 直接创建速度限制信号
  static std::unique_ptr<SpeedLimitSignal> CreateSpeedLimitDirect(
      const std::vector<std::pair<double, double>>& polygon,
      double speed_limit) {
    return std::make_unique<SpeedLimitSignal>(polygon, speed_limit);
  }

  // 便利方法 - 直接创建停车标志信号
  static std::unique_ptr<StopSignSignal> CreateStopSignDirect(
      double x, double y, double threshold = 1.0) {
    return std::make_unique<StopSignSignal>(x, y, threshold);
  }

  // 验证配置有效性
  static bool ValidateConfig(const SignalConfig& config) {
    switch (config.type) {
      case SignalType::kTrafficLight:
        return ValidateTrafficLightConfig(config);
      case SignalType::kSpeedLimit:
        return ValidateSpeedLimitConfig(config);
      case SignalType::kStopSign:
        return ValidateStopSignConfig(config);
      case SignalType::kHalfPlane:
        return ValidateHalfPlaneConfig(config);
      case SignalType::kPolygon:
        return ValidatePolygonConfig(config);
      default:
        return false;
    }
  }

 private:
  // 创建交通灯信号的具体实现
  static std::unique_ptr<TrafficLightSignal> CreateTrafficLight(
      const SignalConfig& config) {
    if (!ValidateTrafficLightConfig(config)) {
      return nullptr;
    }

    double x = GetParameter(config, "x", 0.0);
    double y = GetParameter(config, "y", 0.0);
    int state_int = static_cast<int>(GetParameter(config, "state", 0.0));

    TrafficLightSignal::State state =
        static_cast<TrafficLightSignal::State>(state_int);
    return std::make_unique<TrafficLightSignal>(x, y, state);
  }

  // 创建速度限制信号的具体实现
  static std::unique_ptr<SpeedLimitSignal> CreateSpeedLimit(
      const SignalConfig& config) {
    if (!ValidateSpeedLimitConfig(config)) {
      return nullptr;
    }

    double speed_limit =
        GetParameter(config, "speed_limit", 50.0);  // 默认50 km/h

    // 如果几何形状为空，创建默认的矩形区域
    std::vector<std::pair<double, double>> polygon = config.geometry;
    if (polygon.empty()) {
      double center_x = GetParameter(config, "center_x", 0.0);
      double center_y = GetParameter(config, "center_y", 0.0);
      double width = GetParameter(config, "width", 10.0);
      double height = GetParameter(config, "height", 10.0);

      polygon = CreateRectanglePolygon(center_x, center_y, width, height);
    }

    return std::make_unique<SpeedLimitSignal>(polygon, speed_limit);
  }

  // 创建停车标志信号的具体实现
  static std::unique_ptr<StopSignSignal> CreateStopSign(
      const SignalConfig& config) {
    if (!ValidateStopSignConfig(config)) {
      return nullptr;
    }

    double x = GetParameter(config, "x", 0.0);
    double y = GetParameter(config, "y", 0.0);
    double threshold = GetParameter(config, "stop_threshold", 1.0);

    return std::make_unique<StopSignSignal>(x, y, threshold);
  }

  // 创建半平面信号的具体实现
  static std::unique_ptr<HalfPlaneSignal> CreateHalfPlane(
      const SignalConfig& config) {
    if (!ValidateHalfPlaneConfig(config)) {
      return nullptr;
    }

    double a = GetParameter(config, "a", 1.0);
    double b = GetParameter(config, "b", 0.0);
    double c = GetParameter(config, "c", 0.0);

    return std::make_unique<HalfPlaneSignal>(a, b, c);
  }

  // 创建多边形信号的具体实现
  static std::unique_ptr<PolygonSignal> CreatePolygon(
      const SignalConfig& config) {
    if (!ValidatePolygonConfig(config)) {
      return nullptr;
    }

    return std::make_unique<PolygonSignal>(config.geometry);
  }

  // 验证交通灯配置
  static bool ValidateTrafficLightConfig(const SignalConfig& config) {
    // 检查必需的参数
    auto it_x = config.parameters.find("x");
    auto it_y = config.parameters.find("y");
    auto it_state = config.parameters.find("state");

    bool has_position =
        (it_x != config.parameters.end()) && (it_y != config.parameters.end());
    bool has_state = (it_state != config.parameters.end()) &&
                     (it_state->second >= 0) && (it_state->second <= 5);

    return has_position && has_state;
  }

  // 验证速度限制配置
  static bool ValidateSpeedLimitConfig(const SignalConfig& config) {
    auto it_speed = config.parameters.find("speed_limit");
    bool has_speed_limit =
        (it_speed != config.parameters.end()) && (it_speed->second > 0);

    // 检查是否有几何定义或者中心点+尺寸定义
    bool has_geometry = !config.geometry.empty() && config.geometry.size() >= 3;
    bool has_rect_def = config.parameters.count("center_x") > 0 &&
                        config.parameters.count("center_y") > 0 &&
                        config.parameters.count("width") > 0 &&
                        config.parameters.count("height") > 0;

    return has_speed_limit && (has_geometry || has_rect_def);
  }

  // 验证停车标志配置
  static bool ValidateStopSignConfig(const SignalConfig& config) {
    auto it_x = config.parameters.find("x");
    auto it_y = config.parameters.find("y");

    return (it_x != config.parameters.end()) &&
           (it_y != config.parameters.end());
  }

  // 验证半平面配置
  static bool ValidateHalfPlaneConfig(const SignalConfig& config) {
    auto it_a = config.parameters.find("a");
    auto it_b = config.parameters.find("b");
    auto it_c = config.parameters.find("c");

    bool has_all_params = (it_a != config.parameters.end()) &&
                          (it_b != config.parameters.end()) &&
                          (it_c != config.parameters.end());

    // 验证法向量不为零
    if (has_all_params) {
      double a = it_a->second;
      double b = it_b->second;
      return (a * a + b * b) > 1e-9;  // 避免零法向量
    }

    return false;
  }

  // 验证多边形配置
  static bool ValidatePolygonConfig(const SignalConfig& config) {
    // 多边形至少需要3个顶点
    if (config.geometry.size() < 3) {
      return false;
    }

    // 检查多边形是否闭合（可选，自动闭合）
    // 检查顶点是否有效（不重复等）
    return IsValidPolygon(config.geometry);
  }

  // 辅助函数：获取参数值
  static double GetParameter(const SignalConfig& config, const std::string& key,
                             double default_value) {
    auto it = config.parameters.find(key);
    return (it != config.parameters.end()) ? it->second : default_value;
  }

  // 辅助函数：创建矩形多边形
  static std::vector<std::pair<double, double>> CreateRectanglePolygon(
      double center_x, double center_y, double width, double height) {
    double half_width = width / 2.0;
    double half_height = height / 2.0;

    return {
        {center_x - half_width, center_y - half_height},  // 左下
        {center_x + half_width, center_y - half_height},  // 右下
        {center_x + half_width, center_y + half_height},  // 右上
        {center_x - half_width, center_y + half_height}   // 左上
    };
  }

  // 辅助函数：验证多边形有效性
  static bool IsValidPolygon(
      const std::vector<std::pair<double, double>>& vertices) {
    if (vertices.size() < 3) return false;

    // 检查是否有重复的连续顶点
    for (size_t i = 0; i < vertices.size(); ++i) {
      size_t next = (i + 1) % vertices.size();
      double dx = vertices[i].first - vertices[next].first;
      double dy = vertices[i].second - vertices[next].second;
      if (dx * dx + dy * dy < 1e-9) {
        return false;  // 连续顶点重复
      }
    }

    // 可以添加更多验证：自相交检测等
    return true;
  }
};

}  // namespace planning
}  // namespace e2e_noa