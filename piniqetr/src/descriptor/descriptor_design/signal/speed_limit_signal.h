
#pragma once
#include <utility>
#include <vector>

#include "signal_base.h"

namespace e2e_noa {
namespace planning {

// 限速信号设计思路
class SpeedLimitSignal : public Signal {
 public:
  // 构造函数 - 多边形区域和限速值
  SpeedLimitSignal(
      const std::vector<std::pair<double, double>>& polygon_vertices,
      double speed_limit_mps);

  // 基础访问器
  const std::vector<std::pair<double, double>>& polygon() const;
  double speed_limit() const;

  // 空间判断方法
  bool ContainsPoint(double x, double y) const;
  double DistanceToPolygon(double x, double y) const;  // 到边界距离
  double PenetrationDepth(double x, double y) const;   // 穿透深度

  // 违规判断方法
  double GetSpeedViolation(double current_speed) const;
  bool IsSpeedCompliant(double current_speed) const;

  SignalType Type() const override { return SignalType::kSpeedLimit; }
};

}  // namespace planning
}  // namespace e2e_noa