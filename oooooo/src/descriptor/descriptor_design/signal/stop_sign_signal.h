
#pragma once
#include "signal_base.h"

namespace e2e_noa {
namespace planning {

// 停车标志信号设计思路
class StopSignSignal : public Signal {
 public:
  // 构造函数 - 位置和停车阈值距离
  StopSignSignal(double x, double y, double stop_threshold_distance = 1.0);

  // 基础访问器
  double x() const;
  double y() const;
  double stop_threshold() const;

  // 停车逻辑判断
  bool IsWithinStopZone(double x, double y) const;   // 是否在停车区域
  bool HasPassedStopLine(double x, double y) const;  // 是否已越过停车线
  double DistanceToStopLine(double x, double y) const;  // 到停车线距离

  // 停车状态验证
  bool IsValidStop(double x, double y, double speed) const;  // 是否有效停车

  SignalType Type() const override { return SignalType::kStopSign; }
};

}  // namespace planning
}  // namespace e2e_noa