
#pragma once
#include <cmath>

#include "signal_base.h"

namespace e2e_noa {
namespace planning {

// 交通灯信号设计思路
class TrafficLightSignal : public Signal {
 public:
  // 枚举交通灯状态 - 涵盖所有可能情况
  enum class State {
    kGreen,
    kYellow,
    kRed,
    kFlashingYellow,
    kFlashingRed,
    kBroken
  };
  // 构造函数 - 位置和初始状态
  TrafficLightSignal(double x, double y, State initial_state);

  // 基础访问器
  double x() const;
  double y() const;
  State state() const;
  void set_state(State new_state);

  // 交通规则判断逻辑
  bool RequiresStop() const;      // 是否需要停车
  bool AllowsProceeding() const;  // 是否允许通行
  bool RequiresCaution() const;   // 是否需要谨慎通行

  // 几何计算辅助方法
  double DistanceTo(double x, double y) const;
  double BearingTo(double x, double y) const;  // 方位角计算

  SignalType Type() const override { return SignalType::kTrafficLight; }
};

}  // namespace planning
}  // namespace e2e_noa