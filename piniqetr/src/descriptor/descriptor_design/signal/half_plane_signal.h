
#pragma once
#include <utility>

#include "signal_base.h"

namespace e2e_noa {
namespace planning {

// 半平面信号设计思路（用于边界约束）
class HalfPlaneSignal : public Signal {
 public:
  // 构造函数 - 半平面方程参数 ax + by + c >= 0
  HalfPlaneSignal(double a, double b, double c);

  // 基础访问器
  double a() const;
  double b() const;
  double c() const;

  // 几何判断方法
  bool IsOnPositiveSide(double x, double y) const;  // 是否在正侧
  double SignedDistance(double x, double y) const;  // 有符号距离
  double PenetrationDistance(double x,
                             double y) const;  // 穿透距离（如果违反约束）

  // 向量计算辅助
  std::pair<double, double> GetNormalVector() const;  // 法向量

  SignalType Type() const override { return SignalType::kHalfPlane; }
};

}  // namespace planning
}  // namespace e2e_noa