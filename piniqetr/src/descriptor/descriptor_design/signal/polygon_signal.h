
#pragma once
#include <utility>
#include <vector>

#include "signal_base.h"

namespace e2e_noa {
namespace planning {

// 多边形信号设计思路（通用多边形约束）
class PolygonSignal : public Signal {
 public:
  // 构造函数 - 多边形顶点序列
  PolygonSignal(const std::vector<std::pair<double, double>>& vertices);

  // 基础访问器
  const std::vector<std::pair<double, double>>& vertices() const;

  // 几何判断和计算
  bool ContainsPoint(double x, double y) const;
  double DistanceToPolygon(double x, double y) const;
  double PenetrationDepth(double x, double y) const;

  // 多边形属性计算
  double Area() const;
  std::pair<double, double> Centroid() const;

 private:
  // 内部几何算法
  static bool PointInPolygon(
      double x, double y, const std::vector<std::pair<double, double>>& poly);
  static double PointToPolygonDistance(
      double x, double y, const std::vector<std::pair<double, double>>& poly);

  SignalType Type() const override { return SignalType::kPolygon; }
};

}  // namespace planning
}  // namespace e2e_noa