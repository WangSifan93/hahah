
#pragma once

namespace e2e_noa {
namespace planning {
// Enums
enum class SignalType {
  kHalfPlane = 0,
  kPolygon = 1,
  kSpeedLimit = 2,
  kSteerLimit = 3,
  kTrafficLight = 4,
  kStopSign = 5
};
// 基类设计 - 纯抽象接口
class Signal {
 public:
  virtual ~Signal() = default;
  virtual SignalType Type() const = 0;
  // 考虑添加通用的验证方法
  virtual bool IsValid() const { return true; }
};

}  // namespace planning
}  // namespace e2e_noa