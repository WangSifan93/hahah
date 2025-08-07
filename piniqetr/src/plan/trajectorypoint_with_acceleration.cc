#include "plan/trajectorypoint_with_acceleration.h"

#include "absl/strings/str_format.h"
#include "math/geometry/util.h"

namespace e2e_noa {
namespace planning {

void TrajectoryPointWithAcceleration::FromProto(
    const SecondOrderTrajectoryPointProto& proto) {
  pos_ = Vec2dFromProto(proto.pos());
  s_ = proto.s();
  theta_ = proto.theta();
  kappa_ = proto.kappa();
  steer_angle_ = proto.steer_angle();

  t_ = proto.t();
  v_ = proto.v();
  a_ = proto.a();
}

void TrajectoryPointWithAcceleration::ToProto(
    SecondOrderTrajectoryPointProto* proto) const {
  Vec2dToProto(pos_, proto->mutable_pos());
  proto->set_s(s_);
  proto->set_theta(theta_);
  proto->set_kappa(kappa_);
  proto->set_steer_angle(steer_angle_);

  proto->set_t(t_);
  proto->set_v(v_);
  proto->set_a(a_);
}

void TrajectoryPointWithAcceleration::FromProto(
    const TrajectoryPointProto& proto) {
  pos_ = Vec2dFromProto(proto.pos());
  s_ = proto.s();
  theta_ = proto.theta();
  kappa_ = proto.kappa();
  steer_angle_ = proto.steer_angle();

  t_ = proto.t();
  v_ = proto.v();
  a_ = proto.a();
}

void TrajectoryPointWithAcceleration::ToProto(
    TrajectoryPointProto* proto) const {
  Vec2dToProto(pos_, proto->mutable_pos());
  proto->set_s(s_);
  proto->set_theta(theta_);
  proto->set_kappa(kappa_);
  proto->set_steer_angle(steer_angle_);

  proto->set_t(t_);
  proto->set_v(v_);
  proto->set_a(a_);
}

void TrajectoryPointWithAcceleration::ToProto(
    PoseTrajectoryPointProto* proto) const {
  Vec2dToProto(pos_, proto->mutable_pos());
  proto->set_theta(theta_);
  proto->set_t(t_);
}

std::string TrajectoryPointWithAcceleration::DebugString() const {
  return absl::StrFormat(
      "pos: (%.4f %.4f) s: %.4f theta: %.5f kappa: %.4f t: %.3f v: %.3f a: "
      "%.3f steer_angle: %.3f",
      pos().x(), pos().y(), s(), theta(), kappa(), t(), v(), a(),
      steer_angle());
}

}  // namespace planning
}  // namespace e2e_noa
