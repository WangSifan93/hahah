#include "plan/trajectory_point.h"

#include "absl/strings/str_format.h"
#include "math/vec.h"

namespace e2e_noa {
namespace planning {

void TrajectoryPoint::FromProto(const TrajectoryPointProto& proto) {
  TrajectoryPointWithAcceleration::FromProto(proto);
  j_ = proto.j();
  psi_ = proto.psi();
  chi_ = proto.chi();
}

void TrajectoryPoint::ToProto(TrajectoryPointProto* proto) const {
  TrajectoryPointWithAcceleration::ToProto(proto);
  proto->set_j(j_);
  proto->set_psi(psi_);
  proto->set_chi(chi_);
}

void TrajectoryPoint::FromProto(const ApolloTrajectoryPointProto& proto) {
  set_pos({proto.path_point().x(), proto.path_point().y()});
  set_s(proto.path_point().s());
  set_theta(proto.path_point().theta());
  set_kappa(proto.path_point().kappa());
  set_steer_angle(proto.path_point().steer_angle());
  set_psi(proto.path_point().lambda() * proto.v());

  set_t(proto.relative_time());
  set_v(proto.v());
  set_a(proto.a());
  set_j(proto.j());
}

void TrajectoryPoint::ToProto(ApolloTrajectoryPointProto* proto) const {
  proto->mutable_path_point()->set_x(pos_.x());
  proto->mutable_path_point()->set_y(pos_.y());
  proto->mutable_path_point()->set_s(s_);
  proto->mutable_path_point()->set_theta(theta_);
  proto->mutable_path_point()->set_kappa(kappa_);
  proto->mutable_path_point()->set_lambda(lambda());
  proto->mutable_path_point()->set_steer_angle(steer_angle_);

  proto->set_relative_time(t_);
  proto->set_v(v_);
  proto->set_a(a_);
  proto->set_j(j_);
  proto->set_yaw_rate(kappa_ * v_);
}

void TrajectoryPoint::ToProto(
    zark::e2e_noa::debug::NOADebugInfo::TrajPoint* proto) const {
  proto->set_x(pos_.x());
  proto->set_y(pos_.y());
  proto->set_s(s_);
  proto->set_theta(theta_);
  proto->set_kappa(kappa_);
  proto->set_lambda(lambda());
  proto->set_steer_angle(steer_angle_);

  proto->set_relative_time(t_);
  proto->set_v(v_);
  proto->set_a(a_);
  proto->set_j(j_);
  proto->set_yaw_rate(kappa_ * v_);
  proto->set_chi(chi_);
  proto->set_psi(psi_);
}

void TrajectoryPoint::ToProto(PoseTrajectoryPointProto* proto) const {
  TrajectoryPointWithAcceleration::ToProto(proto);
}

std::string TrajectoryPoint::DebugString() const {
  return absl::StrFormat(
      "pos: (%.4f %.4f) s: %.4f theta: %.5f kappa: %.4f psi: %.4f t: %.3f v: "
      "%.3f a: "
      "%.3f j: %.3f steer_angle: %.3f",
      pos().x(), pos().y(), s(), theta(), kappa(), psi(), t(), v(), a(), j(),
      steer_angle());
}

}  // namespace planning
}  // namespace e2e_noa
