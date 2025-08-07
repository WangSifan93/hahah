#ifndef ONBOARD_PLANNER_SECOND_ORDER_TRAJECTORY_POINT_H_
#define ONBOARD_PLANNER_SECOND_ORDER_TRAJECTORY_POINT_H_

#include <string>

#include "math/vec.h"
#include "trajectory_point.pb.h"

namespace e2e_noa {
namespace planning {

class TrajectoryPointWithAcceleration {
 public:
  TrajectoryPointWithAcceleration() = default;
  explicit TrajectoryPointWithAcceleration(const TrajectoryPointProto& proto) {
    FromProto(proto);
  }

  const Vec2d& pos() const { return pos_; }

  double s() const { return s_; }
  double theta() const { return theta_; }
  double kappa() const { return kappa_; }
  double steer_angle() const { return steer_angle_; }
  void set_pos(const Vec2d& pos) { pos_ = pos; }
  void set_s(double s) { s_ = s; }
  void set_theta(double theta) { theta_ = theta; }
  void set_kappa(double kappa) { kappa_ = kappa; }
  void set_steer_angle(double steer_angle) { steer_angle_ = steer_angle; }

  double t() const { return t_; }
  double v() const { return v_; }
  double a() const { return a_; }
  void set_t(double t) { t_ = t; }
  void set_v(double v) { v_ = v; }
  void set_a(double a) { a_ = a; }

  void FromProto(const SecondOrderTrajectoryPointProto& proto);
  void ToProto(SecondOrderTrajectoryPointProto* proto) const;

  void FromProto(const TrajectoryPointProto& proto);
  void ToProto(TrajectoryPointProto* proto) const;

  void ToProto(PoseTrajectoryPointProto* proto) const;

  std::string DebugString() const;

 protected:
  Vec2d pos_ = Vec2d::Zero();
  double s_ = 0.0;
  double theta_ = 0.0;
  double kappa_ = 0.0;
  double steer_angle_ = 0.0;

  double t_ = 0.0;
  double v_ = 0.0;
  double a_ = 0.0;
};

}  // namespace planning
}  // namespace e2e_noa

#endif
