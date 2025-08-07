#ifndef ONBOARD_PLANNER_TRAJECTORY_POINT_H_
#define ONBOARD_PLANNER_TRAJECTORY_POINT_H_

#include <algorithm>
#include <string>

#include "math/util.h"
#include "messages/planning/driving/noa_debug_info.pb.h"
#include "plan/trajectorypoint_with_acceleration.h"
#include "trajectory_point.pb.h"

namespace e2e_noa {
namespace planning {

class TrajectoryPoint : public TrajectoryPointWithAcceleration {
 public:
  TrajectoryPoint() = default;
  explicit TrajectoryPoint(const TrajectoryPointProto& proto) {
    FromProto(proto);
  }
  explicit TrajectoryPoint(const ApolloTrajectoryPointProto& proto) {
    FromProto(proto);
  }

  static double ComputeLambda(double v, double psi) {
    return psi / (v < 0.0 ? std::min(v, -kSpeedEps) : std::max(v, kSpeedEps));
  }

  static double ComputeMu(double v, double a, double lambda, double chi) {
    return (chi - a * lambda) / std::max(Sqr(v), Sqr(kSpeedEps));
  }
  double lambda() const { return ComputeLambda(v_, psi_); }
  double mu() const { return ComputeMu(v_, a_, lambda(), chi_); }

  double j() const { return j_; }
  void set_j(double j) { j_ = j; }

  double psi() const { return psi_; }
  double chi() const { return chi_; }
  void set_psi(double psi) { psi_ = psi; }
  void set_chi(double chi) { chi_ = chi; }

  void FromProto(const TrajectoryPointProto& proto);
  void ToProto(TrajectoryPointProto* proto) const;

  void FromProto(const ApolloTrajectoryPointProto& proto);
  void ToProto(ApolloTrajectoryPointProto* proto) const;

  void ToProto(PoseTrajectoryPointProto* proto) const;

  void ToProto(zark::e2e_noa::debug::NOADebugInfo::TrajPoint* proto) const;

  std::string DebugString() const;

 private:
  static constexpr double kSpeedEps = 0.001;

  double j_ = 0.0;

  double psi_ = 0.0;
  double chi_ = 0.0;
};

}  // namespace planning
}  // namespace e2e_noa

#endif
