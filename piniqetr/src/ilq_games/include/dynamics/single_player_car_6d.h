//////////////////////////////////////////////////////////////////////////////
//
// Single player dynamics modeling a car. 6 states and 2 control inputs.
// State is [x, y, theta, phi, v, a], control is [omega, j], and dynamics are:
//                     \dot px    = v cos theta
//                     \dot py    = v sin theta
//                     \dot theta = (v / L) * tan phi
//                     \dot phi   = omega
//                     \dot v     = a
//                     \dot a     = j
///////////////////////////////////////////////////////////////////////////////

#ifndef ILQGAMES_DYNAMICS_SINGLE_PLAYER_CAR_6D_H
#define ILQGAMES_DYNAMICS_SINGLE_PLAYER_CAR_6D_H

#include "ilq_games/include/dynamics/single_player_dynamical_system.h"
#include "ilq_games/include/utils/ilq_types.h"

namespace e2e_noa::planning {

class SinglePlayerCar6D : public SinglePlayerDynamicalSystem {
 public:
  ~SinglePlayerCar6D() {}
  SinglePlayerCar6D(float inter_axle_distance)
      : SinglePlayerDynamicalSystem(kNumXDims, kNumUDims),
        inter_axle_distance_(inter_axle_distance) {}

  // Compute time derivative of state.
  VectorXf Evaluate(Time t, const VectorXf& x, const VectorXf& u) const;

  // Compute a discrete-time Jacobian linearization.
  void Linearize(Time t, const VectorXf& x, const VectorXf& u,
                 Eigen::Ref<MatrixXf> A, Eigen::Ref<MatrixXf> B) const;

  // Distance metric between two states.
  float DistanceBetween(const VectorXf& x0, const VectorXf& x1) const;

  // Position dimensions.
  std::vector<Dimension> PositionDimensions() const { return {kPxIdx, kPyIdx}; }

  // Constexprs for state indices.
  static const Dimension kNumXDims;
  static const Dimension kPxIdx;
  static const Dimension kPyIdx;
  static const Dimension kThetaIdx;
  static const Dimension kPhiIdx;
  static const Dimension kVIdx;
  static const Dimension kAIdx;

  // Constexprs for control indices.
  static const Dimension kNumUDims;
  static const Dimension kOmegaIdx;
  static const Dimension kJerkIdx;

 private:
  // Inter-axle distance. Determines turning radius.
  const float inter_axle_distance_;
};  //\class SinglePlayerCar6D

// ----------------------------- IMPLEMENTATION ----------------------------- //

inline VectorXf SinglePlayerCar6D::Evaluate(Time t, const VectorXf& x,
                                            const VectorXf& u) const {
  VectorXf xdot(xdim_);
  xdot(kPxIdx) = x(kVIdx) * std::cos(x(kThetaIdx));
  xdot(kPyIdx) = x(kVIdx) * std::sin(x(kThetaIdx));
  xdot(kThetaIdx) = (x(kVIdx) / inter_axle_distance_) * std::tan(x(kPhiIdx));
  xdot(kPhiIdx) = u(kOmegaIdx);
  xdot(kVIdx) = x(kAIdx);
  xdot(kAIdx) = u(kJerkIdx);

  return xdot;
}

inline void SinglePlayerCar6D::Linearize(Time t, const VectorXf& x,
                                         const VectorXf& u,
                                         Eigen::Ref<MatrixXf> A,
                                         Eigen::Ref<MatrixXf> B) const {
  const float ctheta = std::cos(x(kThetaIdx)) * time::getTimeStep();
  const float stheta = std::sin(x(kThetaIdx)) * time::getTimeStep();
  const float cphi = std::cos(x(kPhiIdx));
  const float tphi = std::tan(x(kPhiIdx));

  A(kPxIdx, kThetaIdx) += -x(kVIdx) * stheta;
  A(kPxIdx, kVIdx) += ctheta;

  A(kPyIdx, kThetaIdx) += x(kVIdx) * ctheta;
  A(kPyIdx, kVIdx) += stheta;

  A(kThetaIdx, kPhiIdx) +=
      x(kVIdx) * time::getTimeStep() / (inter_axle_distance_ * cphi * cphi);
  A(kThetaIdx, kVIdx) += tphi * time::getTimeStep() / inter_axle_distance_;

  A(kVIdx, kAIdx) += time::getTimeStep();

  B(kPhiIdx, kOmegaIdx) = time::getTimeStep();
  B(kAIdx, kJerkIdx) = time::getTimeStep();
}

inline float SinglePlayerCar6D::DistanceBetween(const VectorXf& x0,
                                                const VectorXf& x1) const {
  // Squared distance in position space.
  const float dx = x0(kPxIdx) - x1(kPxIdx);
  const float dy = x0(kPyIdx) - x1(kPyIdx);
  return dx * dx + dy * dy;
}

}  // namespace e2e_noa::planning

#endif
