//////////////////////////////////////////////////////////////////////////////
//
// Base class for all single-player dynamical systems. Supports (discrete-time)
// linearization.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef ILQGAMES_DYNAMICS_SINGLE_PLAYER_DYNAMICAL_SYSTEM_H
#define ILQGAMES_DYNAMICS_SINGLE_PLAYER_DYNAMICAL_SYSTEM_H

#include "ilq_games/include/utils/ilq_types.h"

namespace e2e_noa::planning {

class SinglePlayerDynamicalSystem {
 public:
  virtual ~SinglePlayerDynamicalSystem() {}

  // Compute time derivative of state.
  virtual VectorXf Evaluate(Time t, const VectorXf& x,
                            const VectorXf& u) const = 0;

  // Compute a discrete-time Jacobian linearization.
  // NOTE: assumes A, B already initialized (to I, 0 respectively) for speed.
  // NOTE: this function signature violates Google style guide return by
  // pointer convention intentionally, in order to comply with Eigen standard:
  // https://eigen.tuxfamily.org/dox/TopicFunctionTakingEigenTypes.html
  virtual void Linearize(Time t, const VectorXf& x, const VectorXf& u,
                         Eigen::Ref<MatrixXf> A,
                         Eigen::Ref<MatrixXf> B) const = 0;

  // Distance metric on the state space. By default, just the *squared* 2-norm.
  virtual float DistanceBetween(const VectorXf& x0, const VectorXf& x1) const {
    return (x0 - x1).squaredNorm();
  }

  // Getters.
  Dimension XDim() const { return xdim_; }
  Dimension UDim() const { return udim_; }
  virtual std::vector<Dimension> PositionDimensions() const = 0;

 protected:
  SinglePlayerDynamicalSystem(Dimension xdim, Dimension udim)
      : xdim_(xdim), udim_(udim) {}

  // Dimensions.
  const Dimension xdim_;
  const Dimension udim_;
};  //\class SinglePlayerDynamicalSystem

}  // namespace e2e_noa::planning

#endif
