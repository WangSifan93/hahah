///////////////////////////////////////////////////////////////////////////
//
// Base class for all multi-player dynamical systems. Supports (discrete-time)
// linearization and integration.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef ILQGAMES_DYNAMICS_MULTI_PLAYER_DYNAMICAL_SYSTEM_H
#define ILQGAMES_DYNAMICS_MULTI_PLAYER_DYNAMICAL_SYSTEM_H

#include <vector>

#include "ilq_games/include/dynamics/multi_player_integrable_system.h"
#include "ilq_games/include/utils/ilq_types.h"
#include "ilq_games/include/utils/linear_dynamics_approximation.h"
#include "ilq_games/include/utils/operating_point.h"
#include "ilq_games/include/utils/strategy.h"

namespace e2e_noa::planning {

class MultiPlayerDynamicalSystem : public MultiPlayerIntegrableSystem {
 public:
  virtual ~MultiPlayerDynamicalSystem() {}

  // Compute time derivative of state.
  virtual VectorXf Evaluate(Time t, const VectorXf& x,
                            const std::vector<VectorXf>& us) const = 0;

  // Compute a discrete-time Jacobian linearization.
  virtual LinearDynamicsApproximation Linearize(
      Time t, const VectorXf& x, const std::vector<VectorXf>& us) const = 0;

  // Integrate these dynamics forward in time.
  VectorXf Integrate(Time t0, Time time_interval, const VectorXf& x0,
                     const std::vector<VectorXf>& us) const;

  // Getters.
  virtual Dimension UDim(PlayerIndex player_idx) const = 0;
  virtual PlayerIndex NumPlayers() const = 0;

 protected:
  explicit MultiPlayerDynamicalSystem(Dimension xdim)
      : MultiPlayerIntegrableSystem(xdim) {}

};  //\class MultiPlayerDynamicalSystem

}  // namespace e2e_noa::planning

#endif
