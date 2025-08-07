/////////////////////////////////////////////////////////////////////////////
//
// Multi-player dynamical system comprised of several single player subsystems.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef ILQGAMES_DYNAMICS_CONCATENATED_DYNAMICAL_SYSTEM_H
#define ILQGAMES_DYNAMICS_CONCATENATED_DYNAMICAL_SYSTEM_H

#include <algorithm>

#include "ilq_games/include/dynamics/multi_player_dynamical_system.h"
#include "ilq_games/include/dynamics/single_player_dynamical_system.h"
#include "ilq_games/include/utils/ilq_types.h"
#include "ilq_games/include/utils/linear_dynamics_approximation.h"

namespace e2e_noa::planning {

using SubsystemList = std::vector<std::shared_ptr<SinglePlayerDynamicalSystem>>;

class ConcatenatedDynamicalSystem : public MultiPlayerDynamicalSystem {
 public:
  ~ConcatenatedDynamicalSystem() {}
  ConcatenatedDynamicalSystem(const SubsystemList& subsystems);

  // Compute time derivative of state.
  VectorXf Evaluate(Time t, const VectorXf& x,
                    const std::vector<VectorXf>& us) const;

  // Compute a discrete-time Jacobian linearization.
  LinearDynamicsApproximation Linearize(Time t, const VectorXf& x,
                                        const std::vector<VectorXf>& us) const;

  // Distance metric between two states.
  float DistanceBetween(const VectorXf& x0, const VectorXf& x1) const;

  // Stitch between two states of the system. Interprets the first one as best
  // for ego and the second as best for other players.
  VectorXf Stitch(const VectorXf& x_ego, const VectorXf& x_others) const {
    VectorXf x(x_ego.size());

    const Dimension ego_state_dim = subsystems_[0]->XDim();
    x.head(ego_state_dim) = x_ego.head(ego_state_dim);
    x.tail(x_others.size() - ego_state_dim) =
        x_others.tail(x_others.size() - ego_state_dim);

    return x;
  }

  // Getters.
  const SubsystemList& Subsystems() const { return subsystems_; }
  PlayerIndex NumPlayers() const { return subsystems_.size(); }
  Dimension SubsystemStartDim(PlayerIndex player_idx) const {
    return subsystem_start_dims_[player_idx];
  }
  Dimension SubsystemXDim(PlayerIndex player_idx) const {
    return subsystems_[player_idx]->XDim();
  }
  Dimension UDim(PlayerIndex player_idx) const {
    return subsystems_[player_idx]->UDim();
  }
  std::vector<Dimension> PositionDimensions() const;

 private:
  // List of subsystems, each of which controls the affects of a single player.
  const SubsystemList subsystems_;

  // Cumulative sum of dimensions of each subsystem.
  std::vector<Dimension> subsystem_start_dims_;
};  // namespace ilqgames

}  // namespace e2e_noa::planning

#endif
