/////////////////////////////////////////////////////////////////////////////
//
// Base class for all multi-player *integrable* dynamical systems.
// Supports (discrete-time) linearization and integration.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef ILQGAMES_DYNAMICS_MULTI_PLAYER_INTEGRABLE_SYSTEM_H
#define ILQGAMES_DYNAMICS_MULTI_PLAYER_INTEGRABLE_SYSTEM_H

#include <vector>

#include "ilq_games/include/utils/ilq_types.h"
#include "ilq_games/include/utils/operating_point.h"
#include "ilq_games/include/utils/strategy.h"

namespace e2e_noa::planning {

class MultiPlayerIntegrableSystem {
 public:
  virtual ~MultiPlayerIntegrableSystem() {}

  // Integrate these dynamics forward in time.
  // Options include integration for a single timestep, between arbitrary times,
  // and within a single timestep.
  virtual VectorXf Integrate(Time t0, Time time_interval, const VectorXf& x0,
                             const std::vector<VectorXf>& us) const = 0;
  VectorXf Integrate(Time t0, Time t, const VectorXf& x0,
                     const OperatingPoint& operating_point,
                     const std::vector<Strategy>& strategies) const;
  VectorXf Integrate(size_t initial_timestep, size_t final_timestep,
                     const VectorXf& x0, const OperatingPoint& operating_point,
                     const std::vector<Strategy>& strategies) const;
  VectorXf IntegrateToNextTimeStep(
      Time t0, const VectorXf& x0, const OperatingPoint& operating_point,
      const std::vector<Strategy>& strategies) const;
  VectorXf IntegrateFromPriorTimeStep(
      Time t, const VectorXf& x0, const OperatingPoint& operating_point,
      const std::vector<Strategy>& strategies) const;

  // Make a utility version of the above that operates on Eigen::Refs.
  VectorXf Integrate(Time t0, Time time_interval,
                     const Eigen::Ref<VectorXf>& x0,
                     const std::vector<Eigen::Ref<VectorXf>>& us) const;

  // Can this system be treated as linear for the purposes of LQ solves?
  // For example, linear systems and feedback linearizable systems should
  // return true here.
  virtual bool TreatAsLinear() const { return false; }

  // Stitch between two states of the system. By default, just takes the
  // first one but concatenated systems, e.g., can interpret the first one
  // as best for ego and the second as best for other players.
  virtual VectorXf Stitch(const VectorXf& x_ego,
                          const VectorXf& x_others) const {
    return x_ego;
  }

  // Integrate using single step Euler or not, see below for more extensive
  // description.
  static void IntegrateUsingEuler() { integrate_using_euler_ = true; }
  static void IntegrateUsingRK4() { integrate_using_euler_ = false; }
  static bool IntegrationUsesEuler() { return integrate_using_euler_; }

  // Getters.
  Dimension XDim() const { return xdim_; }
  Dimension TotalUDim() const {
    Dimension total = 0;
    for (PlayerIndex ii = 0; ii < NumPlayers(); ii++) total += UDim(ii);
    return total;
  }
  virtual Dimension UDim(PlayerIndex player_idx) const = 0;
  virtual PlayerIndex NumPlayers() const = 0;
  virtual std::vector<Dimension> PositionDimensions() const = 0;

  // Distance metric between two states. By default, just the *squared* 2-norm.
  virtual float DistanceBetween(const VectorXf& x0, const VectorXf& x1) const {
    return (x0 - x1).squaredNorm();
  }

 protected:
  MultiPlayerIntegrableSystem(Dimension xdim) : xdim_(xdim) {}

  // State dimension.
  const Dimension xdim_;

  // Whether to use single Euler during integration. Typically this is false but
  // it is typically used either for testing (we only derive Nash typically in
  // this case) or for speed.
  static bool integrate_using_euler_;
};  //\class MultiPlayerIntegrableSystem

}  // namespace e2e_noa::planning

#endif
