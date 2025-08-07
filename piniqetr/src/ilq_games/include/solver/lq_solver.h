/////////////////////////////////////////////////////////////////////////////
//
//  Base class for all LQ game solvers. For further details please refer to
//  derived class comments.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef ILQGAMES_SOLVER_LQ_SOLVER_H
#define ILQGAMES_SOLVER_LQ_SOLVER_H

#include <glog/logging.h>

#include <vector>

#include "ilq_games/include/dynamics/multi_player_integrable_system.h"
#include "ilq_games/include/utils/linear_dynamics_approximation.h"
#include "ilq_games/include/utils/quadratic_cost_approximation.h"
#include "ilq_games/include/utils/strategy.h"

namespace e2e_noa::planning {

class LQSolver {
 public:
  virtual ~LQSolver() {}

  // Solve underlying LQ game to a Nash equilibrium. This will differ in derived
  // classes depending on the information structure of the game.
  // Optionally return delta xs and costates.
  virtual std::vector<Strategy> Solve(
      const std::vector<LinearDynamicsApproximation>& linearization,
      const std::vector<std::vector<QuadraticCostApproximation>>&
          quadraticization,
      const VectorXf& x0, std::vector<VectorXf>* delta_xs = nullptr,
      std::vector<std::vector<VectorXf>>* costates = nullptr) = 0;

 protected:
  LQSolver(const std::shared_ptr<const MultiPlayerIntegrableSystem>& dynamics,
           size_t num_time_steps)
      : dynamics_(dynamics), num_time_steps_(num_time_steps) {
    CHECK_NOTNULL(dynamics.get());
  }

  // Dynamics and number of time steps.
  const std::shared_ptr<const MultiPlayerIntegrableSystem> dynamics_;
  const size_t num_time_steps_;
};  // class LQSolver

}  // namespace e2e_noa::planning

#endif
