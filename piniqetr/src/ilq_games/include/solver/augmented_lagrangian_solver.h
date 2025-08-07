////////////////////////////////////////////////////////////////////////////
//
// Solver that implements an augmented Lagrangian method. For reference on these
// methods, please refer to Chapter 17 of Nocedal and Wright or the ALTRO paper:
// https://bjack205.github.io/assets/ALTRO.pdf.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef ILQGAMES_SOLVER_AUGMENTED_LAGRANGIAN_SOLVER_H
#define ILQGAMES_SOLVER_AUGMENTED_LAGRANGIAN_SOLVER_H

#include <glog/logging.h>

#include <chrono>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

#include "ilq_games/include/dynamics/multi_player_dynamical_system.h"
#include "ilq_games/include/dynamics/multi_player_integrable_system.h"
#include "ilq_games/include/examples/muti_vehicle_game.h"
#include "ilq_games/include/solver/game_solver.h"
#include "ilq_games/include/solver/ilq_solver.h"
#include "ilq_games/include/solver/lq_feedback_solver.h"
#include "ilq_games/include/solver/lq_solver.h"
#include "ilq_games/include/solver/problem.h"
#include "ilq_games/include/utils/ilq_types.h"
#include "ilq_games/include/utils/linear_dynamics_approximation.h"
#include "ilq_games/include/utils/operating_point.h"
#include "ilq_games/include/utils/quadratic_cost_approximation.h"
#include "ilq_games/include/utils/solver_log.h"
#include "ilq_games/include/utils/strategy.h"

namespace e2e_noa::planning {

class AugmentedLagrangianSolver : public GameSolver {
 public:
  ~AugmentedLagrangianSolver() {}
  AugmentedLagrangianSolver(const std::shared_ptr<Problem> &problem,
                            const SolverParams &params)
      : GameSolver(problem, params) {
    // Modify parameters for unconstrained solver.
    SolverParams unconstrained_solver_params(params);
    unconstrained_solver_params.max_solver_iters =
        params.unconstrained_solver_max_iters;
    unconstrained_solver_.reset(
        new ILQSolver(problem, unconstrained_solver_params));
  }

  // Solve this game. Returns true if converged. Defaults to 5 s runtime.
  std::shared_ptr<SolverLog> Solve(bool *success = nullptr,
                                   Time max_runtime = 5.0);

  OperatingPoint GetFinalOperatorPoint() const {
    return final_operating_point_;
  }

 private:
  // Lower level (unconstrained) solver.
  std::unique_ptr<ILQSolver> unconstrained_solver_;
  OperatingPoint final_operating_point_;
};  // class AugmentedLagrangianSolver

}  // namespace e2e_noa::planning

#endif
