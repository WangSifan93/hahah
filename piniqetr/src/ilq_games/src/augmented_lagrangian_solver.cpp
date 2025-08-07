///////////////////////////////////////////////////////////////////////////////
//
// Solver that implements an augmented Lagrangian method. For reference on these
// methods, please refer to Chapter 17 of Nocedal and Wright or the ALTRO paper:
// https://bjack205.github.io/assets/ALTRO.pdf.
//
///////////////////////////////////////////////////////////////////////////////

#include "ilq_games/include/solver/augmented_lagrangian_solver.h"

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

std::shared_ptr<SolverLog> AugmentedLagrangianSolver::Solve(bool *success,
                                                            Time max_runtime) {
  if (success) *success = true;

  // Cache initial problem solution so we can restore it at the end.
  const auto &initial_op = problem_->CurrentOperatingPoint();
  const auto &initial_strategies = problem_->CurrentStrategies();

  // Create new log.
  std::shared_ptr<SolverLog> log = CreateNewLog();

  // Determine how much time should be allocated for any individual lower level
  // solver call.
  const Time max_runtime_unconstrained_problem = max_runtime;

  // Solve unconstrained problem.
  bool unconstrained_success = false;
  const auto unconstrained_log = unconstrained_solver_->Solve(
      &unconstrained_success, max_runtime_unconstrained_problem);
  log->AddLog(*unconstrained_log);
  final_operating_point_ = unconstrained_solver_->GetFinalOperatorPoint();

  if (success) *success &= unconstrained_success;

  return log;
}

}  // namespace e2e_noa::planning
