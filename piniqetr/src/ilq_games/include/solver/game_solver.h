////////////////////////////////////////////////////////////////////////////
//
// Base class for all game solvers. All solvers will need linearization,
// quadraticization, and loop timing.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef ILQGAMES_SOLVER_GAME_SOLVER_H
#define ILQGAMES_SOLVER_GAME_SOLVER_H

#include <glog/logging.h>

#include <chrono>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

#include "ilq_games/include/dynamics/multi_player_dynamical_system.h"
#include "ilq_games/include/dynamics/multi_player_integrable_system.h"
#include "ilq_games/include/examples/muti_vehicle_game.h"
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
class GameSolver {
 public:
  virtual ~GameSolver() {}

  // Solve this game. Returns true if converged.
  virtual std::shared_ptr<SolverLog> Solve(
      bool* success = nullptr, Time max_runtime = constants::kInfinity) = 0;

  // Accessors.
  Problem& GetProblem() { return *problem_; }

 protected:
  GameSolver(const std::shared_ptr<Problem>& problem,
             const SolverParams& params)
      : problem_(problem), params_(params) {
    CHECK_NOTNULL(problem_.get());
    CHECK_NOTNULL(problem_->Dynamics().get());
  }

  // Create a new log. This may be overridden by derived classes (e.g., to
  // change the name of the log).
  virtual std::shared_ptr<SolverLog> CreateNewLog() const {
    return std::make_shared<SolverLog>();
  }

  // Store the underlying problem.
  const std::shared_ptr<Problem> problem_;

  // Solver parameters.
  const SolverParams params_;
};  // class GameSolver

}  // namespace e2e_noa::planning

#endif
