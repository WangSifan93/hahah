/////////////////////////////////////////////////////////////////////////////
//
// Base class for all iterative LQ game solvers. Derives from FeedbackSolver.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef ILQGAMES_SOLVER_ILQ_SOLVER_H
#define ILQGAMES_SOLVER_ILQ_SOLVER_H

#include <glog/logging.h>

#include <limits>
#include <memory>
#include <vector>

#include "ilq_games/include/cost/player_cost.h"
#include "ilq_games/include/dynamics/multi_player_dynamical_system.h"
#include "ilq_games/include/examples/muti_vehicle_game.h"
#include "ilq_games/include/solver/game_solver.h"
#include "ilq_games/include/solver/lq_feedback_solver.h"
#include "ilq_games/include/solver/lq_solver.h"
#include "ilq_games/include/utils/ilq_types.h"
#include "ilq_games/include/utils/linear_dynamics_approximation.h"
#include "ilq_games/include/utils/operating_point.h"
#include "ilq_games/include/utils/quadratic_cost_approximation.h"
#include "ilq_games/include/utils/solver_log.h"
#include "ilq_games/include/utils/strategy.h"

namespace e2e_noa::planning {

class ILQSolver : public GameSolver {
 public:
  virtual ~ILQSolver() {}
  ILQSolver(const std::shared_ptr<Problem> &problem,
            const SolverParams &params = SolverParams())
      : GameSolver(problem, params),
        linearization_(time::getNumTimeSteps()),
        cost_quadraticization_(time::getNumTimeSteps()),
        last_merit_function_value_(constants::kInfinity),
        expected_decrease_(constants::kInfinity) {
    // Set up LQ solver.
    lq_solver_.reset(
        new LQFeedbackSolver(problem_->Dynamics(), time::getNumTimeSteps()));

    // Prepopulate quadraticization.
    for (auto &quads : cost_quadraticization_)
      quads.resize(problem_->Dynamics()->NumPlayers(),
                   QuadraticCostApproximation(problem_->Dynamics()->XDim()));

    // Set last quadraticization to current, to start.
    last_cost_quadraticization_ = cost_quadraticization_;
  }

  // Solve this game. Returns true if converged.
  virtual std::shared_ptr<SolverLog> Solve(
      bool *success = nullptr,
      Time max_runtime = std::numeric_limits<Time>::infinity());

  // Accessors.
  // NOTE: these should be primarily used by higher-level solvers.
  std::vector<std::vector<QuadraticCostApproximation>> *Quadraticization() {
    return &cost_quadraticization_;
  }

  std::vector<LinearDynamicsApproximation> *Linearization() {
    return &linearization_;
  }

  OperatingPoint GetFinalOperatorPoint() const {
    return final_operating_point_;
  }

 protected:
  // Modify LQ strategies to improve convergence properties.
  // This function performs an Armijo linesearch and returns true if successful.
  bool ModifyLQStrategies(const std::vector<VectorXf> &delta_xs,
                          const std::vector<std::vector<VectorXf>> &costates,
                          std::vector<Strategy> *strategies,
                          OperatingPoint *current_operating_point,
                          bool *has_converged);

  // Compute distance (infinity norm) between states in the given dimensions.
  // If dimensions empty, checks all dimensions.
  float StateDistance(const VectorXf &x1, const VectorXf &x2,
                      const std::vector<Dimension> &dims) const;

  // Check if solver has converged.
  virtual bool HasConverged(float current_merit_function_value) const {
    return (current_merit_function_value <= last_merit_function_value_) &&
           std::abs(last_merit_function_value_ - current_merit_function_value) <
               params_.convergence_tolerance;
  }

  // Armijo condition check. Returns true if the new operating point satisfies
  // the Armijo condition, and also returns current merit function value.
  bool CheckArmijoCondition(float current_merit_function_value,
                            float current_stepsize) const;

  // Compute current merit function value. Note that to compute the merit
  // function at the given operating point we have to compute a full cost
  // quadraticization there. To do so efficiently, this will overwrite the
  // current cost quadraticization (and presume it has already been used to
  // compute the expected decrease from the last iterate).
  float MeritFunction(const OperatingPoint &current_op,
                      const std::vector<std::vector<VectorXf>> &costates);

  // Compute expected decrease based on current cost quadraticization,
  // (player-indexed) strategies, and (time-indexed) lists of delta states and
  // (also player-indexed) costates.
  float ExpectedDecrease(
      const std::vector<Strategy> &strategies,
      const std::vector<VectorXf> &delta_xs,
      const std::vector<std::vector<VectorXf>> &costates) const;

  // Compute the current operating point based on the current set of
  // strategies and the last operating point.
  void CurrentOperatingPoint(const OperatingPoint &last_operating_point,
                             const std::vector<Strategy> &current_strategies,
                             OperatingPoint *current_operating_point) const;

  // Populate the given vector with a linearization of the dynamics about
  // the given operating point. Provide version with no operating point for use
  // with feedback linearizable systems.
  void ComputeLinearization(
      const OperatingPoint &op,
      std::vector<LinearDynamicsApproximation> *linearization);

  // Compute the quadratic cost approximation at the given operating point.
  void ComputeCostQuadraticization(
      const OperatingPoint &op,
      std::vector<std::vector<QuadraticCostApproximation>> *q);

  // Linearization and quadraticization. Both are time-indexed (and
  // quadraticizations' inner vector is indexed by player). Also keep track of
  // the quadraticization from last iteration.
  std::vector<LinearDynamicsApproximation> linearization_;
  std::vector<std::vector<QuadraticCostApproximation>> cost_quadraticization_;
  std::vector<std::vector<QuadraticCostApproximation>>
      last_cost_quadraticization_;

  // Core LQ Solver.
  std::unique_ptr<LQSolver> lq_solver_;

  // Last merit function value and expected decreases (per step length).
  OperatingPoint final_operating_point_;
  float last_merit_function_value_;
  float expected_decrease_;
};  // class ILQSolver

}  // namespace e2e_noa::planning

#endif
