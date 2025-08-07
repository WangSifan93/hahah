//////////////////////////////////////////////////////////////////////////////
//
// Base class for all iterative LQ game solvers.
// Structured so that derived classes may only modify the `ModifyLQStrategies`
// and `HasConverged` virtual functions.
//
///////////////////////////////////////////////////////////////////////////////

#include "ilq_games/include/solver/ilq_solver.h"

#include <glog/logging.h>

#include <chrono>
#include <memory>
#include <numeric>
#include <vector>

#include "ilq_games/include/cost/player_cost.h"
#include "ilq_games/include/solver/lq_solver.h"
#include "ilq_games/include/utils/ilq_types.h"
#include "ilq_games/include/utils/linear_dynamics_approximation.h"
#include "ilq_games/include/utils/operating_point.h"
#include "ilq_games/include/utils/quadratic_cost_approximation.h"
#include "ilq_games/include/utils/strategy.h"

namespace e2e_noa::planning {

namespace {
// Multiply all alphas in a set of strategies by the given constant.
void ScaleAlphas(float scaling, std::vector<Strategy> *strategies) {
  for (auto &strategy : *strategies) {
    for (auto &alpha : strategy.alphas) alpha *= scaling;
  }
}
}  // namespace

std::shared_ptr<SolverLog> ILQSolver::Solve(bool *success, Time max_runtime) {
  const auto solver_call_time = Clock::now();

  // Create a new log.
  std::shared_ptr<SolverLog> log = CreateNewLog();

  // Last and current operating points.
  OperatingPoint last_operating_point(problem_->CurrentOperatingPoint());
  OperatingPoint current_operating_point(problem_->CurrentOperatingPoint());
  current_operating_point.xs[0] = problem_->InitialState();
  last_operating_point.xs[0] = problem_->InitialState();

  // Current strategies.
  std::vector<Strategy> current_strategies(problem_->CurrentStrategies());

  // Things to keep track of during each iteration.
  size_t num_iterations = 0;
  bool has_converged = false;

  // Swap operating points and compute new current operating point.
  last_operating_point.swap(current_operating_point);
  CurrentOperatingPoint(last_operating_point, current_strategies,
                        &current_operating_point);
  // Quadraticize costs before first iteration.
  ComputeCostQuadraticization(current_operating_point, &cost_quadraticization_);

  // Maintain delta_xs and costates for linesearch.
  std::vector<VectorXf> delta_xs;
  std::vector<std::vector<VectorXf>> costates;

  // Main loop with timer for anytime execution.
  while (num_iterations < params_.max_solver_iters && !has_converged) {
    num_iterations++;
    if (!problem_->Dynamics()->TreatAsLinear()) {
      ComputeLinearization(current_operating_point, &linearization_);
    }
    // Step 1: Solve LQ game.
    current_strategies = lq_solver_->Solve(
        linearization_, cost_quadraticization_,
        problem_->InitialState() - current_operating_point.xs.front(),
        &delta_xs, &costates);

    // Step 2: Modify LQ strategies.
    bool modify_success =
        ModifyLQStrategies(delta_xs, costates, &current_strategies,
                           &current_operating_point, &has_converged);

    if (!modify_success) {
      if (success) *success = false;
      return log;
    }
  }

  log->AddSolverIterate(current_operating_point, current_strategies);

  if (success) *success = true;
  return log;
}

void ILQSolver::CurrentOperatingPoint(
    const OperatingPoint &last_operating_point,
    const std::vector<Strategy> &current_strategies,
    OperatingPoint *current_operating_point) const {
  current_operating_point->t0 = last_operating_point.t0;

  const size_t num_time_steps = time::getNumTimeSteps();
  const size_t num_players = problem_->Dynamics()->NumPlayers();
  const double time_step = time::getTimeStep();

  const auto &dynamics = *problem_->Dynamics();

  VectorXf x = last_operating_point.xs[0];
  current_operating_point->xs.resize(num_time_steps);
  current_operating_point->us.resize(num_time_steps);

  for (size_t kk = 0; kk < num_time_steps; kk++) {
    auto &current_x_kk = current_operating_point->xs[kk];
    auto &current_us_kk = current_operating_point->us[kk];
    const auto &last_x_kk = last_operating_point.xs[kk];
    const auto &last_us_kk = last_operating_point.us[kk];

    current_x_kk = x;
    current_us_kk.resize(num_players);

    // 计算状态偏差
    VectorXf delta_x = x - last_x_kk;

    for (PlayerIndex jj = 0; jj < num_players; jj++) {
      // 使用策略计算控制输入
      current_us_kk[jj] = current_strategies[jj](kk, delta_x, last_us_kk[jj]);
    }

    if (kk < num_time_steps - 1) {
      const Time t = RelativeTimeTracker::RelativeTime(kk);
      x = dynamics.Integrate(t, time_step, std::move(x), current_us_kk);
    }
  }
}

float ILQSolver::StateDistance(const VectorXf &x1, const VectorXf &x2,
                               const std::vector<Dimension> &dims) const {
  auto total_distance = [&dims](const VectorXf &x1, const VectorXf &x2) {
    if (dims.empty()) return (x1 - x2).cwiseAbs().maxCoeff();

    float distance = 0.0;
    for (const Dimension dim : dims) distance += std::abs(x1(dim) - x2(dim));

    return distance;
  };  // total_distance

  return total_distance(x1, x2);
}

bool ILQSolver::ModifyLQStrategies(
    const std::vector<VectorXf> &delta_xs,
    const std::vector<std::vector<VectorXf>> &costates,
    std::vector<Strategy> *strategies, OperatingPoint *current_operating_point,
    bool *has_converged) {
  // Precompute expected decrease before we do anything else.
  expected_decrease_ = ExpectedDecrease(*strategies, delta_xs, costates);

  if (params_.linesearch)
    last_cost_quadraticization_.swap(cost_quadraticization_);

  // Initially scale alphas by a fixed amount to avoid unnecessary
  ScaleAlphas(params_.initial_alpha_scaling, strategies);

  // Compute next operating point and keep track of whether it satisfies the
  // Armijo condition.
  const OperatingPoint last_operating_point(*current_operating_point);
  float current_stepsize = params_.initial_alpha_scaling;

  CurrentOperatingPoint(last_operating_point, *strategies,
                        current_operating_point);
  if (!params_.linesearch) return true;

  // Keep reducing alphas until we satisfy the Armijo condition.
  for (size_t ii = 0; ii < params_.max_backtracking_steps; ii++) {
    // Compute merit function value.
    const float current_merit_function_value =
        MeritFunction(*current_operating_point, costates);

    // Check Armijo condition.
    if (CheckArmijoCondition(current_merit_function_value, current_stepsize)) {
      // Success! Update cached terms and check convergence.
      *has_converged = HasConverged(current_merit_function_value);
      last_merit_function_value_ = current_merit_function_value;

      return true;
    }

    // Scale down the alphas and try again.
    ScaleAlphas(params_.geometric_alpha_scaling, strategies);
    current_stepsize *= params_.geometric_alpha_scaling;
    CurrentOperatingPoint(last_operating_point, *strategies,
                          current_operating_point);
  }

  return false;
}

bool ILQSolver::CheckArmijoCondition(float current_merit_function_value,
                                     float current_stepsize) const {
  // Adjust total expected decrease.
  const float scaled_expected_decrease = params_.expected_decrease_fraction *
                                         current_stepsize * expected_decrease_;

  return (last_merit_function_value_ - current_merit_function_value >=
          scaled_expected_decrease);
}

float ILQSolver::ExpectedDecrease(
    const std::vector<Strategy> &strategies,
    const std::vector<VectorXf> &delta_xs,
    const std::vector<std::vector<VectorXf>> &costates) const {
  float expected_decrease = 0.0;
  const size_t num_time_steps = time::getNumTimeSteps();
  const size_t num_players = problem_->Dynamics()->NumPlayers();
  for (size_t kk = 0; kk < num_time_steps; kk++) {
    const auto &lin = linearization_[kk];

    for (PlayerIndex ii = 0; ii < num_players; ii++) {
      const auto &quad = cost_quadraticization_[kk][ii];
      const auto &costate = costates[kk][ii];
      const auto &neg_ui = strategies[ii].alphas[kk];

      // Accumulate state and control contributions.
      const VectorXf dLdu = quad.control.at(ii).grad;
      expected_decrease -= neg_ui.dot(quad.control.at(ii).hess * dLdu);
      if (kk > 0) {
        const auto &last_costate = costates[kk - 1][ii];
        const VectorXf dLdx = quad.state.grad;
        expected_decrease -= delta_xs[kk].dot(quad.state.hess * dLdx);
      }
    }
  }

  return expected_decrease;
}

float ILQSolver::MeritFunction(
    const OperatingPoint &current_op,
    const std::vector<std::vector<VectorXf>> &costates) {
  // First, quadraticize cost and linearize dynamics around this operating
  // point.
  ComputeCostQuadraticization(current_op, &cost_quadraticization_);

  // Accumulate gradients of Lagrangian (dynamics automatically satisfied).
  float merit = 0.0;
  const size_t num_time_steps = time::getNumTimeSteps();
  const size_t num_players = problem_->Dynamics()->NumPlayers();
  for (size_t kk = 0; kk < num_time_steps; kk++) {
    const auto &lin = linearization_[kk];

    for (PlayerIndex ii = 0; ii < num_players; ii++) {
      const auto &quad = cost_quadraticization_[kk][ii];
      const auto &costate = costates[kk][ii];

      // Accumulate state and control contributions.
      merit += quad.control.at(ii).grad.squaredNorm();

      if (kk > 0) {
        merit += quad.state.grad.squaredNorm();
      }
    }
  }
  return 0.5 * merit;
}

void ILQSolver::ComputeLinearization(
    const OperatingPoint &op,
    std::vector<LinearDynamicsApproximation> *linearization) {
  CHECK_NOTNULL(linearization);

  // Check if linearization is the right length.
  if (linearization->size() != op.xs.size())
    linearization->resize(op.xs.size());

  // Cast dynamics to appropriate type.
  const auto dyn = static_cast<const MultiPlayerDynamicalSystem *>(
      problem_->Dynamics().get());

  // Populate one timestep at a time.
  for (size_t kk = 0; kk < op.xs.size(); kk++) {
    const Time t = RelativeTimeTracker::RelativeTime(kk);
    (*linearization)[kk] = dyn->Linearize(t, op.xs[kk], op.us[kk]);
  }
}

void ILQSolver::ComputeCostQuadraticization(
    const OperatingPoint &op,
    std::vector<std::vector<QuadraticCostApproximation>> *q) {
  for (size_t kk = 0; kk < time::getNumTimeSteps(); kk++) {
    const Time t = RelativeTimeTracker::RelativeTime(kk);
    const auto &x = op.xs[kk];
    const auto &us = op.us[kk];

    // Quadraticize costs.
    for (PlayerIndex ii = 0; ii < problem_->Dynamics()->NumPlayers(); ii++) {
      const PlayerCost &cost = problem_->PlayerCosts()[ii];

      if (cost.IsTimeAdditive() ||
          problem_->PlayerCosts()[ii].TimeOfExtremeCost() == kk)
        (*q)[kk][ii] = cost.Quadraticize(t, x, us);
      else
        (*q)[kk][ii] = cost.QuadraticizeControlCosts(t, x, us);
    }
  }
}

}  // namespace e2e_noa::planning
