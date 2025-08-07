/////////////////////////////////////////////////////////////////////////////
//
// Base class specifying the problem interface for managing calls to the core
// ILQGame solver. Specific examples will be derived from this class.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef ILQGAMES_SOLVER_PROBLEM_H
#define ILQGAMES_SOLVER_PROBLEM_H

#include <limits>
#include <memory>
#include <vector>

#include "ilq_games/include/cost/player_cost.h"
#include "ilq_games/include/dynamics/multi_player_dynamical_system.h"
#include "ilq_games/include/dynamics/multi_player_integrable_system.h"
#include "ilq_games/include/utils/ilq_types.h"
#include "ilq_games/include/utils/solver_log.h"
#include "ilq_games/include/utils/strategy.h"

namespace e2e_noa::planning {

class Problem {
 public:
  virtual ~Problem() {}

  // Initialize this object.
  virtual void Initialize() {
    ConstructDynamics();
    ConstructPlayerCosts();
    ConstructInitialState();
    ConstructInitialOperatingPoint();
    ConstructInitialStrategies();
    initialized_ = true;
  }

  // Reset the initial time and change nothing else.
  void ResetInitialTime(Time t0) {
    CHECK(initialized_);
    operating_point_->t0 = t0;
  }

  void ResetInitialState(const VectorXf& x0) {
    CHECK(initialized_);
    x0_ = x0;
  }

  // Update initial state and modify previous strategies and operating
  // points to start at the specified runtime after the current time t0.
  // Since time is continuous and we will want to maintain the same fixed
  // discretization, we will integrate x0 forward from t0 by approximately
  // planner_runtime, then find the nearest state in the existing plan to that
  // state, and start from there. By default, extends operating points and
  // strategies as follows:
  // 1. new controls are zero
  // 2. new states are those that result from zero control
  // 3. new strategies are also zero
  virtual void SetUpNextRecedingHorizon(const VectorXf& x0, Time t0,
                                        Time planner_runtime = 0.1);

  // Overwrite existing solution with the given operating point and strategies.
  // Truncates to fit in the same memory.
  virtual void OverwriteSolution(const OperatingPoint& operating_point,
                                 const std::vector<Strategy>& strategies);

  // Accessors.

  virtual Time InitialTime() const { return operating_point_->t0; }
  const VectorXf& InitialState() const { return x0_; }

  std::vector<PlayerCost>& PlayerCosts() { return player_costs_; }
  const std::vector<PlayerCost>& PlayerCosts() const { return player_costs_; }
  const std::shared_ptr<const MultiPlayerIntegrableSystem>& Dynamics() const {
    return dynamics_;
  }
  const MultiPlayerDynamicalSystem& NormalDynamics() const {
    CHECK(!dynamics_->TreatAsLinear());
    return *static_cast<const MultiPlayerDynamicalSystem*>(dynamics_.get());
  }

  virtual const OperatingPoint& CurrentOperatingPoint() const {
    return *operating_point_;
  }
  virtual const std::vector<Strategy>& CurrentStrategies() const {
    return *strategies_;
  }

 protected:
  Problem();

  // Functions for initialization. By default, operating point and strategies
  // are initialized to zero.
  virtual void ConstructDynamics() = 0;
  virtual void ConstructPlayerCosts() = 0;
  virtual void ConstructInitialState() = 0;
  virtual void ConstructInitialOperatingPoint() {
    operating_point_.reset(
        new OperatingPoint(time::getNumTimeSteps(), 0.0, dynamics_));
  }
  virtual void ConstructInitialStrategies() {
    strategies_.reset(new std::vector<Strategy>());
    for (PlayerIndex ii = 0; ii < dynamics_->NumPlayers(); ii++)
      strategies_->emplace_back(time::getNumTimeSteps(), dynamics_->XDim(),
                                dynamics_->UDim(ii));
  }

  // Utility used by SetUpNextRecedingHorizon. Integrate the given state
  // forward, set the new initial state and time, and return the first timestep
  // in the new problem.
  size_t SyncToExistingProblem(const VectorXf& x0, Time t0,
                               Time planner_runtime, OperatingPoint& op);

  // Dynamical system.
  std::shared_ptr<const MultiPlayerIntegrableSystem> dynamics_;

  // Player costs. These will not change during operation of this solver.
  std::vector<PlayerCost> player_costs_;

  // Initial condition.
  VectorXf x0_;

  // Strategies and operating points for all players.
  std::unique_ptr<OperatingPoint> operating_point_;
  std::unique_ptr<std::vector<Strategy>> strategies_;

  // Has this object been initialized?
  bool initialized_;
};  // class Problem

}  // namespace e2e_noa::planning

#endif
