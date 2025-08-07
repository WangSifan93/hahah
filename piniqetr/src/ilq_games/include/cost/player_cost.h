//////////////////////////////////////////////////////////////////////////////
//
// Container to store all the cost functions for a single player, and keep track
// of which variables (x, u1, u2, ..., uN) they correspond to.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef ILQGAMES_COST_PLAYER_COST_H
#define ILQGAMES_COST_PLAYER_COST_H

#include <unordered_map>

#include "ilq_games/include/cost/cost.h"
#include "ilq_games/include/utils/ilq_types.h"
#include "ilq_games/include/utils/operating_point.h"
#include "ilq_games/include/utils/quadratic_cost_approximation.h"

namespace e2e_noa::planning {

class PlayerCost {
 public:
  ~PlayerCost() {}

  // Provide default values for all constructor values. If num_time_steps is
  // positive use that to initialize the lambdas.
  explicit PlayerCost(const std::string& name = "",
                      float state_regularization = 0.0,
                      float control_regularization = 0.0)
      : name_(name),
        state_regularization_(state_regularization),
        control_regularization_(control_regularization),
        cost_structure_(CostStructure::SUM),
        time_of_extreme_cost_(0) {}

  // Add new state and control costs for this player.
  void AddStateCost(const std::shared_ptr<Cost>& cost);
  void AddControlCost(PlayerIndex idx, const std::shared_ptr<Cost>& cost);

  // Evaluate this cost at the current time, state, and controls, or
  // integrate over an entire trajectory. The "Offset" here indicates that
  // state costs will be evaluated at the next time step.
  float Evaluate(Time t, const VectorXf& x,
                 const std::vector<VectorXf>& us) const;
  float Evaluate(const OperatingPoint& op, Time time_step) const;
  float Evaluate(const OperatingPoint& op) const;
  float EvaluateOffset(Time t, Time next_t, const VectorXf& next_x,
                       const std::vector<VectorXf>& us) const;

  // Quadraticize this cost at the given time, time step, state, and controls.
  QuadraticCostApproximation Quadraticize(
      Time t, const VectorXf& x, const std::vector<VectorXf>& us) const;

  // Return empty cost quadraticization except for control costs.
  QuadraticCostApproximation QuadraticizeControlCosts(
      Time t, const VectorXf& x, const std::vector<VectorXf>& us) const;

  // Set whether this is a time-additive, max-over-time, or min-over-time cost.
  // At each specific time, all costs are accumulated with the given operation.
  enum CostStructure { SUM, MAX, MIN };
  void SetTimeAdditive() { cost_structure_ = SUM; }
  void SetMaxOverTime() { cost_structure_ = MAX; }
  void SetMinOverTime() { cost_structure_ = MIN; }
  bool IsTimeAdditive() const { return cost_structure_ == SUM; }
  bool IsMaxOverTime() const { return cost_structure_ == MAX; }
  bool IsMinOverTime() const { return cost_structure_ == MIN; }

  // Keep track of the time of extreme costs.
  size_t TimeOfExtremeCost() { return time_of_extreme_cost_; }
  void SetTimeOfExtremeCost(size_t kk) { time_of_extreme_cost_ = kk; }

  // Accessors.
  const PtrVector<Cost>& StateCosts() const { return state_costs_; }
  const PlayerPtrMultiMap<Cost>& ControlCosts() const { return control_costs_; }

 private:
  // Name to be used with error msgs.
  const std::string name_;

  // State costs and control costs.
  PtrVector<Cost> state_costs_;
  PlayerPtrMultiMap<Cost> control_costs_;

  // Regularization on costs.
  const float state_regularization_;
  const float control_regularization_;

  // Ternary variable whether this objective is time-additive, max-over-time, or
  // min-over-time.
  CostStructure cost_structure_;

  // Keep track of the time of extreme costs. This will depend upon the current
  // operating point, and it will only be meaningful if the cost structure is an
  // extremum over time.
  size_t time_of_extreme_cost_;
};  //\class PlayerCost

}  // namespace e2e_noa::planning

#endif
