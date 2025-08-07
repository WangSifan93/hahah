////////////////////////////////////////////////////////////////////////////
//
// Quadratic cost in a particular (or all) dimension(s).
//
///////////////////////////////////////////////////////////////////////////////

#ifndef ILQGAMES_COST_QUADRATIC_COST_H
#define ILQGAMES_COST_QUADRATIC_COST_H

#include <string>

#include "ilq_games/include/cost/time_invariant_cost.h"
#include "ilq_games/include/utils/ilq_types.h"

namespace e2e_noa::planning {

class QuadraticCost : public TimeInvariantCost {
 public:
  // Construct from a multiplicative weight and the dimension in which to apply
  // the quadratic cost (difference from nominal). If dimension < 0, then
  // applies to all dimensions (i.e. ||input - nominal * ones()||^2).
  QuadraticCost(float weight, Dimension dim, float nominal = 0.0,
                const std::string& name = "")
      : TimeInvariantCost(weight, name), dimension_(dim), nominal_(nominal) {}

  // Evaluate this cost at the current input.
  float Evaluate(const VectorXf& input) const;

  // Quadraticize this cost at the given input, and add to the running
  // sum of gradients and Hessians.
  void Quadraticize(const VectorXf& input, MatrixXf* hess,
                    VectorXf* grad) const;

 private:
  // Dimension in which to apply the quadratic cost.
  const Dimension dimension_;

  // Nominal value in this (or all) dimensions.
  const float nominal_;
};  //\class QuadraticCost

}  // namespace e2e_noa::planning

#endif
