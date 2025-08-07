/////////////////////////////////////////////////////////////////////////////
//
// Base class for all time-invariant cost functions.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef ILQGAMES_COST_TIME_INVARIANT_COST_H
#define ILQGAMES_COST_TIME_INVARIANT_COST_H

#include <string>

#include "ilq_games/include/cost/cost.h"
#include "ilq_games/include/utils/ilq_types.h"

namespace e2e_noa::planning {

class TimeInvariantCost : public Cost {
 public:
  virtual ~TimeInvariantCost() {}

  // Evaluate this cost at the given input.
  virtual float Evaluate(const VectorXf& input) const = 0;
  float Evaluate(Time t, const VectorXf& input) const {
    return Evaluate(input);
  }

  // Quadraticize this cost at the given input, and add to the running set of
  // sum of gradients and Hessians.
  virtual void Quadraticize(const VectorXf& input, MatrixXf* hess,
                            VectorXf* grad) const = 0;
  void Quadraticize(Time t, const VectorXf& input, MatrixXf* hess,
                    VectorXf* grad) const {
    Quadraticize(input, hess, grad);
  }

 protected:
  explicit TimeInvariantCost(float weight, const std::string& name)
      : Cost(weight, name) {}
};  //\class TimeInvariantCost

}  // namespace e2e_noa::planning

#endif
