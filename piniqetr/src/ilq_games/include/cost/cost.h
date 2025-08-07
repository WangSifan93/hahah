//////////////////////////////////////////////////////////////////////////////
//
// Base class for all cost functions. All costs must support evaluation and
// quadraticization. By default, cost functions are of only state or control.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef ILQGAMES_COST_COST_H
#define ILQGAMES_COST_COST_H

#include <glog/logging.h>

#include <string>

#include "ilq_games/include/utils/ilq_types.h"
#include "ilq_games/include/utils/relative_time_tracker.h"

namespace e2e_noa::planning {

class Cost : public RelativeTimeTracker {
 public:
  virtual ~Cost() {}

  // Evaluate this cost at the current time and input.
  virtual float Evaluate(Time t, const VectorXf &input) const = 0;

  // Quadraticize this cost at the given time and input, and add to the running
  // sum of gradients and Hessians.
  virtual void Quadraticize(Time t, const VectorXf &input, MatrixXf *hess,
                            VectorXf *grad) const = 0;

  // Reset and scale weight.
  void SetWeight(float weight) { weight_ = weight; }
  void ScaleWeight(float scale) { weight_ *= scale; }

 protected:
  explicit Cost(float weight, const std::string &name)
      : RelativeTimeTracker(name), weight_(weight) {}

  // Multiplicative weight associated to this cost.
  float weight_;
};  //\class Cost

}  // namespace e2e_noa::planning

#endif
