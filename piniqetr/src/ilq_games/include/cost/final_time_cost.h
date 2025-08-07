////////////////////////////////////////////////////////////////////////
//
// Class for applying a given cost only after a fixed time.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef ILQGAMES_COST_FINAL_TIME_COST_H
#define ILQGAMES_COST_FINAL_TIME_COST_H

#include <glog/logging.h>

#include <memory>
#include <string>

#include "ilq_games/include/cost/cost.h"
#include "ilq_games/include/utils/ilq_types.h"

namespace e2e_noa::planning {

class FinalTimeCost : public Cost {
 public:
  ~FinalTimeCost() {}
  FinalTimeCost(const std::shared_ptr<const Cost>& cost, Time threshold_time,
                const std::string& name = "")
      : Cost(0.0, name), cost_(cost), threshold_time_(threshold_time) {
    CHECK_NOTNULL(cost.get());
  }

  // Evaluate this cost at the current time and input.
  float Evaluate(Time t, const VectorXf& input) const {
    return (t >= initial_time_ + threshold_time_) ? cost_->Evaluate(t, input)
                                                  : 0.0;
  }

  // Quadraticize this cost at the given time and input, and add to the running
  // sum of gradients and Hessians.
  void Quadraticize(Time t, const VectorXf& input, MatrixXf* hess,
                    VectorXf* grad) const {
    if (t < initial_time_ + threshold_time_) return;
    cost_->Quadraticize(t, input, hess, grad);
  }

 private:
  // Cost function.
  const std::shared_ptr<const Cost> cost_;

  // Time threshold relative to initial time after which to apply cost.
  const Time threshold_time_;
};  //\class Cost

}  // namespace e2e_noa::planning

#endif
