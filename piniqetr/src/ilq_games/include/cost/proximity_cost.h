/////////////////////////////////////////////////////////////////////////////
//
// Penalizes (thresh - relative distance)^2 between two pairs of state
// dimensions (representing two positions of vehicles whose states have been
// concatenated) whenever relative distance is less than thresh.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef ILQGAMES_COST_PROXIMITY_COST_H
#define ILQGAMES_COST_PROXIMITY_COST_H

#include <string>
#include <utility>

#include "ilq_games/include/cost/time_invariant_cost.h"
#include "ilq_games/include/utils/ilq_types.h"

namespace e2e_noa::planning {

class ProximityCost : public TimeInvariantCost {
 public:
  ProximityCost(float weight,
                const std::pair<Dimension, Dimension>& position_idxs1,
                const std::pair<Dimension, Dimension>& position_idxs2,
                float threshold, const std::string& name = "")
      : TimeInvariantCost(weight, name),
        threshold_(threshold),
        threshold_sq_(threshold * threshold),
        xidx1_(position_idxs1.first),
        yidx1_(position_idxs1.second),
        xidx2_(position_idxs2.first),
        yidx2_(position_idxs2.second) {}

  // Evaluate this cost at the current input.
  float Evaluate(const VectorXf& input) const;

  // Quadraticize this cost at the given input, and add to the running
  // sum of gradients and Hessians.
  void Quadraticize(const VectorXf& input, MatrixXf* hess,
                    VectorXf* grad) const;

 private:
  // Threshold for minimum squared relative distance.
  const float threshold_, threshold_sq_;

  // Position indices for two vehicles.
  const Dimension xidx1_, yidx1_;
  const Dimension xidx2_, yidx2_;
};  //\class ProximityCost

}  // namespace e2e_noa::planning

#endif
