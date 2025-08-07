/////////////////////////////////////////////////////////////////////////////
//
// Quadratic penalty on distance from a given Polyline2.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef ILQGAMES_COST_QUADRATIC_POLYLINE2_COST_H
#define ILQGAMES_COST_QUADRATIC_POLYLINE2_COST_H

#include <string>
#include <tuple>

#include "ilq_games/include/cost/time_invariant_cost.h"
#include "ilq_games/include/geometry/polyline2.h"
#include "ilq_games/include/utils/ilq_types.h"

namespace e2e_noa::planning {

class QuadraticPolyline2Cost : public TimeInvariantCost {
 public:
  // Construct from a multiplicative weight and the input dimensions
  // corresponding to (x, y)-position.
  QuadraticPolyline2Cost(float weight, const Polyline2& polyline,
                         const std::pair<Dimension, Dimension>& position_idxs,
                         const std::string& name = "")
      : TimeInvariantCost(weight, name),
        polyline_(polyline),
        xidx_(position_idxs.first),
        yidx_(position_idxs.second) {}

  // Evaluate this cost at the current input.
  float Evaluate(const VectorXf& input) const;

  // Quadraticize this cost at the given input, and add to the running
  // sum of gradients and Hessians.
  void Quadraticize(const VectorXf& input, MatrixXf* hess,
                    VectorXf* grad) const;

 private:
  // Polyline to compute distances from.
  const Polyline2 polyline_;

  // Dimensions of input corresponding to (x, y)-position.
  const Dimension xidx_;
  const Dimension yidx_;
};  //\class QuadraticPolyline2Cost

}  // namespace e2e_noa::planning

#endif
