///////////////////////////////////////////////////////////////////////////////
//
// Semiquadratic cost on distance from a polyline.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef ILQGAMES_COST_SEMIQUADRATIC_POLYLINE2_COST_H
#define ILQGAMES_COST_SEMIQUADRATIC_POLYLINE2_COST_H

#include <string>
#include <tuple>

#include "ilq_games/include/cost/time_invariant_cost.h"
#include "ilq_games/include/geometry/polyline2.h"
#include "ilq_games/include/utils/ilq_types.h"

namespace e2e_noa::planning {

class SemiquadraticPolyline2Cost : public TimeInvariantCost {
 public:
  // Construct from a multiplicative weight, the input dimensions corresponding
  // to (x, y)-position, a threshold, and a flag for which side to apply it.
  SemiquadraticPolyline2Cost(
      float weight, const Polyline2& polyline,
      const std::pair<Dimension, Dimension>& position_idxs, float threshold,
      bool oriented_right, const std::string& name = "")
      : TimeInvariantCost(weight, name),
        polyline_(polyline),
        xidx_(position_idxs.first),
        yidx_(position_idxs.second),
        threshold_(threshold),
        signed_squared_threshold_(sgn(threshold) * threshold * threshold),
        oriented_right_(oriented_right) {}

  // Evaluate this cost at the current input.
  float Evaluate(const VectorXf& input) const;

  // Quadraticize this cost at the given input, and add to the running
  // sum of gradients and Hessians.
  void Quadraticize(const VectorXf& input, MatrixXf* hess,
                    VectorXf* grad) const;

 private:
  // Check if cost is active.
  bool IsActive(float signed_squared_distance) const {
    return (signed_squared_distance > signed_squared_threshold_ &&
            oriented_right_) ||
           (signed_squared_distance < signed_squared_threshold_ &&
            !oriented_right_);
  }

  // Polyline to compute distances from.
  const Polyline2 polyline_;

  // Dimensions of input corresponding to (x, y)-position.
  const Dimension xidx_;
  const Dimension yidx_;

  // Threshold and orientation.
  const float threshold_;
  const float signed_squared_threshold_;
  const bool oriented_right_;
};  //\class QuadraticPolyline2Cost

}  // namespace e2e_noa::planning

#endif
