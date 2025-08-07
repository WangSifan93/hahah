/////////////////////////////////////////////////////////////////////////////
//
// Signed distance from a given polyline.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef ILQGAMES_COST_POLYLINE2_SIGNED_DISTANCE_COST_H
#define ILQGAMES_COST_POLYLINE2_SIGNED_DISTANCE_COST_H

#include <string>
#include <tuple>

#include "ilq_games/include/cost/time_invariant_cost.h"
#include "ilq_games/include/geometry/polyline2.h"
#include "ilq_games/include/utils/ilq_types.h"

namespace e2e_noa::planning {

class Polyline2SignedDistanceCost : public TimeInvariantCost {
 public:
  // Construct from a multiplicative weight and the input dimensions
  // corresponding to (x, y)-position.
  Polyline2SignedDistanceCost(
      const Polyline2& polyline,
      const std::pair<Dimension, Dimension>& position_idxs,
      const float nominal = 0.0, bool oriented_same_as_polyline = true,
      const std::string& name = "")
      : TimeInvariantCost(1.0, name),
        polyline_(polyline),
        xidx_(position_idxs.first),
        yidx_(position_idxs.second),
        nominal_(nominal),
        oriented_same_as_polyline_(oriented_same_as_polyline) {}

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

  // Nominal value.
  const float nominal_;

  // Whether the orientation is the same or opposite that of the polyline.
  const bool oriented_same_as_polyline_;
};  //\class Polyline2SignedDistanceCost

}  // namespace e2e_noa::planning

#endif
