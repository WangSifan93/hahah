
///////////////////////////////////////////////////////////////////////////////
//
// Quadratic cost in a particular (or all) dimension(s).
//
///////////////////////////////////////////////////////////////////////////////

#include "ilq_games/include/cost/semiquadratic_cost.h"

#include <glog/logging.h>

#include "ilq_games/include/utils/ilq_types.h"

namespace e2e_noa::planning {

// Evaluate this cost at the current input.
float SemiquadraticCost::Evaluate(const VectorXf& input) const {
  CHECK_LT(dimension_, input.size());

  const float diff = input(dimension_) - threshold_;
  if ((diff > 0.0 && oriented_right_) || (diff < 0.0 && !oriented_right_))
    return 0.5 * weight_ * diff * diff;

  return 0.0;
}

// Quadraticize this cost at the given input, and add to the running
// sum of gradients and Hessians (if non-null).
void SemiquadraticCost::Quadraticize(const VectorXf& input, MatrixXf* hess,
                                     VectorXf* grad) const {
  CHECK_LT(dimension_, input.size());

  // Handle no cost case first.
  const float diff = input(dimension_) - threshold_;
  if ((diff < 0.0 && oriented_right_) || (diff > 0.0 && !oriented_right_))
    return;

  // Check dimensions.
  CHECK_NOTNULL(hess);
  CHECK_NOTNULL(grad);
  CHECK_EQ(input.size(), hess->rows());
  CHECK_EQ(input.size(), hess->cols());
  CHECK_EQ(input.size(), grad->size());

  // Compute gradient and Hessian.
  const float dx = weight_ * diff;
  const float ddx = weight_;

  (*grad)(dimension_) += dx;
  (*hess)(dimension_, dimension_) += ddx;
}

}  // namespace e2e_noa::planning
