/////////////////////////////////////////////////////////////////////////////
//
// Quadratic cost in a particular (or all) dimension(s).
//
///////////////////////////////////////////////////////////////////////////////

#include "ilq_games/include/cost/quadratic_cost.h"

#include <glog/logging.h>

#include <numeric>

#include "ilq_games/include/utils/ilq_types.h"

namespace e2e_noa::planning {

float QuadraticCost::Evaluate(const VectorXf& input) const {
  CHECK_LT(dimension_, input.size());

  // If dimension non-negative, then just square the desired dimension.
  if (dimension_ >= 0) {
    const float delta = input(dimension_) - nominal_;
    return 0.5 * weight_ * delta * delta;
  }

  // Otherwise, cost is squared 2-norm of entire input.
  return 0.5 * weight_ *
         (input - VectorXf::Constant(input.size(), nominal_)).squaredNorm();
}

void QuadraticCost::Quadraticize(const VectorXf& input, MatrixXf* hess,
                                 VectorXf* grad) const {
  CHECK_LT(dimension_, input.size());
  CHECK_NOTNULL(hess);
  CHECK_NOTNULL(grad);

  // Check dimensions.
  CHECK_EQ(input.size(), hess->rows());
  CHECK_EQ(input.size(), hess->cols());
  CHECK_EQ(input.size(), grad->size());

  // Handle single dimension case first.
  if (dimension_ >= 0) {
    const float delta = input(dimension_) - nominal_;
    const float dx = weight_ * delta;
    const float ddx = weight_;

    (*grad)(dimension_) += dx;
    (*hess)(dimension_, dimension_) += ddx;
  }

  // Handle dimension < 0 case.
  else {
    const VectorXf delta = input - VectorXf::Constant(input.size(), nominal_);

    *grad += weight_ * delta;
    hess->diagonal() =
        hess->diagonal() + VectorXf::Constant(input.size(), weight_);
  }
}

}  // namespace e2e_noa::planning
