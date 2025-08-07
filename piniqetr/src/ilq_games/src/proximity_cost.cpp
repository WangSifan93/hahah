/////////////////////////////////////////////////////////////////////////////
//
// Penalizes (thresh - relative distance)^2 between two pairs of state
// dimensions (representing two positions of vehicles whose states have been
// concatenated) whenever relative distance is less than thresh.
//
///////////////////////////////////////////////////////////////////////////////

#include "ilq_games/include/cost/proximity_cost.h"

#include <glog/logging.h>

#include "ilq_games/include/utils/ilq_types.h"

namespace e2e_noa::planning {

float ProximityCost::Evaluate(const VectorXf& input) const {
  const float dx = input(xidx1_) - input(xidx2_);
  const float dy = input(yidx1_) - input(yidx2_);
  const float delta_sq = dx * dx + dy * dy;

  if (delta_sq >= threshold_sq_) return 0.0;

  const float gap = threshold_ - std::sqrt(delta_sq);
  return 0.5 * weight_ * gap * gap;
}

void ProximityCost::Quadraticize(const VectorXf& input, MatrixXf* hess,
                                 VectorXf* grad) const {
  CHECK_NOTNULL(hess);
  CHECK_NOTNULL(grad);

  // Check dimensions.
  CHECK_EQ(input.size(), hess->rows());
  CHECK_EQ(input.size(), hess->cols());
  CHECK_EQ(input.size(), grad->size());

  // Compute Hessian and gradient.
  const float dx = input(xidx1_) - input(xidx2_);
  const float dy = input(yidx1_) - input(yidx2_);
  const float delta_sq = dx * dx + dy * dy;

  // Catch cost not active.
  if (delta_sq >= threshold_sq_) return;

  const float delta = std::sqrt(delta_sq);
  const float gap = threshold_ - delta;
  const float weight_delta = weight_ / delta;
  const float dx_delta = dx / delta;
  const float dy_delta = dy / delta;

  const float ddx1 = -weight_delta * gap * dx;
  const float ddy1 = -weight_delta * gap * dy;
  const float hess_x1x1 =
      weight_delta * (dx_delta * (gap * dx_delta + dx) - gap);
  const float hess_y1y1 =
      weight_delta * (dy_delta * (gap * dy_delta + dy) - gap);
  const float hess_x1y1 = weight_delta * (dx_delta * (gap * dy_delta + dy));

  (*grad)(xidx1_) += ddx1;
  (*grad)(xidx2_) -= ddx1;

  (*grad)(yidx1_) += ddy1;
  (*grad)(yidx2_) -= ddy1;

  (*hess)(xidx1_, xidx1_) += hess_x1x1;
  (*hess)(xidx1_, xidx2_) -= hess_x1x1;
  (*hess)(xidx2_, xidx1_) -= hess_x1x1;
  (*hess)(xidx2_, xidx2_) += hess_x1x1;

  (*hess)(yidx1_, yidx1_) += hess_y1y1;
  (*hess)(yidx1_, yidx2_) -= hess_y1y1;
  (*hess)(yidx2_, yidx1_) -= hess_y1y1;
  (*hess)(yidx2_, yidx2_) += hess_y1y1;

  (*hess)(xidx1_, yidx1_) += hess_x1y1;
  (*hess)(yidx1_, xidx1_) += hess_x1y1;

  (*hess)(xidx1_, yidx2_) -= hess_x1y1;
  (*hess)(yidx2_, xidx1_) -= hess_x1y1;

  (*hess)(xidx2_, yidx1_) -= hess_x1y1;
  (*hess)(yidx1_, xidx2_) -= hess_x1y1;

  (*hess)(xidx2_, yidx2_) += hess_x1y1;
  (*hess)(yidx2_, xidx2_) += hess_x1y1;
}

}  // namespace e2e_noa::planning
