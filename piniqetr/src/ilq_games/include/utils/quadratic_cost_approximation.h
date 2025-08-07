//////////////////////////////////////////////////////////////////////////////
//
// Container to store a quadratic approximation of a single player's cost at a
// particular moment in time. That is, each player should have a time-indexed
// set of these QuadraticApproximations.
//
// Notation is taken from Basar and Olsder, Corollary 6.1.
// -- Q is the Hessian with respect to state
// -- l is the gradient with respect to state
// -- Rs[ii] is the Hessian with respect to the control input of player ii
// -- rs[ii] is the gradient with respect to the control input of player ii
//
///////////////////////////////////////////////////////////////////////////////

#ifndef ILQGAMES_UTILS_QUADRATIC_COST_APPROXIMATION_H
#define ILQGAMES_UTILS_QUADRATIC_COST_APPROXIMATION_H

#include <glog/logging.h>

#include <unordered_map>

#include "ilq_games/include/utils/ilq_types.h"

namespace e2e_noa::planning {

struct SingleCostApproximation {
  MatrixXf hess;
  VectorXf grad;

  // Construct from matrix/vector directly.
  SingleCostApproximation(const MatrixXf& hessian, const VectorXf& gradient)
      : hess(hessian), grad(gradient) {
    CHECK_EQ(hess.rows(), hess.cols());
    CHECK_EQ(hess.rows(), grad.size());
  }

  // Construct with zeros.
  SingleCostApproximation(Dimension dim, float regularization = 0.0)
      : hess(regularization * MatrixXf::Identity(dim, dim)),
        grad(VectorXf::Zero(dim)) {}
};  // struct SingleCostApproximation

struct QuadraticCostApproximation {
  SingleCostApproximation state;
  PlayerMap<SingleCostApproximation> control;

  // Construct from state dimension.
  explicit QuadraticCostApproximation(Dimension xdim,
                                      float regularization = 0.0)
      : state(xdim, regularization) {}
};  // struct QuadraticCostApproximation

}  // namespace e2e_noa::planning

#endif
