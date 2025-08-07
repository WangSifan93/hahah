//////////////////////////////////////////////////////////////////////////////
//
// Container to store a single player's time-indexed strategy.
//
// Notation is taken from Basar and Olsder, Corollary 6.1.
// -- alphas are the feedforward terms
// -- Ps are the feedback gains
// i.e. delta u[ii] = -P[ii] delta x - alphas[ii]
//
///////////////////////////////////////////////////////////////////////////////

#ifndef ILQGAMES_UTILS_STRATEGY_H
#define ILQGAMES_UTILS_STRATEGY_H

#include <glog/logging.h>

#include <vector>

#include "ilq_games/include/utils/ilq_types.h"
#include "ilq_games/include/utils/operating_point.h"

namespace e2e_noa::planning {

struct Strategy {
  std::vector<MatrixXf> Ps;
  std::vector<VectorXf> alphas;

  // Preallocate memory during construction.
  Strategy(size_t horizon, Dimension xdim, Dimension udim)
      : Ps(horizon), alphas(horizon) {
    for (size_t ii = 0; ii < horizon; ii++) {
      Ps[ii] = MatrixXf::Zero(udim, xdim);
      alphas[ii] = VectorXf::Zero(udim);
    }
  }

  // Operator for computing control given time index and delta x.
  VectorXf operator()(size_t time_index, const VectorXf& delta_x,
                      const VectorXf& u_ref) const {
    return u_ref - Ps[time_index] * delta_x - alphas[time_index];
  }

  // Number of variables.
  size_t NumVariables() const {
    const size_t horizon = Ps.size();
    CHECK_EQ(horizon, alphas.size());

    return horizon * (Ps.front().size() + alphas.front().size());
  }
};  // struct Strategy

}  // namespace e2e_noa::planning

#endif
