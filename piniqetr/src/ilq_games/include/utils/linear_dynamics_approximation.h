/////////////////////////////////////////////////////////////////////////////
//
// Container to store a linear approximation of the dynamics at a particular
// time.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef ILQGAMES_UTILS_LINEAR_DYNAMICS_APPROXIMATION_H
#define ILQGAMES_UTILS_LINEAR_DYNAMICS_APPROXIMATION_H

#include <vector>

#include "ilq_games/include/utils/ilq_types.h"

namespace e2e_noa::planning {

struct LinearDynamicsApproximation {
  MatrixXf A;
  std::vector<MatrixXf> Bs;

  // Default constructor.
  LinearDynamicsApproximation() {}

  // Construct from a MultiPlayerDynamicalSystem. Templated to avoid include
  // cycle. Initialize A to identity and Bs to zero (since this is for a
  // discrete-time linearization).
  template <typename MultiPlayerSystemType>
  explicit LinearDynamicsApproximation(const MultiPlayerSystemType& system)
      : A(MatrixXf::Identity(system.XDim(), system.XDim())),
        Bs(system.NumPlayers()) {
    for (size_t ii = 0; ii < system.NumPlayers(); ii++)
      Bs[ii] = MatrixXf::Zero(system.XDim(), system.UDim(ii));
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};  // struct LinearDynamicsApproximation

}  // namespace e2e_noa::planning

#endif
