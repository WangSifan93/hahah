///////////////////////////////////////////////////////////////////////////////
//
// Multi-player dynamical system comprised of several single player subsystems.
//
///////////////////////////////////////////////////////////////////////////////

#include "ilq_games/include/dynamics/concatenated_dynamical_system.h"

#include "ilq_games/include/utils/linear_dynamics_approximation.h"
// #include ".DS_Store"

#include <glog/logging.h>

namespace e2e_noa::planning {

ConcatenatedDynamicalSystem::ConcatenatedDynamicalSystem(
    const SubsystemList& subsystems)
    : MultiPlayerDynamicalSystem(std::accumulate(
          subsystems.begin(), subsystems.end(), 0,
          [](Dimension total,
             const std::shared_ptr<SinglePlayerDynamicalSystem>& subsystem) {
            CHECK_NOTNULL(subsystem.get());
            return total + subsystem->XDim();
          })),
      subsystems_(subsystems) {
  // Populate subsystem start dimensions.
  subsystem_start_dims_.push_back(0);
  for (const auto& subsystem : subsystems_) {
    subsystem_start_dims_.push_back(subsystem_start_dims_.back() +
                                    subsystem->XDim());
  }
}

VectorXf ConcatenatedDynamicalSystem::Evaluate(
    Time t, const VectorXf& x, const std::vector<VectorXf>& us) const {
  CHECK_EQ(us.size(), NumPlayers());

  // Populate 'xdot' one subsystem at a time.
  VectorXf xdot(xdim_);
  Dimension dims_so_far = 0;
  for (size_t ii = 0; ii < NumPlayers(); ii++) {
    const auto& subsystem = subsystems_[ii];
    xdot.segment(dims_so_far, subsystem->XDim()) = subsystem->Evaluate(
        t, x.segment(dims_so_far, subsystem->XDim()), us[ii]);
    dims_so_far += subsystem->XDim();
  }

  return xdot;
}

LinearDynamicsApproximation ConcatenatedDynamicalSystem::Linearize(
    Time t, const VectorXf& x, const std::vector<VectorXf>& us) const {
  CHECK_EQ(us.size(), NumPlayers());

  // Populate a block-diagonal A, as well as Bs.
  LinearDynamicsApproximation linearization(*this);

  Dimension dims_so_far = 0;
  for (size_t ii = 0; ii < NumPlayers(); ii++) {
    const auto& subsystem = subsystems_[ii];
    const Dimension xdim = subsystem->XDim();
    const Dimension udim = subsystem->UDim();
    subsystem->Linearize(
        t, x.segment(dims_so_far, xdim), us[ii],
        linearization.A.block(dims_so_far, dims_so_far, xdim, xdim),
        linearization.Bs[ii].block(dims_so_far, 0, xdim, udim));

    dims_so_far += subsystem->XDim();
  }

  return linearization;
}

float ConcatenatedDynamicalSystem::DistanceBetween(const VectorXf& x0,
                                                   const VectorXf& x1) const {
  // HACK: assumes only first subsystem matters.
  return subsystems_[0]->DistanceBetween(x0.head(subsystems_[0]->XDim()),
                                         x1.head(subsystems_[0]->XDim()));
}

std::vector<Dimension> ConcatenatedDynamicalSystem::PositionDimensions() const {
  std::vector<Dimension> dims;

  for (const auto& s : subsystems_) {
    const std::vector<Dimension> sub_dims = s->PositionDimensions();
    dims.insert(dims.end(), sub_dims.begin(), sub_dims.end());
  }

  return dims;
}

}  // namespace e2e_noa::planning
