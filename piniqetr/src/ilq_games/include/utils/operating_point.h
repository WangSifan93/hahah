/////////////////////////////////////////////////////////////////////////////
//
// Container to store an operating point, i.e. states and controls for each
// player.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef ILQGAMES_UTILS_OPERATING_POINT_H
#define ILQGAMES_UTILS_OPERATING_POINT_H

#include <glog/logging.h>

#include <memory>
#include <vector>

#include "ilq_games/include/utils/ilq_types.h"

namespace e2e_noa::planning {

struct OperatingPoint {
  // Time-indexed list of states.
  std::vector<VectorXf> xs;

  // Time-indexed list of controls for all players, i.e. us[kk] is the list of
  // controls for all players at time index kk.
  std::vector<std::vector<VectorXf>> us;

  // Initial time stamp.
  Time t0;

  // Construct with empty vectors of the right size, and optionally zero out if
  // dynamics is non-null.
  OperatingPoint() : xs(), us(), t0(0.0) {}
  OperatingPoint(size_t num_time_steps, PlayerIndex num_players,
                 Time initial_time);

  template <typename MultiPlayerSystemType>
  OperatingPoint(size_t num_time_steps, Time initial_time,
                 const std::shared_ptr<const MultiPlayerSystemType>& dynamics)
      : OperatingPoint(num_time_steps, dynamics->NumPlayers(), initial_time) {
    CHECK_NOTNULL(dynamics.get());
    for (size_t kk = 0; kk < num_time_steps; kk++) {
      xs[kk] = VectorXf::Zero(dynamics->XDim());
      for (PlayerIndex ii = 0; ii < dynamics->NumPlayers(); ii++)
        us[kk][ii] = VectorXf::Zero(dynamics->UDim(ii));
    }
  }

  // Custom swap function.
  void swap(OperatingPoint& other);
};  // struct OperatingPoint

}  // namespace e2e_noa::planning

#endif
