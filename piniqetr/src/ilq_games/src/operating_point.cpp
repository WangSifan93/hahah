//////////////////////////////////////////////////////////////////////////////
//
// Container to store an operating point, i.e. states and controls for each
// player.
//
///////////////////////////////////////////////////////////////////////////////

#include "ilq_games/include/utils/operating_point.h"

#include <vector>

#include "ilq_games/include/dynamics/multi_player_dynamical_system.h"
#include "ilq_games/include/utils/ilq_types.h"

namespace e2e_noa::planning {

OperatingPoint::OperatingPoint(size_t num_time_steps, PlayerIndex num_players,
                               Time initial_time)
    : xs(num_time_steps), us(num_time_steps), t0(initial_time) {
  for (auto& entry : us) entry.resize(num_players);
}

void OperatingPoint::swap(OperatingPoint& other) {
  xs.swap(other.xs);
  us.swap(other.us);
  std::swap(t0, other.t0);
}

}  // namespace e2e_noa::planning
