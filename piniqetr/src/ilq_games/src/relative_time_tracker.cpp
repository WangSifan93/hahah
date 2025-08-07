//////////////////////////////////////////////////////////////////////////////
//
// Base class for all named objects which depend upon the initial time. Examples
// of derived classes are Cost and EqualityConstraint.
//
///////////////////////////////////////////////////////////////////////////////

#include "ilq_games/include/utils/relative_time_tracker.h"

namespace e2e_noa::planning {

Time RelativeTimeTracker::initial_time_ = 0.0;  // s

}  // namespace e2e_noa::planning
