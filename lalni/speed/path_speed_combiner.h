#ifndef PLANNER_SPEED_PATH_SPEED_COMBINER_H_
#define PLANNER_SPEED_PATH_SPEED_COMBINER_H_

#include <vector>

#include "absl/status/status.h"
#include "plan/discretized_path.h"
#include "speed/speed_vector.h"
#include "trajectory_point.pb.h"

namespace e2e_noa::planning {

absl::Status CombinePathAndSpeed(
    const DiscretizedPath& path_data, bool forward,
    const SpeedVector& speed_data,
    std::vector<ApolloTrajectoryPointProto>* trajectory);

absl::Status ExtendTrajectoryLength(
    const DiscretizedPath& path_data, bool forward,
    const SpeedVector& speed_data,
    std::vector<ApolloTrajectoryPointProto>* trajectory);

}  // namespace e2e_noa::planning

#endif
