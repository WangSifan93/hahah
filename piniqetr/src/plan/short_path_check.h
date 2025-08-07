#ifndef ONBOARD_PLANNER_MIN_LENGTH_PATH_EXTENSION_H_
#define ONBOARD_PLANNER_MIN_LENGTH_PATH_EXTENSION_H_

#include <vector>

#include "absl/status/statusor.h"
#include "absl/types/span.h"
#include "trajectory_point.pb.h"

namespace e2e_noa {
namespace planning {

absl::StatusOr<std::vector<PathPoint>> ExtendPathAndDeleteUnreasonablePart(
    absl::Span<const ApolloTrajectoryPointProto> trajectory_points,
    double min_length, double max_curvature);

}
}  // namespace e2e_noa

#endif
