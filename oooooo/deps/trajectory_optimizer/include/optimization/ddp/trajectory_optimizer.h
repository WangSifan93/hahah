#ifndef ONBOARD_PLANNER_OPTIMIZATION_DDP_TRAJECTORY_OPTIMIZER_H_
#define ONBOARD_PLANNER_OPTIMIZATION_DDP_TRAJECTORY_OPTIMIZER_H_
#include <cmath>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <ostream>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "absl/cleanup/cleanup.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"
#include "absl/time/clock.h"
#include "absl/time/time.h"
#include "absl/types/span.h"
#include "async/parallel_for.h"
#include "async/thread_pool.h"
#include "optimization/ddp/trajectory_optimizer_input.h"
#include "optimization/ddp/trajectory_optimizer_output.h"
#include "optimizer.pb.h"
#include "spacetime_search/select_nudge_object.h"

namespace e2e_noa {
namespace planning {

absl::StatusOr<TrajectoryOptimizerOutput> OptimizeTrajectory(
    const TrajectoryOptimizerInput& input,
    TrajectoryOptimizerDebugProto* optimizer_debug, bool is_compare_weight,
    WorkerThreadManager* thread_pool);

}
}  

#endif
