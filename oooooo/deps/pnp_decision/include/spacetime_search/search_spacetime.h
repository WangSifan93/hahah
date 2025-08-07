#ifndef SEARCH_MOTION_H_
#define SEARCH_MOTION_H_

#include <string>

#include "absl/container/flat_hash_set.h"
#include "absl/status/statusor.h"
#include "async/thread_pool.h"
#include "descriptor/descriptor_output.h"
#include "decision_exploration/decision_exploration_output.h"
#include "trajectory_initialization.pb.h"
#include "spacetime_search/spacetime_search_input.h"
#include "spacetime_search/spacetime_search_output.h"

namespace e2e_noa::planning {

absl::StatusOr<SpacetimeSearchOutput> SearchSpacetime(
    const SpacetimeSearchInput& spacetime_input,
    WorkerThreadManager* thread_pool);

absl::StatusOr<TrajectoryInitializationOutput> RunTrajectoryInitialization(
    const InitializationInput& initialization_input,
    absl::flat_hash_set<std::string>* unsafe_object_ids,
    DecisionExplorationOutput* decision_exploration_output,
    Descriptor* decision_output, InitializationDebugProto* debug_proto,
    WorkerThreadManager* thread_pool, std::map<std::string, bool>* obj_lead,
    e2e_noa::planning::PlannerStatusProto::PlannerStatusCode* lc_status_code);

}  

#endif
