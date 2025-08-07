#include "object/spacetime_trajectory_manager_builder.h"

#include <vector>

#include "absl/types/span.h"
#include "common/timer.h"
#include "object/low_likelihood_filter.h"
#include "object/plan_passage_filter.h"
#include "object/trajectory_filter.h"
#include "plan/planner_flags.h"

namespace e2e_noa {
namespace planning {

SpacetimeTrajectoryManager BuildSpacetimeTrajectoryController(
    const SpacetimeTrajectoryManagerBuilderInput& input,
    WorkerThreadManager* thread_pool) {
  Timer timer(__FUNCTION__);

  CHECK_NOTNULL(input.passage);
  CHECK_NOTNULL(input.sl_boundary);
  CHECK_NOTNULL(input.obj_mgr);

  const LowLikelihoodFilter low_likelihood_filter(
      FLAGS_planner_prediction_probability_threshold,
      FLAGS_planner_only_use_most_likely_trajectory);
  const PlanPassageFilter plan_passage_filter(input.passage, input.sl_boundary);
  std::vector<const TrajectoryFilter*> filters;
  filters.push_back(&low_likelihood_filter);
  filters.push_back(&plan_passage_filter);
  return SpacetimeTrajectoryManager(filters, input.obj_mgr->planner_objects(),
                                    thread_pool);
}

}  
}  
