#ifndef ONBOARD_PLANNER_SPACETIME_TRAJECTORY_MANAGER_BUILDER_H_
#define ONBOARD_PLANNER_SPACETIME_TRAJECTORY_MANAGER_BUILDER_H_

#include "async/thread_pool.h"
#include "common/path_sl_boundary.h"
#include "object/planner_object_manager.h"
#include "object/spacetime_trajectory_manager.h"
#include "router/plan_passage.h"

namespace e2e_noa {
namespace planning {
struct SpacetimeTrajectoryManagerBuilderInput {
  const PlanPassage* passage;
  const PathSlBoundary* sl_boundary;
  const PlannerObjectController* obj_mgr;
};

SpacetimeTrajectoryManager BuildSpacetimeTrajectoryController(
    const SpacetimeTrajectoryManagerBuilderInput& input,
    WorkerThreadManager* thread_pool);

}  
}  
#endif
