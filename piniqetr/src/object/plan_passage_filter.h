#ifndef ONBOARD_PLANNER_OBJECT_PLAN_PASSAGE_FILTER_H_
#define ONBOARD_PLANNER_OBJECT_PLAN_PASSAGE_FILTER_H_

#include "common/path_sl_boundary.h"
#include "object/planner_object.h"
#include "object/trajectory_filter.h"
#include "planner_object.pb.h"
#include "prediction/predicted_trajectory.h"
#include "router/plan_passage.h"

namespace e2e_noa {
namespace planning {

class PlanPassageFilter : public TrajectoryFilter {
 public:
  explicit PlanPassageFilter(const PlanPassage* plan_passage,
                             const PathSlBoundary* sl_boundary)
      : plan_passage_(CHECK_NOTNULL(plan_passage)),
        sl_boundary_(CHECK_NOTNULL(sl_boundary)) {}

  FilterReason::Type Filter(
      const PlannerObject& object,
      const prediction::PredictedTrajectory& traj) const override;

 private:
  const PlanPassage* plan_passage_;
  const PathSlBoundary* sl_boundary_;
};

}  // namespace planning
}  // namespace e2e_noa

#endif
