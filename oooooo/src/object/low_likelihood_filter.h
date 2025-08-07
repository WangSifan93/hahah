#ifndef ONBOARD_PLANNER_OBJECT_LOW_LIKELIHOOD_FILTER_H_
#define ONBOARD_PLANNER_OBJECT_LOW_LIKELIHOOD_FILTER_H_

#include "object/planner_object.h"
#include "object/trajectory_filter.h"
#include "planner_object.pb.h"
#include "prediction/predicted_trajectory.h"

namespace e2e_noa {
namespace planning {

class LowLikelihoodFilter : public TrajectoryFilter {
 public:
  explicit LowLikelihoodFilter(double min_prob, bool only_use_most_likely_traj)
      : min_prob_(min_prob),
        only_use_most_likely_traj_(only_use_most_likely_traj) {}

  FilterReason::Type Filter(
      const PlannerObject& object,
      const prediction::PredictedTrajectory& traj) const override;

 private:
  double min_prob_;
  bool only_use_most_likely_traj_;
};

}  // namespace planning
}  // namespace e2e_noa

#endif
