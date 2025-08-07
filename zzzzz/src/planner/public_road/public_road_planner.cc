#include "apps/planning/src/planner/public_road/public_road_planner.h"

#include "apps/planning/src/config/conf_gflags.h"

namespace zark {
namespace planning {

using ::common::Status;
using ::common::TrajectoryPoint;

Status PublicRoadPlanner::Init(const PlanningConfig& config) {
  AINFO << "Init PublicRoadPlanner.............";
  config_ = config;
  return Status::OK();
}

Status PublicRoadPlanner::Plan(const TrajectoryPoint& planning_start_point,
                               Frame* frame, scenario::Scenario* scenario,
                               ADCTrajectory& computed_trajectory) {
  AINFO << "start PublicRoadPlanner.............";

  auto result = scenario->Process(planning_start_point, frame);

  if (result == scenario::Scenario::STATUS_DONE) {
    return Status::OK();
  } else if (result == scenario::Scenario::STATUS_UNKNOWN) {
    return Status(::common::PLANNING_ERROR, "scenario returned unknown");
  } else {
    return Status::OK();
  }
}

}  // namespace planning
}  // namespace zark
