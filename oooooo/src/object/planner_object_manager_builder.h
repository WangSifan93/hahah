#ifndef ONBOARD_PLANNER_OBJECT_PLANNER_OBJECT_MANAGER_BUILDER_H_
#define ONBOARD_PLANNER_OBJECT_PLANNER_OBJECT_MANAGER_BUILDER_H_

#include <optional>
#include <utility>
#include <vector>

#include "absl/status/statusor.h"
#include "async/thread_pool.h"
#include "object/object_vector.h"
#include "object/planner_object.h"
#include "object/planner_object_manager.h"
#include "object/trajectory_filter.h"
#include "perception.pb.h"
#include "planner_object.pb.h"
#include "prediction.pb.h"

namespace e2e_noa {
namespace planning {

class PlannerObjectManagerBuilder {
 public:
  PlannerObjectManagerBuilder& SetAlignTime(double time) {
    aligned_time_ = time;
    return *this;
  }

  PlannerObjectManagerBuilder& set_planner_objects(
      ObjectVector<PlannerObject> planner_objects) {
    planner_objects_ = std::move(planner_objects);
    return *this;
  }

  PlannerObjectManagerBuilder& set_filters(
      std::vector<const TrajectoryFilter*> filters) {
    filters_ = std::move(filters);
    return *this;
  }

  absl::StatusOr<PlannerObjectController> Build(
      FilteredTrajectories* filtered_trajs = nullptr,
      WorkerThreadManager* thread_pool = nullptr);

 private:
  double aligned_time_ = -1.0;

  std::vector<const TrajectoryFilter*> filters_;

  ObjectVector<PlannerObject> planner_objects_;
};

ObjectVector<PlannerObject> BuildPlannerObjects(
    const ObjectsProto* perception, const ObjectsPredictionProto* prediction,
    std::optional<double> align_time,
    WorkerThreadManager* thread_pool = nullptr);
}  
}  

#endif
