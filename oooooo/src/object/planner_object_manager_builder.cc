#include "object/planner_object_manager_builder.h"

#include <algorithm>
#include <utility>

#include "absl/status/status.h"
#include "async/parallel_for.h"
#include "glog/logging.h"
#include "prediction/predicted_trajectory.h"
#include "prediction/prediction.h"
#include "prediction/prediction_util.h"

namespace e2e_noa {
namespace planning {

absl::StatusOr<PlannerObjectController> PlannerObjectManagerBuilder::Build(
    FilteredTrajectories* filtered_trajs, WorkerThreadManager* thread_pool) {
  VLOG(0) << __FUNCTION__ << " start filtering trajectories";
  for (auto& object : planner_objects_) {
    auto& trajs = *object.mutable_prediction()->mutable_trajectories();
    int i = 0;
    for (int j = 0, n = trajs.size(); j < n; ++j) {
      auto& traj = trajs[j];
      bool is_traj_filtered = false;
      for (const auto* filter : filters_) {
        const auto reason = filter->Filter(object, traj);
        if (reason != FilterReason::NONE) {
          VLOG(0) << "object id: " << object.id() << " filter reason" << reason;
          if (filtered_trajs != nullptr) {
            auto* filtered = filtered_trajs->add_filtered();
            filtered->set_reason(reason);
            filtered->set_id(object.id());
            filtered->set_index(traj.index());
          }
          is_traj_filtered = true;
          break;
        }
      }
      if (!is_traj_filtered) {
        if (i != j) {
          trajs[i] = std::move(traj);
        }
        ++i;
      }
    }
    trajs.erase(trajs.begin() + i, trajs.end());
  }

  planner_objects_.erase(
      std::remove_if(planner_objects_.begin(), planner_objects_.end(),
                     [](const auto& obj) { return obj.num_trajs() == 0; }),
      planner_objects_.end());

  return PlannerObjectController(std::move(planner_objects_));
}

ObjectVector<PlannerObject> BuildPlannerObjects(
    const ObjectsProto* perception, const ObjectsPredictionProto* prediction,
    std::optional<double> align_time, WorkerThreadManager* thread_pool) {
  struct ObjectInfo {
    std::string_view id;
    const ObjectProto* obj_ptr = nullptr;
    const ObjectPredictionProto* pred_ptr = nullptr;
  };
  absl::flat_hash_map<std::string_view, int> id_to_index;
  std::vector<ObjectInfo> obj_info;
  obj_info.reserve((perception == nullptr ? 0 : perception->objects_size()) +
                   (prediction == nullptr ? 0 : prediction->objects_size()));
  if (perception != nullptr) {
    for (const auto& obj : perception->objects()) {
      id_to_index[obj.id()] = obj_info.size();
      obj_info.push_back(
          {.id = obj.id(), .obj_ptr = &obj, .pred_ptr = nullptr});
    }
  }
  if (prediction != nullptr) {
    for (const auto& pred : prediction->objects()) {
      const auto& id = pred.perception_object().id();
      if (const auto* index = FindOrNull(id_to_index, id); index != nullptr) {
        obj_info[*index].pred_ptr = &pred;
      } else {
        obj_info.push_back({.id = id, .obj_ptr = nullptr, .pred_ptr = &pred});
      }
    }
  }
  ObjectVector<PlannerObject> planner_objects;
  planner_objects.resize(obj_info.size());
  ParallelFor(0, obj_info.size(), thread_pool, [&](int i) {
    CHECK(obj_info[i].pred_ptr != nullptr)
        << "obj_info[" << i << "] id = " << obj_info[i].id
        << " does not have prediction."
        << "\t pred_ptr = " << obj_info[i].pred_ptr;

    const ObjectProto* latest_obj = nullptr;
    if (obj_info[i].obj_ptr != nullptr &&
        obj_info[i].obj_ptr->timestamp() >
            obj_info[i].pred_ptr->perception_object().timestamp()) {
      latest_obj = obj_info[i].obj_ptr;
    } else if (obj_info[i].pred_ptr->has_perception_object()) {
      latest_obj = &obj_info[i].pred_ptr->perception_object();
    }
    if (latest_obj != nullptr) {
      const double time_shift =
          align_time.has_value() ? *align_time - latest_obj->timestamp() : 0.0;
      planner_objects[ObjectIndex(i)] =
          PlannerObject(prediction::ObjectPrediction(*obj_info[i].pred_ptr,
                                                     time_shift, *latest_obj));
    }
  });

  planner_objects.erase(
      std::remove_if(planner_objects.begin(), planner_objects.end(),
                     [](const PlannerObject& obj) {
                       return obj.num_trajs() == 0 || obj.type() == OT_CONE ||
                              obj.type() == OT_BARRIER;
                     }),
      planner_objects.end());
  return planner_objects;
}

}  
}  
