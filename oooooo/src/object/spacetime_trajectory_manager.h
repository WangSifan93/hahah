#ifndef ONBOARD_PLANNER_OBJECT_SPACETIME_TRAJECTORY_MANAGER_H_
#define ONBOARD_PLANNER_OBJECT_SPACETIME_TRAJECTORY_MANAGER_H_

#include <string>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/strings/string_view.h"
#include "absl/types/span.h"
#include "async/thread_pool.h"
#include "object/planner_object.h"
#include "object/spacetime_object_trajectory.h"
#include "planner_object.pb.h"
#include "prediction/predicted_trajectory.h"
#include "util/map_util.h"

namespace e2e_noa {

namespace planning {

class TrajectoryFilter;

class SpacetimeTrajectoryManager {
 public:
  SpacetimeTrajectoryManager() {}

  SpacetimeTrajectoryManager(absl::Span<const TrajectoryFilter* const> filters,
                             absl::Span<const PlannerObject> planner_objects,
                             WorkerThreadManager* thread_pool);
  explicit SpacetimeTrajectoryManager(
      absl::Span<const PlannerObject> planner_objects,
      WorkerThreadManager* thread_pool = nullptr)
      : SpacetimeTrajectoryManager({}, planner_objects, thread_pool) {}

  explicit SpacetimeTrajectoryManager(
      absl::Span<SpacetimeObjectTrajectory> spacetime_trajectories);

  SpacetimeTrajectoryManager(const SpacetimeTrajectoryManager& other);
  SpacetimeTrajectoryManager& operator=(
      const SpacetimeTrajectoryManager& other);

  SpacetimeTrajectoryManager(SpacetimeTrajectoryManager&& other) = default;
  SpacetimeTrajectoryManager& operator=(SpacetimeTrajectoryManager&& other) =
      default;

  absl::Span<const SpacetimeObjectTrajectory* const> stationary_object_trajs()
      const {
    return considered_stationary_trajs_;
  }

  absl::Span<const SpacetimeObjectTrajectory* const> moving_object_trajs()
      const {
    return considered_moving_trajs_;
  }

  absl::Span<const SpacetimeObjectTrajectory> trajectories() const {
    return considered_trajs_;
  }

  std::vector<SpacetimeObjectTrajectory>* mutable_trajectories() {
    return &considered_trajs_;
  }

  const absl::flat_hash_map<std::string_view,
                            std::vector<const SpacetimeObjectTrajectory*>>&
  object_trajectories_map() const {
    return objects_id_map_;
  }

  absl::Span<const SpacetimeObjectTrajectory* const> FindTrajectoriesByObjectId(
      std::string_view id) const {
    const auto iter = objects_id_map_.find(id);
    if (iter == objects_id_map_.end()) return {};
    return iter->second;
  }

  const PlannerObject* FindObjectByObjectId(std::string_view id) const {
    const auto iter = objects_id_map_.find(id);
    if (iter == objects_id_map_.end()) return nullptr;
    return &(iter->second.front()->planner_object());
  }

  const SpacetimeObjectTrajectory* FindTrajectoryById(
      std::string_view traj_id) const {
    return FindPtrOrNull(trajectories_id_map_, traj_id);
  }

  struct IgnoredTrajectory {
    const prediction::PredictedTrajectory* traj;
    std::string object_id;
    FilterReason::Type reason;
  };
  struct StationaryObject {
    std::string object_id;
    PlannerObject planner_object;
  };

  absl::Span<const IgnoredTrajectory> ignored_trajectories() const {
    return ignored_trajs_;
  }

  absl::Span<const StationaryObject> stationary_objects() const {
    return stationary_objs_;
  }

  void UpdatePointers(int stationary_size);

 protected:
  absl::flat_hash_map<std::string_view,
                      std::vector<const SpacetimeObjectTrajectory*>>
      objects_id_map_;
  absl::flat_hash_map<std::string_view, const SpacetimeObjectTrajectory*>
      trajectories_id_map_;

  std::vector<SpacetimeObjectTrajectory> considered_trajs_;
  std::vector<IgnoredTrajectory> ignored_trajs_;
  std::vector<StationaryObject> stationary_objs_;
  std::vector<const SpacetimeObjectTrajectory*> considered_stationary_trajs_;
  std::vector<const SpacetimeObjectTrajectory*> considered_moving_trajs_;
};

}  // namespace planning
}  // namespace e2e_noa

#endif
