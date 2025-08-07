#ifndef ONBOARD_PLANNER_OBJECT_SPACETIME_PLANNER_OBJECT_TRAJECTORIES_FINDER_H_
#define ONBOARD_PLANNER_OBJECT_SPACETIME_PLANNER_OBJECT_TRAJECTORIES_FINDER_H_

#include <optional>
#include <string>
#include <vector>

#include "absl/container/flat_hash_set.h"
#include "common/path_sl_boundary.h"
#include "maps/lane_path.h"
#include "math/frenet_common.h"
#include "math/geometry/box2d.h"
#include "math/vec.h"
#include "object/spacetime_object_trajectory.h"
#include "plan/planner_semantic_map_manager.h"
#include "planner_object.pb.h"
#include "router/plan_passage.h"
#include "trajectory_point.pb.h"
#include "util/hmi_content_util.h"
#include "vehicle.pb.h"

namespace e2e_noa {
namespace planning {

class SpacetimePlannerObjectTrajectoriesFinder {
 public:
  virtual SpacetimePlannerObjectTrajectoryReason::Type Find(
      const SpacetimeObjectTrajectory& traj) const = 0;
  virtual ~SpacetimePlannerObjectTrajectoriesFinder() = default;
};

class AllSpacetimePlannerObjectTrajectoriesLocator
    : public SpacetimePlannerObjectTrajectoriesFinder {
 public:
  AllSpacetimePlannerObjectTrajectoriesLocator() = default;
  SpacetimePlannerObjectTrajectoryReason::Type Find(
      const SpacetimeObjectTrajectory& traj) const override {
    return SpacetimePlannerObjectTrajectoryReason::ALL;
  };
};

class StationarySpacetimePlannerObjectTrajectoriesFinder
    : public SpacetimePlannerObjectTrajectoriesFinder {
 public:
  StationarySpacetimePlannerObjectTrajectoriesFinder(
      const PlannerSemanticMapManager* psmm,
      const mapping::LanePath& lane_path);
  SpacetimePlannerObjectTrajectoryReason::Type Find(
      const SpacetimeObjectTrajectory& traj) const override;

  std::optional<Box2d> bool_barrier_box_or_;
};

class FrontSideMovingSpacetimePlannerObjectTrajectoriesFinder
    : public SpacetimePlannerObjectTrajectoriesFinder {
 public:
  static constexpr double kComfortableNudgeCheckTime = 3.0;
  static constexpr double kComfortableNudgeLatSpeedCheckTime = 2.0;
  explicit FrontSideMovingSpacetimePlannerObjectTrajectoriesFinder(
      const Box2d& av_box, const PlanPassage* plan_passage,
      const PathSlBoundary* sl_boundary, double av_speed,
      const SpacetimePlannerObjectTrajectoriesProto* prev_st_trajs,
      const std::vector<ApolloTrajectoryPointProto>* time_aligned_prev_traj,
      const VehicleGeometryParamsProto* veh_geo,
      const PlannerSemanticMapManager* psmm,
      const NudgeObjectInfo* nudge_object_info,
      const LaneChangeStateProto* lane_change_state);
  SpacetimePlannerObjectTrajectoryReason::Type Find(
      const SpacetimeObjectTrajectory& traj) const override;

 private:
  Box2d av_box_;
  const PlanPassage* plan_passage_;
  const PathSlBoundary* path_sl_boundary_;
  const VehicleGeometryParamsProto* veh_geo_;
  double av_speed_;
  std::optional<FrenetBox> av_sl_box_;
  absl::flat_hash_set<std::string> prev_st_planner_obj_id_;

  const std::vector<ApolloTrajectoryPointProto>* time_aligned_prev_traj_;
  mutable std::optional<bool> is_prev_traj_effective_;
  mutable std::vector<Box2d> time_aligned_prev_traj_boxes_;

  const PlannerSemanticMapManager* psmm_;
  const NudgeObjectInfo* nudge_object_info_;
  const LaneChangeStateProto* lane_change_state_;
};

class DangerousSideMovingSpacetimePlannerObjectTrajectoriesFinder
    : public SpacetimePlannerObjectTrajectoriesFinder {
 public:
  explicit DangerousSideMovingSpacetimePlannerObjectTrajectoriesFinder(
      const Box2d& av_box, const PlanPassage* plan_passage, double av_velocity);
  SpacetimePlannerObjectTrajectoryReason::Type Find(
      const SpacetimeObjectTrajectory& traj) const override;

 private:
  Box2d av_box_;
  const PlanPassage* plan_passage_;
  Vec2d av_tangent_;
  double av_velocity_;
  std::optional<FrenetBox> av_sl_box_;
};

class FrontMovingSpacetimePlannerObjectTrajectoriesFinder
    : public SpacetimePlannerObjectTrajectoriesFinder {
 public:
  explicit FrontMovingSpacetimePlannerObjectTrajectoriesFinder(
      const PlanPassage* plan_passage,
      const ApolloTrajectoryPointProto* plan_start_point, double av_length);
  SpacetimePlannerObjectTrajectoryReason::Type Find(
      const SpacetimeObjectTrajectory& traj) const override;

 private:
  const PlanPassage* plan_passage_;
  const ApolloTrajectoryPointProto* plan_start_point_;
  FrenetCoordinate av_sl_;
  double av_length_;
};

}  // namespace planning
}  // namespace e2e_noa

#endif
