#ifndef ONBOARD_PLANNER_OBJECT_SPACETIME_PLANNER_OBJECT_TRAJECTORIES_FILTER_H_
#define ONBOARD_PLANNER_OBJECT_SPACETIME_PLANNER_OBJECT_TRAJECTORIES_FILTER_H_

#include <limits>

#include "common/path_sl_boundary.h"
#include "constraint.pb.h"
#include "lane_change.pb.h"
#include "object/spacetime_object_trajectory.h"
#include "plan/planner_semantic_map_manager.h"
#include "router/plan_passage.h"
#include "util/hmi_content_util.h"
#include "vehicle.pb.h"

namespace e2e_noa {
namespace planning {

class SpacetimePlannerObjectTrajectoriesFilter {
 public:
  virtual bool Filter(const SpacetimeObjectTrajectory& traj) const = 0;
  virtual ~SpacetimePlannerObjectTrajectoriesFilter() = default;
};

class CutInSpacetimePlannerObjectTrajectoriesFilter
    : public SpacetimePlannerObjectTrajectoriesFilter {
 public:
  CutInSpacetimePlannerObjectTrajectoriesFilter(
      const PlanPassage* plan_passage,
      const LaneChangeStateProto* lane_change_state, const Box2d& av_box,
      double av_speed);
  bool Filter(const SpacetimeObjectTrajectory& traj) const override;

 private:
  const PlanPassage* plan_passage_;
  const LaneChangeStateProto* lane_change_state_;
  Box2d av_box_;
  std::optional<FrenetBox> av_sl_box_;
  double av_speed_;
};

class CutInVehicleSpacetimePlannerObjectTrajectoriesFilter
    : public SpacetimePlannerObjectTrajectoriesFilter {
 public:
  CutInVehicleSpacetimePlannerObjectTrajectoriesFilter(
      const PlanPassage* plan_passage,
      const LaneChangeStateProto* lane_change_state, const Box2d& av_box,
      double av_speed);
  bool Filter(const SpacetimeObjectTrajectory& traj) const override;

 private:
  const PlanPassage* plan_passage_;
  const LaneChangeStateProto* lane_change_state_;
  Box2d av_box_;
  std::optional<FrenetBox> av_sl_box_;
  double av_speed_;
};

class CrossingSpacetimePlannerObjectTrajectoriesFilter
    : public SpacetimePlannerObjectTrajectoriesFilter {
 public:
  CrossingSpacetimePlannerObjectTrajectoriesFilter(
      const PlanPassage* plan_passage, const PlannerSemanticMapManager* psmm);
  bool Filter(const SpacetimeObjectTrajectory& traj) const override;

 private:
  const PlanPassage* plan_passage_;
  const PlannerSemanticMapManager* psmm_;
};

class ReverseVehicleSpacetimePlannerObjectTrajectoriesFilter
    : public SpacetimePlannerObjectTrajectoriesFilter {
 public:
  ReverseVehicleSpacetimePlannerObjectTrajectoriesFilter(
      const PlanPassage* plan_passage, const PlannerSemanticMapManager* psmm,
      const VehicleGeometryParamsProto* vehicle_geometry_params,
      const PathSlBoundary* sl_boundary,
      const NudgeObjectInfo* nudge_object_info, Box2d av_box, double av_speed);
  bool Filter(const SpacetimeObjectTrajectory& traj) const override;

 private:
  const PlanPassage* plan_passage_;
  const PlannerSemanticMapManager* psmm_;
  const VehicleGeometryParamsProto* vehicle_geometry_params_;
  const PathSlBoundary* sl_boundary_;
  const NudgeObjectInfo* nudge_object_info_;
  const Box2d av_box_;
  const double av_speed_;
};

class BeyondStopLineSpacetimePlannerObjectTrajectoriesFilter
    : public SpacetimePlannerObjectTrajectoriesFilter {
 public:
  BeyondStopLineSpacetimePlannerObjectTrajectoriesFilter(
      const PlanPassage* plan_passage,
      absl::Span<const ConstraintProto::StopLineProto> stop_lines);
  bool Filter(const SpacetimeObjectTrajectory& traj) const override;

 private:
  const PlanPassage* plan_passage_;
  double first_stop_line_s_ = std::numeric_limits<double>::infinity();
};

bool IsCutInObjectTrajectory(const PlanPassage& plan_passage,
                             const bool is_lane_change, const double av_speed,
                             const FrenetBox& av_sl_box,
                             const SpacetimeObjectTrajectory& traj);
}  // namespace planning
}  // namespace e2e_noa

#endif
