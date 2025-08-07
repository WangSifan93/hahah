#ifndef E2E_NOA_PLANNING_SPEED_ST_GRAPH
#define E2E_NOA_PLANNING_SPEED_ST_GRAPH

#include <limits>
#include <map>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "absl/synchronization/mutex.h"
#include "absl/types/span.h"
#include "async/thread_pool.h"
#include "common/path_approx.h"
#include "common/path_sl_boundary.h"
#include "common/type_def.h"
#include "common/vehicle_shape.h"
#include "constraint.pb.h"
#include "descriptor/constraint_manager.h"
#include "gflags/gflags.h"
#include "math/geometry/polygon2d.h"
#include "math/geometry/segment2d.h"
#include "math/segment_matcher/segment_matcher_kdtree.h"
#include "math/vec.h"
#include "object/spacetime_object_state.h"
#include "object/spacetime_object_trajectory.h"
#include "object/spacetime_trajectory_manager.h"
#include "plan/discretized_path.h"
#include "plan/planner_semantic_map_manager.h"
#include "router/plan_passage.h"
#include "speed/slt_info.h"
#include "speed/st_boundary.h"
#include "speed/st_boundary_with_decision.h"
#include "speed/st_close_trajectory.h"
#include "speed/st_graph_defs.h"
#include "speed_planning_params.pb.h"
#include "util/hmi_content_util.h"
#include "vehicle.pb.h"

DECLARE_bool(planner_use_path_approx_based_st_mapping);

namespace e2e_noa::planning {

class StGraph {
 public:
  struct AgentNearestPoint {
    double ra_s = 0.0;
    double ra_heading = 0.0;
    double t = 0.0;
    double obj_v = 0.0;
    double obj_heading = 0.0;
    double lat_dist = 0.0;
    int obj_idx = 0;
  };

  struct StBoundaryOutput {
    std::vector<StBoundaryRef> st_boundaries;
    std::vector<SltInfoRef> slt_infos;
  };

  StGraph(const DiscretizedPath* path_points, int traj_steps,
          double plan_start_v, double max_decel,
          const VehicleGeometryParamsProto* vehicle_geo_params,
          const SpeedPlanningParamsProto::StGraphParamsProto* st_graph_params,
          const std::vector<VehicleShapeBasePtr>* av_shape_on_path_points,
          const KdtreeSegmentMatcher* path_kd_tree,
          const PathApprox* path_approx,
          const PathApprox* path_approx_for_mirrors);

  StBoundaryOutput GetStBoundaries(
      const SpacetimeTrajectoryManager& traj_mgr,
      const std::map<std::string, ConstraintProto::LeadingObjectProto>&
          leading_objs,
      bool consider_lane_change_gap, const ConstraintManager& constraint_mgr,
      const PlannerSemanticMapManager* psman_mgr,
      const PlanPassage* plan_passage, const PathSlBoundary* path_sl_boundary,
      const NudgeObjectInfo* nudge_object_info,
      WorkerThreadManager* thread_pool);

  std::vector<CloseSpaceTimeObject> GetCloseSpaceTimeObjects(
      absl::Span<const StBoundaryWithDecision> st_boundaries_with_decision,
      absl::Span<const SpacetimeObjectTrajectory* const> spacetime_object_trajs,
      double slow_down_radius) const;

  StBoundaryOutput MapMovingSpacetimeObject(
      const SpacetimeObjectTrajectory& spacetime_object,
      bool generate_lane_change_gap, bool calc_moving_close_traj,
      const NudgeObjectInfo* nudge_object_info) const;

  const std::vector<DistanceInfo>& distance_info_to_impassable_boundaries()
      const {
    return distance_info_to_impassable_boundaries_;
  }

  const std::vector<StCloseTrajectory>& moving_close_trajs() const {
    return moving_close_trajs_;
  }

  const std::map<std::string, ObjectSlInfo>& obj_sl_map() const {
    return obj_sl_map_;
  }

  StBoundaryRef MapPathStopLine(
      const ConstraintProto::PathStopLineProto& stop_line) const;

 private:
  std::vector<StBoundaryRef> MapStationarySpacetimeObjects(
      absl::Span<const SpacetimeObjectTrajectory* const>
          stationary_spacetime_objs,
      const std::map<std::string, ConstraintProto::LeadingObjectProto>&
          leading_objs) const;

  StBoundaryRef MapStopLine(
      const ConstraintProto::StopLineProto& stop_line) const;

  StBoundaryRef MapNearestImpassableBoundary(
      const PlannerSemanticMapManager& psman_mgr);

  StBoundaryRef MapNearestPathBoundary(
      const PlanPassage& plan_passage,
      const PathSlBoundary& path_sl_boundary) const;

  std::vector<StBoundaryPoints> GetMovingObjStBoundaryPoints(
      const SpacetimeObjectTrajectory& spacetime_object,
      bool generate_lane_change_gap, bool calc_moving_close_traj,
      std::vector<NearestSlPoint>* const near_sl_points,
      const NudgeObjectInfo* nudge_object_info) const;

  std::optional<StBoundaryPoints> GetStationaryObjStBoundaryPoints(
      const SpacetimeObjectTrajectory& spacetime_object,
      const double max_kappa = 0.0) const;

  bool FindOverlapRangeOrNearestPoint(
      const Vec2d& search_point, double search_radius,
      double search_radius_for_mirrors, const SpacetimeObjectState& obj_state,
      int obj_idx, bool consider_mirrors, double lat_buffer, double lon_buffer,
      int* low_idx, int* high_idx,
      std::vector<AgentNearestPoint>* agent_nearest_points) const;

  void GenerateLatInflatedStBoundary(
      const SpacetimeObjectTrajectory& spacetime_object,
      StBoundaryProto::ProtectionType protection_type, double lat_buffer,
      std::vector<StBoundaryPoints>* st_boundaries_points,
      bool large_vehicle_blind_stop = false) const;

  std::optional<std::pair<int, int>> FindSegment2dOverlapRange(
      const Segment2d& segment) const;

  std::optional<std::pair<int, int>> FindStopLine2dOverlapRange(
      const Segment2d& segment, const bool is_extended) const;

  std::optional<std::pair<int, int>> FindStopLineOverlapRange(
      const ConstraintProto::StopLineProto& stop_line) const;

  std::vector<StBoundaryRef> MapLeadingSpacetimeObjects(
      const SpacetimeTrajectoryManager& traj_mgr,
      const std::map<std::string, ConstraintProto::LeadingObjectProto>&
          leading_objs,
      const bool lc_acc_gap, const PlanPassage& plan_passage,
      const PathSlBoundary& path_sl_boundary) const;

  std::optional<int> FindLeadingObjectLowerIndex(
      const Polygon2d& obj_shape, const PlanPassage& plan_passage,
      const PathSlBoundary& path_sl_boundary) const;

  bool GetStDistancePointInfo(const SpacetimeObjectState& state,
                              double slow_down_radius,
                              StDistancePoint* st_distance_point) const;

  void CalculateObjectSlPosition(const PlanPassage plan_passage,
                                 const SpacetimeTrajectoryManager& traj_mgr);

  std::pair<double, double> GetStBoundaryTRangeByProtectionType(
      const SpacetimeObjectTrajectory& spacetime_object,
      StBoundaryProto::ProtectionType protection_type) const;

 private:
  double total_plan_time_ = 0.0;
  double plan_start_v_ = 0.0;
  double max_decel_ = 0.0;
  const DiscretizedPath* path_points_;

  const std::vector<VehicleShapeBasePtr>* av_shape_on_path_points_;
  const KdtreeSegmentMatcher* path_kd_tree_;
  double ego_radius_ = 0.0;
  double ego_radius_for_mirrors_ = 0.0;
  double min_mirror_height_avg_ = std::numeric_limits<double>::infinity();
  double max_mirror_height_avg_ = -std::numeric_limits<double>::infinity();

  const VehicleGeometryParamsProto* vehicle_geo_params_;
  const SpeedPlanningParamsProto::StGraphParamsProto* st_graph_params_;
  std::vector<DistanceInfo> distance_info_to_impassable_boundaries_;
  const PathApprox* path_approx_;
  const PathApprox* path_approx_for_mirrors_;
  mutable std::vector<StCloseTrajectory> moving_close_trajs_;
  std::map<std::string, ObjectSlInfo> obj_sl_map_;
  mutable absl::Mutex mutex_;
};

}  // namespace e2e_noa::planning

#endif
