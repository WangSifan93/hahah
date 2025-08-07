#ifndef AD_E2E_PLANNING_PLAN_PLANNER_SEMANTIC_MAP_MANAGER_H
#define AD_E2E_PLANNING_PLAN_PLANNER_SEMANTIC_MAP_MANAGER_H

#include <map>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "absl/status/status.h"
#include "async/thread_pool.h"
#include "behavior.pb.h"
#include "maps/map.h"
#include "maps/map_def.h"
#include "maps/semantic_map_defs.h"
#include "math/geometry/segment2d.h"
#include "math/vec.h"

namespace e2e_noa::planning {

constexpr double kDefaultMaxSpeedLimit = 50;

struct PlannerSemanticMapModification {
  std::map<mapping::ElementId, double> lane_speed_limit_map;
  double max_speed_limit = kDefaultMaxSpeedLimit;

  bool IsEmpty() const {
    return max_speed_limit >= kDefaultMaxSpeedLimit &&
           lane_speed_limit_map.empty();
  }
};

struct ImpassableBoundaryInfo {
  Segment2d segment;
  std::string id;
  std::optional<double> height;
  ad_e2e::planning::RoadBoundaryType type;
};

class PlannerSemanticMapManager {
 public:
  explicit PlannerSemanticMapManager(ad_e2e::planning::MapPtr map_ptr);

  explicit PlannerSemanticMapManager(ad_e2e::planning::MapPtr map_ptr,
                                     PlannerSemanticMapModification modifier);

  const ad_e2e::planning::MapPtr& map_ptr() const { return map_ptr_; }

  const bool IsOnHighway() const {
    return map_ptr_->is_on_highway() ||
           GetRoadClass() == ad_e2e::planning::V2RoadClass::EXPRESS_WAY_ROAD ||
           GetRoadClass() == ad_e2e::planning::V2RoadClass::HIGH_WAY_ROAD ||
           GetRoadClass() == ad_e2e::planning::V2RoadClass::NATIOANL_ROAD;
  }

  const bool IsOnHighway(const Behavior_FunctionId& func_id) const {
    return IsOnHighway() || func_id == Behavior_FunctionId_LKA_PLUS ||
           func_id == Behavior_FunctionId_NOA;
  }

  const ad_e2e::planning::V2RoadClass::V2RoadClassType GetRoadClass() const {
    return map_ptr_->GetRoadClass().empty()
               ? ad_e2e::planning::V2RoadClass::V2RoadClassType::UNKNOWN_ROAD
               : map_ptr_->GetRoadClass()[0].type;
  }

  const PlannerSemanticMapModification& GetSemanticMapModifier() const {
    return modifier_;
  }

  void SetSemanticMapModifier(PlannerSemanticMapModification modifier) {
    modifier_ = std::move(modifier);
  }

  double QueryLaneSpeedLimitById(mapping::ElementId lane_id) const;

  std::vector<ad_e2e::planning::LaneConstPtr> GetLanesInRadius(
      const Vec2d& smooth_coord, double radius) const;

  ad_e2e::planning::LaneConstPtr GetNearestLaneWithHeading(
      const Vec2d& smooth_coord, double theta, double radius,
      double max_heading_diff) const;

  std::vector<Segment2d> GetImpassableBoundaries(const Vec2d& smooth_coord,
                                                 double radius) const;
  std::vector<ImpassableBoundaryInfo> GetImpassableBoundariesInfo(
      const Vec2d& smooth_coord, double radius) const;

  bool GetLaneProjection(const Vec2d& smooth_coord, mapping::ElementId lane_id,
                         double* const fraction = nullptr,
                         Vec2d* const point = nullptr,
                         double* const min_dist = nullptr,
                         Segment2d* const segment = nullptr) const;

  ad_e2e::planning::LaneConstPtr GetNearestLane(
      const Vec2d& smooth_coord) const;

  enum class Side {
    kLEFT = 0,
    kRIGHT = 1,
  };

  std::optional<double> ComputeLaneWidth(const Vec2d& smooth_coord,
                                         mapping::ElementId lane_id,
                                         Side side) const;

  std::optional<double> GetLeftLaneWidth(const Vec2d& smooth_coord,
                                         mapping::ElementId lane_id) const;

  std::optional<double> GetRightLaneWidth(const Vec2d& smooth_coord,
                                          mapping::ElementId lane_id) const;

  std::vector<ad_e2e::planning::LaneBoundaryConstPtr> GetLaneBoundaries(
      const Vec2d& smooth_coord, double radius) const;
  std::vector<ad_e2e::planning::RoadBoundaryConstPtr> GetRoadBoundaries(
      const Vec2d& smooth_coord, double radius) const;

  std::vector<ad_e2e::planning::ClearAreaConstPtr> GetClearAreas(
      const Vec2d& smooth_coord, double radius) const;

  std::vector<ad_e2e::planning::CrosswalkConstPtr> GetCrosswalks(
      const Vec2d& smooth_coord, double radius) const;

  ad_e2e::planning::JunctionConstPtr GetNearestJunction(
      const Vec2d& smooth_coord) const;

  ad_e2e::planning::LaneConstPtr FindLaneByIdOrNull(
      mapping::ElementId id) const;
  ad_e2e::planning::LaneConstPtr FindCurveLaneByIdOrNull(
      mapping::ElementId id) const;
  ad_e2e::planning::LaneBoundaryConstPtr FindLaneBoundaryByIdOrNull(
      mapping::ElementId id) const;
  ad_e2e::planning::RoadBoundaryConstPtr FindRoadBoundaryByIdOrNull(
      mapping::ElementId id) const;
  ad_e2e::planning::JunctionConstPtr FindJunctionByIdOrNull(
      mapping::ElementId id) const;
  ad_e2e::planning::StopLineConstPtr FindStopLineByIdOrNull(
      mapping::ElementId id) const;
  ad_e2e::planning::CrosswalkConstPtr FindCrosswalkByIdOrNull(
      mapping::ElementId id) const;
  ad_e2e::planning::SpeedBumpConstPtr FindSpeedBumpByIdOrNull(
      mapping::ElementId id) const;
  ad_e2e::planning::ClearAreaConstPtr FindClearAreaByIdOrNull(
      mapping::ElementId id) const;
  ad_e2e::planning::SectionConstPtr FindSectionByIdOrNull(
      mapping::ElementId id) const;
  std::vector<ad_e2e::planning::CrosswalkConstPtr> GetCrosswalksInRadius(
      const Vec2d& smooth_coord, double radius) const;

 private:
  ad_e2e::planning::MapPtr map_ptr_;

  PlannerSemanticMapModification modifier_;
};
}  // namespace e2e_noa::planning

#endif
