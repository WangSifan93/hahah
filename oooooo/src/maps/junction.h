#ifndef MAP_JUNCTION_H
#define MAP_JUNCTION_H

#include "common/type_def.h"
#include "maps/crosswalk.h"
#include "maps/lane.h"
#include "math/polygon2d.h"

namespace ad_e2e {
namespace planning {

class Junction {
 public:
  Junction() = default;
  Junction(const JunctionInfo& junction_info);

  const std::string& id() const { return id_; }
  const std::vector<Point2d>& points() const { return points_; }
  const std::vector<LaneConstPtr>& entry_lanes() const { return entry_lanes_; }
  const std::vector<LaneConstPtr>& exit_lanes() const { return exit_lanes_; }
  const std::vector<LaneConstPtr>& overlap_lanes() const {
    return overlap_lanes_;
  }
  const std::unordered_set<CrosswalkConstPtr>& crosswalks() const {
    return crosswalks_;
  }
  const bool IsValid() const { return is_polygon_convex_; }
  const math::Polygon2d& polygon() const { return polygon_; };
  const bool IsRoundAbout() const { return is_roundabout_; }

  void SetEntryLanes(const std::vector<LaneConstPtr>& entry_lanes);
  void SetExitLanes(const std::vector<LaneConstPtr>& exit_lanes);
  void PushOverlapLane(const LaneConstPtr& lane_ptr);
  void SetCrosswalks(std::unordered_set<CrosswalkConstPtr>&& crosswalks);

  double DistanceTo(const Point2d& point) const;

  bool HasEntryLanes() const { return has_entry_lanes_; }
  bool HasExitLanes() const { return has_exit_lanes_; }

  bool traffic_light_controlled() const { return true; }

 protected:
  std::unordered_set<CrosswalkConstPtr> crosswalks_;
  std::vector<Point2d> points_;
  std::vector<LaneConstPtr> entry_lanes_;
  std::vector<LaneConstPtr> exit_lanes_;
  std::vector<LaneConstPtr> overlap_lanes_;
  bool has_entry_lanes_ = false;
  bool has_exit_lanes_ = false;
  math::Polygon2d polygon_;
  std::string id_;
  bool is_polygon_convex_ = false;
  bool is_roundabout_ = false;
};

using JunctionPtr = std::shared_ptr<Junction>;
using JunctionConstPtr = std::shared_ptr<const Junction>;

}  // namespace planning
}  // namespace ad_e2e

#endif
