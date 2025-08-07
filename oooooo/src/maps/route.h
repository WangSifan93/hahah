#ifndef MAP_ROUTE_H
#define MAP_ROUTE_H
#include <map>
#include <unordered_map>
#include <utility>

#include "maps/lane.h"
#include "maps/lane_sequence.h"
#include "maps/map_def.h"

namespace ad_e2e {
namespace planning {
class Map;
class Route {
 public:
  Route(const RouteInfo &route_info, const Map *map) : map_(map) {
    route_info_ = route_info;
  };
  ~Route() {
    if (map_) map_ = nullptr;
  }

  const RouteInfo &GetRouteInfo() const { return route_info_; }

  void UpdateNaviPriorityLanes(const LaneConstPtr &ego_lane);

  bool GetSectionById(const std::string &section_id,
                      SectionInfo &section) const;

  bool CanDriveToRouteEnd(const LaneSequencePtr &lane_seq) const;

  LaneConstPtr GetNaviPriorityLane(const std::string &section_id) const;

  int GetPriorityLaneRelation(const LaneConstPtr &lane) const;

  double GetDistanceToX(const XRoadType &x_road_type, const double &x,
                        const double &y) const;

  double GetDistanceToNaviEnd() const;

  const NaviPosition &navi_start() const { return route_info_.navi_start; }
  const NaviPosition &navi_end() const { return route_info_.navi_end; }

  std::vector<std::string> GetNaviRouteIds() const {
    std::vector<std::string> navi_route_ids;
    std::for_each(
        route_info_.sections.begin(), route_info_.sections.end(),
        [&](SectionInfo section) { navi_route_ids.push_back(section.id); });
    return navi_route_ids;
  };

 private:
  struct LANE {
    explicit LANE(LaneConstPtr _lane) { lane = std::move(_lane); }
    LaneConstPtr lane;
    size_t lane_change = 0;
  };

  static double CalculateSequenceCost(const std::vector<LANE> &lanes);

  void UpdateNaviPriorityLaneId(const LaneConstPtr &lane);

  void FindReverseLaneSequence(std::vector<LANE> &lanes,
                               std::vector<std::vector<LANE>> &lane_sequences,
                               const std::string &stop_section_id);
  size_t GetPreNaviLanes(const LaneConstPtr &current_lane,
                         std::vector<LaneConstPtr> &pre_lanes);

 private:
  RouteInfo route_info_;
  const Map *map_ = nullptr;
  std::map<std::string, std::vector<LaneConstPtr>> pre_nextvr_lanes_;
};
typedef std::shared_ptr<Route> RoutePtr;

}  // namespace planning
}  // namespace ad_e2e

#endif
