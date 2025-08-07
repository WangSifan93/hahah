#ifndef PLANNING_MAP_MAP_H
#define PLANNING_MAP_MAP_H
#include <unordered_map>

#include "absl/base/thread_annotations.h"
#include "absl/container/flat_hash_map.h"
#include "absl/status/statusor.h"
#include "absl/synchronization/mutex.h"
#include "async/future.h"
#include "async/thread_pool.h"
#include "maps/clear_aera.h"
#include "maps/crosswalk.h"
#include "maps/junction.h"
#include "maps/lane.h"
#include "maps/lane_boundaries.h"
#include "maps/lane_boundary.h"
#include "maps/lane_sequence.h"
#include "maps/map_def.h"
#include "maps/road_boundary.h"
#include "maps/route.h"
#include "maps/section.h"
#include "maps/semantic_map_defs.h"
#include "maps/speed_bump.h"
#include "maps/stop_line.h"
#include "math/geometry/aabox2d.h"
#include "math/geometry/aabox_kdtree2d.h"
#include "math/geometry/segment2d.h"
#include "math/math_utils.h"
#include "math/vec.h"
#include "util/kdtree.h"
namespace ad_e2e {
namespace planning {

using Vec2d = math::Vec2d;
using TrafficLightInfoPtr = std::shared_ptr<TrafficLightInfo>;

struct KDValue {
  KDValue(const std::string &id, int idx) : boundary_id(id), point_idx(idx) {}
  KDValue() {}
  std::string boundary_id;
  int point_idx = -1;
};

class Map {
 public:
  explicit Map(const MapInfo &map_info);

  const MapType &type() const { return type_; }
  const MapSubType &sub_type() const { return sub_type_; }

  const bool is_on_highway() const { return is_on_highway_; }

  const std::vector<V2RoadClass> GetRoadClass() const {
    return v2_info_.road_class;
  }

  const TransformInfo &utm_to_cur_frame() const { return utm_to_cur_frame_; }

  RoutePtr route() const { return route_; }

  ClearAreaConstPtr GetClearAreaById(const std::string &id) const;

  LaneConstPtr GetLaneById(const std::string &id) const;

  StopLineConstPtr GetStopLineById(const std::string &id) const;

  void UpdateStopLineStatus(const std::string &id, const LightStatus &status);

  SpeedBumpConstPtr GetSpeedBumpById(const std::string &id) const;

  void GetLanes(const Point2d &query_pt, const double &heading,
                const double &dist, const double &heading_limit,
                std::vector<LaneConstPtr> *const lanes) const;
  void GetLanes(const Point2d &query_pt, const double &heading,
                const double &dist,
                std::vector<LaneConstPtr> *const lanes) const;
  void GetLanes(const Point2d &query_pt, const double &dist,
                std::vector<LaneConstPtr> *const lanes) const;
  void OnLanes(const Point2d &query_pt, const double &heading,
               const double &heading_limit,
               std::vector<LaneConstPtr> *const lanes) const;
  void OnLanes(const Point2d &query_pt, const double &heading,
               std::vector<LaneConstPtr> *const lanes) const;
  void OnLanes(const Point2d &query_pt,
               std::vector<LaneConstPtr> *const lanes) const;

  LaneConstPtr GetNearestLane(const Point2d &pt, const double &heading,
                              const double &dis, bool is_navi = false,
                              bool is_virtual = false,
                              double angle_range = M_PI / 6.0) const;

  LaneConstPtr GetNearestLane(const std::vector<LaneConstPtr> &candidate_lanes,
                              const Point2d &pt, const double &heading) const;

  LaneConstPtr GetNearestLane(const Point2d &pt, const double &dis) const;

  LaneConstPtr GetLeftLane(const LaneConstPtr &lane) const;

  LaneConstPtr GetRightLane(const LaneConstPtr &lane) const;

  void GetPrecedeLanes(const LaneConstPtr &lane,
                       std::vector<LaneConstPtr> *const lanes,
                       bool only_navi = false) const;

  bool IsValid() const;

  const std::vector<LaneConstPtr> &lanes() const { return lanes_; }
  const std::unordered_map<std::string, LanePtr> &lane_map() const {
    return lane_map_;
  }

  JunctionConstPtr GetJunctionById(const std::string &id) const;

  void GetJunctions(const Point2d &pt, const double &dist,
                    std::vector<JunctionConstPtr> *const junctions) const;

  const std::vector<JunctionConstPtr> &junctions() const { return junctions_; }

  const std::unordered_map<std::string, JunctionPtr> &junction_map() const {
    return junction_map_;
  }

  TrafficLightInfoPtr GetTrafficLightInfoById(const std::string &id) const;

  const std::unordered_map<std::string, TrafficLightInfoPtr> &
  traffic_light_info_map() const {
    return traffic_light_info_map_;
  }

  CrosswalkConstPtr GetCrosswalkById(const std::string &id) const;
  const std::unordered_map<std::string, CrosswalkPtr> &crosswalk_map() const {
    return crosswalk_map_;
  }

  RoadBoundaryConstPtr GetRoadBoundaryById(const std::string &id) const;
  LaneBoundaryConstPtr GetLaneBoundaryById(const std::string &id) const;
  SectionConstPtr GetSectionById(const std::string &id) const;
  const std::unordered_map<std::string, RoadBoundaryPtr> &road_boundary_map()
      const {
    return road_boundary_map_;
  }
  const std::unordered_map<std::string, LaneBoundaryPtr> &lane_boundary_map()
      const {
    return lane_boundary_map_;
  };
  const std::unordered_map<std::string, ClearAreaPtr> &clear_area_map() const {
    return clear_area_map_;
  }
  const std::unordered_map<std::string, StopLinePtr> &stop_line_map() const {
    return stop_line_map_;
  }
  const std::unordered_map<std::string, SpeedBumpPtr> &speed_bump_map() const {
    return speed_bump_map_;
  };
  TrafficLightStatusMap traffic_light_status_map() const {
    return traffic_light_status_map_;
  }
  const EHPV2Info &v2_info() const { return v2_info_; }
  double timestamp() const { return timestamp_; };
  int64_t seq() const { return seq_; };
  std::vector<std::string> FindNearRoadBoundaryIds(
      const std::string &obs_id, const Vec2d &position, const double heading,
      double range, bool check_angle, double heading_thresh,
      int *left_or_right) const;

  std::vector<LaneConstPtr> GetNextLanes(const LaneConstPtr &lane) const;
  std::vector<LaneConstPtr> GetValidNextLanes(const LaneConstPtr &lane) const;
  LaneConstPtr GetOptimalNextLane(const LaneConstPtr &lane,
                                  const bool &navi_is_priority) const;

  LaneConstPtr GetOptimalSmoothNextLane(const LaneConstPtr &lane,
                                        const bool &navi_is_priority,
                                        std::string &debug) const;

  LaneSequencePtr GetLaneSequence(const LaneConstPtr &lane,
                                  const bool &navi_is_priority) const;

  LaneSequencePtr GetSameLaneSequence(const LaneSequencePtr &lane_sequence,
                                      const double &x, const double &y);
  LaneSequencePtr GetSameLaneSequenceV2(
      const LaneSequencePtr &lane_sequence, const double &x, const double &y,
      const double &heading, std::vector<std::string> &debug,
      std::vector<Point2d> &debug_match_point) const;
  void GetAllLaneSequences(
      const LaneConstPtr &lane,
      std::vector<std::vector<LaneConstPtr>> &all_lane_sequences) const;
  void GetAllLaneSequences(
      const LaneConstPtr &lane, const double &length,
      std::vector<std::vector<LaneConstPtr>> &sequences) const;
  CompositeTurnType CheckCompositeLane(const LanePtr &lane) const;

  absl::StatusOr<e2e_noa::mapping::v2::Segment> FindNearestLaneSegment(
      double lon, double lat) const;
  absl::StatusOr<e2e_noa::mapping::v2::Segment> FindNearestRoadBoundarySegment(
      double lon, double lat) const;
  absl::StatusOr<e2e_noa::mapping::v2::Segment> FindNearestLaneBoundarySegment(
      double lon, double lat) const;
  LaneConstPtr FindNearestLane(double lon, double lat) const;
  LaneBoundaryConstPtr FindNearestLaneBoundary(double lon, double lat) const;
  RoadBoundaryConstPtr FindNearestRoadBoundary(double lon, double lat) const;
  StopLineConstPtr FindNearestStopLine(double lon, double lat) const;
  JunctionConstPtr FindNearestJunction(double lon, double lat) const;
  CrosswalkConstPtr FindNearestCrosswalk(double lon, double lat) const;
  SpeedBumpConstPtr FindNearestSpeedBump(double lon, double lat) const;
  ClearAreaConstPtr FindNearestClearArea(double lon, double lat) const;
  std::vector<e2e_noa::mapping::v2::Segment> FindLaneSegmentsInRadius(
      double lon, double lat, double radius) const;
  std::vector<e2e_noa::mapping::v2::Segment> FindLaneBoundarySegmentsInRadius(
      double lon, double lat, double radius) const;
  std::vector<e2e_noa::mapping::v2::Segment> FindRoadBoundarySegmentsInRadius(
      double lon, double lat, double radius) const;
  std::vector<e2e_noa::mapping::v2::Segment> FindStopLineSegmentsInRadius(
      double lon, double lat, double radius) const;
  std::vector<e2e_noa::mapping::v2::Segment> FindJunctionSegmentsInRadius(
      double lon, double lat, double radius) const;
  std::vector<e2e_noa::mapping::v2::Segment> FindCrosswalkSegmentsInRadius(
      double lon, double lat, double radius) const;
  std::vector<e2e_noa::mapping::v2::Segment> FindSpeedBumpSegmentsInRadius(
      double lon, double lat, double radius) const;
  std::vector<e2e_noa::mapping::v2::Segment> FindClearAreaSegmentsInRadius(
      double lon, double lat, double radius) const;

  std::vector<LaneConstPtr> FindLanesInRadius(double lon, double lat,
                                              double radius) const;
  std::vector<LaneBoundaryConstPtr> FindLaneBoundariesInRadius(
      double lon, double lat, double radius) const;
  std::vector<RoadBoundaryConstPtr> FindRoadBoundariesInRadius(
      double lon, double lat, double radius) const;
  std::vector<StopLineConstPtr> FindStopLinesInRadius(double lon, double lat,
                                                      double radius) const;
  std::vector<JunctionConstPtr> FindJunctionsInRadius(double lon, double lat,
                                                      double radius) const;
  std::vector<CrosswalkConstPtr> FindCrosswalksInRadius(double lon, double lat,
                                                        double radius) const;
  std::vector<SpeedBumpConstPtr> FindSpeedBumpsInRadius(double lon, double lat,
                                                        double radius) const;
  std::vector<ClearAreaConstPtr> FindClearAreasInRadius(double lon, double lat,
                                                        double radius) const;
  void FindAllLaneSequence(
      std::vector<LaneConstPtr> &lanes,
      std::vector<std::vector<LaneConstPtr>> &lane_sequences) const;

 private:
  LaneConstPtr GetSameLane(const LaneConstPtr &lane) const;
  void GetPolygonSRange(const LaneConstPtr &lane,
                        const math::Polygon2d &polygon, double *s_min,
                        double *s_max) const;
  void ConvertMapInfo(const MapInfo &map_info);
  void UpdateLane();
  void UpdateJunction();
  void UpdateSection();
  void UpdateBoundarySection();
  void SetEntryLanesForJunction(JunctionPtr &junction_ptr);
  void SetExitLanesForJunction(JunctionPtr &junction_ptr);
  void SetCrosswalkForJunction(JunctionPtr &junction_ptr);

  inline double GetMinLDistByPointIndex(const Point2d &query_pt,
                                        const std::vector<Point2d> &points,
                                        const int32_t &pt_idx) const;
  LaneBoundariesPtr BuildLaneBoundaries(
      const std::vector<std::string> &lane_boundary_ids);
  RoadBoundariesPtr BuildRoadBoundaries(
      const std::vector<std::string> &road_boundary_ids);
  void FilterPredecessorLanes(const Point2d &query_pt,
                              const std::vector<LaneConstPtr> &candidate_lanes,
                              std::vector<LaneConstPtr> *const lanes) const;

  void FindAllLaneSequence(
      std::vector<LaneConstPtr> &lanes, const double &length_limit,
      std::vector<std::vector<LaneConstPtr>> &lane_sequences) const;

 private:
  MapType type_ = PERCEPTION_MAP;
  MapSubType sub_type_;
  bool is_on_highway_ = false;
  TransformInfo utm_to_cur_frame_;
  std::unordered_map<std::string, LanePtr> lane_map_;
  std::vector<LaneConstPtr> lanes_;
  std::unordered_map<std::string, LaneBoundaryPtr> lane_boundary_map_;
  std::unordered_map<std::string, RoadBoundaryPtr> road_boundary_map_;
  std::unordered_map<std::string, ClearAreaPtr> clear_area_map_;
  std::unordered_map<std::string, JunctionPtr> junction_map_;
  std::unordered_map<std::string, CrosswalkPtr> crosswalk_map_;
  std::unordered_map<std::string, StopLinePtr> stop_line_map_;
  std::unordered_map<std::string, SpeedBumpPtr> speed_bump_map_;
  std::vector<JunctionConstPtr> junctions_;
  TrafficLightStatusMap traffic_light_status_map_;
  std::unordered_map<std::string, TrafficLightInfoPtr> traffic_light_info_map_;
  double timestamp_ = 0.0;
  int64_t seq_ = 0;
  RoutePtr route_;
  std::unordered_map<std::string, SectionPtr> section_map_;
  EHPV2Info v2_info_;

  std::unordered_map<std::string, std::vector<std::string>>
      boundary_id_lane_id_map_;

  struct SmoothPoint {
    double x;
    double y;
  };

  struct AABoxNode {
    AABoxNode(e2e_noa::mapping::ElementId element_id,
              e2e_noa::mapping::SegmentId segment_id, const Vec2d &start,
              const Vec2d &end)
        : element_id(element_id),
          segment_id(segment_id),
          start{start.x(), start.y()},
          end{end.x(), end.y()} {}

    e2e_noa::AABox2d ComputeAABox() const {
      return e2e_noa::AABox2d({start.x, start.y}, {end.x, end.y});
    }
    double DistanceSquareTo(const Vec2d &point) const {
      return e2e_noa::Segment2d({start.x, start.y}, {end.x, end.y})
          .DistanceSquareTo(point);
    }

    e2e_noa::mapping::ElementId GetElementId() const { return element_id; }
    e2e_noa::mapping::SegmentId GetSegmentId() const { return segment_id; }

   private:
    e2e_noa::mapping::ElementId element_id;
    e2e_noa::mapping::SegmentId segment_id;
    SmoothPoint start;
    SmoothPoint end;
  };
  struct AABoxTree {
    std::vector<AABoxNode> aabox_nodes;
    std::unique_ptr<e2e_noa::AABoxKDTree2d<AABoxNode>> aabox_tree;
  };
  absl::flat_hash_map<ad_e2e::planning::FeatureType,
                      e2e_noa::Future<std::shared_ptr<AABoxTree>>>
      feature_aabox_tree_;

  e2e_noa::WorkerThreadManager thread_pool_;
  void InitAABoxKDTree2d();
};

using MapPtr = std::shared_ptr<Map>;
using MapConstPtr = std::shared_ptr<const Map>;

}  // namespace planning
}  // namespace ad_e2e

#endif
