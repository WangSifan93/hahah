#ifndef ONBOARD_PLANNER_ROUTER_PLAN_PASSAGE_H_
#define ONBOARD_PLANNER_ROUTER_PLAN_PASSAGE_H_

#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/types/span.h"
#include "container/strong_int.h"
#include "container/strong_vector.h"
#include "glog/logging.h"
#include "maps/lane_path.h"
#include "maps/lane_point.h"
#include "maps/map_def.h"
#include "maps/semantic_map_defs.h"
#include "math/frenet_common.h"
#include "math/frenet_frame.h"
#include "math/geometry/box2d.h"
#include "math/geometry/polygon2d.h"
#include "math/geometry/segment2d.h"
#include "math/vec.h"

namespace e2e_noa::planning {

DECLARE_STRONG_VECTOR(Station);

using LaneType = ad_e2e::planning::LaneType;
using TurnType = ad_e2e::planning::TurnType;
using SplitTopology = ad_e2e::planning::SplitTopology;
using MergeTopology = ad_e2e::planning::MergeTopology;

struct StationInfo {
  bool is_in_intersection = false;
  bool is_exclusive_right_turn = false;
  bool is_in_roundabout = false;
  double speed_limit = DBL_MAX;
  LaneType lane_type = LaneType::LANE_NORMAL;
  TurnType turn_type = TurnType::NO_TURN;
  SplitTopology split_topo = SplitTopology::TOPOLOGY_SPLIT_NONE;
  MergeTopology merge_topo = MergeTopology::TOPOLOGY_MERGE_NONE;
};

struct StationCenter {
  mapping::ElementId lane_id = mapping::kInvalidElementId;
  double fraction = 0.0;
  Vec2d xy;
  Vec2d tangent;
  double accum_s;
  double speed_limit;
  bool is_virtual;
  bool is_merging;
  bool is_splitting;
  bool is_in_intersection;
  bool has_cross_curb;
  ad_e2e::planning::TurnType turn_type;
  StationInfo station_info;

  mapping::LanePoint GetLanePoint() const {
    return mapping::LanePoint(lane_id, fraction);
  }

  Vec2d lat_point(double signed_offset) const {
    return xy + tangent.Perp() * signed_offset;
  }
  Vec2d lon_point(double signed_offset) const {
    return xy + tangent * signed_offset;
  }
  double lat_offset(const Vec2d& v) const { return tangent.CrossProd(v - xy); }
  double lon_offset(const Vec2d& v) const { return tangent.Dot(v - xy); }
};

enum StationBoundaryType {

  BROKEN_WHITE = 0,
  SOLID_WHITE = 1,
  BROKEN_YELLOW = 2,
  SOLID_YELLOW = 3,
  SOLID_DOUBLE_YELLOW = 4,
  CURB = 5,
  VIRTUAL_CURB = 6,
  BROKEN_LEFT_DOUBLE_WHITE = 7,
  BROKEN_RIGHT_DOUBLE_WHITE = 8,
  VIRTUAL_LANE = 9,

  UNKNOWN_TYPE = 99
};

inline std::string StationBoundaryTypeName(StationBoundaryType type) {
  return "TODO";
}

struct StationBoundary {
  StationBoundaryType type;
  double lat_offset;

  bool IsSolid(double query_lat_offset) const {
    constexpr double kEpsilon = 0.5;
    switch (type) {
      case StationBoundaryType::SOLID_WHITE:
      case StationBoundaryType::SOLID_YELLOW:
      case StationBoundaryType::SOLID_DOUBLE_YELLOW:
      case StationBoundaryType::CURB:
      case StationBoundaryType::VIRTUAL_CURB:
        return true;
      case StationBoundaryType::BROKEN_LEFT_DOUBLE_WHITE:
        return query_lat_offset < lat_offset - kEpsilon;
      case StationBoundaryType::BROKEN_RIGHT_DOUBLE_WHITE:
        return query_lat_offset > lat_offset + kEpsilon;
      case StationBoundaryType::UNKNOWN_TYPE:
      case StationBoundaryType::BROKEN_WHITE:
      case StationBoundaryType::BROKEN_YELLOW:
      case StationBoundaryType::VIRTUAL_LANE:
        return false;
      default:
        throw std::runtime_error("switch case on enum unexpected");
    }
  }
};

using OptionalBoundary = std::optional<StationBoundary>;
struct BoundaryQueryResponse {
  OptionalBoundary right;
  OptionalBoundary left;
};

class Station {
 public:
  explicit Station(StationCenter center, std::vector<StationBoundary> bounds)
      : center_(std::move(center)), boundaries_(std::move(bounds)) {}

  mapping::ElementId lane_id() const { return center_.lane_id; }
  const Vec2d& xy() const { return center_.xy; }
  const Vec2d& tangent() const { return center_.tangent; }
  double accumulated_s() const { return center_.accum_s; }
  double speed_limit() const { return center_.speed_limit; }
  bool is_virtual() const { return center_.is_virtual; }

  StationInfo station_info() const { return center_.station_info; }
  bool is_merging() const { return center_.is_merging; }
  bool is_splitting() const { return center_.is_splitting; }
  bool is_in_intersection() const { return center_.is_in_intersection; }
  bool has_cross_curb() const { return center_.has_cross_curb; }
  ad_e2e::planning::TurnType turn_type() const { return center_.turn_type; }

  Vec2d lat_point(double signed_offset) const {
    return center_.lat_point(signed_offset);
  }
  Vec2d lon_point(double signed_offset) const {
    return center_.lon_point(signed_offset);
  }
  double lat_offset(const Vec2d& v) const { return center_.lat_offset(v); }
  double lon_offset(const Vec2d& v) const { return center_.lon_offset(v); }

  absl::Span<const StationBoundary> boundaries() const { return boundaries_; }

  absl::StatusOr<std::pair<double, double>> QueryCurbOffsetAt(
      double signed_lat) const;

  absl::StatusOr<BoundaryQueryResponse> QueryEnclosingLaneBoundariesAt(
      double signed_lat) const;

  mapping::LanePoint GetLanePoint() const { return center_.GetLanePoint(); }

 private:
  StationCenter center_;

  std::vector<StationBoundary> boundaries_;
};

struct StationWaypoint {
  StationIndex station_index;
  double lon_offset;
  double accum_s;
};

class PlanPassage {
 public:
  PlanPassage() = default;
  PlanPassage(StationVector<Station> stations, mapping::LanePath lane_path,
              mapping::LanePath extend_lane_path, double lane_path_start_s,
              bool reach_destination, FrenetFrameType type,
              std::vector<int> change_index,
              ad_e2e::planning::LaneSeqInfoPtr lane_seq_info = nullptr);

  PlanPassage(PlanPassage const& o)
      : stations_(o.stations_),
        last_real_station_index_(o.last_real_station_index_),
        center_seg_inv_len_(o.center_seg_inv_len_),
        lane_path_(o.lane_path_),
        extend_lane_path_(o.extend_lane_path_),
        beyond_lane_path_(o.beyond_lane_path_),
        reach_destination_(o.reach_destination_),
        lane_path_start_s_(o.lane_path_start_s_),
        segments_(o.segments_),
        type_(o.type_),
        lane_seq_info_(o.lane_seq_info_),
        change_index_(o.change_index_) {
    BuildFrenetFrame();
  }

  PlanPassage& operator=(PlanPassage const& o) {
    stations_ = o.stations_;
    last_real_station_index_ = o.last_real_station_index_;
    center_seg_inv_len_ = o.center_seg_inv_len_;
    lane_path_ = o.lane_path_;
    extend_lane_path_ = o.extend_lane_path_;
    beyond_lane_path_ = o.beyond_lane_path_;
    reach_destination_ = o.reach_destination_;
    lane_path_start_s_ = o.lane_path_start_s_;
    segments_ = o.segments_;
    type_ = o.type_;
    BuildFrenetFrame();
    return *this;
  }

  PlanPassage(PlanPassage&& o) = default;
  PlanPassage& operator=(PlanPassage&& o) = default;

  absl::StatusOr<double> QuerySpeedLimitAt(const Vec2d& point) const;
  absl::StatusOr<double> QuerySpeedLimitAtS(double s) const;

  absl::StatusOr<std::pair<double, double>> QueryCurbOffsetAt(
      const Vec2d& point) const;
  absl::StatusOr<std::pair<double, double>> QueryCurbOffsetAtS(double s) const;

  absl::StatusOr<std::pair<double, double>> QueryNearestBoundaryLateralOffset(
      double s) const;

  absl::StatusOr<std::pair<Vec2d, Vec2d>> QueryCurbPointAt(
      const Vec2d& point) const;

  absl::StatusOr<std::pair<Vec2d, Vec2d>> QueryCurbPointAtS(double s) const;

  absl::StatusOr<BoundaryQueryResponse> QueryEnclosingLaneBoundariesAt(
      const Vec2d& point) const;
  BoundaryQueryResponse QueryEnclosingLaneBoundariesAtS(double s) const;

  absl::StatusOr<Vec2d> QueryLaterallyUnboundedTangentAt(
      const Vec2d& point) const;

  absl::StatusOr<Vec2d> QueryTangentAt(const Vec2d& point) const;
  absl::StatusOr<Vec2d> QueryTangentAtS(double s) const;
  absl::StatusOr<double> QueryTangentAngleAtS(double s) const;

  absl::StatusOr<Vec2d> QueryPointXYAtS(double s) const;

  absl::StatusOr<Vec2d> QueryPointXYAtSL(double s, double l) const;

  absl::StatusOr<StationWaypoint> QueryFrenetLonOffsetAt(
      const Vec2d& point) const;

  absl::StatusOr<double> QueryFrenetLatOffsetAt(const Vec2d& point) const;

  absl::StatusOr<FrenetCoordinate> QueryFrenetCoordinateAt(
      const Vec2d& point) const;

  absl::StatusOr<FrenetBox> QueryFrenetBoxAt(const Box2d& box,
                                             bool zone_checking = true) const;

  absl::StatusOr<FrenetBox> QueryFrenetBoxAtContour(
      const Polygon2d& contour, bool zone_checking = true) const;

  absl::StatusOr<FrenetCoordinate> QueryLaterallyUnboundedFrenetCoordinateAt(
      const Vec2d& point) const;

  absl::StatusOr<FrenetCoordinate> QueryUnboundedFrenetCoordinateAt(
      const Vec2d& point) const;

  absl::StatusOr<std::vector<FrenetCoordinate>> BatchQueryFrenetCoordinates(
      absl::Span<const Vec2d> points) const;

  absl::StatusOr<std::vector<std::optional<FrenetBox>>> BatchQueryFrenetBoxes(
      absl::Span<const Box2d> boxes, bool laterally_bounded) const;

  absl::StatusOr<Vec2d> FindNearestPointOnCenterLine(const Vec2d& point) const;

  StationIndex FindNearestStationIndex(const Vec2d& point) const;
  const Station& FindNearestStation(const Vec2d& point) const {
    return stations_[FindNearestStationIndex(point)];
  }

  StationIndex FindNearestStationIndexAtS(double s) const;
  const Station& FindNearestStationAtS(double s) const {
    return stations_[FindNearestStationIndexAtS(s)];
  }

  double end_s() const { return stations_.back().accumulated_s(); }
  double front_s() const { return stations_.front().accumulated_s(); }
  double lane_path_start_s() const { return lane_path_start_s_; }
  bool beyond_lane_path() const { return beyond_lane_path_; }
  bool reach_destination() const { return reach_destination_; }

  bool ContainIntersection(const int check_step) const;
  double GetDistanceToIntersection(const Vec2d& point) const;
  double GetDistanceToNonIntersection(const Vec2d& point) const;
  const Station& GetFirstNonIntersectionStation(const Vec2d& point) const;

  bool empty() const { return stations_.size() == 0; }
  int size() const { return stations_.size(); }

  const Station& station(StationIndex index) const { return stations_[index]; }
  const StationVector<Station>& stations() const { return stations_; }
  StationIndex last_real_station_index() const {
    return last_real_station_index_;
  }
  const std::vector<int>& change_index() const { return change_index_; }

  const FrenetFrame* frenet_frame() const { return frenet_frame_.get(); }

  const mapping::LanePath& extend_lane_path() const {
    return extend_lane_path_;
  }

  const mapping::LanePath& lane_path() const { return lane_path_; }

  const std::vector<Segment2d>& segments() const { return segments_; }
  const ad_e2e::planning::LaneSeqInfoPtr lane_seq_info() const {
    return lane_seq_info_;
  }

 private:
  struct ProjectionResult {
    StationIndex station_index_1;
    StationIndex station_index_2;
    StationIndex near_station_index;
    double accum_s;
    double signed_l;
    double lerp_factor;
  };

  absl::StatusOr<ProjectionResult> ProjectPointToStations(
      const Vec2d& point, bool allow_extrapolation) const;
  absl::StatusOr<ProjectionResult> ProjectPointToStationsWithinRadius(
      const Vec2d& point, const ProjectionResult& projection,
      const Vec2d& prev_point, double search_radius) const;
  absl::Status IsProjectionResultOnPlanPassage(
      const ProjectionResult& res) const;
  absl::StatusOr<std::optional<FrenetBox>> QueryFrenetBoxWithinRadius(
      const Box2d& box, const ProjectionResult& center_projection,
      double search_radius, bool laterally_bounded) const;
  absl::StatusOr<FrenetBox> QueryFrenetBoxAtContourPoints(
      absl::Span<const Vec2d> contour_points, bool zone_checking = true) const;

  struct BinarySeachResult {
    StationIndex station_index_1;
    StationIndex station_index_2;
    StationIndex near_station_index;
    double ds;
  };

  BinarySeachResult BinarySearchForNearStation(double s) const;

  void BuildFrenetFrame();

  StationVector<Station> stations_;
  StationIndex last_real_station_index_;

  std::vector<double> center_seg_inv_len_;
  mapping::LanePath lane_path_;
  mapping::LanePath extend_lane_path_;
  bool beyond_lane_path_;
  bool reach_destination_;
  double lane_path_start_s_;
  std::vector<Segment2d> segments_;

  FrenetFrameType type_;
  ad_e2e::planning::LaneSeqInfoPtr lane_seq_info_ = nullptr;
  std::vector<int> change_index_;
  std::unique_ptr<FrenetFrame> frenet_frame_;
};

}  // namespace e2e_noa::planning

#endif
