#ifndef PLANNING_MAP_LANE_H
#define PLANNING_MAP_LANE_H

#include <memory>
#include <unordered_set>

#include "common/type_def.h"
#include "maps/crosswalk.h"
#include "maps/lane_boundaries.h"
#include "maps/map_def.h"
#include "maps/road_boundaries.h"
#include "maps/road_boundary.h"
#include "math/curve_limits.h"
#include "math/line_curve2d.h"

namespace ad_e2e {
namespace planning {
enum ValidationRule {
  LANE_TO_CURB_DIST = 1,
  LANE_MAX_LENGTH = 2,
  TL_BIND = 3,
  LANE_TOPOLOGICAL_CONNECTION = 4,
  LANE_CURVATURE = 5,
  LANE_DIRECTION = 6,
  CURB_CONNECTION = 7,
  LANE_INTERSECT_IN_INTERSECTION = 8,
  SECTION_VALIDITY = 9,
  TL_HEADING = 10,
  REPEATING_FEATURE = 11
};

struct ValidationRecord {
  ValidationRule rule;
  std::string note;
};

struct LaneInteraction {
  std::string other_lane_id = "";

  enum GeometricConfiguration { CROSS = 1, MERGE = 2 };
  GeometricConfiguration geometric_configuration;

  enum ReactionRule {

    STOP = 1,

    BOTH_STOP = 2,

    YIELD = 3,

    PROCEED = 4,

    YIELD_ON_RED = 6,

    YIELD_ON_GREEN_CIRCLE = 7,

    FFA = 5,

    YIELD_MERGE = 8,

    PROCEED_MERGE = 9
  };
  ReactionRule reaction_rule;
  std::string intersection_id;
  double this_lane_fraction;
  double other_lane_fraction;
  bool keep_it;
  int32_t belonging_levels;
  ValidationRecord skip_records;
};

class Lane {
 public:
  struct SampledWidth {
    double s = 0.0;
    double dist = 0.0;
  };
  Lane(const LaneInfo &lane_info, LaneBoundariesConstPtr left_boundary,
       LaneBoundariesConstPtr right_boundary,
       RoadBoundariesConstPtr left_road_boundary,
       RoadBoundariesConstPtr right_road_boundary);
  ~Lane() = default;

  void SetValidPredLaneIds(const std::vector<std::string> &lane_ids) {
    valid_pre_lane_ids_ = lane_ids;
  }
  void SetValidNextLaneIds(const std::vector<std::string> &lane_ids) {
    valid_next_lane_ids_ = lane_ids;
  }
  void SetNextTurnTypes(const std::unordered_set<TurnType> &turn_types) {
    next_turn_types_ = turn_types;
  }
  void SetSortedNextLaneIds(const std::vector<std::string> &lane_ids) {
    sorted_next_lane_ids_ = lane_ids;
  }
  void SetLaneIndInSection(std::size_t lane_ind_in_section) {
    lane_ind_in_section_ = lane_ind_in_section;
  }

  void SetSplitTopology(const SplitTopology &split_topology) {
    lane_info_.split_topology = split_topology;
  }
  void SetMergeTopology(const MergeTopology &merge_topology) {
    lane_info_.merge_topology = merge_topology;
  }

  const std::string &id() const { return lane_info_.id; }
  const std::string &section_id() const { return lane_info_.section_id; }
  const std::string &junction_id() const { return lane_info_.junction_id; }
  const LaneType &type() const { return lane_info_.type; }
  const bool &is_navigation() const { return lane_info_.is_navigation; }
  const bool &stop_line() const { return lane_info_.stop_line; }
  const double &speed_limit() const { return lane_info_.speed_limit; }
  const bool &is_split() const { return lane_info_.is_split; }
  const bool &is_merge() const { return lane_info_.is_merge; }
  const SplitTopology &split_topology() const {
    return lane_info_.split_topology;
  }
  const MergeTopology &merge_topology() const {
    return lane_info_.merge_topology;
  }

  const double topo_length() const { return lane_info_.length; }
  const double curve_length() const { return center_line_.length(); }
  const NoneOddType &none_odd_type() const { return lane_info_.none_odd_type; }
  const TurnType &turn_type() const { return lane_info_.turn_type; }
  const LightStatus &light_status() const { return lane_info_.light_status; }
  const std::string &left_lane_id() const { return lane_info_.left_lane_id; }
  const std::string &right_lane_id() const { return lane_info_.right_lane_id; }
  const std::vector<std::string> &pre_lane_ids() const {
    return prev_lane_ids_;
  }
  const std::vector<std::string> &next_lane_ids() const {
    return lane_info_.next_lane_ids;
  }
  const std::vector<std::string> &valid_pre_lane_ids() const {
    return valid_pre_lane_ids_;
  }
  const std::vector<std::string> &valid_next_lane_ids() const {
    return valid_next_lane_ids_;
  }
  const std::unordered_set<TurnType> &next_turn_types() const {
    return next_turn_types_;
  }
  const LaneInfo &lane_info() const { return lane_info_; }
  const LaneBoundariesConstPtr &left_boundary() const { return left_boundary_; }
  const LaneBoundariesConstPtr &right_boundary() const {
    return right_boundary_;
  }
  const RoadBoundariesConstPtr &left_road_boundary() const {
    return left_road_boundary_;
  }
  const RoadBoundariesConstPtr &right_road_boundary() const {
    return right_road_boundary_;
  }
  const std::vector<std::string> &sorted_next_lane_ids() const {
    return sorted_next_lane_ids_;
  };
  const bool IsValid() const;
  const bool IsVirtual() const { return lane_info_.is_virtual; }
  const std::vector<std::string> crosswalks() const {
    return lane_info_.cross_walks;
  }
  const std::size_t lane_ind_in_section() const { return lane_ind_in_section_; }
  const int32_t lane_operation_type() const {
    return lane_info_.lane_operation_type;
  }
  const int32_t arrow_type() const { return lane_info_.arrow_type; }
  bool GetCustomSpeedLimit(double *custom_speed_limit) const;

  void forward_boundary_type(
      const bool &is_left, const double &s,
      std::vector<LaneBoundaryType> &boundary_type) const;

  void forward_boundary_type(
      const bool &is_left, const Point2d &point,
      std::vector<LaneBoundaryType> &boundary_type) const;

  const math::LineCurve2d &center_line() const { return center_line_; }
  const std::vector<Point2d> &points() const { return center_line_.points(); }

  const std::vector<std::string> &overlap_cross_walks() const {
    return lane_info_.cross_walks;
  }
  const std::vector<std::string> &overlap_speed_bumps() const {
    return lane_info_.speed_bumps;
  }
  const std::vector<std::string> &overlap_parking_spaces() const {
    return lane_info_.parking_spaces;
  }
  const std::vector<std::string> &overlap_clear_areas() const {
    return lane_info_.clear_areas;
  }
  const std::vector<std::string> &overlap_stop_lines() const {
    return lane_info_.traffic_stop_lines;
  }

  bool IsOnLane(const math::Vec2d &point) const;

  void GetWidthAtAccumS(const double &s, double &left_w, double &right_w) const;

  double GetWidthAtPoint(const double &x, const double &y) const;

  double GetWidthAtAccumS(const double &s) const;

  bool IsPrecede(const std::string &lane_id) const;

  bool IsNext(const std::string &lane_id) const;

  bool GetSLWithLimit(const math::Vec2d &query_point,
                      SLPoint *const sl_point) const;
  bool GetSLWithoutLimit(const math::Vec2d &query_point,
                         SLPoint *const sl_point) const;
  bool GetXYWithoutLimit(const SLPoint &sl_point,
                         math::Vec2d *const xy_point) const;
  bool GetHeadingFromS(const double &query_s, double *const heading) const;
  bool GetWidthFromS(const double &query_s, double *const lw,
                     double *const rw) const;

  double navi_distance() const { return navi_distance_; }
  int navi_section_cnt() const { return navi_section_cnt_; }

  bool endpoint_toll() const { return endpoint_toll_; };

  void SetIsNavigation(const bool &is_navi, const double navi_distance,
                       const int navi_section_cnt) {
    lane_info_.is_navigation = is_navi;
    navi_distance_ = is_navi ? navi_distance : 0.0;
    navi_section_cnt_ = is_navi ? navi_section_cnt : 0.0;
  };
  void SetIsNavigation(const bool &is_navi) {
    lane_info_.is_navigation = is_navi;
  };
  void SetSectionId(const std::string &id) {
    if (id.length() > 0) {
      lane_info_.section_id = id;
    }
  };
  void SetTrueLength(const double &length) { lane_info_.length = length; };
  void AddPreviousLane(const std::string &id) {
    prev_lane_ids_.emplace_back(id);
  }

  double LaneFraction(int segment, double segment_fraction) const;

  std::pair<int, double> SegmentFraction(double lane_fraction) const;

  bool GetTangent(double fraction, math::Vec2d *tangent) const;

  math::Vec2d GetTangent(double fraction) const;

  math::Vec2d LerpPointFromFraction(double fraction) const;

  bool IsPassengerVehicleAvoidLaneType() const { return false; }
  static bool IsPassengerVehicleAvoidLaneType(const LaneType &lanetype) {
    return LaneType::LANE_BUS_NORMAL == lanetype;
  }

  std::vector<std::string> startpoint_associated_traffic_lights() const {
    return {};
  }
  std::vector<LaneInteraction> interactions() const { return {}; }

 private:
  double CalculateWidth(math::LineSegment2d &base, Point2d &p, int lr) const;
  void GenerateSampleWidth(LaneBoundariesConstPtr &boundary,
                           std::list<SampledWidth> *const sample_width);
  double GetDistanceFromSample(
      const double &query_s,
      const std::list<SampledWidth> *const sampled_widths) const;

 private:
  std::list<SampledWidth> sampled_left_width_;
  std::list<SampledWidth> sampled_right_width_;

  LaneInfo lane_info_;
  std::vector<std::string> prev_lane_ids_;
  std::vector<std::string> valid_pre_lane_ids_;
  std::vector<std::string> valid_next_lane_ids_;
  std::vector<std::string> sorted_next_lane_ids_;
  LaneBoundariesConstPtr left_boundary_ = nullptr;
  LaneBoundariesConstPtr right_boundary_ = nullptr;
  RoadBoundariesConstPtr left_road_boundary_ = nullptr;
  RoadBoundariesConstPtr right_road_boundary_ = nullptr;
  math::LineCurve2d center_line_;
  double navi_distance_ = 0.0;
  int navi_section_cnt_ = 0;
  std::unordered_set<TurnType> next_turn_types_;
  std::size_t lane_ind_in_section_ = 0u;

  bool endpoint_toll_ = false;
};

using LanePtr = std::shared_ptr<Lane>;
using LaneConstPtr = std::shared_ptr<const Lane>;

}  // namespace planning
}  // namespace ad_e2e

#endif
