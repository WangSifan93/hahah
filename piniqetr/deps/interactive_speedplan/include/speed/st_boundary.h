#ifndef E2E_NOA_PLANNING_SPEED_ST_BOUNDARY
#define E2E_NOA_PLANNING_SPEED_ST_BOUNDARY

#include <algorithm>
#include <cstddef>
#include <iterator>
#include <limits>
#include <memory>
#include <optional>
#include <ostream>
#include <string>
#include <utility>
#include <vector>

#include "glog/logging.h"
#include "math/frenet_common.h"
#include "math/geometry/box2d.h"
#include "math/geometry/polygon2d.h"
#include "math/vec.h"
#include "perception.pb.h"
#include "plan/trajectorypoint_with_acceleration.h"
#include "speed/overlap_info.h"
#include "speed/st_point.h"
#include "speed/vt_point.h"
#include "speed_planning.pb.h"

namespace e2e_noa::planning {

class StBoundary;

using StBoundaryRef = std::unique_ptr<StBoundary>;

struct NearestSlPoint {
  double t = 0.0;
  double av_s = 0.0;
  double av_heading = 0.0;
  double obj_v = 0.0;
  double obj_heading = 0.0;
  int obj_idx = 0;
  double lat_dist = 0.0;
  double obj_vl = 0.0;
};

struct ObjectSlInfo {
  double ds = 0.0;
  double dl = 0.0;
  double vs = 0.0;
  double vl = 0.0;
  double yaw_diff = 0.0;
  FrenetPolygon frenet_polygon;
};

enum class InteractionZone {
  Unknown = 0,
  Straight = 1,
  JunctionStraight = 2,
  TurnLeft = 3,
  TurnRight = 4
};

enum class Relationship {
  NotRelevant = 0,
  SameDir = 1,
  OnComing = 2,
  Merge = 3,
  Cross = 4,
  Static = 5,
  Unknown = 6
};

enum class Bearing {
  Unknown = 0,
  Overlap = 1,
  Left = 2,
  Right = 3,
};

struct ObjectDecisionParam {
  double yield_time_factor = 1.0;
  double yield_time_additional_buffer = 0.0;
  double pass_time_factor = 1.0;
  double pass_time_additional_buffer = 0.0;
  double dp_follow_lead_ratio = 0.5;
  bool enable_interact_first_point_decision = true;
  bool enable_interact_last_point_decision = true;
  double interact_av_decel_factor = 1.0;
  double interact_obj_follow_dist_buffer = 0.0;
  bool geometry_theory_use_particular_follow_distance = false;
  double particular_agent_follow_distance = 0.0;
  double geometry_theory_av_follow_distance = 4.0;
  bool enable_add_collision_risk_cost_for_dp = false;
  bool enable_left_turn_overtake_cost = false;
  double agent_reaction_time = 0.0;
};

struct ObjectScenarioInfo {
  InteractionZone interaction_zone = InteractionZone::Unknown;
  Relationship relationship = Relationship::Unknown;
  bool is_av_completely_in_obj_fov = false;
  Bearing bearing = Bearing::Unknown;
  double delta_heading;
  ObjectSlInfo obj_sl_info;
  double current_vl = 0.0;
  ObjectDecisionParam obj_decision_param;
};

struct StBoundaryPoints {
  std::vector<StPoint> lower_points;
  std::vector<StPoint> upper_points;
  std::vector<VtPoint> speed_points;
  std::vector<OverlapInfo> overlap_infos;
  std::vector<NearestSlPoint> nearest_sl_points;
  StBoundaryProto::ProtectionType protection_type =
      StBoundaryProto::NON_PROTECTIVE;
  void Reserve(size_t num) {
    lower_points.reserve(num);
    upper_points.reserve(num);
    speed_points.reserve(num);
    overlap_infos.reserve(num);
    nearest_sl_points.reserve(num);
  }
};

class StBoundary : public Polygon2d {
 public:
  explicit StBoundary(const Box2d& box) = delete;
  explicit StBoundary(std::vector<Vec2d> points) = delete;
  StBoundary(const StBoundary&) = delete;
  StBoundary(StBoundary&&) = delete;
  StBoundary& operator=(const StBoundary&) const = delete;
  StBoundary& operator=(StBoundary&&) const = delete;

  static StBoundaryRef CreateInstance(
      const StBoundaryPoints& st_boundary_points,
      StBoundaryProto::ObjectType object_type, std::string id,
      double probability, bool is_stationary,
      StBoundaryProto::ProtectionType protection_type, bool is_large_vehicle,
      bool is_traffic_light,
      TrajectoryPointWithAcceleration pose = TrajectoryPointWithAcceleration(),
      ObjectSlInfo obj_sl_info = ObjectSlInfo(),
      ObjectScenarioInfo obj_scenario_info = ObjectScenarioInfo());

  static StBoundaryRef CopyInstance(const StBoundary& st_boundary);

  virtual ~StBoundary() = default;

  bool IsEmpty() const { return lower_points_.empty(); }

  bool IsPointInBoundary(const StPoint& st_point) const;

  StPoint upper_left_point() const {
    CHECK(!upper_points_.empty());
    return upper_points_.front();
  }

  StPoint upper_right_point() const {
    CHECK(!upper_points_.empty());
    return upper_points_.back();
  }

  StPoint bottom_left_point() const {
    CHECK(!lower_points_.empty());
    return lower_points_.front();
  }

  StPoint bottom_right_point() const {
    CHECK(!lower_points_.empty());
    return lower_points_.back();
  }

  void ExpandByT(double left, double right);

  StBoundarySourceTypeProto::Type source_type() const { return source_type_; }
  void set_source_type(StBoundarySourceTypeProto::Type source_type) {
    source_type_ = source_type;
  }
  static std::string SourceTypeName(
      StBoundarySourceTypeProto::Type source_type);

  static StBoundarySourceTypeProto::Type ObjectTypeToSourceType(
      StBoundaryProto::ObjectType object_type);

  StBoundaryProto::ObjectType object_type() const { return object_type_; }

  void set_object_type(StBoundaryProto::ObjectType type) {
    CHECK(type != StBoundaryProto::UNKNOWN_OBJECT);

    object_type_ = type;
    set_source_type(ObjectTypeToSourceType(type));
  }

  const std::string& id() const { return id_; }
  void set_id(const std::string& id);
  const std::optional<std::string>& traj_id() const { return traj_id_; }
  const std::optional<std::string>& object_id() const { return object_id_; }

  double probability() const { return probability_; }
  void set_probability(double probability) {
    DCHECK_GE(probability, 0.0);
    DCHECK_LE(probability, 1.0);
    probability_ = probability;
  }

  bool is_stationary() const { return is_stationary_; }
  void set_is_stationary(bool is_stationary) { is_stationary_ = is_stationary; }

  bool is_protective() const {
    return protection_type_ != StBoundaryProto::NON_PROTECTIVE;
  }

  StBoundaryProto::ProtectionType protection_type() const {
    return protection_type_;
  }
  void set_protection_type(StBoundaryProto::ProtectionType protection_type) {
    protection_type_ = protection_type;
  }

  std::optional<std::string> protected_st_boundary_id() const {
    return protected_st_boundary_id_;
  }
  void set_protected_st_boundary_id(std::string protected_st_boundary_id) {
    protected_st_boundary_id_ = std::move(protected_st_boundary_id);
  }

  bool is_large_vehicle() const { return is_large_vehicle_; }
  bool is_traffic_light() const { return is_traffic_light_; }
  void set_is_large_vehicle_vehicle(bool is_large_vehicle) {
    is_large_vehicle_ = is_large_vehicle;
  }

  void set_obj_sl_info(ObjectSlInfo obj_sl_info) { obj_sl_info_ = obj_sl_info; }

  void set_obj_scenario_info(ObjectScenarioInfo obj_scenario_info) {
    obj_scenario_info_ = obj_scenario_info;
  }

  std::optional<std::pair<double, double>> GetBoundarySRange(
      double curr_time) const;

  std::optional<double> GetStBoundarySpeedAtT(double t) const;

  bool GetLowerPointsIndexRange(double t, int* left, int* right) const;

  bool GetUpperPointsIndexRange(double t, int* left, int* right) const;

  bool GetSpeedPointsIndexRange(double t, int* left, int* right) const;

  double min_s() const { return min_s_; }
  double min_t() const { return min_t_; }
  double max_s() const { return max_s_; }
  double max_t() const { return max_t_; }

  ObjectSlInfo obj_sl_info() const { return obj_sl_info_; }

  ObjectScenarioInfo obj_scenario_info() const { return obj_scenario_info_; }
  ObjectScenarioInfo& mutable_obj_scenario_info() { return obj_scenario_info_; }
  TrajectoryPointWithAcceleration obj_pose_info() const { return pose_; }

  const std::vector<StPoint>& upper_points() const { return upper_points_; }
  const std::vector<StPoint>& lower_points() const { return lower_points_; }
  const std::vector<VtPoint>& speed_points() const { return speed_points_; }
  const std::vector<OverlapInfo>& overlap_infos() const {
    return overlap_infos_;
  }
  const std::vector<NearestSlPoint>& nearest_sl_points() const {
    return nearest_sl_points_;
  }

  void set_speed_points(std::vector<VtPoint> speed_points) {
    speed_points_ = std::move(speed_points);
  }

  const std::optional<StOverlapMetaProto>& overlap_meta() const {
    return overlap_meta_;
  }
  void set_overlap_meta(StOverlapMetaProto meta) {
    overlap_meta_ = std::move(meta);
  }

  void Init(std::vector<std::pair<StPoint, StPoint>> point_pairs);

  std::string DebugString() const;

  static std::optional<std::string> RecoverObjectId(
      const std::string& st_boundary_id,
      StBoundarySourceTypeProto::Type source_type);

  static std::optional<std::string> RecoverTrajId(
      const std::string& st_boundary_id,
      StBoundarySourceTypeProto::Type source_type);

 private:
  StBoundary(std::vector<std::pair<StPoint, StPoint>> point_pairs,
             std::vector<VtPoint> speed_points,
             std::vector<OverlapInfo> overlap_infos,
             std::vector<NearestSlPoint> nearest_sl_points,
             StBoundaryProto::ObjectType object_type, std::string id,
             double probability, bool is_stationary,
             StBoundaryProto::ProtectionType protection_type,
             bool is_large_vehicle, bool is_traffic_light,
             std::optional<StOverlapMetaProto> overlap_meta,
             std::optional<std::string> protected_st_boundary_id,
             ObjectSlInfo obj_sl_info, ObjectScenarioInfo obj_scenario_info,
             TrajectoryPointWithAcceleration pose);

  bool IsValid(
      const std::vector<std::pair<StPoint, StPoint>>& point_pairs) const;

  template <typename T>
  bool QueryIndexRange(const std::vector<T>& points, double t, int* left,
                       int* right) const;

 private:
  StBoundaryProto::ObjectType object_type_ = StBoundaryProto::UNKNOWN_OBJECT;
  StBoundarySourceTypeProto::Type source_type_ =
      StBoundarySourceTypeProto::UNKNOWN;
  StBoundaryProto::ProtectionType protection_type_ =
      StBoundaryProto::NON_PROTECTIVE;

  std::vector<StPoint> lower_points_;
  std::vector<StPoint> upper_points_;

  std::vector<VtPoint> speed_points_;

  std::vector<OverlapInfo> overlap_infos_;

  std::vector<NearestSlPoint> nearest_sl_points_;

  std::string id_;

  std::optional<std::string> traj_id_;
  std::optional<std::string> object_id_;

  double probability_ = 0.0;
  bool is_stationary_ = false;
  bool is_large_vehicle_ = false;
  bool is_traffic_light_ = false;

  double min_s_ = std::numeric_limits<double>::max();
  double max_s_ = std::numeric_limits<double>::lowest();
  double min_t_ = std::numeric_limits<double>::max();
  double max_t_ = std::numeric_limits<double>::lowest();

  std::optional<StOverlapMetaProto> overlap_meta_;
  std::optional<std::string> protected_st_boundary_id_ = std::nullopt;
  ObjectSlInfo obj_sl_info_;
  ObjectScenarioInfo obj_scenario_info_;
  TrajectoryPointWithAcceleration pose_;
};

std::string GetStBoundaryIntegrationId(const StBoundary& st_boundary);

StBoundaryProto::ObjectType ToStBoundaryObjectType(ObjectType type);

template <typename T>
bool StBoundary::QueryIndexRange(const std::vector<T>& points, double t,
                                 int* left, int* right) const {
  CHECK_NOTNULL(left);
  CHECK_NOTNULL(right);
  CHECK_GT(points.size(), 1);
  if (t < points.front().t() || t > points.back().t()) {
    LOG(WARNING) << "t is out of range. t = " << t
                 << " range: " << points.front().t() << ", "
                 << points.back().t();
    return false;
  }
  const auto it =
      std::lower_bound(points.begin(), points.end(), t,
                       [](const T& p, double t) { return p.t() < t; });
  const int index = std::distance(points.begin(), it);
  if (index == 0) {
    *left = 0;
    *right = *left + 1;
  } else if (it == points.end()) {
    *left = points.size() - 1;
    *right = *left - 1;
  } else {
    *left = index - 1;
    *right = index;
  }
  return true;
}

}  // namespace e2e_noa::planning

#endif
