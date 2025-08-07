#include "apps/planning/src/common/local_route/local_route.h"

#include "angle.h"
#include "apps/planning/src/common/conversion.h"
#include "cartesian_frenet_conversion.h"

namespace zark {
namespace planning {

using ::common::SLPoint;
using ::math::Box2d;
using ::math::CartesianFrenetConverter;
using ::math::Vec2d;
using zark::planning::common::VehicleConfigHelper;

LocalRoute::LocalRoute(const LocalRouteConfig& config) : config_(config) {}

LocalRoute::LocalRoute(const std::vector<LocalRoutePoint>& local_route_points,
                       const LocalRouteConfig& config)
    : on_current_route_(false),
      is_target_route_(false),
      s_destination_(kMaxForwardDistacne),
      left_route_(nullptr),
      right_route_(nullptr),
      route_points_(local_route_points),
      map_path_info_(std::move(std::vector<hdmap::MapPathPoint>(
          local_route_points.begin(), local_route_points.end()))),
      lanes_(),
      init_frenet_point_(),
      route_lc_info_(),
      config_(config) {
  CHECK_EQ(static_cast<size_t>(map_path_info_.num_points()),
           route_points_.size());
}

LocalRoute::LocalRoute(const std::vector<LocalRoutePoint>& points,
                       const LocalRouteConfig& config, const bool is_side_line)
    : route_points_(points),
      map_path_info_(std::move(std::vector<hdmap::MapPathPoint>(points.begin(),
                                                                points.end())),
                     is_side_line),
      config_(config) {
  CHECK_EQ(static_cast<size_t>(map_path_info_.num_points()),
           route_points_.size());
}

LocalRoute::LocalRoute(const hdmap::RouteSegments& segments,
                       const LocalRouteConfig& config)
    : on_current_route_(false),
      is_target_route_(false),
      s_destination_(kMaxForwardDistacne),
      left_route_(nullptr),
      right_route_(nullptr),
      map_path_info_(segments),
      lanes_(segments),
      init_frenet_point_(),
      route_lc_info_(),
      config_(config) {
  for (const auto& point : map_path_info_.path_points()) {
    DCHECK(!point.lane_waypoints().empty());
    const auto& lane_waypoint = point.lane_waypoints()[0];
    route_points_.emplace_back(
        hdmap::MapPathPoint(point, point.heading(), lane_waypoint), 0.0, 0.0);
  }
  CHECK_EQ(static_cast<size_t>(map_path_info_.num_points()),
           route_points_.size());
}

bool LocalRoute::Init(const common::VehicleState& vehicle_state,
                      const ::common::TrajectoryPoint& planning_start_point) {
  const auto& param = VehicleConfigHelper::GetConfig().vehicle_param();
  Box2d box = CalculateCenterBoxFromCGPose(
      planning_start_point.path_point().x(),
      planning_start_point.path_point().y(),
      planning_start_point.path_point().theta(), param);
  SLBoundary adc_sl_boundary;
  if (!GetSLBoundary(box, adc_sl_boundary)) {
    AERROR << "Failed to get ADC boundary from box: " << box.DebugString();
    return false;
  }

  if (adc_sl_boundary.end_s() < 0 || adc_sl_boundary.start_s() > Length()) {
    AWARN << "Vehicle SL " << adc_sl_boundary.ShortDebugString()
          << " is not on local route:[0, " << Length() << "]";
  }
  static constexpr double kOutOfReferenceLineL = 10.0;  // in meters
  if (adc_sl_boundary.start_l() > kOutOfReferenceLineL ||
      adc_sl_boundary.end_l() < -kOutOfReferenceLineL) {
    AERROR << "Ego vehicle is too far away from local route.";
    return false;
  }
  on_current_route_ = lanes_.GetSegmentType() ==
                              hdmap::RouteSegments::SegmentType::CurrentSegment
                          ? true
                          : false;

  return true;
}

bool LocalRoute::GetLaneWidth(const double s, double& lane_left_width,
                              double& lane_right_width) const {
  if (map_path_info_.path_points().empty()) {
    return false;
  }

  if (!map_path_info_.GetLaneWidth(s, &lane_left_width, &lane_right_width)) {
    return false;
  }
  return true;
}

bool LocalRoute::GetRoadWidth(const double s, double& road_left_width,
                              double& road_right_width) const {
  if (map_path_info_.path_points().empty()) {
    return false;
  }
  return map_path_info_.GetRoadWidth(s, &road_left_width, &road_right_width);
}

bool LocalRoute::GetDistanceToCurb(const double s, double& road_left_width,
                                   double& road_right_width) const {
  if (map_path_info_.path_points().empty()) {
    return false;
  }

  if (map_path_info_.lane_segments().empty()) {
    return false;
  }

  hdmap_new::RoadBoundary::RoadBoundaryType::Type left_boundary_type =
      hdmap_new::RoadBoundary::RoadBoundaryType::Type::
          RoadBoundary_RoadBoundaryType_Type_ROAD_BOUNDARY_FENCE;
  hdmap_new::RoadBoundary::RoadBoundaryType::Type right_boundary_type =
      hdmap_new::RoadBoundary::RoadBoundaryType::Type::
          RoadBoundary_RoadBoundaryType_Type_ROAD_BOUNDARY_FENCE;
  GetRoadBoundaryTypeFromS(s, left_boundary_type, right_boundary_type);
  if (!map_path_info_.GetRoadWidth(s, &road_left_width, &road_right_width)) {
    return false;
  }

  const double kLaneWidth = 3.5;  // [m]
  if (left_boundary_type ==
      hdmap_new::RoadBoundary_RoadBoundaryType_Type_ROAD_BOUNDARY_VIRTUAL) {
    road_left_width = road_left_width + kLaneWidth;
  }
  if (right_boundary_type ==
      hdmap_new::RoadBoundary_RoadBoundaryType_Type_ROAD_BOUNDARY_VIRTUAL) {
    road_right_width = road_right_width + kLaneWidth;
  }
  return true;
}

const std::vector<LocalRoute::LineBoundary> LocalRoute::GetLaneBoundary(
    const double s, const double distance) const {
  std::vector<LineBoundary> bound_types;
  if (map_path_info_.path_points().empty()) {
    AWARN << "NO map_path..";
    return bound_types;
  }
  if (s < 0 || distance < 0) {
    AWARN << " GetLaneBoundary: input error..";
    return bound_types;
  }
  if (map_path_info_.lane_segments().empty()) {
    AWARN << " GetLaneBoundary: lane segments empty..";
    return bound_types;
  }
  double end_s = s + distance;
  std::vector<hdmap::LaneSegment> lane_segments =
      map_path_info_.GetLaneSegments(s, end_s);
  for (auto& seg : lane_segments) {
    LineBoundary line_boundary;
    // note: lane_boundary multi type;
    auto left_bound_info = hdmap::GetLaneBoundaryById(
        hdmap::HDMapUtil::BaseMapPtr(), seg.lane->lane().left_boundary_id());
    auto right_bound_info = hdmap::GetLaneBoundaryById(
        hdmap::HDMapUtil::BaseMapPtr(), seg.lane->lane().right_boundary_id());

    if (left_bound_info &&
        left_bound_info->lane_boundary().boundary_attr().size() > 0) {
      line_boundary.left_type =
          left_bound_info->lane_boundary().boundary_attr().at(0).type();
    }
    if (right_bound_info &&
        right_bound_info->lane_boundary().boundary_attr().size() > 0) {
      line_boundary.right_type =
          right_bound_info->lane_boundary().boundary_attr().at(0).type();
    }
    if (lane_segments.size() == 1) {
      line_boundary.start_s = s;
      line_boundary.end_s = end_s;
    } else if (s >= seg.start_s && s <= seg.end_s) {
      line_boundary.start_s = s;
      line_boundary.end_s = seg.end_s;
    } else if (end_s >= seg.start_s && end_s < seg.end_s) {
      line_boundary.start_s = seg.start_s;
      line_boundary.end_s = end_s;
    } else {
      line_boundary.start_s = seg.start_s;
      line_boundary.end_s = seg.end_s;
    }
    bound_types.emplace_back(line_boundary);
  }
  return bound_types;
}

const double LocalRoute::GetSpeedLimitFromS(const double s) const {
  const auto& map_path_point = GetLocalRoutePoint(s);

  double speed_limit = config_.local_route_speed_limit;
  bool speed_limit_found = false;
  const double kSpeedMph2Kph = 3.6;
  for (const auto& lane_waypoint : map_path_point.lane_waypoints()) {
    if (lane_waypoint.lane == nullptr) {
      continue;
    }
    speed_limit_found = true;
    speed_limit = std::fmin(
        lane_waypoint.lane->lane().max_speed_limit_kmph() / kSpeedMph2Kph,
        speed_limit);
    break;
  }

  if (!speed_limit_found) {
    speed_limit = config_.min_local_route_speed_limit;
  }

  return speed_limit;
}

const hdmap_new::Road_RoadClass LocalRoute::GetRoadClassFromS(
    const double s) const {
  hdmap_new::Road_RoadClass road_class =
      hdmap_new::Road_RoadClass::Road_RoadClass_RC_UNKNOWN;
  auto lane_index = map_path_info_.GetLaneIndexFromS(s);
  if (lane_index.offset + ::math::kMathEpsilon >=
      map_path_info_.lane_segments()[lane_index.id].Length()) {
    lane_index.id += 1;
  }
  const int num_lanes = static_cast<int>(map_path_info_.lane_segments().size());
  if (lane_index.id >= num_lanes) {
    return road_class;
  }
  auto road_id =
      map_path_info_.lane_segments()[lane_index.id].lane->lane().road_id();
  auto road = hdmap::GetRoadById(hdmap::HDMapUtil::BaseMapPtr(), road_id);
  if (!road) {
    return road_class;
  }
  return road->road().road_class();
}

void LocalRoute::GetRoadBoundaryTypeFromS(
    const double s,
    hdmap_new::RoadBoundary::RoadBoundaryType::Type& left_boundary_type,
    hdmap_new::RoadBoundary::RoadBoundaryType::Type& right_boundary_type)
    const {
  auto lane_index = map_path_info_.GetLaneIndexFromS(s);
  if (lane_index.offset + ::math::kMathEpsilon >=
      map_path_info_.lane_segments()[lane_index.id].Length()) {
    lane_index.id += 1;
  }
  const int num_lanes = static_cast<int>(map_path_info_.lane_segments().size());
  if (lane_index.id >= num_lanes) {
    return;
  }
  auto road_section_id = map_path_info_.lane_segments()[lane_index.id]
                             .lane->lane()
                             .road_section_id();
  auto road_section = hdmap::GetRoadSectionById(hdmap::HDMapUtil::BaseMapPtr(),
                                                road_section_id);
  if (!road_section) {
    return;
  }
  auto road_left_boundary_id = road_section->road_section().left_boundary_id();
  auto road_right_boundary_id =
      road_section->road_section().right_boundary_id();
  if (map_path_info_.lane_segments()[0]
          .lane->lane()
          .center_line()
          .offset_cm()
          .size() < 1) {
    return;
  }
  double offset = map_path_info_.lane_segments()[0]
                      .lane->lane()
                      .center_line()
                      .offset_cm()[0];
  double s_offset = s + offset / 100.0;
  auto road_left_boundary = hdmap::GetRoadBoundaryById(
      hdmap::HDMapUtil::BaseMapPtr(), road_left_boundary_id);
  auto road_right_boundary = hdmap::GetRoadBoundaryById(
      hdmap::HDMapUtil::BaseMapPtr(), road_right_boundary_id);
  const double kCmToM = 100.0;
  if (road_left_boundary) {
    for (auto boundary_type :
         road_left_boundary->road_boundary().boundary_attr()) {
      if (s_offset >=
              boundary_type.distance_info().start_offset_cm() / kCmToM &&
          s_offset < boundary_type.distance_info().end_offset_cm() / kCmToM) {
        left_boundary_type = boundary_type.type();
        break;
      }
    }
  }
  if (road_right_boundary) {
    for (auto boundary_type :
         road_right_boundary->road_boundary().boundary_attr()) {
      if (s_offset >=
              boundary_type.distance_info().start_offset_cm() / kCmToM &&
          s_offset < boundary_type.distance_info().end_offset_cm() / kCmToM) {
        right_boundary_type = boundary_type.type();
        break;
      }
    }
  }

  return;
}

const hdmap_new::Lane_LaneType LocalRoute::GetLaneTypeFromS(
    const double s) const {
  hdmap_new::Lane_LaneType lane_type =
      hdmap_new::Lane_LaneType_LANE_TYPE_UNKNOWN;
  auto lane_index = map_path_info_.GetLaneIndexFromS(s);
  if (lane_index.offset + ::math::kMathEpsilon >=
      map_path_info_.lane_segments()[lane_index.id].Length()) {
    lane_index.id += 1;
  }
  const int num_lanes = static_cast<int>(map_path_info_.lane_segments().size());
  if (lane_index.id >= num_lanes) {
    return lane_type;
  }
  lane_type = static_cast<hdmap_new::Lane_LaneType>(
      map_path_info_.lane_segments()[lane_index.id].lane->lane().type());
  return lane_type;
}

const double LocalRoute::GetCurvatureFromS(const double s) const {
  LocalRoutePoint route_point = GetLocalRoutePoint(s);
  return route_point.kappa();
}

const LocalRoutePoint LocalRoute::GetLocalRoutePoint(const double s) const {
  const double kDistanceEpsilon = 1.0e-2;
  const auto& accumulated_s = map_path_info_.accumulated_s();
  if (s < accumulated_s.front() - kDistanceEpsilon) {
    ADEBUG << "The requested s: " << s << " < 0.";
    return route_points_.front();
  }
  if (s > accumulated_s.back() + kDistanceEpsilon) {
    ADEBUG << "The requested s: " << s
           << " > local route length: " << accumulated_s.back();
    return route_points_.back();
  }

  auto interpolate_index = map_path_info_.GetIndexFromS(s);

  size_t index = interpolate_index.id;
  size_t next_index = index + 1;
  if (next_index >= route_points_.size()) {
    next_index = route_points_.size() - 1;
  }

  const auto& p0 = route_points_[index];
  const auto& p1 = route_points_[next_index];

  const double s0 = accumulated_s[index];
  const double s1 = accumulated_s[next_index];
  return InterpolateWithMatchedIndex(p0, s0, p1, s1, interpolate_index);
}

const LocalRoutePoint LocalRoute::InterpolateWithMatchedIndex(
    const LocalRoutePoint& p0, const double s0, const LocalRoutePoint& p1,
    const double s1, const hdmap::InterpolatedIndex& index) const {
  const double kLengthEpsilon = 1.0e-6;
  if (std::fabs(s0 - s1) < ::math::kMathEpsilon) {
    return p0;
  }
  double s = s0 + index.offset;
  DCHECK_LE(s0 - kLengthEpsilon, s)
      << "s: " << s << " is less than s0 : " << s0;
  DCHECK_LE(s, s1 + kLengthEpsilon)
      << "s: " << s << " is larger than s1: " << s1;

  auto map_path_point = map_path_info_.GetSmoothPoint(index);
  const double kappa = ::math::lerp(p0.kappa(), s0, p1.kappa(), s1, s);
  const double dkappa = ::math::lerp(p0.dkappa(), s0, p1.dkappa(), s1, s);

  return LocalRoutePoint(map_path_point, kappa, dkappa);
}

std::vector<LocalRoutePoint> LocalRoute::GetLocalRoutePoints(
    double start_s, double end_s) const {
  if (start_s < 0.0) {
    start_s = 0.0;
  }
  if (end_s > Length()) {
    end_s = Length();
  }
  std::vector<LocalRoutePoint> route_points;
  auto start_index = GetNearestPointIndex(start_s);
  auto end_index = GetNearestPointIndex(end_s);
  if (start_index < end_index) {
    route_points.assign(route_points_.begin() + start_index,
                        route_points_.begin() + end_index);
  }
  return route_points;
}

bool LocalRoute::IsIrrelevantObstacle(const SLBoundary& adc_sl_boundary,
                                      const Obstacle& obstacle) {
  // if adc is on the road, and obstacle behind adc, ignore
  const auto& obstacle_boundary = obstacle.PerceptionSLBoundary();
  if (obstacle_boundary.end_s() > Length()) {
    return true;
  }
  if (on_current_route_ && !IsChangeLanePath() &&
      obstacle_boundary.end_s() < adc_sl_boundary.end_s() &&
      (IsOnLane(obstacle_boundary) ||
       obstacle_boundary.end_s() < 0.0)) {  // if obstacle is far backward
    return true;
  }
  return false;
}

bool LocalRoute::AddObstaclesSLBoundary(
    const std::vector<const Obstacle*>& obstacles) {
  for (const auto* obstacle : obstacles) {
    if (!obstacle) {
      AERROR << "The provided obstacle is empty";
      continue;
    }
    SLBoundary perception_sl;
    if (!GetSLBoundary(obstacle->PerceptionBoundingBox(), perception_sl)) {
      AERROR << "Failed to get sl boundary for obstacle: " << obstacle->Id();
      continue;
    } else {
      obstacle_sl_boundaries_.Add(obstacle, perception_sl);
    }
  }
  return true;
}

bool LocalRoute::AddObstacleSLBoundary(const Obstacle* obstacle) {
  if (!obstacle) {
    AERROR << "The provided obstacle is empty";
    return false;
  }
  SLBoundary perception_sl;
  if (!GetSLBoundary(obstacle->PerceptionBoundingBox(), perception_sl)) {
    AERROR << "Failed to get sl boundary for obstacle: " << obstacle->Id();
    return false;
  } else {
    obstacle_sl_boundaries_.Add(obstacle, perception_sl);
  }

  return true;
}

bool LocalRoute::GetSLBoundary(const ::math::Box2d& box,
                               SLBoundary& sl_boundary) const {
  double start_s(std::numeric_limits<double>::max());
  double end_s(std::numeric_limits<double>::lowest());
  double start_l(std::numeric_limits<double>::max());
  double end_l(std::numeric_limits<double>::lowest());
  std::vector<::math::Vec2d> corners;
  box.GetAllCorners(&corners);

  // The order must be counter-clockwise
  std::vector<SLPoint> sl_corners;
  for (const auto& point : corners) {
    SLPoint sl_point;
    if (!XYToSL(point, sl_point)) {
      AERROR << "Failed to get projection for point: " << point.DebugString()
             << " on local route.";
      return false;
    }
    sl_corners.push_back(std::move(sl_point));
  }

  for (size_t i = 0; i < corners.size(); ++i) {
    auto index0 = i;
    auto index1 = (i + 1) % corners.size();
    const auto& p0 = corners[index0];
    const auto& p1 = corners[index1];

    const auto p_mid = (p0 + p1) * 0.5;
    SLPoint sl_point_mid;
    if (!XYToSL(p_mid, sl_point_mid)) {
      AERROR << "Failed to get projection for point: " << p_mid.DebugString()
             << " on local route.";
      return false;
    }

    Vec2d v0(sl_corners[index1].s() - sl_corners[index0].s(),
             sl_corners[index1].l() - sl_corners[index0].l());

    Vec2d v1(sl_point_mid.s() - sl_corners[index0].s(),
             sl_point_mid.l() - sl_corners[index0].l());

    *sl_boundary.add_boundary_point() = sl_corners[index0];

    // sl_point is outside of polygon; add to the vertex list
    if (v0.CrossProd(v1) < 0.0) {
      *sl_boundary.add_boundary_point() = sl_point_mid;
    }
  }

  for (const auto& sl_point : sl_boundary.boundary_point()) {
    start_s = std::fmin(start_s, sl_point.s());
    end_s = std::fmax(end_s, sl_point.s());
    start_l = std::fmin(start_l, sl_point.l());
    end_l = std::fmax(end_l, sl_point.l());
  }

  sl_boundary.set_start_s(start_s);
  sl_boundary.set_end_s(end_s);
  sl_boundary.set_start_l(start_l);
  sl_boundary.set_end_l(end_l);
  return true;
}

bool LocalRoute::XYToSL(const Vec2d& xy_point, SLPoint& sl_point) const {
  double s = 0.0;
  double l = 0.0;
  if (!map_path_info_.GetProjection(xy_point, &s, &l)) {
    AERROR << "Cannot get nearest point from path.";
    return false;
  }
  sl_point.set_s(s);
  sl_point.set_l(l);
  return true;
}

bool LocalRoute::SLToXY(const SLPoint& sl_point, Vec2d& xy_point) const {
  if (map_path_info_.num_points() < 2) {
    AERROR << "The local route has too few points.";
    return false;
  }

  const auto matched_point = GetLocalRoutePoint(sl_point.s());
  const auto angle = ::math::Angle16::from_rad(matched_point.heading());
  xy_point.set_x(matched_point.x() - ::math::sin(angle) * sl_point.l());
  xy_point.set_y(matched_point.y() + ::math::cos(angle) * sl_point.l());
  return true;
}

bool LocalRoute::SLToXY(const ::common::SLPoint& sl_point,
                        ::math::Vec2d& xy_point,
                        LocalRoutePoint& matched_point) const {
  if (map_path_info_.num_points() < 2) {
    AERROR << "The local route has too few points.";
    return false;
  }

  matched_point = GetLocalRoutePoint(sl_point.s());
  const auto angle = ::math::Angle16::from_rad(matched_point.heading());
  xy_point.set_x(matched_point.x() - ::math::sin(angle) * sl_point.l());
  xy_point.set_y(matched_point.y() + ::math::cos(angle) * sl_point.l());
  return true;
}

FrenetPoint LocalRoute::ToFrenetFrame(const TrajectoryPoint& traj_point) const {
  ACHECK(!route_points_.empty());

  SLPoint sl_point;
  XYToSL(Vec2d(traj_point.path_point().x(), traj_point.path_point().y()),
         sl_point);

  FrenetPoint frenet_pt;
  frenet_pt.t = 0.0;
  LocalRoutePoint local_route_point = GetLocalRoutePoint(sl_point.s());
  CartesianFrenetConverter::cartesian_to_frenet(
      sl_point.s(), local_route_point.x(), local_route_point.y(),
      local_route_point.heading(), local_route_point.kappa(),
      local_route_point.dkappa(), traj_point.path_point().x(),
      traj_point.path_point().y(), traj_point.v(), traj_point.a(),
      traj_point.path_point().theta(), traj_point.path_point().kappa(),
      &frenet_pt.s, &frenet_pt.l);

  return frenet_pt;
}

FrenetPoint LocalRoute::ToFrenetFrame(const CorridorPoint& traj_point) const {
  ACHECK(!route_points_.empty());

  SLPoint sl_point;
  XYToSL(traj_point.xy_ref, sl_point);

  FrenetPoint frenet_pt;
  frenet_pt.t = 0.0;
  LocalRoutePoint local_route_point = GetLocalRoutePoint(sl_point.s());
  CartesianFrenetConverter::cartesian_to_frenet(
      sl_point.s(), local_route_point.x(), local_route_point.y(),
      local_route_point.heading(), local_route_point.kappa(),
      local_route_point.dkappa(), traj_point.xy_ref.x(), traj_point.xy_ref.y(),
      traj_point.v, traj_point.a, traj_point.theta, traj_point.kappa,
      &frenet_pt.s, &frenet_pt.l);

  return frenet_pt;
}

const double LocalRoute::GetDrivingWidth(const SLBoundary& sl_boundary) const {
  double lane_left_width = 0.0;
  double lane_right_width = 0.0;
  GetLaneWidth(sl_boundary.start_s(), lane_left_width, lane_right_width);

  double driving_width = std::max(lane_left_width - sl_boundary.end_l(),
                                  lane_right_width + sl_boundary.start_l());
  driving_width = std::min(lane_left_width + lane_right_width, driving_width);
  ADEBUG << "Driving width [" << driving_width << "].";
  return driving_width;
}

bool LocalRoute::IsOnLane(const SLBoundary& sl_boundary) const {
  if (sl_boundary.end_s() < 0 || sl_boundary.start_s() > Length()) {
    return false;
  }
  double middle_s = (sl_boundary.start_s() + sl_boundary.end_s()) / 2.0;
  double lane_left_width = 0.0;
  double lane_right_width = 0.0;
  map_path_info_.GetLaneWidth(middle_s, &lane_left_width, &lane_right_width);
  return sl_boundary.start_l() <= lane_left_width &&
         sl_boundary.end_l() >= -lane_right_width;
}

const size_t LocalRoute::GetNearestPointIndex(const double s) const {
  const double kDisThreshold = 1.0e-2;
  const auto& accumulated_s = map_path_info_.accumulated_s();
  if (s < accumulated_s.front() - kDisThreshold) {
    ADEBUG << "The requested s: " << s << " < 0.";
    return 0;
  }
  if (s > accumulated_s.back() + kDisThreshold) {
    ADEBUG << "The requested s: " << s << " > local_route length "
           << accumulated_s.back();
    return route_points_.size() - 1;
  }
  auto it_lower =
      std::lower_bound(accumulated_s.begin(), accumulated_s.end(), s);
  return std::distance(accumulated_s.begin(), it_lower);
}

bool LocalRoute::Segment(const double s, const double look_backward,
                         const double look_forward) {
  const auto& accumulated_s = map_path_info_.accumulated_s();

  // inclusive
  auto start_index =
      std::distance(accumulated_s.begin(),
                    std::lower_bound(accumulated_s.begin(), accumulated_s.end(),
                                     s - look_backward));

  // exclusive
  auto end_index =
      std::distance(accumulated_s.begin(),
                    std::upper_bound(accumulated_s.begin(), accumulated_s.end(),
                                     s + look_forward));

  if (end_index - start_index < 2) {
    AERROR << "Too few route points after shrinking.";
    return false;
  }

  route_points_ = std::vector<LocalRoutePoint>(
      route_points_.begin() + start_index, route_points_.begin() + end_index);

  map_path_info_ = hdmap::Path(std::vector<hdmap::MapPathPoint>(
      route_points_.begin(), route_points_.end()));
  return true;
}

bool LocalRoute::Stitch(const LocalRoute& other) {
  if (other.RoutePoints().empty()) {
    AWARN << "The other reference line is empty.";
    return true;
  }
  auto first_point = route_points_.front();
  Point2D point_trans1;
  point_trans1.set_x(first_point.x());
  point_trans1.set_y(first_point.y());
  SLPoint first_sl;
  if (!other.XYToSL(Vec2d(point_trans1.x(), point_trans1.y()), first_sl)) {
    AWARN << "Failed to project the first point to the other reference line.";
    return false;
  }
  bool first_join = first_sl.s() > 0 && first_sl.s() < other.Length();

  auto last_point = route_points_.back();
  Point2D point_trans2;
  point_trans2.set_x(last_point.x());
  point_trans2.set_y(last_point.y());
  SLPoint last_sl;
  if (!other.XYToSL(Vec2d(point_trans2.x(), point_trans2.y()), last_sl)) {
    AWARN << "Failed to project the last point to the other reference line.";
    return false;
  }
  bool last_join = last_sl.s() > 0 && last_sl.s() < other.Length();

  if (!first_join && !last_join) {
    AERROR << "These reference lines are not connected.";
    return false;
  }

  const auto& accumulated_s = other.MapPathInfo().accumulated_s();
  const auto& other_points = other.RoutePoints();
  auto lower = accumulated_s.begin();
  static constexpr double kStitchingError = 1e-1;
  if (first_join) {
    if (first_sl.l() > kStitchingError) {
      AERROR << "lateral stitching error on first join of reference line too "
                "big, stitching fails";
      return false;
    }
    lower = std::lower_bound(accumulated_s.begin(), accumulated_s.end(),
                             first_sl.s());
    size_t start_i = std::distance(accumulated_s.begin(), lower);
    route_points_.insert(route_points_.begin(), other_points.begin(),
                         other_points.begin() + start_i);
  }
  if (last_join) {
    if (last_sl.l() > kStitchingError) {
      AERROR << "lateral stitching error on first join of reference line too "
                "big, stitching fails";
      return false;
    }
    auto upper = std::upper_bound(lower, accumulated_s.end(), last_sl.s());
    auto end_i = std::distance(accumulated_s.begin(), upper);
    route_points_.insert(route_points_.end(), other_points.begin() + end_i,
                         other_points.end());
  }
  map_path_info_ = hdmap::Path(std::move(std::vector<hdmap::MapPathPoint>(
      route_points_.begin(), route_points_.end())));
  return true;
}

LocalRoutePoint LocalRoute::GetNearestLocalRoutePoint(const double s) const {
  const double kDisThreshold = 1.0e-2;
  const auto& accumulated_s = map_path_info_.accumulated_s();
  if (s < accumulated_s.front() - kDisThreshold) {
    ADEBUG << "The requested s: " << s << " < 0.";
    return route_points_.front();
  }
  if (s > accumulated_s.back() + kDisThreshold) {
    ADEBUG << "The requested s: " << s
           << " > reference line length: " << accumulated_s.back();
    return route_points_.back();
  }
  auto it_lower =
      std::lower_bound(accumulated_s.begin(), accumulated_s.end(), s);
  if (it_lower == accumulated_s.begin()) {
    return route_points_.front();
  }
  auto index = std::distance(accumulated_s.begin(), it_lower);
  if (std::fabs(accumulated_s[index - 1] - s) <
      std::fabs(accumulated_s[index] - s)) {
    return route_points_[index - 1];
  }
  return route_points_[index];
}

const std::vector<const Obstacle*> LocalRoute::FilterObstacles(
    const std::vector<const Obstacle*>& obstacles,
    const SLBoundary& adc_sl_boundary) {
  std::vector<const Obstacle*> filter_obstacles;
  // TODO fuhh refine lane width
  const double current_lane_start_l = -config_.default_local_route_width * 0.5;
  const double current_lane_end_l = config_.default_local_route_width * 0.5;
  const double left_lane_start_l = config_.default_local_route_width * 0.5;
  const double left_lane_end_l = 3 * config_.default_local_route_width * 0.5;
  const double right_lane_start_l =
      -3 * config_.default_local_route_width * 0.5;
  const double right_lane_end_l = -config_.default_local_route_width * 0.5;
  const double left_left_lane_start_l =
      3 * config_.default_local_route_width * 0.5;
  const double left_left_lane_end_l =
      5 * config_.default_local_route_width * 0.5;
  const double right_right_lane_start_l =
      -5 * config_.default_local_route_width * 0.5;
  const double right_right_lane_end_l =
      -3 * config_.default_local_route_width * 0.5;
  for (const auto* obstacle : obstacles) {
    SLBoundary sl_boundary;
    if (!GetSLBoundary(obstacle->PerceptionBoundingBox(), sl_boundary)) {
      AERROR << " Failed to get sl boundary for obstacle:" << obstacle->Id();
      continue;
    }
    const double center_l = (sl_boundary.start_l() + sl_boundary.end_l()) * 0.5;
    const double delta_s = sl_boundary.start_s() - adc_sl_boundary.end_s();
    if (center_l > current_lane_start_l && center_l < current_lane_end_l) {
      Obstacle* current_lane_obstacle = const_cast<Obstacle*>(obstacle);
      current_lane_obstacle->SetPerceptionSlBoundary(sl_boundary);

      if (delta_s > 0.0 && delta_s < config_.obs_filter_front_end_s) {
        current_lane_obstacle->SetRelativeRegion2LocalRoute(
            RelativeRegionType::FRONT);
        filter_obstacles.emplace_back(current_lane_obstacle);
      } else if (delta_s < 0.0 && delta_s > config_.obs_filter_rear_start_s) {
        current_lane_obstacle->SetRelativeRegion2LocalRoute(
            RelativeRegionType::REAR);
        filter_obstacles.emplace_back(current_lane_obstacle);
      }

    } else if (center_l > left_lane_start_l && center_l < left_lane_end_l) {
      Obstacle* left_lane_obstacle = const_cast<Obstacle*>(obstacle);
      left_lane_obstacle->SetPerceptionSlBoundary(sl_boundary);

      if (delta_s >= config_.obs_filter_front_start_s &&
          delta_s < config_.obs_filter_front_end_s) {
        left_lane_obstacle->SetRelativeRegion2LocalRoute(
            RelativeRegionType::LEFT_FRONT);
        filter_obstacles.emplace_back(left_lane_obstacle);
      } else if (delta_s < config_.obs_filter_front_start_s &&
                 delta_s > config_.obs_filter_rear_end_s) {
        left_lane_obstacle->SetRelativeRegion2LocalRoute(
            RelativeRegionType::LEFT);
        filter_obstacles.emplace_back(left_lane_obstacle);
      } else if (delta_s <= config_.obs_filter_rear_end_s &&
                 delta_s > config_.obs_filter_rear_start_s) {
        left_lane_obstacle->SetRelativeRegion2LocalRoute(
            RelativeRegionType::LEFT_REAR);
        filter_obstacles.emplace_back(left_lane_obstacle);
      }

    } else if (center_l > right_lane_start_l && center_l < right_lane_end_l) {
      Obstacle* right_lane_obstacle = const_cast<Obstacle*>(obstacle);
      right_lane_obstacle->SetPerceptionSlBoundary(sl_boundary);

      if (delta_s >= config_.obs_filter_front_start_s &&
          delta_s < config_.obs_filter_front_end_s) {
        right_lane_obstacle->SetRelativeRegion2LocalRoute(
            RelativeRegionType::RIGHT_FRONT);
        filter_obstacles.emplace_back(right_lane_obstacle);
      } else if (delta_s < config_.obs_filter_front_start_s &&
                 delta_s > config_.obs_filter_rear_end_s) {
        right_lane_obstacle->SetRelativeRegion2LocalRoute(
            RelativeRegionType::RIGHT);
        filter_obstacles.emplace_back(right_lane_obstacle);
      } else if (delta_s <= config_.obs_filter_rear_end_s &&
                 delta_s > config_.obs_filter_rear_start_s) {
        right_lane_obstacle->SetRelativeRegion2LocalRoute(
            RelativeRegionType::RIGHT_REAR);
        filter_obstacles.emplace_back(right_lane_obstacle);
      }

    } else if (center_l > left_left_lane_start_l &&
               center_l < left_left_lane_end_l) {
      Obstacle* left_left_lane_obstacle = const_cast<Obstacle*>(obstacle);
      left_left_lane_obstacle->SetPerceptionSlBoundary(sl_boundary);

      if (delta_s >= config_.obs_filter_rear_end_s &&
          delta_s < config_.obs_filter_front_end_s) {
        left_left_lane_obstacle->SetRelativeRegion2LocalRoute(
            RelativeRegionType::LEFT_LEFT_FRONT);
        filter_obstacles.emplace_back(left_left_lane_obstacle);
      }

    } else if (center_l > right_right_lane_start_l &&
               center_l < right_right_lane_end_l) {
      Obstacle* right_right_lane_obstacle = const_cast<Obstacle*>(obstacle);
      right_right_lane_obstacle->SetPerceptionSlBoundary(sl_boundary);

      if (delta_s >= config_.obs_filter_rear_end_s &&
          delta_s < config_.obs_filter_front_end_s) {
        right_right_lane_obstacle->SetRelativeRegion2LocalRoute(
            RelativeRegionType::RIGHT_RIGHT_FRONT);
        filter_obstacles.emplace_back(right_right_lane_obstacle);
      }
    }
  }
  return filter_obstacles;
}

const double LocalRoute::GetEndS() const {
  return map_path_info_.accumulated_s().back();
}

}  // namespace planning
}  // namespace zark
