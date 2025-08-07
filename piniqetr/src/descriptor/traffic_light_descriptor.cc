#include "descriptor/traffic_light_descriptor.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <optional>
#include <ostream>
#include <string>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_set.h"
#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"
#include "behavior.pb.h"
#include "common/speed/constant_acc_builder.h"
#include "container/strong_int.h"
#include "glog/logging.h"
#include "maps/semantic_map_defs.h"
#include "math/geometry/halfplane.h"
#include "math/util.h"
#include "piecewise_linear_function.pb.h"
#include "plan/planner_defs.h"
#include "speed/speed_point.h"
#include "traffic_light_info.pb.h"
#include "util/map_util.h"
#include "util/vehicle_geometry_util.h"

#define TLR_DEBUG 0

namespace e2e_noa {
namespace planning {

namespace {
constexpr double kTrafficLightStandoff = 1.0;
constexpr double kEpsilon = 1e-5;
constexpr double kExtraLeftWaitStopBuffer = 1.5;
}  // namespace

using TrafficLightStatusMap = ad_e2e::planning::TrafficLightStatusMap;
using TrafficLightStatus = ad_e2e::planning::TrafficLightStatus;
using FsdTrafficLightDeciderInfo = ad_e2e::planning::FsdTrafficLightDeciderInfo;
using StopLineReason = ad_e2e::planning::StopLineReason;
using StopLineInterface = ad_e2e::planning::StopLineInterface;
using LightStatus = ad_e2e::planning::LightStatus;
using CompositeTurnType = ad_e2e::planning::CompositeTurnType;
using LanePtr = ad_e2e::planning::LanePtr;
using TrafficLightInfoPtr = ad_e2e::planning::TrafficLightInfoPtr;
using TrafficLightSubType = ad_e2e::planning::TrafficLightSubType;

bool LeftWaitCheck(const mapping::ElementId left_wait_lane,
                   const mapping::ElementId focus_lane,
                   const LightStatus left_wait_lane_light,
                   const LightStatus focus_lane_light,
                   const bool has_set_leftwait, const bool ego_is_in_leftwait) {
  if (left_wait_lane != mapping::kInvalidElementId &&
      left_wait_lane_light == LightStatus::RED_LIGHT) {
    return false;
  }
  if ((left_wait_lane != mapping::kInvalidElementId && has_set_leftwait) ||
      (ego_is_in_leftwait && has_set_leftwait)) {
    return true;
  }
  if (left_wait_lane != mapping::kInvalidElementId &&
      focus_lane != mapping::kInvalidElementId &&
      left_wait_lane_light == LightStatus::GREEN_LIGHT &&
      (focus_lane_light == LightStatus::RED_LIGHT ||
       focus_lane_light == LightStatus::FAIL_DETECTION)) {
    return true;
  }
  return false;
}

double ComputeDisToStopLine(
    const PlanPassage& passage, ad_e2e::planning::StopLinePtr stop_line,
    const ApolloTrajectoryPointProto& plan_start_point) {
  double stop_line_s = std::numeric_limits<double>::max();
  double start_point_s = std::numeric_limits<double>::max();
  const auto& start_point = plan_start_point.path_point();
  if (stop_line && !stop_line->id().empty() && stop_line->points().size() > 1) {
    if (!passage.lane_seq_info()) {
      std::vector<std::string> tl_input_debug;
      tl_input_debug.emplace_back(" NO lane_seq !");
      return std::numeric_limits<double>::max();
    }
    const double delta_x =
        stop_line->points().back().x() - stop_line->points().front().x();
    const double delta_y =
        stop_line->points().back().y() - stop_line->points().front().y();
    const double k_stop_line = delta_y / (delta_x + kEpsilon);
    const double k_start_point = tan(start_point.theta());
    if (fabs(k_start_point - k_stop_line) < kEpsilon) {
      return std::numeric_limits<double>::max();
    }
    double stop_line_x = (k_stop_line * stop_line->points().front().x() -
                          stop_line->points().front().y() -
                          (k_start_point * start_point.x() - start_point.y())) /
                         (k_stop_line - k_start_point);
    const double min_x = std::min(stop_line->points().front().x(),
                                  stop_line->points().back().x());
    const double max_x = std::max(stop_line->points().front().x(),
                                  stop_line->points().back().x());

    if (stop_line_x < min_x) {
      stop_line_x = min_x;
    } else if (stop_line_x > max_x) {
      stop_line_x = max_x;
    }
    const double stop_line_y =
        k_stop_line * (stop_line_x - stop_line->points().front().x()) +
        stop_line->points().front().y();
    if (passage.lane_seq_info() && passage.lane_seq_info()->lane_seq) {
      const auto& laneseq = passage.lane_seq_info()->lane_seq;
      laneseq->GetProjectionDistance({stop_line_x, stop_line_y}, &stop_line_s);
      laneseq->GetProjectionDistance({start_point.x(), start_point.y()},
                                     &start_point_s);
      if (stop_line_s == std::numeric_limits<double>::max() ||
          start_point_s == std::numeric_limits<double>::max()) {
        return std::numeric_limits<double>::max();
      }
      return stop_line_s - start_point_s;
    } else {
      std::vector<std::string> tl_input_debug;
      tl_input_debug.emplace_back(" NO lane_seq !");
      return std::numeric_limits<double>::max();
    }
  } else {
    std::vector<std::string> tl_input_debug;
    tl_input_debug.emplace_back(" stop_line size short !");
    return std::numeric_limits<double>::max();
  }
}

bool IfNeedYellowLightBrake(const double& front_edge_to_center_dis,
                            const double& dis_to_stopline,
                            const ApolloTrajectoryPointProto& plan_start_point,
                            const StopLineReason& last_stopline_reason,
                            const bool& has_green_to_yellow,
                            const double& has_green_to_yellow_time) {
  std::vector<std::string> tl_output_debug;
  if (last_stopline_reason == StopLineReason::REASON_LIGHT_YELLOW) {
    tl_output_debug.emplace_back(
        " last_stopline_reason is REASON_LIGHT_YELLOW ");
    return true;
  }
  if (!has_green_to_yellow) {
    tl_output_debug.emplace_back("!has_green_to_yellow");
    return true;
  }

  double head_to_stopline = dis_to_stopline - front_edge_to_center_dis;
  double target_a = -4.0;
  double target_v = 0.0;
  const double dt = 5.0;
  auto speed_builder = std::make_shared<ad_e2e::planning::ConstantAccBuilder>();
  ad_e2e::planning::SpeedPoint start_sp(0.0, 0.0, plan_start_point.v(),
                                        plan_start_point.a(),
                                        plan_start_point.j());
  ad_e2e::planning::SpeedPoint target_sp(dt, 0.0, target_v, target_a, 0.0);
  auto speed_profile = speed_builder->Build(start_sp, target_sp, dt);
  double target_s = speed_profile->GetSpeedPointAtTime(dt).s;
  double target_v_f = speed_profile->GetSpeedPointAtTime(dt).v;
  double add_dis = 0.0;
  if (target_v_f > 0.0) {
    add_dis = target_v_f * target_v_f / 2 / 4.0;
  }
  double brake_distance = target_s + add_dis;
  double brake_buffer = 1.0;
  tl_output_debug.emplace_back("yellow-brake_distance:" +
                               std::to_string(brake_distance));
  tl_output_debug.emplace_back("yellow-head_to_stopline:" +
                               std::to_string(head_to_stopline));
  if (brake_distance < (head_to_stopline + brake_buffer)) {
    tl_output_debug.emplace_back(" can brake before stopline(buffer 1.0)");
    return true;
  }

  double cur_time = plan_start_point.relative_time();
  double keep_time = 2.25 - (cur_time - has_green_to_yellow_time);
  keep_time = keep_time < 0.0 ? 0.0 : keep_time;
  double keep_distance = plan_start_point.v() * keep_time +
                         0.5 * plan_start_point.a() * keep_time * keep_time;
  if (keep_distance < dis_to_stopline) {
    tl_output_debug.emplace_back("yellow-4");
    return true;
  }
  return false;
}

bool ExtendStopLines(const PlannerSemanticMapManager& psmm,
                     const PlanPassage& passage,
                     const std::string& first_virtual_lane_id,
                     const std::string& focus_lane_id,
                     std::vector<ConstraintProto::StopLineProto>& tl_stop_lines,
                     const double stop_s, const double adapt_tl_standoff,
                     bool need_precede) {
  const auto& first_virtual_lane =
      psmm.map_ptr()->GetLaneById(first_virtual_lane_id);
  if (!passage.lane_seq_info()) return false;
  const auto& laneseq = passage.lane_seq_info()->lane_seq;
  if (!laneseq) return false;
  const LanePtr final_common_lane =
      laneseq->GetPreLaneOnLaneSequence(first_virtual_lane);
  if (!final_common_lane) return false;
  const auto& first_focus_lane =
      need_precede ? final_common_lane : first_virtual_lane;
  const auto& final_section =
      psmm.map_ptr()->GetSectionById(first_focus_lane->section_id());
  if (!final_section) return false;

  for (const auto& lane : final_section->lanes()) {
    if (lane == first_focus_lane->id()) continue;
    const auto& lane_ptr = psmm.map_ptr()->GetLaneById(lane);
    if (!lane_ptr || lane_ptr->points().empty()) continue;
    const Vec2d center = lane_ptr->points().back();
    ConstraintProto::StopLineProto stop_line;
    const auto heading = passage.QueryTangentAngleAtS(stop_s);
    const auto& s = passage.QueryFrenetLonOffsetAt(center);
    if (!s.ok() || stop_s - s.value().accum_s > 15.0) {
      continue;
    }
    if (!heading.ok()) {
      continue;
    }
    const Vec2d unit = Vec2d::FastUnitFromAngle(*heading);

    HalfPlane fence(center - 1.5 * unit.Perp(), center + 1.5 * unit.Perp());
    fence.ToProto(stop_line.mutable_half_plane());
    stop_line.set_s(s.value().accum_s);
    stop_line.set_standoff(0.7);
    stop_line.set_time(0.0);
    stop_line.set_id(
        absl::StrFormat("extended_traffic_light_%s", focus_lane_id));
    stop_line.set_is_extended(true);
    stop_line.set_is_traffic_light(true);
    stop_line.mutable_source()->mutable_traffic_light()->set_lane_id(
        focus_lane_id + " extended " + lane);
    stop_line.mutable_source()->mutable_traffic_light()->set_id(
        focus_lane_id + " extended " + lane);
    tl_stop_lines.push_back(std::move(stop_line));
    std::vector<Vec2d> stop_line_points;
    auto left_point = center - 1.5 * unit.Perp();
    stop_line_points.push_back(left_point);
    auto right_point = center + 1.5 * unit.Perp();
    stop_line_points.push_back(right_point);
  }
  return true;
}

FsdTrafficLightDeciderInfo GetFsdTrafficLightDeciderInfo(
    const PlanPassage& passage, const mapping::LanePath& lane_path_from_start,
    const TrafficLightStatusMap& traffic_light_status_map,
    const PlannerSemanticMapManager& psmm,
    const Behavior_FunctionId& map_func_id,
    const ApolloTrajectoryPointProto& plan_start_point) {
  FsdTrafficLightDeciderInfo info;
  double distance = 0.0;
  for (const auto& seg : lane_path_from_start) {
    const auto& lane_info = traffic_light_status_map.find(seg.lane_id);
    if (lane_info == traffic_light_status_map.end()) {
      break;
    }
    const auto& traffic_light_status = lane_info->second;
    if (traffic_light_status.junction_id.has_value() &&
        info.first_virtual_lane == mapping::kInvalidElementId) {
      info.first_virtual_lane = seg.lane_id;
      info.first_virtual_lane_light = traffic_light_status.light_status;
      info.first_virtual_lane_is_leftwait =
          traffic_light_status.is_left_wait_lane;
      info.dist_to_stopline = distance;
    }
    if (traffic_light_status.junction_id.has_value() &&
        !traffic_light_status.is_left_wait_lane) {
      info.focus_lane = seg.lane_id;
      info.focus_lane_light = traffic_light_status.light_status;
      info.dist_to_leftwait_stopline = distance;
      break;
    }
    if (traffic_light_status.junction_id.has_value() &&
        traffic_light_status.is_left_wait_lane) {
      info.left_wait_lane = seg.lane_id;
      info.left_wait_lane_light = traffic_light_status.light_status;
      info.focus_lane = seg.lane_id;
      info.focus_lane_light = LightStatus::UNKNOWN_LIGHT;
      info.dist_to_leftwait_stopline = distance;
    }
    distance += (seg.end_s - seg.start_s);
  }

  const auto& start_lane_info =
      traffic_light_status_map.find(lane_path_from_start.lane_ids().front());
  if (start_lane_info != traffic_light_status_map.end()) {
    info.ego_is_in_leftwait = start_lane_info->second.is_left_wait_lane;
  }

  std::vector<std::string> tl_input_debug;
  tl_input_debug.emplace_back("dist_to_stopline:" +
                              std::to_string(info.dist_to_stopline));
  tl_input_debug.emplace_back("dist_to_leftwait_stopline:" +
                              std::to_string(info.dist_to_leftwait_stopline));
  tl_input_debug.emplace_back("ego_is_in_leftwait:" +
                              std::to_string(info.ego_is_in_leftwait));
  tl_input_debug.emplace_back("virtual_lane:" + info.first_virtual_lane + "*" +
                              std::to_string(info.first_virtual_lane_light));
  tl_input_debug.emplace_back("focus_lane:" + info.focus_lane + "*" +
                              std::to_string(info.focus_lane_light));
  tl_input_debug.emplace_back("left_wait_lane:" + info.left_wait_lane + "*" +
                              std::to_string(info.left_wait_lane_light));
  return info;
}

FsdTrafficLightDeciderInfo GetFsdTrafficLightDeciderInformationForMapless(
    const PlanPassage& passage, const mapping::LanePath& lane_path_from_start,
    const TrafficLightStatusMap& traffic_light_status_map,
    const PlannerSemanticMapManager& psmm,
    const Behavior_FunctionId& map_func_id,
    const ApolloTrajectoryPointProto& plan_start_point, bool& need_precede) {
  FsdTrafficLightDeciderInfo info;
  const auto& stop_line_map = psmm.map_ptr()->stop_line_map();
  for (const auto& seg : lane_path_from_start) {
    const auto& lane_ptr = psmm.map_ptr()->GetLaneById(seg.lane_id);
    if (!lane_ptr) {
      break;
    }
    if (lane_ptr->overlap_stop_lines().empty()) {
      continue;
    }
    if (lane_ptr->overlap_stop_lines().front().substr(0, 3) == "vr_") {
      continue;
    }
    const auto& stop_line_found =
        stop_line_map.find(lane_ptr->overlap_stop_lines().front());
    if (stop_line_found == stop_line_map.end()) {
      break;
    }
    const auto& stop_line_info = stop_line_found->second;

    const double raw_dist =
        ComputeDisToStopLine(passage, stop_line_info, plan_start_point);
    if (!lane_ptr->overlap_stop_lines().empty() &&
        info.first_virtual_lane == mapping::kInvalidElementId) {
      info.first_virtual_lane = lane_ptr->id();
      info.first_virtual_lane_light = stop_line_info->light_type();
      info.first_virtual_lane_is_leftwait = (stop_line_info->sub_type() == 4);
      info.dist_to_stopline = raw_dist;
    }
    if (!lane_ptr->overlap_stop_lines().empty() &&
        stop_line_info->sub_type() != 4) {
      info.focus_lane = lane_ptr->id();
      info.focus_lane_light = stop_line_info->light_type();
      info.dist_to_leftwait_stopline = raw_dist;
      const auto& lane_info = traffic_light_status_map.find(seg.lane_id);
      if (lane_info != traffic_light_status_map.end()) {
        const auto& traffic_light_status = lane_info->second;
        info.focus_lane_light_info = traffic_light_status.traffic_light_info;
        info.focus_lane_light = traffic_light_status.light_status;
        info.first_virtual_lane_light = traffic_light_status.light_status;
      }
      break;
    }
    if (!lane_ptr->overlap_stop_lines().empty() &&
        stop_line_info->sub_type() == 4) {
      info.left_wait_lane = lane_ptr->id();
      info.left_wait_lane_light = stop_line_info->light_type();
      info.focus_lane = lane_ptr->id();
      info.focus_lane_light = LightStatus::UNKNOWN_LIGHT;
      info.dist_to_leftwait_stopline = raw_dist;
      const auto& lane_info = traffic_light_status_map.find(seg.lane_id);
      if (lane_info != traffic_light_status_map.end()) {
        const auto& traffic_light_status = lane_info->second;
        info.focus_lane_light_info = traffic_light_status.traffic_light_info;
        info.left_wait_lane_light = traffic_light_status.light_status;
        info.first_virtual_lane_light = traffic_light_status.light_status;
      }
    }
  }
  if (info.dist_to_stopline == std::numeric_limits<double>::max()) {
    double distance = 0.0;
    for (const auto& seg : lane_path_from_start) {
      const auto& lane_info = traffic_light_status_map.find(seg.lane_id);
      if (lane_info == traffic_light_status_map.end()) {
        break;
      }
      const auto& traffic_light_status = lane_info->second;
      if (traffic_light_status.junction_id.has_value() &&
          info.first_virtual_lane == mapping::kInvalidElementId) {
        info.first_virtual_lane = seg.lane_id;
        info.first_virtual_lane_light = traffic_light_status.light_status;
        info.first_virtual_lane_is_leftwait =
            traffic_light_status.is_left_wait_lane;
        info.dist_to_stopline = distance;
        info.focus_lane_light_info = traffic_light_status.traffic_light_info;
        need_precede = true;
      }
      if (traffic_light_status.junction_id.has_value() &&
          !traffic_light_status.is_left_wait_lane) {
        info.focus_lane = seg.lane_id;
        info.focus_lane_light = traffic_light_status.light_status;
        info.dist_to_leftwait_stopline = distance;
        info.focus_lane_light_info = traffic_light_status.traffic_light_info;
        break;
      }
      if (traffic_light_status.junction_id.has_value() &&
          traffic_light_status.is_left_wait_lane) {
        info.left_wait_lane = seg.lane_id;
        info.left_wait_lane_light = traffic_light_status.light_status;
        info.focus_lane = seg.lane_id;
        info.focus_lane_light = LightStatus::UNKNOWN_LIGHT;
        info.dist_to_leftwait_stopline = distance;
      }
      distance += (seg.end_s - seg.start_s);
    }
    if (info.dist_to_stopline == std::numeric_limits<double>::max()) {
      const auto& route_info = psmm.map_ptr()->route()->GetRouteInfo();
      const auto& sections_info = route_info.sections;
      double stop_line_distance_oh = 0.0;
      bool has_start = false;
      for (const auto& section_info : sections_info) {
        if (!has_start) {
          if (section_info.id == route_info.navi_start.section_id) {
            has_start = true;
          } else {
            continue;
          }
        }
        if (section_info.lane_ids.empty()) {
          DLOG(ERROR) << "section " << section_info.id << " lane_ids empty";
          continue;
        }
        const auto& lane_id =
            static_cast<mapping::ElementId>(section_info.lane_ids.at(0));
        const auto& lane_info_oh = traffic_light_status_map.find(lane_id);
        if (lane_info_oh == traffic_light_status_map.end()) {
          break;
        }
        const auto& traffic_light_status_oh = lane_info_oh->second;
        if (traffic_light_status_oh.junction_id.has_value() &&
            info.first_virtual_lane == mapping::kInvalidElementId) {
          info.first_virtual_lane = lane_id;
          info.first_virtual_lane_light = traffic_light_status_oh.light_status;
          info.first_virtual_lane_is_leftwait =
              traffic_light_status_oh.is_left_wait_lane;
          info.dist_to_stopline = stop_line_distance_oh;
          info.focus_lane_light_info =
              traffic_light_status_oh.traffic_light_info;
          need_precede = true;
        }
        if (traffic_light_status_oh.junction_id.has_value() &&
            !traffic_light_status_oh.is_left_wait_lane) {
          info.focus_lane = lane_id;
          info.focus_lane_light = traffic_light_status_oh.light_status;
          info.focus_lane_light_info =
              traffic_light_status_oh.traffic_light_info;
          info.dist_to_leftwait_stopline = stop_line_distance_oh;
          break;
        }
        if (traffic_light_status_oh.junction_id.has_value() &&
            traffic_light_status_oh.is_left_wait_lane) {
          info.left_wait_lane = lane_id;
          info.left_wait_lane_light = traffic_light_status_oh.light_status;
          info.focus_lane = lane_id;
          info.focus_lane_light = LightStatus::UNKNOWN_LIGHT;
        }
        if (section_info.id == route_info.navi_start.section_id) {
          stop_line_distance_oh +=
              section_info.length - route_info.navi_start.s_offset;
        } else {
          stop_line_distance_oh += section_info.length;
        }
      }
    }
  }

  const auto& start_lane_info =
      traffic_light_status_map.find(lane_path_from_start.lane_ids().front());
  if (start_lane_info != traffic_light_status_map.end()) {
    info.ego_is_in_leftwait = start_lane_info->second.is_left_wait_lane;
    if (info.ego_is_in_leftwait) {
      info.dist_to_stopline = 0.0;
    }
  }

  std::vector<std::string> tl_input_debug;
  tl_input_debug.emplace_back("dist_to_stopline:" +
                              std::to_string(info.dist_to_stopline));
  tl_input_debug.emplace_back("dist_to_leftwait_stopline:" +
                              std::to_string(info.dist_to_leftwait_stopline));
  tl_input_debug.emplace_back("ego_is_in_leftwait:" +
                              std::to_string(info.ego_is_in_leftwait));
  tl_input_debug.emplace_back("virtual_lane:" + info.first_virtual_lane + "*" +
                              std::to_string(info.first_virtual_lane_light));
  tl_input_debug.emplace_back("focus_lane:" + info.focus_lane + "*" +
                              std::to_string(info.focus_lane_light));
  tl_input_debug.emplace_back("left_wait_lane:" + info.left_wait_lane + "*" +
                              std::to_string(info.left_wait_lane_light));

#if TLR_DEBUG
  LOG(INFO) << " distance to stop line: " << info.dist_to_stopline
            << " virtual_lane: " << info.first_virtual_lane
            << " virtual_lane_light: " << int(info.first_virtual_lane_light)
            << " focus_lane_light: " << int(info.focus_lane_light)
            << " focus_lane: " << info.focus_lane
            << " sub type = " << int(info.focus_lane_light_info.sub_type)
            << " light id = " << info.focus_lane_light_info.id;
#endif
  return info;
}

bool IfNeedBlockLightReset(
    const PlannerSemanticMapManager& psmm, const PlanPassage& passage,
    const SpacetimeTrajectoryManager& st_traj_mgr,
    const StopLineReason& last_stop_reason, const double dist_to_stopline,
    const e2e_noa::VehicleGeometryParamsProto& vehicle_geo_params,
    const std::string virtual_lane,
    const ApolloTrajectoryPointProto& plan_start_point) {
  if (last_stop_reason != StopLineReason::REASON_LIGHT_RED &&
      last_stop_reason != StopLineReason::REASON_LIGHT_YELLOW) {
    return false;
  }

  if (!passage.lane_seq_info()) return false;
  const auto& laneseq = passage.lane_seq_info()->lane_seq;
  if (!laneseq) return false;

  const auto& vr_lane = psmm.map_ptr()->GetLaneById(virtual_lane);
  const LanePtr pre_virtual_lane = laneseq->GetPreLaneOnLaneSequence(vr_lane);
  CompositeTurnType turn_type =
      psmm.map_ptr()->CheckCompositeLane(pre_virtual_lane);
  if (turn_type != CompositeTurnType::NORMAL_TURN) {
    return false;
  }

  for (const auto& obj : st_traj_mgr.object_trajectories_map()) {
    if (obj.second.empty()) {
      continue;
    }
    const auto& obstacle = obj.second.front();
    if (obstacle->object_type() != OT_VEHICLE &&
        obstacle->object_type() != OT_LARGE_VEHICLE) {
      continue;
    }
    if (obstacle->is_stationary()) {
      continue;
    }

    constexpr double kBlockHeadingThreshold = M_PI / 6.0;
    const double theta_diff = std::fabs(NormalizeAngle(
        obstacle->pose().theta() - plan_start_point.path_point().theta()));
    if (theta_diff > kBlockHeadingThreshold) {
      continue;
    }

    const absl::StatusOr<FrenetBox> frenet_box_or =
        passage.QueryFrenetBoxAtContour(obstacle->contour());
    if (!frenet_box_or.ok()) {
      continue;
    }
    const double ego_front_to_center =
        vehicle_geo_params.front_edge_to_center();
    const double ego_back_to_center = vehicle_geo_params.back_edge_to_center();
    const double ego_half_width = vehicle_geo_params.width() * 0.5;
    double ds = 0.0;
    double dl = 0.0;
    if (frenet_box_or->s_min > ego_front_to_center) {
      ds = frenet_box_or->s_min - ego_front_to_center;
    } else if (frenet_box_or->s_max < (-1.0 * ego_front_to_center)) {
      ds = frenet_box_or->s_max + ego_back_to_center;
    } else {
      ds = 0.0;
    }
    if (frenet_box_or->l_min > ego_half_width) {
      dl = frenet_box_or->l_min - ego_half_width;
    } else if (frenet_box_or->l_max < (-1.0 * ego_half_width)) {
      dl = frenet_box_or->l_max + ego_half_width;
    } else {
      dl = 0.0;
    }

    if (ds > kEpsilon && ds < 20.0 && ds < dist_to_stopline &&
        std::fabs(dl) < kEpsilon) {
      return true;
    }
  }
  return false;
}

void LKATrafficLightPreBrake(
    FsdTrafficLightDeciderStateProto tld_state, const double dist_to_stopline,
    bool& need_prebrake_for_lka,
    const ApolloTrajectoryPointProto& plan_start_point) {
  if (dist_to_stopline < kEpsilon || dist_to_stopline > 160.0) {
    return;
  }

  if (tld_state.green_to_yellow() &&
      tld_state.tl_stop_interface() == StopLineInterface::STOP_LINE_NONE) {
    return;
  }

  if (tld_state.pass_junction()) {
    return;
  }

  if (plan_start_point.v() > Kph2Mps(70.0)) {
    need_prebrake_for_lka = true;
  }
}

TrafficLightInfoPtr GetTrafficLightInfo(
    const PlannerSemanticMapManager& psmm,
    const std::vector<std::string>& traffic_lights,
    const ad_e2e::planning::TurnType turn_type) {
  for (const auto& tl_id : traffic_lights) {
    const auto& tl_ptr = psmm.map_ptr()->GetTrafficLightInfoById(tl_id);
    if (!tl_ptr) {
      continue;
    }
    if ((tl_ptr->sub_type & TrafficLightSubType::TRAFFICLIGHT_DIRECTION_LEFT) &&
        turn_type == ad_e2e::planning::TurnType::LEFT_TURN) {
      return tl_ptr;
    }
    if ((tl_ptr->sub_type &
         TrafficLightSubType::TRAFFICLIGHT_DIRECTION_RIGHT) &&
        turn_type == ad_e2e::planning::TurnType::RIGHT_TURN) {
      return tl_ptr;
    }
    if ((tl_ptr->sub_type & TrafficLightSubType::TRAFFICLIGHT_DIRECTION_UP) &&
        turn_type == ad_e2e::planning::TurnType::NO_TURN) {
      return tl_ptr;
    }
    if ((tl_ptr->sub_type &
         TrafficLightSubType::TRAFFICLIGHT_DIRECTION_UTURN) &&
        turn_type == ad_e2e::planning::TurnType::U_TURN) {
      return tl_ptr;
    }
  }
  if (turn_type == ad_e2e::planning::TurnType::RIGHT_TURN) {
    return nullptr;
  }
  for (const auto& tl_id : traffic_lights) {
    const auto& tl_ptr = psmm.map_ptr()->GetTrafficLightInfoById(tl_id);
    if (!tl_ptr) {
      continue;
    }
    return tl_ptr;
  }
  return nullptr;
}

bool GenerateTrafficLightStatusMap(
    const mapping::LanePath& lane_path_from_start,
    const PlannerSemanticMapManager& psmm,
    TrafficLightStatusMap& traffic_light_status_map) {
  for (size_t i = 0; i < lane_path_from_start.size(); i++) {
    const auto& lane_id = lane_path_from_start.lane_id(i);
    const auto& lane_ptr = psmm.map_ptr()->GetLaneById(lane_id);
    if (!lane_ptr) {
      break;
    }
    TrafficLightStatus traffic_light_status;
    traffic_light_status.lane_id = lane_ptr->id();
    if (i < lane_path_from_start.size() - 1) {
      const auto& next_lane_id = lane_path_from_start.lane_id(i + 1);
      const auto& next_lane_ptr = psmm.map_ptr()->GetLaneById(next_lane_id);
      if (next_lane_ptr && !next_lane_ptr->junction_id().empty()) {
        traffic_light_status.junction_id = next_lane_ptr->junction_id();
        const auto& traffic_light_info =
            GetTrafficLightInfo(psmm, lane_ptr->lane_info().traffic_lights,
                                next_lane_ptr->turn_type());
        if (traffic_light_info) {
          traffic_light_status.light_status =
              static_cast<LightStatus>(traffic_light_info->light_status);
          traffic_light_status.traffic_light_info = *traffic_light_info;
        }
      }
    } else {
      bool set_traffic_light = false;
      for (const auto& tl_id : lane_ptr->lane_info().traffic_lights) {
        const auto& tl_ptr = psmm.map_ptr()->GetTrafficLightInfoById(tl_id);
        if (!tl_ptr) {
          continue;
        }
        if (!set_traffic_light) {
          traffic_light_status.light_status =
              static_cast<LightStatus>(tl_ptr->light_status);
          traffic_light_status.traffic_light_info = *tl_ptr;
          if (tl_ptr->light_status != LightStatus::UNKNOWN_LIGHT &&
              tl_ptr->light_status != LightStatus::NONE_LIGHT) {
            break;
          }
          set_traffic_light = true;
        } else {
          bool need_update = tl_ptr->light_status == LightStatus::GREEN_LIGHT ||
                             tl_ptr->light_status == LightStatus::RED_LIGHT ||
                             tl_ptr->light_status == LightStatus::YELLOW_LIGHT;
          if (need_update) {
#if TLR_DEBUG
            DLOG(INFO) << "update id " << tl_id << " light status "
                       << int(traffic_light_status.light_status);
#endif
            traffic_light_status.light_status =
                static_cast<LightStatus>(tl_ptr->light_status);
            traffic_light_status.traffic_light_info = *tl_ptr;
            break;
          }
        }
      }
    }

    for (const auto& stop_line_id : lane_ptr->lane_info().traffic_stop_lines) {
      psmm.map_ptr()->UpdateStopLineStatus(stop_line_id,
                                           traffic_light_status.light_status);
    }

    traffic_light_status.stop_line = lane_ptr->stop_line(),
    traffic_light_status.is_left_wait_lane =
        lane_ptr->type() == ad_e2e::planning::LANE_LEFT_WAIT ? true : false;
    traffic_light_status_map[traffic_light_status.lane_id] =
        std::move(traffic_light_status);
  }
  return true;
}

bool CheckNeedPreStop(const FsdTrafficLightDeciderInfo& fsd_tl_info,
                      const ApolloTrajectoryPointProto& plan_start_point) {
  if (fsd_tl_info.focus_lane_light != LightStatus::GREEN_LIGHT) {
    return false;
  }
  constexpr int count_down_sec_limit = 30;
  if (fsd_tl_info.focus_lane_light_info.count_down_sec > count_down_sec_limit ||
      fsd_tl_info.focus_lane_light_info.count_down_sec <= 0) {
    return false;
  }
  constexpr double junction_length = 30.0;
  const double current_v = std::max(plan_start_point.v(), kEpsilon);
  const double t_cross_junction =
      (junction_length + fsd_tl_info.dist_to_stopline) / current_v;
  if (t_cross_junction >
      double(fsd_tl_info.focus_lane_light_info.count_down_sec)) {
    return true;
  }
  return false;
}

bool CurrentJunctionTrafficLightDecider(
    const e2e_noa::VehicleGeometryParamsProto& vehicle_geometry_params,
    const ApolloTrajectoryPointProto& plan_start_point,
    const PlanPassage& passage, const mapping::LanePath& lane_path_from_start,
    const TrafficLightStatusMap& traffic_light_status_map,
    const PlannerSemanticMapManager& psmm,
    const TrafficLightDeciderStateProto& raw_tld_state,
    const bool& enable_tl_ok_btn, const bool& override_passable,
    const DecisionConstraintConfigProto& config,
    const SpacetimeTrajectoryManager& st_traj_mgr,
    TrafficLightDescriptor& output, const Behavior_FunctionId& map_func_id,
    const int plan_id, bool& need_prebrake_for_lka) {
  FsdTrafficLightDeciderInfo fsd_tl_info;
  bool need_precede = false;
  if (map_func_id == Behavior_FunctionId::Behavior_FunctionId_LKA ||
      map_func_id == Behavior_FunctionId::Behavior_FunctionId_MAPLESS_NOA ||
      map_func_id == Behavior_FunctionId::Behavior_FunctionId_CITY_NOA) {
    TrafficLightStatusMap traffic_light_status_map_new;
    GenerateTrafficLightStatusMap(lane_path_from_start, psmm,
                                  traffic_light_status_map_new);
    fsd_tl_info = GetFsdTrafficLightDeciderInformationForMapless(
        passage, lane_path_from_start, traffic_light_status_map_new, psmm,
        map_func_id, plan_start_point, need_precede);
  } else {
    fsd_tl_info = GetFsdTrafficLightDeciderInfo(passage, lane_path_from_start,
                                                traffic_light_status_map, psmm,
                                                map_func_id, plan_start_point);
  }
  if (fsd_tl_info.first_virtual_lane == mapping::kInvalidElementId ||
      fsd_tl_info.focus_lane == mapping::kInvalidElementId) {
    std::vector<ConstraintProto::StopLineProto> tl_stop_lines;
    std::vector<ConstraintProto::SpeedProfileProto> tl_speed_profiles;
    TrafficLightDeciderStateProto state;
    output = TrafficLightDescriptor{
        .stop_lines = std::move(tl_stop_lines),
        .speed_profiles = std::move(tl_speed_profiles),
        .traffic_light_decision_state = std::move(state)};
    return false;
  }
  double front_edge_to_center_dis =
      vehicle_geometry_params.front_edge_to_center();
  bool extern_button_ok = enable_tl_ok_btn;
  bool extern_override_passable = override_passable;
  double start_time = plan_start_point.relative_time();
  double start_v = plan_start_point.v();
  double start_a = plan_start_point.a();

  const int light_color = int(fsd_tl_info.focus_lane_light);

  const double distosl = fsd_tl_info.dist_to_stopline;

  FsdTrafficLightDeciderStateProto tld_state = raw_tld_state.fsd_tld_state();
  if (map_func_id == Behavior_FunctionId::Behavior_FunctionId_LKA) {
    LKATrafficLightPreBrake(tld_state, fsd_tl_info.dist_to_stopline,
                            need_prebrake_for_lka, plan_start_point);
  }

  tld_state.set_tl_stop_interface(StopLineInterface::STOP_LINE_NONE);
  if (tld_state.dist_to_junction() < -5.0 ||
      (fsd_tl_info.dist_to_stopline - tld_state.dist_to_junction()) > 12.0 ||
      (fsd_tl_info.dist_to_stopline - tld_state.dist_to_junction()) < -35.0) {
    tld_state.set_last_light_status(LightStatus::NONE_LIGHT);
    tld_state.set_valid_light_status(LightStatus::NONE_LIGHT);
    tld_state.set_light_stop_reason(StopLineReason::REASON_NONE);
    tld_state.set_light_counter(0);
    tld_state.set_green_to_yellow(false);
    tld_state.set_green_to_yellow_time(0.0);
    tld_state.set_set_leftwait(false);
    tld_state.set_pass_junction(false);
    tld_state.set_normal_override_passable(false);
    tld_state.set_change_to_unknown_time(0.0);
    tld_state.set_unknown_check(false);
    tld_state.set_fail_detect_enter_leftwait(false);
    tld_state.clear_multi_junc_tl_info();
    tld_state.set_override_passable(false);
    tld_state.set_override_count(0);
    tld_state.set_block_light_count(0);
  }

  LightStatus current_front_light = fsd_tl_info.focus_lane_light;

  bool is_block_fail = false;
  if (current_front_light == LightStatus::BLOCK_FAIL) {
    is_block_fail = true;
    current_front_light = LightStatus::UNKNOWN_LIGHT;
    int block_count_temp = tld_state.block_light_count();
    tld_state.set_block_light_count(block_count_temp + 1);
    if (tld_state.block_light_count() > 3 &&
        IfNeedBlockLightReset(
            psmm, passage, st_traj_mgr,
            StopLineReason(tld_state.light_stop_reason()),
            fsd_tl_info.dist_to_stopline, vehicle_geometry_params,
            fsd_tl_info.first_virtual_lane, plan_start_point)) {
      current_front_light = LightStatus::GREEN_LIGHT;
    }
  } else {
    tld_state.set_block_light_count(0);
  }

  bool is_fail_detection = false;
  if (current_front_light == LightStatus::FAIL_DETECTION) {
    is_fail_detection = true;
    if (tld_state.pass_junction()) {
      current_front_light = LightStatus::GREEN_LIGHT;
    } else {
      current_front_light = LightStatus::RED_LIGHT;
    }
  }
  if (!extern_override_passable) {
    tld_state.set_override_count(0);
  } else {
    int count_temp = tld_state.override_count();
    tld_state.set_override_count(count_temp + 1);
  }

  if (current_front_light == LightStatus::GREEN_LIGHT ||
      current_front_light == LightStatus::YELLOW_LIGHT ||
      current_front_light == LightStatus::RED_LIGHT) {
    tld_state.set_normal_override_passable(false);
  }

  if (CheckNeedPreStop(fsd_tl_info, plan_start_point)) {
#if TLR_DEBUG
    DLOG(INFO) << "Need pre Stop for light count :"
               << fsd_tl_info.focus_lane_light_info.count_down_sec;
#endif
    current_front_light = LightStatus::YELLOW_LIGHT;
  }

  LightStatus valid_front_light = current_front_light;

  if (valid_front_light == LightStatus::GREEN_LIGHT ||
      valid_front_light == LightStatus::NONE_LIGHT) {
    tld_state.set_last_light_status(current_front_light);
    tld_state.set_valid_light_status(valid_front_light);
    tld_state.set_light_stop_reason(StopLineReason::REASON_NONE);
    tld_state.set_dist_to_junction(fsd_tl_info.dist_to_stopline);
    tld_state.set_green_to_yellow(false);
    tld_state.set_green_to_yellow_time(0.0);
    tld_state.set_set_leftwait(false);
    tld_state.set_change_to_unknown_time(0.0);
    tld_state.set_unknown_check(false);
    tld_state.set_fail_detect_enter_leftwait(false);
    tld_state.set_tl_stop_interface(StopLineInterface::STOP_LINE_NONE);
    tld_state.set_light_id(fsd_tl_info.focus_lane_light_info.id);
    tld_state.set_junction_lane_id(fsd_tl_info.focus_lane);

    if ((fsd_tl_info.dist_to_stopline < kEpsilon &&
         !fsd_tl_info.ego_is_in_leftwait) ||
        is_fail_detection) {
      tld_state.set_pass_junction(true);
    } else {
      tld_state.set_pass_junction(false);
    }
    std::vector<ConstraintProto::StopLineProto> tl_stop_lines;
    std::vector<ConstraintProto::SpeedProfileProto> tl_speed_profiles;
    TrafficLightDeciderStateProto state;
    auto mutable_fsd_tld_state = state.mutable_fsd_tld_state();
    *mutable_fsd_tld_state = tld_state;
    output = TrafficLightDescriptor{
        .stop_lines = std::move(tl_stop_lines),
        .speed_profiles = std::move(tl_speed_profiles),
        .traffic_light_decision_state = std::move(state)};
#if TLR_DEBUG
    DLOG(INFO) << "tld state current front light: " << int(current_front_light)
               << " valid front light: " << int(valid_front_light)
               << " dist to junction: " << fsd_tl_info.dist_to_stopline
               << " stop line size = " << tl_stop_lines.size()
               << " light id = " << tld_state.light_id()
               << " focus lane = " << tld_state.junction_lane_id();
#endif
    return true;
  }

  bool set_stopline = false;
  StopLineReason stop_reason = StopLineReason::REASON_NONE;
  double stop_s = 0.0;
  if (fsd_tl_info.dist_to_stopline > kEpsilon) {
    bool green_to_yellow_side =
        tld_state.valid_light_status() == LightStatus::GREEN_LIGHT &&
        (valid_front_light == LightStatus::YELLOW_LIGHT ||
         valid_front_light == LightStatus::UNKNOWN_LIGHT);
    tld_state.set_green_to_yellow(tld_state.green_to_yellow() ||
                                  green_to_yellow_side);
    if (green_to_yellow_side) {
      tld_state.set_green_to_yellow_time(start_time);
    }

    if (valid_front_light == LightStatus::YELLOW_LIGHT &&
        IfNeedYellowLightBrake(
            front_edge_to_center_dis, fsd_tl_info.dist_to_stopline,
            plan_start_point, StopLineReason(tld_state.light_stop_reason()),
            tld_state.green_to_yellow(), tld_state.green_to_yellow_time())) {
      set_stopline = true;
      stop_reason = StopLineReason::REASON_LIGHT_YELLOW;
    }

    if (valid_front_light == LightStatus::RED_LIGHT) {
      set_stopline = true;
      stop_reason = StopLineReason::REASON_LIGHT_RED;
    }

    if (valid_front_light == LightStatus::UNKNOWN_LIGHT &&
        (tld_state.light_stop_reason() == StopLineReason::REASON_LIGHT_RED ||
         tld_state.light_stop_reason() ==
             StopLineReason::REASON_LIGHT_YELLOW) &&
        !tld_state.normal_override_passable()) {
      set_stopline = true;
      stop_reason = StopLineReason(tld_state.light_stop_reason());
    }
    if (set_stopline) {
      stop_s = passage.lane_path_start_s() + fsd_tl_info.dist_to_stopline -
               config.normal_offset();
    }
  } else {
    if (valid_front_light == LightStatus::RED_LIGHT &&
        (tld_state.light_stop_reason() != StopLineReason::REASON_NONE ||
         fsd_tl_info.ego_is_in_leftwait)) {
      set_stopline = true;
      stop_reason = StopLineReason::REASON_LIGHT_RED;
    }

    if (valid_front_light == LightStatus::YELLOW_LIGHT &&
        start_v < Kph2Mps(10.0) &&
        tld_state.light_stop_reason() != StopLineReason::REASON_NONE) {
      set_stopline = true;
      stop_reason = StopLineReason::REASON_LIGHT_YELLOW;
    }

    if (valid_front_light == LightStatus::UNKNOWN_LIGHT &&
        (tld_state.light_stop_reason() == StopLineReason::REASON_LIGHT_RED ||
         tld_state.light_stop_reason() ==
             StopLineReason::REASON_LIGHT_YELLOW)) {
      set_stopline = true;
      stop_reason = StopLineReason(tld_state.light_stop_reason());
    }

    if (tld_state.pass_junction() || tld_state.override_passable()) {
      set_stopline = false;
      stop_reason = StopLineReason::REASON_NONE;
    }
    if (!set_stopline &&
        (!fsd_tl_info.ego_is_in_leftwait ||
         tld_state.light_stop_reason() == StopLineReason::REASON_NONE)) {
      tld_state.set_pass_junction(true);
    }
    if (set_stopline) {
      stop_s = passage.lane_path_start_s() - config.normal_offset();
    }
  }

  tld_state.set_fail_detect_enter_leftwait(
      is_fail_detection && fsd_tl_info.dist_to_stopline > kEpsilon &&
      fsd_tl_info.first_virtual_lane_is_leftwait &&
      fsd_tl_info.first_virtual_lane_light == LightStatus::FAIL_DETECTION &&
      (extern_button_ok || tld_state.fail_detect_enter_leftwait()));
  if (tld_state.fail_detect_enter_leftwait()) {
    tld_state.set_set_leftwait(true);

    std::vector<std::string> tl_output_debug;
    tl_output_debug.emplace_back("ok-button: enter leftwait");
  }
  if (set_stopline &&
      LeftWaitCheck(fsd_tl_info.left_wait_lane, fsd_tl_info.focus_lane,
                    fsd_tl_info.left_wait_lane_light,
                    fsd_tl_info.focus_lane_light, tld_state.set_leftwait(),
                    fsd_tl_info.ego_is_in_leftwait)) {
    stop_s = passage.lane_path_start_s() +
             fsd_tl_info.dist_to_leftwait_stopline - config.normal_offset() -
             kExtraLeftWaitStopBuffer;
    tld_state.set_set_leftwait(true);
  } else {
    tld_state.set_set_leftwait(false);
  }

  if (tld_state.valid_light_status() != LightStatus::UNKNOWN_LIGHT &&
      valid_front_light == LightStatus::UNKNOWN_LIGHT) {
    tld_state.set_change_to_unknown_time(start_time);
  } else if (valid_front_light != LightStatus::UNKNOWN_LIGHT) {
    tld_state.set_change_to_unknown_time(0.0);
  }
  if (!tld_state.unknown_check() && set_stopline &&
      fsd_tl_info.dist_to_stopline < kEpsilon && start_v < 0.5 &&
      valid_front_light == LightStatus::UNKNOWN_LIGHT &&
      (tld_state.light_stop_reason() == StopLineReason::REASON_LIGHT_RED ||
       tld_state.light_stop_reason() == StopLineReason::REASON_LIGHT_YELLOW)) {
    double unknown_time = start_time - tld_state.change_to_unknown_time();
    double time_buffer = 15.0;
    if (is_block_fail) {
      time_buffer = 10.0;
    } else if (fsd_tl_info.ego_is_in_leftwait) {
      time_buffer = 5.0;
    }
    if (unknown_time > time_buffer &&
        tld_state.change_to_unknown_time() > kEpsilon) {
      tld_state.set_unknown_check(true);
    }
  }

  if (!fsd_tl_info.ego_is_in_leftwait &&
      fsd_tl_info.dist_to_stopline < kEpsilon &&
      tld_state.override_count() > 3) {
    tld_state.set_override_passable(true);
  }

  if ((fsd_tl_info.dist_to_stopline > kEpsilon &&
       fsd_tl_info.dist_to_stopline < 30.0 && tld_state.override_count() > 4)) {
    tld_state.set_normal_override_passable(true);
  }

  if (tld_state.unknown_check() ||
      (is_fail_detection &&
       (fsd_tl_info.dist_to_stopline - config.normal_offset()) <
           std::fmax(start_v * 6.0, 10.0))) {
    if (tld_state.unknown_check()) {
      tld_state.set_tl_stop_interface(StopLineInterface::STOP_LINE_UNKNOWN);

    } else if (is_fail_detection) {
      tld_state.set_tl_stop_interface(StopLineInterface::STOP_LINE_FAIL_DETECT);
    }
    if (extern_button_ok) {
      tld_state.set_pass_junction(true);
    }

    if (is_fail_detection && fsd_tl_info.dist_to_stopline > kEpsilon &&
        fsd_tl_info.first_virtual_lane_is_leftwait &&
        fsd_tl_info.first_virtual_lane_light != LightStatus::FAIL_DETECTION) {
      tld_state.set_pass_junction(false);
      tld_state.set_tl_stop_interface(stop_reason);
    }
    if (tld_state.fail_detect_enter_leftwait()) {
      tld_state.set_pass_junction(false);
      tld_state.set_tl_stop_interface(stop_reason);
    }
    if (tld_state.pass_junction()) {
      tld_state.set_tl_stop_interface(StopLineInterface::STOP_LINE_NONE);
    }
  } else {
    tld_state.set_tl_stop_interface(stop_reason);
  }

  std::vector<ConstraintProto::StopLineProto> tl_stop_lines;
  std::vector<ConstraintProto::SpeedProfileProto> tl_speed_profiles;
  if (set_stopline) {
    const auto& curbs = passage.QueryCurbPointAtS(stop_s);
    if (!curbs.ok()) {
      TrafficLightDeciderStateProto state;
      output = TrafficLightDescriptor{
          .stop_lines = std::move(tl_stop_lines),
          .speed_profiles = std::move(tl_speed_profiles),
          .traffic_light_decision_state = std::move(state)};
      return false;
    }
    double adapt_tl_standoff = config.tl_standoff();
    if (start_v > Kph2Mps(config.tl_standoff_adapt_v())) {
      double dist_to_stop = fsd_tl_info.dist_to_stopline -
                            config.normal_offset() - front_edge_to_center_dis;
      double ttc = dist_to_stop / std::max(1e-6, start_v);
      adapt_tl_standoff =
          PiecewiseLinearFunctionFromProto(config.tl_standoff_ttc_plf())(ttc);
    }

    ConstraintProto::StopLineProto stop_line;
    stop_line.set_s(stop_s);
    stop_line.set_standoff(0.7);
    stop_line.set_time(0.0);
    stop_line.set_id(
        absl::StrFormat("traffic_light_%s", fsd_tl_info.focus_lane));
    stop_line.mutable_source()->mutable_traffic_light()->set_lane_id(
        fsd_tl_info.focus_lane);
    stop_line.mutable_source()->mutable_traffic_light()->set_id(
        fsd_tl_info.focus_lane);
    stop_line.set_is_traffic_light(true);
    HalfPlane halfplane(curbs->first, curbs->second);
    halfplane.ToProto(stop_line.mutable_half_plane());

    tl_stop_lines.push_back(std::move(stop_line));
    if (map_func_id == Behavior_FunctionId_LKA ||
        map_func_id == Behavior_FunctionId_MAPLESS_NOA) {
      ExtendStopLines(psmm, passage, fsd_tl_info.first_virtual_lane,
                      fsd_tl_info.focus_lane, tl_stop_lines, stop_s,
                      adapt_tl_standoff, need_precede);
    }

    std::vector<std::string> tl_output_debug;
    tl_output_debug.emplace_back("fsd-traffic-stop-s:" +
                                 std::to_string(stop_s));
  }
  tld_state.set_last_light_status(current_front_light);
  tld_state.set_valid_light_status(valid_front_light);
  tld_state.set_light_stop_reason(stop_reason);
  tld_state.set_dist_to_junction(fsd_tl_info.dist_to_stopline);
  tld_state.set_light_id(fsd_tl_info.focus_lane_light_info.id);
  tld_state.set_junction_lane_id(fsd_tl_info.focus_lane);
#if TLR_DEBUG
  DLOG(INFO) << "tld state current front light: " << int(current_front_light)
             << " valid front light: " << int(valid_front_light)
             << " stop reason: " << int(stop_reason)
             << " dist to junction: " << fsd_tl_info.dist_to_stopline
             << " stop line size = " << tl_stop_lines.size()
             << " stop s = " << stop_s << " light id = " << tld_state.light_id()
             << " original light id = " << fsd_tl_info.focus_lane_light_info.id
             << " focus lane = " << tld_state.junction_lane_id();
#endif
  TrafficLightDeciderStateProto state;
  auto mutable_fsd_tld_state = state.mutable_fsd_tld_state();
  *mutable_fsd_tld_state = tld_state;
  output =
      TrafficLightDescriptor{.stop_lines = std::move(tl_stop_lines),
                             .speed_profiles = std::move(tl_speed_profiles),
                             .traffic_light_decision_state = std::move(state)};
  return true;
}

absl::StatusOr<TrafficLightDescriptor> BuildTrafficLightConstraints(
    const e2e_noa::VehicleGeometryParamsProto& vehicle_geometry_params,
    const ApolloTrajectoryPointProto& plan_start_point,
    const PlanPassage& passage, const mapping::LanePath& lane_path_from_start,
    const TrafficLightStatusMap& traffic_light_status_map,
    const PlannerSemanticMapManager& psmm,
    const TrafficLightDeciderStateProto& tld_state,
    const bool& enable_tl_ok_btn, const bool& override_passable,
    const DecisionConstraintConfigProto& config,
    const Behavior_FunctionId& map_func_id,
    const SpacetimeTrajectoryManager& st_traj_mgr, const int plan_id,
    const bool traffic_light_fun_enable, bool& need_prebrake_for_lka) {
  TrafficLightDescriptor output;
  if (map_func_id == Behavior_FunctionId_LKA && !traffic_light_fun_enable) {
    return output;
  }
  if (!CurrentJunctionTrafficLightDecider(
          vehicle_geometry_params, plan_start_point, passage,
          lane_path_from_start, traffic_light_status_map, psmm, tld_state,
          enable_tl_ok_btn, override_passable, config, st_traj_mgr, output,
          map_func_id, plan_id, need_prebrake_for_lka)) {
    return output;
  }

  if (output.traffic_light_decision_state.fsd_tld_state().light_stop_reason() !=
      StopLineReason::REASON_NONE) {
    return output;
  }

  return output;
}
}  // namespace planning
}  // namespace e2e_noa
