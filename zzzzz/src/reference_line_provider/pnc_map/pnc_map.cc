//
// Copyright 2024 ZDrive.AI. All Rights Reserved.
//

#include "apps/planning/src/reference_line_provider/pnc_map/pnc_map.h"

#include <algorithm>
#include <limits>

#include "messages/map_service/all_map_new.pb.h"
#include "apps/planning/src/common/clock.h"
#include "apps/planning/src/common/log.h"
#include "math_utils.h"
#include "point_factory.h"
#include "string_util.h"
#include "util.h"
#include "apps/planning/src/config/conf_gflags.h"
#include "apps/planning/src/config/config_main.h"
#include "google/protobuf/text_format.h"

namespace zark {
namespace planning {
namespace hdmap {

using ::common::PointENU;
using common::VehicleState;
using ::util::PointFactory;
using zark::routing::RoutingResponse;
using namespace zark::map_service::zmap;

namespace {

// Maximum lateral error used in trajectory approximation.
const double kTrajectoryApproximationMaxError = 2.0;

}  // namespace

PncMap::PncMap() { init(); }

void PncMap::init() {
  new_hdmap_ = std::make_shared<zark::map_service::zmap::ZpilotMapInfo>();
  std::string config_file(PlanningGflags::local_route_path);
  zark::planning::Config config_instance{config_file};
  config_instance.SetLocalRouteConfig(local_route_config_);
}

bool PncMap::UpdateLocalHdMap(const MapInfo &new_local_map,
                              const bool &is_online_map) {
  update_main_path_ = false;
  if (!HDMapUtil::MapServicePtr()) {
    std::string error_msg = " map_service_not_construct";
    AERROR << error_msg;
    ClearNavInfoGrouop(error_msg);
    return false;
  }
  if (new_local_map.map_lane_size == 0) {
    std::string error_msg = " map_no_lane";
    AERROR << error_msg;
    ClearNavInfoGrouop(error_msg);
    return false;
  }
  map_type_ = new_local_map.map_type;
  AINFO << "map type: " << map_type_;
  if (local_route_config_.use_map_state &&
      map_type_ !=
          hdmap_new::MapHeader_MapType::MapHeader_MapType_MAP_TYPE_HDPRO &&
      map_type_ !=
          hdmap_new::MapHeader_MapType::MapHeader_MapType_MAP_TYPE_HDLITE &&
      map_type_ !=
          hdmap_new::MapHeader_MapType::MapHeader_MapType_MAP_TYPE_SD_HDPRO &&
      map_type_ !=
          hdmap_new::MapHeader_MapType::MapHeader_MapType_MAP_TYPE_SD_HDLITE &&
      map_type_ !=
          hdmap_new::MapHeader_MapType::MapHeader_MapType_MAP_TYPE_CP) {
    AERROR << "map type " << map_type_ << " not support!";
    std::string error_msg = "map_type_error";
    ClearNavInfoGrouop(error_msg);
    return false;
  }
  if (!local_route_config_.self_test_local_route &&
      new_local_map.map_path.size() == 0) {
    if (main_path_.road_section_id_size() > 0) {
      main_path_.Clear();
    }
    std::string error_msg = "map_no_path";
    AERROR << error_msg;
    ClearNavInfoGrouop(error_msg);
    return false;
  }
  if (last_map_timestamp_ != new_local_map.map_timestamp) {
    const double start_time = zark::common::Clock::NowInSeconds();
    last_map_timestamp_ = new_local_map.map_timestamp;
    if (HDMapUtil::MapServicePtr()->LockAndUpdata(true)) {
      std::unique_lock<std::mutex> map_service_lock(
          HDMapUtil::map_service_mutex_);
      HDMapUtil::MapServicePtr(false)->GetZpilotMapInfoPtr(&new_hdmap_);
      HDMapUtil::SetBaseMapPtr(new_hdmap_);
      HDMapUtil::MapServicePtr(false)->UnLock();
    }
    const double end_time = zark::common::Clock::NowInSeconds();
    AERROR << "update local_map, time diff: " << end_time - start_time;
    bool has_new_path = false;
    AINFO << "map path count: " << new_local_map.map_path.size();
    for (auto &path : new_local_map.map_path) {
      if (path.is_mpp()) {
        has_new_path = true;
        break;
      }
    }
    if (!is_online_map) {
      if (!has_new_path) {
        std::string error_msg = "map_no_main_path";
        AERROR << error_msg;
        ClearNavInfoGrouop(error_msg);
        return false;
      }
      if (new_local_map.guidance_info.section_size() == 0) {
        std::string error_msg = "map_no_guidance_info";
        AERROR << error_msg;
        ClearNavInfoGrouop(error_msg);
        return false;
      }
      guidance_info_.Clear();
      guidance_info_.CopyFrom(new_local_map.guidance_info);
    }
    main_path_.Clear();
    AINFO << "section in path count"
          << new_local_map.map_path.at(0).road_section_id().size();
    if (!is_online_map) {
      for (auto &road_section : new_local_map.guidance_info.section()) {
        main_path_.add_road_section_id()->CopyFrom(road_section.id());
      }
    } else {
      auto &road_section_id = new_local_map.map_path.at(0).road_section_id(0);
      main_path_.add_road_section_id()->CopyFrom(road_section_id);
    }
    const double routing_start_time = zark::common::Clock::NowInSeconds();
    bool res = updateRouting(main_path_);
    const double routing_end_time = zark::common::Clock::NowInSeconds();
    AERROR << "update routing, time diff: "
           << routing_end_time - routing_start_time;
    return res;
  }
  return true;
}

LaneWaypoint PncMap::ToLaneWaypoint(
    const zark::routing::LaneWaypoint &waypoint) const {
  hdmap_new::Id id;
  auto lane = GetLaneById(new_hdmap_, MakeMapId(waypoint.id()), false);
  ACHECK(lane) << "Invalid lane id: " << waypoint.id();
  return LaneWaypoint(lane, waypoint.s());
}

double PncMap::LookForwardDistance(double velocity) {
  const double kTimeBuffer = 0.5;
  auto forward_distance =
      velocity * (local_route_config_.look_forward_time_sec + kTimeBuffer);
  return std::fmax(forward_distance, 70.0);

  //   return forward_distance > local_route_config_.look_forward_short_distance
  //              ? local_route_config_.look_forward_long_distance
  //              : local_route_config_.look_forward_short_distance;
}

LaneSegment PncMap::ToLaneSegment(
    const zark::routing::LaneSegment &segment) const {
  auto lane = GetLaneById(new_hdmap_, MakeMapId(segment.id()), false);
  if (lane == nullptr) {
    AERROR << "GetLaneById FAILD , id: " << segment.id();
  }
  ACHECK(lane) << "Invalid lane id: " << segment.id();
  return LaneSegment(lane, segment.start_s(), segment.end_s());
}

void PncMap::UpdateNextRoutingWaypointIndex(int cur_index) {}

std::vector<zark::routing::LaneWaypoint> PncMap::FutureRouteWaypoints() const {
  const auto &waypoints = routing_.routing_request().waypoint();
  return std::vector<zark::routing::LaneWaypoint>(
      waypoints.begin() + next_routing_waypoint_index_, waypoints.end());
}

void PncMap::UpdateRoutingRange(int adc_index) {
  // Track routing range.
  range_lane_ids_.clear();
  range_start_ = std::max(0, adc_index - 1);
  range_end_ = range_start_;
  while (range_end_ < static_cast<int>(route_indices_.size())) {
    const auto &lane_id =
        route_indices_[range_end_].segment.lane->lane().id().id();
    if (range_lane_ids_.count(lane_id) != 0) {
      break;
    }
    range_lane_ids_.insert(lane_id);
    ++range_end_;
  }
}

bool PncMap::ForwardTriggerNoaLine(const std::list<NavSection> &sections,
                                   const LaneTopoInfo &current_lane,
                                   const VehicleState &vehicle_state,
                                   NoaLine &last_trigger_info,
                                   const bool &continue_check) {
  bool res = false;
  if (current_lane.remain_distance >
      local_route_config_.NoaLine_section_length +
          local_route_config_.noa_line_forward_dis) {
    return res;
  }
  //   hdmap_new::Id final_id;
  //   final_id.set_id(current_lane.final_id);
  //   auto final_lane = GetLaneById(HDMapUtil::BaseMapPtr(), final_id);
  //   std::string connect_id, final_section_id;
  //   if (final_lane) {
  //     final_section_id = final_lane->lane().road_section_id().id();
  //   } else {
  //     return res;
  //   }
  bool confirm_start = false;
  if (current_lane.lane_topo.id().id() == last_trigger_info.trigger_id) {
    return false;
  } else if (last_trigger_info.is_trigger) {
    // if target lane in forward ,keep trigger;
    int8_t cycle = 5;
    bool need_keep = false;
    for (auto section_iter = sections.begin(); section_iter != sections.end();
         section_iter++) {
      const auto &find_lane =
          section_iter->lanes.find(current_lane.lane_topo.id().id());
      if (find_lane != section_iter->lanes.end()) {
        std::string next_id = find_lane->second.lane_topo.id().id();
        for (const auto &successor_id :
             find_lane->second.lane_topo.successor_id()) {
          if (successor_id.id() == last_trigger_info.trigger_id) {
            need_keep = true;
            break;
          }
        }
        if (need_keep) {
          break;
        }
        auto next_section = std::next(section_iter);
        const auto &next_topo =
            section_iter->topo_range.priority_sucessor_range.find(
                find_lane->second.lane_sequence);
        if (next_topo ==
            section_iter->topo_range.priority_sucessor_range.end()) {
          need_keep = false;
          break;
        }
        next_id = next_topo->second;
        while (next_section != sections.end() && cycle > 0) {
          auto next_lane = next_section->lanes.find(next_id);
          if (next_lane != next_section->lanes.end()) {
            if (next_lane->second.lane_topo.successor_id().size() > 1 ||
                next_lane->second.lane_topo.successor_id().size() == 0) {
              need_keep = false;
              break;
            } else {
              next_id ==
                  next_lane->second.lane_topo.successor_id().begin()->id();
              if (next_id == last_trigger_info.trigger_id) {
                need_keep = true;
                break;
              }
            }
          } else {
            need_keep = false;
            break;
          }
          next_section = std::next(next_section);
          cycle--;
        }
        break;
      }
    }
    AINFO << "need_keep : " << need_keep;
    return need_keep;
  }
  std::string connect_id;
  ::math::Vec2d ego_point(vehicle_state.x(), vehicle_state.y());
  // TODO: when contniue trigger, second trigger forward start.
  const double forward_trigger_distance =
      continue_check ? local_route_config_.noa_line_forward_dis + 1.0
                     : local_route_config_.noa_line_forward_dis;
  for (const auto &section : sections) {
    if (section.section_id != current_lane.lane_topo.section_id().id()) {
      if (!confirm_start) continue;
    } else {
      confirm_start = true;
      if (current_lane.is_recommend) {
        const auto &lane_connect_topo =
            section.topo_range.priority_sucessor_range.find(
                current_lane.lane_sequence);
        if (lane_connect_topo !=
            section.topo_range.priority_sucessor_range.end()) {
          connect_id = lane_connect_topo->second;
          continue;
        } else {
          AERROR << "lane:" << current_lane.lane_topo.id().id()
                 << " is recommend, bunt find next lane falied";
          break;
        }
      }
      continue;
    }
    // check forward section:
    const auto &forward_lane = section.lanes.find(connect_id);
    if (forward_lane == section.lanes.end()) break;
    hdmap_new::Id map_lane_id;
    map_lane_id.set_id(connect_id);
    const auto &map_lane_info =
        GetLaneById(HDMapUtil::BaseMapPtr(), map_lane_id);
    double s = 0.0;
    double l = 0.0;

    if (!map_lane_info || !map_lane_info->GetProjection(ego_point, &s, &l) ||
        s < -forward_trigger_distance) {
      break;
    }
    if (forward_lane->second.is_recommend) {
      if (CheckSectionTrigger(section, forward_lane->second, vehicle_state,
                              forward_trigger_distance, last_trigger_info)) {
        res = true;
        break;
      } else {
        const auto &connect_lane =
            section.topo_range.priority_sucessor_range.find(
                forward_lane->second.lane_sequence);
        if (connect_lane != section.topo_range.priority_sucessor_range.end()) {
          connect_id = connect_lane->second;
          continue;
        } else {
          AERROR << "lane:" << forward_lane->first
                 << " is recommend, bunt find next lane falied";
          res = false;
          break;
        }
      }
    } else {
      //   if (section.end_s - section.start_s >
      //       RefereneLineGflags::NoaLine_section_length) {
      //     break;
      //   }
      if (CheckSectionTrigger(section, forward_lane->second, vehicle_state,
                              forward_trigger_distance, last_trigger_info)) {
        res = true;
      }
      break;
    }
    // if (section.section_id == final_section_id) {
    //   break;
    // }
  }
  return res;
}

bool PncMap::CheckSectionTrigger(const NavSection &section,
                                 const LaneTopoInfo &forward_lane,
                                 const VehicleState &vehicle_state,
                                 const double forward_trigger_distance,
                                 NoaLine &last_trigger_info) {
  bool res = false;
  //   if (forward_lane.remain_distance >
  //       RefereneLineGflags::NoaLine_section_length) {
  //     return res;
  //   }
  for (const auto &neighbor_lane : section.lanes) {
    if (neighbor_lane.second.remain_distance + kDistThreshold <=
            forward_lane.remain_distance ||
        neighbor_lane.second.change_count >= forward_lane.change_count) {
      continue;
    }
    if (neighbor_lane.second.lane_sequence == forward_lane.lane_sequence - 1 ||
        neighbor_lane.second.lane_sequence == forward_lane.lane_sequence + 1) {
      if (!neighbor_lane.second.is_recommend ||
          neighbor_lane.second.lane_topo.transition() !=
              hdmap_new::Lane_LaneTransition::
                  Lane_LaneTransition_LANE_TRANSITION_SPLIT) {
        break;
      }
      hdmap_new::Id neighbor_lane_id;
      neighbor_lane_id.set_id(neighbor_lane.first);
      auto map_lane_info =
          GetLaneById(HDMapUtil::BaseMapPtr(), neighbor_lane_id);
      if (!map_lane_info) break;
      double ego_s = 0.0;
      double ego_l = 0.0;
      if (!map_lane_info->GetProjection(
              ::math::Vec2d(vehicle_state.x(), vehicle_state.y()), &ego_s,
              &ego_l)) {
        break;
      }
      if (fabs(ego_s) <= forward_trigger_distance) {
        auto forward_lane_info =
            GetLaneById(HDMapUtil::BaseMapPtr(), forward_lane.lane_topo.id());
        if (!forward_lane_info) {
          break;
        }
        if (map_lane_info->lane().center_line().point().size() == 0 ||
            forward_lane_info->lane().center_line().point().size() == 0) {
          break;
        }
        ::math::Vec2d neighbor_start_point(
            map_lane_info->lane().center_line().point().begin()->x(),
            map_lane_info->lane().center_line().point().begin()->y());
        ::math::Vec2d forward_start_point(
            forward_lane_info->lane().center_line().point().begin()->x(),
            forward_lane_info->lane().center_line().point().begin()->y());
        if (neighbor_start_point.DistanceTo(forward_start_point) > 0.5) {
          continue;
        }
        res = true;
        if (neighbor_lane.second.lane_sequence ==
            forward_lane.lane_sequence - 1) {
          last_trigger_info.direction = -1;
        } else {
          last_trigger_info.direction = 1;
        }
        last_trigger_info.trigger_id = neighbor_lane.first;
        last_trigger_info.is_trigger = true;
      }
      break;
    }
  }
  return res;
}

bool PncMap::UpdateVehicleState(const VehicleState &vehicle_state,
                                const bool &is_online_map) {
  if (main_path_.road_section_id_size() == 0) {
    AERROR << "The routing is invalid when updating vehicle state.";
    error_msg_ = "main path no section";
    return false;
  }
  //   if ((::util::DistanceXY(adc_state_, vehicle_state) >
  //        PlanningGflags::replan_lateral_distance_threshold +
  //            PlanningGflags::replan_longitudinal_distance_threshold)) {
  //     // Position is reset, but not replan.
  //     adc_route_index_ = -1;
  //     stop_for_destination_ = false;
  //   }

  adc_state_ = vehicle_state;
  if (!GetNearestPointFromRouting(vehicle_state, &adc_waypoint_)) {
    AERROR << "Failed to get waypoint from routing with point: "
           << "(" << vehicle_state.x() << ", " << vehicle_state.y() << ", "
           << vehicle_state.z() << ").";
    return false;
  }
  int route_index = GetWaypointIndex(adc_waypoint_);
  if (route_index < 0 ||
      route_index >= static_cast<int>(route_indices_.size())) {
    AERROR << "Cannot find waypoint: " << adc_waypoint_.DebugString();
    error_msg_ = "route_index error";
    return false;
  }

  // Track how many routing request waypoints the adc have passed.
  // UpdateNextRoutingWaypointIndex(route_index);
  adc_route_index_ = route_index;
  UpdateRoutingRange(adc_route_index_);

  // TODO(@yfh): add calculate navigation lane change infomation
  if (!is_online_map) {
    const double start_time = zark::common::Clock::NowInSeconds();
    ExtractNavLanechangeInfo(guidance_info_, vehicle_state,
                             adc_waypoint_.lane->lane().road_section_id().id(),
                             adc_waypoint_.lane->lane().id().id(),
                             navigation_info_);
    const double end_time = zark::common::Clock::NowInSeconds();
    AINFO << " ExtractNavLanechangeInfo time diff ms: "
          << (end_time - start_time) * 1000;
    static const size_t kMaxHistoryInfo = 5;
    navigation_info_.ego_speed = vehicle_state.linear_velocity();
    navigation_info_.info_timestamp = end_time;
    for (const auto &section : navigation_info_.sections) {
      if (section.section_id ==
          adc_waypoint_.lane->lane().road_section_id().id()) {
        for (const auto &lane : section.lanes) {
          if (lane.first == adc_waypoint_.lane->lane().id().id()) {
            navigation_info_.current_lane_info = lane.second;
            ADEBUG << "CURRENT NAVI LANE " << lane.first << "   : CHANGE COUT  "
                   << int(lane.second.change_count)
                   << "  change dir :" << lane.second.change_direction
                   << "  remain_dist: " << lane.second.remain_distance
                   << "  non recommend remain: " << lane.second.remain_distance
                   << "  finale lane: " << lane.second.final_id;
            if (lane.second.be_merged) {
              ADEBUG << "merge lane id: " << lane.second.merged_id
                     << "  dir: " << lane.second.merge_dir
                     << " dis: " << lane.second.dist_merge_end;
            }
            if (lane.second.be_split) {
              ADEBUG << "split lane id: " << lane.second.split_id
                     << "  dir: " << lane.second.split_dir
                     << " dis: " << lane.second.dist_split_end;
            }
            break;
          }
        }
        break;
      }
    }
    {
      const double time_now = zark::common::Clock::NowInSeconds();
      bool keep_trigger = false;
      if (ForwardTriggerNoaLine(navigation_info_.sections,
                                navigation_info_.current_lane_info,
                                vehicle_state, last_trigger_info_) ||
          FollowTrigger(navigation_info_.sections,
                        navigation_info_.current_lane_info, last_trigger_info_,
                        vehicle_state, last_noa_line_.final_lane_id) ||
          (KeepTrigger(navigation_info_.sections,
                       navigation_info_.current_lane_info,
                       last_noa_line_.final_lane_id, time_now, vehicle_state,
                       keep_trigger, last_trigger_info_))) {
        navigation_info_.noa_line = last_trigger_info_;
        if (!keep_trigger) {
          last_trigger_info_.last_trigger_time =
              zark::common::Clock::NowInSeconds();
        }
      } else {
        navigation_info_.noa_line.ResetTriggerInfo();
        last_trigger_info_.ResetTriggerInfo();
      }
      std::lock_guard<std::mutex> lock(navinfo_lock_);
      navigation_info_group_.emplace_back(navigation_info_);
      if (navigation_info_group_.size() > kMaxHistoryInfo) {
        navigation_info_group_.pop_front();
      }
    }
  }

  return true;
}

bool PncMap::FollowTrigger(const std::list<NavSection> &sections,
                           const LaneTopoInfo &current_lane,
                           const NoaLine &last_trigger_info,
                           const VehicleState &ego_state,
                           const std::string &final_id) {
  bool res = false;
  if (current_lane.lane_topo.id().id() == last_trigger_info.trigger_id &&
      current_lane.lane_topo.transition() ==
          hdmap_new::Lane_LaneTransition::
              Lane_LaneTransition_LANE_TRANSITION_SPLIT) {
    res = true;
    auto left_boundary = GetLaneBoundaryById(
        HDMapUtil::BaseMapPtr(), adc_waypoint_.lane->lane().left_boundary_id());
    auto right_boundary =
        GetLaneBoundaryById(HDMapUtil::BaseMapPtr(),
                            adc_waypoint_.lane->lane().right_boundary_id());
    if (left_boundary && right_boundary) {
      ::math::Vec2d ego_point(ego_state.x(), ego_state.y());
      double s, left_l, right_l;
      if (left_boundary->GetProjection(ego_point, &s, &left_l) &&
          right_boundary->GetProjection(ego_point, &s, &right_l)) {
        if ((right_l - left_l) >= local_route_config_.normal_lane_width * 0.7 &&
            fabs(fabs(left_l) - fabs(right_l)) < 0.2) {
          res = false;
        }
        if (res && current_lane.lane_topo.id().id() == final_id) {
          if ((left_l < 0.0 && right_l > 0.0) &&
              (right_l - left_l) >
                  local_route_config_.normal_lane_width * 0.85) {
            NoaLine temp_trigger_info;
            if (ForwardTriggerNoaLine(sections, current_lane, ego_state,
                                      temp_trigger_info, true)) {
              AINFO << "stop current trigger, start check new trigger.";
              res = false;
            }
          }
        }
      }
    }
  }
  return res;
}

bool PncMap::KeepTrigger(const std::list<NavSection> &sections,
                         const LaneTopoInfo &current_lane, std::string final_id,
                         const double &time_now, const VehicleState &ego_state,
                         bool &is_keep, NoaLine &last_trigger_info) {
  const std::string cur_id = adc_waypoint_.lane->lane().id().id();
  if (time_now - last_trigger_info.last_trigger_time <=
      local_route_config_.keep_trigger_time + 1.5) {
    if (cur_id != last_trigger_info.trigger_id) {
      hdmap_new::Id follow_id;
      follow_id.set_id(last_trigger_info.trigger_id);
      const auto &trigger_lane =
          GetLaneById(HDMapUtil::BaseMapPtr(), follow_id);
      if (!trigger_lane) return true;
      // check right lane
      for (const auto &left_id :
           trigger_lane->lane().left_neighbor_forward_lane_id()) {
        if (left_id.id() == cur_id) {
          is_keep = false;
          AERROR << "current lane in noa line left, stop keep strigger.";
          return false;
        }
      }
      // check right lane
      for (const auto &right_id :
           trigger_lane->lane().right_neighbor_forward_lane_id()) {
        if (right_id.id() == cur_id) {
          is_keep = false;
          AERROR << "current lane in noa line right, stop keep strigger.";
          return false;
        }
      }
    }
    is_keep = true;
    auto left_boundary = GetLaneBoundaryById(
        HDMapUtil::BaseMapPtr(), adc_waypoint_.lane->lane().left_boundary_id());
    auto right_boundary =
        GetLaneBoundaryById(HDMapUtil::BaseMapPtr(),
                            adc_waypoint_.lane->lane().right_boundary_id());
    if (left_boundary && right_boundary) {
      ::math::Vec2d ego_point(ego_state.x(), ego_state.y());
      double s, left_l, right_l;
      if (left_boundary->GetProjection(ego_point, &s, &left_l) &&
          right_boundary->GetProjection(ego_point, &s, &right_l)) {
        if ((right_l - left_l) >= local_route_config_.normal_lane_width * 0.7 &&
            fabs(fabs(left_l) - fabs(right_l)) <= 0.2) {
          if (time_now - last_trigger_info.last_trigger_time >= 1.0) {
            is_keep = false;
            AINFO << "cancel keep trigger.";
          }
        }
      }
      if (is_keep && cur_id == final_id) {
        if ((left_l < 0.0 && right_l > 0.0) &&
            (right_l - left_l) > local_route_config_.normal_lane_width * 0.85) {
          NoaLine temp_trigger_info;
          if (ForwardTriggerNoaLine(sections, current_lane, ego_state,
                                    temp_trigger_info, true)) {
            AINFO << "stop current trigger, start check new trigger.";
            is_keep = false;
          }
        }
      }
    }
    return is_keep;
  }
  return false;
}

bool PncMap::IsNewRouting(const zark::routing::RoutingResponse &routing) const {
  return IsNewRouting(routing_, routing);
}

bool PncMap::IsNewRouting(const zark::routing::RoutingResponse &prev,
                          const zark::routing::RoutingResponse &routing) {
  if (!ValidateRouting(routing)) {
    ADEBUG << "The provided routing is invalid.";
    return false;
  }
  return false;
}

bool PncMap::updateRouting(const zark::hdmap_new::Path &routing_path) {
  if (routing_path.road_section_id_size() == 0) {
    AERROR << "route path is empty, please check map msg!!!!";
    // for simulation scenario without path, use self path
    if (local_route_config_.self_test_local_route) {
      for (int i = 0; i < 4; i++) {
        if (i == 0) {
          hdmap_new::Id *id = main_path_.add_road_section_id();
          id->set_id("4295702826_0");
        } else if (i == 1) {
          hdmap_new::Id *id = main_path_.add_road_section_id();
          id->set_id("4295709350_0");
        } else if (i == 2) {
          hdmap_new::Id *id = main_path_.add_road_section_id();
          id->set_id("8589934592_0");
        } else if (i == 3) {
          hdmap_new::Id *id = main_path_.add_road_section_id();
          id->set_id("8589938745_0");
        }
      }
    } else {
      error_msg_ = "main_path no section";
      return false;
    }
  }
  AERROR << " ROAD SECTION SIZE: " << main_path_.road_section_id_size();
  return UpdateRoutingResponse(main_path_);
}

bool PncMap::CheckLaneType(const int32_t &type) {
  if ((type & hdmap_new::Lane_LaneType::Lane_LaneType_LANE_TYPE_UNKNOWN) ||
      (type &
       hdmap_new::Lane::LaneType::Lane_LaneType_LANE_TYPE_EMERGENCY_LANE) ||
      (type & hdmap_new::Lane::LaneType::Lane_LaneType_LANE_TYPE_SIDEWALK) ||
      (type &
       hdmap_new::Lane::LaneType::Lane_LaneType_LANE_TYPE_SHOULDER_LANE) ||
      (type &
       hdmap_new::Lane::LaneType::Lane_LaneType_LANE_TYPE_ISOLATED_LANE) ||
      (type & hdmap_new::Lane::LaneType::
                  Lane_LaneType_LANE_TYPE_CUSTOMS_SUPERVISION_LANE) ||
      (type & hdmap_new::Lane::LaneType::
                  Lane_LaneType_LANE_TYPE_EMERGENCY_PARKING_LANE)) {
    return false;
  }
  return true;
}

bool PncMap::UpdateRoutingResponse(const zark::hdmap_new::Path &main_path) {
  if (main_path.road_section_id_size() == 0) {
    AERROR << "no routing.";
    error_msg_ = "main_path no section";
    return false;
  }
  range_lane_ids_.clear();
  route_indices_.clear();
  all_lane_ids_.clear();

  for (int road_section_index = 0;
       road_section_index < main_path.road_section_id().size();
       ++road_section_index) {
    const auto section_id = main_path.road_section_id().at(road_section_index);
    auto section_info = GetRoadSectionById(new_hdmap_, section_id, false);
    if (!section_info) {
      ADEBUG << "find section failed, id: " << section_id.id();
      continue;
    }
    AINFO << "find section success, id: " << section_id.id();
    for (int passage_index = 0;
         passage_index < section_info->road_section().lane_id().size();
         ++passage_index) {
      const auto passage =
          section_info->road_section().lane_id().at(passage_index);
      auto lane = GetLaneById(new_hdmap_, passage, false);
      if (lane == nullptr) {
        AWARN << "GetLaneById FAILD , id: " << passage.id();
        continue;
      }
      if (CheckLaneType(lane->lane().type())) {
        all_lane_ids_.insert(passage.id());
      } else {
        ADEBUG << "lane is not drivable, type: " << lane->lane().type()
               << " id: " << lane->lane().id().id();
        continue;
      }
      ACHECK(lane) << "Invalid lane id: " << passage.id();
      route_indices_.emplace_back();
      LaneSegment seg(lane, 0.0, lane->total_length());
      route_indices_.back().segment = seg;
      route_indices_.back().index = {road_section_index, passage_index, 0};
    }
  }

  if (route_indices_.size() == 0 || all_lane_ids_.size() == 0) {
    AERROR << " updata routing error, route_indices_ size: "
           << route_indices_.size()
           << "  all_lane_ids size: " << all_lane_ids_.size();
    error_msg_ = "updata main_path error";
    return false;
  }

  range_start_ = 0;
  range_end_ = 0;
  adc_route_index_ = -1;
  next_routing_waypoint_index_ = 0;
  UpdateRoutingRange(adc_route_index_);
  adc_waypoint_ = LaneWaypoint();
  stop_for_destination_ = false;
  update_main_path_ = true;
  return true;
}

const zark::routing::RoutingResponse &PncMap::routing_response() const {
  return routing_;
}

bool PncMap::ValidateRouting(const zark::routing::RoutingResponse &routing) {
  const int num_road = routing.road().size();
  if (num_road == 0) {
    AERROR << "Route is empty.";
    return false;
  }
  return true;
}

int PncMap::SearchForwardWaypointIndex(int start,
                                       const LaneWaypoint &waypoint) const {
  int i = std::max(start, 0);
  while (
      i < static_cast<int>(route_indices_.size()) &&
      !RouteSegments::WithinLaneSegment(route_indices_[i].segment, waypoint)) {
    ++i;
  }
  return i;
}

int PncMap::SearchBackwardWaypointIndex(int start,
                                        const LaneWaypoint &waypoint) const {
  int i = std::min(static_cast<int>(route_indices_.size() - 1), start);
  while (i >= 0 && !RouteSegments::WithinLaneSegment(route_indices_[i].segment,
                                                     waypoint)) {
    --i;
  }
  return i;
}

int PncMap::NextWaypointIndex(int index) const {
  if (index >= static_cast<int>(route_indices_.size() - 1)) {
    return static_cast<int>(route_indices_.size()) - 1;
  } else if (index < 0) {
    return 0;
  } else {
    return index + 1;
  }
}

int PncMap::GetWaypointIndex(const LaneWaypoint &waypoint) const {
  int forward_index = SearchForwardWaypointIndex(adc_route_index_, waypoint);
  if (forward_index >= static_cast<int>(route_indices_.size())) {
    return SearchBackwardWaypointIndex(adc_route_index_, waypoint);
  }
  if (forward_index == adc_route_index_ ||
      forward_index == adc_route_index_ + 1) {
    return forward_index;
  }
  auto backward_index = SearchBackwardWaypointIndex(adc_route_index_, waypoint);
  if (backward_index < 0) {
    return forward_index;
  }

  return (backward_index + 1 == adc_route_index_) ? backward_index
                                                  : forward_index;
}

bool PncMap::PassageToSegments(const hdmap_new::Id &passage_lane_id,
                               RouteSegments *segments) const {
  CHECK_NOTNULL(segments);
  segments->clear();
  zmap::LaneInfoConstPtr lane_ptr =
      GetLaneById(new_hdmap_, passage_lane_id, false);
  if (!lane_ptr) {
    AERROR << "Failed to find lane: " << passage_lane_id.id();
    return false;
  }
  segments->emplace_back(lane_ptr, 0.0, lane_ptr->total_length());
  return !segments->empty();
}

std::vector<std::pair<int, RouteSegments::SegmentType>>
PncMap::GetNeighborPassages(const zmap::RoadSectionInfoConstPtr &road_section,
                            int start_passage) const {
  CHECK_LE(start_passage, road_section->road_section().lane_id().size());
  std::vector<std::pair<int, RouteSegments::SegmentType>> result;
  const auto &source_passage =
      road_section->road_section().lane_id().at(start_passage);
  result.emplace_back(std::make_pair<>(
      start_passage, RouteSegments::SegmentType::CurrentSegment));
  RouteSegments source_segments;
  if (!PassageToSegments(source_passage, &source_segments)) {
    AERROR << "Failed to convert passage to segments";
    return result;
  }
  std::unordered_set<std::string> left_neighbor_lanes, right_neighbor_lanes;
  for (const auto &segment : source_segments) {
    for (const auto &left_id :
         segment.lane->lane().left_neighbor_forward_lane_id()) {
      if (left_neighbor_lanes.count(left_id.id())) {
        continue;
      }
      if (!range_lane_ids_.count(left_id.id())) {
        continue;
      }
      left_neighbor_lanes.insert(left_id.id());
      break;  // only add one
    }
  }
  for (const auto &segment : source_segments) {
    for (const auto &right_id :
         segment.lane->lane().right_neighbor_forward_lane_id()) {
      if (right_neighbor_lanes.count(right_id.id())) {
        continue;
      }
      if (!range_lane_ids_.count(right_id.id())) {
        continue;
      }
      right_neighbor_lanes.insert(right_id.id());
      break;  // only add one
    }
  }

  for (int i = 0; i < road_section->road_section().lane_id().size(); ++i) {
    if (i == start_passage) {
      continue;
    }
    const auto &target_passage = road_section->road_section().lane_id().at(i);
    if (left_neighbor_lanes.count(target_passage.id())) {
      result.emplace_back(
          std::make_pair<>(i, RouteSegments::SegmentType::LeftSegment));
    } else if (right_neighbor_lanes.count(target_passage.id())) {
      result.emplace_back(
          std::make_pair<>(i, RouteSegments::SegmentType::RightSegment));
    }
  }
  return result;
}

bool PncMap::GetRouteSegments(const VehicleState &vehicle_state,
                              std::list<RouteSegments> *const route_segments,
                              const bool &is_online_map) {
  double look_forward_distance =
      LookForwardDistance(vehicle_state.linear_velocity());
  double look_backward_distance = local_route_config_.look_backward_distance;
  return GetRouteSegments(vehicle_state, look_backward_distance,
                          look_forward_distance, route_segments, is_online_map);
}

bool PncMap::GetRouteSegments(const VehicleState &vehicle_state,
                              const double backward_length,
                              const double forward_length,
                              std::list<RouteSegments> *const route_segments,
                              const bool &is_online_map) {
  if (!UpdateVehicleState(vehicle_state, is_online_map)) {
    AERROR << "Failed to update vehicle state in pnc_map.";
    return false;
  }
  // Vehicle has to be this close to lane center before considering change
  // lane
  if (!adc_waypoint_.lane || adc_route_index_ < 0 ||
      adc_route_index_ >= static_cast<int>(route_indices_.size())) {
    AERROR << "Invalid vehicle state in pnc_map, update vehicle state first.";
    error_msg_ = "Invalid vehicle state in pnc_map";
    return false;
  }
  const auto &route_index = route_indices_[adc_route_index_].index;
  const int road_section_index = route_index[0];
  const int passage_index = route_index[1];
  const auto &road_section_id =
      main_path_.road_section_id().at(road_section_index);
  // Raw filter to find all neighboring passages
  auto section_info = GetRoadSectionById(new_hdmap_, road_section_id, false);
  auto drive_passages = GetNeighborPassages(section_info, passage_index);
  for (const auto &index : drive_passages) {
    const auto &passage =
        section_info->road_section().lane_id().at(index.first);
    RouteSegments segments;
    if (!PassageToSegments(passage, &segments)) {
      AINFO << "Failed to convert passage to lane segments.";
      continue;
    }
    AINFO << "segments: size: " << segments.size();
    AINFO << "segments[0] :" << segments[0].DebugString();
    ::math::Vec2d nearest_point;
    if (index.first == passage_index) {
      auto map_point = adc_waypoint_.lane->GetSmoothPoint(adc_waypoint_.s);
      nearest_point.set_x(map_point.x());
      nearest_point.set_y(map_point.y());
    } else {
      nearest_point.set_x(adc_state_.x());
      nearest_point.set_y(adc_state_.y());
    }
    ::common::SLPoint sl;
    LaneWaypoint segment_waypoint;
    if (!segments.GetProjection(nearest_point, &sl, &segment_waypoint)) {
      AINFO << "Failed to get projection from point: ";
      continue;
    }
    if (index.first != passage_index) {
      if (!segments.CanDriveFrom(adc_waypoint_, is_online_map)) {
        ADEBUG << "You cannot drive from current waypoint to passage: "
               << index.first;
        continue;
      }
    }
    segments.SetSegmentType(index.second);
    route_segments->emplace_back();
    const auto last_waypoint = segments.LastWaypoint();
    if (!ExtendSegments(segments, sl.s() - backward_length,
                        sl.s() + forward_length, &route_segments->back())) {
      AERROR << "Failed to extend segments with s=" << sl.s()
             << ", backward: " << backward_length
             << ", forward: " << forward_length;
      error_msg_ = "Failed to extend segments";
      return false;
    }
    if (route_segments->back().IsWaypointOnSegment(last_waypoint)) {
      AINFO << "Last waypoint is on segment, set route end waypoint";
      route_segments->back().SetRouteEndWaypoint(last_waypoint);
    }
    std::string ss =
        std::to_string(road_section_index) + "_" + std::to_string(index.first);
    const std::string route_segment_id = ss;
    AINFO << "route segment id: " << ss;
    route_segments->back().SetId(route_segment_id);
    route_segments->back().SetStopForDestination(stop_for_destination_);
    route_segments->back().SetSegmentType(index.second);
    if (index.first == passage_index) {
      route_segments->back().SetPreviousAction(
          zark::routing::ChangeLaneType::FORWARD);
    } else if (sl.l() > 0) {
      route_segments->back().SetPreviousAction(
          zark::routing::ChangeLaneType::RIGHT);
    } else {
      route_segments->back().SetPreviousAction(
          zark::routing::ChangeLaneType::LEFT);
    }
  }
  if (!route_segments->empty()) {
    error_msg_.clear();
  } else {
    error_msg_ = "route_segments_empty";
  }
  last_confirm_lanes_.clear();
  if (route_segments->size() > 0) {
    for (const auto &seg : *route_segments->begin()) {
      last_confirm_lanes_.emplace(seg.lane->lane().id().id());
    }
  }
  AINFO << "route_segments size: " << route_segments->size();
  for (auto &rs : *route_segments) {
    AINFO << "route_segment id: " << rs.Id();
  }
  return !route_segments->empty();
}

void PncMap::CheckLastIdentifyLane(const LaneWaypoint &check_lane,
                                   double &distance) {
  // if check lane has been picked, decrease the distance between check lane and
  // ego car to increase stability of current lane pick at lane change scenario;
  if (check_lane.lane == nullptr || last_confirm_lanes_.empty()) {
    return;
  }
  const double kLCDistThreshold = 0.30;  //[m]
  if (last_confirm_lanes_.count(check_lane.lane->lane().id().id()) > 0) {
    distance = std::fmax(0.0, distance - kLCDistThreshold);
  }
}

bool PncMap::GetNearestPointFromRouting(const VehicleState &state,
                                        LaneWaypoint *waypoint) {
  static constexpr double kMaxLonDisLane = 2.0;
  static constexpr double kMaxLatDisLane = 3.5;
  waypoint->lane = nullptr;
  std::vector<LaneInfoConstPtr> lanes;
  const auto point = PointFactory::ToPointENU(state);
  std::vector<zmap::LaneInfoConstPtr> valid_lanes;
  for (auto lane_id : all_lane_ids_) {
    zark::hdmap_new::Id id = MakeMapId(lane_id);
    auto lane = GetLaneById(new_hdmap_, id, false);
    if (nullptr != lane) {
      valid_lanes.emplace_back(lane);
    }
  }

  // Get nearest_waypoints for current position
  std::vector<LaneWaypoint> valid_way_points;
  for (const auto &lane : valid_lanes) {
    if (range_lane_ids_.count(lane->lane().id().id()) == 0) {
      continue;
    }
    double s = 0.0;
    double l = 0.0;
    {
      if (!lane->GetProjection({point.x(), point.y()}, &s, &l)) {
        continue;
      }
      // this value must less than kCurSegmentationEpsilon in route_segments.cc
      static constexpr double kEpsilon = 0.5;
      if (s > (lane->total_length() + kEpsilon) || (s + kEpsilon) < 0.0) {
        continue;
      }
    }

    valid_way_points.emplace_back();
    auto &last = valid_way_points.back();
    last.lane = lane;
    last.s = s;
    last.l = l;
    ADEBUG << "distance:" << std::fabs(l);
  }
  if (valid_way_points.empty()) {
    std::string error_msg = "valid_way_points_empty";
    AERROR << error_msg;
    SetErrorMsg(error_msg);
    return false;
  }

  // Choose the lane with the right heading if there is more than one candiate
  // lanes. If there is no lane with the right heading, choose the closest one.
  size_t closest_index = 0;
  int right_heading_index = -1;
  // The distance as the sum of the lateral and longitude distance, to estimate
  // the distance from the vehicle to the lane.
  double distance = std::numeric_limits<double>::max();
  double lane_heading = 0.0;
  double vehicle_heading = state.heading();
  std::vector<hdmap::LaneWaypoint> range_way_points;
  for (size_t i = 0; i < valid_way_points.size(); i++) {
    double distance_to_lane = std::fabs(valid_way_points[i].l);
    if (valid_way_points[i].s > valid_way_points[i].lane->total_length()) {
      distance_to_lane +=
          (valid_way_points[i].s - valid_way_points[i].lane->total_length());
    } else if (valid_way_points[i].s < 0.0) {
      distance_to_lane -= valid_way_points[i].s;
    }

    CheckLastIdentifyLane(valid_way_points[i], distance_to_lane);
    if (distance > distance_to_lane) {
      distance = distance_to_lane;
      closest_index = i;
    }
    lane_heading = valid_way_points[i].lane->Heading(valid_way_points[i].s);
    if (std::abs(::math::AngleDiff(lane_heading, vehicle_heading)) < M_PI_2) {
      // Choose the lane with the closest distance to the vehicle and with the
      // right heading.
      if (-1 == right_heading_index || closest_index == i) {
        waypoint->lane = valid_way_points[i].lane;
        waypoint->s = valid_way_points[i].s;
        waypoint->l = valid_way_points[i].l;
        right_heading_index = i;
      }
      float lateral_diff = (last_trigger_info_.is_trigger &&
                            last_confirm_lanes_.count(
                                valid_way_points[i].lane->lane().id().id()))
                               ? local_route_config_.normal_lane_width + 0.25
                               : local_route_config_.latral_select_distance;
      if (valid_way_points[i].s < valid_way_points[i].lane->total_length() &&
          (valid_way_points[i].s >= 0.0 ||
           LonLaneSelect(valid_way_points[i], state)) &&
          fabs(valid_way_points[i].l) <= lateral_diff) {
        range_way_points.emplace_back(valid_way_points[i]);
      }
    }
  }
  // Use the lane with the closest distance to the current position of the
  // vehicle.
  if (-1 == right_heading_index) {
    waypoint->lane = valid_way_points[closest_index].lane;
    waypoint->s = valid_way_points[closest_index].s;
    waypoint->l = valid_way_points[closest_index].l;
    AWARN << "Find no lane with the right heading, use the cloesest lane! :"
          << waypoint->lane->lane().id().id();
  }
  AERROR << "final lane :" << waypoint->lane->lane().id().id();
  {
    // check final lane distance:
    double s = 0.0;
    double l = 0.0;
    {
      if (!waypoint->lane->GetProjection({point.x(), point.y()}, &s, &l)) {
        std::string error_msg = "proj_lane_failed";
        AERROR << error_msg;
        SetErrorMsg(error_msg);
        return false;
      }
      if (s > (waypoint->lane->total_length() + kMaxLonDisLane) ||
          (s + kMaxLonDisLane) < 0.0 || fabs(l) > kMaxLatDisLane) {
        AERROR << "final lane distance ego far, ego s: " << s
               << "  ego l: " << l
               << "  lane length: " << waypoint->lane->total_length();

        std::string error_msg = "lane_dist_ego_error";
        SetErrorMsg(error_msg);
        return false;
      }
    }
  }
  if (last_trigger_info_.is_trigger && range_way_points.size() == 1 &&
      waypoint->lane &&
      waypoint->lane->lane().id().id() !=
          range_way_points.begin()->lane->lane().id().id()) {
    range_way_points.emplace_back(*waypoint);
  }
  ADEBUG << "range_way_points.size()" << range_way_points.size();
  if (range_way_points.size() > 1) {
    // process split lane select,find max remain_distance lane, min_count lane;
    uint8_t min_change_num = kMaxLaneSize;
    double max_remmain_distance = -1.0;
    double min_heading_diff = M_PI * 2;
    bool has_max_remain = false;
    bool has_min_count = false;
    bool has_heading_select = false;
    bool last_connected = false;
    hdmap::LaneWaypoint distance_select_lane, count_select_lane,
        last_connected_lane, heading_select_lane;
    bool max_remain_is_continue = false;
    bool min_count_is_continue = false;
    for (const auto &way_point : range_way_points) {
      ADEBUG << "  RANGE POINT ID: " << way_point.lane->lane().id().id()
             << "   L: " << way_point.l;
      for (const auto &section : navigation_info_.sections) {
        if (section.section_id ==
            way_point.lane->lane().road_section_id().id()) {
          const auto nav_lane =
              section.lanes.find(way_point.lane->lane().id().id());
          if (nav_lane != section.lanes.end()) {
            const double lane_remain_distance =
                nav_lane->second.remain_distance;
            const uint8_t change_count = nav_lane->second.change_count;
            bool non_noaline =
                nav_lane->second.lane_topo.transition() ==
                        hdmap_new::Lane_LaneTransition::
                            Lane_LaneTransition_LANE_TRANSITION_CONTINUE
                    ? true
                    : false;
            if (lane_remain_distance >
                max_remmain_distance * (1 + kDistExceedRatio)) {
              max_remain_is_continue = non_noaline;
              if (max_remmain_distance > 0.0) {
                ADEBUG << "HAS_MAX REMAIN 1: " << lane_remain_distance;
                has_max_remain = true;
              }
              max_remmain_distance = lane_remain_distance;
              distance_select_lane = way_point;
            } else if (lane_remain_distance * (1 + kDistExceedRatio) <
                       max_remmain_distance) {
              ADEBUG << "HAS_MAX REMAIN 2: " << lane_remain_distance;
              has_max_remain = true;
            }
            if (change_count < min_change_num) {
              min_count_is_continue = non_noaline;
              if (min_change_num < kMaxLaneSize) {
                ADEBUG << "HAS_min_change_num 1: " << change_count;
                has_min_count = true;
              }
              min_change_num = change_count;
              count_select_lane = way_point;
            } else if (change_count > min_change_num) {
              ADEBUG << "HAS_min_change_num 2: " << change_count;
              has_min_count = true;
            }
            if ((nav_lane->second.lane_topo.transition() !=
                     hdmap_new::Lane_LaneTransition::
                         Lane_LaneTransition_LANE_TRANSITION_SPLIT ||
                 last_trigger_info_.is_trigger) &&
                nav_lane->second.lane_topo.transition() !=
                    hdmap_new::Lane_LaneTransition::
                        Lane_LaneTransition_LANE_TRANSITION_MERGE &&
                way_point.lane->lane().center_line().point().size() >=
                    kMinPointSize) {
              const auto &second_point =
                  way_point.lane->lane().center_line().point().at(1);
              const auto &first_point =
                  way_point.lane->lane().center_line().point().at(0);
              double heading_diff = ::math::NormalizeAngle(
                  std::atan2(second_point.y() - first_point.y(),
                             second_point.x() - first_point.x()));
              if (fabs(heading_diff) < fabs(min_heading_diff)) {
                min_heading_diff = heading_diff;
                heading_select_lane = way_point;
                has_heading_select = true;
              }
            }
            if (last_confirm_lanes_.count(way_point.lane->lane().id().id())) {
              last_connected = true;
              last_connected_lane = way_point;
              ADEBUG << "confirm from last connected : "
                     << way_point.lane->lane().id().id();
            }
          }
          break;
        }
      }
    }
    if (has_max_remain &&
        (last_trigger_info_.is_trigger || max_remain_is_continue)) {
      ADEBUG << "FINAL has_max_remain";
      *waypoint = distance_select_lane;
    } else if (has_min_count &&
               (last_trigger_info_.is_trigger || min_count_is_continue)) {
      ADEBUG << "FINAL has_min_count";
      *waypoint = count_select_lane;
    } else if (last_connected) {
      *waypoint = last_connected_lane;
      ADEBUG << "FINAL last_connected_lane:"
             << last_connected_lane.lane->lane().id().id();
    } else if (has_heading_select) {
      *waypoint = heading_select_lane;
      ADEBUG << "FINAL has_heading_select";
    }
  }
  if (waypoint->lane) {
    // process merge lane select
    MergeCurLaneSelect(state, waypoint);
  }
  AINFO << "FINAL FUSION LANE: " << waypoint->lane->lane().id().id();
  return true;
}

bool PncMap::LonLaneSelect(const hdmap::LaneWaypoint &lane_way_point,
                           const VehicleState &ego_state) {
  if (lane_way_point.lane == nullptr ||
      last_confirm_lanes_.count(lane_way_point.lane->lane().id().id()) == 0) {
    return false;
  }
  const double kLonSelectDist = 0.20;
  if (lane_way_point.s >= 0.0 || lane_way_point.s <= -kLonSelectDist) {
    return false;
  }
  for (const hdmap_new::Id predecssor_id :
       lane_way_point.lane->lane().predecessor_id()) {
    if (last_confirm_lanes_.count(predecssor_id.id()) > 0) {
      const auto predeccssor_lane = GetLaneById(new_hdmap_, predecssor_id);
      double pred_s, pred_l;
      if (predeccssor_lane != nullptr &&
          predeccssor_lane->GetProjection({ego_state.x(), ego_state.y()},
                                          &pred_s, &pred_l)) {
        if (pred_s < predeccssor_lane->total_length() || pred_s < 0.0) {
          return false;
        }
        AINFO << "pass predecessor, select lane: "
              << lane_way_point.lane->lane().id().id()
              << " s: " << lane_way_point.s;
        return true;
      }
      break;
    }
  }
  return false;
}

void PncMap::MergeCurLaneSelect(const VehicleState &state,
                                LaneWaypoint *waypoint) {
  const float kHalfWidthBuffer = 0.10;
  for (const auto &section : navigation_info_.sections) {
    if (section.section_id == waypoint->lane->lane().road_section_id().id()) {
      const auto &nvi_lane =
          section.lanes.find(waypoint->lane->lane().id().id());
      if (nvi_lane == section.lanes.end() ||
          nvi_lane->second.lane_topo.transition() !=
              hdmap_new::Lane_LaneTransition::
                  Lane_LaneTransition_LANE_TRANSITION_MERGE) {
        break;
      }
      const uint8_t neighbor_lane_seq =
          nvi_lane->second.lane_sequence > 1
              ? nvi_lane->second.lane_sequence - 1
              : nvi_lane->second.lane_sequence + 1;
      for (const auto &neighbor_navi_lane : section.lanes) {
        if (neighbor_navi_lane.second.lane_sequence == neighbor_lane_seq) {
          hdmap_new::Id map_id;
          map_id.set_id(neighbor_navi_lane.first);
          const auto &map_lane_info =
              GetLaneById(HDMapUtil::BaseMapPtr(), map_id);
          if (!map_lane_info) break;
          double ego_s = 0.0;
          double ego_l = 0.0;
          if (map_lane_info->GetProjection(::math::Vec2d(state.x(), state.y()),
                                           &ego_s, &ego_l)) {
            const auto lane_width_info = map_lane_info->GetWidth(ego_s);
            float compare_half_width =
                neighbor_lane_seq > nvi_lane->second.lane_sequence
                    ? lane_width_info.right_width_cm()
                    : lane_width_info.left_width_cm();
            if (fabs(ego_l) + kHalfWidthBuffer <=
                compare_half_width * kCentimeterToMetre) {
              waypoint->lane = map_lane_info;
              waypoint->s = ego_s;
              waypoint->l = ego_l;
              AINFO << "cross merge lane, select neighbor lane as current "
                       "lane, half width: "
                    << compare_half_width << "  ego_l: " << ego_l;
            }
          }
          break;
        }
      }
      break;
    }
  }
}

zmap::LaneInfoConstPtr PncMap::GetRouteSuccessor(
    zmap::LaneInfoConstPtr lane, const RouteSegments::SegmentType &type) const {
  if (lane->lane().successor_id().empty()) {
    return nullptr;
  }
  zark::hdmap_new::Id preferred_id;
  preferred_id.set_id("no_routing_sucessor_id");
  if (lane->lane().successor_id_size() == 1) {
    const hdmap_new::Id &next_id = lane->lane().successor_id().at(0);
    if (range_lane_ids_.count(next_id.id()) != 0) {
      preferred_id = next_id;
    }
  } else {
    // need process multi successor ;
    double lane_heading = 0.0;
    double max_remmain_distance = -1;
    uint8_t min_change_num = kMaxLaneSize;
    double min_heading_diff = M_PI * 2;
    hdmap_new::Id min_heading_diff_id, min_change_id, max_remain_dist_id;
    if (!lane->lane().center_line().heading_degree().empty()) {
      double heading_degree =
          *lane->lane().center_line().heading_degree().rbegin();
      lane_heading = heading_degree * M_PI / 180.0;
    }
    bool find_section = false;
    NavSection next_section;
    uint8_t remain_lane_index = 0;
    uint8_t count_lane_index = 0;
    bool has_max_remain = false;
    bool has_min_count = false;
    bool max_remain_is_continue = false;
    bool min_count_is_continue = false;
    for (const auto &lane_id : lane->lane().successor_id()) {
      // if lane has multiple successor lanes, mark the minimum heading
      // difference lane, max remmain distance lane and min change count lane;
      if (range_lane_ids_.count(lane_id.id()) == 0) continue;
      auto next_lane = GetLaneById(new_hdmap_, lane_id, false);
      if (next_lane) {
        if (!find_section) {
          for (const auto &section : navigation_info_.sections) {
            if (next_lane->lane().road_section_id().id() ==
                section.section_id) {
              find_section = true;
              next_section = section;
              break;
            }
          }
        }
        if (find_section) {
          const auto next_navi_lane = next_section.lanes.find(lane_id.id());
          if (next_navi_lane != next_section.lanes.end()) {
            if (type != RouteSegments::SegmentType::CurrentSegment &&
                next_navi_lane->second.lane_topo.transition() ==
                    hdmap_new::Lane_LaneTransition::
                        Lane_LaneTransition_LANE_TRANSITION_SPLIT) {
              continue;
            }
            bool non_noaline =
                next_navi_lane->second.lane_topo.transition() ==
                        hdmap_new::Lane_LaneTransition::
                            Lane_LaneTransition_LANE_TRANSITION_CONTINUE
                    ? true
                    : false;
            const double lane_remain_distance =
                next_navi_lane->second.remain_distance;
            const uint8_t change_count = next_navi_lane->second.change_count;
            ADEBUG << "REMAIN DIST: " << lane_remain_distance
                   << "  count: " << int(change_count);
            if (lane_remain_distance >
                max_remmain_distance * (1 + kDistExceedRatio)) {
              max_remmain_distance = lane_remain_distance;
              max_remain_dist_id = lane_id;
              max_remain_is_continue = non_noaline;
              if (remain_lane_index != 0) {
                has_max_remain = true;
                ADEBUG << "has_max_remain";
              }
              remain_lane_index++;
            } else if (lane_remain_distance * (1 + kDistExceedRatio) <
                       max_remmain_distance) {
              has_max_remain = true;
              ADEBUG << "has_max_remain";
            }
            if (change_count < min_change_num) {
              min_change_num = change_count;
              min_change_id = lane_id;
              min_count_is_continue = non_noaline;
              if (count_lane_index != 0) {
                has_min_count = true;
                ADEBUG << "has_min_count";
              }
              count_lane_index++;
            } else if (change_count > min_change_num) {
              has_min_count = true;
              ADEBUG << "has_min_count";
            }
            if ((next_navi_lane->second.lane_topo.transition() !=
                     hdmap_new::Lane_LaneTransition::
                         Lane_LaneTransition_LANE_TRANSITION_SPLIT ||
                 last_trigger_info_.is_trigger) &&
                next_lane->lane().center_line().heading_degree_size() >=
                    kMinPointSize) {
              double next_heaing_degree =
                  next_lane->lane().center_line().heading_degree().at(1);
              double heading_diff = ::math::NormalizeAngle(
                  fabs(next_heaing_degree * M_PI / 180.0 - lane_heading));
              if (fabs(heading_diff) < fabs(min_heading_diff)) {
                min_heading_diff = heading_diff;
                min_heading_diff_id = lane_id;
              }
            }
          }
        }
      }
    }
    if (has_max_remain &&
        (last_trigger_info_.is_trigger || max_remain_is_continue)) {
      ADEBUG << "final max remain";
      preferred_id = max_remain_dist_id;
    } else if (has_min_count &&
               (last_trigger_info_.is_trigger || min_count_is_continue)) {
      preferred_id = min_change_id;
      ADEBUG << "final min count";
    } else {
      preferred_id = min_heading_diff_id;
    }
  }
  return GetLaneById(new_hdmap_, preferred_id, false);
}

zmap::LaneInfoConstPtr PncMap::GetRoutePredecessor(
    zmap::LaneInfoConstPtr lane, const RouteSegments::SegmentType &type,
    const std::unordered_set<std::string> last_segment_ids) const {
  if (lane->lane().predecessor_id().empty()) {
    return nullptr;
  }

  std::unordered_set<std::string> predecessor_ids;
  for (const auto &lane_id : lane->lane().predecessor_id()) {
    predecessor_ids.insert(lane_id.id());
  }

  zark::hdmap_new::Id preferred_id = lane->lane().predecessor_id().at(0);
  for (size_t i = 1; i < route_indices_.size(); ++i) {
    auto &lane = route_indices_[i].segment.lane->lane().id();
    if (predecessor_ids.count(lane.id()) != 0) {
      if (type == RouteSegments::SegmentType::CurrentSegment) {
        if (last_segment_ids.count(lane.id()) != 0) {
          preferred_id = lane;
          break;
        }
      } else {
        preferred_id = lane;
        break;
      }
    }
  }
  return GetLaneById(new_hdmap_, preferred_id, false);
}

bool PncMap::ExtendSegments(const RouteSegments &segments,
                            const PointENU &point, double look_backward,
                            double look_forward,
                            RouteSegments *extended_segments) {
  SLPoint sl;
  LaneWaypoint waypoint;
  if (!segments.GetProjection(point, &sl, &waypoint)) {
    AERROR /*<< "point: " << point.ShortDebugString() */ << " is not on "
                                                            "segment";
    return false;
  }
  return ExtendSegments(segments, sl.s() - look_backward, sl.s() + look_forward,
                        extended_segments);
}

bool PncMap::ExtendSegments(const RouteSegments &segments, double start_s,
                            double end_s,
                            RouteSegments *const truncated_segments) const {
  if (segments.empty()) {
    AERROR << "The input segments is empty";
    return false;
  }
  CHECK_NOTNULL(truncated_segments);
  truncated_segments->SetProperties(segments);

  if (start_s >= end_s) {
    AERROR << "start_s(" << start_s << " >= end_s(" << end_s << ")";
    return false;
  }
  std::unordered_set<std::string> unique_lanes;
  static constexpr double kRouteEpsilon = 1e-3;
  // Extend the trajectory towards the start of the trajectory.
  if (start_s < 0) {
    const auto &first_segment = *segments.begin();
    auto lane = first_segment.lane;
    double s = first_segment.start_s;
    double extend_s = -start_s;
    std::vector<LaneSegment> extended_lane_segments;
    while (extend_s > kRouteEpsilon) {
      if (s <= kRouteEpsilon) {
        lane = GetRoutePredecessor(lane, segments.GetSegmentType(),
                                   last_confirm_lanes_);
        if (lane == nullptr ||
            unique_lanes.find(lane->lane().id().id()) != unique_lanes.end()) {
          break;
        }
        s = lane->total_length();
      } else {
        const double length = std::min(s, extend_s);
        extended_lane_segments.emplace_back(lane, s - length, s);
        extend_s -= length;
        s -= length;
        unique_lanes.insert(lane->lane().id().id());
      }
    }
    truncated_segments->insert(truncated_segments->begin(),
                               extended_lane_segments.rbegin(),
                               extended_lane_segments.rend());
  }
  bool found_loop = false;
  double router_s = 0;
  for (const auto &lane_segment : segments) {
    const double adjusted_start_s = std::max(
        start_s - router_s + lane_segment.start_s, lane_segment.start_s);
    const double adjusted_end_s =
        std::min(end_s - router_s + lane_segment.start_s, lane_segment.end_s);
    if (adjusted_start_s < adjusted_end_s) {
      if (!truncated_segments->empty() &&
          truncated_segments->back().lane->lane().id().id() ==
              lane_segment.lane->lane().id().id()) {
        truncated_segments->back().end_s = adjusted_end_s;
      } else if (unique_lanes.find(lane_segment.lane->lane().id().id()) ==
                 unique_lanes.end()) {
        truncated_segments->emplace_back(lane_segment.lane, adjusted_start_s,
                                         adjusted_end_s);
        unique_lanes.insert(lane_segment.lane->lane().id().id());
      } else {
        found_loop = true;
        break;
      }
    }
    router_s += (lane_segment.end_s - lane_segment.start_s);
    if (router_s > end_s) {
      break;
    }
  }
  if (found_loop) {
    return true;
  }
  // Extend the trajectory towards the end of the trajectory.
  if (router_s < end_s && !truncated_segments->empty()) {
    auto &back = truncated_segments->back();
    if (back.lane->total_length() > back.end_s) {
      double origin_end_s = back.end_s;
      back.end_s =
          std::min(back.end_s + end_s - router_s, back.lane->total_length());
      router_s += back.end_s - origin_end_s;
    }
  }
  auto last_lane = segments.back().lane;
  while (router_s < end_s - kRouteEpsilon) {
    last_lane = GetRouteSuccessor(last_lane, segments.GetSegmentType());
    if (last_lane == nullptr ||
        unique_lanes.find(last_lane->lane().id().id()) != unique_lanes.end()) {
      break;
    }
    const double length = std::min(end_s - router_s, last_lane->total_length());
    truncated_segments->emplace_back(last_lane, 0, length);
    unique_lanes.insert(last_lane->lane().id().id());
    router_s += length;
  }
  if (truncated_segments->empty()) {
    return false;
  }
  return true;
}

void PncMap::AppendLaneToPoints(LaneInfoConstPtr lane, const double start_s,
                                const double end_s,
                                std::vector<MapPathPoint> *const points) {
  if (points == nullptr || start_s >= end_s) {
    return;
  }
  double accumulate_s = 0.0;
  for (size_t i = 0; i < lane->points().size(); ++i) {
    if (accumulate_s >= start_s && accumulate_s <= end_s) {
      ::math::Vec2d map_point(lane->points()[i].x(), lane->points()[i].y());
      points->emplace_back(map_point, lane->headings()[i],
                           LaneWaypoint(lane, accumulate_s));
    }
    if (i < lane->segments().size()) {
      const auto &segment = lane->segments()[i];
      const double next_accumulate_s = accumulate_s + segment.length();
      if (start_s > accumulate_s && start_s < next_accumulate_s) {
        auto orig_map_point = segment.start() + segment.unit_direction() *
                                                    (start_s - accumulate_s);
        ::math::Vec2d map_point(orig_map_point.x(), orig_map_point.y());
        points->emplace_back(map_point, lane->headings()[i],
                             LaneWaypoint(lane, start_s));
      }
      if (end_s > accumulate_s && end_s < next_accumulate_s) {
        auto orig_map_point =
            segment.start() + segment.unit_direction() * (end_s - accumulate_s);
        ::math::Vec2d map_point(orig_map_point.x(), orig_map_point.y());
        points->emplace_back(map_point, lane->headings()[i],
                             LaneWaypoint(lane, end_s));
      }
      accumulate_s = next_accumulate_s;
    }
    if (accumulate_s > end_s) {
      break;
    }
  }
}

void PncMap::ExtractNavLanechangeInfo(const hdmap_new::Guidance &guidance_info,
                                      const VehicleState &vehicle_state,
                                      const std::string &current_section_id,
                                      const std::string &current_lane_id,
                                      NavigationInfo &navi_info) {
  const int kMinSectionNum = 2;
  uint8_t current_seq = 0;
  navi_info.valid_info = false;
  auto &sections_lane_quene = navi_info.sections;
  if (sections_lane_quene.size() > 0) {
    const int kMaxCompareCount = 10;
    bool new_path_section = false;
    std::string inner_section_id, guidance_section_id;
    int i = 0;
    int k = guidance_info.section_size() - 1;
    auto iter = sections_lane_quene.rbegin();
    while (k >= 0 && i < kMaxCompareCount &&
           iter != sections_lane_quene.rend()) {
      inner_section_id = iter->section_id;
      guidance_section_id = guidance_info.section().at(k).id().id();
      if (inner_section_id != guidance_section_id) {
        new_path_section = true;
        break;
      }
      i++;
      k--;
      iter--;
    }
    if (!new_path_section) {
      AERROR << " path section no update, only update current section info.";
      const double start_time = zark::common::Clock::NowInSeconds();
      UpdateCurrentSectionInfo(guidance_info, vehicle_state, current_section_id,
                               current_lane_id, navi_info);
      const double end_time = zark::common::Clock::NowInSeconds();
      AINFO << " UpdateCurrentSectionInfo time diff ms: "
            << (end_time - start_time) * 1000;
      return;
    }
  }
  sections_lane_quene.clear();
  navi_info.special_section.valid = false;
  AERROR << "START NAVI PROCESS.";
  if (guidance_info.section_size() == 0 || guidance_info.lane_size() == 0) {
    std::string error_msg = "guidance_info_error";
    AERROR << error_msg;
    SetErrorMsg(error_msg);
    ResetNavInfo(navi_info);
    return;
  }
  std::unordered_map<std::string, hdmap_new::LaneInfo> lanes_topo;
  std::unordered_map<std::string, uint32_t> speed_limits;
  for (const auto &speed_limit : guidance_info.speed_limit()) {
    const uint32_t speed_value = speed_limit.value_kmph();
    for (const auto &lane_id : speed_limit.lane_id()) {
      speed_limits.emplace(lane_id.id(), speed_value);
    }
  }
  for (auto &guidance_lane : guidance_info.lane()) {
    if (CheckLaneType(guidance_lane.type())) {
      std::string lane_id = guidance_lane.id().id();
      if (lanes_topo.count(lane_id) == 0) {
        lanes_topo.emplace(lane_id, guidance_lane);
      } else {
        AWARN << "exist repeat lane in guidance info: " << lane_id;
      }
    } else {
      ADEBUG << "lane is not drivable, type: " << guidance_lane.type()
             << " id: " << guidance_lane.id().id();
    }
  }
  navi_info.change_points.clear();
  for (auto &point : guidance_info.change_point()) {
    if (!point.has_distance_info() &&
        point.distance_info().distance_to_vehicle_cm() * kCentimeterToMetre +
                kPointDistBuffer <
            0)
      continue;
    navi_info.change_points.emplace_back(point);
  }
  // extract all section queues
  // set final section all connect;
  NavSection final_section;
  final_section.all_lane_connect = true;
  final_section.next_new_road = false;
  SetLaneSequence(*guidance_info.section().rbegin(), lanes_topo,
                  current_lane_id, current_seq, final_section);
  navi_info.sections.emplace_back(final_section);
  NavSection forward_section = final_section;
  std::unordered_set<std::string> saved_section_id;
  if (guidance_info.section_size() >= kMinSectionNum) {
    for (int i = guidance_info.section_size() - 2; i >= 0; i--) {
      NavSection current_section;
      const auto &map_section = guidance_info.section().at(i);
      if (saved_section_id.count(map_section.id().id())) {
        AERROR << "map_guidance_section repeat: " << map_section.id().id();
        continue;
      } else {
        saved_section_id.emplace(map_section.id().id());
      }
      if (map_section.id().id() != current_section_id &&
          map_section.distance_info().distance_to_vehicle_cm() *
                      kCentimeterToMetre +
                  kSectionDisBuffer <
              0) {
        break;
      }
      SetLaneSequence(map_section, lanes_topo, current_lane_id, current_seq,
                      current_section);
      CalculateTopoRange(forward_section, lanes_topo, current_section_id,
                         current_section);
      if (!CalculateChangeCount(forward_section, speed_limits,
                                current_section_id, current_section)) {
        navi_info.error_sections.emplace_back(current_section);
        continue;
      }
      IdentitySpecialSection(forward_section, current_section, map_section,
                             navi_info);
      ExtractNonTargetSplit(forward_section, current_section_id,
                            navi_info.sections, current_section);
      navi_info.sections.emplace_front(current_section);
      forward_section = current_section;
    }
    bool find_special = false;
    if (navi_info.special_section.valid &&
        (navi_info.special_section.attribute &
         hdmap_new::RoadSection_RoadAttribute::
             RoadSection_RoadAttribute_ATTR_TOLLSTATION)) {
      bool has_save_virtual = false;
      if (virtual_section_ids_.count(
              navi_info.special_section.target_section_id)) {
        has_save_virtual = true;
      }
      for (auto &check_section : navi_info.sections) {
        if (!find_special) {
          if (check_section.section_id != navi_info.special_section.id) {
            continue;
          } else {
            find_special = true;
            continue;
          }
        }
        if (!has_save_virtual) {
          virtual_section_ids_.emplace(check_section.section_id);
        }
        if (check_section.section_id ==
            navi_info.special_section.target_section_id) {
          break;
        }
      }
    } else {
      virtual_section_ids_.clear();
    }
  }
  navi_info.special_section_ids = virtual_section_ids_;
  navi_info.currrent_lane_pose = current_seq;
  navi_info.current_lane_id = current_lane_id;
  navi_info.current_section_id = current_section_id;
  navi_info.valid_info = true;
}

bool PncMap::IdentitySpecialSection(const NavSection &forward_section,
                                    NavSection &current_section,
                                    const hdmap_new::SectionInfo &map_section,
                                    NavigationInfo &navi_info) {
  bool res = false;
  const float distance_ego =
      map_section.distance_info().distance_to_vehicle_cm() * kCentimeterToMetre;
  if (distance_ego > 0 && (current_section.attribute &
                           hdmap_new::RoadSection_RoadAttribute::
                               RoadSection_RoadAttribute_ATTR_TOLLSTATION)) {
    // TODO: add other special section;
    current_section.all_transition_continue = true;
    current_section.all_lane_connect = true;
    for (auto &lane : current_section.lanes) {
      lane.second.reset();
    }
    if (navi_info.special_section.distance_section > distance_ego) {
      navi_info.special_section.distance_section = distance_ego;
      navi_info.special_section.attribute = current_section.attribute;
      navi_info.special_section.valid = true;
      navi_info.special_section.target_section_id = current_section.section_id;
      virtual_section_ids_.clear();
      navi_info.special_section.id.clear();
      AINFO << " find special section, type: " << current_section.attribute
            << "  distance: " << distance_ego
            << "  id: " << current_section.section_id;
    }
    res = true;
  } else if (navi_info.special_section.valid &&
             (navi_info.special_section.attribute &
              hdmap_new::RoadSection_RoadAttribute::
                  RoadSection_RoadAttribute_ATTR_TOLLSTATION)) {
    const float kMaxIdentiyDis = DecisionGflags::alc_toll_station_range;
    if (navi_info.special_section.distance_section - current_section.end_s <
        kMaxIdentiyDis) {
      size_t lane_size = current_section.lanes.size();
      const uint8_t kMinSuccessorNum = 3;
      if (lane_size < kMinSuccessorNum && lane_size > 0) {
        bool all_successor_next = true;
        const uint8_t check_size =
            lane_size == 1 ? kMinSuccessorNum : kMinSuccessorNum - 1;
        for (const auto &lane : current_section.lanes) {
          if (!all_successor_next) break;
          if (lane.second.lane_topo.successor_id_size() < check_size) {
            all_successor_next = false;
            break;
          }
          for (const auto &succesor_id : lane.second.lane_topo.successor_id()) {
            if (!forward_section.lanes.count(succesor_id.id())) {
              all_successor_next = false;
              break;
            }
          }
        }
        res = all_successor_next;
      } else if (lane_size >= kMinSuccessorNum) {
        bool all_successor_next = true;
        for (const auto &lane : current_section.lanes) {
          if (lane.second.lane_sequence == 1 ||
              lane.second.lane_sequence == lane_size) {
            continue;
          }
          if (!all_successor_next) break;
          if (lane.second.lane_topo.successor_id_size() < 2) {
            all_successor_next = false;
            break;
          }
          for (const auto &succesor_id : lane.second.lane_topo.successor_id()) {
            if (!forward_section.lanes.count(succesor_id.id())) {
              all_successor_next = false;
              break;
            }
          }
        }
        res = all_successor_next;
      }
      if (res) {
        current_section.all_transition_continue = true;
        current_section.all_lane_connect = true;
        for (auto &lane : current_section.lanes) {
          lane.second.reset();
        }
        navi_info.special_section.id = current_section.section_id;
        AINFO << " start enter special station range, section: "
              << current_section.section_id;
      }
    }
  }
  return res;
}

void PncMap::SetLaneSequence(
    const hdmap_new::SectionInfo &section,
    const std::unordered_map<std::string, hdmap_new::LaneInfo> &lanes_topo,
    const std::string &current_lane_id, uint8_t &current_lane_seq,
    NavSection &section_lane_queue) {
  const int8_t kMaxSectionLanes = static_cast<int>(kMaxLaneSize);
  section_lane_queue.section_id = section.id().id();
  section_lane_queue.start_s =
      section.distance_info().distance_to_vehicle_cm() * kCentimeterToMetre;
  section_lane_queue.end_s = (section.distance_info().end_offset_cm() -
                              section.distance_info().start_offset_cm()) *
                                 kCentimeterToMetre +
                             section_lane_queue.start_s;
  section_lane_queue.attribute = section.attribute();
  for (auto &lane_id : section.lane_id()) {
    auto lane_topo = lanes_topo.find(lane_id.id());
    if (lane_topo == lanes_topo.end() || !lanes_topo.count(lane_id.id())) {
      continue;
    }
    std::string right_neigbor_id =
        lane_topo->second.right_neighbor_forward_lane_id().id();
    // find right most lane util right neighbor id is empty
    if (right_neigbor_id.empty() || !lanes_topo.count(right_neigbor_id)) {
      LaneTopoInfo save_topo_info;
      // the sequence of right most lane is 1
      save_topo_info.lane_sequence = 1;
      save_topo_info.lane_topo.CopyFrom(lane_topo->second);
      section_lane_queue.lanes.insert(
          std::make_pair<>(lane_id.id(), save_topo_info));
      section_lane_queue.valid_lane_size++;
      if (lane_id.id() == current_lane_id) {
        current_lane_seq = 1;
      }
      int8_t lanes_num = 1;
      // search every left neighbor lane form right msot lane;
      std::string left_neighbor_id =
          lane_topo->second.left_neighbor_forward_lane_id().id();
      // util lanes size excced kMaxSectionLanes or left neighbor lane id is
      // empty, stop search;
      while (lanes_num < kMaxSectionLanes) {
        if (left_neighbor_id.empty()) break;
        auto left_lane = lanes_topo.find(left_neighbor_id);
        if (left_lane == lanes_topo.end() ||
            !lanes_topo.count(left_lane->first)) {
          break;
        }
        lanes_num++;
        LaneTopoInfo new_lane_change;
        new_lane_change.lane_sequence = lanes_num;
        if (left_neighbor_id == current_lane_id) {
          current_lane_seq = lanes_num;
        }
        new_lane_change.lane_topo.CopyFrom(left_lane->second);
        section_lane_queue.lanes.insert(
            std::make_pair<>(left_neighbor_id, new_lane_change));
        section_lane_queue.valid_lane_size++;
        left_neighbor_id =
            left_lane->second.left_neighbor_forward_lane_id().id();
      }
      break;
    }
  }
}

void PncMap::CalculateTopoRange(
    const NavSection &forward_section,
    const std::unordered_map<std::string, hdmap_new::LaneInfo> &lanes_topo,
    const std::string &ego_section_id, NavSection &current_section) {
  SectionTopoRange &section_topo_range = current_section.topo_range;
  float dist_merge_end;
  bool has_merge = false;
  eLCRD_ForceLaneChgReqDir merge_dir = eLCRD_ForceLaneChgReqDir::LCRD_None_LC;
  uint8_t merge_seq = kMaxLaneSize;
  for (auto &lane : current_section.lanes) {
    bool lane_confirm = false;
    uint8_t successsor_index = 0;
    for (const auto &forward_id : lane.second.lane_topo.successor_id()) {
      if (lane_confirm) break;
      auto forward_lane = forward_section.lanes.find(forward_id.id());
      if (forward_lane != forward_section.lanes.end()) {
        if (lane.second.lane_topo.transition() !=
            hdmap_new::Lane::LaneTransition::
                Lane_LaneTransition_LANE_TRANSITION_MERGE) {
          if (forward_lane->second.lane_topo.transition() !=
              hdmap_new::Lane::LaneTransition::
                  Lane_LaneTransition_LANE_TRANSITION_SPLIT) {
            section_topo_range.priority_sucessor_range.insert(
                {lane.second.lane_sequence, forward_lane->first});
            if (forward_lane->second.lane_sequence <
                section_topo_range.first_successor_seq) {
              section_topo_range.first_successor_seq =
                  forward_lane->second.lane_sequence;
            }
            if (forward_lane->second.lane_sequence >
                section_topo_range.last_successor_seq) {
              section_topo_range.last_successor_seq =
                  forward_lane->second.lane_sequence;
            }
            lane_confirm = true;
            break;
          }
        } else {
          if (forward_lane->second.lane_sequence <
              section_topo_range.first_successor_seq) {
            section_topo_range.first_successor_seq =
                forward_lane->second.lane_sequence;
          }
          if (forward_lane->second.lane_sequence >
              section_topo_range.last_successor_seq) {
            section_topo_range.last_successor_seq =
                forward_lane->second.lane_sequence;
          }
          lane_confirm = true;
          lane.second.is_recommend = false;
          lane.second.final_id = lane.first;
          AWARN << "lane is merge, not recommend: " << lane.first;
          current_section.all_transition_continue = false;
          break;
        }
      } else {
        if (lane.second.lane_topo.successor_id().size() == 1 ||
            successsor_index ==
                lane.second.lane_topo.successor_id().size() - 1) {
          lane_confirm = true;
          lane.second.is_recommend = false;
          lane.second.final_id = lane.first;
          AWARN << "lane is non connected, not recommend: " << lane.first;
        }
      }
      successsor_index++;
    }
    if (!lane_confirm && !current_section.next_new_road) {
      current_section.next_new_road = true;
      current_section.all_lane_connect = false;
    }
    // merge extract
    if (lane.second.lane_topo.transition() ==
            hdmap_new::Lane::LaneTransition::
                Lane_LaneTransition_LANE_TRANSITION_MERGE &&
        current_section.lanes.size() > 2 &&
        (lane.second.lane_sequence == 1 ||
         lane.second.lane_sequence == current_section.lanes.size())
        /*&& lane type check*/) {
      dist_merge_end =
          (lane.second.lane_topo.distance_info().end_offset_cm() -
           lane.second.lane_topo.distance_info().start_offset_cm()) *
          kCentimeterToMetre;
      if (ego_section_id == current_section.section_id) {
        dist_merge_end +=
            lane.second.lane_topo.distance_info().distance_to_vehicle_cm() *
            kCentimeterToMetre;
      }
      has_merge = true;
      merge_seq = lane.second.lane_sequence;
      if (lane.second.lane_sequence == 1) {
        merge_dir = eLCRD_ForceLaneChgReqDir::LCRD_Left_LC;
      } else {
        merge_dir = eLCRD_ForceLaneChgReqDir::LCRD_Right_LC;
      }
    }
  }
  if (current_section.all_lane_connect && !current_section.next_new_road &&
      current_section.all_transition_continue &&
      (current_section.topo_range.first_successor_seq > 1 ||
       current_section.topo_range.last_successor_seq <
           forward_section.valid_lane_size) &&
      current_section.valid_lane_size < forward_section.valid_lane_size) {
    for (const auto &new_lane : forward_section.lanes) {
      if (current_section.next_new_road) break;
      for (const auto &id : new_lane.second.lane_topo.predecessor_id()) {
        if (!current_section.lanes.count(id.id())) {
          AERROR << "next section is similary main road,current section is: "
                 << current_section.section_id;
          current_section.next_new_road = true;
          break;
        }
      }
    }
  }
  for (auto &connect_lane : current_section.lanes) {
    const uint8_t lane_seq = connect_lane.second.lane_sequence;
    const auto top_iter =
        current_section.topo_range.priority_sucessor_range.find(lane_seq);
    if (top_iter == current_section.topo_range.priority_sucessor_range.end()) {
      continue;
    }
    if (has_merge && (lane_seq == merge_seq + 1 || lane_seq == merge_seq - 1)) {
      connect_lane.second.be_merged = true;
      connect_lane.second.dist_merge_end = dist_merge_end;
      connect_lane.second.merge_dir = merge_dir;
      connect_lane.second.merged_id = connect_lane.first;
      break;
    } else {
      const auto next_lane = forward_section.lanes.find(top_iter->second);
      if (next_lane != forward_section.lanes.end()) {
        if (next_lane->second.be_merged) {
          connect_lane.second.be_merged = true;
          float distance_forward =
              (connect_lane.second.lane_topo.distance_info().end_offset_cm() -
               connect_lane.second.lane_topo.distance_info()
                   .start_offset_cm()) *
              kCentimeterToMetre;
          if (ego_section_id == current_section.section_id) {
            distance_forward += connect_lane.second.lane_topo.distance_info()
                                    .distance_to_vehicle_cm() *
                                kCentimeterToMetre;
          }
          connect_lane.second.dist_merge_end =
              next_lane->second.dist_merge_end + distance_forward;
          connect_lane.second.merge_dir = next_lane->second.merge_dir;
          connect_lane.second.merged_id = next_lane->second.merged_id;
          break;
        }
      }
    }
  }
}

void PncMap::ExtractNonTargetSplit(const NavSection &forward_section,
                                   const std::string &ego_section_id,
                                   std::list<NavSection> &sections,
                                   NavSection &current_section) {
  const float kMaxSplitRemainDis = 250.0;
  for (const auto &lane : current_section.lanes) {
    if (lane.second.lane_topo.transition() !=
        hdmap_new::Lane_LaneTransition::
            Lane_LaneTransition_LANE_TRANSITION_SPLIT) {
      continue;
    }
    if (lane.second.change_count == 0) {
      continue;
    }
    auto left_lane = current_section.lanes.find(
        lane.second.lane_topo.left_neighbor_forward_lane_id().id());
    auto right_lane = current_section.lanes.find(
        lane.second.lane_topo.right_neighbor_forward_lane_id().id());
    if (left_lane != current_section.lanes.end() &&
        left_lane->second.remain_distance >
            lane.second.remain_distance + kMaxSplitRemainDis &&
        lane.second.change_direction ==
            eLCRD_ForceLaneChgReqDir::LCRD_Left_LC) {
      left_lane->second.be_split = true;
      left_lane->second.split_dir = eLCRD_ForceLaneChgReqDir::LCRD_Right_LC;
      left_lane->second.dist_split_end = lane.second.remain_distance;
      left_lane->second.split_id = lane.first;
      FillFinalSplitInfo(current_section, lane.second,
                         left_lane->second.split_dir, sections);
    } else if (right_lane != current_section.lanes.end() &&
               right_lane->second.remain_distance >
                   lane.second.remain_distance + kMaxSplitRemainDis &&
               lane.second.change_direction ==
                   eLCRD_ForceLaneChgReqDir::LCRD_Right_LC) {
      right_lane->second.be_split = true;
      right_lane->second.split_dir = eLCRD_ForceLaneChgReqDir::LCRD_Left_LC;
      right_lane->second.dist_split_end = lane.second.remain_distance;
      right_lane->second.split_id = lane.first;
      FillFinalSplitInfo(current_section, lane.second,
                         right_lane->second.split_dir, sections);
    }
  }
  // check next lane be split
  for (auto &connect_lane : current_section.lanes) {
    if (connect_lane.second.be_split) {
      continue;
    }
    const auto top_iter =
        current_section.topo_range.priority_sucessor_range.find(
            connect_lane.second.lane_sequence);
    if (top_iter == current_section.topo_range.priority_sucessor_range.end()) {
      continue;
    }
    auto next_lane = forward_section.lanes.find(top_iter->second);
    if (next_lane == forward_section.lanes.end() ||
        !next_lane->second.be_split) {
      continue;
    }
    connect_lane.second.be_split = true;
    connect_lane.second.split_dir = next_lane->second.split_dir;
    connect_lane.second.split_id = next_lane->second.split_id;
    float distance_forward =
        (connect_lane.second.lane_topo.distance_info().end_offset_cm() -
         connect_lane.second.lane_topo.distance_info().start_offset_cm()) *
        kCentimeterToMetre;
    if (ego_section_id == current_section.section_id) {
      distance_forward += connect_lane.second.lane_topo.distance_info()
                              .distance_to_vehicle_cm() *
                          kCentimeterToMetre;
    }
    connect_lane.second.dist_split_end =
        next_lane->second.dist_split_end + distance_forward;
  }
}

void PncMap::FillFinalSplitInfo(const NavSection &current_section,
                                const LaneTopoInfo &current_lane,
                                const eLCRD_ForceLaneChgReqDir &split_dir,
                                std::list<NavSection> &sections) {
  if (!current_lane.is_recommend) {
    return;
  }
  const auto top_iter = current_section.topo_range.priority_sucessor_range.find(
      current_lane.lane_sequence);
  if (top_iter == current_section.topo_range.priority_sucessor_range.end()) {
    return;
  }
  std::string next_id = top_iter->second;
  for (auto &section : sections) {
    const auto next_lane = section.lanes.find(next_id);
    if (next_lane != section.lanes.end()) {
      std::string neighbor_id;
      if (split_dir == eLCRD_ForceLaneChgReqDir::LCRD_Right_LC) {
        neighbor_id =
            next_lane->second.lane_topo.left_neighbor_forward_lane_id().id();
      } else if (split_dir == eLCRD_ForceLaneChgReqDir::LCRD_Left_LC) {
        neighbor_id =
            next_lane->second.lane_topo.right_neighbor_forward_lane_id().id();
      }
      auto neighbor_lane = section.lanes.find(neighbor_id);
      if (neighbor_lane == section.lanes.end()) {
        break;
      }
      neighbor_lane->second.be_split = true;
      neighbor_lane->second.dist_split_end = next_lane->second.remain_distance;
      neighbor_lane->second.split_dir = split_dir;
      neighbor_lane->second.split_id = current_lane.lane_topo.id().id();
      if (next_id == current_lane.final_id) {
        break;
      } else {
        const auto forward_top_iter =
            section.topo_range.priority_sucessor_range.find(
                next_lane->second.lane_sequence);
        if (forward_top_iter ==
            section.topo_range.priority_sucessor_range.end()) {
          break;
        } else {
          next_id = forward_top_iter->second;
        }
      }
    }
  }
}

bool PncMap::CalculateChangeCount(
    const NavSection &forward_section,
    const std::unordered_map<std::string, uint32_t> speed_limits,
    const std::string &current_section_id, NavSection &current_section) {
  if (current_section.topo_range.priority_sucessor_range.size() == 0) {
    AERROR << "current section is not connected next: "
           << current_section.section_id;
    return false;
  }
  // 1. process straight conencted lane
  for (auto &lane : current_section.lanes) {
    uint8_t lane_seq = lane.second.lane_sequence;
    // whteher straight connected
    auto iter =
        current_section.topo_range.priority_sucessor_range.find(lane_seq);
    if (iter != current_section.topo_range.priority_sucessor_range.end()) {
      const auto &next_lane = forward_section.lanes.find(iter->second);
      if (next_lane == forward_section.lanes.end()) {
        AERROR << "find forward lane in next section failed, forward id: "
               << iter->second;
        break;
      } else {
        // update remain_distance
        if (next_lane->second.remain_distance + kLimitDisBuff <
            DecisionGflags::alc_forcelc_distlim_valid) {
          double new_distance =
              (lane.second.lane_topo.distance_info().end_offset_cm() -
               lane.second.lane_topo.distance_info().start_offset_cm()) *
              kCentimeterToMetre;
          if (current_section_id == lane.second.lane_topo.section_id().id()) {
            new_distance +=
                lane.second.lane_topo.distance_info().distance_to_vehicle_cm() *
                kCentimeterToMetre;
          }
          lane.second.remain_distance =
              next_lane->second.remain_distance + new_distance;
          lane.second.final_id = next_lane->second.final_id;
        }
        // update change count, consider next section and current lane size;
        const uint8_t connect_count = next_lane->second.change_count;
        const eLCRD_ForceLaneChgReqDir connect_dir =
            next_lane->second.change_direction;
        if (connect_count > 0) {
          lane.second.change_direction = connect_dir;
          if (connect_dir == eLCRD_ForceLaneChgReqDir::LCRD_Left_LC) {
            uint8_t current_seq_diff =
                current_section.topo_range.priority_sucessor_range.rbegin()
                    ->first -
                lane.second.lane_sequence;
            current_seq_diff = std::min(kMaxLaneSize, current_seq_diff);
            lane.second.change_count =
                std::min(current_seq_diff, connect_count);
          } else if (connect_dir == eLCRD_ForceLaneChgReqDir::LCRD_Right_LC) {
            uint8_t current_seq_diff =
                lane.second.lane_sequence -
                current_section.topo_range.priority_sucessor_range.begin()
                    ->first;
            current_seq_diff = std::min(kMaxLaneSize, current_seq_diff);
            lane.second.change_count =
                std::min(current_seq_diff, connect_count);
          }
          if (lane.second.change_count == 0) {
            lane.second.change_direction =
                eLCRD_ForceLaneChgReqDir::LCRD_None_LC;
          }
        }
        lane.second.forward_is_recommend =
            next_lane->second.is_recommend ? true : false;
      }
      if (!lane.second.forward_is_recommend) {
        CalNextNonRecomDis(forward_section, iter->second, lane.second);
      }
    }
  }
  // 2. process non-connected or non-straight conencted lane;
  for (auto &lane : current_section.lanes) {
    const uint8_t lane_seq = lane.second.lane_sequence;
    if (current_section.topo_range.priority_sucessor_range.count(lane_seq)) {
      continue;
    }
    lane.second.is_recommend = false;
    double new_distance =
        (lane.second.lane_topo.distance_info().end_offset_cm() -
         lane.second.lane_topo.distance_info().start_offset_cm()) *
        kCentimeterToMetre;
    if (current_section_id == lane.second.lane_topo.section_id().id()) {
      new_distance +=
          lane.second.lane_topo.distance_info().distance_to_vehicle_cm() *
          kCentimeterToMetre;
    }
    lane.second.final_id = lane.first;
    lane.second.remain_distance = new_distance;
    if (lane_seq > current_section.topo_range.last_successor_seq ||
        lane_seq < current_section.topo_range.first_successor_seq) {
      current_section.all_lane_connect = false;
      AERROR << "road change after currrent section: "
             << current_section.section_id << "lane: " << lane.first;
    }
    const auto start_iter =
        current_section.topo_range.priority_sucessor_range.begin();
    const auto final_iter =
        current_section.topo_range.priority_sucessor_range.rbegin();
    if (lane_seq < start_iter->first) {
      // current lane in road right
      lane.second.change_direction = eLCRD_ForceLaneChgReqDir::LCRD_Left_LC;
      for (const auto &priority_lane : current_section.lanes) {
        if (priority_lane.second.lane_sequence == start_iter->first) {
          if (priority_lane.second.change_count > 0 &&
              priority_lane.second.change_direction ==
                  eLCRD_ForceLaneChgReqDir::LCRD_Left_LC) {
            lane.second.change_count = start_iter->first - lane_seq +
                                       priority_lane.second.change_count;
          } else {
            lane.second.change_count = start_iter->first - lane_seq;
          }
          break;
        }
      }
    } else if (lane_seq > final_iter->first) {
      // current lane in road left
      lane.second.change_direction = eLCRD_ForceLaneChgReqDir::LCRD_Right_LC;
      for (const auto &priority_lane : current_section.lanes) {
        if (priority_lane.second.lane_sequence == final_iter->first) {
          if (priority_lane.second.change_count > 0 &&
              priority_lane.second.change_direction ==
                  eLCRD_ForceLaneChgReqDir::LCRD_Right_LC) {
            lane.second.change_count = lane_seq - final_iter->first +
                                       priority_lane.second.change_count;
          } else {
            lane.second.change_count = lane_seq - final_iter->first;
          }
          break;
        }
      }
    }
  }
  // 3. fusion staright connected lane count;
  for (auto &lane : current_section.lanes) {
    if (!lane.second.is_recommend) {
      continue;
    }
    // fill lane speed limit;
    const auto &speed_limit = speed_limits.find(lane.first);
    if (speed_limit != speed_limits.end()) {
      const float value = speed_limit->second;
      lane.second.speed_limit = value;
      if (current_section.max_speed_limit < value) {
        current_section.max_speed_limit = value;
      }
      if (current_section.min_spee_limit > value) {
        current_section.min_spee_limit = value;
      }
    }
    if (lane.second.change_direction ==
        eLCRD_ForceLaneChgReqDir::LCRD_Left_LC) {
      // find left lane, compare remian distance:
      const auto &left_lane = current_section.lanes.find(
          lane.second.lane_topo.left_neighbor_forward_lane_id().id());
      if (left_lane == current_section.lanes.end() ||
          left_lane->second.remain_distance + kLimitDisBuff <
              lane.second.remain_distance) {
        lane.second.change_count = 0;
        lane.second.change_direction = eLCRD_ForceLaneChgReqDir::LCRD_None_LC;
      }
    } else if (lane.second.change_direction ==
               eLCRD_ForceLaneChgReqDir::LCRD_Right_LC) {
      // find right lane, compare remian distance:
      const auto &right_lane = current_section.lanes.find(
          lane.second.lane_topo.right_neighbor_forward_lane_id().id());
      if (right_lane == current_section.lanes.end() ||
          right_lane->second.remain_distance + kLimitDisBuff <
              lane.second.remain_distance) {
        lane.second.change_count = 0;
        lane.second.change_direction = eLCRD_ForceLaneChgReqDir::LCRD_None_LC;
      }
    }
  }

  // 4. remain_distance add line type check;
  if (current_section.start_s < 0.0 &&
      current_section.section_id != current_section_id) {
    return true;
  }
  CalSolidBoundaryRemainDist(forward_section, current_section);
  return true;
}

void PncMap::CalSolidBoundaryRemainDist(const NavSection &forward_section,
                                        NavSection &check_section) {
  const float kDistBuffer = 2.0;  // [m]
  for (auto &lane : check_section.lanes) {
    eLCRD_ForceLaneChgReqDir nav_change_dir = lane.second.change_direction;
    if (lane.second.remain_check_boundary) {
      continue;
    }
    if (!lane.second.is_recommend &&
        (lane.second.lane_sequence <
             check_section.topo_range.first_successor_seq ||
         lane.second.lane_sequence >
             check_section.topo_range.last_successor_seq)) {
      CheckBoundaryRemainDist(lane.second);
      CheckNeighborLaneRemain(check_section, lane.second);

    } else if (nav_change_dir != eLCRD_ForceLaneChgReqDir::LCRD_None_LC) {
      const auto &forward_topo =
          check_section.topo_range.priority_sucessor_range.find(
              lane.second.lane_sequence);
      if (forward_topo ==
          check_section.topo_range.priority_sucessor_range.end()) {
        continue;
      }
      const auto &forward_lane =
          forward_section.lanes.find(forward_topo->second);
      if (forward_lane == forward_section.lanes.end()) {
        continue;
      }
      if (forward_lane->second.remain_distance <=
          kMinRemainDist + kDistBuffer) {
        CheckBoundaryRemainDist(lane.second);
        CheckNeighborLaneRemain(check_section, lane.second);
      }
    }
  }
}

void PncMap::CheckBoundaryRemainDist(LaneTopoInfo &check_lane,
                                     const bool &is_first_check) {
  if (is_first_check && check_lane.remain_check_boundary) return;
  check_lane.remain_check_boundary = true;
  if (check_lane.change_direction == eLCRD_ForceLaneChgReqDir::LCRD_None_LC ||
      check_lane.change_count == 0) {
    return;
  }
  const auto &boundary_type =
      check_lane.change_direction == eLCRD_ForceLaneChgReqDir::LCRD_Right_LC
          ? check_lane.lane_topo.right_boundary_attr()
          : check_lane.lane_topo.left_boundary_attr();
  bool last_revise = false;
  for (int8_t i = boundary_type.size() - 1; i >= 0; i--) {
    const hdmap_new::LaneBoundaryAttribute::Type &type =
        boundary_type[i].type();
    if (type ==
            hdmap_new::LaneBoundaryAttribute::Type::
                LaneBoundaryAttribute_Type_LANEBOUNDARY_TYPE_SINGLE_SOLID_LINE ||
        type == hdmap_new::LaneBoundaryAttribute::Type::
                    LaneBoundaryAttribute_Type_LANEBOUNDARY_TYPE_DOUBLE_SOLID ||
        (check_lane.change_direction ==
             eLCRD_ForceLaneChgReqDir::LCRD_Right_LC &&
         type ==
             hdmap_new::LaneBoundaryAttribute::Type::
                 LaneBoundaryAttribute_Type_LANEBOUNDARY_TYPE_LEFT_SOLID_AND_RIGHT_DASH) ||
        (check_lane.change_direction ==
             eLCRD_ForceLaneChgReqDir::LCRD_Left_LC &&
         type ==
             hdmap_new::LaneBoundaryAttribute::Type::
                 LaneBoundaryAttribute_Type_LANEBOUNDARY_TYPE_RIGHT_SOLID_AND_LEFT_DASH)) {
      if (i == boundary_type.size() - 1 || last_revise) {
        double solid_distance = 0.0;
        if (boundary_type[i].distance_info().distance_to_vehicle_cm() > 0.0) {
          solid_distance =
              std::fmax((boundary_type[i].distance_info().end_offset_cm() -
                         boundary_type[i].distance_info().start_offset_cm()) *
                            kCentimeterToMetre,
                        0.0);
        } else {
          solid_distance = std::fmax(
              (boundary_type[i].distance_info().end_offset_cm() -
               boundary_type[i].distance_info().start_offset_cm() +
               boundary_type[i].distance_info().distance_to_vehicle_cm()) *
                  kCentimeterToMetre,
              0.0);
        }
        check_lane.remain_distance = std::fmax(
            check_lane.remain_distance - solid_distance, kMinRemainDist);
        last_revise = true;
      }
    } else {
      last_revise = false;
    }
  }
}

void PncMap::CheckNeighborLaneRemain(NavSection &check_section,
                                     LaneTopoInfo &check_lane) {
  std::string start_id;
  uint8_t start_seq = 0;
  bool find_target = false;
  const eLCRD_ForceLaneChgReqDir target_dir = check_lane.change_direction;
  float min_remain_dist = DecisionGflags::alc_forcelc_distlim_valid;
  if (check_lane.change_count == 1) {
    start_seq = target_dir == eLCRD_ForceLaneChgReqDir::LCRD_Right_LC
                    ? check_lane.lane_sequence + 1
                    : check_lane.lane_sequence - 1;
    start_id = target_dir == eLCRD_ForceLaneChgReqDir::LCRD_Right_LC
                   ? check_lane.lane_topo.left_neighbor_forward_lane_id().id()
                   : check_lane.lane_topo.right_neighbor_forward_lane_id().id();
    find_target = true;
  } else {
    const uint8_t sequence_diff = check_lane.change_count - 1;
    const uint8_t compare_seq =
        target_dir == eLCRD_ForceLaneChgReqDir::LCRD_Right_LC
            ? check_lane.lane_sequence - sequence_diff
            : check_lane.lane_sequence + sequence_diff;
    if (compare_seq > check_section.lanes.size() || compare_seq == 0) {
      AERROR << "compare seq error: " << compare_seq;
      return;
    }
    for (auto &compare_lane : check_section.lanes) {
      if (compare_lane.second.lane_sequence == compare_seq) {
        if (!compare_lane.second.remain_check_boundary) {
          compare_lane.second.remain_check_boundary = true;
          CheckBoundaryRemainDist(compare_lane.second);
        }
        if (min_remain_dist > compare_lane.second.remain_distance) {
          min_remain_dist = compare_lane.second.remain_distance;
        }
        start_seq = target_dir == eLCRD_ForceLaneChgReqDir::LCRD_Right_LC
                        ? compare_lane.second.lane_sequence + 1
                        : compare_lane.second.lane_sequence - 1;
        start_id =
            target_dir == eLCRD_ForceLaneChgReqDir::LCRD_Right_LC
                ? compare_lane.second.lane_topo.left_neighbor_forward_lane_id()
                      .id()
                : compare_lane.second.lane_topo.right_neighbor_forward_lane_id()
                      .id();
        find_target = true;
        break;
      }
    }
  }
  if (!find_target) return;
  // reverse compare lane remain distance util check lane
  uint8_t cycle_cnt = 50;  // limit cycle times
  while (start_seq > 0 && start_seq <= check_section.lanes.size() &&
         cycle_cnt > 0) {
    auto neighbor_lane = check_section.lanes.find(start_id);
    if (neighbor_lane == check_section.lanes.end()) {
      break;
    }
    if (neighbor_lane->second.change_count == 0) {
      break;
    }
    if (neighbor_lane->second.change_direction != target_dir) {
      break;
    }
    CheckBoundaryRemainDist(neighbor_lane->second);
    if (neighbor_lane->second.remain_distance > min_remain_dist) {
      neighbor_lane->second.remain_distance = min_remain_dist;
    } else {
      min_remain_dist = neighbor_lane->second.remain_distance;
    }
    start_id =
        target_dir == eLCRD_ForceLaneChgReqDir::LCRD_Right_LC
            ? neighbor_lane->second.lane_topo.left_neighbor_forward_lane_id()
                  .id()
            : neighbor_lane->second.lane_topo.right_neighbor_forward_lane_id()
                  .id();
    start_seq = target_dir == eLCRD_ForceLaneChgReqDir::LCRD_Right_LC
                    ? start_seq + 1
                    : start_seq - 1;
    cycle_cnt--;
  }
}

void PncMap::CalNextNonRecomDis(const NavSection &forward_section,
                                const std::string &next_lane_id,
                                LaneTopoInfo &current_lane) {
  const auto &next_lane = forward_section.lanes.find(next_lane_id);
  if (next_lane == forward_section.lanes.end()) {
    AERROR << "CalNextNonRecomDis find forward lane failed " << next_lane_id;
  }
  current_lane.non_recommend_distance =
      (current_lane.lane_topo.distance_info().end_offset_cm() -
       current_lane.lane_topo.distance_info().start_offset_cm() +
       current_lane.lane_topo.distance_info().distance_to_vehicle_cm()) *
      kCentimeterToMetre;
}

void PncMap::UpdateCurrentSectionInfo(const hdmap_new::Guidance &map_guidance,
                                      const VehicleState &vehicle_state,
                                      const std::string &current_section_id,
                                      const std::string &current_lane_id,
                                      NavigationInfo &navi_info) {
  navi_info.current_lane_id = current_lane_id;
  navi_info.current_section_id = current_section_id;
  hdmap_new::SectionInfo current_section_info;
  while (navi_info.error_sections.size() > 20) {
    navi_info.error_sections.pop_front();
  }
  bool find_current_section = false;
  bool find_current_lane = false;
  for (const auto &section_info : map_guidance.section()) {
    if (section_info.id().id() == current_section_id) {
      current_section_info.CopyFrom(section_info);
      find_current_section = true;
      break;
    }
  }
  if (!find_current_section) {
    AERROR << "find current section failed in guidance.";
    error_msg_ = "failed find cur section";
    ResetNavInfo(navi_info);
    return;
  }
  navi_info.valid_info = true;
  navi_info.change_points.clear();
  ::math::Vec2d ego_point(vehicle_state.x(), vehicle_state.y());
  for (const auto &point : map_guidance.change_point()) {
    if (point.distance_info().distance_to_vehicle_cm() * kCentimeterToMetre +
            kPointDistBuffer <
        0) {
      continue;
    }
    navi_info.change_points.emplace_back(point);
  }
  int16_t current_section_index = 0;
  uint8_t update_map_lane_count = 0;
  for (auto section = navi_info.sections.begin();
       section != navi_info.sections.end(); section++) {
    if (section->section_id == current_section_id) {
      for (const auto &map_lane_info : map_guidance.lane()) {
        if (update_map_lane_count >= section->lanes.size()) {
          break;
        }
        auto update_lane = section->lanes.find(map_lane_info.id().id());
        if (update_lane == section->lanes.end()) {
          continue;
        }
        update_lane->second.lane_topo.CopyFrom(map_lane_info);
        update_lane->second.remain_check_boundary = false;
        if (map_lane_info.id().id() == current_lane_id) {
          find_current_lane = true;
        }
        update_map_lane_count++;
      }
      if (!find_current_lane) {
        error_msg_ = "find current lane failed in guidance.";
        AERROR << error_msg_;
        ResetNavInfo(navi_info);
        return;
      }
      auto forward_section = std::next(section);
      bool has_forward =
          forward_section != navi_info.sections.end() ? true : false;
      for (auto &lane : section->lanes) {
        if (lane.first == current_lane_id) {
          navi_info.currrent_lane_pose = lane.second.lane_sequence;
        }
        double distance_forward = 0.0;
        distance_forward =
            (current_section_info.distance_info().end_offset_cm() -
             current_section_info.distance_info().start_offset_cm() +
             current_section_info.distance_info().distance_to_vehicle_cm()) *
            kCentimeterToMetre;
        section->start_s =
            current_section_info.distance_info().distance_to_vehicle_cm() *
            kCentimeterToMetre;
        section->end_s = distance_forward;
        if (!lane.second.is_recommend) {
          lane.second.remain_distance = distance_forward;
        } else if (lane.second.remain_distance + kLimitDisBuff <
                   DecisionGflags::alc_forcelc_distlim_valid) {
          if (has_forward) {
            lane.second.remain_distance = UpdateRemainDist(
                *forward_section, *section, current_section_info,
                lane.second.lane_sequence, distance_forward);
          } else {
            AWARN << "apdate current section " << section->section_id
                  << "  no forward section";
            lane.second.remain_distance = distance_forward;
          }
        }

        if (!lane.second.forward_is_recommend) {
          lane.second.non_recommend_distance = distance_forward;
        } else if (lane.second.non_recommend_distance + kLimitDisBuff <
                   DecisionGflags::alc_forcelc_distlim_valid) {
          if (has_forward) {
            lane.second.non_recommend_distance = UpdateNonReconmmenDist(
                *forward_section, *section, current_section_info,
                lane.second.lane_sequence, distance_forward);
          } else {
            lane.second.non_recommend_distance = distance_forward;
          }
        }
        // update merge distance
        if (lane.second.be_merged) {
          if (!has_forward || lane.second.merged_id == lane.first) {
            lane.second.dist_merge_end = distance_forward;
          } else {
            for (const auto &next_id : lane.second.lane_topo.successor_id()) {
              const auto next_lane_info =
                  forward_section->lanes.find(next_id.id());
              if (next_lane_info != forward_section->lanes.end() &&
                  next_lane_info->second.be_merged) {
                lane.second.dist_merge_end =
                    next_lane_info->second.dist_merge_end + distance_forward;
                break;
              }
            }
          }
        }
        // update non-target split distance
        if (lane.second.be_split) {
          if (!has_forward) {
            lane.second.dist_split_end = distance_forward;
          } else {
            bool find_next_split = false;
            for (const auto &next_id : lane.second.lane_topo.successor_id()) {
              const auto next_lane_info =
                  forward_section->lanes.find(next_id.id());
              if (next_lane_info != forward_section->lanes.end() &&
                  next_lane_info->second.be_split) {
                lane.second.dist_split_end =
                    next_lane_info->second.dist_split_end + distance_forward;
                find_next_split = true;
                break;
              }
            }
            if (!find_next_split) {
              lane.second.dist_split_end = distance_forward;
            }
          }
        }
      }
      if (has_forward) {
        CalSolidBoundaryRemainDist(*forward_section, *section);
      }
      break;
    }
    current_section_index++;
  }
  const int16_t kMaxBackSectionSize = 10;
  if (current_section_index > kMaxBackSectionSize) {
    const int kMaxCycle = 100;
    int16_t cnt =
        std::min(current_section_index - kMaxBackSectionSize, kMaxCycle);
    while (navi_info.sections.size() > 0 && cnt > 0) {
      if (navi_info.sections.begin()->section_id != current_section_id) {
        navi_info.sections.pop_front();
      } else {
        break;
      }
      cnt--;
    }
  }
}

double PncMap::UpdateRemainDist(const NavSection &forward_section,
                                const NavSection &current_section,
                                const hdmap_new::SectionInfo &cur_map_section,
                                const uint8_t current_lane_seq,
                                const double distance_forward) {
  double distance = distance_forward;
  const auto &lane =
      current_section.topo_range.priority_sucessor_range.find(current_lane_seq);
  if (lane != current_section.topo_range.priority_sucessor_range.end()) {
    const auto &forward_lane = forward_section.lanes.find(lane->second);
    if (forward_lane != forward_section.lanes.end() &&
        forward_lane->second.remain_distance + kLimitDisBuff <
            DecisionGflags::alc_forcelc_distlim_valid) {
      distance += forward_lane->second.remain_distance;
    }
  }
  return distance;
}

double PncMap::UpdateNonReconmmenDist(
    const NavSection &forward_section, const NavSection &current_section,
    const hdmap_new::SectionInfo &cur_map_section,
    const uint8_t current_lane_seq, const double distance_forward) {
  double distance = distance_forward;
  const auto &lane =
      current_section.topo_range.priority_sucessor_range.find(current_lane_seq);
  if (lane != current_section.topo_range.priority_sucessor_range.end()) {
    const auto &forward_lane = forward_section.lanes.find(lane->second);
    if (forward_lane != forward_section.lanes.end() &&
        forward_lane->second.non_recommend_distance + kLimitDisBuff <
            DecisionGflags::alc_forcelc_distlim_valid) {
      distance += forward_lane->second.non_recommend_distance;
    }
  }
  return distance;
}

void PncMap::ResetNavInfo(NavigationInfo &navi_info) {
  std::lock_guard<std::mutex> lock(navinfo_lock_);
  navi_info.valid_info = false;
  navi_info.change_points.clear();
  navi_info.sections.clear();
  navi_info.error_sections.clear();
  navi_info.info_timestamp = 0.0;
  last_confirm_lanes_.clear();
  last_noa_line_.ClearFixInfo();
}

void PncMap::ClearNavInfoGrouop(const std::string &error_msg) {
  error_msg_ = error_msg;
  ClearNavInfoGrouop();
}

void PncMap::ClearNavInfoGrouop() {
  ResetNavInfo(navigation_info_);
  std::lock_guard<std::mutex> lock(navinfo_lock_);
  navigation_info_group_.clear();
  last_trigger_info_.ResetTriggerInfo();
  virtual_section_ids_.clear();
}

}  // namespace hdmap
}  // namespace planning
}  // namespace zark
