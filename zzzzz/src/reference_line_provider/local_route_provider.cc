//
// Copyright 2024 zpilot. All Rights Reserved.
//

/**
 * @file
 * @brief Implementation of the class LocalRouteProvider.
 */

#include <algorithm>
#include <chrono>
#include <limits>
#include <thread>
#include <utility>

#include "apps/planning/src/common/clock.h"
#include "apps/planning/src/common/data_reader/data_reader.h"
#include "apps/planning/src/common/configs/vehicle_config_helper.h"
#include "math_utils.h"
#include "util.h"
#include "apps/planning/src/config/conf_gflags.h"
#include "apps/planning/src/config/config_main.h"
#include "apps/planning/src/reference_line_provider/local_route_provider.h"
#include "apps/planning/src/reference_line_provider/pnc_map/hdmap_api.h"
#include "apps/planning/src/reference_line_provider/pnc_map/path.h"

/**
 * @namespace zark::planning
 * @brief zark::planning
 */
namespace zark {
namespace planning {

using common::VehicleState;
using ::math::AngleDiff;
using ::math::Vec2d;
using planning::hdmap::LaneWaypoint;
using planning::hdmap::MapPathPoint;
using planning::hdmap::PncMap;
using planning::hdmap::RouteSegments;
using zark::ads_decision::DEC_Outputs;
using zark::planning::common::VehicleConfigHelper;
using namespace zark::ads_common;

LocalRouteProvider::LocalRouteProvider()
    : pnc_map_(std::make_unique<hdmap::PncMap>()),
      polynomial_fitter_(std::make_unique<MapPolynomialFit>()) {
  std::string config_file(PlanningGflags::local_route_path);
  zark::planning::Config config_instance{config_file};
  config_instance.SetLocalRouteConfig(local_route_config_);
  smoother_.reset(new DiscretePointsLocalRouteSmoother(
      local_route_config_.smoother_config));
  acc_trajectory_builder_.reset(new ACCTrajectoryBuilder(local_route_config_));
}

bool LocalRouteProvider::Start() {
  std::thread local_route_thread(&LocalRouteProvider::GenerateThread, this);
  pthread_setname_np(local_route_thread.native_handle(), "local_route");
  local_route_thread.detach();
  AINFO << "LocalRouteProvider thread.detach.";
  return true;
}

void LocalRouteProvider::Stop() {
  std::lock_guard<std::mutex> lock(stop_flag_mutex_);
  is_stop_ = true;
  AINFO << "stop local route thread.";
}

void LocalRouteProvider::UpdateLocalRoute(
    const std::list<LocalRoute> &local_routes,
    const std::list<hdmap::RouteSegments> &route_segments) {
  if (local_routes.size() != route_segments.size()) {
    AERROR << "The calculated local route size(" << local_routes.size()
           << ") and route_segments size(" << route_segments.size()
           << ") are different";
    return;
  }
  std::lock_guard<std::mutex> lock(local_routes_mutex_);
  // update history
  local_route_history_.push(local_routes);
  route_segments_history_.push(route_segments);
  static constexpr int kMaxHistoryNum = 3;
  if (local_route_history_.size() > kMaxHistoryNum) {
    local_route_history_.pop();
    route_segments_history_.pop();
  }

  is_local_route_updated_ = true;
}

void LocalRouteProvider::ThreadSleep(const double start_time,
                                     const double end_time) {
  if (start_time < 0 || end_time < 0 || end_time < start_time) {
    return;
  }
  double process_diff = (end_time - start_time) * 1000.0;
  int sleep_time = static_cast<int>(std::fmax(
      0.0, local_route_config_.local_route_thread_time - process_diff));
  ADEBUG << "local route thread sleep time ms: " << sleep_time
         << "  end - start: " << process_diff;
  std::this_thread::sleep_for(std::chrono::milliseconds(sleep_time));
}

void LocalRouteProvider::GenerateThread() {
  while (true) {
    if (is_stop_) {
      break;
    }
    const double start_time = zark::common::Clock::NowInSeconds();
    VehicleState vehicle_state;
    auto localization_info = DataReader::GetInstance()->GetLocalizationData();
    if (localization_info.loc_state() !=
            localization::LocalizationState::LocalizationState_Normal ||
        !localization_info.has_local_pose() ||
        !localization_info.has_header()) {
      AERROR << "Localization state is not normal, stop generate refline.."
             << localization_info.loc_state();
      ClearFittedLine();
      pnc_map_->ClearNavInfoGrouop();
      error_msg_ = "Localization_state_error";
      ThreadSleep(start_time, zark::common::Clock::NowInSeconds());
      continue;
    } else {
      vehicle_state.set_x(localization_info.local_pose().pose().t().x());
      vehicle_state.set_y(localization_info.local_pose().pose().t().y());
      vehicle_state.set_heading(localization_info.local_pose().yaw());
      vehicle_state.set_linear_velocity(
          localization_info.local_pose().velocity().x());
    }
    AINFO << "local route thread state x: " << vehicle_state.x()
          << "  y: " << vehicle_state.y()
          << " heading: " << vehicle_state.heading()
          << "  vel: " << vehicle_state.linear_velocity();

    if (!local_route_config_.offline_pack) {
      refline_start_time_ = localization_info.header().timestamp_nano();
    } else {
      refline_start_time_ =
          std::chrono::duration_cast<std::chrono::nanoseconds>(
              std::chrono::system_clock::now().time_since_epoch())
              .count();
    }

    const auto map_state = DataReader::GetInstance()->GetMapState();
    if (!map_state.has_state() ||
        map_state.state().state() !=
            mapfusion::MapLocalizationState::MapLocalizationState_Normal ||
        map_state.state().err_code() ==
            mapfusion::MapLocalizationErrorCode::
                MapLocalizationErrorCode_Inconsistent) {
      AERROR
          << "mapfusion localization state error, fail to generate ref_line.";
      error_msg_ = "MapLocalizationState error";
      ClearFittedLine();
      pnc_map_->ClearNavInfoGrouop();
      ThreadSleep(start_time, zark::common::Clock::NowInSeconds());
      continue;
    }

    MapInfo map_info = DataReader::GetInstance()->GetMapData();
    {
      std::lock_guard<std::mutex> lock(is_olm_mutex_);
      is_olm_ = PlanningGflags::enable_online_map &&
                (map_info.map_type == zark::hdmap_new::MapHeader::MAP_TYPE_CP);
      // TODO(@yfh): Need consider local map type for lane level planning from
      // map guidance info
    }
    if (!pnc_map_->UpdateLocalHdMap(map_info, is_olm_)) {
      AERROR << "Failed to update local hdmap";
      ClearFittedLine();
      pnc_map_->ClearNavInfoGrouop();
      error_msg_ = "update_map_" + pnc_map_->GetErrorMsg();
      ThreadSleep(start_time, zark::common::Clock::NowInSeconds());
      continue;
    }
    std::list<LocalRoute> local_routes;
    std::list<hdmap::RouteSegments> segments;
    if (!CreateLocalRoute(vehicle_state, local_routes, segments)) {
      AERROR << "Fail to get local route";
      ClearFittedLine();
      pnc_map_->ClearNavInfoGrouop();
      ThreadSleep(start_time, zark::common::Clock::NowInSeconds());
      continue;
    }
    if (!is_olm_) {
      DEC_Outputs fitted_result;
      const double fit_start_time = zark::common::Clock::NowInSeconds();
      if (!FitReferenceLine(local_routes, segments, fitted_result)) {
        ClearFittedLine();
        pnc_map_->ClearNavInfoGrouop();
        ThreadSleep(start_time, zark::common::Clock::NowInSeconds());
        continue;
      }
      UpdateHostLaneChangeInfo(fitted_result);
      SetHostLCResult(fitted_result, local_routes);
      const double fit_end_time = zark::common::Clock::NowInSeconds();
      AINFO << " all line fitted time diff ms: "
            << (fit_end_time - fit_start_time) * 1000;
      {
        int64_t nano_stamp =
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::system_clock::now().time_since_epoch())
                .count();
        std::lock_guard<std::mutex> lock(fitted_line_mutex_);
        fitted_line_result_.emplace_back(std::make_pair<>(
            (nano_stamp - refline_start_time_) / 1e6, fitted_result));
        DataReader::GetInstance()->SetFittedResult(fitted_line_result_.back());
        if (fitted_line_result_.size() > 3) {
          fitted_line_result_.pop_front();
        }
      }
    }
    UpdateLocalRoute(local_routes, segments);
    AINFO << "updated local routes: " << local_routes.size() << "  "
          << segments.size();
    // update lane level planning result;
    NavigationInfo lane_plan_result;
    if (!is_olm_) {
      lane_plan_result = *pnc_map_->GetNavInfo();
    }
    DataReader::GetInstance()->SetNaviResult(lane_plan_result);
    reference_line_proto::ReferenceLineState refline_state;
    refline_state.set_state(reference_line_proto::ReferenceLineState::State::
                                ReferenceLineState_State_NORMAL_STATE);
    refline_state.set_error_msg("NORMAL");
    DataReader::GetInstance()->SetRefLineState(refline_state);
    const double end_time = zark::common::Clock::NowInSeconds();
    ThreadSleep(start_time, end_time);
    last_calculation_time_ = end_time - start_time;
    AINFO << "local route create time ms: " << last_calculation_time_ * 1000;
  }
}

void LocalRouteProvider::UpdateFitState() {
  VehicleState vehicle_state;
  auto localization_info = DataReader::GetInstance()->GetLocalizationData();
  if (localization_info.loc_state() !=
          localization::LocalizationState::LocalizationState_Normal ||
      !localization_info.has_local_pose() || !localization_info.has_header()) {
    return;
  } else {
    const double kNano2Sec = 1.0e-9;
    const double kMinTimeOffset = 0.05;  //[s]
    const double kMaxTimeOffset = 0.30;  //[s]
    vehicle_state.set_x(localization_info.local_pose().pose().t().x());
    vehicle_state.set_y(localization_info.local_pose().pose().t().y());
    double offset_t = 0.0;
    if (local_route_config_.offline_pack) {
      offset_t = 0.1;
    } else {
      offset_t = std::fmax(
          kMinTimeOffset,
          zark::common::Clock::NowInSeconds() -
              double(localization_info.header().timestamp_nano()) * kNano2Sec);
      offset_t = std::fmin(kMaxTimeOffset, offset_t);
    }
    double heading = localization_info.local_pose().yaw() +
                     offset_t * localization_info.local_pose().yawrate();
    vehicle_state.set_heading(heading);
    vehicle_state.set_linear_velocity(
        localization_info.local_pose().velocity().x());
  }
  polynomial_fitter_->SetVehicleState(vehicle_state);
}

void LocalRouteProvider::SetHostLCResult(const DEC_Outputs &fitted_result,
                                         std::list<LocalRoute> &local_routes) {
  for (auto &local_route : local_routes) {
    if (local_route.Lanes().GetSegmentType() ==
        hdmap::RouteSegments::SegmentType::CurrentSegment) {
      if (fitted_result.dec_out_lanechangeinfo_bus()
              .dec_out_is_hostlnechgtolft_bl() > 0) {
        local_route.SetHostLC2Left(true);
      } else if (fitted_result.dec_out_lanechangeinfo_bus()
                     .dec_out_is_hostlnechgtorgt_bl() > 0) {
        local_route.SetHostLC2Right(true);
      }
      break;
    }
  }
}

bool LocalRouteProvider::FitReferenceLine(
    const std::list<LocalRoute> &reference_lines,
    const std::list<hdmap::RouteSegments> &segments,
    DEC_Outputs &fitted_result) {
  UpdateFitState();
  const VehicleState ego_state = polynomial_fitter_->GetVehicleState();
  const auto &current_id = pnc_map_->GetNavInfo()->current_lane_id;
  hdmap_new::Id map_lane_id;
  map_lane_id.set_id(current_id);
  const auto map_lane_info =
      hdmap::GetLaneById(hdmap::HDMapUtil::BaseMapPtr(), map_lane_id);
  if (!map_lane_info) {
    AWARN << "can not find current lane info in map service. " << current_id;
    error_msg_ = "failed_find_cur_line_in_map_service";
    return false;
  }
  double ego_map_s = 0.0;
  double ego_map_l = 0.0;
  if (!map_lane_info->GetProjection(::math::Vec2d(ego_state.x(), ego_state.y()),
                                    &ego_map_s, &ego_map_l)) {
    AWARN << "can not projection ego state in current lane.";
    error_msg_ = "failed_projec_ego_state_to_cur_line";
    return false;
  }
  auto segment_iter = segments.begin();
  const bool trigger_noa_line = pnc_map_->GetNavInfo()->noa_line.is_trigger;
  AINFO << "noa line trigger: " << trigger_noa_line;
  if (!trigger_noa_line) {
    fix_noa_line_.first.ClearFixInfo();
    fix_noa_line_.second.clear();
  }
  B1_LaneLineInfo noaline;
  std::vector<LocalRoutePoint> fit_points;
  for (auto iter = reference_lines.begin();
       iter != reference_lines.end() && segment_iter != segments.end();
       iter++, segment_iter++) {
    MapPolynomialFit::FittedLine fit_line;
    SpecialLaneInfo split_merge_info;
    const auto &segment_type = segment_iter->GetSegmentType();
    AERROR << "SEGMENT TYPE: " << int(segment_iter->GetSegmentType())
           << " points size: " << iter->RoutePoints().size()
           << "  length: " << iter->Length();
    if (FindSplit(*iter, *segment_iter, split_merge_info) ||
        (segment_type == hdmap::RouteSegments::SegmentType::CurrentSegment &&
         trigger_noa_line)) {
      ReSampleRefPoints(*iter, fit_points);
      if (segment_type == hdmap::RouteSegments::SegmentType::CurrentSegment &&
          trigger_noa_line) {
        polynomial_fitter_->DiscretePointsFit(fit_points, kLineFitteOrder,
                                              fit_line);
        FillOutputLine(fit_line, noaline);
      } else {
        polynomial_fitter_->DiscretePointsFit(fit_points, kEdgeFitteOrder,
                                              fit_line);
      }
    }
    switch (segment_type) {
      case RouteSegments::SegmentType::CurrentSegment:
        AERROR << "start fitting current side line.";
        FillOutputLine(
            fit_line, *fitted_result.mutable_dec_out_hostlnerearcentline_bus());
        ProcessBoundaryLine(
            *iter, *segment_iter, fit_line, ego_map_s, split_merge_info,
            *fitted_result.mutable_dec_out_hostlnelftline_bus(),
            *fitted_result.mutable_dec_out_hostlnergtline_bus());
        AERROR << "start fitting road egde.";
        ProcessRoadEdge(*iter, *segment_iter,
                        *fitted_result.mutable_dec_out_lftroadedge_bus(),
                        *fitted_result.mutable_dec_out_rgtroadedge_bus());
        CurveNodeProcess(*pnc_map_->GetNavInfo(), *segment_iter, ego_map_s,
                         fitted_result);
        break;
      case RouteSegments::SegmentType::LeftSegment:
        AERROR << "start fitting left lane side line.";
        FillOutputLine(fit_line,
                       *fitted_result.mutable_dec_out_lftlnerearcentline_bus());
        ProcessBoundaryLine(*iter, *segment_iter, fit_line, ego_map_s,
                            split_merge_info,
                            *fitted_result.mutable_dec_out_lftlnelftline_bus(),
                            *fitted_result.mutable_dec_out_lftlnergtline_bus());
        break;
      case RouteSegments::SegmentType::RightSegment:
        AERROR << "start fitting right lane side line.";
        FillOutputLine(fit_line,
                       *fitted_result.mutable_dec_out_rgtlnerearcentline_bus());
        ProcessBoundaryLine(*iter, *segment_iter, fit_line, ego_map_s,
                            split_merge_info,
                            *fitted_result.mutable_dec_out_rgtlnelftline_bus(),
                            *fitted_result.mutable_dec_out_rgtlnergtline_bus());
        break;
      case RouteSegments::SegmentType::InvalidSegment:
      default:
        AERROR << "segment_type invalid!!!";
        break;
    }
  }
  pnc_map_->SetFixNoaLineInfo(fix_noa_line_.first);
  if (pnc_map_->GetNavInfo()->noa_line.is_trigger) {
    fitted_result.mutable_dec_out_noarampline_bus()->CopyFrom(noaline);
  }
  return true;
}

void LocalRouteProvider::ProcessRoadEdge(const LocalRoute &refline,
                                         const hdmap::RouteSegments &segment,
                                         B_RoadEdge_st &left_edge,
                                         B_RoadEdge_st &rigth_edge) {
  std::vector<LocalRoutePoint> left_points, rigth_points;
  const uint8_t kNumPtsMin = 2;
  if (left_points.size() > kNumPtsMin) {
    LocalRoute original_left_line(left_points, local_route_config_, true);
    LocalRoute smoothed_left_line;
    if (SmoothLocalRoute(original_left_line, smoothed_left_line, true)) {
      MapPolynomialFit::FittedLine fitted_line;
      AERROR << "start fitting left road egde.";
      polynomial_fitter_->DiscretePointsFit(smoothed_left_line.RoutePoints(),
                                            kEdgeFitteOrder, fitted_line);
      FillOutputLine(fitted_line, left_edge);
    }
  }
  if (rigth_points.size() > kNumPtsMin) {
    LocalRoute original_right_line(rigth_points, local_route_config_, true);
    LocalRoute smoothed_right_line;
    if (SmoothLocalRoute(original_right_line, smoothed_right_line, true)) {
      MapPolynomialFit::FittedLine fitted_line;
      AERROR << "start fitting right road egde.";
      polynomial_fitter_->DiscretePointsFit(smoothed_right_line.RoutePoints(),
                                            kEdgeFitteOrder, fitted_line);
      FillOutputLine(fitted_line, rigth_edge);
    }
  }
}

void LocalRouteProvider::ExtractEdgePoint(
    const LocalRoute &refline, const hdmap::RouteSegments &segment,
    std::vector<LocalRoutePoint> &left_points,
    std::vector<LocalRoutePoint> &right_points) {
  const uint8_t kBoudaryDist = 3;
  ::common::SLPoint sl;
  const auto &ego_state = polynomial_fitter_->GetVehicleState();
  ::math::Vec2d ego_point(ego_state.x(), ego_state.y());
  if (!refline.XYToSL(ego_point, sl)) {
    AERROR << "ego projection to refline failed: ";
    return;
  }
  bool extract_left = false;
  bool extract_right = false;
  const auto navi_info = pnc_map_->GetNavInfo();
  for (const auto &section : navi_info->sections) {
    if (section.section_id == navi_info->current_section_id) {
      const auto current_lane = section.lanes.find(navi_info->current_lane_id);
      if (current_lane != section.lanes.end()) {
        if (current_lane->second.lane_sequence > 1) {
          if (current_lane->second.lane_sequence == section.valid_lane_size) {
            extract_left = true;
          }
        } else {
          extract_right = true;
          hdmap_new::Id id;
          id.set_id(navi_info->current_lane_id);
          auto map_lane =
              hdmap::GetLaneById(hdmap::HDMapUtil::BaseMapPtr(), id, false);
          int8_t left_neighbors = 0;
          int8_t right_neighbors = 0;
          if (map_lane) {
            left_neighbors =
                map_lane->lane().left_neighbor_forward_lane_id_size();
            right_neighbors =
                map_lane->lane().right_neighbor_forward_lane_id_size();
          }
          if (right_neighbors > 0) {
            extract_right = false;
          }
          if (section.valid_lane_size == 1 && left_neighbors > 0) {
            extract_left = true;
          }
        }
      }
      if (!extract_left && !extract_right) {
        return;
      }
      break;
    }
  }
  const auto &s_range = refline.MapPathInfo().accumulated_s();
  double left_width = 0;
  double right_width = 0;
  int16_t point_index = 0;
  int16_t last_index = -100;
  int s_size = s_range.size();
  for (const auto &point : refline.RoutePoints()) {
    if (point_index >= s_size) break;
    if (point_index - last_index < kBoudaryDist) {
      point_index++;
      continue;
    }
    if (!refline.GetRoadWidth(s_range[point_index], left_width, right_width)) {
      break;
    }
    if (extract_left) {
      left_points.emplace_back(std::move(LocalRoutePoint(
          std::move(hdmap::MapPathPoint(
              std::move(GetWidthPoint(point, left_width)), 0.0)),
          0.0, 0.0)));
    }
    if (extract_right) {
      right_points.emplace_back(std::move(LocalRoutePoint(
          std::move(hdmap::MapPathPoint(
              std::move(GetWidthPoint(point, -right_width)), 0.0)),
          0.0, 0.0)));
    }
    point_index++;
  }
}

void LocalRouteProvider::FillOutputLine(
    const MapPolynomialFit::FittedLine &fit_line,
    zark::ads_common::B1_LaneLineInfo &out_line) {
  if (fit_line.valid_length_ < kMinValidLength) {
    AERROR << "fit line length error: " << fit_line.valid_length_;
    return;
  }
  if (fit_line.polynomial_coefficients_.size() > kEdgeFitteOrder) {
    out_line.set_lli_d_linelatposa0_sg(fit_line.polynomial_coefficients_[0]);
    out_line.set_lli_angr_linehdanga1_sg(fit_line.polynomial_coefficients_[1]);
    out_line.set_lli_cv_linecrva2_sg(fit_line.polynomial_coefficients_[2]);
    out_line.set_lli_cvr_linecrvrta3_sg(fit_line.polynomial_coefficients_[3]);
    out_line.set_lli_d_linerngstart_sg(0.0);
    out_line.set_lli_d_linerngend_sg(fit_line.valid_length_);
    out_line.set_lli_stat_lineqltycnfd_elcf(
        eLCF_LaneCnfdn::LCF_RelbleforInterv);
  }
  if (fit_line.polynomial_coefficients_.size() > kLineFitteOrder) {
    out_line.set_lli_dcvr_linedcrvrta4_sg(fit_line.polynomial_coefficients_[4]);
    out_line.set_lli_ddcvr_lineddcrvrta5_sg(
        fit_line.polynomial_coefficients_[5]);
  }
}

void LocalRouteProvider::FillOutputLine(
    const MapPolynomialFit::FittedLine &fit_line,
    zark::ads_common::B_RoadEdge_st &out_line) {
  if (fit_line.valid_length_ < kMinValidLength) {
    AERROR << "fit line length error: " << fit_line.valid_length_;
    return;
  }
  if (fit_line.polynomial_coefficients_.size() > kEdgeFitteOrder) {
    out_line.set_da_in_d_roadedgea0_sg(fit_line.polynomial_coefficients_[0]);
    out_line.set_da_in_angr_roadedgea1_sg(fit_line.polynomial_coefficients_[1]);
    out_line.set_da_in_cv_roadedgea2_sg(fit_line.polynomial_coefficients_[2]);
    out_line.set_da_in_cvr_roadedgea3_sg(fit_line.polynomial_coefficients_[3]);
    out_line.set_da_in_d_roadedgerangend_sg(fit_line.valid_length_);
    out_line.set_da_in_stat_roadedgecnfdn_elcf(
        eLCF_LaneCnfdn::LCF_RelbleforInterv);
  }
}

void LocalRouteProvider::GetMapBoundaryPoint(
    const map_service::zmap::LaneBoundaryInfoConstPtr &boundary_info,
    const double start_s, const double end_s,
    const std::pair<bool, float> &move_info,
    std::vector<LocalRoutePoint> &extract_point, const bool &is_first_seg) {
  const float kMinMoveDist = 0.20;
  const float kMaxMoveDist = 0.35;
  bool real_move = false;
  float move_dist = 0.0;
  if (move_info.first && move_info.second >= kMinMoveDist) {
    real_move = true;
    move_dist = std::fmin(move_info.second, kMaxMoveDist);
  }
  auto offset_iter = boundary_info->lane_boundary().curve().offset_cm().begin();
  for (auto boundary_point =
           boundary_info->lane_boundary().curve().point().begin();
       boundary_point != boundary_info->lane_boundary().curve().point().end() &&
       offset_iter != boundary_info->lane_boundary().curve().offset_cm().end();
       boundary_point++, offset_iter++) {
    if ((*offset_iter -
         *boundary_info->lane_boundary().curve().offset_cm().begin()) *
                kCentimeterToMetre +
            5.0 <
        start_s) {
      bool jump = true;
      auto next_check_iter = std::next(offset_iter);
      if (next_check_iter !=
          boundary_info->lane_boundary().curve().offset_cm().end()) {
        if (is_first_seg &&
            (*next_check_iter -
             *boundary_info->lane_boundary().curve().offset_cm().begin()) *
                    kCentimeterToMetre >
                start_s + 0.5) {
          jump = false;
        }
      }
      if (jump) continue;
    }
    ::math::Vec2d original_point(boundary_point->x(), boundary_point->y());
    if (extract_point.size() > 0) {
      if (original_point.DistanceTo(extract_point.back()) < 0.1) {
        continue;
      }
    }
    if (!real_move) {
      extract_point.emplace_back(std::move(LocalRoutePoint(
          std::move(hdmap::MapPathPoint(original_point, 0.0)), 0.0, 0.0)));
    } else {
      float heading = 0.0;
      if (boundary_point ==
          boundary_info->lane_boundary().curve().point().begin()) {
        auto next_point = std::next(boundary_point);
        heading = std::atan2(next_point->y() - boundary_point->y(),
                             next_point->x() - boundary_point->x());
      } else {
        auto prev_point = std::prev(boundary_point);
        heading = std::atan2(boundary_point->y() - prev_point->y(),
                             boundary_point->x() - prev_point->x());
      }
      extract_point.emplace_back(std::move(LocalRoutePoint(
          std::move(hdmap::MapPathPoint(
              std::move(GetWidthPoint(original_point, move_dist, heading)),
              0.0)),
          0.0, 0.0)));
    }
    if ((*offset_iter -
         *boundary_info->lane_boundary().curve().offset_cm().begin()) *
            kCentimeterToMetre >
        end_s + 5.0) {
      break;
    }
  }
  // check boundary point num
  const uint8_t kMinBoundPointNum = 2;
  const float kMinLerpDist = 1.0;
  if (extract_point.size() == kMinBoundPointNum) {
    AINFO << "extract only 2 boundary points";
    float distance = extract_point.begin()->DistanceTo(*extract_point.rbegin());
    if (distance < kMinLerpDist) return;
    ::math::Vec2d dir_point =
        (*extract_point.rbegin() - *extract_point.begin());
    dir_point.Normalize();
    auto lerp_point = *extract_point.rbegin() + dir_point * 0.5;
    extract_point.insert(
        extract_point.begin() + 1,
        std::move(LocalRoutePoint(
            std::move(hdmap::MapPathPoint(lerp_point, 0.0)), 0.0, 0.0)));
  }
}

void LocalRouteProvider::ExtractBoudaryPoint(
    const LocalRoute &refline, const hdmap::RouteSegments &segment,
    const double ego_map_s, const SpecialLaneInfo &split_merge_info,
    std::pair<hdmap_new::LaneBoundaryAttribute,
              hdmap_new::LaneBoundaryAttribute> &line_type,
    std::pair<std::vector<LocalRoutePoint>, std::vector<LocalRoutePoint>>
        &boundary_points,
    int &move_base) {
  const double kBackBoudaryDisBuffer = 3.0;     //[m]
  const double kForwardBoudaryDisBuffer = 6.0;  //[m]

  ::common::SLPoint ego_sl;
  if (!refline.XYToSL(polynomial_fitter_->GetEgoPoint(), ego_sl)) {
    return;
  }

  const double backward_s = std::fmax(0.0, ego_sl.s() - kBoundaryBackDist);
  const auto &start_point = refline.GetLocalRoutePoint(backward_s);
  const auto &end_point = refline.GetLocalRoutePoint(
      ego_sl.s() + polynomial_fitter_->GetVehicleState().linear_velocity() *
                       local_route_config_.look_forward_time_sec);
  if (end_point.lane_waypoints().size() == 0 ||
      start_point.lane_waypoints().size() == 0) {
    return;
  }
  std::string end_point_lane =
      end_point.lane_waypoints().at(0).lane->lane().id().id();
  hdmap_new::Id left_boundary_id, right_boundary_id;
  size_t seg_size = segment.size();
  bool need_extract_left =
      (!split_merge_info.left_is_merge && !split_merge_info.split_in_right)
          ? true
          : false;
  bool need_extract_right =
      (!split_merge_info.right_is_merge && !split_merge_info.split_in_left)
          ? true
          : false;
  int move_dir = 0;
  if (segment.GetSegmentType() ==
          hdmap::RouteSegments::SegmentType::CurrentSegment &&
      need_extract_left && need_extract_right &&
      !pnc_map_->GetNavInfo()->noa_line.is_trigger) {
    move_dir = CheckLaneWidth(refline, last_move_info_);
    if (move_dir == 1) {
      need_extract_right = false;
    } else if (move_dir == -1) {
      need_extract_left = false;
    }
  }
  move_base = move_dir;
  AINFO << "need_extract_left : " << need_extract_left
        << "  need_extract_right: " << need_extract_right
        << " move base: " << move_base;
  const std::string final_id =
      pnc_map_->GetNavInfo()->current_lane_info.final_id;
  const std::string current_id =
      start_point.lane_waypoints().begin()->lane->lane().id().id();
  bool confirm_current = false;
  size_t stop_left_index = 10000;
  size_t stop_right_index = 10000;
  for (size_t i = 0; i < seg_size; i++) {
    // check merge lane:
    std::string seg_lane_id = segment.at(i).lane->lane().id().id();
    if (!confirm_current && seg_lane_id == current_id) {
      confirm_current = true;
    }
    if (!confirm_current) continue;
    if (final_id == seg_lane_id && pnc_map_->GetNavInfo()) {
      const auto &navi_sections = pnc_map_->GetNavInfo()->sections;
      for (const auto &section : navi_sections) {
        auto final_lane = section.lanes.find(final_id);
        if (final_lane != section.lanes.end()) {
          if (final_lane->second.lane_topo.transition() ==
              hdmap_new::Lane_LaneTransition::
                  Lane_LaneTransition_LANE_TRANSITION_MERGE) {
            if (final_lane->second.change_direction ==
                eLCRD_ForceLaneChgReqDir::LCRD_Left_LC) {
              stop_left_index = i;
            } else if (final_lane->second.change_direction ==
                       eLCRD_ForceLaneChgReqDir::LCRD_Right_LC) {
              stop_right_index = i;
            }
          }
          break;
        }
      }
    }
    left_boundary_id = segment.at(i).lane->lane().left_boundary_id();
    right_boundary_id = segment.at(i).lane->lane().right_boundary_id();
    const auto &left_boundary_info = hdmap::GetLaneBoundaryById(
        hdmap::HDMapUtil::BaseMapPtr(), left_boundary_id);
    const auto &right_boundary_info = hdmap::GetLaneBoundaryById(
        hdmap::HDMapUtil::BaseMapPtr(), right_boundary_id);
    double left_start_s = 0.0;
    double right_start_s = 0.0;
    double l = 0.0;
    double left_end_s = 0.0;
    double right_end_s = 0.0;
    bool left_info_valid = false;
    bool right_info_valid = false;
    if (seg_size == 1) {
      if (need_extract_left && left_boundary_info &&
          left_boundary_info->GetProjection(start_point, &left_start_s, &l) &&
          left_boundary_info->GetProjection(end_point, &left_end_s, &l)) {
        left_info_valid = true;
      }
      if (need_extract_right && right_boundary_info &&
          right_boundary_info->GetProjection(start_point, &right_start_s, &l) &&
          right_boundary_info->GetProjection(end_point, &right_end_s, &l)) {
        right_info_valid = true;
      }
    } else {
      if (i == 0) {
        if (need_extract_left && left_boundary_info &&
            left_boundary_info->GetProjection(start_point, &left_start_s, &l)) {
          left_info_valid = true;
          left_end_s = left_boundary_info->total_length();
        }
        if (need_extract_right && right_boundary_info &&
            right_boundary_info->GetProjection(start_point, &right_start_s,
                                               &l)) {
          right_info_valid = true;
          right_end_s = right_boundary_info->total_length();
        }
      } else if (i < seg_size - 1) {
        if (need_extract_left && left_boundary_info) {
          left_info_valid = true;
          left_end_s = left_boundary_info->total_length();
        }
        if (need_extract_right && right_boundary_info) {
          right_info_valid = true;
          right_end_s = right_boundary_info->total_length();
        }
      } else {
        if (need_extract_left && left_boundary_info &&
            left_boundary_info->GetProjection(end_point, &left_end_s, &l)) {
          left_info_valid = true;
        }
        if (need_extract_right && right_boundary_info &&
            right_boundary_info->GetProjection(end_point, &right_end_s, &l)) {
          right_info_valid = true;
        }
      }
    }
    if (left_info_valid && i <= stop_left_index) {
      auto move_info = GetBoundaryMoveInfo(left_boundary_info);
      GetMapBoundaryPoint(left_boundary_info, left_start_s, left_end_s,
                          move_info, boundary_points.first, true);
    }
    if (right_info_valid && i <= stop_right_index) {
      auto move_info = GetBoundaryMoveInfo(right_boundary_info);
      GetMapBoundaryPoint(right_boundary_info, right_start_s, right_end_s,
                          move_info, boundary_points.second, true);
    }
  }

  const auto segment_type = segment.GetSegmentType();
  if (segment_type == hdmap::RouteSegments::SegmentType::InvalidSegment) {
    return;
  }
  hdmap_new::Id map_id;
  map_id.set_id(current_id);
  const auto &map_lane_info =
      hdmap::GetLaneById(hdmap::HDMapUtil::BaseMapPtr(), map_id);
  if (!map_lane_info ||
      map_lane_info->lane().center_line().offset_cm().size() <= 0) {
    return;
  }
  const double ego_path_s =
      ego_map_s + map_lane_info->lane().center_line().offset_cm().at(0) *
                      kCentimeterToMetre;
  hdmap_new::Id left_boundary_type_id, right_boundary_type_id;
  if (segment_type == hdmap::RouteSegments::SegmentType::CurrentSegment) {
    left_boundary_type_id = map_lane_info->lane().left_boundary_id();
    right_boundary_type_id = map_lane_info->lane().right_boundary_id();
  } else {
    for (const auto &section : pnc_map_->GetNavInfo()->sections) {
      if (section.section_id == pnc_map_->GetNavInfo()->current_section_id) {
        uint8_t target_seq = 0;
        if (segment_type == hdmap::RouteSegments::SegmentType::LeftSegment) {
          target_seq = pnc_map_->GetNavInfo()->currrent_lane_pose + 1;
        } else if (segment_type ==
                   hdmap::RouteSegments::SegmentType::RightSegment) {
          target_seq = pnc_map_->GetNavInfo()->currrent_lane_pose - 1;
        } else {
          return;
        }
        for (const auto &target_lane : section.lanes) {
          if (target_lane.second.lane_sequence == target_seq) {
            hdmap_new::Id target_id;
            target_id.set_id(target_lane.first);
            const auto &target_lane_info =
                hdmap::GetLaneById(hdmap::HDMapUtil::BaseMapPtr(), target_id);
            if (!target_lane_info) return;
            left_boundary_type_id = target_lane_info->lane().left_boundary_id();
            right_boundary_type_id =
                target_lane_info->lane().right_boundary_id();
            break;
          }
        }
        break;
      }
    }
  }
  const auto &left_boundary_info = hdmap::GetLaneBoundaryById(
      hdmap::HDMapUtil::BaseMapPtr(), left_boundary_type_id);
  const auto &right_boundary_info = hdmap::GetLaneBoundaryById(
      hdmap::HDMapUtil::BaseMapPtr(), right_boundary_type_id);

  if (left_boundary_info) {
    for (const auto &type :
         left_boundary_info->lane_boundary().boundary_attr()) {
      if (ego_path_s + kForwardBoudaryDisBuffer >=
              type.distance_info().start_offset_cm() * kCentimeterToMetre &&
          ego_path_s <
              type.distance_info().end_offset_cm() * kCentimeterToMetre +
                  kBackBoudaryDisBuffer) {
        line_type.first = type;
        break;
      }
    }
  }

  if (right_boundary_info) {
    for (const auto &type :
         right_boundary_info->lane_boundary().boundary_attr()) {
      if (ego_path_s + kForwardBoudaryDisBuffer >=
              type.distance_info().start_offset_cm() * kCentimeterToMetre &&
          ego_path_s <
              type.distance_info().end_offset_cm() * kCentimeterToMetre +
                  kBackBoudaryDisBuffer) {
        line_type.second = type;
        break;
      }
    }
  }
}

std::pair<bool, float> LocalRouteProvider::GetBoundaryMoveInfo(
    const map_service::zmap::LaneBoundaryInfoConstPtr &map_boundary_info) {
  std::pair<bool, float> move_info{false, 0.0};
  uint32_t curren_line_cnt = map_boundary_info->lane_boundary().line_count();
  if (curren_line_cnt > 1) {
    const float boundry_width =
        map_boundary_info->lane_boundary().boundary_width_cm() *
        kCentimeterToMetre;
    move_info.first = true;
    move_info.second = 0.85 * boundry_width *
                       (static_cast<float>((curren_line_cnt - 1)) /
                        static_cast<float>(curren_line_cnt));
    ADEBUG << "move width: " << move_info.second
           << "   bound width: " << boundry_width
           << "   line cnt: " << curren_line_cnt;
  }
  return move_info;
}

int LocalRouteProvider::CheckLaneWidth(const LocalRoute &refline,
                                       BoundaryMoveInfo &last_move_info) {
  int res = 0;
  // TODO(@YFH): move boundary need consider last boudanry or centerline
  return res;
  const double kMaxLaneWidth = local_route_config_.max_lane_width;
  std::string last_lane_id;
  std::list<std::pair<map_service::zmap::LaneBoundaryInfoConstPtr,
                      map_service::zmap::LaneBoundaryInfoConstPtr>>
      boundary_infos;
  double ref_s, left_ref_l, right_ref_l;
  bool find_error_width = false;
  std::string width_error_lane, width_error_section;
  ::math::Vec2d last_ref_point;

  ::common::SLPoint ego_sl;
  if (!refline.XYToSL(polynomial_fitter_->GetEgoPoint(), ego_sl)) {
    return res;
  }
  double forward_distance =
      polynomial_fitter_->GetVehicleState().linear_velocity() * 4.5;
  forward_distance = std::fmax(forward_distance, 50.0);
  uint32_t final_index =
      refline.GetNearestPointIndex(ego_sl.s() + forward_distance);
  std::string ego_lane_id = pnc_map_->GetNavInfo()->current_lane_id;
  bool confirm_start = false;

  uint32_t index = 0;
  for (const auto &ref_point : refline.RoutePoints()) {
    if (ref_point.lane_waypoints().size() == 0) {
      break;
    }
    if (index >= final_index) {
      break;
    }
    std::string current_lane_id =
        ref_point.lane_waypoints().begin()->lane->lane().id().id();

    if (!confirm_start && current_lane_id == ego_lane_id) {
      confirm_start = true;
    }
    if (!confirm_start) {
      index++;
      continue;
    }
    if (index != 0) {
      if (ref_point.DistanceTo(last_ref_point) < 5.0) {
        index++;
        continue;
      }
    }

    last_ref_point = ref_point;
    if (current_lane_id != last_lane_id) {
      const auto &left_boundary = hdmap::GetLaneBoundaryById(
          hdmap::HDMapUtil::BaseMapPtr(),
          ref_point.lane_waypoints().begin()->lane->lane().left_boundary_id());
      const auto &right_boundary = hdmap::GetLaneBoundaryById(
          hdmap::HDMapUtil::BaseMapPtr(),
          ref_point.lane_waypoints().begin()->lane->lane().right_boundary_id());
      if (!left_boundary || !right_boundary) {
        break;
      }
      boundary_infos.emplace_back(
          std::make_pair<>(left_boundary, right_boundary));
      last_lane_id = current_lane_id;
    }
    if (boundary_infos.size() == 0) {
      break;
    }
    if (!boundary_infos.back().first->GetProjection(ref_point, &ref_s,
                                                    &left_ref_l) ||
        !boundary_infos.back().second->GetProjection(ref_point, &ref_s,
                                                     &right_ref_l)) {
      break;
    }
    if (fabs(left_ref_l - right_ref_l) > kMaxLaneWidth) {
      find_error_width = true;
      width_error_lane = current_lane_id;
      width_error_section =
          ref_point.lane_waypoints().at(0).lane->lane().road_section_id().id();
      AINFO << "find lane widht error, left_ref_l: " << left_ref_l
            << "  right_ref_l: " << right_ref_l
            << "   lane: " << width_error_lane;
      break;
    }
    index++;
  }
  double time_now = zark::common::Clock::NowInSeconds();
  if (!find_error_width) {
    if (last_move_info.move_base != 0 &&
        time_now - last_move_info.start_timestamp <= 0.5) {
      res = last_move_info.move_base;
    } else {
      last_move_info.ResetMoveInfo();
    }
    return res;
  } else {
    if (last_move_info.move_base != 0) {
      res = last_move_info.move_base;
      last_move_info.start_timestamp = time_now;
      return res;
    }
  }

  const std::string current_lane = pnc_map_->GetNavInfo()->current_lane_id;
  const std::string current_section =
      pnc_map_->GetNavInfo()->current_section_id;
  const auto &navi_sections = pnc_map_->GetNavInfo()->sections;
  const uint8_t current_lane_seq =
      pnc_map_->GetNavInfo()->current_lane_info.lane_sequence;
  const int8_t kMaxCycle = 20;
  uint8_t max_remain_seq = 0;
  std::pair<bool, bool> neighbor_lc_cur;  // left lane change to right;right
                                          // lane change to left;
  for (auto navi_section = navi_sections.begin();
       navi_section != navi_sections.end(); navi_section++) {
    if (navi_section->section_id == current_section) {
      for (const auto &lane : navi_section->lanes) {
        uint8_t lane_seq = lane.second.lane_sequence;
        if (lane_seq == current_lane_seq + 1 &&
            lane.second.change_direction ==
                eLCRD_ForceLaneChgReqDir::LCRD_Right_LC) {
          neighbor_lc_cur.first = true;
        }
        if (lane_seq == current_lane_seq - 1 &&
            lane.second.change_direction ==
                eLCRD_ForceLaneChgReqDir::LCRD_Left_LC) {
          neighbor_lc_cur.second = true;
        }
      }
    }
    if (neighbor_lc_cur.first || neighbor_lc_cur.second) {
      AINFO << "lane width error, but neighbor change to cur.";
      break;
    }
    if (navi_section->section_id == width_error_section) {
      double max_remain = 0.0;
      auto next_section = navi_section;
      std::string next_id = width_error_lane;
      int8_t j = 0;
      while (next_section != navi_sections.end() && j < kMaxCycle) {
        j++;
        const auto next_lane = next_section->lanes.find(next_id);
        if (next_lane == next_section->lanes.end()) {
          break;
        }
        if (next_lane->second.lane_topo.successor_id_size() == 0) {
          break;
        }
        if (next_lane->second.lane_topo.successor_id_size() > 1) {
          for (const auto &successor_id :
               next_lane->second.lane_topo.successor_id()) {
            auto forward_section = std::next(next_section);
            if (forward_section == navi_sections.end()) {
              break;
            }
            const auto successor_lane =
                forward_section->lanes.find(successor_id.id());
            if (successor_lane != forward_section->lanes.end()) {
              if (successor_lane->second.remain_distance > max_remain) {
                max_remain = successor_lane->second.remain_distance;
                max_remain_seq = successor_lane->second.lane_sequence;
              }
            } else {
              break;
            }
          }
          break;
        } else {
          auto change_dir = next_lane->second.change_direction;
          if (next_lane->second.change_count > 0 &&
              change_dir != eLCRD_ForceLaneChgReqDir::LCRD_None_LC) {
            if (change_dir == eLCRD_ForceLaneChgReqDir::LCRD_Left_LC) {
              AINFO << "change to left lane, use left boundary move to "
                       "replace left";
              max_remain_seq = 2;
            } else {
              max_remain_seq = 1;
              AINFO << "change to right lane, use right boundary move to "
                       "replace right";
            }
            break;
          }
          next_id = next_lane->second.lane_topo.successor_id().begin()->id();
        }
        next_section++;
      }
      break;
    }
  }
  if (neighbor_lc_cur.first) {
    res = -1;  // left lane change dir is to cur, use right line as base line;
  } else if (neighbor_lc_cur.second) {
    res = 1;  // right lane change dir is to cur, use left line as base line;
  } else if (max_remain_seq == 1) {
    res = -1;
  } else if (max_remain_seq > 1) {
    res = 1;
  }
  if (res != 0) {
    last_move_info.move_base = res;
    last_move_info.start_timestamp = zark::common::Clock::NowInSeconds();
  }
  return res;
}

void LocalRouteProvider::FillBoundaryType(
    const hdmap_new::LaneBoundaryAttribute &boundary_type,
    zark::ads_common::B1_LaneLineInfo &out_line) {
  switch (boundary_type.type()) {
    case hdmap_new::LaneBoundaryAttribute_Type::
        LaneBoundaryAttribute_Type_LANEBOUNDARY_TYPE_UNKNOWN:
      out_line.set_lli_stat_linetype_elmt(
          zark::ads_common::eLMT_LaneMrkrTyp::LMT_Unknown);
      break;
    case hdmap_new::LaneBoundaryAttribute_Type::
        LaneBoundaryAttribute_Type_LANEBOUNDARY_TYPE_NO_MARKING:
      out_line.set_lli_stat_linetype_elmt(
          zark::ads_common::eLMT_LaneMrkrTyp::LMT_None);
      break;
    case hdmap_new::LaneBoundaryAttribute_Type::
        LaneBoundaryAttribute_Type_LANEBOUNDARY_TYPE_LONG_DASHED_LINE:
    case hdmap_new::LaneBoundaryAttribute_Type::
        LaneBoundaryAttribute_Type_LANEBOUNDARY_TYPE_SHORT_DASHED_LINE:
    case hdmap_new::LaneBoundaryAttribute_Type::
        LaneBoundaryAttribute_Type_LANEBOUNDARY_TYPE_SINGLE_DASHED_LINE:
    case hdmap_new::LaneBoundaryAttribute_Type::
        LaneBoundaryAttribute_Type_LANEBOUNDARY_TYPE_DENSE_WIDE_DASH:
    case hdmap_new::LaneBoundaryAttribute_Type::
        LaneBoundaryAttribute_Type_LANEBOUNDARY_TYPE_DIAMOND_DECELERATION_LINE:
      out_line.set_lli_stat_linetype_elmt(
          zark::ads_common::eLMT_LaneMrkrTyp::LMT_Dashed);
      break;
    case hdmap_new::LaneBoundaryAttribute_Type::
        LaneBoundaryAttribute_Type_LANEBOUNDARY_TYPE_DOUBLE_DASH:
    case hdmap_new::LaneBoundaryAttribute_Type::
        LaneBoundaryAttribute_Type_LANEBOUNDARY_TYPE_DOUBLE_DIAMOND_DECELERATION:
    case hdmap_new::LaneBoundaryAttribute_Type::
        LaneBoundaryAttribute_Type_LANEBOUNDARY_TYPE_LEFT_DASHED_AND_RIGHT_DIAMOND_DECELERATION:
    case hdmap_new::LaneBoundaryAttribute_Type::
        LaneBoundaryAttribute_Type_LANEBOUNDARY_TYPE_RIGHT_DASHED_AND_LEFT_DIAMOND_DECELERATION:
      out_line.set_lli_stat_linetype_elmt(
          zark::ads_common::eLMT_LaneMrkrTyp::LMT_DoubleDashedLine);
      break;
    case hdmap_new::LaneBoundaryAttribute_Type::
        LaneBoundaryAttribute_Type_LANEBOUNDARY_TYPE_SINGLE_SOLID_LINE:
      out_line.set_lli_stat_linetype_elmt(
          zark::ads_common::eLMT_LaneMrkrTyp::LMT_Solid);
      break;
    case hdmap_new::LaneBoundaryAttribute_Type::
        LaneBoundaryAttribute_Type_LANEBOUNDARY_TYPE_DOUBLE_SOLID:
      out_line.set_lli_stat_linetype_elmt(
          zark::ads_common::eLMT_LaneMrkrTyp::LMT_DoubleSolidLine);
      break;
    case hdmap_new::LaneBoundaryAttribute_Type::
        LaneBoundaryAttribute_Type_LANEBOUNDARY_TYPE_LEFT_SOLID_AND_RIGHT_DASH:
    case hdmap_new::LaneBoundaryAttribute_Type::
        LaneBoundaryAttribute_Type_LANEBOUNDARY_TYPE_LEFT_SOLID_AND_RIGHT_DIAMOND_DECELERATION:
      out_line.set_lli_stat_linetype_elmt(
          zark::ads_common::eLMT_LaneMrkrTyp::LMT_LeftSolidRightDashed);
      break;
    case hdmap_new::LaneBoundaryAttribute_Type::
        LaneBoundaryAttribute_Type_LANEBOUNDARY_TYPE_RIGHT_SOLID_AND_LEFT_DASH:
    case hdmap_new::LaneBoundaryAttribute_Type::
        LaneBoundaryAttribute_Type_LANEBOUNDARY_TYPE_RIGHT_SOLID_AND_LEFT_DIAMOND_DECELERATION:
      out_line.set_lli_stat_linetype_elmt(
          zark::ads_common::eLMT_LaneMrkrTyp::LMT_RightSolidLeftDashed);
      break;
    case hdmap_new::LaneBoundaryAttribute_Type::
        LaneBoundaryAttribute_Type_LANEBOUNDARY_TYPE_CURB:
      out_line.set_lli_stat_linetype_elmt(
          zark::ads_common::eLMT_LaneMrkrTyp::LMT_Curbstone);
      break;
    default:
      out_line.set_lli_stat_linetype_elmt(
          zark::ads_common::eLMT_LaneMrkrTyp::LMT_None);
      break;
  }
  switch (boundary_type.color()) {
    case hdmap_new::Color::COLOR_UNKNOWN:
    case hdmap_new::Color::COLOR_OTHERS:
      out_line.set_lli_stat_linecolor_elmc(
          zark::ads_common::eLMC_LaneMrkrColor::LMC_Unknown);
      break;
    case hdmap_new::Color::COLOR_WHITE:
      out_line.set_lli_stat_linecolor_elmc(
          zark::ads_common::eLMC_LaneMrkrColor::LMC_White);
      break;
    case hdmap_new::Color::COLOR_YELLOW:
      out_line.set_lli_stat_linecolor_elmc(
          zark::ads_common::eLMC_LaneMrkrColor::LMC_Yellow);
      break;
    case hdmap_new::Color::COLOR_ORANGE:
      out_line.set_lli_stat_linecolor_elmc(
          zark::ads_common::eLMC_LaneMrkrColor::LMC_Orange);
      break;
    case hdmap_new::Color::COLOR_BLUE:
      out_line.set_lli_stat_linecolor_elmc(
          zark::ads_common::eLMC_LaneMrkrColor::LMC_Blue);
      break;
    case hdmap_new::Color::COLOR_GREEN:
      out_line.set_lli_stat_linecolor_elmc(
          zark::ads_common::eLMC_LaneMrkrColor::LMC_Green);
      break;
    case hdmap_new::Color::COLOR_GRAY:
      out_line.set_lli_stat_linecolor_elmc(
          zark::ads_common::eLMC_LaneMrkrColor::LMC_Gray);
      break;
    case hdmap_new::Color::COLOR_RED:
      out_line.set_lli_stat_linecolor_elmc(
          zark::ads_common::eLMC_LaneMrkrColor::LMC_Red);
      break;
    case hdmap_new::Color::COLOR_LEFT_YELLOW_RIGHT_WHITE:
      AERROR << "COLOR_LEFT_YELLOW_RIGHT_WHITE...";
      out_line.set_lli_stat_linecolor_elmc(
          zark::ads_common::eLMC_LaneMrkrColor::LMC_Unknown);
      break;
    case hdmap_new::Color::COLOR_LEFT_GRAY_RIGHT_YELLOW:
      AERROR << "COLOR_LEFT_GRAY_RIGHT_YELLOW...";
      out_line.set_lli_stat_linecolor_elmc(
          zark::ads_common::eLMC_LaneMrkrColor::LMC_Unknown);
      break;
    default:
      out_line.set_lli_stat_linecolor_elmc(
          zark::ads_common::eLMC_LaneMrkrColor::LMC_NA);
      break;
  }
}

void LocalRouteProvider::ProcessBoundaryLine(
    const LocalRoute &refline, const hdmap::RouteSegments &segment,
    const MapPolynomialFit::FittedLine &center_line, const double ego_s,
    const SpecialLaneInfo &split_merge_info, B1_LaneLineInfo &left_bound_line,
    B1_LaneLineInfo &right_bound_line) {
  std::pair<hdmap_new::LaneBoundaryAttribute, hdmap_new::LaneBoundaryAttribute>
      boundary_type;
  // need consider split lane, generate virtual boundary;
  MapPolynomialFit::FittedLine left_fit_line, right_fit_line, noa_fitted_line;
  std::pair<std::vector<LocalRoutePoint>, std::vector<LocalRoutePoint>>
      boundary_point;
  int move_base = 0;
  const int fit_order = kEdgeFitteOrder;
  const double kNormalLaneWidth = local_route_config_.normal_lane_width;
  ExtractBoudaryPoint(refline, segment, ego_s, split_merge_info, boundary_type,
                      boundary_point, move_base);
  FillBoundaryType(boundary_type.first, left_bound_line);
  FillBoundaryType(boundary_type.second, right_bound_line);
  const uint8_t kMinSmoothPointNum = 3;
  CalRoutePointHeading(boundary_point.first);
  CalRoutePointHeading(boundary_point.second);
  if (boundary_point.first.size() >= kMinSmoothPointNum) {
    LocalRoute left_line(boundary_point.first, local_route_config_, true);
    LocalRoute smoothed_left_line;
    const double smooth_start_time = zark::common::Clock::NowInSeconds();
    if (SmoothBoundaryLine(left_line, smoothed_left_line)) {
      const double smooth_end_time = zark::common::Clock::NowInSeconds();
      AINFO << " left bound smooth time diff ms: "
            << (smooth_end_time - smooth_start_time) * 1000;
      polynomial_fitter_->DiscretePointsFit(smoothed_left_line.RoutePoints(),
                                            fit_order, left_fit_line);
      FillOutputLine(left_fit_line, left_bound_line);
      if (move_base == 1) {
        boundary_point.second.clear();
        std::vector<double> headings;
        if (PostHeadingProcess(smoothed_left_line.RoutePoints(), headings)) {
          uint32_t index = 0;
          for (const auto &point : smoothed_left_line.RoutePoints()) {
            boundary_point.second.emplace_back(std::move(LocalRoutePoint(
                std::move(hdmap::MapPathPoint(
                    std::move(GetWidthPoint(point, -kNormalLaneWidth,
                                            headings[index])),
                    0.0)),
                0.0, 0.0)));
            index++;
          }
        }
      }
    } else {
      AERROR << "smoothed boundary line failed..";
    }
  } else {
    AERROR << "Extract LEFT BoudaryPoint failed, point size: "
           << boundary_point.first.size();
  }
  if (boundary_point.second.size() >= kMinSmoothPointNum) {
    LocalRoute right_line(boundary_point.second, local_route_config_, true);
    LocalRoute smoothed_right_line;
    const double smooth_start_time = zark::common::Clock::NowInSeconds();
    if (move_base == 1) {
      polynomial_fitter_->DiscretePointsFit(right_line.RoutePoints(), fit_order,
                                            right_fit_line);
      FillOutputLine(right_fit_line, right_bound_line);
    } else if (SmoothBoundaryLine(right_line, smoothed_right_line)) {
      const double smooth_end_time = zark::common::Clock::NowInSeconds();
      AINFO << " right bound smooth time diff ms: "
            << (smooth_end_time - smooth_start_time) * 1000;
      polynomial_fitter_->DiscretePointsFit(smoothed_right_line.RoutePoints(),
                                            fit_order, right_fit_line);
      FillOutputLine(right_fit_line, right_bound_line);
      if (move_base == -1) {
        boundary_point.first.clear();
        std::vector<double> headings;
        if (PostHeadingProcess(smoothed_right_line.RoutePoints(), headings)) {
          uint32_t index = 0;
          for (const auto &point : smoothed_right_line.RoutePoints()) {
            boundary_point.first.emplace_back(std::move(LocalRoutePoint(
                std::move(hdmap::MapPathPoint(
                    std::move(GetWidthPoint(point, kNormalLaneWidth,
                                            headings[index])),
                    0.0)),
                0.0, 0.0)));
            index++;
          }
        }
        polynomial_fitter_->DiscretePointsFit(boundary_point.first, fit_order,
                                              left_fit_line);
        FillOutputLine(left_fit_line, left_bound_line);
      }
    } else {
      AERROR << "smoothed boundary line failed...";
    }
  } else {
    AERROR << "Extract right BoudaryPoint failed, point size: "
           << boundary_point.second.size();
  }
  if (split_merge_info.split_in_left) {
    MapPolynomialFit::FittedLine virtual_line;
    ProcessSplitBoundary(center_line, left_fit_line, virtual_line);
    FillOutputLine(virtual_line, right_bound_line);
  } else if (split_merge_info.split_in_right) {
    MapPolynomialFit::FittedLine virtual_line;
    ProcessSplitBoundary(center_line, right_fit_line, virtual_line);
    FillOutputLine(virtual_line, left_bound_line);
  } else if (split_merge_info.right_is_merge) {
    MapPolynomialFit::FittedLine virtual_line;
    ProcessMergeBoundary(segment, split_merge_info, left_fit_line,
                         virtual_line);
    FillOutputLine(virtual_line, right_bound_line);
  } else if (split_merge_info.left_is_merge) {
    MapPolynomialFit::FittedLine virtual_line;
    ProcessMergeBoundary(segment, split_merge_info, right_fit_line,
                         virtual_line);
    FillOutputLine(virtual_line, left_bound_line);
  }
}

void LocalRouteProvider::ProcessSplitBoundary(
    const MapPolynomialFit::FittedLine &center_line,
    const MapPolynomialFit::FittedLine &oppsite_fitted_line,
    MapPolynomialFit::FittedLine &bound_line) {
  if (center_line.polynomial_coefficients_.size() == 0 ||
      oppsite_fitted_line.polynomial_coefficients_.size() == 0) {
    return;
  }
  double lateral_dist = center_line.polynomial_coefficients_[0] -
                        oppsite_fitted_line.polynomial_coefficients_[0];
  if (lateral_dist >= 0.0) {
    lateral_dist = std::clamp(lateral_dist, 1.25,
                              local_route_config_.normal_lane_width / 2.0);
  } else {
    lateral_dist = std::clamp(
        lateral_dist, -local_route_config_.normal_lane_width / 2.0, -1.25);
  }
  bound_line = oppsite_fitted_line;
  bound_line.polynomial_coefficients_[0] += lateral_dist * 2;
}

void LocalRouteProvider::ProcessMergeBoundary(
    const hdmap::RouteSegments &segment,
    const SpecialLaneInfo &split_merge_info,
    const MapPolynomialFit::FittedLine &oppsite_fitted_line,
    MapPolynomialFit::FittedLine &bound_line) {
  if (oppsite_fitted_line.polynomial_coefficients_.size() == 0) return;
  const uint8_t kSegPointNum = 5;
  float lane_width_sum = 0.0;
  uint8_t final_width_num = 0;
  float mean_width = 0.0;
  for (const auto &lane_seg : segment) {
    const float lane_lenght = lane_seg.lane->total_length();
    const float s_diff = lane_lenght / kSegPointNum;
    double s = 0.0;
    for (uint8_t i = 0; i <= kSegPointNum; i++) {
      s += s_diff * i;
      zark::hdmap_new::LaneSampleAssociation width_info =
          lane_seg.lane->GetWidth(s);
      lane_width_sum += width_info.width_cm() * kCentimeterToMetre;
      final_width_num++;
    }
  }
  if (final_width_num > 0) {
    const float kValidLaneWidth = 2.5;
    mean_width = lane_width_sum / final_width_num;
    if (mean_width >= kValidLaneWidth) {
      bound_line = oppsite_fitted_line;
      bound_line.polynomial_coefficients_[0] =
          split_merge_info.left_is_merge
              ? bound_line.polynomial_coefficients_[0] + mean_width
              : bound_line.polynomial_coefficients_[0] - mean_width;
      if (split_merge_info.split_in_right) {
        bound_line = oppsite_fitted_line;
        bound_line.polynomial_coefficients_[0] =
            oppsite_fitted_line.polynomial_coefficients_[0] + mean_width;
      } else if (split_merge_info.split_in_left) {
        bound_line = oppsite_fitted_line;
        bound_line.polynomial_coefficients_[0] =
            oppsite_fitted_line.polynomial_coefficients_[0] - mean_width;
      }
    }
  }
}

bool LocalRouteProvider::FindSplit(const LocalRoute &current_line,
                                   const hdmap::RouteSegments &segment,
                                   SpecialLaneInfo &split_merge_info) {
  bool res = false;
  ::common::SLPoint ego_sl;
  if (!current_line.XYToSL(polynomial_fitter_->GetEgoPoint(), ego_sl)) {
    return res;
  }
  const uint8_t kRightMostLane = 1;
  const double kMinForwardDist = 50.0;
  const double kMaxForwardTime = 4.5;  //[m]
  double forward_distance =
      polynomial_fitter_->GetVehicleState().linear_velocity() * kMaxForwardTime;
  forward_distance = std::fmax(forward_distance, kMinForwardDist);
  LocalRoutePoint final_point =
      current_line.GetLocalRoutePoint(ego_sl.s() + forward_distance);
  if (final_point.lane_waypoints().size() == 0) {
    return res;
  }
  const std::string final_point_id =
      final_point.lane_waypoints().at(0).lane->lane().id().id();
  for (const auto &lane_seg : segment) {
    std::string section_id = lane_seg.lane->lane().road_section_id().id();
    for (const auto &section_info : pnc_map_->GetNavInfo()->sections) {
      if (section_info.section_id == section_id) {
        const auto &lane_info =
            section_info.lanes.find(lane_seg.lane->lane().id().id());
        if (lane_info != section_info.lanes.end()) {
          const uint8_t cur_seq = lane_info->second.lane_sequence;
          if (!split_merge_info.split_in_right &&
              !split_merge_info.split_in_left &&
              lane_info->second.lane_topo.transition() ==
                  hdmap_new::Lane_LaneTransition::
                      Lane_LaneTransition_LANE_TRANSITION_SPLIT) {
            res = true;
            if (cur_seq == kRightMostLane) {
              split_merge_info.split_in_right = true;
            } else {
              if (cur_seq == section_info.lanes.size()) {
                split_merge_info.split_in_left = true;
              } else {
                std::unordered_set<std::string> predecessor_id;
                for (const auto &pre_id :
                     lane_info->second.lane_topo.predecessor_id()) {
                  predecessor_id.insert(pre_id.id());
                }
                std::string left_id =
                    lane_info->second.lane_topo.left_neighbor_forward_lane_id()
                        .id();
                std::string right_id =
                    lane_info->second.lane_topo.right_neighbor_forward_lane_id()
                        .id();
                const auto &left_lane = section_info.lanes.find(left_id);
                const auto &right_lane = section_info.lanes.find(right_id);
                if (left_lane != section_info.lanes.end()) {
                  for (const auto &pre_id :
                       left_lane->second.lane_topo.predecessor_id()) {
                    if (predecessor_id.count(pre_id.id())) {
                      split_merge_info.split_in_right = true;
                      break;
                    }
                  }
                }
                if (!split_merge_info.split_in_right &&
                    right_lane != section_info.lanes.end()) {
                  for (const auto &pre_id :
                       right_lane->second.lane_topo.predecessor_id()) {
                    if (predecessor_id.count(pre_id.id())) {
                      split_merge_info.split_in_left = true;
                      break;
                    }
                  }
                }
              }
            }
            AERROR << "find split lane id: " << lane_info->first
                   << "  seq: " << int(cur_seq);
          }
          const uint8_t right_seq = cur_seq - 1;
          const uint8_t left_seq = cur_seq + 1;
          for (const auto &neighbor_lane : section_info.lanes) {
            const uint8_t neighbor_seq = neighbor_lane.second.lane_sequence;
            if (neighbor_seq == cur_seq) continue;
            if (neighbor_lane.second.lane_topo.transition() ==
                hdmap_new::Lane_LaneTransition::
                    Lane_LaneTransition_LANE_TRANSITION_MERGE) {
              if (neighbor_seq == left_seq) {
                split_merge_info.left_is_merge = true;
              } else if (neighbor_seq == right_seq) {
                split_merge_info.right_is_merge = true;
                AINFO << " right_is_merge..";
              }
            }
          }
        }
        break;
      }
    }
    if (lane_seg.lane->lane().id().id() == final_point_id) {
      break;
    }
  }
  return res;
}

const double LocalRouteProvider::LastTimeDelay() {
  return last_calculation_time_;
}

bool LocalRouteProvider::GetLocalRoutes(
    const ::common::TrajectoryPoint &planning_start_point,
    const bool is_acc_mode, std::list<LocalRoute> &local_routes) {
  if (local_route_history_.empty()) {
    AERROR << "Failed to use local route latest history";
    return false;
  }
  {
    std::lock_guard<std::mutex> lock(local_routes_mutex_);
    local_routes.assign(local_route_history_.back().begin(),
                        local_route_history_.back().end());
  }
  AINFO << "Acquir local routes from history!";

  LocalRoute *local_route;
  for (auto &lr : local_routes) {
    if (lr.Lanes().GetSegmentType() ==
        hdmap::RouteSegments::SegmentType::CurrentSegment) {
      local_route = &lr;
    }
  }
  if (local_route == nullptr) {
    AERROR << "Faied to find the current local route";
    return false;
  }

  if (is_acc_mode &&
      (local_route->GetEndS() -
           local_route->ToFrenetFrame(planning_start_point).s[kIdxS] <=
       16)) {
    LocalRoute local_route_acc =
        acc_trajectory_builder_->GenerateLocalRouteForACC(planning_start_point);
    {
      std::lock_guard<std::mutex> lock(local_routes_mutex_);
      local_routes.clear();
      local_routes.push_back(local_route_acc);
    }
    AINFO << "Use local route from acc trajectory builder!";
  }
  return true;
}

bool LocalRouteProvider::CreateRouteSegments(
    const VehicleState &vehicle_state,
    std::list<hdmap::RouteSegments> &segments) {
  {
    std::lock_guard<std::mutex> lock(pnc_map_mutex_);
    if (!pnc_map_->GetRouteSegments(vehicle_state, &segments, is_olm_)) {
      AERROR << "Failed to extract segments from routing";
      return false;
    }
  }
  return !segments.empty();
}

bool LocalRouteProvider::CreateLocalRoute(
    const common::VehicleState &vehicle_state,
    std::list<LocalRoute> &local_routes,
    std::list<hdmap::RouteSegments> &segments) {
  bool update_path = false;
  {
    // Update routing path in pnc_map
    std::lock_guard<std::mutex> lock(pnc_map_mutex_);
    if (pnc_map_->UpdateMainPath()) {
      update_path = true;
    }
  }

  polynomial_fitter_->SetVehicleState(vehicle_state);

  if (!CreateRouteSegments(vehicle_state, segments)) {
    AERROR << "Failed to create local route from routing";
    error_msg_ = "CreateRouteSegments_" + pnc_map_->GetErrorMsg();
    return false;
  }

  if (!pnc_map_->GetNavInfo()) {
    error_msg_ = "pncmap_no_nav_info";
    AERROR << error_msg_;
    return false;
  }

  bool trigger_noaline = pnc_map_->GetNavInfo()->noa_line.is_trigger;

  if (true || update_path) {
    for (auto iter = segments.begin(); iter != segments.end();) {
      local_routes.emplace_back(local_route_config_);
      bool need_smooth = trigger_noaline &&
                         (iter->GetSegmentType() ==
                          hdmap::RouteSegments::SegmentType::CurrentSegment);
      if (!SmoothRouteSegment(*iter, local_routes.back(), need_smooth)) {
        AERROR << "Failed to create local route from route segments";
        if (iter->GetSegmentType() ==
            hdmap::RouteSegments::SegmentType::CurrentSegment) {
          AERROR << "construct current center refline failed!";
          error_msg_ = "smooth_cur_line_error";
          return false;
        }
        local_routes.pop_back();
        iter = segments.erase(iter);
      } else {
        ::common::SLPoint sl;
        if (!local_routes.back().XYToSL(
                Vec2d(vehicle_state.x(), vehicle_state.y()), sl)) {
          AWARN << "Failed to project point: {" << vehicle_state.x() << ","
                << vehicle_state.y() << "} to stitched local route";
        }
        Shrink(sl, local_routes.back(), (*iter));
        local_routes.back().SetLanes(*iter);
        ++iter;
      }
    }
    return true;
  } else {  // stitching local route
    for (auto iter = segments.begin(); iter != segments.end();) {
      local_routes.emplace_back(local_route_config_);
      if (!ExtendLocalRoute(vehicle_state, *iter, local_routes.back())) {
        AERROR << "Failed to extend local route";
        local_routes.pop_back();
        iter = segments.erase(iter);
      } else {
        local_routes.back().SetLanes(*iter);
        ++iter;
      }
    }
  }
  return true;
}

bool LocalRouteProvider::ExtendLocalRoute(const VehicleState &state,
                                          RouteSegments &segments,
                                          LocalRoute &local_route) {
  // stitch previous local route will cause kappa jumps, so cancel the stitch.
  return SmoothRouteSegment(segments, local_route);
  RouteSegments segment_properties;
  if (local_route_history_.empty() || route_segments_history_.empty()) {
    AERROR << "Failed to extend local route from empty history";
    return false;
  }
  auto &cur_local_routes = local_route_history_.back();
  auto &cur_route_segments = route_segments_history_.back();
  segment_properties.SetProperties(segments);
  auto prev_segment = cur_route_segments.begin();
  auto prev_ref = cur_local_routes.begin();
  std::list<std::pair<std::list<LocalRoute>::iterator,
                      std::list<RouteSegments>::iterator>>
      confirm_table;
  while (prev_segment != cur_route_segments.end()) {
    if (prev_segment->IsConnectedSegment(segments)) {
      confirm_table.emplace_back(prev_ref, prev_segment);
    }
    ++prev_segment;
    ++prev_ref;
  }
  if (confirm_table.size() > 1) {
    uint16_t connect_count = 0;
    for (auto &table_iter : confirm_table) {
      uint16_t table_seg_con_cnt = 0;
      for (auto table_seg = table_iter.second->begin();
           table_seg != table_iter.second->end(); table_seg++) {
        for (auto lane_seg = segments.begin(); lane_seg != segments.end();
             lane_seg++) {
          if (table_seg->lane->lane().id().id() ==
              lane_seg->lane->lane().id().id()) {
            table_seg_con_cnt++;
          }
        }
      }
      if (table_seg_con_cnt > connect_count) {
        connect_count = table_seg_con_cnt;
        prev_segment = table_iter.second;
        prev_ref = table_iter.first;
      }
    }
  } else if (confirm_table.size() == 1) {
    prev_segment = confirm_table.begin()->second;
    prev_ref = confirm_table.begin()->first;
  }
  if (prev_segment == cur_route_segments.end()) {
    if (!cur_route_segments.empty() && segments.IsOnSegment()) {
      AWARN << "Current route segment is not connected with previous route "
               "segment";
    }
    return SmoothRouteSegment(segments, local_route);
  }
  ::common::SLPoint sl_point;
  Vec2d vec2d(state.x(), state.y());
  LaneWaypoint waypoint;
  if (!prev_segment->GetProjection(vec2d, &sl_point, &waypoint)) {
    AWARN << "Vehicle current point: " << vec2d.DebugString()
          << " not on previous local route";
    return SmoothRouteSegment(segments, local_route);
  }
  const double prev_segment_length = RouteSegments::Length(*prev_segment);
  const double remain_s = prev_segment_length - sl_point.s();
  const double look_forward_required_distance =
      pnc_map_->LookForwardDistance(state.linear_velocity());
  if (remain_s > look_forward_required_distance) {
    segments = *prev_segment;
    segments.SetProperties(segment_properties);
    local_route = *prev_ref;
    ADEBUG << "Local route remain " << remain_s
           << ", which is more than required " << look_forward_required_distance
           << " and no need to extend";
    local_route.SetLanes(segments);
    return true;
  }
  double future_start_s =
      std::max(sl_point.s(),
               prev_segment_length -
                   local_route_config_.local_route_stitch_overlap_distance);
  double future_end_s =
      prev_segment_length + local_route_config_.look_forward_extend_distance;
  RouteSegments shifted_segments;
  std::unique_lock<std::mutex> lock(pnc_map_mutex_);
  if (!pnc_map_->ExtendSegments(*prev_segment, future_start_s, future_end_s,
                                &shifted_segments)) {
    lock.unlock();
    AERROR << "Failed to shift route segments forward";
    return SmoothRouteSegment(segments, local_route);
  }
  lock.unlock();
  if (prev_segment->IsWaypointOnSegment(shifted_segments.LastWaypoint())) {
    segments = *prev_segment;
    segments.SetProperties(segment_properties);
    local_route = *prev_ref;
    local_route.SetLanes(segments);
    ADEBUG << "Could not further extend local route";
    return true;
  }
  LocalRoute new_ref(shifted_segments, local_route_config_);
  if (!SmoothPrefixedLocalRoute(*prev_ref, new_ref, local_route)) {
    AWARN << "Failed to smooth forward shifted local route";
    return SmoothRouteSegment(segments, local_route);
  }
  if (!local_route.Stitch(*prev_ref)) {
    AWARN << "Failed to stitch local route";
    return SmoothRouteSegment(segments, local_route);
  }
  if (!shifted_segments.Stitch(*prev_segment)) {
    AWARN << "Failed to stitch route segments";
    return SmoothRouteSegment(segments, local_route);
  }
  segments = shifted_segments;
  segments.SetProperties(segment_properties);
  ::common::SLPoint sl;
  if (!local_route.XYToSL(vec2d, sl)) {
    AWARN << "Failed to project point: " << vec2d.DebugString()
          << " to stitched local route";
  }
  return Shrink(sl, local_route, segments);
}

bool LocalRouteProvider::GetLastRefRoute(const hdmap::RouteSegments &segments,
                                         LocalRoute &last_ref_route) {
  if (segments.size() == 0 || local_route_history_.size() == 0 ||
      route_segments_history_.size() == 0) {
    return false;
  }
  auto &cur_local_routes = local_route_history_.back();
  auto &cur_route_segments = route_segments_history_.back();
  auto prev_segment = cur_route_segments.begin();
  auto prev_ref = cur_local_routes.begin();
  std::list<std::pair<std::list<LocalRoute>::iterator,
                      std::list<RouteSegments>::iterator>>
      confirm_table;
  while (prev_segment != cur_route_segments.end()) {
    if (prev_segment->IsConnectedSegment(segments)) {
      confirm_table.emplace_back(prev_ref, prev_segment);
    }
    ++prev_segment;
    ++prev_ref;
  }
  if (confirm_table.size() > 1) {
    uint16_t connect_count = 0;
    for (auto &table_iter : confirm_table) {
      uint16_t table_seg_con_cnt = 0;
      for (auto table_seg = table_iter.second->begin();
           table_seg != table_iter.second->end(); table_seg++) {
        for (auto lane_seg = segments.begin(); lane_seg != segments.end();
             lane_seg++) {
          if (table_seg->lane->lane().id().id() ==
              lane_seg->lane->lane().id().id()) {
            table_seg_con_cnt++;
          }
        }
      }
      if (table_seg_con_cnt > connect_count) {
        connect_count = table_seg_con_cnt;
        prev_segment = table_iter.second;
        prev_ref = table_iter.first;
      }
    }
  } else if (confirm_table.size() == 1) {
    prev_segment = confirm_table.begin()->second;
    prev_ref = confirm_table.begin()->first;
  } else {
    return false;
  }
  if (prev_segment != cur_route_segments.end()) {
    last_ref_route = *prev_ref;
    if (last_ref_route.RoutePoints().size() > 0) {
      return true;
    }
  }
  return false;
}

bool LocalRouteProvider::Shrink(const ::common::SLPoint &sl,
                                LocalRoute &local_route,
                                RouteSegments &segments) {
  static constexpr double kMaxHeadingDiff = M_PI * 5.0 / 6.0;
  // shrink local route
  double new_backward_distance = sl.s();
  double new_forward_distance = local_route.Length() - sl.s();
  bool need_shrink = false;
  if (sl.s() > local_route_config_.look_backward_distance * 1.5) {
    ADEBUG << "local route back side is " << sl.s()
           << ", shrink local route: origin length: " << local_route.Length();
    new_backward_distance = local_route_config_.look_backward_distance;
    need_shrink = true;
  }
  // check heading
  const auto index = local_route.GetNearestPointIndex(sl.s());
  const auto &ref_points = local_route.RoutePoints();
  const double cur_heading = ref_points[index].heading();
  auto last_index = index;
  while (last_index < ref_points.size() &&
         AngleDiff(cur_heading, ref_points[last_index].heading()) <
             kMaxHeadingDiff) {
    ++last_index;
  }
  --last_index;
  if (last_index != ref_points.size() - 1) {
    need_shrink = true;
    ::common::SLPoint forward_sl;
    local_route.XYToSL(ref_points[last_index], forward_sl);
    new_forward_distance = forward_sl.s() - sl.s();
  }
  if (need_shrink) {
    if (!local_route.Segment(sl.s(), new_backward_distance,
                             new_forward_distance)) {
      AWARN << "Failed to shrink local route";
    }
    if (!segments.Shrink(sl.s(), new_backward_distance, new_forward_distance)) {
      AWARN << "Failed to shrink route segment";
    }
  }
  local_route.SetLanes(segments);
  return true;
}

bool LocalRouteProvider::IsLocalRouteSmoothValid(
    const LocalRoute &raw, const LocalRoute &smoothed) const {
  static constexpr double kLocalRouteDiffCheckStep = 10.0;
  for (double s = 0.0; s < smoothed.Length(); s += kLocalRouteDiffCheckStep) {
    auto xy_new = smoothed.GetLocalRoutePoint(s);

    ::common::SLPoint sl_new;
    if (!raw.XYToSL(xy_new, sl_new)) {
      AERROR << "Fail to change xy point on smoothed local route to sl "
                "point respect to raw local route.";
      return false;
    }

    const double diff = std::fabs(sl_new.l());
    if (diff > local_route_config_.smoothed_local_route_max_diff) {
      AERROR << "Fail to provide local route because too large diff "
                "between smoothed and raw reference lines. diff: "
             << diff;
      return false;
    }
  }
  return true;
}

RouteAnchorPoint LocalRouteProvider::GetAnchorPoint(
    const LocalRoute &local_route, double s) const {
  RouteAnchorPoint anchor;
  anchor.longitudinal_bound =
      local_route_config_.smoother_config.longitudinal_boundary_bound();
  auto route_point = local_route.GetLocalRoutePoint(s);
  if (route_point.lane_waypoints().empty()) {
    anchor.path_point = route_point.ToPathPoint(s);
    anchor.lateral_bound =
        local_route_config_.smoother_config.max_lateral_boundary_bound();
    return anchor;
  }

  const double adc_width =
      VehicleConfigHelper::GetConfig().vehicle_param().width();
  const Vec2d left_vec =
      Vec2d::CreateUnitVec2d(route_point.heading() + M_PI / 2.0);
  auto waypoint = route_point.lane_waypoints().front();
  zark::hdmap_new::LaneSampleAssociation lane_width =
      waypoint.lane->GetWidth(waypoint.s / hdmap::kCentimeterToMetre);
  const double kEpislon = 1e-8;
  double effective_width = 0.0;

  // shrink width by vehicle width, curb
  double safe_lane_width = lane_width.width_cm() * hdmap::kCentimeterToMetre;
  safe_lane_width -= adc_width;
  bool is_lane_width_safe = true;

  if (safe_lane_width < kEpislon) {
    ADEBUG << "lane width ["
           << lane_width.width_cm() * hdmap::kCentimeterToMetre << "] "
           << "is smaller than adc width [" << adc_width << "]";
    effective_width = kEpislon;
    is_lane_width_safe = false;
  }

  double center_shift = 0.0;
  if (hdmap::RightBoundaryType(waypoint) ==
      hdmap_new::LaneBoundaryAttribute_Type::
          LaneBoundaryAttribute_Type_LANEBOUNDARY_TYPE_CURB) {
    safe_lane_width -= local_route_config_.smoother_config.curb_shift();
    if (safe_lane_width < kEpislon) {
      ADEBUG << "lane width smaller than adc width and right curb shift";
      effective_width = kEpislon;
      is_lane_width_safe = false;
    } else {
      center_shift += 0.5 * local_route_config_.smoother_config.curb_shift();
    }
  }
  if (hdmap::LeftBoundaryType(waypoint) ==
      hdmap_new::LaneBoundaryAttribute_Type::
          LaneBoundaryAttribute_Type_LANEBOUNDARY_TYPE_CURB) {
    safe_lane_width -= local_route_config_.smoother_config.curb_shift();
    if (safe_lane_width < kEpislon) {
      ADEBUG << "lane width smaller than adc width and left curb shift";
      effective_width = kEpislon;
      is_lane_width_safe = false;
    } else {
      center_shift -= 0.5 * local_route_config_.smoother_config.curb_shift();
    }
  }

  //  apply buffer if possible
  const double buffered_width =
      safe_lane_width -
      2.0 * local_route_config_.smoother_config.lateral_buffer();
  safe_lane_width =
      buffered_width < kEpislon ? safe_lane_width : buffered_width;

  // shift center depending on the road width
  if (is_lane_width_safe) {
    effective_width = 0.5 * safe_lane_width;
  }

  route_point += left_vec * center_shift;
  anchor.path_point = route_point.ToPathPoint(s);
  anchor.lateral_bound = ::math::Clamp(
      effective_width,
      local_route_config_.smoother_config.min_lateral_boundary_bound(),
      local_route_config_.smoother_config.max_lateral_boundary_bound());

  return anchor;
}

void LocalRouteProvider::GetAnchorPoints(
    const LocalRoute &local_route,
    std::vector<RouteAnchorPoint> &anchor_points) const {
  const double interval = GetSampleInterval();
  int num_of_anchors =
      std::max(2, static_cast<int>(local_route.Length() / interval + 0.5));
  std::vector<double> anchor_s;
  ::util::uniform_slice(0.0, local_route.Length(), num_of_anchors - 1,
                        &anchor_s);
  for (const double s : anchor_s) {
    RouteAnchorPoint anchor = GetAnchorPoint(local_route, s);
    anchor_points.emplace_back(anchor);
  }
  anchor_points.front().longitudinal_bound = kMinBoundValue;
  anchor_points.front().lateral_bound = kMinBoundValue;
  anchor_points.front().enforced = true;
  anchor_points.back().longitudinal_bound = kMinBoundValue;
  anchor_points.back().lateral_bound = kMinBoundValue;
  anchor_points.back().enforced = true;
}

void LocalRouteProvider::ReSampleRefPoints(
    const LocalRoute &local_route, std::vector<LocalRoutePoint> &ref_points) {
  ::common::SLPoint ego_sl;
  float s = 0.0;
  float start_s = 0.0;
  const uint8_t kPointsNum = local_route_config_.fit_points_size;
  float k = local_route.Length() / ((kPointsNum - 1) * (kPointsNum - 1));
  if (local_route.XYToSL(polynomial_fitter_->GetEgoPoint(), ego_sl)) {
    start_s = std::fmax(ego_sl.s(), 0.0);
    k = (local_route.Length() - start_s) /
        ((kPointsNum - 1) * (kPointsNum - 1));
    s = start_s;
  }

  for (uint8_t i = 0; i < kPointsNum; i++) {
    s = start_s + k * i * i;
    ref_points.emplace_back(local_route.GetLocalRoutePoint(s));
  }
}

void LocalRouteProvider::GetSufficientDistPoint(
    const LocalRoutePoint &original_point, const LocalRoutePoint &fix_point,
    const double safe_dist, RouteAnchorPoint &anchor) {
  if (original_point.lane_waypoints().empty()) {
    return;
  }
  double move_dist = 0.0;
  if (SetDifferentBound(original_point.lane_waypoints().front().lane, fix_point,
                        safe_dist, move_dist)) {
    ::math::Vec2d move_point = GetWidthPoint(fix_point, move_dist);
    anchor.path_point.set_x(move_point.x());
    anchor.path_point.set_y(move_point.y());
  }
}

void LocalRouteProvider::GetNOALineAnchorPoints(
    std::pair<FixNoaLineInfo, std::vector<LocalRoutePoint>> &fix_noa_line,
    const LocalRoute &local_route, const RouteSegments &segments,
    const int8_t noaline_dir, std::vector<RouteAnchorPoint> *anchor_points,
    std::vector<LocalRoutePoint> &last_ref_points,
    std::pair<uint32_t, uint32_t> &ref_index_range) {
  CHECK_NOTNULL(anchor_points);
  LocalRoute fix_ref_line;
  bool use_fix = false;
  ::common::SLPoint fix_start_sl, fix_end_sl;
  if (fix_noa_line.second.size() >= 2) {
    fix_ref_line = LocalRoute(fix_noa_line.second, local_route_config_, true);
    if (local_route.XYToSL(fix_noa_line.second.at(0), fix_start_sl) &&
        local_route.XYToSL(fix_noa_line.second.back(), fix_end_sl)) {
      use_fix = true;
    }
  }
  ::common::SLPoint ego_fix_sl;
  bool passed_fix_line = false;
  if (fix_ref_line.XYToSL(polynomial_fitter_->GetEgoPoint(), ego_fix_sl)) {
    if (ego_fix_sl.s() >= fix_ref_line.Length()) {
      passed_fix_line = true;
      AINFO << " pass fix noa line, clear.";
    }
  }

  // noa line also add last route similarity cost
  LocalRoute last_ref_route;
  double forward_ref_s, backward_ref_s;
  bool add_similarity_cost = AddLastSimilarity(local_route, last_ref_route,
                                               forward_ref_s, backward_ref_s);
  const double last_ref_length =
      add_similarity_cost ? last_ref_route.Length() : 0.0;
  uint32_t index = 0;
  uint32_t min_index = 1000;
  uint32_t max_index = 0;

  const double interval = GetSampleInterval();
  int num_of_anchors =
      std::max(2, static_cast<int>(local_route.Length() / interval + 0.5));
  std::vector<double> anchor_s;
  ::util::uniform_slice(0.0, local_route.Length(), num_of_anchors - 1,
                        &anchor_s);
  for (const double s : anchor_s) {
    RouteAnchorPoint anchor = GetAnchorPoint(local_route, s);
    const auto &route_point = local_route.GetLocalRoutePoint(s);
    bool boundary_check = !route_point.lane_waypoints().empty();
    if (use_fix && s >= fix_start_sl.s() && s <= fix_end_sl.s()) {
      auto fix_point = fix_ref_line.GetLocalRoutePoint(s - fix_start_sl.s());
      anchor.path_point.set_x(fix_point.x());
      anchor.path_point.set_y(fix_point.y());
      if (boundary_check) {
        GetSufficientDistPoint(route_point, fix_point, kNOALineSafeDist,
                               anchor);
      }
    } else if (boundary_check) {
      GetSufficientDistPoint(route_point, route_point, kNOALineSafeDist,
                             anchor);
    }
    if (add_similarity_cost) {
      ::common::SLPoint last_ref_sl;
      if (last_ref_route.XYToSL(route_point, last_ref_sl) &&
          last_ref_sl.s() >= 0.0 && last_ref_sl.s() <= last_ref_length &&
          last_ref_sl.s() >= backward_ref_s &&
          last_ref_sl.s() <= forward_ref_s) {
        last_ref_points.emplace_back(
            std::move(last_ref_route.GetLocalRoutePoint(last_ref_sl.s())));
        if (index < min_index) {
          min_index = index;
        }
        if (index > max_index) {
          max_index = index;
        }
      } else {
        last_ref_points.emplace_back(route_point);
      }
    }
    index++;
    anchor_points->emplace_back(anchor);
  }
  ref_index_range.first = min_index;
  ref_index_range.second = max_index;

  anchor_points->front().longitudinal_bound = kMinBoundValue;
  anchor_points->front().lateral_bound = kMinBoundValue;
  anchor_points->front().enforced = true;
  anchor_points->back().longitudinal_bound = kMinBoundValue;
  anchor_points->back().lateral_bound = kMinBoundValue;
  anchor_points->back().enforced = true;

  if (passed_fix_line) {
    fix_noa_line.first.ClearFixInfo();
    fix_noa_line.second.clear();
  }
}

bool LocalRouteProvider::AddLastSimilarity(const LocalRoute &local_route,
                                           LocalRoute &last_ref_route,
                                           double &forward_ref_s,
                                           double &backward_ref_s) {
  bool res = false;
  ::common::SLPoint ego_last_ref_sl;
  if (GetLastRefRoute(local_route.Lanes(), last_ref_route) &&
      last_ref_route.RoutePoints().size() > 0 &&
      last_ref_route.XYToSL(polynomial_fitter_->GetEgoPoint(),
                            ego_last_ref_sl)) {
    res = true;
  }
  if (res) {
    const double kForwardRefDist = 50.0;
    const double kBackwardRefDist = 20.0;
    forward_ref_s = ego_last_ref_sl.s() + kForwardRefDist;
    backward_ref_s = std::fmax(0.0, ego_last_ref_sl.s() - kBackwardRefDist);
  }
  return res;
}

bool LocalRouteProvider::SmoothRouteSegment(const RouteSegments &segments,
                                            LocalRoute &local_route,
                                            const bool &need_smooth) {
  if (!is_olm_) {
    if (segments.size() == 1) {
      const float kValidSegRemainDist = 3.0;
      const auto seg = segments.begin();
      double ego_s = 0.0;
      double ego_l = 0.0;
      const auto vehicle_state = polynomial_fitter_->GetEgoPoint();
      if (!seg->lane ||
          !seg->lane->GetProjection(vehicle_state, &ego_s, &ego_l)) {
        return false;
      }
      if (ego_s + kValidSegRemainDist >= seg->lane->total_length()) {
        AERROR << "current seg remain invlaid, ego_s:  " << ego_s
               << " length: " << seg->lane->total_length()
               << "  seg end_s: " << seg->end_s;
        return false;
      }
    }
    LocalRoute raw_local_route(segments, local_route_config_);
    const NoaLine noaline_info = pnc_map_->GetNavInfo()->noa_line;
    const hdmap::RouteSegments::SegmentType seg_type =
        segments.GetSegmentType();
    if ((seg_type == hdmap::RouteSegments::SegmentType::CurrentSegment &&
         noaline_info.is_trigger) &&
        CalFixNOALine(raw_local_route, segments, noaline_info.direction,
                      fix_noa_line_, &local_route)) {
      if (!history_noa_lines_.count(fix_noa_line_.first.start_lane_id) &&
          fix_noa_line_.first.valid) {
        history_noa_lines_.emplace(std::make_pair<>(
            fix_noa_line_.first.start_lane_id, fix_noa_line_.first));
      }
      return true;
    } else {
      LocalRoute fix_local_route;
      if (seg_type == hdmap::RouteSegments::SegmentType::CurrentSegment &&
          FollowNOALine(history_noa_lines_, fix_local_route)) {
        return SmoothCurrentLocalRoute(raw_local_route, fix_local_route,
                                       local_route);
      } else {
        return SmoothLocalRoute(raw_local_route, local_route, need_smooth);
      }
    }
  } else {
    LocalRoute raw_local_route(segments, local_route_config_);
    return SmoothLocalRoute(raw_local_route, local_route, need_smooth);
  }
}

bool LocalRouteProvider::FollowNOALine(
    std::unordered_map<std::string, FixNoaLineInfo> &noalines,
    LocalRoute &follow_local_route) {
  if (noalines.size() == 0) {
    return false;
  }
  std::vector<LocalRoute> fix_local_routes;
  std::vector<LocalRoutePoint> fix_points;
  std::vector<std::string> passeed_routes;
  for (const auto &noaline : noalines) {
    hdmap_new::Id start_id, end_id;
    start_id.set_id(noaline.second.start_lane_id);
    end_id.set_id(noaline.second.final_lane_id);
    auto start_lane_info =
        hdmap::GetLaneById(hdmap::HDMapUtil::BaseMapPtr(), start_id);
    auto end_lane_info =
        hdmap::GetLaneById(hdmap::HDMapUtil::BaseMapPtr(), end_id);
    if (start_lane_info && end_lane_info) {
      ::math::Vec2d s_p(
          start_lane_info->lane().center_line().point().begin()->x(),
          start_lane_info->lane().center_line().point().begin()->y());
      auto t_p = end_lane_info->GetSmoothPoint(noaline.second.final_s);
      LocalRoutePoint start_p = LocalRoutePoint(
          std::move(hdmap::MapPathPoint(s_p, noaline.second.start_heading)),
          0.0, 0.0);
      LocalRoutePoint final_p = LocalRoutePoint(
          std::move(hdmap::MapPathPoint(t_p, noaline.second.final_heading)),
          0.0, 0.0);
      polynomial_fitter_->SolvePolynomialCoefficients(
          noaline.second.t, noaline.second.fix_vel, start_p, final_p,
          fix_points);
      fix_local_routes.emplace_back(
          std::move(LocalRoute(fix_points, local_route_config_)));
    } else {
      passeed_routes.emplace_back(noaline.second.start_lane_id);
    }
  }
  bool keep_noa_line = false;
  ::math::Vec2d ego_point = polynomial_fitter_->GetEgoPoint();
  ::common::SLPoint fix_ego_sl;
  for (const LocalRoute &fix_local_route : fix_local_routes) {
    if (fix_local_route.XYToSL(ego_point, fix_ego_sl)) {
      if (fix_ego_sl.s() <= fix_local_route.Length() && fix_ego_sl.s() > 0.0) {
        keep_noa_line = true;
        follow_local_route = fix_local_route;
        break;
      }
    }
  }
  if (!keep_noa_line) {
    noalines.clear();
  } else {
    for (const std::string &passed_id : passeed_routes) {
      if (noalines.count(passed_id)) {
        noalines.erase(passed_id);
      }
    }
  }
  AINFO << "keep follow noaline:" << keep_noa_line;
  return keep_noa_line;
}

bool LocalRouteProvider::SmoothCurrentLocalRoute(
    const LocalRoute &raw_local_route, const LocalRoute &fix_noaline,
    LocalRoute &local_route) {
  local_route.SetLanes(raw_local_route.Lanes());
  std::vector<RouteAnchorPoint> anchor_points;
  double s_diff = 0.0;
  ::math::Vec2d ego_point = polynomial_fitter_->GetEgoPoint();
  ::common::SLPoint fix_ego_sl, cur_route_sl;
  if (!raw_local_route.XYToSL(ego_point, cur_route_sl) ||
      !fix_noaline.XYToSL(ego_point, fix_ego_sl)) {
    return false;
  } else {
    s_diff = cur_route_sl.s() - fix_ego_sl.s();
  }

  // last similarity cost.
  LocalRoute last_ref_route;
  double forward_ref_s, backward_ref_s;
  bool add_similarity_cost = AddLastSimilarity(local_route, last_ref_route,
                                               forward_ref_s, backward_ref_s);
  const double last_ref_length =
      add_similarity_cost ? last_ref_route.Length() : 0.0;
  std::vector<LocalRoutePoint> last_ref_points;
  std::pair<uint32_t, uint32_t> ref_index_range;
  uint32_t index = 0;
  uint32_t min_index = 1000;
  uint32_t max_index = 0;

  const double interval = GetSampleInterval();
  const int kMinPointsNum = 2;
  int num_of_anchors =
      std::max(kMinPointsNum,
               static_cast<int>(raw_local_route.Length() / interval + 0.5));
  std::vector<double> anchor_s;
  ::util::uniform_slice(0.0, raw_local_route.Length(), num_of_anchors - 1,
                        &anchor_s);
  RouteAnchorPoint anchor;
  anchor.lateral_bound =
      local_route_config_.smoother_config.max_lateral_boundary_bound();
  anchor.longitudinal_bound =
      local_route_config_.smoother_config.longitudinal_boundary_bound();
  const double fix_length = fix_noaline.Length();
  for (const double s : anchor_s) {
    const auto &route_point = raw_local_route.GetLocalRoutePoint(s);
    anchor.path_point.set_x(route_point.x());
    anchor.path_point.set_y(route_point.y());
    bool boundary_check = !route_point.lane_waypoints().empty();
    const double fix_s = s - s_diff;
    if (fix_s > 0.0 && fix_s <= fix_length) {
      const auto &fix_point = fix_noaline.GetLocalRoutePoint(fix_s);
      anchor.path_point.set_x(fix_point.x());
      anchor.path_point.set_y(fix_point.y());
      if (boundary_check) {
        GetSufficientDistPoint(route_point, fix_point, kNOALineSafeDist,
                               anchor);
      }
    } else if (boundary_check) {
      GetSufficientDistPoint(route_point, route_point, kNOALineSafeDist,
                             anchor);
    }
    if (add_similarity_cost) {
      ::common::SLPoint last_ref_sl;
      if (last_ref_route.XYToSL(route_point, last_ref_sl) &&
          last_ref_sl.s() >= 0.0 && last_ref_sl.s() <= last_ref_length &&
          last_ref_sl.s() >= backward_ref_s &&
          last_ref_sl.s() <= forward_ref_s) {
        last_ref_points.emplace_back(
            std::move(last_ref_route.GetLocalRoutePoint(last_ref_sl.s())));
        if (index < min_index) {
          min_index = index;
        }
        if (index > max_index) {
          max_index = index;
        }
      } else {
        last_ref_points.emplace_back(route_point);
      }
    }
    index++;
    anchor_points.emplace_back(anchor);
  }
  if (anchor_points.size() < kMinPointsNum) {
    AERROR << "anchor_points error, size: " << anchor_points.size();
    return false;
  }
  anchor_points.front().longitudinal_bound = kMinBoundValue;
  anchor_points.front().lateral_bound = kMinBoundValue;
  anchor_points.front().enforced = true;
  anchor_points.back().longitudinal_bound = kMinBoundValue;
  anchor_points.back().lateral_bound = kMinBoundValue;
  anchor_points.back().enforced = true;

  smoother_->SetAnchorPoints(anchor_points);
  if (last_ref_points.size() > 0) {
    ref_index_range.first = min_index;
    ref_index_range.second = max_index;
    smoother_->SetLastRefPoints(ref_index_range, last_ref_points);
  }
  if (!smoother_->Smooth(raw_local_route, &local_route)) {
    AERROR << "Failed to smooth local route with anchor points";
    return false;
  }
  return true;
}

bool LocalRouteProvider::SetDifferentBound(
    const map_service::zmap::LaneInfoConstPtr &lane_info,
    const ::math::Vec2d &route_point, const double safe_dist,
    double &move_dist) {
  if (!lane_info) {
    return false;
  }
  const NavigationInfo *navi_info = pnc_map_->GetNavInfo();
  if (!navi_info) {
    return false;
  }
  const std::string target_section_id =
      lane_info->lane().road_section_id().id();
  const auto it_section =
      std::find_if(navi_info->sections.begin(), navi_info->sections.end(),
                   [target_section_id](const NavSection &section) {
                     return section.section_id == target_section_id;
                   });
  uint8_t lane_sequence = 0;
  uint8_t lane_size = 0;
  if (it_section == navi_info->sections.end()) {
    return false;
  }
  const std::string target_lane_id = lane_info->lane().id().id();
  const auto navi_lane = it_section->lanes.find(target_lane_id);
  if (navi_lane == it_section->lanes.end()) {
    return false;
  }
  lane_sequence = navi_lane->second.lane_sequence;
  lane_size = it_section->lanes.size();

  bool move_to_left = false;
  bool move_to_right = false;
  if (lane_size == 1) {
    if (lane_info->lane().right_neighbor_forward_lane_id_size() == 0 &&
        lane_info->lane().left_neighbor_forward_lane_id_size() > 0) {
      move_to_left = true;
    } else if (lane_info->lane().left_neighbor_forward_lane_id_size() == 0 &&
               lane_info->lane().right_neighbor_forward_lane_id_size() > 0) {
      move_to_right = true;
    }
  }
  ADEBUG << "MEOVE LEFT: " << int(move_to_left)
         << "  to right: " << int(move_to_right)
         << "  tar id: " << target_lane_id
         << "  lane seq: " << int(lane_sequence)
         << "  lane size: " << int(lane_size);
  const double kMinValidMoveDist = 0.05;
  const double half_ego_width = local_route_config_.adc_width / 2.0;
  if ((lane_sequence == 1 && lane_size > 1) || move_to_left) {
    // in right most lane, right bound need faraway lane right_line;
    auto right_boundary_info = hdmap::GetLaneBoundaryById(
        hdmap::HDMapUtil::BaseMapPtr(), lane_info->lane().right_boundary_id());
    double proj_s, proj_l;
    if (right_boundary_info &&
        right_boundary_info->GetProjection(route_point, &proj_s, &proj_l) &&
        proj_l > 0.0) {
      const double bound_remain_dist = proj_l - (half_ego_width + safe_dist);
      if (bound_remain_dist >= 0.0) {
        return false;
      } else {
        move_dist = fabs(bound_remain_dist);
        ADEBUG << "move dist : " << bound_remain_dist;
        return move_dist >= kMinValidMoveDist;
      }
    }
  } else if ((lane_sequence == lane_size && lane_size > 1) || move_to_right) {
    // in left most lane, left bound need faraway lane left_line;
    auto left_boundary_info = hdmap::GetLaneBoundaryById(
        hdmap::HDMapUtil::BaseMapPtr(), lane_info->lane().left_boundary_id());
    double proj_s, proj_l;
    if (left_boundary_info &&
        left_boundary_info->GetProjection(route_point, &proj_s, &proj_l) &&
        proj_l < 0.0) {
      const double bound_remain_dist = proj_l + (half_ego_width + safe_dist);
      if (bound_remain_dist <= 0.0) {
        return false;
      } else {
        move_dist = -bound_remain_dist;
        ADEBUG << "move dist : " << bound_remain_dist;
        return fabs(move_dist) >= kMinValidMoveDist;
      }
    }
  }
  return false;
}

bool LocalRouteProvider::SmoothPrefixedLocalRoute(const LocalRoute &prefix_ref,
                                                  const LocalRoute &raw_ref,
                                                  LocalRoute &local_route) {
  if (!local_route_config_.enable_smooth_local_route) {
    local_route = raw_ref;
    return true;
  }
  // generate anchor points:
  std::vector<RouteAnchorPoint> anchor_points;
  GetAnchorPoints(raw_ref, anchor_points);
  // modify anchor points based on prefix_ref
  for (auto &point : anchor_points) {
    ::common::SLPoint sl_point;
    if (!prefix_ref.XYToSL(Vec2d(point.path_point.x(), point.path_point.y()),
                           sl_point)) {
      continue;
    }
    if (sl_point.s() < 0 || sl_point.s() > prefix_ref.Length()) {
      continue;
    }
    auto prefix_ref_point = prefix_ref.GetNearestLocalRoutePoint(sl_point.s());
    point.path_point.set_x(prefix_ref_point.x());
    point.path_point.set_y(prefix_ref_point.y());
    point.path_point.set_z(0.0);
    point.path_point.set_theta(prefix_ref_point.heading());
    point.longitudinal_bound = kMinBoundValue;
    point.lateral_bound = kMinBoundValue;
    point.enforced = true;
    break;
  }

  smoother_->SetAnchorPoints(anchor_points);
  if (!smoother_->Smooth(raw_ref, &local_route)) {
    AERROR << "Failed to smooth prefixed local route with anchor points";
    return false;
  }
  if (!IsLocalRouteSmoothValid(raw_ref, local_route)) {
    AERROR << "The smoothed local route error is too large";
    return false;
  }
  return true;
}

bool LocalRouteProvider::GetBoundaryDistance(const LocalRoutePoint &ref_point,
                                             double &left_l, double &right_l) {
  bool res = false;
  if (ref_point.lane_waypoints().size() == 0) {
    return false;
  }
  const auto &select_lane_info = ref_point.lane_waypoints().begin()->lane;
  auto left_bound =
      hdmap::GetLaneBoundaryById(hdmap::HDMapUtil::BaseMapPtr(),
                                 select_lane_info->lane().left_boundary_id());
  auto right_bound =
      hdmap::GetLaneBoundaryById(hdmap::HDMapUtil::BaseMapPtr(),
                                 select_lane_info->lane().right_boundary_id());
  double s = 0.0;
  if (left_bound && right_bound &&
      left_bound->GetProjection(ref_point, &s, &left_l) &&
      right_bound->GetProjection(ref_point, &s, &right_l)) {
    res = true;
  }
  return res;
}

bool LocalRouteProvider::GetBoundaryDistance(const ::math::Vec2d &center_point,
                                             const std::string &lane_id,
                                             double &left_l, double &right_l) {
  bool res = false;

  std::pair<map_service::zmap::LaneBoundaryInfoConstPtr,
            map_service::zmap::LaneBoundaryInfoConstPtr>
      boundary_infos;

  if (!GetBoundaryInfo(lane_id, boundary_infos)) {
    return false;
  }
  double s = 0.0;
  if (boundary_infos.first->GetProjection(center_point, &s, &left_l) &&
      boundary_infos.second->GetProjection(center_point, &s, &right_l)) {
    res = true;
  }
  return res;
}

bool LocalRouteProvider::GetBoundaryInfo(
    const std::string &lane_id,
    std::pair<map_service::zmap::LaneBoundaryInfoConstPtr,
              map_service::zmap::LaneBoundaryInfoConstPtr> &boundary_infos) {
  hdmap_new::Id id;
  id.set_id(lane_id);
  const auto select_lane_info =
      hdmap::GetLaneById(hdmap::HDMapUtil::BaseMapPtr(), id);
  if (!select_lane_info) {
    return false;
  }
  auto left_bound =
      hdmap::GetLaneBoundaryById(hdmap::HDMapUtil::BaseMapPtr(),
                                 select_lane_info->lane().left_boundary_id());
  auto right_bound =
      hdmap::GetLaneBoundaryById(hdmap::HDMapUtil::BaseMapPtr(),
                                 select_lane_info->lane().right_boundary_id());
  if (left_bound && right_bound) {
    boundary_infos.first = left_bound;
    boundary_infos.second = right_bound;
    return true;
  }
  return false;
}

bool LocalRouteProvider::GetBoundaryDistance(
    const ::math::Vec2d &center_point,
    const std::pair<map_service::zmap::LaneBoundaryInfoConstPtr,
                    map_service::zmap::LaneBoundaryInfoConstPtr>
        &boundary_infos,
    double &left_l, double &right_l) {
  double s = 0.0;
  if (boundary_infos.first->GetProjection(center_point, &s, &left_l) &&
      boundary_infos.second->GetProjection(center_point, &s, &right_l)) {
    return true;
  }
  return false;
}

bool LocalRouteProvider::CalFixNOALine(
    const LocalRoute &raw_reference_line, const RouteSegments &segments,
    const int8_t noaline_dir,
    std::pair<FixNoaLineInfo, std::vector<LocalRoutePoint>> &fix_noa_line,
    LocalRoute *local_route) {
  const double kDistTolerance = 5.0;     // [m]
  const double kMinSearchRemain = 75.0;  // [m]
  const double kCrossTime = 2.5;         // [s]
  if (!fix_noa_line.first.valid && fix_noa_line.second.size() == 0) {
    auto ego_state = polynomial_fitter_->GetVehicleState();
    const std::string current_lane_id = pnc_map_->GetNavInfo()->current_lane_id;
    LocalRoutePoint start_point, end_point;
    // find start point
    double lane_speed = 0.0;
    map_service::zmap::LaneInfoConstPtr start_point_lane = nullptr;
    bool find_start = false;
    bool confirm_current = false;
    for (auto seg = segments.begin(); seg != segments.end(); seg++) {
      if (!confirm_current && seg->lane->lane().id().id() == current_lane_id) {
        confirm_current = true;
      }
      if (!confirm_current) continue;
      if (seg->lane->lane().successor_id_size() > 1 &&
          seg->lane->lane().center_line().point().size() > 0) {
        const auto start_map_point =
            seg->lane->lane().center_line().point().rbegin();
        ::common::SLPoint start_sl;
        if (raw_reference_line.XYToSL(*start_map_point, start_sl)) {
          auto last_ref_point =
              raw_reference_line.GetLocalRoutePoint(start_sl.s() - 1.0);
          auto last_point = std::next(start_map_point);
          double heading = std::atan2(start_map_point->y() - last_point->y(),
                                      start_map_point->x() - last_point->x());
          start_point = LocalRoutePoint(
              std::move(hdmap::MapPathPoint(
                  std::move(::math::Vec2d(start_map_point->x(),
                                          start_map_point->y())),
                  heading)),
              0.0, 0.0);
          lane_speed = seg->lane->lane().max_speed_limit_kmph() / 3.6;
          AINFO << "start popin x: " << start_point.x()
                << "  y: " << start_point.y()
                << "  heaidng: " << start_point.heading()
                << "  ego head: " << ego_state.heading();
          auto next_seg = std::next(seg);
          if (next_seg != segments.end()) {
            start_point_lane = next_seg->lane;
            find_start = true;
            fix_noa_line.first.start_heading = heading;
            fix_noa_line.first.start_lane_id = next_seg->lane->lane().id().id();
          }
        }
        break;
      }
    }
    bool remain_sufficient = true;
    double lane_remain_dist = 0.0;
    std::string first_split_lane;
    if (start_point_lane) {
      const auto &navi_sections = pnc_map_->GetNavInfo()->sections;
      const auto noa_line_dir = pnc_map_->GetNavInfo()->noa_line.direction;
      first_split_lane = start_point_lane->lane().id().id();
      for (const auto &section : navi_sections) {
        const auto navi_lane = section.lanes.find(first_split_lane);
        if (navi_lane != section.lanes.end()) {
          for (const auto &neighbor_lane : section.lanes) {
            if ((neighbor_lane.second.lane_sequence ==
                     navi_lane->second.lane_sequence + 1 &&
                 noa_line_dir == -1) ||
                (neighbor_lane.second.lane_sequence ==
                     navi_lane->second.lane_sequence - 1 &&
                 noa_line_dir == 1)) {
              if (neighbor_lane.second.remain_distance < kMinSearchRemain) {
                remain_sufficient = false;
                lane_remain_dist = neighbor_lane.second.remain_distance;
                break;
              }
            }
          }
          break;
        }
      }
    }
    if (find_start) {
      double forward_dist = std::fmin(
          std::fmax(
              std::fmax(ego_state.linear_velocity(), lane_speed) * kCrossTime,
              25.0),
          70.0);
      AINFO << "forward dist: " << forward_dist
            << "  ego v: " << ego_state.linear_velocity()
            << "  lane v: " << lane_speed;
      fix_noa_line.first.fix_vel = ego_state.linear_velocity();
      if (!remain_sufficient) {
        forward_dist = lane_remain_dist * 0.60;
        AINFO << "remain not sufficient, set half remain: " << forward_dist;
      }

      std::string start_id;
      math::Vec2d search_start_point;
      double heading = 0.0;
      bool find_heading = false;
      if (forward_dist <= start_point_lane->total_length() + kDistTolerance) {
        start_id = start_point_lane->lane().id().id();
        search_start_point = start_point_lane->GetSmoothPoint(forward_dist);
        if (!remain_sufficient) {
          if (forward_dist < start_point_lane->total_length()) {
            ::math::Vec2d next_point =
                start_point_lane->GetSmoothPoint(forward_dist + 2.0);
            heading = std::atan2(next_point.y() - search_start_point.y(),
                                 next_point.x() - search_start_point.x());
          } else {
            ::math::Vec2d last_point = start_point_lane->GetSmoothPoint(
                start_point_lane->total_length() - 1.0);
            heading = std::atan2(search_start_point.y() - last_point.y(),
                                 search_start_point.x() - last_point.x());
          }
          find_heading = true;
          fix_noa_line.first.final_s = forward_dist;
          fix_noa_line.first.final_heading = heading;
          fix_noa_line.first.final_lane_id = start_id;
        }
      } else {
        int max_cycle = 5;
        auto confirm_start_lane = start_point_lane;
        double distance_diff = forward_dist - start_point_lane->total_length();
        AINFO << "  distance_diff: " << distance_diff;
        while (max_cycle >= 0 && confirm_start_lane) {
          max_cycle--;
          auto next_lane = pnc_map_->GetRouteSuccessor(
              confirm_start_lane,
              hdmap::RouteSegments::SegmentType::CurrentSegment);
          if (next_lane) {
            if (distance_diff <= next_lane->total_length() + kDistTolerance) {
              start_id = next_lane->lane().id().id();
              search_start_point =
                  next_lane->GetSmoothPoint(fabs(distance_diff));
              AINFO << "new_point X: " << search_start_point.x()
                    << "  Y: " << search_start_point.y()
                    << "  lane: " << start_id;
              if (!remain_sufficient) {
                if (distance_diff < next_lane->total_length()) {
                  ::math::Vec2d next_point =
                      next_lane->GetSmoothPoint(distance_diff + 2.0);
                  heading = std::atan2(next_point.y() - search_start_point.y(),
                                       next_point.x() - search_start_point.x());
                } else {
                  ::math::Vec2d last_point = next_lane->GetSmoothPoint(
                      next_lane->total_length() - 1.0);
                  heading = std::atan2(search_start_point.y() - last_point.y(),
                                       search_start_point.x() - last_point.x());
                }
                find_heading = true;
                fix_noa_line.first.final_s = distance_diff;
                fix_noa_line.first.final_heading = heading;
                fix_noa_line.first.final_lane_id = start_id;
              }
              break;
            } else {
              distance_diff -= next_lane->total_length();
              confirm_start_lane = next_lane;
            }
          } else {
            break;
          }
        }
      }
      if (!remain_sufficient && find_heading) {
        end_point = LocalRoutePoint(
            std::move(hdmap::MapPathPoint(search_start_point, heading)), 0.0,
            0.0);
        AINFO << "!remain_sufficient, end point : " << end_point.x()
              << "  y: " << end_point.y();
        polynomial_fitter_->SolvePolynomialCoefficients(
            kCrossTime - 1.0, ego_state.linear_velocity(), start_point,
            end_point, fix_noa_line.second);
        fix_noa_line.first.valid = true;
      } else if (SearchMeetWidthPoint(start_id, first_split_lane,
                                      search_start_point, fix_noa_line.first,
                                      end_point)) {
        AINFO << "end point : " << end_point.x() << "  y: " << end_point.y();
        polynomial_fitter_->SolvePolynomialCoefficients(
            kCrossTime - 1.0, ego_state.linear_velocity(), start_point,
            end_point, fix_noa_line.second);
        fix_noa_line.first.valid = true;
      }
      const double kTimeBuffer = 1.0;
      fix_noa_line.first.t = kCrossTime - kTimeBuffer;
    }
  } else if (fix_noa_line.first.valid) {
    // find start and end point use fix info;
    hdmap_new::Id start_id, final_id;
    start_id.set_id(fix_noa_line.first.start_lane_id);
    final_id.set_id(fix_noa_line.first.final_lane_id);
    auto start_lane_info =
        hdmap::GetLaneById(hdmap::HDMapUtil::BaseMapPtr(), start_id);
    auto final_lane_info =
        hdmap::GetLaneById(hdmap::HDMapUtil::BaseMapPtr(), final_id);
    if (!start_lane_info || !final_lane_info ||
        start_lane_info->lane().center_line().point().size() < 2 ||
        final_lane_info->lane().center_line().point().size() < 2) {
      fix_noa_line.first.valid = false;
      fix_noa_line.second.clear();
    } else {
      ::math::Vec2d s_p(
          start_lane_info->lane().center_line().point().begin()->x(),
          start_lane_info->lane().center_line().point().begin()->y());
      auto t_p = final_lane_info->GetSmoothPoint(fix_noa_line.first.final_s);
      LocalRoutePoint start_p = LocalRoutePoint(
          std::move(hdmap::MapPathPoint(s_p, fix_noa_line.first.start_heading)),
          0.0, 0.0);
      LocalRoutePoint final_p = LocalRoutePoint(
          std::move(hdmap::MapPathPoint(t_p, fix_noa_line.first.final_heading)),
          0.0, 0.0);
      polynomial_fitter_->SolvePolynomialCoefficients(
          kCrossTime - 1.0, fix_noa_line.first.fix_vel, start_p, final_p,
          fix_noa_line.second);
    }
  }
  std::vector<RouteAnchorPoint> anchor_points;
  std::vector<LocalRoutePoint> last_ref_points;
  std::pair<uint32_t, uint32_t> ref_index_range;
  GetNOALineAnchorPoints(fix_noa_line, raw_reference_line, segments,
                         noaline_dir, &anchor_points, last_ref_points,
                         ref_index_range);
  smoother_->SetAnchorPoints(anchor_points);
  if (last_ref_points.size() > 0) {
    smoother_->SetLastRefPoints(ref_index_range, last_ref_points);
  }
  if (!smoother_->Smooth(raw_reference_line, local_route, false)) {
    AERROR << "Failed to smooth noa line with anchor points";
    return false;
  }
  return true;
}

bool LocalRouteProvider::SearchMeetWidthPoint(const std::string &start_lane_id,
                                              const std::string &first_split_id,
                                              const ::math::Vec2d &start_point,
                                              FixNoaLineInfo &fix_info,
                                              LocalRoutePoint &target_point) {
  double map_s, map_l;
  std::pair<map_service::zmap::LaneBoundaryInfoConstPtr,
            map_service::zmap::LaneBoundaryInfoConstPtr>
      boundary_infos;
  ::math::Vec2d last_confirm_point;
  bool find_target = false;
  int max_cycle = 10;
  hdmap_new::Id map_id;
  map_id.set_id(start_lane_id);
  const auto select_lane_info =
      hdmap::GetLaneById(hdmap::HDMapUtil::BaseMapPtr(), map_id);
  if (!select_lane_info) {
    return false;
  }
  const double kForwardSearchDist = 15.0;
  auto search_lane_info = select_lane_info;
  auto last_lane_info = select_lane_info;
  auto search_start_point = start_point;
  bool find_final_dist = false;
  ::math::Vec2d confirm_point;
  double next_distance_diff = 0.0;
  double heading = 0.0;
  double accumulate_s = 0;
  while (max_cycle >= 0 && search_lane_info) {
    max_cycle--;
    std::string search_id = search_lane_info->lane().id().id();
    const double lane_length = search_lane_info->total_length();
    if (find_target) {
      ADEBUG << "next_distance_diff : " << next_distance_diff
             << "  lane_length: " << lane_length;
      if (next_distance_diff <= lane_length + 1.0) {
        last_confirm_point =
            search_lane_info->GetSmoothPoint(next_distance_diff);
        auto prev_point =
            search_lane_info->GetSmoothPoint(next_distance_diff - 0.5);
        heading = std::atan2(last_confirm_point.y() - prev_point.y(),
                             last_confirm_point.x() - prev_point.x());
        find_final_dist = true;
        fix_info.final_lane_id = search_id;
        fix_info.final_s = next_distance_diff;
        fix_info.final_heading = heading;
        break;
      } else {
        next_distance_diff -= lane_length;
      }
    } else if (GetBoundaryInfo(search_id, boundary_infos)) {
      if (search_lane_info->GetProjection(search_start_point, &map_s, &map_l)) {
        double boudray_first_distance = 100.0;
        bool is_continue_split = false;
        if (search_id != first_split_id &&
            boundary_infos.first->lane_boundary().curve().point().size() > 0 &&
            boundary_infos.second->lane_boundary().curve().point().size() > 0) {
          const auto left_first_point =
              boundary_infos.first->lane_boundary().curve().point().begin();
          const auto right_first_point =
              boundary_infos.second->lane_boundary().curve().point().begin();
          boudray_first_distance =
              ::math::Vec2d(left_first_point->x(), left_first_point->y())
                  .DistanceTo(::math::Vec2d(right_first_point->x(),
                                            right_first_point->y()));
          is_continue_split = boudray_first_distance <= 0.10 ? true : false;
        }
        for (double search_s = map_s; search_s <= lane_length;
             search_s += 2.0) {
          auto need_point = search_lane_info->GetSmoothPoint(search_s);
          double left_bound_l, right_bound_l;
          if (GetBoundaryDistance(need_point, boundary_infos, left_bound_l,
                                  right_bound_l)) {
            last_confirm_point = need_point;
            const double real_lane_widht = fabs(right_bound_l - left_bound_l);
            if ((fabs(fabs(right_bound_l) - fabs(left_bound_l)) <= 0.10 &&
                 real_lane_widht >= 2.5) ||
                (real_lane_widht >= 3.2 && accumulate_s > 25.0) ||
                is_continue_split) {
              // mark this point and continue forward serach 15m;
              find_target = true;
              AINFO << "GET MEET point x: " << last_confirm_point.x()
                    << " y: " << last_confirm_point.y()
                    << "  search_s: " << search_s
                    << "  lane lenght: " << lane_length
                    << "  left l: " << left_bound_l
                    << "  right l: " << right_bound_l
                    << "  is_continue_split: " << is_continue_split;
              fix_info.is_continue_split = is_continue_split;
              double final_distance = search_s + kForwardSearchDist;
              if (final_distance <= lane_length + 1.0) {
                last_confirm_point =
                    search_lane_info->GetSmoothPoint(final_distance);
                ::math::Vec2d prev_point;
                if (final_distance > 1.0) {
                  prev_point =
                      search_lane_info->GetSmoothPoint(final_distance - 1.5);
                } else {
                  prev_point = last_lane_info->GetSmoothPoint(
                      last_lane_info->total_length() - 1.5);
                }
                heading = std::atan2(last_confirm_point.y() - prev_point.y(),
                                     last_confirm_point.x() - prev_point.x());
                find_final_dist = true;
                fix_info.final_lane_id = search_id;
                fix_info.final_s = final_distance;
                fix_info.final_heading = heading;
              } else {
                next_distance_diff = final_distance - lane_length;
              }
              break;
            }
          } else {
            AERROR << "get boundary distance failed.";
            break;
          }
          accumulate_s += 2.0;
        }
      } else {
        break;
      }
    } else {
      break;
    }
    if (find_final_dist) {
      break;
    }
    search_start_point = last_confirm_point;
    last_lane_info = search_lane_info;
    search_lane_info = pnc_map_->GetRouteSuccessor(
        search_lane_info, hdmap::RouteSegments::SegmentType::CurrentSegment);
  }
  if (find_final_dist) {
    target_point = LocalRoutePoint(
        std::move(hdmap::MapPathPoint(last_confirm_point, heading)), 0.0, 0.0);
  }
  return find_final_dist;
}

bool LocalRouteProvider::SmoothLocalRoute(const LocalRoute &raw_local_route,
                                          LocalRoute &local_route,
                                          const bool &need_smooth) {
  local_route.SetLanes(raw_local_route.Lanes());
  auto segment_type = raw_local_route.Lanes().GetSegmentType();
  if (!local_route_config_.enable_smooth_local_route || !need_smooth ||
      segment_type != hdmap::RouteSegments::SegmentType::CurrentSegment) {
    local_route = raw_local_route;
    return true;
  }
  // generate anchor points:
  std::vector<RouteAnchorPoint> anchor_points;
  std::vector<LocalRoutePoint> last_ref_points;
  LocalRoute last_ref_route;
  std::pair<uint32_t, uint32_t> ref_index_range;
  if (GetLastRefRoute(raw_local_route.Lanes(), last_ref_route)) {
    GetAnchorPointsWithLastRef(raw_local_route, last_ref_route, anchor_points,
                               last_ref_points, ref_index_range);
  } else {
    GetAnchorPoints(raw_local_route, anchor_points);
  }
  const uint8_t kMinPointNum = 2;
  if (anchor_points.size() < kMinPointNum) {
    AERROR << "anchor_points error, size: " << anchor_points.size();
    return false;
  }
  smoother_->SetAnchorPoints(anchor_points);
  if (last_ref_points.size() >= kMinPointNum) {
    smoother_->SetLastRefPoints(ref_index_range, last_ref_points);
  }
  if (!smoother_->Smooth(raw_local_route, &local_route)) {
    AERROR << "Failed to smooth local route with anchor points";
    return false;
  }
  if (!IsLocalRouteSmoothValid(raw_local_route, local_route)) {
    AERROR << "The smoothed local route error is too large";
    return false;
  }
  return true;
}

void LocalRouteProvider::GetAnchorPointsWithLastRef(
    const LocalRoute &local_route, const LocalRoute &last_ref_route,
    std::vector<RouteAnchorPoint> &anchor_points,
    std::vector<LocalRoutePoint> &last_ref_points,
    std::pair<uint32_t, uint32_t> &ref_index_range) {
  const double interval = GetSampleInterval();
  int num_of_anchors =
      std::max(2, static_cast<int>(local_route.Length() / interval + 0.5));
  std::vector<double> anchor_s;
  ::util::uniform_slice(0.0, local_route.Length(), num_of_anchors - 1,
                        &anchor_s);
  const double last_ref_length = last_ref_route.Length();
  ::common::SLPoint ego_last_ref_sl;
  last_ref_route.XYToSL(polynomial_fitter_->GetEgoPoint(), ego_last_ref_sl);
  const double kForwardRefDist = 50.0;
  const double kBackwardRefDist = 20.0;
  double forward_ref_s = ego_last_ref_sl.s() + kForwardRefDist;
  double backward_ref_s =
      std::fmax(0.0, ego_last_ref_sl.s() - kBackwardRefDist);
  uint32_t index = 0;
  uint32_t min_index = 1000;
  uint32_t max_index = 0;
  bool is_current = local_route.Lanes().GetSegmentType() ==
                    hdmap::RouteSegments::SegmentType::CurrentSegment;
  for (const double s : anchor_s) {
    RouteAnchorPoint anchor;
    anchor.longitudinal_bound =
        local_route_config_.smoother_config.longitudinal_boundary_bound();
    anchor.lateral_bound =
        local_route_config_.smoother_config.max_lateral_boundary_bound();
    const auto &route_point = local_route.GetLocalRoutePoint(s);
    bool boundary_check = is_current && !route_point.lane_waypoints().empty();
    anchor.path_point = route_point.ToPathPoint(s);
    ::common::SLPoint last_ref_sl;
    if (last_ref_route.XYToSL(route_point, last_ref_sl) &&
        last_ref_sl.s() >= 0.0 && last_ref_sl.s() <= last_ref_length &&
        last_ref_sl.s() >= backward_ref_s && last_ref_sl.s() <= forward_ref_s) {
      last_ref_points.emplace_back(
          std::move(last_ref_route.GetLocalRoutePoint(last_ref_sl.s())));
      if (index < min_index) {
        min_index = index;
      }
      if (index > max_index) {
        max_index = index;
      }
    } else {
      last_ref_points.emplace_back(route_point);
    }
    if (boundary_check) {
      GetSufficientDistPoint(route_point, route_point, kHostLineSafeDist,
                             anchor);
    }
    index++;
    anchor_points.emplace_back(anchor);
  }
  ref_index_range.first = min_index;
  ref_index_range.second = max_index;
  anchor_points.front().longitudinal_bound = kMinBoundValue;
  anchor_points.front().lateral_bound = kMinBoundValue;
  anchor_points.front().enforced = true;
  anchor_points.back().longitudinal_bound = kMinBoundValue;
  anchor_points.back().lateral_bound = kMinBoundValue;
  anchor_points.back().enforced = true;
}

bool LocalRouteProvider::SmoothBoundaryLine(const LocalRoute &raw_local_route,
                                            LocalRoute &local_route) {
  std::vector<RouteAnchorPoint> anchor_points;
  GetBoundryAnchorPoints(raw_local_route, &anchor_points);
  const uint8_t kMinPointNum = 2;
  if (anchor_points.size() < kMinPointNum) {
    AERROR << "anchor_points error, size: " << anchor_points.size();
    return false;
  }
  smoother_->SetAnchorPoints(anchor_points);
  smoother_->SetBoundaryFlag(true);
  if (!smoother_->Smooth(raw_local_route, &local_route)) {
    AERROR << "Failed to smooth boundary with anchor points";
    return false;
  }
  return true;
}

void LocalRouteProvider::GetBoundryAnchorPoints(
    const LocalRoute &local_route,
    std::vector<RouteAnchorPoint> *anchor_points) const {
  CHECK_NOTNULL(anchor_points);
  ::common::SLPoint ego_sl;
  float s = 0.0;
  float start_s = 0.0;
  const uint8_t kPointsNum = local_route_config_.fit_points_size;
  // consider interval: (kPointsNum - 1) * (kPointsNum - 1)
  float k = local_route.Length() / ((kPointsNum - 1));
  if (local_route.XYToSL(polynomial_fitter_->GetVehicleState(), ego_sl)) {
    start_s = std::fmax(ego_sl.s() - kBoundaryBackDist, 0.0);
    k = (local_route.Length() - start_s) / ((kPointsNum - 1));
    s = start_s;
  }
  double last_heading = 0.0;
  const double kMinHeadingDiff = 1.5 * M_PI / 180.0;
  bool last_continue = false;
  LocalRoutePoint last_check_point;
  for (uint8_t i = 0; i < kPointsNum; i++) {
    s = start_s + k * i;  // i*i
    const LocalRoutePoint &current_point = local_route.GetLocalRoutePoint(s);
    RouteAnchorPoint anchor;
    anchor.diff_bound = true;
    anchor.lateral_bound = 0.15;
    anchor.lateral_low_bound = 0.1;
    anchor.path_point.set_x(current_point.x());
    anchor.path_point.set_y(current_point.y());
    double cur_heading = current_point.heading();
    if (last_continue) {
      cur_heading = ::math::NormalizeAngle(
          std::atan2(current_point.y() - last_check_point.y(),
                     current_point.x() - last_check_point.x()));
    }
    if (i > 0) {
      if (fabs(cur_heading - last_heading) > kMinHeadingDiff) {
        ADEBUG << "HEADING ERROR, x:  " << current_point.x()
               << "  y: " << current_point.y();
        anchor.need_change_weight = true;
        anchor.lateral_bound = 0.5;
        last_continue = true;
        // continue;
      } else {
        last_continue = false;
      }
    }
    last_heading = cur_heading;
    if (!last_continue) {
      last_check_point = current_point;
    }
    anchor_points->emplace_back(anchor);
  }
}

void LocalRouteProvider::UpdateHostLaneChangeInfo(DEC_Outputs &fitted_result) {
  zark::hdmap_new::Id adc_current_lane_id =
      pnc_map_->GetAdcWaypoint().lane->lane().id();
  auto adc_pre_lane = hdmap::GetLaneById(hdmap::HDMapUtil::BaseMapPtr(),
                                         adc_pre_lane_id_, false);
  if (!adc_pre_lane) {
    AERROR << " get pre lane info failed :" << adc_pre_lane_id_.id();
    adc_pre_lane_id_ = adc_current_lane_id;
    return;
  }
  static int change_count_left = 0;
  static int change_count_right = 0;
  const int lane_keep = 0;
  const int lane_change_left = 1;
  const int lane_change_right = 2;
  const int change_count_limit = 6;
  int change_direction = lane_keep;
  bool is_lane_keep = false;
  for (auto &pre_lane_successor_id : adc_pre_lane->lane().successor_id()) {
    if (pre_lane_successor_id.id() == adc_current_lane_id.id()) {
      is_lane_keep = true;
      break;
    }
  }
  if (!is_lane_keep) {
    for (auto &pre_lane_successor_id : adc_pre_lane->lane().successor_id()) {
      auto adc_pre_lane_successor = hdmap::GetLaneById(
          hdmap::HDMapUtil::BaseMapPtr(), pre_lane_successor_id, false);

      if (!adc_pre_lane_successor) {
        AERROR << " get pre lane successor info failed :"
               << pre_lane_successor_id.id();
        continue;
      }

      for (auto &left_neighbor_lane :
           adc_pre_lane_successor->lane().left_neighbor_forward_lane_id()) {
        if (left_neighbor_lane.id() == adc_current_lane_id.id()) {
          change_direction = lane_change_left;
          break;
        }
      }
      for (auto &right_neighbor_lane :
           adc_pre_lane_successor->lane().right_neighbor_forward_lane_id()) {
        if (right_neighbor_lane.id() == adc_current_lane_id.id()) {
          change_direction = lane_change_right;
          break;
        }
      }
    }
  }

  for (auto &left_neighbor_lane :
       adc_pre_lane->lane().left_neighbor_forward_lane_id()) {
    if (left_neighbor_lane.id() == adc_current_lane_id.id()) {
      change_direction = lane_change_left;
      break;
    }
  }
  for (auto &right_neighbor_lane :
       adc_pre_lane->lane().right_neighbor_forward_lane_id()) {
    if (right_neighbor_lane.id() == adc_current_lane_id.id()) {
      change_direction = lane_change_right;
      break;
    }
  }

  if (change_direction == lane_change_left) {
    change_count_left++;
    change_count_right = 0;
  } else if (change_direction == lane_change_right) {
    change_count_right++;
    change_count_left = 0;
  } else {
    change_count_left = 0;
    change_count_right = 0;
  }

  if (change_count_left > 0 && change_count_left < change_count_limit) {
    fitted_result.mutable_dec_out_lanechangeinfo_bus()
        ->set_dec_out_is_hostlnechgtolft_bl(1);
    AINFO << " set_dec_out_is_hostlnechgtolft_bl ";
    if (change_count_left == change_count_limit - 1) {
      adc_pre_lane_id_ = adc_current_lane_id;
      change_count_left = 0;
    }
    return;
  }

  if (change_count_right > 0 && change_count_right < change_count_limit) {
    fitted_result.mutable_dec_out_lanechangeinfo_bus()
        ->set_dec_out_is_hostlnechgtorgt_bl(1);
    AINFO << " set_dec_out_is_hostlnechgtorgt_bl";
    if (change_count_right == change_count_limit - 1) {
      adc_pre_lane_id_ = adc_current_lane_id;
      change_count_right = 0;
    }
    return;
  }

  if (change_count_left == 0 && change_count_right == 0) {
    adc_pre_lane_id_ = adc_current_lane_id;
  }
  return;
}

void LocalRouteProvider::ExtractLaneCurveNode(
    const NavSection &section_info, const float ego_path_offset,
    std::vector<EMMsg_Curv_Node> &msg_curv_nodes, float &distance_ego,
    std::string &target_id) {
  for (const auto &next_lane : section_info.lanes) {
    if (next_lane.first == target_id) {
      bool meet_max_distance = false;
      hdmap_new::Id next_id;
      next_id.set_id(next_lane.first);
      const auto &next_lane_info =
          hdmap::GetLaneById(hdmap::HDMapUtil::BaseMapPtr(), next_id);
      if (!next_lane_info ||
          next_lane_info->lane().center_line().offset_cm().size() <= 0 ||
          next_lane_info->lane().center_line().offset_cm().size() !=
              next_lane_info->lane().center_line().curve().size()) {
        break;
      }
      int32_t index = 0;
      for (const auto &curve_value :
           next_lane_info->lane().center_line().curve()) {
        const float offset =
            next_lane_info->lane().center_line().offset_cm().at(index) *
            hdmap::kCentimeterToMetre;
        index++;
        if (offset < ego_path_offset) {
          continue;
        }
        distance_ego = offset - ego_path_offset;
        if (msg_curv_nodes.size() > 0 &&
            distance_ego <= msg_curv_nodes.back().dist_curv_node() + 0.5) {
          continue;
        }
        EMMsg_Curv_Node msg_curv;
        msg_curv.set_curvature_val(curve_value);
        msg_curv.set_dist_curv_node(distance_ego);
        msg_curv_nodes.emplace_back(msg_curv);
        if (distance_ego >= kMaxCurvDistance) {
          break;
          meet_max_distance = true;
        }
      }
      if (!meet_max_distance &&
          section_info.topo_range.priority_sucessor_range.size() > 0) {
        const auto priority_top =
            section_info.topo_range.priority_sucessor_range.find(
                next_lane.second.lane_sequence);
        if (priority_top ==
            section_info.topo_range.priority_sucessor_range.end()) {
          target_id =
              section_info.topo_range.priority_sucessor_range.begin()->second;
          break;
        } else {
          target_id = priority_top->second;
        }
      }
      break;
    }
  }
}

void LocalRouteProvider::ExtractLaneCurveNode(
    const zark::map_service::zmap::LaneInfoConstPtr map_lane_info,
    const float ego_path_offset, std::vector<EMMsg_Curv_Node> &msg_curv_nodes,
    float &distance_ego) {
  if (map_lane_info->lane().center_line().offset_cm().size() <= 0 ||
      map_lane_info->lane().center_line().offset_cm().size() !=
          map_lane_info->lane().center_line().curve().size()) {
    return;
  }
  int32_t index = 0;
  for (const auto &curve_value : map_lane_info->lane().center_line().curve()) {
    const float offset =
        map_lane_info->lane().center_line().offset_cm().at(index) *
        hdmap::kCentimeterToMetre;
    index++;
    if (offset < ego_path_offset) {
      continue;
    }
    distance_ego = offset - ego_path_offset;
    if (msg_curv_nodes.size() > 0 &&
        distance_ego <= msg_curv_nodes.back().dist_curv_node() + 0.5) {
      continue;
    }
    EMMsg_Curv_Node msg_curv;
    msg_curv.set_curvature_val(curve_value);
    msg_curv.set_dist_curv_node(distance_ego);
    msg_curv_nodes.emplace_back(msg_curv);
    if (distance_ego >= kMaxCurvDistance) {
      break;
    }
  }
}

void LocalRouteProvider::CurveNodeProcess(const NavigationInfo &nav_info,
                                          const hdmap::RouteSegments &segment,
                                          const double ego_map_s,
                                          DEC_Outputs &decision_result) {
  const double start_t = zark::common::Clock::NowInSeconds();
  auto map_lane_info =
      hdmap::GetLaneById(hdmap::HDMapUtil::BaseMapPtr(),
                         nav_info.current_lane_info.lane_topo.id());
  if (!map_lane_info) return;
  float ego_path_offset = 0.0;
  if (map_lane_info->lane().center_line().offset_cm().size() <= 0 ||
      map_lane_info->lane().center_line().offset_cm().size() !=
          map_lane_info->lane().center_line().curve().size()) {
    return;
  }
  ego_path_offset =
      ego_map_s + map_lane_info->lane().center_line().offset_cm().at(0) *
                      hdmap::kCentimeterToMetre;
  std::vector<EMMsg_Curv_Node> msg_curv_nodes;
  float distance_ego = 0.0;
  std::string refline_end_id;
  bool meet_max_distance = false;
  const double kDistEgoBuffer = 1.0;
  for (const auto &lane_seg : segment) {
    if (!lane_seg.lane) break;
    ExtractLaneCurveNode(lane_seg.lane, ego_path_offset, msg_curv_nodes,
                         distance_ego);
    if (distance_ego + kDistEgoBuffer >= kMaxCurvDistance) {
      meet_max_distance = true;
      break;
    }
    refline_end_id = lane_seg.lane->lane().id().id();
  }
  if (!meet_max_distance) {
    for (auto section_iter = nav_info.sections.begin();
         section_iter != nav_info.sections.end(); section_iter++) {
      auto lane_iter = section_iter->lanes.find(refline_end_id);
      if (lane_iter == section_iter->lanes.end()) continue;
      auto next_section = std::next(section_iter);
      if (next_section == nav_info.sections.end()) {
        break;
      }
      std::string next_lane_id;
      const uint8_t current_seq = nav_info.current_lane_info.lane_sequence;
      const auto priority_top =
          section_iter->topo_range.priority_sucessor_range.find(current_seq);
      if (priority_top !=
          section_iter->topo_range.priority_sucessor_range.end()) {
        next_lane_id = priority_top->second;
      } else if (section_iter->topo_range.priority_sucessor_range.size() > 0) {
        next_lane_id =
            section_iter->topo_range.priority_sucessor_range.begin()->second;
      }
      const auto forward_lane = next_section->lanes.find(next_lane_id);
      if (forward_lane == next_section->lanes.end()) break;
      int8_t kMaxCycle = 20;
      while (kMaxCycle > 0 &&
             distance_ego + kDistEgoBuffer < kMaxCurvDistance) {
        if (next_section == nav_info.sections.end()) {
          break;
        }
        ExtractLaneCurveNode(*next_section, ego_path_offset, msg_curv_nodes,
                             distance_ego, next_lane_id);
        if (distance_ego >= kMaxCurvDistance) {
          break;
        } else {
          next_section = std::next(next_section);
        }
        kMaxCycle--;
      }
      break;
    }
  }
  FillCurvNodes(msg_curv_nodes, decision_result);
  const double end_t = zark::common::Clock::NowInSeconds();
  AINFO << " curvature nodes procss time ms: " << (end_t - start_t) * 1000.0;
}

void LocalRouteProvider::FillCurvNodes(
    const std::vector<EMMsg_Curv_Node> &msg_curv_nodes,
    DEC_Outputs &decision_result) {
  const int8_t kNodeSize = 5;
  const float distance_diff = kMaxCurvDistance / kNodeSize;
  const float mean_dist_range = distance_diff / 2.0;
  const float dist_threshold = 5.0;
  const float mean_dist_threshold = 1.0;
  float dist_value[kNodeSize];
  for (int8_t i = 0; i < kNodeSize; i++) {
    dist_value[i] = (i + 1) * distance_diff;
  }
  const auto comp = [&dist_threshold](const EMMsg_Curv_Node &node,
                                      const float t) {
    return node.dist_curv_node() < t - dist_threshold;
  };
  const auto mean_comp = [](const EMMsg_Curv_Node &node, const float t) {
    return node.dist_curv_node() < t;
  };
  uint32_t valid_nodes = 0;
  auto output_curve = decision_result.mutable_dec_out_curv_path_bus();
  std::vector<EMMsg_Curv_Node> curve;
  if (msg_curv_nodes.size() > 0) {
    for (size_t i = 0; i < kNodeSize; i++) {
      EMMsg_Curv_Node out_node;
      float t = dist_value[i];
      out_node.set_dist_curv_node(t);
      auto ori_iter = std::lower_bound(msg_curv_nodes.begin(),
                                       msg_curv_nodes.end(), t, comp);
      auto index = std::distance(msg_curv_nodes.begin(), ori_iter);
      if (index < static_cast<int>(msg_curv_nodes.size())) {
        float diff = msg_curv_nodes.at(index).dist_curv_node() - t;
        if (std::fabs(diff) > dist_threshold) {
          float low_value = t - mean_dist_range;
          float up_value = t + mean_dist_range;
          auto lowe_iter =
              std::lower_bound(msg_curv_nodes.begin(), msg_curv_nodes.end(),
                               low_value, mean_comp);
          auto up_iter =
              std::lower_bound(msg_curv_nodes.begin(), msg_curv_nodes.end(),
                               up_value, mean_comp);
          auto low_index = std::distance(msg_curv_nodes.begin(), lowe_iter);
          auto up_index = std::distance(msg_curv_nodes.begin(), up_iter);
          float mean_value = 0.0;
          auto mean_index = low_index;
          int32_t num_points = 0;
          while (mean_index < static_cast<int>(msg_curv_nodes.size()) &&
                 mean_index <= up_index) {
            if (mean_index == up_index &&
                fabs(msg_curv_nodes.at(up_index).dist_curv_node() - up_value) >
                    mean_dist_threshold) {
              break;
            }
            mean_value += msg_curv_nodes.at(mean_index).curvature_val();
            mean_index++;
            num_points++;
          }
          if (mean_index <= static_cast<int>(msg_curv_nodes.size()) &&
              up_index == low_index) {
            if (msg_curv_nodes.at(low_index).dist_curv_node() - up_value >
                mean_dist_threshold) {
              num_points = 0;
            }
          }
          if (num_points > 0) {
            mean_value = float(mean_value / num_points);
            out_node.set_curvature_val(mean_value);
            out_node.set_is_valid_node(true);
            valid_nodes++;
          } else {
            out_node.set_curvature_val(0.0);
            out_node.set_is_valid_node(false);
          }
        } else {
          out_node.set_curvature_val(msg_curv_nodes.at(index).curvature_val());
          out_node.set_is_valid_node(true);
          valid_nodes++;
        }
      } else {
        out_node.set_curvature_val(0.0);
        out_node.set_is_valid_node(false);
      }
      curve.emplace_back(out_node);
    }
  } else {
    for (size_t i = 0; i < kNodeSize; i++) {
      EMMsg_Curv_Node out_node;
      out_node.set_is_valid_node(false);
      out_node.set_dist_curv_node(dist_value[i]);
      out_node.set_curvature_val(0.0);
      curve.emplace_back(out_node);
      valid_nodes = 0;
    }
  }
  output_curve->set_size_curv_nodes(valid_nodes);

  for (int i = 0; i < static_cast<int>(curve.size()) && i < kNodeSize; i++) {
    if (i == 0) {
      output_curve->mutable_curv_nodes1_bus()->CopyFrom(curve[i]);
    } else if (i == 1) {
      output_curve->mutable_curv_nodes2_bus()->CopyFrom(curve[i]);
    } else if (i == 2) {
      output_curve->mutable_curv_nodes3_bus()->CopyFrom(curve[i]);
    } else if (i == 3) {
      output_curve->mutable_curv_nodes4_bus()->CopyFrom(curve[i]);
    } else if (i == 4) {
      output_curve->mutable_curv_nodes5_bus()->CopyFrom(curve[i]);
    }
  }
}

bool LocalRouteProvider::PostHeadingProcess(
    const std::vector<LocalRoutePoint> &xy_points,
    std::vector<double> &headings) {
  const uint8_t kMinPointsNum = 2;
  if (xy_points.size() < kMinPointsNum) {
    return false;
  }

  // Get finite difference approximated dx and dy for heading calculation
  headings.clear();
  std::size_t points_size = xy_points.size();
  for (std::size_t i = 0; i < points_size; ++i) {
    double x_delta = 0.0;
    double y_delta = 0.0;
    if (i == 0) {
      x_delta = (xy_points[i + 1].x() - xy_points[i].x());
      y_delta = (xy_points[i + 1].y() - xy_points[i].y());
    } else if (i == points_size - 1) {
      x_delta = (xy_points[i].x() - xy_points[i - 1].x());
      y_delta = (xy_points[i].y() - xy_points[i - 1].y());
    } else {
      x_delta = 0.5 * (xy_points[i + 1].x() - xy_points[i - 1].x());
      y_delta = 0.5 * (xy_points[i + 1].y() - xy_points[i - 1].y());
    }
    headings.emplace_back(std::atan2(y_delta, x_delta));
  }
  return headings.size() == xy_points.size();
}

void LocalRouteProvider::CalRoutePointHeading(
    std::vector<LocalRoutePoint> &xy_points) {
  const uint8_t kMinPointsNum = 2;
  if (xy_points.size() < kMinPointsNum) {
    return;
  }
  std::size_t points_size = xy_points.size();
  for (std::size_t i = 0; i < points_size; ++i) {
    double x_delta = 0.0;
    double y_delta = 0.0;
    if (i == 0) {
      x_delta = (xy_points[i + 1].x() - xy_points[i].x());
      y_delta = (xy_points[i + 1].y() - xy_points[i].y());
    } else if (i == points_size - 1) {
      x_delta = (xy_points[i].x() - xy_points[i - 1].x());
      y_delta = (xy_points[i].y() - xy_points[i - 1].y());
    } else {
      x_delta = 0.5 * (xy_points[i + 1].x() - xy_points[i - 1].x());
      y_delta = 0.5 * (xy_points[i + 1].y() - xy_points[i - 1].y());
    }
    xy_points[i].set_heading(
        ::math::NormalizeAngle(std::atan2(y_delta, x_delta)));
  }
}

const bool LocalRouteProvider::IsLargeCurvatureSample() const {
  if (fitted_line_result_.size() == 0) {
    return false;
  }
  const auto &last_result = fitted_line_result_.rbegin()->second;
  if (!last_result.has_dec_out_curv_path_bus()) {
    return false;
  }
  const float kLargeCurvatureValue = 0.0025;
  if (last_result.dec_out_curv_path_bus().has_curv_nodes1_bus()) {
    if (fabs(last_result.dec_out_curv_path_bus()
                 .curv_nodes1_bus()
                 .curvature_val()) > kLargeCurvatureValue) {
      return true;
    }
  }
  if (last_result.dec_out_curv_path_bus().has_curv_nodes2_bus()) {
    if (fabs(last_result.dec_out_curv_path_bus()
                 .curv_nodes2_bus()
                 .curvature_val()) > kLargeCurvatureValue) {
      return true;
    }
  }
  if (last_result.dec_out_curv_path_bus().has_curv_nodes3_bus()) {
    if (fabs(last_result.dec_out_curv_path_bus()
                 .curv_nodes3_bus()
                 .curvature_val()) > kLargeCurvatureValue) {
      return true;
    }
  }
  if (last_result.dec_out_curv_path_bus().has_curv_nodes4_bus()) {
    if (fabs(last_result.dec_out_curv_path_bus()
                 .curv_nodes4_bus()
                 .curvature_val()) > kLargeCurvatureValue) {
      return true;
    }
  }
  if (last_result.dec_out_curv_path_bus().has_curv_nodes5_bus()) {
    if (fabs(last_result.dec_out_curv_path_bus()
                 .curv_nodes5_bus()
                 .curvature_val()) > kLargeCurvatureValue) {
      return true;
    }
  }
  return false;
}

const double LocalRouteProvider::GetSampleInterval() const {
  const double kSampleDiff = 1.0;
  double interval =
      IsLargeCurvatureSample()
          ? local_route_config_.smoother_config.max_constraint_interval() -
                kSampleDiff
          : local_route_config_.smoother_config.max_constraint_interval();
  interval = std::fmax(interval, kMinSampleDist);
  return interval;
}

}  // namespace planning
}  // namespace zark
