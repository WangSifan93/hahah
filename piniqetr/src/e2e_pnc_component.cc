#include "e2e_pnc_component.h"

#include <signal.h>

#include <boost/stacktrace.hpp>
#include <csignal>
#include <functional>
#include <list>
#include <string>
#include <unordered_set>
#include <vector>

#include "absl/status/status.h"
#include "common/log_data.h"
#include "communication/common/types.h"
#include "dataflow/module/port.h"
#include "dataflow/module_loader/register_module_macro.h"
#include "message/proto/proto_msg.hpp"
#include "message/proto/proto_serializer.hpp"
#include "messages/map_service/zdrive_map.pb.h"
#include "messages/planning/driving/noa_debug_info.pb.h"
#include "plan/planner_defs.h"
#include "zlog.h"

#ifndef GIT_COMMIT_HASH
#define GIT_COMMIT_HASH "unknown"
#endif
#ifndef GIT_COMMIT_TIMESTAMP
#define GIT_COMMIT_TIMESTAMP ""
#endif

constexpr int kMicroToMilli = 1000;

namespace zark {
namespace e2e_noa::debug {
using NOADebugInfoMsg = hobot::message::ProtoMsg<NOADebugInfo>;

using NOADebugInfoMsgSerializer =
    hobot::message::ProtobufSerializer<NOADebugInfo>;
}  // namespace e2e_noa::debug
}  // namespace zark

namespace {
void PostProcessDebugMSG(
    std::shared_ptr<zark::e2e_noa::debug::NOADebugInfoMsg> msg) {
  if (nullptr == msg) {
    return;
  }
  const auto mission_debug = msg->proto.mission_debug_info();
  const std::string selected_id = mission_debug.count("Selected_ID")
                                      ? mission_debug.at("Selected_ID")
                                      : "Error";
  const std::string seq_num =
      mission_debug.count("seq_num") ? mission_debug.at("seq_num") : "ERROR";
  auto all_planning_debug = msg->proto.planning_debug_info_map();
  msg->proto.clear_planning_debug_info_map();
  for (auto &single_debug : all_planning_debug) {
    if (single_debug.first != "Default") {
      const auto &numbers = single_debug.second.numbers();
      std::string key =
          seq_num + "_" +
          (single_debug.second.plan_id() == "" ? "Error"
                                               : single_debug.second.plan_id());
      if (selected_id == single_debug.second.plan_id()) {
        key = seq_num + "_" + "Selected";
        single_debug.second.set_plan_id("Selected");
      }

      (*msg->proto.mutable_planning_debug_info_map())[key] =
          single_debug.second;
    } else {
      (*msg->proto.mutable_planning_debug_info_map())[single_debug.first] =
          single_debug.second;
    }
  }

  if (msg->proto.has_choose_lane_debug_info() &&
      !msg->proto.choose_lane_debug_info().plan_passages().empty()) {
    for (const auto &single_debug :
         msg->proto.choose_lane_debug_info().plan_passages()) {
      if (single_debug.first == selected_id) {
        (*msg->proto.mutable_choose_lane_debug_info()
              ->mutable_plan_passages())[single_debug.first]
            .set_plan_id("Selected");
      }
    }
  }
}
}  // namespace

E2EPncComponent::E2EPncComponent(
    const hobot::dataflow::ModuleOption &module_option)
    : hobot::dataflow::Module(module_option),
      map_msg_("/apps/mapfusion/local_hdmap_data_new#0",
               [](const zark::MapMsgType &msg) {
                 return static_cast<double>(msg.proto.header().timestamp_ns()) *
                        1e-9;
               }),
      predic_objs_msg_(
          "/apps/prediction/prediction_objects#0",
          [](const zark::PredictionObjs &msg) {
            return static_cast<double>(msg.proto.header().timestamp_ns()) *
                   1e-9;
          }),
      map_fusion_msg_(
          "/apps/mapfusion/maplocalization_data#0",
          [](const zark::MapFusionMsgType &msg) {
            return static_cast<double>(msg.proto.header().timestamp_ns()) *
                   1e-9;
          }),
      localization_msg_(
          "/apps/localization/localization_data#0", 100,
          [](const zark::OdometryMsgType &msg) {
            return static_cast<double>(msg.proto.header().timestamp_ns()) *
                   1e-9;
          }),
      veh_status_msg_(
          "/cheryos/vehicle_service/chassis_pt_info#0", 50,
          [](const zark::VehicleStatusType &msg) {
            return static_cast<double>(msg.proto.header().timestamp_ns()) *
                   1e-9;
          }),
      fct_msg_("/apps/plan_control/fct_data#0", 50,
               [](const zark::FctMsgType &msg) {
                 return static_cast<double>(msg.proto.header().timestamp_ns()) *
                        1e-3;
               }) {
  INIT_ZLOG;

  clock_wrapper_ =
      std::make_shared<hobot::time::ClockWrapper>(hobot::time::kSystemTime);
  clock_ = clock_wrapper_->clock();

  map_msg_.RegisterCounter(kPlanningCounter, 10);
  predic_objs_msg_.RegisterCounter(kPlanningCounter, 4);

  localization_msg_.RegisterCounter(kPlanningCounter, 3);
  veh_status_msg_.RegisterCounter(kPlanningCounter, 3);
  fct_msg_.RegisterCounter(kPlanningCounter, 3);

  map_container_.Init();
}

E2EPncComponent::~E2EPncComponent() { DEINIT_ZLOG; }

void E2EPncComponent::InitPortsAndProcs() {
  DF_MODULE_INIT_IDL_INPUT_PORT("PredictionObstacles",
                                zark::prediction::proto::PredictionObjects);
  DF_MODULE_INIT_IDL_INPUT_PORT("localization",
                                zark::localization::LocalizationInfo);
  DF_MODULE_INIT_IDL_INPUT_PORT("local_map", zark::zdrive_map::ZarkMap);
  DF_MODULE_INIT_IDL_INPUT_PORT("map_localization_state",
                                zark::mapfusion::MapLocalizationInfo);
  DF_MODULE_INIT_IDL_INPUT_PORT("vehicle_status",
                                zark::cheryos::VehiclePowerAndChassisMsg);
  DF_MODULE_INIT_IDL_INPUT_PORT("fct_outputs", zark::ads_fct::FCT_Outputs);

  DF_MODULE_INIT_IDL_OUTPUT_PORT("planner_debug_pub",
                                 zark::e2e_noa::debug::NOADebugInfo);
  DF_MODULE_INIT_IDL_OUTPUT_PORT("planner_res_pub",
                                 zark::e2e_noa::PlanningResult);
  DF_MODULE_INIT_IDL_OUTPUT_PORT("planner_heartbeat_pub",
                                 zark::common::HeartBeat);

  DF_MODULE_INIT_IDL_OUTPUT_PORT("planner_debug_output",
                                 zark::prediction::proto::PredictionObjects);

  DF_MODULE_REGISTER_HANDLE_MSGS_PROC(
      "PredictionCallback", E2EPncComponent, PredictionCallback,
      hobot::dataflow::ProcType::DF_MSG_COND_PROC,
      DF_VECTOR("PredictionObstacles"), DF_VECTOR());
  DF_MODULE_REGISTER_HANDLE_MSGS_PROC(
      "LocalizationCallback", E2EPncComponent, LocalizationCallback,
      hobot::dataflow::ProcType::DF_MSG_COND_PROC, DF_VECTOR("localization"),
      DF_VECTOR());
  DF_MODULE_REGISTER_HANDLE_MSGS_PROC(
      "MapCallback", E2EPncComponent, MapCallback,
      hobot::dataflow::ProcType::DF_MSG_COND_PROC, DF_VECTOR("local_map"),
      DF_VECTOR());
  DF_MODULE_REGISTER_HANDLE_MSGS_PROC(
      "MapLocalizationCallback", E2EPncComponent, MapLocalizationCallback,
      hobot::dataflow::ProcType::DF_MSG_COND_PROC,
      DF_VECTOR("map_localization_state"), DF_VECTOR());
  DF_MODULE_REGISTER_HANDLE_MSGS_PROC(
      "VehicleStatusCallback", E2EPncComponent, VehicleStatusCallback,
      hobot::dataflow::ProcType::DF_MSG_COND_PROC, DF_VECTOR("vehicle_status"),
      DF_VECTOR());
  DF_MODULE_REGISTER_HANDLE_MSGS_PROC(
      "FctOutputsCallback", E2EPncComponent, FctOutputsCallback,
      hobot::dataflow::ProcType::DF_MSG_COND_PROC, DF_VECTOR("fct_outputs"),
      DF_VECTOR());

  DF_MODULE_REGISTER_HANDLE_MSGS_PROC(
      "E2EPncProc", E2EPncComponent, E2EPncProc,
      hobot::dataflow::ProcType::DF_MSG_TIMER_PROC, DF_VECTOR(),
      DF_VECTOR("planner_res_pub", "planner_heartbeat_pub"));

  DF_MODULE_REGISTER_HANDLE_MSGS_PROC(
      "E2EPncDebugProc", E2EPncComponent, E2EPncDebugProc,
      hobot::dataflow::ProcType::DF_MSG_TIMER_PROC, DF_VECTOR(),
      DF_VECTOR("planner_debug_pub"));
}

int32_t E2EPncComponent::Init() {
  DFHLOG_I("E2EPnc Init Begin.");

  core_.reset(new ad_e2e::planning::E2EPlanningCore());

  if (!core_->Init()) {
    DFHLOG_E("Planner core init Failed.");
    return false;
  };

  DFHLOG_I("E2EPnc Init Success.");

  return 0;
}

int32_t E2EPncComponent::Start() { return hobot::dataflow::Module::Start(); }

int32_t E2EPncComponent::Stop() { return hobot::dataflow::Module::Stop(); }

void E2EPncComponent::Reset() { hobot::dataflow::Module::Reset(); }

int32_t E2EPncComponent::DeInit() { return hobot::dataflow::Module::DeInit(); }

void E2EPncComponent::PredictionCallback(
    hobot::dataflow::spMsgResourceProc proc,
    const hobot::dataflow::MessageLists &msgs) {
  auto &input_pre_msgs = msgs[proc->GetResultIndex("PredictionObstacles")];
  for (auto &msg : *(input_pre_msgs.get())) {
    auto act_msg = std::dynamic_pointer_cast<
        zark::prediction::proto::PredictionObjectsMsg>(msg);
    if (nullptr == act_msg) {
      continue;
    }
    predic_objs_msg_.OnNewMessage(*act_msg);
  }
}

void E2EPncComponent::LocalizationCallback(
    hobot::dataflow::spMsgResourceProc proc,
    const hobot::dataflow::MessageLists &msgs) {
  auto &input_loc_msgs = msgs[proc->GetResultIndex("localization")];
  if (input_loc_msgs->empty()) {
    return;
  }
  auto act_msg =
      std::dynamic_pointer_cast<zark::localization::LocalizationInfoMsg>(
          input_loc_msgs->back());
  localization_msg_.OnNewMessage(*act_msg);
}

using MapInfo = ad_e2e::planning::MapInfo;
using RoadBoundaryInfo = ad_e2e::planning::RoadBoundaryInfo;
using BoundaryType = ad_e2e::planning::BoundaryType;
using LineType = ad_e2e::planning::LineType;
using LineColor = ad_e2e::planning::LineColor;
using LaneType = ad_e2e::planning::LaneType;
using TurnType = ad_e2e::planning::TurnType;
using StopLineType = ad_e2e::planning::StopLineType;
using LightStatus = ad_e2e::planning::LightStatus;
using NoneOddType = ad_e2e::planning::NoneOddType;
using zMapType = zark::zdrive_map::MapHeader_MapType;
using zLane = zark::zdrive_map::Lane;
using zRoadBoundType = zark::zdrive_map::RoadBoundary::RoadBoundaryType;
using zLaneBoundType = zark::zdrive_map::LaneBoundaryAttribute;
using zColor = zark::zdrive_map::Color;
using zRoadMarkerType = zark::zdrive_map::RoadMarker;
MapInfo E2EPncComponent::CreateMapInfo(const zark::MapMsgType &map_msg,
                                       const std::string &ego_road_section_id,
                                       bool is_on_highway) {
  MapInfo map_info;
  map_info.is_on_highway = is_on_highway;
  const auto &zmap = map_msg.proto;
  const zark::zdrive_map::SectionInfo *navi_start_section = nullptr;

  constexpr double kCentiMeter2Meter = 0.01;
  constexpr int32_t kBackwardKeepDistancecm = 5000;
  std::unordered_set<std::string> route_circle_check;
  route_circle_check.clear();
  map_info.timestamp = static_cast<double>(zmap.header().timestamp_ns()) * 1e-9;
  map_info.seq = zmap.header().sequence_num();
  if (!zmap.guidance().section().empty()) {
    map_info.route.id = zmap.guidance().section().rbegin()->id().id();
  }
  bool is_section_before_ego = true;
  for (const auto &section_env : zmap.guidance().section()) {
    if (section_env.id().id() == ego_road_section_id) {
      navi_start_section = &section_env;
      is_section_before_ego = false;
    }
    if (section_env.distance_info().distance_to_vehicle_cm() >
            -kBackwardKeepDistancecm ||
        !is_section_before_ego) {
      ad_e2e::planning::SectionInfo section_tmp;
      std::size_t last_size = route_circle_check.size();
      route_circle_check.insert(section_env.id().id());
      if (route_circle_check.size() == last_size) {
        LOG(ERROR) << "Navi route circle!";
        break;
      }
      section_tmp.id = section_env.id().id();
      section_tmp.length =
          static_cast<double>(section_env.distance_info().end_offset_cm() -
                              section_env.distance_info().start_offset_cm()) *
          kCentiMeter2Meter;
      if (!section_env.section_lane_infos().empty() &&
          !section_env.section_lane_infos().at(0).lane_ids().empty()) {
        const auto &lane_ids =
            section_env.section_lane_infos().at(0).lane_ids();
        for (auto &id : lane_ids) {
          if (id.id() == "2392467882635270404") continue;
          section_tmp.lane_ids.push_back(id.id());
        }
      }
      map_info.route.sections.emplace_back(std::move(section_tmp));
    }
  }
  if (!zmap.guidance().section().empty() && is_section_before_ego) {
    map_info.seq = -1;
    LOG(ERROR) << "[Map Info][Fatal Error] Can't find ego section in Guidance, "
                  "set invalid map seq!";
    return map_info;
  }

  const auto &road_boundary_segments_env = zmap.map().hdmap().road_boundary();
  for (const auto &road_boundary_env : road_boundary_segments_env) {
    if (road_boundary_env.id().id().empty() ||
        road_boundary_env.curve().point().empty()) {
      continue;
    }
    RoadBoundaryInfo road_boundary_segment_info_tmp;
    road_boundary_segment_info_tmp.id = road_boundary_env.id().id();
    auto &rb_type = road_boundary_segment_info_tmp.boundary_type;
    rb_type = BoundaryType::CURB;
    if (!road_boundary_env.boundary_attr().empty()) {
      switch (road_boundary_env.boundary_attr().at(0).type()) {
        case zRoadBoundType::ROAD_BOUNDARY_KERB:
          rb_type = BoundaryType::CURB;
          break;
        case zRoadBoundType::ROAD_BOUNDARY_FENCE:
          rb_type = BoundaryType::FENCE;
          break;
        case zRoadBoundType::ROAD_BOUNDARY_NATURAL:
          rb_type = BoundaryType::PAVE;
          break;
        case zRoadBoundType::ROAD_BOUNDARY_VIRTUAL:
          rb_type = BoundaryType::VIRTUAL;
          break;
        case zRoadBoundType::ROAD_BOUNDARY_DITCH:
          rb_type = BoundaryType::DITCH;
          break;
        case zRoadBoundType::ROAD_BOUNDARY_OTHER:
        case zRoadBoundType::ROAD_BOUNDARY_UNKNOWN:
          rb_type = BoundaryType::UNKNOWN_BOUNDARY;
          break;
      }
      rb_type = road_boundary_env.virtual_() ? BoundaryType::VIRTUAL : rb_type;
    }
    road_boundary_segment_info_tmp.width =
        static_cast<double>(road_boundary_env.boundary_width_cm()) *
        kCentiMeter2Meter;
    for (const auto &pt : road_boundary_env.curve().point()) {
      road_boundary_segment_info_tmp.points.emplace_back(pt.x(), pt.y());
    }
    map_info.all_road_boundaries_vec.emplace_back(
        std::move(road_boundary_segment_info_tmp));
  }

  std::unordered_map<std::string, std::pair<std::vector<std::string>,
                                            std::vector<std::string>>>
      road_boundary_map;
  for (auto const &road : zmap.map().hdmap().road()) {
    for (auto const &section : road.section()) {
      std::string section_id = section.id().id();
      std::vector<std::string> left_road_boundarys;
      for (auto const &road_boundary_id : section.left_boundary_id()) {
        left_road_boundarys.emplace_back(road_boundary_id.id());
      }
      std::vector<std::string> right_road_boundarys;
      for (auto const &road_boundary_id : section.right_boundary_id()) {
        right_road_boundarys.emplace_back(road_boundary_id.id());
      }
      const auto road_boundary_pair =
          std::make_pair(left_road_boundarys, right_road_boundarys);
      road_boundary_map[section_id] = road_boundary_pair;
    }
  }

  std::unordered_map<std::string, std::vector<std::string>>
      lane_split_boundary_ids;
  auto &lane_boundaries = map_info.all_lane_boundaries_vec;
  const auto &boundary_segments_env = zmap.map().hdmap().lane_boundary();
  for (const auto &boundary_env : boundary_segments_env) {
    if (boundary_env.id().id().empty() ||
        boundary_env.curve().point().empty()) {
      continue;
    }
    if (lane_split_boundary_ids.find(boundary_env.id().id()) !=
        lane_split_boundary_ids.end()) {
      continue;
    }
    if (!boundary_env.boundary_attr().empty()) {
      std::vector<ad_e2e::planning::LaneBoundaryInfo> lane_boundary_infos{};
      lane_boundary_infos.resize(boundary_env.boundary_attr_size());
      int stitch_index = 0;
      for (const auto &boundary_attr_type : boundary_env.boundary_attr()) {
        auto &lb_type =
            lane_boundary_infos[stitch_index].boundary_type.line_type;
        lb_type = LineType::UNKNOWN;
        auto &line_color =
            lane_boundary_infos[stitch_index].boundary_type.line_color;
        line_color = LineColor::COLOR_UNKNOWN;
        lane_boundary_infos[stitch_index].id =
            boundary_env.id().id() + '_' + std::to_string(stitch_index);
        switch (boundary_attr_type.type()) {
          case zLaneBoundType::LANEBOUNDARY_TYPE_NO_MARKING:
          case zLaneBoundType::LANEBOUNDARY_TYPE_DENSE_WIDE_DASH:
          case zLaneBoundType::LANEBOUNDARY_TYPE_SINGLE_DASHED_LINE:
          case zLaneBoundType::LANEBOUNDARY_TYPE_SHORT_DASHED_LINE:
          case zLaneBoundType::LANEBOUNDARY_TYPE_LONG_DASHED_LINE:
          case zLaneBoundType::LANEBOUNDARY_TYPE_DIAMOND_DECELERATION_LINE:
            lb_type = LineType::DASHED;
            break;
          case zLaneBoundType::LANEBOUNDARY_TYPE_SINGLE_SOLID_LINE:
          case zLaneBoundType::LANEBOUNDARY_TYPE_CURB:
          case zLaneBoundType::LANEBOUNDARY_TYPE_WALL_FLAT:
          case zLaneBoundType::LANEBOUNDARY_TYPE_END_OF_ROAD:
          case zLaneBoundType::LANEBOUNDARY_TYPE_GORE:
          case zLaneBoundType::LANEBOUNDARY_TYPE_OTHER_BARRIER:
          case zLaneBoundType::LANEBOUNDARY_TYPE_VARIABLE_DIRECTION_LINE:
            lb_type = LineType::SOLID;
            break;
          case zLaneBoundType::LANEBOUNDARY_TYPE_DOUBLE_SOLID:
            lb_type = LineType::SOLID_SOLID;
            break;
          case zLaneBoundType::LANEBOUNDARY_TYPE_DOUBLE_DASH:
          case zLaneBoundType::LANEBOUNDARY_TYPE_DOUBLE_DIAMOND_DECELERATION:
            lb_type = LineType::DASHED_DASHED;
            break;
          case zLaneBoundType::LANEBOUNDARY_TYPE_LEFT_SOLID_AND_RIGHT_DASH:
            lb_type = LineType::SOLID_DASHED;
            break;
          case zLaneBoundType::LANEBOUNDARY_TYPE_RIGHT_SOLID_AND_LEFT_DASH:
            lb_type = LineType::DASHED_SOLID;
            break;
          case zLaneBoundType::
              LANEBOUNDARY_TYPE_DOUBLE_DIAMOND_SOLID_DECELERATION:
            lb_type = LineType::FISH_LINE;
            break;
          case zLaneBoundType::
              LANEBOUNDARY_TYPE_LEFT_SOLID_AND_RIGHT_DIAMOND_DECELERATION:
          case zLaneBoundType::
              LANEBOUNDARY_TYPE_RIGHT_SOLID_AND_LEFT_DIAMOND_DECELERATION:
            lb_type = LineType::FISH_SOLID;
            break;
          case zLaneBoundType::
              LANEBOUNDARY_TYPE_DOUBLE_DIAMOND_DASHED_DECELERATION:
          case zLaneBoundType::
              LANEBOUNDARY_TYPE_LEFT_DASHED_AND_RIGHT_DIAMOND_DECELERATION:
          case zLaneBoundType::
              LANEBOUNDARY_TYPE_RIGHT_DASHED_AND_LEFT_DIAMOND_DECELERATION:
            lb_type = LineType::FISH_DASH;
            break;
          case zLaneBoundType::LANEBOUNDARY_TYPE_UNKNOWN:
          case zLaneBoundType::LANEBOUNDARY_TYPE_ARTIFICIAL_VIRTUAL:
          case zLaneBoundType::LANEBOUNDARY_TYPE_OFFSET_SPACE:
            lb_type = LineType::UNKNOWN;
            break;
        }
        switch (boundary_attr_type.color()) {
          case zark::zdrive_map::Color::COLOR_WHITE:
            line_color = LineColor::COLOR_WHITE;
            break;
          case zark::zdrive_map::Color::COLOR_YELLOW:
            line_color = LineColor::COLOR_YELLOW;
            break;
          case zark::zdrive_map::Color::COLOR_ORANGE:
            line_color = LineColor::COLOR_ORANGE;
            break;
          case zark::zdrive_map::Color::COLOR_BLUE:
            line_color = LineColor::COLOR_BLUE;
            break;
          case zark::zdrive_map::Color::COLOR_GREEN:
            line_color = LineColor::COLOR_GREEN;
            break;
          case zark::zdrive_map::Color::COLOR_GRAY:
            line_color = LineColor::COLOR_GRAY;
            break;
          case zark::zdrive_map::Color::COLOR_LEFT_GRAY_RIGHT_YELLOW:
            line_color = LineColor::LEFT_GRAY_RIGHT_YELLOW;
            break;
          case zark::zdrive_map::Color::COLOR_LEFT_YELLOW_RIGHT_WHITE:
            line_color = LineColor::LEFT_YELLOW_RIGHT_WHITE;
            break;
          case zark::zdrive_map::Color::COLOR_RED:
          case zark::zdrive_map::Color::COLOR_OTHERS:
            line_color = LineColor::OTHER;
            break;
          case zark::zdrive_map::Color::COLOR_UNKNOWN:
            line_color = LineColor::COLOR_UNKNOWN;
            break;
        }
        if (lb_type == LineType::FISH_SOLID) {
          lane_boundary_infos[stitch_index].boundary_type.line_type =
              LineType::SOLID;

        } else if (lb_type == LineType::FISH_DASH) {
          lane_boundary_infos[stitch_index].boundary_type.line_type =
              LineType::DASHED;
        }

        if (boundary_env.virtual_() && 1 == boundary_env.boundary_attr_size()) {
          for (const auto &pt : boundary_env.curve().point()) {
            lane_boundary_infos[0].points.emplace_back(pt.x(), pt.y());
          }
          lane_split_boundary_ids[boundary_env.id().id()].emplace_back(
              lane_boundary_infos[0].id);
          break;
        }

        const auto &boundary_start_s =
            boundary_attr_type.distance_info().start_offset_cm();
        const auto &boundary_end_s =
            boundary_attr_type.distance_info().end_offset_cm();
        int curva_pt_ind = 0;
        while (curva_pt_ind != boundary_env.curve().point_size()) {
          auto const &pt_offset =
              boundary_env.curve().offset_cm().at(curva_pt_ind);
          if (pt_offset < boundary_start_s) {
            curva_pt_ind++;
            continue;
          }
          if (pt_offset > boundary_end_s) break;
          if (pt_offset >= boundary_start_s && pt_offset <= boundary_end_s) {
            lane_boundary_infos[stitch_index].points.emplace_back(
                boundary_env.curve().point().at(curva_pt_ind).x(),
                boundary_env.curve().point().at(curva_pt_ind).y());
          }
          curva_pt_ind++;
        }

        lane_split_boundary_ids[boundary_env.id().id()].emplace_back(
            lane_boundary_infos[stitch_index].id);
        stitch_index++;
      }
      lane_boundaries.insert(lane_boundaries.end(), lane_boundary_infos.begin(),
                             lane_boundary_infos.end());
    } else {
      ad_e2e::planning::LaneBoundaryInfo lane_boundary_info_tmp;
      lane_boundary_info_tmp.id = boundary_env.id().id();
      lane_boundary_info_tmp.boundary_type.line_type = LineType::DASHED;
      lane_boundary_info_tmp.boundary_type.line_color = LineColor::COLOR_WHITE;
      lane_boundary_info_tmp.points.clear();
      for (const auto &pt : boundary_env.curve().point()) {
        lane_boundary_info_tmp.points.emplace_back(pt.x(), pt.y());
      }
      lane_boundaries.emplace_back(std::move(lane_boundary_info_tmp));
      lane_split_boundary_ids[boundary_env.id().id()].emplace_back(
          boundary_env.id().id());
    }
  }

  auto &stop_lines = map_info.all_stop_lines_vec;
  auto &cross_walks = map_info.all_cross_walks_vec;
  auto &clear_areas = map_info.all_clear_areas_vec;
  stop_lines.clear();
  cross_walks.clear();
  clear_areas.clear();
  for (const auto &marker : zmap.map().hdmap().road_marker()) {
    if (zRoadMarkerType::RM_TYPE_STOP_LINE == marker.type()) {
      ad_e2e::planning::StopLineInfo stop_line_info_tmp;
      stop_line_info_tmp.id = marker.id().id();
      if (zRoadMarkerType::RM_ARROW_STRAIGHT == marker.sub_type()) {
        stop_line_info_tmp.type = StopLineType::STOPLINETYPE_STRAIGHT;
      } else {
        stop_line_info_tmp.type = StopLineType::STOPLINETYPE_UNKNOWN;
      }

      stop_line_info_tmp.points.clear();

      stop_line_info_tmp.points.emplace_back(marker.p1().x(), marker.p1().y());
      stop_line_info_tmp.points.emplace_back(marker.p2().x(), marker.p2().y());
      stop_lines.emplace_back(std::move(stop_line_info_tmp));

    } else if (zRoadMarkerType::RM_TYPE_PEDESTRIAN == marker.type()) {
      if (marker.bounding_box().point().size() < 4) {
        LOG(ERROR) << "crosswalks point size < 4";
        continue;
      }
      auto &cross_walk = cross_walks.emplace_back();
      cross_walk.id = marker.id().id();
      cross_walk.points.clear();
      for (const auto &point : marker.bounding_box().point()) {
        cross_walk.points.emplace_back(point.x(), point.y());
      }
    } else if (zRoadMarkerType::RM_TYPE_NO_PARKING == marker.type()) {
      if (marker.bounding_box().point().size() < 4) {
        LOG(ERROR) << "clear_area point size < 4";
        continue;
      }
      auto &clear_area = clear_areas.emplace_back();
      clear_area.id = marker.id().id();

      clear_area.points.clear();
      for (const auto &point : marker.bounding_box().point()) {
        clear_area.points.emplace_back(point.x(), point.y());
      }
    }
  }

  const auto &lanes_env = zmap.map().hdmap().lane();
  auto &lanes = map_info.all_lanes_vec;
  std::unordered_map<std::string, size_t> lanes_map;
  for (const auto &lane_env : lanes_env) {
    if (lane_env.id().id().empty()) continue;
    if (lanes_map.find(lane_env.id().id()) != lanes_map.end()) continue;

    ad_e2e::planning::LaneInfo lane_info_tmp;
    lane_info_tmp.id = lane_env.id().id();
    lane_info_tmp.section_id = lane_env.road_section_id().id();
    lane_info_tmp.junction_id =
        lane_env.in_junction() ? lane_env.junction_id().id() : "";

    auto l_boudary_id = lane_env.left_boundary_id().id();
    lane_info_tmp.left_lane_boundary_ids.clear();
    if (lane_split_boundary_ids.find(l_boudary_id) !=
            lane_split_boundary_ids.end() &&
        !lane_split_boundary_ids[l_boudary_id].empty()) {
      lane_info_tmp.left_lane_boundary_ids.insert(
          lane_info_tmp.left_lane_boundary_ids.end(),
          lane_split_boundary_ids[l_boudary_id].begin(),
          lane_split_boundary_ids[l_boudary_id].end());
    }

    auto r_boudary_id = lane_env.right_boundary_id().id();
    lane_info_tmp.right_lane_boundary_ids.clear();
    if (lane_split_boundary_ids.find(r_boudary_id) !=
            lane_split_boundary_ids.end() &&
        !lane_split_boundary_ids[r_boudary_id].empty()) {
      lane_info_tmp.right_lane_boundary_ids.insert(
          lane_info_tmp.right_lane_boundary_ids.end(),
          lane_split_boundary_ids[r_boudary_id].begin(),
          lane_split_boundary_ids[r_boudary_id].end());
    }

    auto const &road_boundary_pair =
        road_boundary_map.find(lane_info_tmp.section_id);
    if (road_boundary_pair != road_boundary_map.end()) {
      lane_info_tmp.left_road_boundary_ids = road_boundary_pair->second.first;
      lane_info_tmp.right_road_boundary_ids = road_boundary_pair->second.second;
    }

    lane_info_tmp.left_lane_id =
        !lane_env.left_neighbor_forward_lane_ids().empty()
            ? lane_env.left_neighbor_forward_lane_ids().at(0).lane_id().id()
            : "";
    lane_info_tmp.right_lane_id =
        !lane_env.right_neighbor_forward_lane_ids().empty()
            ? lane_env.right_neighbor_forward_lane_ids().at(0).lane_id().id()
            : "";
    lane_info_tmp.next_lane_ids.clear();
    std::transform(lane_env.successor_id().begin(),
                   lane_env.successor_id().end(),
                   std::back_inserter(lane_info_tmp.next_lane_ids),
                   [](const zark::zdrive_map::Id &id) { return id.id(); });

    if (lane_env.center_line().point().empty()) {
      const zark::zdrive_map::LaneBoundary *left_boundary = nullptr;
      const zark::zdrive_map::LaneBoundary *right_boundary = nullptr;
      const auto &lane_boundaries = zmap.map().hdmap().lane_boundary();

      for (const auto &lane_candidate : lane_boundaries) {
        if (lane_candidate.id().id() == lane_env.left_boundary_id().id()) {
          left_boundary = &lane_candidate;
        }
        if (lane_candidate.id().id() == lane_env.right_boundary_id().id()) {
          right_boundary = &lane_candidate;
        }
        if (left_boundary && right_boundary) {
          break;
        }
      }
      if (left_boundary != nullptr && right_boundary != nullptr) {
        const auto &left_points = left_boundary->curve().point();
        const auto &right_points = right_boundary->curve().point();

        for (const auto &left_point : left_points) {
          double min_distance = std::numeric_limits<double>::max();
          const ad_e2e::planning::Point2d *closest_right_point = nullptr;

          for (const auto &right_point : right_points) {
            double distance =
                std::sqrt(std::pow(left_point.x() - right_point.x(), 2) +
                          std::pow(left_point.y() - right_point.y(), 2));

            if (distance < min_distance) {
              min_distance = distance;
              closest_right_point = new ad_e2e::planning::Point2d(
                  right_point.x(), right_point.y());
            }
          }

          if (closest_right_point != nullptr) {
            double center_x = (left_point.x() + closest_right_point->x()) / 2.0;
            double center_y = (left_point.y() + closest_right_point->y()) / 2.0;

            lane_info_tmp.points.emplace_back(center_x, center_y);
            delete closest_right_point;
          }
        }
      }
    } else {
      for (const auto &pt : lane_env.center_line().point()) {
        lane_info_tmp.points.emplace_back(pt.x(), pt.y());
      }
    }

    lane_info_tmp.turn_type = TurnType::NO_TURN;
    lane_info_tmp.turn_type_group = lane_env.topo_arrow();
    switch (lane_env.turn()) {
      case zLane::LANE_TURN_UNKNOWN:
      case zLane::LANE_TURN_NO_TURN:
        lane_info_tmp.turn_type = TurnType::NO_TURN;
        break;
      case zLane::LANE_TURN_LEFT_TURN:
        lane_info_tmp.turn_type = TurnType::LEFT_TURN;
        break;
      case zLane::LANE_TURN_RIGHT_TURN:
        lane_info_tmp.turn_type = TurnType::RIGHT_TURN;
        break;
      case zLane::LANE_TURN_U_LTURN:
      case zLane::LANE_TURN_U_RTURN:
        lane_info_tmp.turn_type = TurnType::U_TURN;
        break;
    }
    lane_info_tmp.is_virtual = lane_env.is_virtual();
    if (lane_info_tmp.is_virtual) {
      lane_info_tmp.type = TurnType::NO_TURN == lane_info_tmp.turn_type
                               ? LaneType::LANE_VIRTUAL_COMMON
                               : LaneType::LANE_VIRTUAL_JUNCTION;
    } else {
      lane_info_tmp.type = LaneType::LANE_NORMAL;
      if (!lane_env.lane_type().empty()) {
        const auto &lane_type = lane_env.lane_type().at(0).type();
        switch (lane_type) {
          case zLane::LANE_TYPE_NORMAL_LANE:
          case zLane::LANE_TYPE_MIXED_LANE:
          case zLane::LANE_TYPE_INTERSECTION_LANE:
          case zLane::LANE_TYPE_ETC_LANE:
          case zLane::LANE_TYPE_TOLL_STATION_LANE:
          case zLane::LANE_TYPE_CHECKPOINT_LANE:
          case zLane::LANE_TYPE_JCT:
          case zLane::LANE_TYPE_ISOLATED_LANE:
          case zLane::LANE_TYPE_DIVERSION_BELT_LANE:
          case zLane::LANE_TYPE_DANGEROUS_GOODS_LANE:
          case zLane::LANE_TYPE_CLIMBING_LANE:
          case zLane::LANE_TYPE_VARIABLE_LANE:
          case zLane::LANE_TYPE_CUSTOMS_SUPERVISION_LANE:
          case zLane::LANE_TYPE_REFUGE_LANE_APPROACH:
          case zLane::LANE_TYPE_TIDAL_LANE:
            lane_info_tmp.type = LaneType::LANE_NORMAL;
            break;
          case zLane::LANE_TYPE_EMERGENCY_LANE:
          case zLane::LANE_TYPE_EMERGENCY_PARKING_LANE:
          case zLane::LANE_TYPE_PARKING_LANE:
            lane_info_tmp.type = LaneType::LANE_EMERGENCY;
            break;
          case zLane::LANE_TYPE_HOV_LANE:
            lane_info_tmp.type = LaneType::LANE_HOV_NORMAL;
            break;
          case zLane::LANE_TYPE_ACCELERATION_LANE:
            lane_info_tmp.type = LaneType::LANE_ACC;
            break;
          case zLane::LANE_TYPE_U_TURN_LANE:
            lane_info_tmp.type = LaneType::LANE_LEFT_WAIT;
            break;
          case zLane::LANE_TYPE_DECELERATION_LANE:
            lane_info_tmp.type = LaneType::LANE_DEC;
            break;
          case zLane::LANE_TYPE_NON_MOTORIZED_LANE:
          case zLane::LANE_TYPE_SIDEWALK:
          case zLane::LANE_TYPE_MOTORCYCLE_LANE:
            lane_info_tmp.type = LaneType::LANE_NON_MOTOR;
            break;
          case zLane::LANE_TYPE_RAMP:
            lane_info_tmp.type = LaneType::LANE_RAMP;
            break;
          case zLane::LANE_TYPE_DEDICATED_LANE:
          case zLane::LANE_TYPE_BUS_LANE:
            lane_info_tmp.type = LaneType::LANE_BUS_NORMAL;
            break;
          case zLane::LANE_TYPE_UNKNOWN:
            lane_info_tmp.type = LaneType::LANE_UNKNOWN;
            break;
        }
        if (lane_type & zLane::LANE_TYPE_BUS_LANE) {
          lane_info_tmp.type = LaneType::LANE_BUS_NORMAL;
        }
      }
    }

    lane_info_tmp.is_merge =
        !lane_env.in_junction() &&
        lane_env.transition() == zark::zdrive_map::Lane::LANE_TRANSITION_MERGE;
    lane_info_tmp.is_split =
        lane_env.transition() ==
            zark::zdrive_map::Lane::LANE_TRANSITION_SPLIT ||
        (lane_env.in_junction() && lane_info_tmp.is_merge);
    lane_info_tmp.length =
        static_cast<double>(lane_env.length_cm()) * kCentiMeter2Meter;
    lane_info_tmp.speed_limit = lane_env.max_speed_limit_kmph() <= 0.1
                                    ? 40 / 3.6
                                    : lane_env.max_speed_limit_kmph() / 3.6;

    for (const auto &object : lane_env.objects()) {
      const zark::zdrive_map::AttachedElement::AttachLinkNodeElementType &type =
          object.type();
      if (type == zark::zdrive_map::AttachedElement::TRAFFIC_LIGHT) {
        auto &traffic_light = lane_info_tmp.traffic_lights.emplace_back();
        traffic_light = object.id().id();
      }
    }

    for (auto const &attached_object : lane_env.objects()) {
      if (attached_object.type() ==
          zark::zdrive_map::AttachedElement::ROAD_MARKER) {
        if (attached_object.roadmark_subtype() ==
            zRoadMarkerType::RM_TYPE_PEDESTRIAN) {
          lane_info_tmp.cross_walks.emplace_back(attached_object.id().id());
        } else if (attached_object.roadmark_subtype() ==
                   zRoadMarkerType::RM_TYPE_STOP_LINE) {
          lane_info_tmp.traffic_stop_lines.emplace_back(
              attached_object.id().id());
        } else if (attached_object.roadmark_subtype() ==
                   zRoadMarkerType::RM_TYPE_NO_PARKING) {
          lane_info_tmp.clear_areas.emplace_back(attached_object.id().id());
        }
      }
    }
    lane_info_tmp.stop_line = !lane_info_tmp.traffic_stop_lines.empty();

    lanes.emplace_back(std::move(lane_info_tmp));
    lanes_map.insert({lanes.back().id, lanes.size() - 1});
  }

  const auto &lanes_route = zmap.guidance().lane();
  for (const auto &lane_env : lanes_route) {
    if (lane_env.id().id().empty()) continue;
    if (lanes_map.find(lane_env.id().id()) != lanes_map.end()) {
      auto &lane_in_map = lanes.at(lanes_map[lane_env.id().id()]);
      auto &next_lane_ids = lane_in_map.next_lane_ids;
      for (int i = 0; i < next_lane_ids.size(); ++i) {
        if (lanes_map.find(next_lane_ids[i]) != lanes_map.end()) continue;
        if (next_lane_ids[i].find("nextvr") != next_lane_ids[i].npos) continue;
        next_lane_ids[i] += "_nextvr";
      }
      continue;
    }

    ad_e2e::planning::LaneInfo lane_info_tmp;
    lane_info_tmp.id = lane_env.id().id() + "_nextvr";
    lane_info_tmp.section_id = lane_env.section_id().id();
    lane_info_tmp.junction_id =
        lane_env.in_junction() ? lane_env.junction_id().id() : "";

    auto get_neighbor_id = [](const auto &neighbors) {
      return !neighbors.empty() ? neighbors.at(0).lane_id().id() + "_nextvr"
                                : "";
    };
    lane_info_tmp.left_lane_id =
        get_neighbor_id(lane_env.left_neighbor_forward_lane_id());
    lane_info_tmp.right_lane_id =
        get_neighbor_id(lane_env.right_neighbor_forward_lane_id());

    lane_info_tmp.next_lane_ids.clear();
    std::transform(
        lane_env.successor_id().begin(), lane_env.successor_id().end(),
        std::back_inserter(lane_info_tmp.next_lane_ids),
        [](const zark::zdrive_map::Id &id) { return id.id() + "_nextvr"; });

    lane_info_tmp.type = LaneType::LANE_NORMAL;
    if (!lane_env.lane_type().empty()) {
      const auto &lane_type = lane_env.lane_type().at(0).type();
      switch (lane_type) {
        case zLane::LANE_TYPE_NORMAL_LANE:
        case zLane::LANE_TYPE_MIXED_LANE:
        case zLane::LANE_TYPE_INTERSECTION_LANE:
        case zLane::LANE_TYPE_ETC_LANE:
        case zLane::LANE_TYPE_TOLL_STATION_LANE:
        case zLane::LANE_TYPE_CHECKPOINT_LANE:
        case zLane::LANE_TYPE_JCT:
        case zLane::LANE_TYPE_ISOLATED_LANE:
        case zLane::LANE_TYPE_DIVERSION_BELT_LANE:
        case zLane::LANE_TYPE_DANGEROUS_GOODS_LANE:
        case zLane::LANE_TYPE_CLIMBING_LANE:
        case zLane::LANE_TYPE_VARIABLE_LANE:
        case zLane::LANE_TYPE_CUSTOMS_SUPERVISION_LANE:
        case zLane::LANE_TYPE_REFUGE_LANE_APPROACH:
        case zLane::LANE_TYPE_TIDAL_LANE:
          lane_info_tmp.type = LaneType::LANE_NORMAL;
          break;
        case zLane::LANE_TYPE_EMERGENCY_LANE:
        case zLane::LANE_TYPE_EMERGENCY_PARKING_LANE:
        case zLane::LANE_TYPE_PARKING_LANE:
          lane_info_tmp.type = LaneType::LANE_EMERGENCY;
          break;
        case zLane::LANE_TYPE_HOV_LANE:
          lane_info_tmp.type = LaneType::LANE_HOV_NORMAL;
          break;
        case zLane::LANE_TYPE_ACCELERATION_LANE:
          lane_info_tmp.type = LaneType::LANE_ACC;
          break;
        case zLane::LANE_TYPE_U_TURN_LANE:
          lane_info_tmp.type = LaneType::LANE_LEFT_WAIT;
          break;
        case zLane::LANE_TYPE_DECELERATION_LANE:
          lane_info_tmp.type = LaneType::LANE_DEC;
          break;
        case zLane::LANE_TYPE_NON_MOTORIZED_LANE:
        case zLane::LANE_TYPE_SIDEWALK:
        case zLane::LANE_TYPE_MOTORCYCLE_LANE:
          lane_info_tmp.type = LaneType::LANE_NON_MOTOR;
          break;
        case zLane::LANE_TYPE_RAMP:
          lane_info_tmp.type = LaneType::LANE_RAMP;
          break;
        case zLane::LANE_TYPE_DEDICATED_LANE:
        case zLane::LANE_TYPE_BUS_LANE:
          lane_info_tmp.type = LaneType::LANE_BUS_NORMAL;
          break;
        case zLane::LANE_TYPE_UNKNOWN:
          lane_info_tmp.type = LaneType::LANE_UNKNOWN;
          break;
      }
      if (lane_type & zLane::LANE_TYPE_BUS_LANE) {
        lane_info_tmp.type = LaneType::LANE_BUS_NORMAL;
      }
    }
    lane_info_tmp.is_merge =
        !lane_env.in_junction() &&
        lane_env.transition() == zark::zdrive_map::Lane::LANE_TRANSITION_MERGE;
    lane_info_tmp.is_split =
        lane_env.transition() ==
            zark::zdrive_map::Lane::LANE_TRANSITION_SPLIT ||
        (lane_env.in_junction() && lane_info_tmp.is_merge);
    lane_info_tmp.length =
        static_cast<double>(lane_env.distance_info().end_offset_cm() -
                            lane_env.distance_info().start_offset_cm()) *
        kCentiMeter2Meter;

    lanes.emplace_back(std::move(lane_info_tmp));
  }

  for (auto &route_sec : map_info.route.sections) {
    for (auto &lane_id : route_sec.lane_ids) {
      if (lanes_map.find(lane_id) != lanes_map.end()) continue;
      if (lane_id.find("nextvr") != lane_id.npos) continue;
      lane_id += "_nextvr";
    }
  }

  auto &junctions = map_info.all_junctions_vec;
  for (const auto &junction : zmap.map().hdmap().junction()) {
    auto &e2e_junction = junctions.emplace_back();
    e2e_junction.id = junction.id().id();
    e2e_junction.points.clear();
    for (const auto &point : junction.polygon().points()) {
      e2e_junction.points.emplace_back(point.x(), point.y());
    }
  }

  auto &traffic_lights = map_info.all_traffic_lights_vec;
  for (const auto &traffic_light : zmap.map().hdmap().traffic_light()) {
    auto &e2e_traffic_light = traffic_lights.emplace_back();
    e2e_traffic_light.id = traffic_light.id().id();
    e2e_traffic_light.center_point.set_x(traffic_light.center_point().x());
    e2e_traffic_light.center_point.set_y(traffic_light.center_point().y());
    e2e_traffic_light.sub_type = traffic_light.sub_type();
    e2e_traffic_light.count_down_sec = traffic_light.count_down_sec();
    e2e_traffic_light.is_blinking = traffic_light.is_flashing();
    switch (traffic_light.color()) {
      case zColor::COLOR_UNKNOWN:
        e2e_traffic_light.light_status = LightStatus::UNKNOWN_LIGHT;
        break;
      case zColor::COLOR_GREEN:
        e2e_traffic_light.light_status = LightStatus::GREEN_LIGHT;
        break;
      case zColor::COLOR_RED:
        e2e_traffic_light.light_status = LightStatus::RED_LIGHT;
        break;
      case zColor::COLOR_YELLOW:
        e2e_traffic_light.light_status = LightStatus::YELLOW_LIGHT;
        break;
      default:
        e2e_traffic_light.light_status = LightStatus::NONE_LIGHT;
        break;
    }
  }

  auto &navi_start = map_info.route.navi_start;
  if (navi_start_section != nullptr) {
    navi_start.section_id = ego_road_section_id;
    navi_start.s_offset =
        -static_cast<double>(
            navi_start_section->distance_info().distance_to_vehicle_cm()) *
        kCentiMeter2Meter;
  }

  auto &navi_end = map_info.route.navi_end;
  const auto end_section = zmap.guidance().section().rbegin();
  if (!end_section->id().id().empty()) {
    navi_end.section_id = end_section->id().id();
    navi_end.s_offset =
        static_cast<double>(zmap.navigation_info().hd_remain_distance_m()) -
        static_cast<double>(
            end_section->distance_info().distance_to_vehicle_cm()) *
            kCentiMeter2Meter;
  }

  switch (zmap.map().map_header().type()) {
    case zMapType::MapHeader_MapType_MAP_TYPE_HDPRO:
    case zMapType::MapHeader_MapType_MAP_TYPE_SD_HDPRO:

    case zMapType::MapHeader_MapType_MAP_TYPE_HDLITE:
    case zMapType::MapHeader_MapType_MAP_TYPE_SD_HDLITE:
      map_info.type = ad_e2e::planning::MapType::HD_MAP;
      break;
  }

  return map_info;
}

void E2EPncComponent::MapCallback(hobot::dataflow::spMsgResourceProc proc,
                                  const hobot::dataflow::MessageLists &msgs) {
  auto &input_map_msgs = msgs[proc->GetResultIndex("local_map")];
  if (input_map_msgs->empty() || ego_road_section_id_.empty()) return;
  auto act_msg = std::dynamic_pointer_cast<zark::zdrive_map::ZarkMapMsg>(
      input_map_msgs->back());
  if (act_msg == nullptr || !act_msg->proto.has_map() ||
      !act_msg->proto.map().has_map_header() ||
      !act_msg->proto.has_guidance() || (act_msg->proto.path_size() <= 0)) {
    return;
  }
  zMapType act_msg_type = act_msg->proto.map().map_header().type();
  bool HDMAP_Type_Enable =
      ((act_msg_type == zMapType::MapHeader_MapType_MAP_TYPE_HDPRO) ||
       (act_msg_type == zMapType::MapHeader_MapType_MAP_TYPE_SD_HDPRO));
  bool HD_LITE_Type_Enable =
      ((act_msg_type == zMapType::MapHeader_MapType_MAP_TYPE_HDLITE) ||
       (act_msg_type == zMapType::MapHeader_MapType_MAP_TYPE_SD_HDLITE));
  bool CP_Type_Enable =
      (act_msg_type == zMapType::MapHeader_MapType_MAP_TYPE_CP);
  bool LOCAL_Type_Enable =
      (act_msg_type == zMapType::MapHeader_MapType_MAP_TYPE_LOCAL);
  bool HPA_Type_Enable =
      (act_msg_type == zMapType::MapHeader_MapType_MAP_TYPE_HPA);
  bool MOD_Type_Enable =
      (act_msg_type == zMapType::MapHeader_MapType_MAP_TYPE_MOD);
  bool SD_Type_Enable =
      ((act_msg_type == zMapType::MapHeader_MapType_MAP_TYPE_SDPRO) ||
       (act_msg_type == zMapType::MapHeader_MapType_MAP_TYPE_SD));
  bool Map_Type_Enable = (HDMAP_Type_Enable || HD_LITE_Type_Enable ||
                          CP_Type_Enable || LOCAL_Type_Enable ||
                          HPA_Type_Enable || MOD_Type_Enable || SD_Type_Enable);
  if (!Map_Type_Enable) return;
  map_msg_.OnNewMessage(*act_msg);

  map_container_.InsertMap(CreateMapInfo(*act_msg, ego_road_section_id_));
}

void E2EPncComponent::MapLocalizationCallback(
    hobot::dataflow::spMsgResourceProc proc,
    const hobot::dataflow::MessageLists &msgs) {
  auto &input_mf_msg = msgs[proc->GetResultIndex("map_localization_state")];
  if (input_mf_msg->empty()) {
    ego_road_section_id_ = "";
    return;
  }
  auto act_msg =
      std::dynamic_pointer_cast<zark::mapfusion::MapLocalizationInfoMsg>(
          input_mf_msg->back());
  ego_road_section_id_ = act_msg->proto.road_section_id();

  map_fusion_msg_.OnNewMessage(*act_msg);
}

void E2EPncComponent::VehicleStatusCallback(
    hobot::dataflow::spMsgResourceProc proc,
    const hobot::dataflow::MessageLists &msgs) {
  auto &input_veh_status_msg = msgs[proc->GetResultIndex("vehicle_status")];

  if (input_veh_status_msg->empty()) {
    return;
  }
  auto act_msg =
      std::dynamic_pointer_cast<zark::cheryos::VehiclePowerAndChassisMsgProto>(
          input_veh_status_msg->back());
  veh_status_msg_.OnNewMessage(*act_msg);
}

void E2EPncComponent::FctOutputsCallback(
    hobot::dataflow::spMsgResourceProc proc,
    const hobot::dataflow::MessageLists &msgs) {
  auto &input_fct_msgs = msgs[proc->GetResultIndex("fct_outputs")];
  if (input_fct_msgs->empty()) {
    return;
  }
  auto act_msg = std::dynamic_pointer_cast<zark::ads_fct::FctMsgProto>(
      input_fct_msgs->back());
  fct_msg_.OnNewMessage(*act_msg);
}

std::pair<ad_e2e::planning::PlanResult,
          std::shared_ptr<ad_e2e::planning::PlanningInputFrame>>
E2EPncComponent::GetPlanningInputFrame() {
  ad_e2e::planning::PlanResult result = ad_e2e::planning::PlanResult::PLAN_OK;
  auto res = std::make_shared<ad_e2e::planning::PlanningInputFrame>();
  double prediction_stamp{0.0};

  std::string error_msg = "[";
  if (auto latest = localization_msg_.Back(kPlanningCounter); !latest.ok()) {
    LOG(ERROR) << "[InputMsg] odometry: " << latest.status().ToString();
    result = (result == ad_e2e::planning::PlanResult::PLAN_OK
                  ? ad_e2e::planning::PlanResult::PLAN_MSG_ODOMETRY_TIMEOUT
                  : result);
    error_msg += "odometry,";
  } else {
    res->odometry = latest->second;
    res->last_msg_timestamp = std::max(res->last_msg_timestamp, latest->first);
    res->last_odometry_timestamp = latest->first;
  }

  if (auto latest = veh_status_msg_.Back(kPlanningCounter); !latest.ok()) {
    LOG(ERROR) << "[InputMsg] vehicle_status: " << latest.status().ToString();
    result = (result == ad_e2e::planning::PlanResult::PLAN_OK
                  ? ad_e2e::planning::PlanResult::PLAN_MSG_VEHICLE_TIMEOUT
                  : result);
    error_msg += "vehicle_status,";
  } else {
    res->vehicle_status = latest->second;
    res->last_msg_timestamp = std::max(res->last_msg_timestamp, latest->first);
#if 0

    res->estimate_a =
        latest->second->vehicle_control_status().lon_control().longitude_acc();
    const double delta_time = 0.3;
    auto prev = veh_status_msg_.GetMsgAt(latest->first - delta_time);
    if (prev.ok()) {
      const auto dt = latest->first - prev->first;
      if (dt > 0.5 * delta_time) {
        const double dv = latest->second->vehicle_control_status()
                              .lon_control()
                              .vehicle_speed()
                              .vehicle_speed() -
                          prev->second->vehicle_control_status()
                              .lon_control()
                              .vehicle_speed()
                              .vehicle_speed();
        res->estimate_a = dv / dt;
      }
    }
#endif
  }

  if (auto latest = map_fusion_msg_.Get(kPlanningCounter); !latest.ok()) {
    LOG(ERROR) << "[InputMsg] mapfusion: " << latest.status().ToString();
    result = (result == ad_e2e::planning::PlanResult::PLAN_OK
                  ? ad_e2e::planning::PlanResult::PLAN_MSG_MAPLOCATION_TIMEOUT
                  : result);
    error_msg += "mapfusion,";
  } else {
    res->mapfusion = latest->second;
    res->last_msg_timestamp = std::max(res->last_msg_timestamp, latest->first);
  }

  if (auto latest = map_msg_.Get(kPlanningCounter); !latest.ok()) {
    LOG(ERROR) << "[InputMsg] env_map: " << latest.status().ToString();
    result = (result == ad_e2e::planning::PlanResult::PLAN_OK
                  ? ad_e2e::planning::PlanResult::PLAN_MSG_MAP_TIMEOUT
                  : result);
    error_msg += "env_map,";
  } else {
    res->map_input_data = latest->second;
    res->last_msg_timestamp = std::max(res->last_msg_timestamp, latest->first);
  }

  if (auto latest = predic_objs_msg_.Get(kPlanningCounter); !latest.ok()) {
    LOG(ERROR) << "[InputMsg] prediction " << latest.status().ToString();
    result = (result == ad_e2e::planning::PlanResult::PLAN_OK
                  ? ad_e2e::planning::PlanResult::PLAN_MSG_OBSTACLE_TIMEOUT
                  : result);
    error_msg += "prediction,";
  } else {
    res->prediction = latest->second;
    res->last_msg_timestamp = std::max(res->last_msg_timestamp, latest->first);

    prediction_stamp =
        latest->second->proto.prediction_object().empty()
            ? latest->first
            : static_cast<double>(
                  latest->second->proto.header().timestamp_ns()) *
                  1e-1;
  }

  if (auto latest = fct_msg_.Back(kPlanningCounter); !latest.ok()) {
    LOG(ERROR) << "[InputMsg] fct_outputs: " << latest.status().ToString();
    result = (result == ad_e2e::planning::PlanResult::PLAN_OK
                  ? ad_e2e::planning::PlanResult::PLAN_MSG_BEHAVIOR_TIMEOUT
                  : result);
    error_msg += "fct_outputs,";
  } else {
    res->behavior = latest->second;
    res->last_msg_timestamp = std::max(res->last_msg_timestamp, latest->first);
  }

  ad_e2e::planning::MapElement map_element;
  if (auto ret = map_container_.GetMap(&map_element);
      ad_e2e::planning::ErrorCode::PLANNING_OK != ret) {
    result = (result == ad_e2e::planning::PlanResult::PLAN_OK
                  ? ad_e2e::planning::PlanResult::PLAN_FAIL_NO_MAP
                  : result);
    error_msg += "map_container,";
  }
  res->map_ptr = map_element.map_ptr;
  res->use_hd_map = true;
  e2e_noa::planning::Log2FG::LogMissionData("MsgError", error_msg + "]");
  return std::pair<ad_e2e::planning::PlanResult,
                   std::shared_ptr<ad_e2e::planning::PlanningInputFrame>>(
      result, res);
}

void handler(int sig) {
  ::signal(sig, SIG_DFL);
  std::cout << boost::stacktrace::stacktrace() << std::endl;
}

void E2EPncComponent::E2EPncProc(hobot::dataflow::spMsgResourceProc proc,
                                 const hobot::dataflow::MessageLists &msgs) {
  {
    std::scoped_lock<std::mutex> lock(thread_info_mutex_);

    planning_callback_tid_ = pthread_self();
  }
  signal(SIGSEGV, &handler);
  auto input_frame = GetPlanningInputFrame();
  static uint64_t seq_num = 0;
  seq_num++;
  if (seq_num > 1000000) {
    seq_num = 0;
  }
  auto begin_time = clock_->Microseconds() / kMicroToMilli;

  if (input_frame.second != nullptr) {
    input_frame.second->seq_num = seq_num;
  }
  if (e2e_noa::planning::kSimEnable) {
    while (input_frame.first != ad_e2e::planning::PlanResult::PLAN_OK) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      input_frame = GetPlanningInputFrame();
    }
  }

  auto planning_result = core_->PlanningCallback(input_frame);

  std::shared_ptr<ad_e2e::planning::planning_result_msg_type> msg(
      &(planning_result->first),
      [](ad_e2e::planning::planning_result_msg_type *) {});

  auto planner_res_port = proc->GetOutputPort("planner_res_pub");
  if (planning_result && planner_res_port) {
    msg->SetGenTimestamp(clock_->Microseconds() / kMicroToMilli);
    msg->SetDoneTimestamp(clock_->Microseconds() / kMicroToMilli);
    planner_res_port->Send(msg);
  }

  zark::common::MODULE_STATUS zark_e2e_pnc_res =
      zark::common::MODULE_STATUS::NORMAL;
  if (input_frame.first != ad_e2e::planning::PlanResult::PLAN_OK) {
    zark_e2e_pnc_res = zark::common::MODULE_STATUS::PLANNING_DATA_ABNORMAL;
    if (e2e_noa::InRange(input_frame.first,
                         ad_e2e::planning::PLAN_MSG_MAP_TIMEOUT,
                         ad_e2e::planning::PLAN_FAIL_CURRENT_NEAREST_LANE)) {
      zark_e2e_pnc_res = zark::common::MODULE_STATUS::HDMAP_DATA_ABNORMAL;
    }
    if (e2e_noa::InRange(
            input_frame.first, ad_e2e::planning::PLAN_MSG_OBSTACLE_TIMEOUT,
            ad_e2e::planning::PLAN_MSG_OBSTACLE_STAMP_NONALIGNED)) {
      zark_e2e_pnc_res = zark::common::MODULE_STATUS::PERCEPTION_DATA_ABNORMAL;
    }
    if (e2e_noa::InRange(input_frame.first,
                         ad_e2e::planning::PLAN_MSG_MAPLOCATION_TIMEOUT,
                         ad_e2e::planning::PLAN_MSG_ODOMETRY_DELAY)) {
      zark_e2e_pnc_res =
          zark::common::MODULE_STATUS::LOCALIZATION_DATA_ABNORMAL;
    }
    if (e2e_noa::InRange(input_frame.first,
                         ad_e2e::planning::PLAN_MSG_VEHICLE_TIMEOUT,
                         ad_e2e::planning::PLAN_MSG_VEHICLE_TRANSFORM_FAIL)) {
      zark_e2e_pnc_res = zark::common::MODULE_STATUS::VEHICLE_DATA_ABNORMAL;
    }
    if (e2e_noa::InRange(input_frame.first,
                         ad_e2e::planning::PLAN_FAIL_REF_LINE_POINTS_NUM_SHORT,
                         ad_e2e::planning::PLAN_FAIL_REF_LINE_CURVE_FITTING)) {
      zark_e2e_pnc_res =
          zark::common::MODULE_STATUS::REFERENCE_LINE_DATA_ABNORMAL;
    }
  } else {
    switch (planning_result->second.result) {
      case ad_e2e::planning::RESULT_FAIL_GET_MAP:
      case ad_e2e::planning::RESULT_FAIL_GET_TRAFFIC:
        zark_e2e_pnc_res = zark::common::MODULE_STATUS::HDMAP_DATA_ABNORMAL;
        break;

      case ad_e2e::planning::RESULT_FAIL_GET_VEHICLE:
        zark_e2e_pnc_res = zark::common::MODULE_STATUS::VEHICLE_DATA_ABNORMAL;
        break;

      case ad_e2e::planning::RESULT_FAIL_GET_LOCALIZATION:
        zark_e2e_pnc_res =
            zark::common::MODULE_STATUS::LOCALIZATION_DATA_ABNORMAL;
        break;

      case ad_e2e::planning::RESULT_INACTIVE:
      case ad_e2e::planning::RESULT_FAIL:
      case ad_e2e::planning::RESULT_FAIL_UPDATE_MANAGER:
      case ad_e2e::planning::RESULT_FAIL_MANAGER_STAMP:
      case ad_e2e::planning::RESULT_FAIL_MANAGER_NO_MAP:
      case ad_e2e::planning::RESULT_FAIL_MANAGER_TARGET_LANE:
      case ad_e2e::planning::RESULT_FAIL_MANAGER_UPDATE_REFLINE:
      case ad_e2e::planning::RESULT_FAIL_MANAGER_CURRENT_LANE:
      case ad_e2e::planning::RESULT_FAIL_MANAGER_PLANNER:
        zark_e2e_pnc_res = zark::common::MODULE_STATUS::PLANNING_DATA_ABNORMAL;
        break;

      case ad_e2e::planning::RESULT_OK:
      default:
        zark_e2e_pnc_res = zark::common::MODULE_STATUS::NORMAL;
        break;
    }
  }

  std::shared_ptr<zark::common::HeartBeatMsg> heart_msg =
      std::make_shared<zark::common::HeartBeatMsg>();
  zark::common::HeartBeat &heart_proto = heart_msg->proto;
  heart_proto.set_module_name("e2e_pnc");
  heart_proto.set_module_status(zark_e2e_pnc_res);

  const char *full_version_info = GIT_COMMIT_HASH " (" GIT_COMMIT_TIMESTAMP ")";
  std::string version_as_string(full_version_info);
  heart_proto.mutable_version()->set_module_version(version_as_string);

  heart_msg->SetGenTimestamp(clock_->Microseconds() / kMicroToMilli);
  heart_proto.set_begin_timestamp(static_cast<uint64_t>(begin_time));
  heart_proto.set_end_timestamp(
      static_cast<uint64_t>(clock_->Microseconds() / kMicroToMilli));
  heart_msg->SetDoneTimestamp(clock_->Microseconds() / kMicroToMilli);
  auto heart_output_port = proc->GetOutputPort("planner_heartbeat_pub");
  if (heart_output_port) {
    heart_output_port->Send(heart_msg);
  }

  DFHLOG_I("E2EPncProc callback finished!");
}

void E2EPncComponent::E2EPncDebugProc(
    hobot::dataflow::spMsgResourceProc proc,
    const hobot::dataflow::MessageLists &msgs) {
  std::shared_ptr<zark::e2e_noa::debug::NOADebugInfoMsg> msg =
      std::make_shared<zark::e2e_noa::debug::NOADebugInfoMsg>();
  {
    std::lock_guard<std::mutex> loc(debug_info_lock_);
    msg->proto.Swap(&NOA_DEBUG_INFO);
    NOA_DEBUG_CLEAR;
  }

  PostProcessDebugMSG(msg);

  msg->SetGenTimestamp(clock_->Microseconds() / kMicroToMilli);
  msg->SetDoneTimestamp(clock_->Microseconds() / kMicroToMilli);
  auto output_port = proc->GetOutputPort("planner_debug_pub");
  if (output_port) {
    output_port->Send(msg);
  }

  DFHLOG_I("E2EPncDebugProc callback finished!");
}

DATAFLOW_REGISTER_MODULE(E2EPncComponent)
