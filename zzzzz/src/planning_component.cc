
// #include <config/config.h>
#include "apps/planning/src/planning_component.h"

#include <csignal>
#include <functional>
#include <list>
#include <string>
#include <vector>

#include "apps/planning/src/common/clock.h"
#include "apps/planning/src/common/conversion.h"
#include "apps/planning/src/common/data_reader/data_reader.h"
#include "apps/planning/src/common/data_reader/data_transform.h"
#include "apps/planning/src/common/log.h"
#include "apps/planning/src/common/proto_convertor/proto_convertor.h"
#include "apps/planning/src/config/conf_gflags.h"
#include "apps/planning/src/config/config_main.h"
#include "apps/planning/src/on_lane_planning.h"
#include "apps/planning/src/planning_msgs/planning.h"
#include "apps/planning/src/planning_msgs/planning_config.h"
#include "communication/common/types.h"
#include "dataflow/module/port.h"
#include "dataflow/module_loader/register_module_macro.h"
#include "message/proto/proto_msg.hpp"
#include "message/proto/proto_serializer.hpp"
#include "messages/common/heartbeat.pb.h"
#include "messages/localization/localization.pb.h"
#include "messages/map_service/all_map_new.pb.h"
#include "messages/mapfusion/map_fusion.pb.h"
#include "messages/planning/ads_adptrin.pb.h"
#include "messages/planning/ads_com.pb.h"
#include "messages/planning/ads_fct.pb.h"
#include "messages/planning/path_and_reference_line.pb.h"
#include "messages/planning/planning.pb.h"
#include "messages/planning/planning_debug.pb.h"
#include "messages/sensors/vehicle_upstream.pb.h"
#include "apps/planning/src/common/local_route/local_route.h"

namespace zark {

namespace prediction {
namespace proto {
using PredictionObjectsMsg = hobot::message::ProtoMsg<PredictionObjects>;
using PredictionObjectsMsgSerializer =
    hobot::message::ProtobufSerializer<PredictionObjects>;
}  // namespace proto
}  // namespace prediction

namespace ads_fct {
using FCT_OutputsMsg = hobot::message::ProtoMsg<FCT_Outputs>;
using FCT_OutputsMsgSerializer =
    hobot::message::ProtobufSerializer<FCT_Outputs>;
}  // namespace ads_fct

namespace ads_decision {
using DEC_OutputsMSG = hobot::message::ProtoMsg<DEC_Outputs>;
using DEC_OutputsMSGSerializer =
    hobot::message::ProtobufSerializer<DEC_Outputs>;
using DecisionMeasInfoMSG = hobot::message::ProtoMsg<DecisionMeasInfo>;
using DecisionMeasInfoMSGSerializer =
    hobot::message::ProtobufSerializer<DecisionMeasInfo>;
}  // namespace ads_decision

namespace localization {
using LocalizationInfoMsg = hobot::message::ProtoMsg<LocalizationInfo>;
using LocalizationInfoMsgSerializer =
    hobot::message::ProtobufSerializer<LocalizationInfo>;
}  // namespace localization

namespace hdmap_new {
using ZarkMapMsg = hobot::message::ProtoMsg<ZarkMap>;
using ZarkMapMsgSerializer = hobot::message::ProtobufSerializer<ZarkMap>;
}  // namespace hdmap_new

namespace mapfusion {
using MapLocalizationInfoMsg = hobot::message::ProtoMsg<MapLocalizationInfo>;
using MapLocalizationInfoMsgSerializer =
    hobot::message::ProtobufSerializer<MapLocalizationInfo>;
}  // namespace mapfusion

using SteerStateMsg = hobot::message::ProtoMsg<VehicleUpstream::SteerSate>;
using SteerStateMsgSerializer =
    hobot::message::ProtobufSerializer<VehicleUpstream::SteerSate>;

using CanDataMsg = hobot::message::ProtoMsg<ads_adptrin::ADAS_Inputs>;
using CanDataMsgSerializer =
    hobot::message::ProtobufSerializer<ads_adptrin::ADAS_Inputs>;

namespace planning {
using PlanningOutputMsgMsg = hobot::message::ProtoMsg<PlanningOutputMsg>;
using PlanningOutputMsgMsgSerializer =
    hobot::message::ProtobufSerializer<PlanningOutputMsg>;
using PathAndReferenceLineMsg = hobot::message::ProtoMsg<PathAndReferenceLine>;
using PathAndReferenceLineMsgSerializer =
    hobot::message::ProtobufSerializer<PathAndReferenceLine>;
using PlanningDebugInfoMsg = hobot::message::ProtoMsg<PlanningDebugInfo>;
using PlanningDebugInfoMsgSerializer =
    hobot::message::ProtobufSerializer<PlanningDebugInfo>;
}  // namespace planning

namespace heartbeat {
using HeartBeatMsg = hobot::message::ProtoMsg<HeartBeat>;
using HeartBeatMsgSerializer = hobot::message::ProtobufSerializer<HeartBeat>;
}  // namespace heartbeat
}  // namespace zark

/**
 * To resolve unused warning, you can delete
 * the marco after variables actually used.
 */

PlanningComponent::PlanningComponent(
    const hobot::dataflow::ModuleOption &module_option)
    : hobot::dataflow::Module(module_option) {
  // init zlog env config
  INIT_ZLOG;
}

PlanningComponent::~PlanningComponent() {
  // deinit zlog env config
  DEINIT_ZLOG;
}

void PlanningComponent::InitPortsAndProcs() {
  DF_MODULE_INIT_IDL_INPUT_PORT("PredictionObstacles",
                                zark::prediction::proto::PredictionObjects);
  DF_MODULE_INIT_IDL_INPUT_PORT("localization",
                                zark::localization::LocalizationInfo);
  DF_MODULE_INIT_IDL_INPUT_PORT("local_map", zark::hdmap_new::ZarkMap);
  DF_MODULE_INIT_IDL_INPUT_PORT("map_localization_state",
                                zark::mapfusion::MapLocalizationInfo);
  DF_MODULE_INIT_IDL_INPUT_PORT("fct_outputs", zark::ads_fct::FCT_Outputs);
  DF_MODULE_INIT_IDL_INPUT_PORT("steerstate", zark::VehicleUpstream::SteerSate);
  DF_MODULE_INIT_IDL_INPUT_PORT("adptrin_outputs",
                                zark::ads_adptrin::ADAS_Inputs);
  DF_MODULE_INIT_IDL_OUTPUT_PORT("PlanningOutputMsg",
                                 zark::planning::PlanningOutputMsg);
  DF_MODULE_INIT_IDL_OUTPUT_PORT("PlanningDebugInfo",
                                 zark::planning::PlanningDebugInfo);
  DF_MODULE_INIT_IDL_OUTPUT_PORT("PlanningOutputRefLine",
                                 zark::planning::PathAndReferenceLine);
  DF_MODULE_INIT_IDL_OUTPUT_PORT("Heartbeat", zark::heartbeat::HeartBeat);

  DF_MODULE_INIT_IDL_OUTPUT_PORT("NoaDecisionOutput",
                                 zark::ads_decision::DEC_Outputs);
  DF_MODULE_INIT_IDL_OUTPUT_PORT("NoaDecisionMeasureOutput",
                                 zark::ads_decision::DecisionMeasInfo);

  DF_MODULE_REGISTER_HANDLE_MSGS_PROC(
      "PredictionCallback", PlanningComponent, PredictionCallback,
      hobot::dataflow::ProcType::DF_MSG_COND_PROC,
      DF_VECTOR("PredictionObstacles"), DF_VECTOR());
  DF_MODULE_REGISTER_HANDLE_MSGS_PROC(
      "LocalizationCallback", PlanningComponent, LocalizationCallback,
      hobot::dataflow::ProcType::DF_MSG_COND_PROC, DF_VECTOR("localization"),
      DF_VECTOR());
  DF_MODULE_REGISTER_HANDLE_MSGS_PROC(
      "MapCallback", PlanningComponent, MapCallback,
      hobot::dataflow::ProcType::DF_MSG_COND_PROC, DF_VECTOR("local_map"),
      DF_VECTOR());
  DF_MODULE_REGISTER_HANDLE_MSGS_PROC(
      "MapLocalizationCallback", PlanningComponent, MapLocalizationCallback,
      hobot::dataflow::ProcType::DF_MSG_COND_PROC,
      DF_VECTOR("map_localization_state"), DF_VECTOR());
  DF_MODULE_REGISTER_HANDLE_MSGS_PROC(
      "FctDataCallback", PlanningComponent, FctDataCallback,
      hobot::dataflow::ProcType::DF_MSG_COND_PROC, DF_VECTOR("fct_outputs"),
      DF_VECTOR());
  DF_MODULE_REGISTER_HANDLE_MSGS_PROC(
      "CanDataCallback", PlanningComponent, CanDataCallback,
      hobot::dataflow::ProcType::DF_MSG_COND_PROC, DF_VECTOR("adptrin_outputs"),
      DF_VECTOR());
  DF_MODULE_REGISTER_HANDLE_MSGS_PROC(
      "PlanningProc", PlanningComponent, PlanningProc,
      hobot::dataflow::ProcType::DF_MSG_TIMER_PROC, DF_VECTOR(),
      DF_VECTOR("PlanningOutputMsg", "NoaDecisionOutput"));
  DF_MODULE_REGISTER_HANDLE_MSGS_PROC(
      "PlanningDebugProc", PlanningComponent, PlanningDebugProc,
      hobot::dataflow::ProcType::DF_MSG_TIMER_PROC, DF_VECTOR(),
      DF_VECTOR("PlanningDebugInfo"));
  DF_MODULE_REGISTER_HANDLE_MSGS_PROC(
      "PlanningRefLineProc", PlanningComponent, PlanningRefLineProc,
      hobot::dataflow::ProcType::DF_MSG_TIMER_PROC, DF_VECTOR(),
      DF_VECTOR("PlanningOutputRefLine"));
  DF_MODULE_REGISTER_HANDLE_MSGS_PROC(
      "HeartbeatProc", PlanningComponent, HeartbeatProc,
      hobot::dataflow::ProcType::DF_MSG_TIMER_PROC, DF_VECTOR(),
      DF_VECTOR("Heartbeat"));
  DF_MODULE_REGISTER_HANDLE_MSGS_PROC(
      "NoaDecisionMeasureProc", PlanningComponent, NoaDecisionMeasureProc,
      hobot::dataflow::ProcType::DF_MSG_TIMER_PROC, DF_VECTOR(),
      DF_VECTOR("NoaDecisionMeasureOutput"));
}

int32_t PlanningComponent::Init() {
  // read config file
  std::string config_file("apps/planning/config/json/planning_conf.json");
  // TODO setplningconfig
  zark::planning::Config config{config_file};
  config.SetPlanningConfig(config_);
  injector_ = std::make_shared<zark::planning::DependencyInjector>();
  planning_base_ = std::make_unique<zark::planning::OnLanePlanning>(injector_);
  planning_base_->Init(config_);
  zark::planning::DecisionGflags::setDecisionConfig(
      zark::planning::PlanningGflags::decision_path);
  InitLocalView();
  DFHLOG_I("Planning Init Success.");
  return 0;
}

int32_t PlanningComponent::Start() { return hobot::dataflow::Module::Start(); }

int32_t PlanningComponent::Stop() { return hobot::dataflow::Module::Stop(); }

void PlanningComponent::Reset() { hobot::dataflow::Module::Reset(); }

int32_t PlanningComponent::DeInit() {
  return hobot::dataflow::Module::DeInit();
}

void PlanningComponent::PredictionCallback(
    hobot::dataflow::spMsgResourceProc proc,
    const hobot::dataflow::MessageLists &msgs) {
  DFHLOG_I("Prediction callback...");

  auto &input_pre_msgs = msgs[proc->GetResultIndex("PredictionObstacles")];
  for (auto &msg : *(input_pre_msgs.get())) {
    auto act_msg = std::dynamic_pointer_cast<
        zark::prediction::proto::PredictionObjectsMsg>(msg);
    if (nullptr == act_msg) {
      continue;
    }
    std::lock_guard<std::mutex> lock(local_view_lock_);
    prediction_obstacles_ = act_msg->proto;
    local_view_.prediction_obstacles =
        std::make_shared<zark::prediction::proto::PredictionObjects>(
            prediction_obstacles_);
  }
}

void PlanningComponent::LocalizationCallback(
    hobot::dataflow::spMsgResourceProc proc,
    const hobot::dataflow::MessageLists &msgs) {
  auto &input_loc_msgs = msgs[proc->GetResultIndex("localization")];
  if (input_loc_msgs->empty()) {
    return;
  }

  auto act_msg =
      std::dynamic_pointer_cast<zark::localization::LocalizationInfoMsg>(
          input_loc_msgs->back());
  if (act_msg != nullptr) {
    std::lock_guard<std::mutex> lock(local_view_lock_);
    local_view_.localization_estimate =
        std::make_shared<zark::localization::LocalizationInfo>(act_msg->proto);
    zark::planning::DataReader::GetInstance()->SetLocalizationData(
        act_msg->proto);
  }
}

void PlanningComponent::ChassisCallback(
    hobot::dataflow::spMsgResourceProc proc,
    const hobot::dataflow::MessageLists &msgs) {
  UNUSED(proc);
  UNUSED(msgs);
}

void PlanningComponent::CanDataCallback(
    hobot::dataflow::spMsgResourceProc proc,
    const hobot::dataflow::MessageLists &msgs) {
  auto &input_msgs = msgs[proc->GetResultIndex("adptrin_outputs")];
  if (input_msgs->empty()) {
    return;
  }

  auto act_msg =
      std::dynamic_pointer_cast<zark::CanDataMsg>(input_msgs->back());
  if (act_msg != nullptr) {
    std::lock_guard<std::mutex> lock(local_view_lock_);
    local_view_.can_data =
        std::make_shared<zark::ads_adptrin::ADAS_Inputs>(act_msg->proto);
  }
}

void PlanningComponent::MapCallback(hobot::dataflow::spMsgResourceProc proc,
                                    const hobot::dataflow::MessageLists &msgs) {
  auto &input_map_msgs = msgs[proc->GetResultIndex("local_map")];
  if (input_map_msgs->empty()) {
    return;
  }

  auto act_msg = std::dynamic_pointer_cast<zark::hdmap_new::ZarkMapMsg>(
      input_map_msgs->back());
  if (act_msg != nullptr) {
    double start_t = zark::common::Clock::NowInSeconds();
    zark::planning::hdmap::HDMapUtil::MapServicePtr(false)
        ->SetAndMarkHasNewData(act_msg->proto);
    double update_end_t = zark::common::Clock::NowInSeconds();
    AERROR << "MapCallback  mapservice update new map msg over, time diff ms: "
           << (update_end_t - start_t) * 1000.0;

    zark::planning::MapInfo map_info;
    const zark::hdmap_new::ZarkMap &map_msg = act_msg->proto;
    map_info.map_path.CopyFrom(map_msg.path());
    if (map_msg.has_header()) {
      map_info.map_timestamp = map_msg.header().timestamp_nano();
    }
    if (map_msg.has_guidance()) {
      map_info.guidance_info.CopyFrom(map_msg.guidance());
    }
    if (map_msg.has_map()) {
      if (map_msg.map().has_map_header()) {
        map_info.map_type = map_msg.map().map_header().type();
      }
      if (map_msg.map().has_hdmap()) {
        map_info.map_lane_size = map_msg.map().hdmap().lane_size();
      }
    }
    if (map_msg.has_navigation_info()) {
      map_info.navigation_info.CopyFrom(map_msg.navigation_info());
    }
    zark::planning::DataReader::GetInstance()->SetMapData(map_info);
    double call_end_t = zark::common::Clock::NowInSeconds();
    AERROR << "MapCallback process new map msg over, time diff ms: "
           << (call_end_t - start_t) * 1000.0;
  }
}

void PlanningComponent::MapLocalizationCallback(
    hobot::dataflow::spMsgResourceProc proc,
    const hobot::dataflow::MessageLists &msgs) {
  auto &input_msg = msgs[proc->GetResultIndex("map_localization_state")];
  if (input_msg->empty()) {
    return;
  }

  auto act_msg =
      std::dynamic_pointer_cast<zark::mapfusion::MapLocalizationInfoMsg>(
          input_msg->back());
  if (act_msg != nullptr) {
    zark::planning::DataReader::GetInstance()->SetMapState(act_msg->proto);
  }
}

void PlanningComponent::FctDataCallback(
    hobot::dataflow::spMsgResourceProc proc,
    const hobot::dataflow::MessageLists &msgs) {
  auto &input_pre_msgs = msgs[proc->GetResultIndex("fct_outputs")];
  if (input_pre_msgs->empty()) {
    return;
  }

  auto act_msg = std::dynamic_pointer_cast<zark::ads_fct::FCT_OutputsMsg>(
      input_pre_msgs->back());
  if (act_msg != nullptr) {
    std::lock_guard<std::mutex> lock(local_view_lock_);
    local_view_.fct_output =
        std::make_shared<zark::ads_fct::FCT_Outputs>(act_msg->proto);
  }
}

void PlanningComponent::PlanningProc(
    hobot::dataflow::spMsgResourceProc proc,
    const hobot::dataflow::MessageLists &msgs) {
  DFHLOG_I("PlanningProc callback ...");

  const auto start_time = zark::common::Clock::NowInSeconds();

  std::shared_ptr<zark::planning::PlanningOutputMsgMsg> msg =
      std::make_shared<zark::planning::PlanningOutputMsgMsg>();
  msg->SetGenTimestamp(static_cast<int64_t>(start_time * 1000));

  // TODO(qj)
  // set_local_view();
  zark::planning::ADCTrajectory adc_trajectory_pb;
  // use temporary local_view to do multithread guard
  zark::planning::LocalView local_view_tmp;
  {
    std::lock_guard<std::mutex> lock(local_view_lock_);
    local_view_tmp = local_view_;
  }
  planning_base_->RunOnce(local_view_tmp, adc_trajectory_pb);
  zark::planning::PlanningOutputMsg &output = msg->proto;
  zark::planning::common::ProtoConvertor::RecordPlanningOutput(
      adc_trajectory_pb, output);

  output.mutable_header()->mutable_common_header()->set_sequence_num(
      sequence_num_++);
  // tmp add loc time stamp
  if (config_.tranform_traj_to_body_frame()) {
    output.mutable_header()->set_radar_timestamp(
        local_view_tmp.localization_estimate->header().timestamp_nano());
  }

  const auto end_time = zark::common::Clock::NowInSeconds();
  msg->SetDoneTimestamp(static_cast<int64_t>(end_time * 1000));
  double total_time = (end_time - start_time) * 1000;
  AINFO << "planning seq_num:" << sequence_num_ - 1;
  {
    std::lock_guard<std::mutex> lock(debug_info_lock_);
    debug_info_.CopyFrom(adc_trajectory_pb.planning_debug_info());
    auto *task_stats = debug_info_.add_task_latencies();
    task_stats->set_task_name("PLANNING_TOTAL");
    task_stats->set_latency(total_time);
  }
  auto output_port = proc->GetOutputPort("PlanningOutputMsg");
  if (output_port) {
    output_port->Send(msg);
    SendNoaDecisionResult(proc);
  }

  {
    std::lock_guard<std::mutex> lock(path_ref_line_lock_);
    ref_lines_.Clear();
    if (injector_->frame_history()->Latest() != nullptr) {
      zark::planning::common::ProtoConvertor::RecordLocalRoute(
          injector_->frame_history()->Latest()->LocalRoutes(), ref_lines_);
    } else {
      AWARN << "Frame history is emptry";
    }
  }
}

void PlanningComponent::PlanningDebugProc(
    hobot::dataflow::spMsgResourceProc proc,
    const hobot::dataflow::MessageLists &msgs) {
  zark::planning::PlanningDebugInfo out_debug_info;
  {
    std::lock_guard<std::mutex> lock(debug_info_lock_);
    out_debug_info.CopyFrom(debug_info_);
  }
  // TODO(qijun) timestamp_nano
  out_debug_info.mutable_header()->mutable_common_header()->set_timestamp_nano(
      local_view_.localization_estimate->header().timestamp_nano());
  auto msg = std::make_shared<zark::planning::PlanningDebugInfoMsg>();
  msg->proto = out_debug_info;
  int64_t ts = std::chrono::duration_cast<std::chrono::milliseconds>(
                   std::chrono::system_clock::now().time_since_epoch())
                   .count();
  msg->SetGenTimestamp(ts);
  msg->SetDoneTimestamp(ts);
  auto output_port = proc->GetOutputPort("PlanningDebugInfo");
  if (output_port) {
    output_port->Send(msg);
  }
}

void PlanningComponent::PlanningRefLineProc(
    hobot::dataflow::spMsgResourceProc proc,
    const hobot::dataflow::MessageLists &msgs) {
  zark::planning::PathAndReferenceLine out_lines;
  {
    std::lock_guard<std::mutex> lock(path_ref_line_lock_);
    if (ref_lines_.reference_lines_size() <= 0) {
      AWARN << "ref_lines is empty";
    }
    out_lines.mutable_reference_lines()->CopyFrom(ref_lines_);
  }
  // TODO(qijun) timestamp_nano
  out_lines.mutable_reference_lines()->mutable_refline_state()->CopyFrom(
      zark::planning::DataReader::GetInstance()->GetRefLineState());
  out_lines.mutable_header()->mutable_common_header()->set_timestamp_nano(
      local_view_.localization_estimate->header().timestamp_nano());
  std::shared_ptr<zark::planning::PathAndReferenceLineMsg> msg =
      std::make_shared<zark::planning::PathAndReferenceLineMsg>();
  msg->proto = out_lines;
  int64_t ts = std::chrono::duration_cast<std::chrono::milliseconds>(
                   std::chrono::system_clock::now().time_since_epoch())
                   .count();
  msg->SetGenTimestamp(ts);
  msg->SetDoneTimestamp(ts);
  auto output_port = proc->GetOutputPort("PlanningOutputRefLine");
  if (output_port) {
    output_port->Send(msg);
  }
}

void PlanningComponent::HeartbeatProc(
    hobot::dataflow::spMsgResourceProc proc,
    const hobot::dataflow::MessageLists &msgs) {
  std::shared_ptr<zark::heartbeat::HeartBeatMsg> msg =
      std::make_shared<zark::heartbeat::HeartBeatMsg>();
  zark::heartbeat::HeartBeat &output = msg->proto;
  output.set_module_name("planning");
  output.set_module_status(zark::heartbeat::MODULE_STATUS::NORMAL);
  int64_t ts = std::chrono::duration_cast<std::chrono::milliseconds>(
                   std::chrono::system_clock::now().time_since_epoch())
                   .count();
  msg->SetGenTimestamp(ts);
  msg->SetDoneTimestamp(ts);
  auto output_port = proc->GetOutputPort("Heartbeat");
  if (output_port) {
    output_port->Send(msg);
  }
}

void PlanningComponent::SendNoaDecisionResult(
    const hobot::dataflow::spMsgResourceProc &proc) {
  static uint32_t cnt = 0;
  static double decision_start_t = zark::common::Clock::NowInSeconds();
  if (cnt < 15) {
    cnt++;
  } else {
    cnt = 0;
  }
  std::shared_ptr<zark::ads_decision::DEC_OutputsMSG> msg =
      std::make_shared<zark::ads_decision::DEC_OutputsMSG>();
  zark::ads_decision::DEC_Outputs &out_put_result = msg->proto;
  out_put_result.CopyFrom(
      zark::planning::DataReader::GetInstance()->GetForceLCResult());
  auto ref_line_result =
      zark::planning::DataReader::GetInstance()->GetFittedResult();
  zark::planning::CombineDecOutputsForPnC(ref_line_result, out_put_result);
  out_put_result.set_dec_out_decrollingcounter_u8(cnt);
  int64_t ts = std::chrono::duration_cast<std::chrono::milliseconds>(
                   std::chrono::system_clock::now().time_since_epoch())
                   .count();
  int64_t nano_stamp = std::chrono::duration_cast<std::chrono::nanoseconds>(
                           std::chrono::system_clock::now().time_since_epoch())
                           .count();
  out_put_result.set_dec_out_t_dectimestamp_u64(nano_stamp);
  msg->SetGenTimestamp(ts);
  msg->SetDoneTimestamp(ts);
  zark::planning::EmptyMsgCheck(out_put_result);
  double decision_end_t = zark::common::Clock::NowInSeconds();
  double decision_time_diff_ms = (decision_end_t - decision_start_t) * 1000.0;
  out_put_result.mutable_dec_out_lftroadedge_bus()
      ->set_da_in_d_roadedgerangend_sg(decision_time_diff_ms);
  decision_start_t = decision_end_t;
  auto output_port = proc->GetOutputPort("NoaDecisionOutput");
  if (output_port) {
    if (output_port->Send(msg) < 0) {
      AERROR << "send decision msg error.";
    }
  }
}

void PlanningComponent::NoaDecisionMeasureProc(
    hobot::dataflow::spMsgResourceProc proc,
    const hobot::dataflow::MessageLists &msgs) {
  UNUSED(proc);
  UNUSED(msgs);

  std::shared_ptr<zark::ads_decision::DecisionMeasInfoMSG> msg =
      std::make_shared<zark::ads_decision::DecisionMeasInfoMSG>();
  zark::ads_decision::DecisionMeasInfo &out_measinfo = msg->proto;
  const zark::planning::NavigationInfo refline_navi_info =
      zark::planning::DataReader::GetInstance()->GetNaviResult();
  const zark::planning::ForceLcMeasureInfo force_measure =
      zark::planning::DataReader::GetInstance()->GetMeasureInfo();
  zark::planning::TransformMeas2Proto(&refline_navi_info, force_measure,
                                      out_measinfo);
  int64_t ts = std::chrono::duration_cast<std::chrono::milliseconds>(
                   std::chrono::system_clock::now().time_since_epoch())
                   .count();
  int64_t nano_stamp = std::chrono::duration_cast<std::chrono::nanoseconds>(
                           std::chrono::system_clock::now().time_since_epoch())
                           .count();
  out_measinfo.mutable_header()->set_timestamp_nano(nano_stamp);
  msg->SetGenTimestamp(ts);
  msg->SetDoneTimestamp(ts);
  auto output_port = proc->GetOutputPort("NoaDecisionMeasureOutput");
  if (output_port) {
    output_port->Send(msg);
  }
}

void PlanningComponent::InitLocalView() {
  std::lock_guard<std::mutex> lock(local_view_lock_);
  local_view_.prediction_obstacles =
      std::make_shared<zark::prediction::proto::PredictionObjects>();
  local_view_.localization_estimate =
      std::make_shared<zark::localization::LocalizationInfo>();
  local_view_.can_data = std::make_shared<zark::ads_adptrin::ADAS_Inputs>();
  local_view_.fct_output = std::make_shared<zark::ads_fct::FCT_Outputs>();
  local_view_.dec_output = std::make_shared<zark::ads_decision::DEC_Outputs>();
}

DATAFLOW_REGISTER_MODULE(PlanningComponent)
