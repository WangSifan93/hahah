/******************************************************************************
 * Copyright 2023 The zpilot Author Fuhuanhuan. All Rights Reserved.
 *****************************************************************************/

#include "apps/planning/src/on_lane_planning.h"

#include <algorithm>
#include <limits>
#include <list>
#include <utility>

#include "apps/planning/src/common/base/macros.h"
#include "apps/planning/src/common/clock.h"
#include "apps/planning/src/common/configs/vehicle_config_helper.h"
#include "apps/planning/src/common/conversion.h"
#include "apps/planning/src/common/log.h"
#include "apps/planning/src/common/proto_convertor/proto_convertor.h"
#include "apps/planning/src/common/util/util.h"
#include "apps/planning/src/common/vehicle_state/vehicle_state_provider.h"
#include "apps/planning/src/config/conf_gflags.h"
#include "apps/planning/src/config/config_main.h"
#include "apps/planning/src/tasks/task_factory.h"
#include "gtest/gtest_prod.h"
#include "messages/planning/ads_com.pb.h"
#include "quaternion.h"
#include "string_util.h"

namespace zark {
namespace planning {

using namespace ::math;
using namespace zark::common;

constexpr int kPolyFitOrder = 5;
constexpr double kMinEpsilon = 1e-3;

OnLanePlanning::~OnLanePlanning() {
  if (local_route_provider_) {
    local_route_provider_->Stop();
  }
  planner_->Stop();
  injector_->frame_history()->Clear();
}

std::string OnLanePlanning::Name() const { return "on_lane_planning"; }

Status OnLanePlanning::Init(const PlanningConfig& config) {
  config_ = config;
  if (!CheckPlanningConfig(config_)) {
    return ::common::Status(::common::ErrorCode::PLANNING_ERROR,
                            "planning config error: " + config_.DebugString());
  }

  PlanningBase::Init(config_);
  planner_dispatcher_->Init();

  // instantiate local route provider
  local_route_provider_ = std::make_unique<LocalRouteProvider>();
  local_route_provider_->Start();
  noa_decision_ = std::make_unique<ForceLaneChangeDecision>();

  // init scenario
  scenario_manager_.Init(config);

  // dispatch planner
  planner_ = planner_dispatcher_->DispatchPlanner(config_, injector_);
  if (!planner_) {
    return ::common::Status(
        ::common::ErrorCode::PLANNING_ERROR,
        "planning is not initialized with config : " + config_.DebugString());
  }
  start_time_ = Clock::NowInSeconds();

  return planner_->Init(config_);
}

void OnLanePlanning::RunOnce(const LocalView& local_view,
                             ADCTrajectory& trajectory_pb) {
  local_view_ = local_view;
  const double start_system_timestamp = zark::common::Clock::NowInSeconds();

  // noa_decision every cycle need run
  NoaDecisionProcess();
  // V1 planning just need run noa_decision
  return;

  UpdateAutoMode();

  // TODO need merge two status function
  ::common::Status status = injector_->vehicle_state()->Update(
      *local_view_.localization_estimate, *local_view_.can_data);
  trajectory_pb.set_planning_status(PlanningStatus::INIT);

  const common::VehicleState vehicle_state =
      injector_->vehicle_state()->vehicle_state();
  localitzation_timestamp_ = vehicle_state.nano_timestamp();
  const double start_timestamp = kNanoToDecimal(localitzation_timestamp_);

  AINFO << "get vehicle state: x = " << vehicle_state.x()
        << " y = " << vehicle_state.y()
        << " velocity = " << vehicle_state.linear_velocity()
        << " heading = " << vehicle_state.heading()
        << " acceleration = " << vehicle_state.linear_acceleration()
        << " timestamp = " << vehicle_state.timestamp();

  if (!status.ok() || !util::IsVehicleStateValid(vehicle_state)) {
    const std::string msg =
        "Update VehicleStateProvider failed "
        "or the vehicle state is out dated.";
    AERROR << msg;
    status.Save(trajectory_pb.mutable_header()->mutable_status());
    return;
  }

  if (auto_mode_ != AutoMode::AUTO_LON) {
    // early return when reference line fails to update after rerouting
    if (!local_route_provider_->IsLocalRouteUpdated()) {
      const std::string msg =
          "Failed to update reference line after rerouting.";
      AERROR << msg;
      status.Save(trajectory_pb.mutable_header()->mutable_status());
      injector_->frame_history()->Clear();
      return;
    }
  }

  auto GetDtPrev = [&]() {
    double dt_prev = 1.0 / PlanningGflags::planning_loop_rate;
    if (injector_->frame_history() != nullptr &&
        injector_->frame_history()->Latest() != nullptr) {
      const double prev_timestamp = injector_->frame_history()
                                        ->Latest()
                                        ->current_frame_planned_trajectory()
                                        .header()
                                        .timestamp_sec();
      if (start_timestamp - prev_timestamp > kMinEpsilon) {
        dt_prev = start_timestamp - prev_timestamp;
      }
    }
    const double kMinDeltaT = 6e-2;
    return std::max(dt_prev, kMinDeltaT);
  };
  const double dt_prev = GetDtPrev();

  std::string replan_reason;
  const bool need_replan = CheckNeedReplan(replan_reason);
  const bool need_lat_replan = CheckNeedLatReplan(replan_reason);
  if (need_replan) {
    injector_->frame_history()->Clear();
  }
  const common::VehicleParam& vehicle_param =
      common::VehicleConfigHelper::GetConfig().vehicle_param();
  ::common::TrajectoryPoint planning_start_point;
  if (need_replan) {
    planning_start_point = ComputeTrajectoryPointFromVehicleState(
        dt_prev, vehicle_state, vehicle_param);
  } else if (need_lat_replan) {
    planning_start_point = ComputePlanningStartPointForACC(
        dt_prev, *last_publishable_trajectory_, vehicle_state, vehicle_param);
  } else {
    planning_start_point = ComputePlanningStartPoint(
        dt_prev, *last_publishable_trajectory_, vehicle_state);
  }
  common::ConvertFromRearAxleToCG(vehicle_param.rear_axle_to_cg(),
                                  planning_start_point);
  const uint32_t frame_num = static_cast<uint32_t>(seq_num_++);
  status = InitFrame(frame_num, planning_start_point, vehicle_state);
  frame_->SetIsReplan(need_replan);
  frame_->SetIsLatReplan(need_lat_replan);

  if (!status.ok()) {
    AERROR << status.ToString();
    trajectory_pb.set_planning_status(PlanningStatus::ABNORMAL);
    trajectory_pb.clear_trajectory_point();
    frame_->set_current_frame_planned_trajectory(trajectory_pb);
    const uint32_t n = frame_->SequenceNum();
    injector_->frame_history()->Add(n, std::move(frame_));
    return;
  }

  status =
      Plan(start_timestamp, planning_start_point, trajectory_pb, need_replan);

  AINFO << "total planning time spend: "
        << (zark::common::Clock::NowInSeconds() - start_system_timestamp) * 1000
        << " ms.";

  if (!status.ok()) {
    status.Save(trajectory_pb.mutable_header()->mutable_status());
    AERROR << "Planning failed:" << status.ToString();
    trajectory_pb.set_planning_status(PlanningStatus::ABNORMAL);
    trajectory_pb.clear_trajectory_point();
  } else {
    trajectory_pb.set_planning_status(PlanningStatus::NORMAL);
  }
  trajectory_pb.set_is_replan(need_replan);
  auto* ref_line_task = frame_->MutableCurrentFramePlannedTrajectory()
                            ->mutable_latency_stats()
                            ->add_task_stats();
  ref_line_task->set_time_ms(local_route_provider_->LastTimeDelay() * 1000.0);
  ref_line_task->set_name("LOCAL_ROUTE_PROVIDER");

  // traj polynomial fit
  const DiscretizedTrajectory traj_interp(trajectory_pb.trajectory_point());
  zark::planning::PolynomialFit::TrajPolynomial poly_fit_out =
      poly_fit_.TrajPointsPolyFit(traj_interp, vehicle_state);
  trajectory_pb.set_traj_polyfit_out(poly_fit_out);

  if (frame_ != nullptr) {
    const CorridorInfo* corridor_info_prev = nullptr;
    if (injector_->frame_history() != nullptr &&
        injector_->frame_history()->Latest() != nullptr &&
        !injector_->frame_history()->Latest()->GetProposals().empty()) {
      corridor_info_prev = injector_->frame_history()
                               ->Latest()
                               ->GetProposals()
                               .front()
                               .GetCorridorInfo();
    }
    common::ProtoConvertor::RecordPlanningDebugInfo(
        frame_->GetCorridorInfos(), frame_->GetProposals(), corridor_info_prev,
        frame_->GetMission(),
        frame_->current_frame_planned_trajectory().latency_stats(),
        need_replan || need_lat_replan, replan_reason,
        *trajectory_pb.mutable_planning_debug_info());
  }
  DataReader::GetInstance()->SetGapChoiceResult(frame_->GetLCGap());

  frame_->set_current_frame_planned_trajectory(trajectory_pb);
  const uint32_t n = frame_->SequenceNum();
  injector_->frame_history()->Add(n, std::move(frame_));

  // trans traj_point to ego coordinate accord to config
  if (config_.tranform_traj_to_body_frame()) {
    AERROR << " Transform  trajectory point to ego coordinate!!!";

    DiscretizedTrajectory trans_traj =
        ConvertTrajectoryToBodyFrame(traj_interp, vehicle_state);
    trajectory_pb.set_trajectory_point(trans_traj);
  }
}

Status OnLanePlanning::Plan(const double current_time_stamp,
                            const TrajectoryPoint& planning_start_point,
                            ADCTrajectory& trajectory_pb,
                            const bool need_replan) {
  // sainty check
  if (!frame_) {
    trajectory_pb.set_planning_status(PlanningStatus::ABNORMAL);
    const std::string msg = "frame is nullptr!";
    return ::common::Status(::common::ErrorCode::PLANNING_ERROR, msg);
  }

  scenario_manager_.Update(planning_start_point, *frame_.get());
  auto scenario = scenario_manager_.mutable_scenario();

  for (auto& local_route : frame_->MutableLocalRoutes()) {
    local_route.SetInitFrenetState(
        local_route.ToFrenetFrame(planning_start_point));
  }

  auto status = planner_->Plan(planning_start_point, frame_.get(), scenario,
                               trajectory_pb);

  if (frame_ != nullptr) {
    SetEgoVehicleParam(trajectory_pb);
    if (frame_->GetProposals().size() > 0) {
      auto TransformTrajectoryFromCGToRearAxle =
          [](const DiscretizedTrajectory& traj_in) {
            const auto& vehicle_param =
                common::VehicleConfigHelper::GetConfig().vehicle_param();
            auto traj_out = traj_in;
            for (auto& pt : traj_out) {
              common::ConvertFromCGToRearAxle(vehicle_param.rear_axle_to_cg(),
                                              pt);
            }
            return traj_out;
          };

      last_publishable_trajectory_.reset(new PublishableTrajectory(
          current_time_stamp,
          TransformTrajectoryFromCGToRearAxle(
              frame_->GetProposals().front().GetTrajectory())));

      last_publishable_trajectory_->PopulateTrajectoryProtobuf(trajectory_pb);

      // hotfix for num of trajectory points
      auto traj_temp = trajectory_pb.trajectory_point();
      if (traj_temp.size() > 0 && traj_temp.size() < 41) {
        auto traj_sup = std::vector<::common::TrajectoryPoint>(
            41 - traj_temp.size(), traj_temp.back());
        traj_temp.insert(traj_temp.end(), traj_sup.begin(), traj_sup.end());
      }
      trajectory_pb.set_trajectory_point(traj_temp);

      trajectory_pb.set_planning_status(PlanningStatus::READY);
    } else {
      const std::string msg = "planner failed to make a driving plan";
      ZSLOG_ERROR << msg;
      if (last_publishable_trajectory_) {
        last_publishable_trajectory_->Clear();
      }
      trajectory_pb.set_planning_status(PlanningStatus::ABNORMAL);
      return ::common::Status(::common::ErrorCode::PLANNING_ERROR, msg);
    }
  } else {
    const std::string msg = "planner failed to make a driving plan";
    ZSLOG_ERROR << msg;
    if (last_publishable_trajectory_) {
      last_publishable_trajectory_->Clear();
    }
    trajectory_pb.set_planning_status(PlanningStatus::ABNORMAL);
    return ::common::Status(::common::ErrorCode::PLANNING_ERROR, msg);
  }

  return status;
}

Status OnLanePlanning::InitFrame(const uint32_t sequence_num,
                                 const TrajectoryPoint& planning_start_point,
                                 const common::VehicleState& vehicle_state) {
  frame_.reset(new Frame(sequence_num, local_view_, planning_start_point,
                         vehicle_state));

  if (frame_ == nullptr) {
    return ::common::Status(::common::ErrorCode::PLANNING_ERROR,
                            "Fail to init frame: nullptr.");
  }

  frame_->SetNoaDecisionMeas(noa_decision_->GetForceLcMeasureInfo());
  frame_->SetFittedLaneLine(refline_fitted_result_);

  frame_->SetAutoMode(auto_mode_);
  frame_->SetIsInIESMode(is_in_ies_mode_);

  if (!local_route_provider_->GetLocalRoutes(planning_start_point,
                                             auto_mode_ == AutoMode::AUTO_LON,
                                             frame_->MutableLocalRoutes())) {
    const std::string msg = "Failed to create reference line.";
    AERROR << msg;
    return ::common::Status(::common::ErrorCode::PLANNING_ERROR, msg);
  }

  //  TODO: report take-over in HMI when current local route is too short
  auto IsLocalRoutesValid = [this](const TrajectoryPoint& planning_start_point,
                                   const double threshhold) {
    LocalRoute* local_route = frame_->FindCurrentLocalRoute();
    if (local_route != nullptr &&
        local_route->GetEndS() -
                local_route->ToFrenetFrame(planning_start_point).s[kIdxS] <
            threshhold) {
      return false;
    }
    return true;
  };

  if (!IsLocalRoutesValid(planning_start_point, kMinEpsilon)) {
    const std::string msg =
        "The local route is behind the planning start point.";
    AERROR << msg;
    return ::common::Status(::common::ErrorCode::PLANNING_ERROR, msg);
  } else if (!IsInAutoMode() &&
             !IsLocalRoutesValid(planning_start_point,
                                 planning_start_point.v() *
                                     PlanningGflags::trajectory_time_length)) {
    const std::string msg = "The local route is too short for planning.";
    AERROR << msg;
    return ::common::Status(::common::ErrorCode::PLANNING_ERROR, msg);
  }

  auto IsHeadingValid = [this](const TrajectoryPoint& planning_start_point) {
    const double s_init = this->frame_->LocalRoutes()
                              .front()
                              .ToFrenetFrame(planning_start_point)
                              .s[kIdxS];
    const double theta_ref =
        frame_->LocalRoutes().front().GetLocalRoutePoint(s_init).heading();
    const double cos_theta_delta =
        std::cos(theta_ref - planning_start_point.path_point().theta());
    const double kMinCosAngle = 0.5;
    return cos_theta_delta > kMinCosAngle;
  };
  if (!IsInAutoMode() && !IsHeadingValid(planning_start_point)) {
    const std::string msg =
        "The angle between the vehicle and the local route is greater than 60 "
        "degrees";
    AERROR << msg;
    return ::common::Status(::common::ErrorCode::PLANNING_ERROR, msg);
  }

  std::vector<routing::LaneWaypoint> future_route_waypoints;

  AINFO << "Eric InitFrame get obs size: "
        << local_view_.prediction_obstacles->prediction_object_size();

  auto status =
      frame_->Init(injector_->vehicle_state(), future_route_waypoints);

  if (!status.ok()) {
    AERROR << "failed to init frame:" << status.ToString();
    return status;
  }
  return Status::OK();
}

bool OnLanePlanning::CheckPlanningConfig(const PlanningConfig& config) {
  if (!config.has_standard_planning_config()) {
    return false;
  }
  if (!config.standard_planning_config().has_planner_public_road_config()) {
    return false;
  }
  // TODO(All): check other config params
  return true;
}

void OnLanePlanning::SetEgoVehicleParam(ADCTrajectory& trajectory) {
  const common::VehicleParam& vehicle_param =
      common::VehicleConfigHelper::GetConfig().vehicle_param();
  trajectory.mutable_planning_debug_info()
      ->mutable_ego_vehicle_param()
      ->set_length(vehicle_param.length());
  trajectory.mutable_planning_debug_info()
      ->mutable_ego_vehicle_param()
      ->set_width(vehicle_param.width());
  trajectory.mutable_planning_debug_info()
      ->mutable_ego_vehicle_param()
      ->set_back_edge_to_center(vehicle_param.back_edge_to_center());
  trajectory.mutable_planning_debug_info()
      ->mutable_ego_vehicle_param()
      ->set_back_edge_to_cg(vehicle_param.back_edge_to_center() +
                            vehicle_param.rear_axle_to_cg());
}

::common::TrajectoryPoint
OnLanePlanning::ComputeTrajectoryPointFromVehicleState(
    const double& dt_prev, const common::VehicleState& vehicle_state,
    const common::VehicleParam& vehicle_param) const {
  TrajectoryPoint planning_start_point;
  planning_start_point.mutable_path_point()->set_x(vehicle_state.x());
  planning_start_point.mutable_path_point()->set_y(vehicle_state.y());
  planning_start_point.mutable_path_point()->set_z(vehicle_state.z());
  planning_start_point.mutable_path_point()->set_theta(vehicle_state.heading());
  planning_start_point.mutable_path_point()->set_kappa(vehicle_state.kappa());
  planning_start_point.mutable_path_point()->set_s(0.0);
  planning_start_point.mutable_path_point()->set_dkappa(0.0);   // TODO
  planning_start_point.mutable_path_point()->set_ddkappa(0.0);  // TODO

  planning_start_point.set_v(vehicle_state.linear_velocity());
  planning_start_point.set_a(vehicle_state.linear_acceleration());
  planning_start_point.set_dt_prev(dt_prev);
  planning_start_point.set_relative_time(0.0);
  planning_start_point.set_da(vehicle_state.front_wheel_angle_rate());
  planning_start_point.set_steer(vehicle_state.front_wheel_angle());
  planning_start_point.set_steer_rate(vehicle_state.front_wheel_angle_rate());
  planning_start_point.set_omega(vehicle_state.angular_velocity());
  planning_start_point.set_v_y(vehicle_state.angular_velocity() *
                               vehicle_param.rear_axle_to_cg());

  return planning_start_point;
}

::common::TrajectoryPoint OnLanePlanning::ComputePlanningStartPointForACC(
    const double dt_prev, const PublishableTrajectory& traj_prev,
    const common::VehicleState& vehicle_state,
    const common::VehicleParam& vehicle_param) const {
  TrajectoryPoint planning_start_point = ComputeTrajectoryPointFromVehicleState(
      dt_prev, vehicle_state, vehicle_param);
  const Vec2d pt(vehicle_state.x(), vehicle_state.y());
  const double s =
      ComputePositionProjection(pt.x(), pt.y(),
                                traj_prev[traj_prev.QueryNearestPoint(pt)])
          .first;

  planning_start_point.set_v(traj_prev.EvaluateByS(s).v());
  planning_start_point.set_a(traj_prev.EvaluateByT(dt_prev, true).a());

  return planning_start_point;
}

::common::TrajectoryPoint OnLanePlanning::ComputePlanningStartPoint(
    const double dt_prev, const PublishableTrajectory& traj_prev,
    const common::VehicleState& vehicle_state) const {
  // lat: projected from vehicle_state
  const Vec2d pt(vehicle_state.x(), vehicle_state.y());
  const double s =
      ComputePositionProjection(pt.x(), pt.y(),
                                traj_prev[traj_prev.QueryNearestPoint(pt)])
          .first;
  TrajectoryPoint planning_start_point = traj_prev.EvaluateByS(s);

  // lon: interpolated by time difference
  const ::common::TrajectoryPoint planning_start_point_lon =
      traj_prev.EvaluateByT(dt_prev, true);
  planning_start_point.set_relative_time(0.0);
  planning_start_point.mutable_path_point()->set_s(0.0);
  planning_start_point.set_a(planning_start_point_lon.a());
  planning_start_point.set_dt_prev(dt_prev);

  return planning_start_point;
}

bool OnLanePlanning::CheckNeedReplan(std::string& replan_reason) const {
  if ((last_publishable_trajectory_.get() == nullptr) ||
      (last_publishable_trajectory_->size() < 1)) {
    replan_reason = "last published trajectory is empty";
    return true;
  }

  if (std::isnan(last_publishable_trajectory_->GetSpatialLength())) {
    replan_reason = "last published trajectory len is nan";
    return true;
  }

  if (!(injector_->frame_history() != nullptr &&
        injector_->frame_history()->Latest() != nullptr &&
        injector_->frame_history()->Latest()->GetProposals().size() > 0)) {
    replan_reason = "frame history is empty";
    return true;
  }

  if (!IsInAutoMode()) {
    replan_reason = "not in AUTO state";
    return true;
  }

  if (auto_mode_prev_ == AutoMode::MANUAL && IsInAutoMode()) {
    replan_reason = "switch to AUTO state";
    return true;
  }

  return false;
}

bool OnLanePlanning::CheckNeedLatReplan(std::string& replan_reason) const {
  if (auto_mode_ != auto_mode_prev_) {
    replan_reason = "auto mode is swtiched";
    return true;
  }

  if (auto_mode_ == AutoMode::AUTO_LON) {
    replan_reason = "auto mode is ACC";
    return true;
  }

  return false;
}

void OnLanePlanning::UpdateAutoMode() {
  const uint8_t function_noa_state =
      static_cast<uint8_t>(local_view_.fct_output->fct_out_bus_nopinfosts()
                               .fct_out_sts_hnopsyssts_enopss());

  const uint8_t activated_function =
      static_cast<uint8_t>(local_view_.fct_output->fct_out_bus_lineinfo()
                               .fct_out_sts_activefunction_eaf());
  const uint8_t ies_state =
      static_cast<uint8_t>(local_view_.fct_output->fct_out_bus_iesinfosts()
                               .fct_out_sts_iesstate_eies());

  AINFO << "function_noa_state: " << static_cast<int>(function_noa_state);
  AINFO << "activated_function: " << static_cast<int>(activated_function);

  auto_mode_prev_ = auto_mode_;
  if (function_noa_state == zark::ads_common::eNOPSS_HOActv ||
      function_noa_state == zark::ads_common::eNOPSS_HFActv ||
      activated_function == zark::ads_common::PLT) {
    auto_mode_ = AutoMode::AUTO_FULL;
    AINFO << "The vehicle is in auto mode. ";
  } else if (activated_function == zark::ads_common::ACC) {
    auto_mode_ = AutoMode::AUTO_LON;
    AINFO << "The vehicle is in longitudinal mode. ";
  } else {
    auto_mode_ = AutoMode::MANUAL;
    AINFO << "The vehicle is in manual mode. ";
  }

  // check if IES is enabled
  if (ies_state == zark::ads_common::IES_ACTIVE_LFT_RE ||
      ies_state == zark::ads_common::IES_ACTIVE_LFT_OBJ ||
      ies_state == zark::ads_common::IES_ACTIVE_RHT_RE ||
      ies_state == zark::ads_common::IES_ACTIVE_RHT_OBJ) {
    is_in_ies_mode_ = true;
    AINFO << "IES is enabled. ";
  } else {
    is_in_ies_mode_ = false;
  }
}

void OnLanePlanning::NoaDecisionProcess() {
  refline_navigation_info_ = DataReader::GetInstance()->GetNaviResult();
  refline_fitted_result_ = DataReader::GetInstance()->GetFittedResult().second;
  noa_decision_->DeciderProc(refline_navigation_info_, refline_fitted_result_);
}

DiscretizedTrajectory OnLanePlanning::ConvertTrajectoryToBodyFrame(
    const DiscretizedTrajectory& traj,
    const common::VehicleState& vehicle_state) {
  DiscretizedTrajectory traj_output;
  traj_output.clear();

  if (traj.empty()) {
    AERROR << "No Traj Point !!!";
    return traj_output;
  }

  for (const auto& traj_p : traj) {
    auto traj_tmp = traj_p;
    double x = (traj_tmp.path_point().x() - vehicle_state.x()) *
                   std::cos(vehicle_state.heading()) +
               (traj_tmp.path_point().y() - vehicle_state.y()) *
                   std::sin(vehicle_state.heading());
    double y = (traj_tmp.path_point().y() - vehicle_state.y()) *
                   std::cos(vehicle_state.heading()) -
               (traj_tmp.path_point().x() - vehicle_state.x()) *
                   std::sin(vehicle_state.heading());
    double theta =
        NormalizeAngle(traj_tmp.path_point().theta() - vehicle_state.yaw());

    traj_tmp.mutable_path_point()->set_x(x);
    traj_tmp.mutable_path_point()->set_y(y);
    traj_tmp.mutable_path_point()->set_theta(theta);

    traj_output.emplace_back(traj_tmp);
  }

  return traj_output;
}

}  // namespace planning
}  // namespace zark
