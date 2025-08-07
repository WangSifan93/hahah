#include "nodes/planner/city/city_planner.h"

#include <memory>
#include <utility>

#include "absl/cleanup/cleanup.h"
#include "async/async_util.h"
#include "behavior.pb.h"
#include "common/gflags.h"
#include "common/planning_macros.h"
#include "common/timer.h"
#include "gflags/gflags.h"
#include "glog.pb.h"
#include "messages/planning/driving/e2e_planning_state.pb.h"
#include "messages/planning/driving/e2e_planning_traj.pb.h"
#include "nodes/planner/city/msg_proxy.h"
#include "plan/planner_flags.h"
#include "plan/planner_main_loop_internal.h"
#include "plan/planner_params_builder.h"
#include "plan/planner_util.h"
#include "plan/planner_world.h"
#include "plan/trajectory_util.h"
#include "speed_planning.pb.h"
#include "util/file_util.h"
#include "util/status_macros.h"

using ::google::protobuf::RepeatedPtrField;
namespace ad_e2e {
namespace planning {

constexpr double kDefaultVehicleLength = 7.0;
constexpr double kDefaultMainTargetS = 9999.0;

CityPlanner::CityPlanner(int num_workers)
    : thread_pool_(new e2e_noa::WorkerThreadManager(num_workers)),
      planner_params_(new e2e_noa::PlannerParamsProto()),
      vehicle_params_(new e2e_noa::VehicleParamsProto()),
      planner_state_(new e2e_noa::planning::PlannerState()),
      planning_result_msg_(new planning_result_type()) {}

bool CityPlanner::Init(const std::string& params_dir) {
  if (!InitParams(params_dir)) {
    return false;
  }

  planner_state_->previous_autonomy_state.set_autonomy_state(
      e2e_noa::AutonomyStateProto::NOT_READY);
  return true;
}

bool CityPlanner::InitParams(const std::string& params_dir) {
  if (!e2e_noa::file_util::FileToProto(
          params_dir + FLAGS_ad_e2e_vehicle_params_path,
          vehicle_params_.get())) {
    LOG(ERROR) << "failed to load vehicle params";
    return false;
  }

  LOG(INFO) << "loaded vehicle params from "
            << FLAGS_ad_e2e_vehicle_params_path;

  auto ret = e2e_noa::planning::BuildPlannerParams(
      params_dir, vehicle_params_->vehicle_geometry_params());
  if (!ret.ok()) {
    LOG(ERROR) << "load planner params failed:" << ret.status();
    return false;
  }
  *planner_params_ = ret.value();
  LOG(INFO) << "loaded planner params from " << params_dir;
  for (const auto& circle : planner_params_->vehicle_models_params()
                                .trajectory_optimizer_vehicle_model_params()
                                .circles()) {
    LOG(INFO) << "circle: dist_to_rac: " << circle.dist_to_rac()
              << ", angle_to_axis: " << circle.angle_to_axis()
              << ", radius: " << circle.radius() << ", name: " << circle.name();
  }

  return true;
}

void CityPlanner::Run(const PlanningInputFrame* f) {
  LOG(INFO) << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> CityPlanner Run!";

  const absl::Time current_time =
      e2e_noa::FromUnixDoubleSeconds(f->last_msg_timestamp);
  const absl::Time predicted_plan_time =
      current_time + absl::Milliseconds(FLAGS_planner_lookforward_time_ms);

  const std::unique_ptr<e2e_noa::planning::PlannerWorld> planner_world =
      AdaptPlannerWorld(f, *planner_params_, *vehicle_params_,
                        predicted_plan_time, thread_pool_.get(),
                        planner_state_.get());

  if (planner_world == nullptr) {
    if (planning_result_msg_) {
      auto& planning_res_msg = planning_result_msg_->first.proto;
      planning_res_msg.mutable_trajectory()->set_timestamp(
          f->last_msg_timestamp);
      if (ComputerFailCodeHoldCounter(planner_state_->planner_status_code)) {
        planning_res_msg.mutable_state()->set_result(
            zark::e2e_noa::Result::RESULT_OK);
        planning_res_msg.mutable_state()->set_fail_reason(0);
        planning_result_msg_->second.result = Result::RESULT_OK;
        planning_result_msg_->second.fail_reason = 0;
      } else {
        if (fail_number_ > 0) {
          --fail_number_;
          planning_res_msg.mutable_state()->set_result(
              zark::e2e_noa::Result::RESULT_OK);
        } else {
          planning_res_msg.mutable_state()->set_result(
              zark::e2e_noa::Result::RESULT_FAIL);
          planning_res_msg.mutable_state()->set_fail_reason(
              static_cast<uint32_t>(planner_state_->planner_status_code));
          planning_res_msg.mutable_trajectory()->clear_points();

          planning_result_msg_->second.result = Result::RESULT_FAIL;
          planning_result_msg_->second.fail_reason =
              static_cast<uint8_t>(planner_state_->planner_status_code);
          Log2FG::LogMissionData("Planner State Code",
                                 PlannerStatusProto_PlannerStatusCode_Name(
                                     planner_state_->planner_status_code));
        }
      }
    }
    return;
  }

  FLAGS_planner_mapless_status = !(f->use_hd_map);

  e2e_planner_.init();

  planner_status_ =
      e2e_planner_.run(planner_world->input_ref(), *planner_state_,
                       planner_world->output_pt(), thread_pool_.get());
  Log2FG::LogMissionData("Planner Status", planner_status_.message().data());
  LOG(INFO) << "planner_status: "
            << e2e_noa::planning::PlannerStatusProto::PlannerStatusCode_Name(
                   planner_status_.status_code())
            << ", message: " << planner_status_.message() << std::endl;
  planner_state_->agent_status_history =
      planner_world->output_pt()->agent_status_history;

  UpdatePlannerState(&planner_world->input_ref(), planner_world->output_pt(),
                     predicted_plan_time);
  PlanningResultPub(f, &planner_world->input_ref(), planner_world->output_pt());

  e2e_noa::DestroyContainerAsyncMarkSource(std::move(*planner_world), "");

  LOG(INFO) << "CityPlanner Run End ...";
}

void CityPlanner::UpdatePlannerState(
    const e2e_noa::planning::PlannerWorldInput* planner_input,
    const e2e_noa::planning::PlannerWorldOutput* planner_output,
    const absl::Time predicted_plan_time) {
  planner_state_->Clear();
  planner_state_->selector_state = std::move(planner_output->selector_state);
  planner_state_->nudge_object_info = planner_output->nudge_object_info;
  planner_state_->if_continuous_lc = planner_output->if_continuous_lc;
  planner_state_->lc_push_dir = planner_output->lc_push_dir;
  planner_state_->last_lc_style = planner_output->last_lc_style;

  if (planner_status_.ok()) {
    const auto& selected_decision_exploration =
        planner_output->st_planner_output_list.at(0).decision_exploration;

    e2e_planner_.ParseDecisionExplorationOutputToPlannerState(
        selected_decision_exploration, planner_state_.get(),
        planner_state_->planner_semantic_map_manager->map_ptr());
    e2e_planner_.ParseStPlannerOutputToPlannerState(
        planner_output->st_planner_output_list[0], planner_state_.get());
    for (const auto& est_output : planner_output->st_planner_output_list) {
      if (!est_output.decision_state.traffic_light_decision_state()
               .fsd_tld_state()
               .junction_lane_id()
               .empty()) {
        planner_state_->decision_state.mutable_traffic_light_decision_state()
            ->mutable_fsd_tld_state()
            ->set_junction_lane_id(
                est_output.decision_state.traffic_light_decision_state()
                    .fsd_tld_state()
                    .junction_lane_id());
        break;
      }
    }
    UpdateObjectsHistory(
        planner_state_->object_history_manager, planner_input->objects_proto,
        planner_output->st_planner_output_list[0].st_boundaries_with_decision,
        planner_output->st_planner_output_list[0]
            .st_planner_object_trajectories,
        planner_output->st_planner_output_list[0].obj_lead,
        *planner_input->stalled_objects, planner_output->nudge_object_info);

    planner_state_->ego_history.UpdateEgoHistory(
        planner_output->curr_selected_ego_frame);

    const auto past_points = e2e_noa::planning::CreatePastPointsList(
        planner_input->start_point_info->plan_time,
        planner_state_->previous_trajectory,
        planner_input->start_point_info->reset,
        e2e_noa::planning::kMaxPastPointNum);

    planner_state_->previous_trajectory.Clear();
    FillTrajectoryProto(
        planner_input->start_point_info->plan_time,
        planner_output->st_planner_output_list[0].traj_points, past_points,
        selected_decision_exploration.plan_passage.lane_path(),
        selected_decision_exploration.lane_change_state,
        e2e_noa::TURN_SIGNAL_NONE,

        planner_output->st_planner_debug_list[0].traj_validation_result,
        &planner_state_->previous_trajectory);

    for (const auto& est_output : planner_output->st_planner_output_list) {
      if ("none" != est_output.lc_lead_obj_id &&
          planner_state_->lc_lead_obj_ids.end() ==
              std::find(planner_state_->lc_lead_obj_ids.begin(),
                        planner_state_->lc_lead_obj_ids.end(),
                        est_output.lc_lead_obj_id)) {
        planner_state_->lc_lead_obj_ids.push_back(est_output.lc_lead_obj_id);
      }
    }
  }

  if (!planner_state_->preferred_lane_path.IsEmpty() && !planner_status_.ok()) {
    planner_state_->preferred_lane_path = e2e_noa::mapping::LanePath();
    planner_state_->alc_state = e2e_noa::ALC_STANDBY_ENABLE;
    planner_state_->lane_change_command = e2e_noa::DriverAction::LC_CMD_NONE;
    planner_state_->plc_prepare_start_time = std::nullopt;
    LOG(ERROR) << "Teleop lane change failed: all branches failed.";
  }
  if (planner_output->plc_result.has_value()) {
    planner_state_->preferred_lane_path =
        std::move(planner_output->plc_result->preferred_lane_path);
    planner_state_->alc_state = planner_output->alc_state;
    planner_state_->lane_change_command =
        planner_output->plc_result->lane_change_command;

    if (planner_state_->alc_state == e2e_noa::ALC_PREPARE) {
      if (!planner_state_->plc_prepare_start_time.has_value()) {
        planner_state_->plc_prepare_start_time = predicted_plan_time;
      }
    } else if (planner_state_->plc_prepare_start_time.has_value()) {
      planner_state_->plc_prepare_start_time = std::nullopt;
    }
  }

  if (planner_status_.ok()) {
    planner_state_->tl_stop_interface = planner_output->tl_stop_interface;

    planner_state_->speed_state = planner_output->speed_state;
  }

  const auto& map_func_id = planner_input->behavior.has_value()
                                ? planner_input->behavior->function_id()
                                : e2e_noa::Behavior_FunctionId_NONE;
  if (planner_status_.ok() &&
      map_func_id != e2e_noa::Behavior_FunctionId_NONE) {
    const auto& decision_exploration =
        planner_output->st_planner_output_list.at(0).decision_exploration;
    planner_state_->turn_signal_result = DecideTurnSignal(
        *planner_state_->planner_semantic_map_manager,
        planner_output->selector_state.pre_turn_signal,
        decision_exploration.plan_passage.lane_path(),
        planner_output->st_planner_output_list[0].redlight_lane_id,
        decision_exploration.lane_change_state,
        decision_exploration.plan_passage,
        decision_exploration.av_frenet_box_on_plan_passage,
        planner_input->pnp_infos,
        e2e_noa::planning::TurnSignalResult{
            .signal = decision_exploration.planner_turn_signal,
            .reason = decision_exploration.turn_signal_reason},
        planner_input->pose_proto, planner_output->if_continuous_lc,
        decision_exploration.plan_passage.lane_seq_info(),
        planner_output->turn_type_signal);

    if (map_func_id == e2e_noa::Behavior_FunctionId_LKA) {
      if (!planner_input->behavior->auto_navi_lc_enable_status() ||
          planner_state_->turn_signal_result.reason !=
              e2e_noa::TURNING_TURN_SIGNAL) {
        planner_state_->turn_signal_result =
            e2e_noa::planning::TurnSignalResult{e2e_noa::TURN_SIGNAL_NONE,
                                                e2e_noa::TURN_SIGNAL_OFF};
      }
    }
  }

  VLOG(INFO) << "UpdatePlannerState end ..." << std::endl;

  return;
}

void CityPlanner::Reset() {
  e2e_noa::Timer timer("CityPlanner::Reset::planner_state_");
  planner_state_.reset(new e2e_noa::planning::PlannerState());
  planning_result_msg_.reset(new ad_e2e::planning::planning_result_type());
}

void CityPlanner::PlanningResultPub(
    const PlanningInputFrame* f,
    const e2e_noa::planning::PlannerWorldInput* planner_input,
    const e2e_noa::planning::PlannerWorldOutput* planner_output) {
  auto msg_plan_result = std::make_shared<planning_result_type>();

  auto& plan_res_proto = msg_plan_result->first.proto;

  plan_res_proto.mutable_trajectory()->set_timestamp(f->last_msg_timestamp);
  plan_res_proto.mutable_trajectory()->set_coordinate_type(0);
  if (!planner_state_->previous_trajectory.past_points().empty()) {
    int start_index =
        planner_state_->previous_trajectory.past_points_size() - 1;
    for (int i = planner_state_->previous_trajectory.past_points_size(); i > 0;
         --i) {
      if (planner_state_->previous_trajectory.trajectory_start_timestamp() +
              planner_state_->previous_trajectory.past_points(i - 1)
                  .relative_time() <
          f->last_msg_timestamp) {
        start_index = i - 1;
        break;
      }
      start_index = i - 1;
    }
    for (int i = start_index;
         i < planner_state_->previous_trajectory.past_points_size(); ++i) {
      auto point = planner_state_->previous_trajectory.past_points(i);
      zark::e2e_noa::TrajPoint* out_point =
          plan_res_proto.mutable_trajectory()->add_points();
      out_point->set_x(point.path_point().x());
      out_point->set_y(point.path_point().y());
      out_point->set_theta(e2e_noa::NormalizeAngle(point.path_point().theta()));
      out_point->set_kappa(point.path_point().kappa());
      out_point->set_lambda(point.path_point().lambda());

      out_point->set_s(point.path_point().s());
      out_point->set_t(
          planner_state_->previous_trajectory.trajectory_start_timestamp() +
          point.relative_time());
      out_point->set_relative_time(point.relative_time());
      out_point->set_v(point.v());
      out_point->set_a(point.a());
      out_point->set_j(point.j());
      out_point->mutable_refer_info()->set_yaw_rate(point.yaw_rate());
      out_point->mutable_refer_info()->set_steering_angle(
          point.path_point().steer_angle());
    }
  }

  std::string main_target_id = "";
  double main_target_speed = 0.0;
  double main_target_acc = 0.0;
  double main_target_ds = 0.0;
  double main_target_dl = 0.0;
  if (!planner_output->st_planner_output_list.empty()) {
    double s_min = std::numeric_limits<double>::max();
    for (const auto& traj :
         planner_output->st_planner_output_list[0].leading_trajs) {
      if (e2e_noa::planning::ConstraintProto::LeadingObjectProto::
                  AFTER_STOPLINE != traj.second.reason() &&
          traj.second.st_constraints_size() > 0 &&
          traj.second.st_constraints().begin()->s() < s_min) {
        s_min = traj.second.st_constraints().begin()->s();
        main_target_id = e2e_noa::planning::SpacetimeObjectTrajectory::
            GetObjectIdFromTrajectoryId(traj.first);
      }
    }
    if (!main_target_id.empty()) {
      for (const auto& st_boundary : planner_output->st_planner_output_list[0]
                                         .st_boundaries_with_decision) {
        if (st_boundary.object_id().has_value() &&
            st_boundary.object_id().value() == main_target_id) {
          main_target_speed = st_boundary.obj_pose_info().v();
          main_target_acc = st_boundary.obj_pose_info().a();
          main_target_ds = st_boundary.ds();
          main_target_dl = st_boundary.dl();
          break;
        }
      }
    }
    plan_res_proto.mutable_state()->set_main_target_id(main_target_id);

    // for nudge
    if (planner_output->nudge_object_info.has_value() &&
        planner_output->nudge_object_info->id != main_target_id) {
      plan_res_proto.mutable_state()->add_nudge_obstacles(
          planner_output->nudge_object_info->id);
      plan_res_proto.mutable_state()->set_nudge_obstacle_id(
          planner_output->nudge_object_info->id);
      if (planner_output->nudge_object_info->direction == 1) {
        if (planner_output->nudge_object_info->nudge_state ==
            e2e_noa::planning::NudgeObjectInfo::NudgeState::NUDGE) {
          plan_res_proto.mutable_state()->set_nudge_state(
              zark::e2e_noa::NudgeState::NUDGE_LEFT);
        } else {
          plan_res_proto.mutable_state()->set_lane_borrow_state(
              zark::e2e_noa::LaneBorrowState::BORROW_LEFT);
          plan_res_proto.mutable_state()->set_lane_borrow_reason(
              zark::e2e_noa::LaneBorrowReason::LB_REASON_FOR_GENERAL);
        }
      } else {
        if (planner_output->nudge_object_info->nudge_state ==
            e2e_noa::planning::NudgeObjectInfo::NudgeState::NUDGE) {
          plan_res_proto.mutable_state()->set_nudge_state(
              zark::e2e_noa::NudgeState::NUDGE_RIGHT);
        } else {
          plan_res_proto.mutable_state()->set_lane_borrow_state(
              zark::e2e_noa::LaneBorrowState::BORROW_RIGHT);
          plan_res_proto.mutable_state()->set_lane_borrow_reason(
              zark::e2e_noa::LaneBorrowReason::LB_REASON_FOR_GENERAL);
        }
      }
    }

    double main_target_s = kDefaultMainTargetS;
    const auto& st_boundaries_with_decision =
        planner_output->st_planner_output_list[0].st_boundaries_with_decision;
    if (!main_target_id.empty()) {
      for (const auto& st_boundary : st_boundaries_with_decision) {
        if ((st_boundary.decision_type() == e2e_noa::StBoundaryProto::YIELD ||
             st_boundary.decision_type() == e2e_noa::StBoundaryProto::FOLLOW) &&
            st_boundary.object_id().has_value() &&
            st_boundary.object_id().value() == main_target_id &&
            st_boundary.st_boundary() &&
            st_boundary.st_boundary()->GetBoundarySRange(0).has_value()) {
          main_target_s =
              st_boundary.st_boundary()->GetBoundarySRange(0).value().second;
          break;
        }
      }
    }

    double main_target_length = kDefaultVehicleLength;
    const auto* main_target_car =
        planner_input->object_manager->FindObjectById(main_target_id);
    if (main_target_car) {
      main_target_length = main_target_car->bounding_box().length();
    }

    for (const auto& st_boundary : st_boundaries_with_decision) {
      zark::e2e_noa::YieldObstacles* yield_obs =
          plan_res_proto.mutable_state()->add_yield_obstacles();
      if ((st_boundary.decision_type() == e2e_noa::StBoundaryProto::YIELD ||
           st_boundary.decision_type() == e2e_noa::StBoundaryProto::FOLLOW) &&
          st_boundary.object_id().has_value() &&
          st_boundary.object_id().value() != main_target_id &&
          st_boundary.st_boundary() &&
          st_boundary.st_boundary()->GetBoundarySRange(0).has_value() &&
          st_boundary.st_boundary()->GetBoundarySRange(0).value().first <
              main_target_length + main_target_s) {
        yield_obs->set_id(st_boundary.object_id().value());
      }
    }

    if (!plan_res_proto.state().yield_obstacles().empty()) {
      plan_res_proto.mutable_state()->set_yield_obstacle_id(
          plan_res_proto.state().yield_obstacles(0).id());
    }

    for (const auto& st_boundary : st_boundaries_with_decision) {
      if (st_boundary.st_boundary() &&
          st_boundary.decision_type() == e2e_noa::StBoundaryProto::OVERTAKE &&
          st_boundary.object_id().has_value()) {
        plan_res_proto.mutable_state()->add_overtake_obstacles(
            st_boundary.object_id().value());
      }
    }
  }

  const RepeatedPtrField<e2e_noa::ApolloTrajectoryPointProto>& pts_vec =
      planner_state_->previous_trajectory.trajectory_point();
  for (auto iter = pts_vec.begin(); iter != pts_vec.end(); ++iter) {
    zark::e2e_noa::TrajPoint* out_point =
        plan_res_proto.mutable_trajectory()->add_points();
    out_point->set_x(iter->path_point().x());
    out_point->set_y(iter->path_point().y());
    out_point->set_theta(e2e_noa::NormalizeAngle(iter->path_point().theta()));
    out_point->set_kappa(iter->path_point().kappa());

    out_point->set_lambda(iter->path_point().lambda());
    out_point->set_s(iter->path_point().s());
    out_point->set_t(
        planner_state_->previous_trajectory.trajectory_start_timestamp() +
        iter->relative_time());
    out_point->set_relative_time(iter->relative_time());
    out_point->set_v(iter->v());
    out_point->set_a(iter->a());
    out_point->set_j(iter->j());
    out_point->mutable_refer_info()->set_yaw_rate(iter->yaw_rate());
    out_point->mutable_refer_info()->set_steering_angle(
        iter->path_point().steer_angle());
  }

  /* Pub traj type */
  plan_res_proto.mutable_trajectory()->set_trajectory_type(
      zark::e2e_noa::PlanningTrajectory::TYPE_NONE);
  if (ComputerFailCodeHoldCounter(planner_status_.status_code())) {
    plan_res_proto.mutable_state()->set_result(
        zark::e2e_noa::Result::RESULT_OK);
    plan_res_proto.mutable_state()->set_fail_reason(0);
    if (planner_state_->selector_state.is_deviate_navi) {
      plan_res_proto.mutable_state()->set_fail_reason(static_cast<uint32_t>(
          e2e_noa::planning::PlannerStatusProto::DEVIATE_NAVI));
    }
  } else {
    if (fail_number_ > 0) {
      --fail_number_;
      plan_res_proto.mutable_state()->set_result(
          zark::e2e_noa::Result::RESULT_OK);
    } else {
      plan_res_proto.mutable_state()->set_result(
          zark::e2e_noa::Result::RESULT_FAIL);
      plan_res_proto.mutable_state()->set_fail_reason(
          static_cast<uint32_t>(planner_status_.status_code()));
      plan_res_proto.mutable_trajectory()->clear_points();
    }
  }

  /* Pub unmovable and queue obstacles */
  for (const auto& stalled_obj : planner_state_->stalled_cars) {
    plan_res_proto.mutable_state()
        ->mutable_obstacles_intention()
        ->add_obstacles_unmovable(stalled_obj);
  }
  for (const auto& queue_obj : planner_state_->in_queue_cars) {
    plan_res_proto.mutable_state()
        ->mutable_obstacles_intention()
        ->add_obstacles_in_queue(queue_obj);
  }

  /* Pub light status */
  auto turn_light_req = zark::e2e_noa::TurnLight::TURN_LIGHT_NONE;
  auto turn_light_reason = zark::e2e_noa::TurnLightReason::NONE;
  if (planner_state_->turn_signal_result.signal ==
      e2e_noa::TurnSignal::TURN_SIGNAL_LEFT) {
    turn_light_req = zark::e2e_noa::TurnLight::TURN_LIGHT_LEFT;
  } else if (planner_state_->turn_signal_result.signal ==
             e2e_noa::TurnSignal::TURN_SIGNAL_RIGHT) {
    turn_light_req = zark::e2e_noa::TurnLight::TURN_LIGHT_RIGHT;
  }
  switch (planner_state_->turn_signal_result.reason) {
    case e2e_noa::TurnSignalReason::CONTINUE_LANE_CHANGE_SIGNAL:
      turn_light_reason = zark::e2e_noa::TurnLightReason::CONTINUE_LC;
      break;
    case e2e_noa::TurnSignalReason::TURNING_TURN_SIGNAL:
      turn_light_reason = zark::e2e_noa::TurnLightReason::TURN;
      break;
    case e2e_noa::TurnSignalReason::MERGE_TURN_SIGNAL:
      turn_light_reason = zark::e2e_noa::TurnLightReason::MERGE;
      break;
    case e2e_noa::TurnSignalReason::FORK_TURN_SIGNAL:
      turn_light_reason = zark::e2e_noa::TurnLightReason::SPLIT;
      break;
    default:
      break;
  }

  plan_res_proto.mutable_state()->set_turn_light_request(turn_light_req);
  plan_res_proto.mutable_state()->set_turn_light_reason(turn_light_reason);

  /* Pub lane change state */
  auto lc_state = zark::e2e_noa::LaneChangeState::Lane_Keeping;
  if (planner_state_->lane_change_state.stage() !=
      e2e_noa::LaneChangeStage::LCS_NONE) {
    if (planner_state_->lane_change_state.lc_left()) {
      lc_state = zark::e2e_noa::LaneChangeState::Lane_ChangeLeft;
    } else {
      lc_state = zark::e2e_noa::LaneChangeState::Lane_ChangeRight;
    }
    if (planner_state_->lane_change_state.stage() ==
        e2e_noa::LaneChangeStage::LCS_RETURN) {
      lc_state = zark::e2e_noa::LaneChangeState::Lane_ChangeCancel;
    }
  } else {
    lc_state = zark::e2e_noa::LaneChangeState::Lane_Keeping;
    if (planner_state_->selector_state.lane_change_prepare_state ==
        e2e_noa::planning::LaneChangePrepareState::Lane_PrepareLeft) {
      lc_state = zark::e2e_noa::LaneChangeState::Lane_PrepareLeft;
    }
    if (planner_state_->selector_state.lane_change_prepare_state ==
        e2e_noa::planning::LaneChangePrepareState::Lane_PrepareRight) {
      lc_state = zark::e2e_noa::LaneChangeState::Lane_PrepareRight;
    }
    if (planner_output->if_cancel_lc) {
      lc_state = zark::e2e_noa::LaneChangeState::Lane_ChangeCancel;
    }
  }
  plan_res_proto.mutable_state()->set_lc_state(lc_state);

  /* Pub lane change reason */
  if (planner_state_->last_lc_reason ==
          e2e_noa::planning::LaneChangeReason::MANUAL_CHANGE &&
      planner_state_->lane_change_state.stage() ==
          e2e_noa::LaneChangeStage::LCS_NONE) {
    planner_state_->last_manual_lc_time++;
  }
  if (planner_state_->last_lc_reason !=
      e2e_noa::planning::LaneChangeReason::MANUAL_CHANGE) {
    planner_state_->last_manual_lc_time = 0;
  }
  auto lc_reason = zark::e2e_noa::LcReason::LC_REASON_NONE;
  if (planner_state_->lane_change_state.stage() !=
          e2e_noa::LaneChangeStage::LCS_NONE ||
      planner_state_->selector_state.lane_change_prepare_state !=
          e2e_noa::planning::LaneChangePrepareState::Lane_Keeping) {
    if (planner_state_->selector_state.last_lane_change_reason ==
        e2e_noa::planning::LaneChangeReason::NO_CHANGE) {
      planner_state_->selector_state.last_lane_change_reason =
          planner_state_->selector_state.lane_change_reason;
    }
    if (planner_state_->selector_state.last_lane_change_reason !=
            planner_state_->selector_state.lane_change_reason &&
        planner_state_->selector_state.last_lane_change_reason !=
            e2e_noa::planning::LaneChangeReason::MANUAL_CHANGE &&
        planner_output->input_lc_cmd != e2e_noa::DriverAction::LC_CMD_LEFT &&
        planner_output->input_lc_cmd != e2e_noa::DriverAction::LC_CMD_RIGHT) {
      planner_state_->selector_state.lane_change_reason =
          planner_state_->selector_state.last_lane_change_reason;
    }
    if (planner_state_->lane_change_state.stage() ==
            e2e_noa::LaneChangeStage::LCS_NONE &&
        planner_state_->selector_state.lane_change_prepare_state ==
            e2e_noa::planning::LaneChangePrepareState::Lane_Keeping) {
      planner_state_->selector_state.last_lane_change_reason ==
          e2e_noa::planning::LaneChangeReason::NO_CHANGE;
    }
    if (planner_state_->selector_state.lane_change_reason ==
        e2e_noa::planning::LaneChangeReason::PROGRESS_CHANGE) {
      lc_reason = zark::e2e_noa::LcReason::LC_REASON_OVERTAKE;
    }
    if (planner_state_->selector_state.lane_change_reason ==
            e2e_noa::planning::LaneChangeReason::ROUTE_CHANGE ||
        planner_state_->selector_state.lane_change_reason ==
            e2e_noa::planning::LaneChangeReason::DEFAULT_CHANGE ||
        planner_state_->selector_state.lane_change_reason ==
            e2e_noa::planning::LaneChangeReason::LCC_ROUTE_CHANGE) {
      lc_reason = zark::e2e_noa::LcReason::LC_REASON_FOR_NAVI;
    }
    if (planner_state_->selector_state.lane_change_reason ==
        e2e_noa::planning::LaneChangeReason::MANUAL_CHANGE) {
      lc_reason = zark::e2e_noa::LcReason::LC_REASON_MANUAL;
    }
    if (planner_state_->selector_state.lane_change_reason ==
        e2e_noa::planning::LaneChangeReason::CENTER_CHANGE) {
      lc_reason = zark::e2e_noa::LcReason::LC_REASON_CENTER;
    }
    if (planner_state_->selector_state.lane_change_reason ==
        e2e_noa::planning::LaneChangeReason::MERGE_CHANGE) {
      lc_reason = zark::e2e_noa::LcReason::LC_REASON_FOR_NAVI;
    }
    if (planner_state_->selector_state.lane_change_reason ==
        e2e_noa::planning::AVOID_STATIC_OBJECT) {
      lc_reason = zark::e2e_noa::LcReason::LC_REASON_FOR_AVOID_STATIC_OBJECT;
    }
    if (planner_state_->selector_state.lane_change_reason ==
        e2e_noa::planning::LaneChangeReason::AVOID_VEHICLE_CHANGE) {
      lc_reason = zark::e2e_noa::LcReason::LC_REASON_FOR_AVOID_VEHICLE;
    }
    if (planner_state_->selector_state.lane_change_reason ==
        e2e_noa::planning::LaneChangeReason::AVOID_ROADWORK_CHANGE) {
      lc_reason = zark::e2e_noa::LcReason::LC_REASON_FOR_AVOID_ROADWORK;
    }
    if (planner_state_->selector_state.lane_change_reason ==
        e2e_noa::planning::LaneChangeReason::AVOID_LANE_BUS) {
      lc_reason = zark::e2e_noa::LcReason::LC_REASON_FOR_AVOID_LANE_BUS;
    }
  }
  if (planner_state_->lane_change_state.stage() !=
      e2e_noa::LaneChangeStage::LCS_NONE) {
    planner_state_->last_lc_reason =
        planner_state_->selector_state.lane_change_reason;
    if (planner_output->input_lc_cmd == e2e_noa::DriverAction::LC_CMD_LEFT ||
        planner_output->input_lc_cmd == e2e_noa::DriverAction::LC_CMD_RIGHT) {
      planner_state_->last_lc_reason =
          e2e_noa::planning::LaneChangeReason::MANUAL_CHANGE;
    }
  }

  plan_res_proto.mutable_state()->set_lc_reason(lc_reason);

  /* Pub lane change gap info */
  plan_res_proto.mutable_state()->mutable_lc_gap_obstacles()->set_gap_front(
      planner_state_->selector_state.gap_front_id);

  plan_res_proto.mutable_state()->mutable_lc_gap_obstacles()->set_gap_back(
      planner_state_->selector_state.gap_back_id);

  /* Pub lane change gap info */
  auto lc_left_unable_reason = zark::e2e_noa::LcUnableReason::REASON_NONE;
  auto lc_right_unable_reason = zark::e2e_noa::LcUnableReason::REASON_NONE;
  if ((planner_output->input_lc_cmd == e2e_noa::DriverAction::LC_CMD_LEFT ||
       planner_state_->selector_state.lane_change_prepare_state ==
           e2e_noa::planning::LaneChangePrepareState::Lane_PrepareLeft) &&
      planner_state_->lane_change_state.stage() ==
          e2e_noa::LaneChangeStage::LCS_NONE) {
    if (planner_state_->selector_state.lc_unable_reason ==
            e2e_noa::planning::LcFeasibility::FEASIBILITY_NO_LANE ||
        planner_output->lc_unable_reason ==
            e2e_noa::planning::LcFeasibility::FEASIBILITY_NO_LANE) {
      lc_left_unable_reason = zark::e2e_noa::LcUnableReason::REASON_NO_LANE;
      lc_state = zark::e2e_noa::LaneChangeState::Lane_PrepareLeft;
    }
    if (planner_state_->selector_state.lc_unable_reason ==
            e2e_noa::planning::LcFeasibility::FEASIBILITY_OBS_TARGET_FRONT ||
        planner_output->lc_unable_reason ==
            e2e_noa::planning::LcFeasibility::FEASIBILITY_OBS_TARGET_FRONT) {
      lc_left_unable_reason =
          zark::e2e_noa::LcUnableReason::REASON_OBSTACLE_FRONT;
      lc_state = zark::e2e_noa::LaneChangeState::Lane_PrepareLeft;
    }
    if (planner_state_->selector_state.lc_unable_reason ==
        e2e_noa::planning::LcFeasibility::FEASIBILITY_OBS_TARGET_BACK) {
      lc_left_unable_reason =
          zark::e2e_noa::LcUnableReason::REASON_OBSTACLE_REAR;
      lc_state = zark::e2e_noa::LaneChangeState::Lane_PrepareLeft;
    }
    if (planner_state_->selector_state.lc_unable_reason ==
        e2e_noa::planning::LcFeasibility::FEASIBILITY_CURVATURE) {
      lc_left_unable_reason = zark::e2e_noa::LcUnableReason::REASON_CURVATURE;
      lc_state = zark::e2e_noa::LaneChangeState::Lane_PrepareLeft;
    }
    if (planner_state_->selector_state.lc_unable_reason ==
        e2e_noa::planning::LcFeasibility::FEASIBILITY_LINE_TYPE) {
      lc_left_unable_reason = zark::e2e_noa::LcUnableReason::REASON_LINE_SOLID;
      lc_state = zark::e2e_noa::LaneChangeState::Lane_PrepareLeft;
    }
    if (planner_state_->selector_state.lc_unable_reason ==
        e2e_noa::planning::LcFeasibility::FEASIBILITY_CONES) {
      lc_left_unable_reason =
          zark::e2e_noa::LcUnableReason::REASON_SATIC_OBSTACLE;
      lc_state = zark::e2e_noa::LaneChangeState::Lane_PrepareLeft;
    }
    if (planner_state_->selector_state.lc_unable_reason ==
        e2e_noa::planning::LcFeasibility::FEASIBILITY_GENERAL) {
      lc_left_unable_reason =
          zark::e2e_noa::LcUnableReason::REASON_GENERAL_REASON;
      lc_state = zark::e2e_noa::LaneChangeState::Lane_PrepareLeft;
    }
  }

  plan_res_proto.mutable_state()->set_lc_state(lc_state);
  plan_res_proto.mutable_state()->set_lc_unable_reason_left(
      lc_left_unable_reason);

  if ((planner_output->input_lc_cmd == e2e_noa::DriverAction::LC_CMD_RIGHT ||
       planner_state_->selector_state.lane_change_prepare_state ==
           e2e_noa::planning::LaneChangePrepareState::Lane_PrepareRight) &&
      planner_state_->lane_change_state.stage() ==
          e2e_noa::LaneChangeStage::LCS_NONE) {
    if (planner_state_->selector_state.lc_unable_reason ==
            e2e_noa::planning::LcFeasibility::FEASIBILITY_NO_LANE ||
        planner_output->lc_unable_reason ==
            e2e_noa::planning::LcFeasibility::FEASIBILITY_NO_LANE) {
      lc_right_unable_reason = zark::e2e_noa::LcUnableReason::REASON_NO_LANE;
      lc_state = zark::e2e_noa::LaneChangeState::Lane_PrepareRight;
    }
    if (planner_state_->selector_state.lc_unable_reason ==
            e2e_noa::planning::LcFeasibility::FEASIBILITY_OBS_TARGET_FRONT ||
        planner_output->lc_unable_reason ==
            e2e_noa::planning::LcFeasibility::FEASIBILITY_OBS_TARGET_FRONT) {
      lc_right_unable_reason =
          zark::e2e_noa::LcUnableReason::REASON_OBSTACLE_FRONT;
      lc_state = zark::e2e_noa::LaneChangeState::Lane_PrepareRight;
    }
    if (planner_state_->selector_state.lc_unable_reason ==
        e2e_noa::planning::LcFeasibility::FEASIBILITY_OBS_TARGET_BACK) {
      lc_right_unable_reason =
          zark::e2e_noa::LcUnableReason::REASON_OBSTACLE_REAR;
      lc_state = zark::e2e_noa::LaneChangeState::Lane_PrepareRight;
    }
    if (planner_state_->selector_state.lc_unable_reason ==
        e2e_noa::planning::LcFeasibility::FEASIBILITY_CURVATURE) {
      lc_right_unable_reason = zark::e2e_noa::LcUnableReason::REASON_CURVATURE;
      lc_state = zark::e2e_noa::LaneChangeState::Lane_PrepareRight;
    }
    if (planner_state_->selector_state.lc_unable_reason ==
        e2e_noa::planning::LcFeasibility::FEASIBILITY_LINE_TYPE) {
      lc_right_unable_reason = zark::e2e_noa::LcUnableReason::REASON_LINE_SOLID;
      lc_state = zark::e2e_noa::LaneChangeState::Lane_PrepareRight;
    }
    if (planner_state_->selector_state.lc_unable_reason ==
        e2e_noa::planning::LcFeasibility::FEASIBILITY_CONES) {
      lc_right_unable_reason =
          zark::e2e_noa::LcUnableReason::REASON_SATIC_OBSTACLE;
      lc_state = zark::e2e_noa::LaneChangeState::Lane_PrepareRight;
    }
    if (planner_state_->selector_state.lc_unable_reason ==
        e2e_noa::planning::LcFeasibility::FEASIBILITY_GENERAL) {
      lc_right_unable_reason =
          zark::e2e_noa::LcUnableReason::REASON_GENERAL_REASON;
      lc_state = zark::e2e_noa::LaneChangeState::Lane_PrepareRight;
    }
  }
  if ((planner_output->input_lc_cmd == e2e_noa::DriverAction::LC_CMD_LEFT ||
       planner_output->input_lc_cmd == e2e_noa::DriverAction::LC_CMD_RIGHT) &&
      (planner_state_->lane_change_state.stage() !=
       e2e_noa::LaneChangeStage::LCS_EXECUTING) &&
      planner_state_->lc_command_number > 273) {
    lc_state = zark::e2e_noa::LaneChangeState::Lane_ChangeCancel;
  }
  if (planner_state_->lane_change_state.stage() ==
      e2e_noa::LaneChangeStage::LCS_EXECUTING) {
    planner_state_->lc_command_number = 0;
  }

  if (planner_state_->selector_state.prev_lc_stage ==
          e2e_noa::LaneChangeStage::LCS_EXECUTING &&
      planner_state_->lane_change_state.stage() ==
          e2e_noa::LaneChangeStage::LCS_NONE &&
      lc_state != zark::e2e_noa::LaneChangeState::Lane_ChangeCancel) {
    lc_state = zark::e2e_noa::LaneChangeState::Lane_Keeping;
    planner_state_->selector_state.pre_lane_change_state =
        e2e_noa::planning::LaneChangeState::Lc_Keeping;
    planner_state_->lc_keeping_delay_number = 1;
  } else if ((planner_state_->selector_state.prev_lc_stage ==
                  e2e_noa::LaneChangeStage::LCS_RETURN &&
              planner_state_->lane_change_state.stage() ==
                  e2e_noa::LaneChangeStage::LCS_NONE) ||
             lc_state == zark::e2e_noa::LaneChangeState::Lane_ChangeCancel) {
    lc_state = zark::e2e_noa::LaneChangeState::Lane_ChangeCancel;
    planner_state_->selector_state.pre_lane_change_state =
        e2e_noa::planning::LaneChangeState::Lc_ChangeCancel;
    planner_state_->lc_cancel_delay_number = 1;
  } else {
    if (planner_state_->lc_keeping_delay_number > 0 &&
        planner_state_->lc_keeping_delay_number <= 2 &&
        (planner_state_->selector_state.pre_lane_change_state ==
             e2e_noa::planning::LaneChangeState::Lc_Keeping ||
         planner_state_->selector_state.pre_lane_change_state ==
             e2e_noa::planning::LaneChangeState::Lc_ChangeCancel)) {
      lc_state = zark::e2e_noa::LaneChangeState::Lane_Keeping;
      planner_state_->selector_state.pre_lane_change_state =
          e2e_noa::planning::LaneChangeState::Lc_Keeping;
      planner_state_->lc_keeping_delay_number++;
    } else {
      planner_state_->lc_keeping_delay_number = 0;
    }

    if (planner_state_->lc_cancel_delay_number > 0 &&
        planner_state_->lc_cancel_delay_number <= 2 &&
        planner_state_->selector_state.pre_lane_change_state ==
            e2e_noa::planning::LaneChangeState::Lc_ChangeCancel) {
      lc_state = zark::e2e_noa::LaneChangeState::Lane_ChangeCancel;
      planner_state_->selector_state.pre_lane_change_state =
          e2e_noa::planning::LaneChangeState::Lc_ChangeCancel;
      planner_state_->lc_cancel_delay_number++;
    } else if (planner_state_->lc_cancel_delay_number == 3) {
      planner_state_->lc_keeping_delay_number = 1;
      planner_state_->lc_cancel_delay_number = 0;
    }
  }

  plan_res_proto.mutable_state()->set_lc_state(lc_state);
  plan_res_proto.mutable_state()->set_lc_unable_reason_right(
      lc_right_unable_reason);

  /* Congestion lane change notice */
  auto lc_notice = e2e_noa::planning::LaneChangeNotice::Notice_None;
  if (planner_output->if_lc_to_congestion &&
      !planner_state_->has_triggered_lc_notice &&
      (lc_state == zark::e2e_noa::LaneChangeState::Lane_PrepareRight ||
       lc_state == zark::e2e_noa::LaneChangeState::Lane_PrepareLeft)) {
    planner_state_->lc_notice_pub_counter = 1;
    planner_state_->lc_notice = planner_output->lc_notice;
  }

  if (planner_state_->lc_notice_pub_counter > 0 &&
      planner_state_->lc_notice_pub_counter < 25) {
    lc_notice = planner_state_->lc_notice;
  } else if (planner_state_->lc_notice_pub_counter >= 25 &&
             planner_state_->lc_notice_pub_counter < 45) {
    if (lc_state != zark::e2e_noa::LaneChangeState::Lane_PrepareRight &&
        lc_state != zark::e2e_noa::LaneChangeState::Lane_PrepareLeft) {
      planner_state_->lc_notice_pub_counter = 50;
    } else {
      lc_notice = planner_state_->lc_notice;
    }
  }
  // auto lc_notice_idl =
  //     static_cast<zark::e2e_noa::LaneChangeNotice_def::type>(lc_notice);
  // plan_res_proto.mutable_state()->set_lc_notice(lc_notice_idl);
  if (planner_state_->lc_notice_pub_counter > 0) {
    planner_state_->has_triggered_lc_notice = true;
    planner_state_->lc_notice_pub_counter += 1;
  }
  if (planner_output->has_passed_this_junction) {
    planner_state_->has_triggered_lc_notice = false;
  }

  /* Pub traffic stop line */
  plan_res_proto.mutable_state()->set_stop_line(
      static_cast<zark::e2e_noa::StopLineInterface>(
          planner_state_->tl_stop_interface));

  if (planner_input->start_point_info->reset) {
    plan_res_proto.mutable_trajectory()->set_trajectory_pln_mode(
        zark::e2e_noa::PlanningTrajectory::REPLAN);
  } else {
    plan_res_proto.mutable_trajectory()->set_trajectory_pln_mode(
        zark::e2e_noa::PlanningTrajectory::NORMAL);
  }

  /* Pub planning actual cruising speed limit */
  constexpr double kDefaultCruisingSpeedLimit = 50.0;  // 50 km/h
  double cruising_speed_limit =
      planner_input->cruising_speed_limit.has_value()
          ? *(planner_input->cruising_speed_limit) * 3.6
      : planner_input->behavior.has_value()
          ? planner_input->behavior->cruising_speed_limit() * 3.6
          : kDefaultCruisingSpeedLimit;
  plan_res_proto.mutable_state()->set_planning_speed_limit(
      cruising_speed_limit);

  AddTrafficLightInfo(planner_state_->planner_semantic_map_manager->map_ptr(),
                      plan_res_proto);

  planning_result_msg_.swap(msg_plan_result);

  return;
}

std::shared_ptr<ad_e2e::planning::planning_result_type>
CityPlanner::GetPlanningResult() const {
  return planning_result_msg_;
}

bool CityPlanner::ComputerFailCodeHoldCounter(
    const e2e_noa::planning::PlannerStatusProto::PlannerStatusCode& status_code)
    const {
  static e2e_noa::planning::PlannerStatusProto::PlannerStatusCode
      last_status_code = e2e_noa::planning::PlannerStatusProto::OK;
  if (status_code == e2e_noa::planning::PlannerStatusProto::OK) {
    last_status_code = status_code;
    return true;
  } else if (last_status_code == e2e_noa::planning::PlannerStatusProto::OK) {
    switch (status_code) {
      case e2e_noa::planning::PlannerStatusProto::INPUT_INCORRECT: {
        fail_number_ = 10;
        break;
      }
      case e2e_noa::planning::PlannerStatusProto::TARGET_LANE_JUMPED_FAIL: {
        fail_number_ = 0;
        break;
      }
      default: {
        return true;
      }
    }

    last_status_code = status_code;
    return false;
  }
  return false;
}

void CityPlanner::AddTrafficLightInfo(const ad_e2e::planning::MapPtr& map,
                                      zark::e2e_noa::PlanningResult& plan_res) {
  auto SetLightInfo =
      [](const TrafficLightInfo* local_light_info,
         zark::ads_adptrout::TrafficLightInfo* traffic_light_info) {
        switch (local_light_info->light_status) {
          case GREEN_LIGHT:
            traffic_light_info->set_traffic_light_color(
                zark::ads_adptrout::TLR_COLOR_GREEN);
            break;
          case YELLOW_LIGHT:
            traffic_light_info->set_traffic_light_color(
                zark::ads_adptrout::TLR_COLOR_YELLOW);
            break;
          case RED_LIGHT:
            traffic_light_info->set_traffic_light_color(
                zark::ads_adptrout::TLR_COLOR_RED);
            break;
          default:
            break;
        }
        traffic_light_info->set_traffic_light_countdown(
            std::min(local_light_info->count_down_sec, 99));
      };

  if (map == nullptr) {
    DLOG(ERROR) << "map is nullptr ";
    return;
  }
  const std::string focus_lane_id =
      planner_state_->decision_state.traffic_light_decision_state()
          .fsd_tld_state()
          .junction_lane_id();

  const auto& lane_ptr = map->GetLaneById(focus_lane_id);
  if (!lane_ptr) {
    return;
  }
  bool has_set_left = false;
  bool has_set_right = false;
  bool has_set_up = false;
  auto tl = plan_res.mutable_state()->mutable_planning_tlr_info();
  for (const auto& tl_id : lane_ptr->lane_info().traffic_lights) {
    const auto tl_ptr = map->GetTrafficLightInfoById(tl_id);
    if (!tl_ptr) {
      continue;
    }
    bool need_update = tl_ptr->light_status == LightStatus::GREEN_LIGHT ||
                       tl_ptr->light_status == LightStatus::RED_LIGHT ||
                       tl_ptr->light_status == LightStatus::YELLOW_LIGHT;
    if (!need_update) {
      continue;
    }
    if ((tl_ptr->sub_type & TrafficLightSubType::TRAFFICLIGHT_DIRECTION_LEFT) &&
        !has_set_left) {
      has_set_left = true;
      auto tl_left = tl->mutable_tlr_left_info();
      tl_left->set_traffic_light_type(
          zark::ads_adptrout::TrafficLightType::TLR_LEFT_ARROW);
      SetLightInfo(tl_ptr.get(), tl_left);
    }
    if ((tl_ptr->sub_type &
         TrafficLightSubType::TRAFFICLIGHT_DIRECTION_UTURN) &&
        !has_set_left) {
      has_set_left = true;
      auto tl_left = tl->mutable_tlr_left_info();
      tl_left->set_traffic_light_type(
          zark::ads_adptrout::TrafficLightType::TLR_LEFT_AROUND_ARROW);
      SetLightInfo(tl_ptr.get(), tl_left);
    }
    if ((tl_ptr->sub_type &
         TrafficLightSubType::TRAFFICLIGHT_DIRECTION_RIGHT) &&
        !has_set_right) {
      has_set_right = true;
      auto tl_right = tl->mutable_tlr_right_info();
      tl_right->set_traffic_light_type(
          zark::ads_adptrout::TrafficLightType::TLR_RIGHT_ARROW);
      SetLightInfo(tl_ptr.get(), tl_right);
    }
    if ((tl_ptr->sub_type & TrafficLightSubType::TRAFFICLIGHT_DIRECTION_UP) &&
        !has_set_up) {
      has_set_up = true;
      auto tl_up = tl->mutable_tlr_straight_info();
      tl_up->set_traffic_light_type(
          zark::ads_adptrout::TrafficLightType::TLR_STRAIGHT_ARROW);
      SetLightInfo(tl_ptr.get(), tl_up);
    }
  }
  if (has_set_left || has_set_right || has_set_up) {
    tl->set_tlr_sw_sts(zark::ads_adptrout::TLR_SW_ON);
    tl->set_tlr_sts(zark::ads_adptrout::TLR_ON);
  }
  return;
}

}  // namespace planning
}  // namespace ad_e2e
