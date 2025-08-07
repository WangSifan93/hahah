
#include "apps/planning/src/config/config_main.h"

#include <fstream>
#include <map>

#include "apps/planning/src/common/log.h"
#include "apps/planning/src/config/string_to_enum.hpp"

namespace zark {
namespace planning {

using nlohmann::json;

Config::Config(const std::string &config_file) {
  std::ifstream jdata(config_file);
  if (!jdata.is_open()) {
    AFATAL << "Open " << config_file << " failed";
  }
  jnode_ = nlohmann::json::parse(jdata);
}

void Config::SetScenarioConfig(ScenarioConfig &scenario_config,
                               PlanningConfig &planning_config) {
  std::string scenario_type;
  jnode_.at("scenario_type").get_to(scenario_type);
  scenario_config.set_scenario_type(
      EnumParser<ScenarioType>::Parse(scenario_type));

  scenario_config.clear_stage_type();
  for (auto stage_type_jnode : jnode_.at("stage_type")) {
    std::string stage_type;
    stage_type_jnode.get_to(stage_type);
    scenario_config.add_stage_type(EnumParser<StageType>::Parse(stage_type));
  }

  scenario_config.clear_stage_config();
  for (auto stage_config_jnode : jnode_.at("stage_config")) {
    ScenarioConfig::StageConfig stage_config;
    std::string stage_type;
    stage_config_jnode.at("stage_type").get_to(stage_type);
    stage_config.set_stage_type(EnumParser<StageType>::Parse(stage_type));

    bool enabled;
    stage_config_jnode.at("enabled").get_to(enabled);
    stage_config.set_enabled(enabled);

    for (auto task_type_jnode : stage_config_jnode.at("task_type")) {
      std::string task_type;
      task_type_jnode.get_to(task_type);
      stage_config.add_task_type(
          EnumParser<TaskConfig::TaskType>::Parse(task_type));
    }

    for (auto task_config_jnode : stage_config_jnode.at("task_config")) {
      TaskConfig task_config;
      std::string task_type;
      task_config_jnode.at("task_type").get_to(task_type);
      task_config.set_task_type(
          EnumParser<TaskConfig::TaskType>::Parse(task_type));

      SwitchTaskConfig(task_config, task_config_jnode,
                       EnumParser<TaskConfig::TaskType>::Parse(task_type));
      task_config.set_task_config(task_config_);
      stage_config.add_task_config(task_config);
    }

    scenario_config.add_stage_config(stage_config);
  }
}

void Config::SetPlanningConfig(PlanningConfig &planning_config) {
  // StandardPlanningConfig
  planning::StandardPlanningConfig standard_planning_config;
  for (auto &planner_type :
       jnode_.at("standard_planning_config").at("planner_type")) {
    std::string type;
    planner_type.get_to(type);
    standard_planning_config.add_planner_type(
        EnumParser<PlannerType>::Parse(type));
  }

  bool tranform_traj_to_body_frame = jnode_.at("tranform_traj_to_body_frame");
  planning_config.set_tranform_traj_to_body_frame(tranform_traj_to_body_frame);

  PlannerPublicRoadConfig planner_public_road_config;
  standard_planning_config.set_planner_public_road_config(
      planner_public_road_config);
  planning_config.set_standard_planning_config(standard_planning_config);
  PlanningGflags::setPlanningConfig(jnode_);
}

void Config::SetDefaultTaskConfig(PlanningConfig &planning_config) {
  // default_task_config
  for (auto &default_task_config : jnode_.at("default_task_config")) {
    TaskConfig task_config;
    std::string task_type;
    default_task_config.at("task_type").get_to(task_type);
    auto task_type_enum = EnumParser<TaskConfig::TaskType>::Parse(task_type);
    SwitchTaskConfig(task_config, default_task_config, task_type_enum);
    task_config.set_task_config(task_config_);
    task_config.set_task_type(task_type_enum);
    planning_config.add_default_task_config(task_config);
  }
}

void Config::SwitchTaskConfig(TaskConfig &task_config,
                              const nlohmann::json jnode,
                              const TaskConfig::TaskType task_type) {
  switch (task_type) {
    case TaskConfig::TRAFFIC_RULE_DECIDER:
      if (jnode.count("traffic_rule_decider_config")) {
        task_config_.traffic_rule_decider_config = SetTrafficRuleDeciderConfig(
            jnode.at("traffic_rule_decider_config"));
      }
      break;
    case TaskConfig::MISSION_DECIDER:
      if (jnode.count("mission_decider_config")) {
        task_config_.mission_decider_config =
            SetMissionDeciderConfig(jnode.at("mission_decider_config"));
      }
      break;
    case TaskConfig::LATERAL_DECIDER:
      if (jnode.count("lateral_decider_config")) {
        const auto &lateralDeciderConfig = jnode.at("lateral_decider_config");
        auto SetFrenetTrajectorySamplerConfig =
            [](const nlohmann::json jnode,
               zark::planning::LateralDeciderConfig::FrenetTrajectorySampler
                   &target) {
              const auto &cost_lon = jnode.at("cost_lon");
              const auto &cost_lat = jnode.at("cost_lat");
              double dt;
              jnode.at("dt").get_to(dt);
              target.dt = dt;
              double dT;
              jnode.at("dT").get_to(dT);
              target.dT = dT;
              double t_total;
              jnode.at("t_total").get_to(t_total);
              target.t_total = t_total;
              double t_end;
              jnode.at("t_end").get_to(t_end);
              target.t_end = t_end;
              double k_lon;
              cost_lon.at("k").get_to(k_lon);
              target.cost_lon.k = k_lon;
              double k_j_lon;
              cost_lon.at("k_j").get_to(k_j_lon);
              target.cost_lon.k_j = k_j_lon;
              double k_T_lon;
              cost_lon.at("k_T").get_to(k_T_lon);
              target.cost_lon.k_T = k_T_lon;
              double k_lat;
              cost_lat.at("k").get_to(k_lat);
              target.cost_lat.k = k_lat;
              double k_j_lat;
              cost_lat.at("k_j").get_to(k_j_lat);
              target.cost_lat.k_j = k_j_lat;
              double k_T_lat;
              cost_lat.at("k_T").get_to(k_T_lat);
              target.cost_lat.k_T = k_T_lat;
              double k_os_lat;
              cost_lat.at("k_overshoot").get_to(k_os_lat);
              target.cost_lat.k_overshoot = k_os_lat;
              double k_md_lat;
              cost_lat.at("k_deviation").get_to(k_md_lat);
              target.cost_lat.k_deviation = k_md_lat;
            };
        SetFrenetTrajectorySamplerConfig(
            lateralDeciderConfig.at("frenet_trajectory_sampler_lk"),
            task_config_.lateral_decider_config.frenet_trajectory_sampler_lk);
        SetFrenetTrajectorySamplerConfig(
            lateralDeciderConfig.at("frenet_trajectory_sampler_lc"),
            task_config_.lateral_decider_config.frenet_trajectory_sampler_lc);
        SetFrenetTrajectorySamplerConfig(
            lateralDeciderConfig.at("frenet_trajectory_sampler_nudge"),
            task_config_.lateral_decider_config
                .frenet_trajectory_sampler_nudge);
        const auto &corridor = lateralDeciderConfig.at("corridor");

        double t_rear;
        corridor.at("t_rear").get_to(t_rear);
        task_config_.lateral_decider_config.corridor.t_rear = t_rear;
        double extra_half_width;
        corridor.at("extra_half_width").get_to(extra_half_width);
        task_config_.lateral_decider_config.corridor.extra_half_width =
            extra_half_width;
        double length_max;
        corridor.at("length_max").get_to(length_max);
        task_config_.lateral_decider_config.corridor.length_max = length_max;
        double length_min;
        corridor.at("length_min").get_to(length_min);
        task_config_.lateral_decider_config.corridor.length_min = length_min;
        double t_total_corridor;
        corridor.at("t_total").get_to(t_total_corridor);
        task_config_.lateral_decider_config.corridor.t_total = t_total_corridor;
        double v_min;
        corridor.at("v_min").get_to(v_min);
        task_config_.lateral_decider_config.corridor.v_min = v_min;
        double t_staging;
        corridor.at("t_staging").get_to(t_staging);
        task_config_.lateral_decider_config.corridor.t_staging = t_staging;
        const auto &nudge = corridor.at("nudge");
        uint8_t hysteresis_cycle_number;
        nudge.at("hysteresis_cycle_number").get_to(hysteresis_cycle_number);
        task_config_.lateral_decider_config.corridor.hysteresis_cycle_number =
            hysteresis_cycle_number;
        double delta_l_target;
        nudge.at("delta_l_target").get_to(delta_l_target);
        task_config_.lateral_decider_config.corridor.delta_l_target =
            delta_l_target;
        uint8_t num_l_target;
        nudge.at("num_l_target").get_to(num_l_target);
        task_config_.lateral_decider_config.corridor.num_l_target =
            num_l_target;
        double v_obs_min_to_enable_nudge;
        nudge.at("v_obs_min_to_enable_nudge").get_to(v_obs_min_to_enable_nudge);
        task_config_.lateral_decider_config.corridor.v_obs_min_to_enable_nudge =
            v_obs_min_to_enable_nudge;
        double v_relative_min_to_enable_nudge;
        nudge.at("v_relative_min_to_enable_nudge")
            .get_to(v_relative_min_to_enable_nudge);
        task_config_.lateral_decider_config.corridor
            .v_relative_min_to_enable_nudge = v_relative_min_to_enable_nudge;
        double s_distance_thres;
        corridor.at("s_distance_thres").get_to(s_distance_thres);
        task_config_.lateral_decider_config.corridor.s_distance_thres =
            s_distance_thres;
        double lane_width_thres;
        corridor.at("lane_width_thres").get_to(lane_width_thres);
        task_config_.lateral_decider_config.corridor.lane_width_thres =
            lane_width_thres;
        double obs_collision_buffer;
        corridor.at("obs_collision_buffer").get_to(obs_collision_buffer);
        task_config_.lateral_decider_config.corridor.obs_collision_buffer =
            obs_collision_buffer;
        double t_obs_collision_check;
        corridor.at("t_obs_collision_check").get_to(t_obs_collision_check);
        task_config_.lateral_decider_config.corridor.t_obs_collision_check =
            t_obs_collision_check;
        double sample_dist_min;
        corridor.at("sample_dist_min").get_to(sample_dist_min);
        task_config_.lateral_decider_config.corridor.sample_dist_min =
            sample_dist_min;
        double t_forward_by_curvature_spd_limit;
        corridor.at("t_forward_by_curvature_spd_limit")
            .get_to(t_forward_by_curvature_spd_limit);
        task_config_.lateral_decider_config.corridor
            .t_forward_by_curvature_spd_limit =
            t_forward_by_curvature_spd_limit;
        const auto &obstacle_filter = corridor.at("obstacle_filter");
        double s_distance_obstacle_need_ignore;
        obstacle_filter.at("s_distance_obstacle_need_ignore")
            .get_to(s_distance_obstacle_need_ignore);
        task_config_.lateral_decider_config.corridor
            .s_distance_obstacle_need_ignore = s_distance_obstacle_need_ignore;
        double a_need_ignore;
        obstacle_filter.at("a_need_ignore").get_to(a_need_ignore);
        task_config_.lateral_decider_config.corridor.a_need_ignore =
            a_need_ignore;
        task_config_.lateral_decider_config.corridor.v_target_coeff_table =
            GetLookupTableConfig(corridor.at("v_target_coeff_table"));
        task_config_.lateral_decider_config.corridor.a_lat_max_table =
            GetLookupTableConfig(corridor.at("a_lat_max_table"));
        corridor.at("obs_collision_buffer_table")
            .at("regular_car")
            .get_to(task_config_.lateral_decider_config.corridor
                        .obs_collision_buffer_table.regular_car);
        corridor.at("obs_collision_buffer_table")
            .at("large_car")
            .get_to(task_config_.lateral_decider_config.corridor
                        .obs_collision_buffer_table.large_car);
        corridor.at("obs_collision_buffer_table")
            .at("others")
            .get_to(task_config_.lateral_decider_config.corridor
                        .obs_collision_buffer_table.others);
      }
      break;
    case TaskConfig::LONGITUDINAL_DECIDER:
      if (jnode.count("longitudinal_decider_config")) {
        const auto &longitudinaDeciderConfig =
            jnode.at("longitudinal_decider_config");

        const auto &speedLimitBuilder =
            longitudinaDeciderConfig.at("speed_limit_builder");
        task_config_.longitudinal_decider_config.speed_limit_config
            .a_lat_max_table =
            GetLookupTableConfig(speedLimitBuilder.at("a_lat_max_table"));
        double v_max;
        speedLimitBuilder.at("v_max").get_to(v_max);
        task_config_.longitudinal_decider_config.speed_limit_config.v_max =
            v_max;
        double t_sliding_window;
        speedLimitBuilder.at("t_sliding_window").get_to(t_sliding_window);
        task_config_.longitudinal_decider_config.speed_limit_config
            .t_sliding_window = t_sliding_window;
        const auto &stTopologyConfig =
            longitudinaDeciderConfig.at("st_topology_config");
        double alpha;
        stTopologyConfig.at("alpha").get_to(alpha);
        task_config_.longitudinal_decider_config.st_topology_config.alpha =
            alpha;

        const auto &stProposalConfig =
            longitudinaDeciderConfig.at("st_proposal_config");
        int max_num_proposals;
        stProposalConfig.at("max_num_proposals").get_to(max_num_proposals);
        double w_front;
        stProposalConfig.at("w_front").get_to(w_front);
        double w_rear;
        stProposalConfig.at("w_rear").get_to(w_rear);
        double a_min_stiff;
        stProposalConfig.at("a_min_stiff").get_to(a_min_stiff);
        double cost_max;
        stProposalConfig.at("cost_max").get_to(cost_max);
        double t_start_blocking_max;
        stProposalConfig.at("t_start_blocking_max")
            .get_to(t_start_blocking_max);
        double a_decel_limit;
        stProposalConfig.at("a_decel_limit").get_to(a_decel_limit);
        task_config_.longitudinal_decider_config.st_proposal_config
            .narrow_gap_cost_table =
            GetLookupTableConfig(stProposalConfig.at("narrow_gap_cost_table"));

        task_config_.longitudinal_decider_config.st_proposal_config
            .max_num_proposals = max_num_proposals;
        task_config_.longitudinal_decider_config.st_proposal_config.w_front =
            w_front;
        task_config_.longitudinal_decider_config.st_proposal_config.w_rear =
            w_rear;
        task_config_.longitudinal_decider_config.st_proposal_config
            .a_min_stiff = a_min_stiff;
        task_config_.longitudinal_decider_config.st_proposal_config.cost_max =
            cost_max;
        task_config_.longitudinal_decider_config.st_proposal_config
            .t_start_blocking_max = t_start_blocking_max;
        task_config_.longitudinal_decider_config.st_proposal_config
            .a_decel_limit = a_decel_limit;
        task_config_.longitudinal_decider_config.st_proposal_config
            .a_acc_max_table =
            GetLookupTableConfig(stProposalConfig.at("a_accel_max_table"));

        const auto &stGraphConfig =
            longitudinaDeciderConfig.at("st_graph_config");
        double lateral_buffer;
        stGraphConfig.at("lateral_buffer").get_to(lateral_buffer);
        double safety_l_buffer;
        stGraphConfig.at("safety_l_buffer").get_to(safety_l_buffer);
        bool use_obs_sl_for_st_graph;
        stGraphConfig.at("use_obs_sl_for_st_graph")
            .get_to(use_obs_sl_for_st_graph);
        double t_plan;
        stGraphConfig.at("t_plan").get_to(t_plan);
        double fully_blocking_buffer;
        stGraphConfig.at("fully_blocking_buffer").get_to(fully_blocking_buffer);
        int st_boundary_counter_min;
        stGraphConfig.at("st_boundary_counter_min")
            .get_to(st_boundary_counter_min);
        double delta_t_oncoming_obs;
        stGraphConfig.at("delta_t_oncoming_obs").get_to(delta_t_oncoming_obs);
        task_config_.longitudinal_decider_config.st_graph_config
            .lateral_buffer = lateral_buffer;
        task_config_.longitudinal_decider_config.st_graph_config
            .safety_l_buffer = safety_l_buffer;
        task_config_.longitudinal_decider_config.st_graph_config
            .use_obs_sl_for_st_graph = use_obs_sl_for_st_graph;
        task_config_.longitudinal_decider_config.st_graph_config.t_plan =
            t_plan;
        task_config_.longitudinal_decider_config.st_graph_config
            .fully_blocking_buffer = fully_blocking_buffer;
        task_config_.longitudinal_decider_config.st_graph_config
            .st_boundary_counter_min = st_boundary_counter_min;
        task_config_.longitudinal_decider_config.st_graph_config
            .delta_t_oncoming_obs = delta_t_oncoming_obs;
      }
      break;
    case TaskConfig::LONGITUDINAL_OPTIMIZER: {
      if (jnode.count("longitudinal_optimizer_config")) {
        task_config_.longitudinal_optimizer_config =
            SetLongitudinalOptimizerConfig(
                jnode.at("longitudinal_optimizer_config"));
      }
      break;
    }
    case TaskConfig::LATERAL_OPTIMIZER: {
      if (jnode.count("lateral_optimizer_config")) {
        task_config_.lateral_optimizer_config =
            SetLateralOptimizerConfig(jnode.at("lateral_optimizer_config"));
      }
      break;
    }
    case TaskConfig::EVALUATION_DECIDER: {
      if (jnode.count("evaluation_decider_config")) {
        task_config_.evaluation_decider_config =
            SetEvaluationDeciderConfig(jnode.at("evaluation_decider_config"));
      }
    } break;
    default:
      break;
  }
}

void Config::SetVehicleConfig(common::VehicleConfig &config) {
  // header
  ::common::Header header;

  double timestamp_sec;
  jnode_.at("header").at("timestamp_sec").get_to(timestamp_sec);
  header.set_timestamp_sec(timestamp_sec);

  std::string module_name;
  jnode_.at("header").at("module_name").get_to(module_name);
  header.set_module_name(module_name);

  uint32_t sequence_num;
  jnode_.at("header").at("sequence_num").get_to(sequence_num);
  header.set_sequence_num(sequence_num);

  uint64_t lidar_timestamp;
  jnode_.at("header").at("lidar_timestamp").get_to(lidar_timestamp);
  header.set_lidar_timestamp(lidar_timestamp);

  uint64_t camera_timestamp;
  jnode_.at("header").at("camera_timestamp").get_to(camera_timestamp);
  header.set_camera_timestamp(camera_timestamp);

  uint64_t radar_timestamp;
  jnode_.at("header").at("radar_timestamp").get_to(radar_timestamp);
  header.set_timestamp_sec(radar_timestamp);

  uint32_t version;
  jnode_.at("header").at("version").get_to(version);
  header.set_version(version);

  // header.status
  ::common::StatusPb status;
  int error_code;
  jnode_.at("header").at("status").at("error_code").get_to(error_code);
  status.set_error_code(::common::ErrorCode(error_code));

  std::string msg;
  jnode_.at("header").at("status").at("msg").get_to(msg);
  status.set_msg(msg);
  header.set_status(status);

  std::string frame_id;
  jnode_.at("header").at("frame_id").get_to(frame_id);
  header.set_frame_id(frame_id);
  config.set_header(header);

  // vehicle param
  common::VehicleParam vehicle_param;
  int brand;
  jnode_.at("vehicle_param").at("brand").get_to(brand);
  vehicle_param.set_brand(common::VehicleBrand(brand));

  double front_edge_to_center;
  jnode_.at("vehicle_param")
      .at("front_edge_to_center")
      .get_to(front_edge_to_center);
  vehicle_param.set_front_edge_to_center(front_edge_to_center);

  double back_edge_to_center;
  jnode_.at("vehicle_param")
      .at("back_edge_to_center")
      .get_to(back_edge_to_center);
  vehicle_param.set_back_edge_to_center(back_edge_to_center);

  double left_edge_to_center;
  jnode_.at("vehicle_param")
      .at("left_edge_to_center")
      .get_to(left_edge_to_center);
  vehicle_param.set_left_edge_to_center(left_edge_to_center);

  double right_edge_to_center;
  jnode_.at("vehicle_param")
      .at("right_edge_to_center")
      .get_to(right_edge_to_center);
  vehicle_param.set_right_edge_to_center(right_edge_to_center);

  double length;
  jnode_.at("vehicle_param").at("length").get_to(length);
  vehicle_param.set_length(length);

  double width;
  jnode_.at("vehicle_param").at("width").get_to(width);
  vehicle_param.set_width(width);

  double height;
  jnode_.at("vehicle_param").at("height").get_to(height);
  vehicle_param.set_height(height);

  double rear_axle_to_cg;
  jnode_.at("vehicle_param").at("rear_axle_to_cg").get_to(rear_axle_to_cg);
  vehicle_param.set_rear_axle_to_cg(rear_axle_to_cg);

  double min_turn_radius;
  jnode_.at("vehicle_param").at("min_turn_radius").get_to(min_turn_radius);
  vehicle_param.set_min_turn_radius(min_turn_radius);

  double max_acceleration;
  jnode_.at("vehicle_param").at("max_acceleration").get_to(max_acceleration);
  vehicle_param.set_max_acceleration(max_acceleration);

  double max_deceleration;
  jnode_.at("vehicle_param").at("max_deceleration").get_to(max_deceleration);
  vehicle_param.set_max_deceleration(max_deceleration);

  double max_steer_angle;
  jnode_.at("vehicle_param").at("max_steer_angle").get_to(max_steer_angle);
  vehicle_param.set_max_steer_angle(max_steer_angle);

  double max_steer_angle_rate;
  jnode_.at("vehicle_param")
      .at("max_steer_angle_rate")
      .get_to(max_steer_angle_rate);
  vehicle_param.set_max_steer_angle_rate(max_steer_angle_rate);

  double steer_ratio;
  jnode_.at("vehicle_param").at("steer_ratio").get_to(steer_ratio);
  vehicle_param.set_steer_ratio(steer_ratio);

  double wheel_base;
  jnode_.at("vehicle_param").at("wheel_base").get_to(wheel_base);
  vehicle_param.set_wheel_base(wheel_base);

  double wheel_rolling_radius;
  jnode_.at("vehicle_param")
      .at("wheel_rolling_radius")
      .get_to(wheel_rolling_radius);
  vehicle_param.set_wheel_rolling_radius(wheel_rolling_radius);

  double front_wheel_corner_stiffness;
  jnode_.at("vehicle_param")
      .at("front_wheel_corner_stiffness")
      .get_to(front_wheel_corner_stiffness);
  vehicle_param.set_front_wheel_corner_stiffness(front_wheel_corner_stiffness);

  double rear_wheel_corner_stiffness;
  jnode_.at("vehicle_param")
      .at("rear_wheel_corner_stiffness")
      .get_to(rear_wheel_corner_stiffness);
  vehicle_param.set_rear_wheel_corner_stiffness(rear_wheel_corner_stiffness);

  double mass;
  jnode_.at("vehicle_param").at("mass").get_to(mass);
  vehicle_param.set_mass(mass);

  double moment_of_inertia;
  jnode_.at("vehicle_param").at("moment_of_inertia").get_to(moment_of_inertia);
  vehicle_param.set_moment_of_inertia(moment_of_inertia);

  double max_abs_speed_when_stopped;
  jnode_.at("vehicle_param")
      .at("max_abs_speed_when_stopped")
      .get_to(max_abs_speed_when_stopped);
  vehicle_param.set_max_abs_speed_when_stopped(max_abs_speed_when_stopped);

  {
    common::LatencyParam steering_latency_param;
    double dead_time;
    jnode_.at("vehicle_param")
        .at("steering_latency_param")
        .at("dead_time")
        .get_to(dead_time);
    steering_latency_param.set_dead_time(dead_time);

    double rise_time;
    jnode_.at("vehicle_param")
        .at("steering_latency_param")
        .at("rise_time")
        .get_to(rise_time);
    steering_latency_param.set_rise_time(rise_time);

    double peak_time;
    jnode_.at("vehicle_param")
        .at("steering_latency_param")
        .at("peak_time")
        .get_to(peak_time);
    steering_latency_param.set_dead_time(peak_time);

    double settling_time;
    jnode_.at("vehicle_param")
        .at("steering_latency_param")
        .at("settling_time")
        .get_to(settling_time);
    steering_latency_param.set_dead_time(settling_time);

    vehicle_param.set_steering_latency_param(steering_latency_param);
  }
  {
    common::LatencyParam throttle_latency_param;
    double dead_time;
    jnode_.at("vehicle_param")
        .at("steering_latency_param")
        .at("dead_time")
        .get_to(dead_time);
    throttle_latency_param.set_dead_time(dead_time);

    double rise_time;
    jnode_.at("vehicle_param")
        .at("steering_latency_param")
        .at("rise_time")
        .get_to(rise_time);
    throttle_latency_param.set_rise_time(rise_time);

    double peak_time;
    jnode_.at("vehicle_param")
        .at("steering_latency_param")
        .at("peak_time")
        .get_to(peak_time);
    throttle_latency_param.set_dead_time(peak_time);

    double settling_time;
    jnode_.at("vehicle_param")
        .at("steering_latency_param")
        .at("settling_time")
        .get_to(settling_time);
    throttle_latency_param.set_dead_time(settling_time);

    vehicle_param.set_throttle_latency_param(throttle_latency_param);
  }
  {
    common::LatencyParam brake_latency_param;
    double dead_time;
    jnode_.at("vehicle_param")
        .at("steering_latency_param")
        .at("dead_time")
        .get_to(dead_time);
    brake_latency_param.set_dead_time(dead_time);

    double rise_time;
    jnode_.at("vehicle_param")
        .at("steering_latency_param")
        .at("rise_time")
        .get_to(rise_time);
    brake_latency_param.set_rise_time(rise_time);

    double peak_time;
    jnode_.at("vehicle_param")
        .at("steering_latency_param")
        .at("peak_time")
        .get_to(peak_time);
    brake_latency_param.set_dead_time(peak_time);

    double settling_time;
    jnode_.at("vehicle_param")
        .at("steering_latency_param")
        .at("settling_time")
        .get_to(settling_time);
    brake_latency_param.set_dead_time(settling_time);

    vehicle_param.set_throttle_latency_param(brake_latency_param);
  }

  common::Extrinsics extrinsics;
  extrinsics.clear_transform();

  for (auto &trans : jnode_.at("extrinsics").at("transform"))

  {
    common::Transform transform;
    std::string source_frame;
    trans.at("source_frame").get_to(source_frame);
    transform.set_source_frame(source_frame);

    std::string target_frame;
    trans.at("target_frame").get_to(target_frame);
    transform.set_target_frame(target_frame);

    ::common::Point3D translation;
    double x;
    trans.at("translation").at("x").get_to(x);
    translation.set_x(x);

    double y;
    trans.at("translation").at("y").get_to(y);
    translation.set_y(y);

    double z;
    trans.at("translation").at("z").get_to(z);
    translation.set_z(z);

    std::string rotation;
    trans.at("rotation").get_to(rotation);
    transform.set_rotation(rotation);

    extrinsics.add_transform(transform);
  }

  config.set_header(header);
  config.set_vehicle_param(vehicle_param);
  config.set_extrinsics(extrinsics);
}

void Config::SetLocalRouteConfig(LocalRouteConfig &local_route_config) {
  jnode_.at("use_map_state").get_to(local_route_config.use_map_state);
  jnode_.at("self_test_local_route")
      .get_to(local_route_config.self_test_local_route);
  jnode_.at("local_route_speed_limit")
      .get_to(local_route_config.local_route_speed_limit);
  jnode_.at("min_local_route_speed_limit")
      .get_to(local_route_config.min_local_route_speed_limit);
  jnode_.at("local_route_thread_time")
      .get_to(local_route_config.local_route_thread_time);
  jnode_.at("enable_local_route_stitching")
      .get_to(local_route_config.enable_local_route_stitching);
  jnode_.at("enable_smooth_local_route")
      .get_to(local_route_config.enable_smooth_local_route);
  jnode_.at("look_forward_extend_distance")
      .get_to(local_route_config.look_forward_extend_distance);
  jnode_.at("local_route_stitch_overlap_distance")
      .get_to(local_route_config.local_route_stitch_overlap_distance);
  jnode_.at("default_local_route_width")
      .get_to(local_route_config.default_local_route_width);
  jnode_.at("smoothed_local_route_max_diff")
      .get_to(local_route_config.smoothed_local_route_max_diff);
  jnode_.at("default_lane_width").get_to(local_route_config.default_lane_width);
  jnode_.at("look_backward_distance")
      .get_to(local_route_config.look_backward_distance);
  jnode_.at("look_forward_short_distance")
      .get_to(local_route_config.look_forward_short_distance);
  jnode_.at("look_forward_long_distance")
      .get_to(local_route_config.look_forward_long_distance);
  jnode_.at("obs_filter_front_start_s")
      .get_to(local_route_config.obs_filter_front_start_s);
  jnode_.at("obs_filter_front_end_s")
      .get_to(local_route_config.obs_filter_front_end_s);
  jnode_.at("obs_filter_rear_start_s")
      .get_to(local_route_config.obs_filter_rear_start_s);
  jnode_.at("obs_filter_rear_end_s")
      .get_to(local_route_config.obs_filter_rear_end_s);
  jnode_.at("look_forward_time_sec")
      .get_to(local_route_config.look_forward_time_sec);
  jnode_.at("adc_width").get_to(local_route_config.adc_width);
  jnode_.at("fit_points_size").get_to(local_route_config.fit_points_size);
  jnode_.at("noa_line_forward_dis")
      .get_to(local_route_config.noa_line_forward_dis);
  jnode_.at("offline_pack").get_to(local_route_config.offline_pack);
  jnode_.at("keep_trigger_time").get_to(local_route_config.keep_trigger_time);
  jnode_.at("latral_select_distance")
      .get_to(local_route_config.latral_select_distance);
  jnode_.at("NoaLine_section_length")
      .get_to(local_route_config.NoaLine_section_length);
  jnode_.at("max_lane_width").get_to(local_route_config.max_lane_width);
  jnode_.at("normal_lane_width").get_to(local_route_config.normal_lane_width);

  // smoother:
  LocalRouteSmootherConfig smoother_config;

  double max_constraint_interval;
  double longitudinal_boundary_bound;
  double max_lateral_boundary_bound;
  double min_lateral_boundary_bound;
  uint32_t num_of_total_points;
  double curb_shift;
  double lateral_buffer;
  double resolution;

  jnode_.at("LocalRouteSmootherConfig")
      .at("max_constraint_interval")
      .get_to(max_constraint_interval);
  jnode_.at("LocalRouteSmootherConfig")
      .at("longitudinal_boundary_bound")
      .get_to(longitudinal_boundary_bound);
  jnode_.at("LocalRouteSmootherConfig")
      .at("max_lateral_boundary_bound")
      .get_to(max_lateral_boundary_bound);
  jnode_.at("LocalRouteSmootherConfig")
      .at("min_lateral_boundary_bound")
      .get_to(min_lateral_boundary_bound);
  jnode_.at("LocalRouteSmootherConfig")
      .at("num_of_total_points")
      .get_to(num_of_total_points);
  jnode_.at("LocalRouteSmootherConfig")
      .at("lateral_buffer")
      .get_to(lateral_buffer);
  jnode_.at("LocalRouteSmootherConfig").at("curb_shift").get_to(curb_shift);
  jnode_.at("LocalRouteSmootherConfig").at("resolution").get_to(resolution);

  smoother_config.set_max_constraint_interval(max_constraint_interval);
  smoother_config.set_longitudinal_boundary_bound(longitudinal_boundary_bound);
  smoother_config.set_max_lateral_boundary_bound(max_lateral_boundary_bound);
  smoother_config.set_min_lateral_boundary_bound(min_lateral_boundary_bound);
  smoother_config.set_num_of_total_points(num_of_total_points);
  smoother_config.set_curb_shift(curb_shift);
  smoother_config.set_lateral_buffer(lateral_buffer);
  smoother_config.set_curb_shift(resolution);

  std::string smoother_method;
  jnode_.at("LocalRouteSmootherConfig").at("smoother").get_to(smoother_method);
  if (smoother_method == "discrete_points") {
    FemPosDeviationSmootherConfig discrete_smoother;
    DiscretePointsSmootherConfig discrete_points_smoother;
    double weight_fem_pos_deviation;
    double weight_ref_deviation;
    double weight_last_ref_deviation;
    double weight_path_length;
    double curvature_constraint;
    double weight_curvature_constraint_slack_var;
    bool apply_curvature_constraint;
    bool use_sqp;

    jnode_.at("LocalRouteSmootherConfig")
        .at("DiscretePointsSmootherConfig")
        .at("weight_fem_pos_deviation")
        .get_to(weight_fem_pos_deviation);
    jnode_.at("LocalRouteSmootherConfig")
        .at("DiscretePointsSmootherConfig")
        .at("weight_ref_deviation")
        .get_to(weight_ref_deviation);
    jnode_.at("LocalRouteSmootherConfig")
        .at("DiscretePointsSmootherConfig")
        .at("weight_last_ref_deviation")
        .get_to(weight_last_ref_deviation);
    jnode_.at("LocalRouteSmootherConfig")
        .at("DiscretePointsSmootherConfig")
        .at("weight_path_length")
        .get_to(weight_path_length);
    jnode_.at("LocalRouteSmootherConfig")
        .at("DiscretePointsSmootherConfig")
        .at("apply_curvature_constraint")
        .get_to(apply_curvature_constraint);
    jnode_.at("LocalRouteSmootherConfig")
        .at("DiscretePointsSmootherConfig")
        .at("use_sqp")
        .get_to(use_sqp);
    jnode_.at("LocalRouteSmootherConfig")
        .at("DiscretePointsSmootherConfig")
        .at("weight_curvature_constraint_slack_var")
        .get_to(weight_curvature_constraint_slack_var);
    jnode_.at("LocalRouteSmootherConfig")
        .at("DiscretePointsSmootherConfig")
        .at("curvature_constraint")
        .get_to(curvature_constraint);

    discrete_smoother.set_weight_fem_pos_deviation(weight_fem_pos_deviation);
    discrete_smoother.set_weight_ref_deviation(weight_ref_deviation);
    discrete_smoother.set_last_weight_ref_deviation(weight_last_ref_deviation);
    discrete_smoother.set_weight_path_length(weight_path_length);
    discrete_smoother.set_apply_curvature_constraint(
        apply_curvature_constraint);
    discrete_smoother.set_use_sqp(use_sqp);
    discrete_smoother.set_curvature_constraint(curvature_constraint);
    discrete_smoother.set_weight_curvature_constraint_slack_var(
        weight_curvature_constraint_slack_var);
    discrete_points_smoother.set_fem_pos_deviation_smoothing(discrete_smoother);
    smoother_config.set_discrete_points(discrete_points_smoother);
  }
  local_route_config.smoother_config = smoother_config;

  ACCRouteConfig acc_route_config;
  ACCTrajectoryBuilderConfig acc_trajectory_builder_config;
  double t_toal;
  jnode_.at("ACCRouteConfig")
      .at("acc_trajectory_builder")
      .at("t_toal")
      .get_to(t_toal);

  acc_trajectory_builder_config.set_t_total(t_toal);
  acc_route_config.set_acc_trajectory_builder_config(
      acc_trajectory_builder_config);
  local_route_config.acc_route_config = acc_route_config;
}

LookupTableConfig Config::GetLookupTableConfig(
    const nlohmann::json &table_jnode) {
  LookupTableConfig lookup_table_config;
  std::vector<double> points_x;
  std::vector<double> points_y;
  for (const auto &pt : table_jnode) {
    points_x.push_back(pt[0]);
    points_y.push_back(pt[1]);
  }
  lookup_table_config.points_x = points_x;
  lookup_table_config.points_y = points_y;
  return lookup_table_config;
}

TrafficRuleDeciderConfig Config::SetTrafficRuleDeciderConfig(
    const nlohmann::json &jnode) {
  TrafficRuleDeciderConfig traffic_rule_decider_config;
  const auto &jnode_rule_destination = jnode.at("rule_destination");
  jnode_rule_destination.at("enable").get_to(
      traffic_rule_decider_config.destination.enable);

  const auto &jnode_rule_crosswalk = jnode.at("rule_crosswalk");
  jnode_rule_crosswalk.at("enable").get_to(
      traffic_rule_decider_config.crosswalk.enable);

  const auto &jnode_rule_traffic_light = jnode.at("rule_traffic_light");
  jnode_rule_traffic_light.at("enable").get_to(
      traffic_rule_decider_config.traffic_light.enable);

  return traffic_rule_decider_config;
}

MissionDeciderConfig Config::SetMissionDeciderConfig(
    const nlohmann::json &jnode) {
  MissionDeciderConfig mission_decider_config;
  const auto &envInfo = jnode.at("env_info");
  envInfo.at("invalid_lane")
      .get_to(mission_decider_config.env_info.invalid_lane);
  envInfo.at("epsilon").get_to(mission_decider_config.env_info.epsilon);
  envInfo.at("force_lane_block_cnt")
      .get_to(mission_decider_config.env_info.force_lane_block_cnt);
  envInfo.at("default_spd_limit")
      .get_to(mission_decider_config.env_info.default_spd_limit);
  envInfo.at("comf_low_spd_ratio_gate")
      .get_to(mission_decider_config.env_info.comf_low_spd_ratio_gate);
  envInfo.at("default_lc_finish_time")
      .get_to(mission_decider_config.env_info.default_lc_finish_time);
  envInfo.at("default_lc_congestion_coeff")
      .get_to(mission_decider_config.env_info.default_lc_congestion_coeff);

  const auto &lc_request = jnode.at("lc_request");
  lc_request.at("freeze_time")
      .get_to(mission_decider_config.lc_request.freeze_time);
  lc_request.at("enable_mannual_lc")
      .get_to(mission_decider_config.lc_request.enable_mannual_lc);
  lc_request.at("enable_force_lc")
      .get_to(mission_decider_config.lc_request.enable_force_lc);
  lc_request.at("enable_effi_lc")
      .get_to(mission_decider_config.lc_request.enable_effi_lc);
  lc_request.at("enable_hazard_right_lc")
      .get_to(mission_decider_config.lc_request.enable_hazard_right_lc);
  lc_request.at("forc_distance_buffer")
      .get_to(mission_decider_config.lc_request.forc_distance_buffer);
  lc_request.at("forc_fast_pass_speed_rate")
      .get_to(mission_decider_config.lc_request.forc_fast_pass_speed_rate);
  lc_request.at("forc_min_pass_speed")
      .get_to(mission_decider_config.lc_request.forc_min_pass_speed);
  lc_request.at("confirm_cur_hazard_time")
      .get_to(mission_decider_config.lc_request.confirm_cur_hazard_time);
  lc_request.at("confirm_hazard_clear_time")
      .get_to(mission_decider_config.lc_request.confirm_hazard_clear_time);
  lc_request.at("speed_upper_bound")
      .get_to(mission_decider_config.lc_request.speed_upper_bound);
  lc_request.at("effi_lc_speed_rate")
      .get_to(mission_decider_config.lc_request.effi_lc_speed_rate);
  lc_request.at("confirm_effi_lane_time")
      .get_to(mission_decider_config.lc_request.confirm_effi_lane_time);
  lc_request.at("front_ignore_dis")
      .get_to(mission_decider_config.lc_request.front_ignore_dis);
  lc_request.at("front_ignore_time")
      .get_to(mission_decider_config.lc_request.front_ignore_time);
  lc_request.at("front_low_spd_confirm_time")
      .get_to(mission_decider_config.lc_request.front_low_spd_confirm_time);
  lc_request.at("urgency_max")
      .get_to(mission_decider_config.lc_request.urgency_max);
  lc_request.at("urgency_min")
      .get_to(mission_decider_config.lc_request.urgency_min);
  lc_request.at("waiting_time")
      .get_to(mission_decider_config.lc_request.waiting_time);
  lc_request.at("ramp_size")
      .get_to(mission_decider_config.lc_request.ramp_size);
  lc_request.at("lc_look_forward_dis")
      .get_to(mission_decider_config.lc_request.lc_look_forward_dis);

  return mission_decider_config;
}

LongitudinalOptimizerConfig Config::SetLongitudinalOptimizerConfig(
    const nlohmann::json &jnode) {
  LongitudinalOptimizerConfig config;
  // Model
  jnode.at("model").at("dt").get_to(config.model.dt);
  jnode.at("model").at("num_steps").get_to(config.model.num_steps);
  jnode.at("model").at("num_states").get_to(config.model.num_states);
  jnode.at("model").at("num_ctrls").get_to(config.model.num_ctrls);
  jnode.at("model").at("num_ctrl_rates").get_to(config.model.num_ctrl_rates);

  // Reference
  jnode.at("ref")
      .at("a_decel_speed_limit")
      .get_to(config.ref.a_decel_speed_limit);
  jnode.at("ref").at("a_decel_obs").get_to(config.ref.a_decel_obs);
  config.ref.a_accel_table =
      GetLookupTableConfig(jnode.at("ref").at("a_accel_table"));

  // Constraint
  jnode.at("constraint")
      .at("front_stiff_padding_table")
      .at("regular_car")
      .get_to(config.con.front_stiff_padding_table.regular_car);
  jnode.at("constraint")
      .at("front_stiff_padding_table")
      .at("large_car")
      .get_to(config.con.front_stiff_padding_table.large_car);
  jnode.at("constraint")
      .at("front_stiff_padding_table")
      .at("bicycle")
      .get_to(config.con.front_stiff_padding_table.bicycle);
  jnode.at("constraint")
      .at("front_stiff_padding_table")
      .at("pedestrian")
      .get_to(config.con.front_stiff_padding_table.pedestrian);
  jnode.at("constraint")
      .at("front_stiff_padding_table")
      .at("virtual_fence")
      .get_to(config.con.front_stiff_padding_table.virtual_fence);
  jnode.at("constraint")
      .at("front_stiff_padding_table")
      .at("others")
      .get_to(config.con.front_stiff_padding_table.others);
  jnode.at("constraint")
      .at("rear_stiff_padding_table")
      .at("regular_car")
      .get_to(config.con.rear_stiff_padding_table.regular_car);
  jnode.at("constraint")
      .at("rear_stiff_padding_table")
      .at("large_car")
      .get_to(config.con.rear_stiff_padding_table.large_car);
  jnode.at("constraint")
      .at("rear_stiff_padding_table")
      .at("bicycle")
      .get_to(config.con.rear_stiff_padding_table.bicycle);
  jnode.at("constraint")
      .at("rear_stiff_padding_table")
      .at("pedestrian")
      .get_to(config.con.rear_stiff_padding_table.pedestrian);
  jnode.at("constraint")
      .at("rear_stiff_padding_table")
      .at("virtual_fence")
      .get_to(config.con.rear_stiff_padding_table.virtual_fence);
  jnode.at("constraint")
      .at("rear_stiff_padding_table")
      .at("others")
      .get_to(config.con.rear_stiff_padding_table.others);

  config.con.front_padding_speed_tolerance_table = GetLookupTableConfig(
      jnode.at("constraint").at("front_padding_speed_tolerance_table"));
  config.con.rear_padding_speed_tolerance_table = GetLookupTableConfig(
      jnode.at("constraint").at("rear_padding_speed_tolerance_table"));
  config.con.front_padding_relative_speed_multiplier_table =
      GetLookupTableConfig(
          jnode.at("constraint")
              .at("front_padding_relative_speed_multiplier_table"));
  config.con.rear_padding_relative_speed_multiplier_table =
      GetLookupTableConfig(
          jnode.at("constraint")
              .at("rear_padding_relative_speed_multiplier_table"));
  config.con.front_soft_padding_table = GetLookupTableConfig(
      jnode.at("constraint").at("front_soft_padding_table"));
  config.con.rear_soft_padding_table = GetLookupTableConfig(
      jnode.at("constraint").at("rear_soft_padding_table"));
  jnode.at("constraint")
      .at("v_soft_to_stiff")
      .get_to(config.con.v_soft_to_stiff);
  config.con.a_max_stiff_table =
      GetLookupTableConfig(jnode.at("constraint").at("a_max_stiff_table"));
  config.con.a_max_soft_table =
      GetLookupTableConfig(jnode.at("constraint").at("a_max_soft_table"));
  jnode.at("constraint").at("a_min_stiff").get_to(config.con.a_min_stiff);
  config.con.a_min_soft_table =
      GetLookupTableConfig(jnode.at("constraint").at("a_min_soft_table"));
  jnode.at("constraint")
      .at("a_decel_speed_limit_violation")
      .get_to(config.con.a_decel_speed_limit_violation);
  jnode.at("constraint").at("j_max_stiff").get_to(config.con.j_max_stiff);
  jnode.at("constraint").at("j_max_soft").get_to(config.con.j_max_soft);
  jnode.at("constraint").at("j_min_stiff").get_to(config.con.j_min_stiff);
  jnode.at("constraint").at("j_min_soft").get_to(config.con.j_min_soft);
  jnode.at("constraint")
      .at("constraint_padding_multiplier")
      .get_to(config.con.constraint_padding_multiplier);

  // Cost
  jnode.at("cost").at("Q").get_to(config.cost.Q);
  jnode.at("cost").at("R").get_to(config.cost.R);
  jnode.at("cost").at("R_dot").get_to(config.cost.R_dot);

  jnode.at("cost").at("stiff_min").at("s").get_to(config.cost.stiff_min.s);
  jnode.at("cost").at("stiff_min").at("v").get_to(config.cost.stiff_min.v);
  jnode.at("cost").at("stiff_min").at("a").get_to(config.cost.stiff_min.a);
  jnode.at("cost").at("stiff_min").at("j").get_to(config.cost.stiff_min.j);

  jnode.at("cost").at("soft_min").at("s").get_to(config.cost.soft_min.s);
  jnode.at("cost").at("soft_min").at("v").get_to(config.cost.soft_min.v);
  jnode.at("cost").at("soft_min").at("a").get_to(config.cost.soft_min.a);
  jnode.at("cost").at("soft_min").at("j").get_to(config.cost.soft_min.j);

  jnode.at("cost").at("stiff_max").at("s").get_to(config.cost.stiff_max.s);
  jnode.at("cost").at("stiff_max").at("v").get_to(config.cost.stiff_max.v);
  jnode.at("cost").at("stiff_max").at("a").get_to(config.cost.stiff_max.a);
  jnode.at("cost").at("stiff_max").at("j").get_to(config.cost.stiff_max.j);

  jnode.at("cost").at("soft_max").at("s").get_to(config.cost.soft_max.s);
  jnode.at("cost").at("soft_max").at("v").get_to(config.cost.soft_max.v);
  jnode.at("cost").at("soft_max").at("a").get_to(config.cost.soft_max.a);
  jnode.at("cost").at("soft_max").at("j").get_to(config.cost.soft_max.j);

  // StopHold
  jnode.at("stop_hold").at("s_tol").get_to(config.stop_hold.s_tol);
  jnode.at("stop_hold").at("v_min").get_to(config.stop_hold.v_min);

  return config;
}

LateralOptimizerConfig Config::SetLateralOptimizerConfig(
    const nlohmann::json &jnode) {
  LateralOptimizerConfig config;

  // Model
  jnode.at("model").at("dt").get_to(config.model.dt);
  jnode.at("model").at("num_steps").get_to(config.model.num_steps);
  jnode.at("model").at("num_states").get_to(config.model.num_states);
  jnode.at("model").at("num_ctrls").get_to(config.model.num_ctrls);
  jnode.at("model").at("num_ctrl_rates").get_to(config.model.num_ctrl_rates);

  // Reference
  jnode.at("ref").at("dl_dt").get_to(config.ref.dl_dt);

  // Constraint
  jnode.at("constraint")
      .at("stiff_padding_table")
      .at("regular_car")
      .get_to(config.con.stiff_padding_table.regular_car);
  jnode.at("constraint")
      .at("stiff_padding_table")
      .at("large_car")
      .get_to(config.con.stiff_padding_table.large_car);
  jnode.at("constraint")
      .at("stiff_padding_table")
      .at("bicycle")
      .get_to(config.con.stiff_padding_table.bicycle);
  jnode.at("constraint")
      .at("stiff_padding_table")
      .at("pedestrian")
      .get_to(config.con.stiff_padding_table.pedestrian);
  jnode.at("constraint")
      .at("stiff_padding_table")
      .at("curb")
      .get_to(config.con.stiff_padding_table.curb);
  jnode.at("constraint")
      .at("stiff_padding_table")
      .at("others")
      .get_to(config.con.stiff_padding_table.others);
  config.con.soft_padding_table =
      GetLookupTableConfig(jnode.at("constraint").at("soft_padding_table"));
  config.con.l_stiff_buffer = jnode.at("constraint").at("l_stiff_buffer");
  config.con.l_soft_buffer = jnode.at("constraint").at("l_soft_buffer");

  // Cost
  jnode.at("cost").at("Q").get_to(config.cost.Q);
  jnode.at("cost").at("R").get_to(config.cost.R);
  jnode.at("cost").at("R_dot").get_to(config.cost.R_dot);
  jnode.at("cost").at("w_terminal").get_to(config.cost.w_terminal);

  jnode.at("cost").at("stiff_min").at("l").get_to(config.cost.stiff_min.l);
  jnode.at("cost")
      .at("stiff_min")
      .at("l_dot")
      .get_to(config.cost.stiff_min.l_dot);
  jnode.at("cost").at("stiff_min").at("psi").get_to(config.cost.stiff_min.psi);
  jnode.at("cost")
      .at("stiff_min")
      .at("psi_dot")
      .get_to(config.cost.stiff_min.psi_dot);
  jnode.at("cost")
      .at("stiff_min")
      .at("delta")
      .get_to(config.cost.stiff_min.delta);
  jnode.at("cost")
      .at("stiff_min")
      .at("delta_dot")
      .get_to(config.cost.stiff_min.delta_dot);

  jnode.at("cost").at("stiff_max").at("l").get_to(config.cost.stiff_max.l);
  jnode.at("cost")
      .at("stiff_max")
      .at("l_dot")
      .get_to(config.cost.stiff_max.l_dot);
  jnode.at("cost").at("stiff_max").at("psi").get_to(config.cost.stiff_max.psi);
  jnode.at("cost")
      .at("stiff_max")
      .at("psi_dot")
      .get_to(config.cost.stiff_max.psi_dot);
  jnode.at("cost")
      .at("stiff_max")
      .at("delta")
      .get_to(config.cost.stiff_max.delta);
  jnode.at("cost")
      .at("stiff_max")
      .at("delta_dot")
      .get_to(config.cost.stiff_max.delta_dot);

  jnode.at("cost").at("soft_min").at("l").get_to(config.cost.soft_min.l);
  jnode.at("cost")
      .at("soft_min")
      .at("l_dot")
      .get_to(config.cost.soft_min.l_dot);
  jnode.at("cost").at("soft_min").at("psi").get_to(config.cost.soft_min.psi);
  jnode.at("cost")
      .at("soft_min")
      .at("psi_dot")
      .get_to(config.cost.soft_min.psi_dot);
  jnode.at("cost")
      .at("soft_min")
      .at("delta")
      .get_to(config.cost.soft_min.delta);
  jnode.at("cost")
      .at("soft_min")
      .at("delta_dot")
      .get_to(config.cost.soft_min.delta_dot);

  jnode.at("cost").at("soft_max").at("l").get_to(config.cost.soft_max.l);
  jnode.at("cost")
      .at("soft_max")
      .at("l_dot")
      .get_to(config.cost.soft_max.l_dot);
  jnode.at("cost").at("soft_max").at("psi").get_to(config.cost.soft_max.psi);
  jnode.at("cost")
      .at("soft_max")
      .at("psi_dot")
      .get_to(config.cost.soft_max.psi_dot);
  jnode.at("cost")
      .at("soft_max")
      .at("delta")
      .get_to(config.cost.soft_max.delta);
  jnode.at("cost")
      .at("soft_max")
      .at("delta_dot")
      .get_to(config.cost.soft_max.delta_dot);

  return config;
}

EvaluationDeciderConfig Config::SetEvaluationDeciderConfig(
    const nlohmann::json &jnode) {
  EvaluationDeciderConfig config;
  std::vector<double> weight_lat_slack_x, weight_lat_slack_u,
      weight_lat_slack_u_dot, weight_lon_slack_x, weight_lon_slack_u,
      weight_lon_slack_u_dot;
  jnode.at("safety")
      .at("weight_lat_slack")
      .at("weight_x")
      .get_to(weight_lat_slack_x);
  jnode.at("safety")
      .at("weight_lat_slack")
      .at("weight_u")
      .get_to(weight_lat_slack_u);
  jnode.at("safety")
      .at("weight_lat_slack")
      .at("weight_u_dot")
      .get_to(weight_lat_slack_u_dot);
  jnode.at("safety")
      .at("weight_lon_slack")
      .at("weight_x")
      .get_to(weight_lon_slack_x);
  jnode.at("safety")
      .at("weight_lon_slack")
      .at("weight_u")
      .get_to(weight_lon_slack_u);
  jnode.at("safety")
      .at("weight_lon_slack")
      .at("weight_u_dot")
      .get_to(weight_lon_slack_u_dot);
  config.safety.weight_lat_slack.weight_x = weight_lat_slack_x;
  config.safety.weight_lat_slack.weight_u = weight_lat_slack_u;
  config.safety.weight_lat_slack.weight_u_dot = weight_lat_slack_u_dot;
  config.safety.weight_lon_slack.weight_x = weight_lon_slack_x;
  config.safety.weight_lon_slack.weight_u = weight_lon_slack_u;
  config.safety.weight_lon_slack.weight_u_dot = weight_lon_slack_u_dot;

  jnode.at("comfort")
      .at("weight_lat_slack")
      .at("weight_x")
      .get_to(weight_lat_slack_x);
  jnode.at("comfort")
      .at("weight_lat_slack")
      .at("weight_u")
      .get_to(weight_lat_slack_u);
  jnode.at("comfort")
      .at("weight_lat_slack")
      .at("weight_u_dot")
      .get_to(weight_lat_slack_u_dot);
  jnode.at("comfort")
      .at("weight_lon_slack")
      .at("weight_x")
      .get_to(weight_lon_slack_x);
  jnode.at("comfort")
      .at("weight_lon_slack")
      .at("weight_u")
      .get_to(weight_lon_slack_u);
  jnode.at("comfort")
      .at("weight_lon_slack")
      .at("weight_u_dot")
      .get_to(weight_lon_slack_u_dot);
  config.comfort.weight_lat_slack.weight_x = weight_lat_slack_x;
  config.comfort.weight_lat_slack.weight_u = weight_lat_slack_u;
  config.comfort.weight_lat_slack.weight_u_dot = weight_lat_slack_u_dot;
  config.comfort.weight_lon_slack.weight_x = weight_lon_slack_x;
  config.comfort.weight_lon_slack.weight_u = weight_lon_slack_u;
  config.comfort.weight_lon_slack.weight_u_dot = weight_lon_slack_u_dot;

  config.progress.progress_cost_table =
      GetLookupTableConfig(jnode.at("progress").at("progress_cost_table"));
  // dangerous safety cost
  jnode.at("safety")
      .at("enable_debug_info")
      .get_to(config.safety.enable_debug_info);
  jnode.at("safety")
      .at("lc_max_collision_dangerous_cost")
      .get_to(config.safety.lc_max_collision_dangerous_cost);
  jnode.at("safety")
      .at("lk_near_side_front_collision_coef")
      .get_to(config.safety.lk_near_side_front_collision_coef);
  jnode.at("safety")
      .at("lk_far_side_front_collision_coef")
      .get_to(config.safety.lk_far_side_front_collision_coef);

  jnode.at("safety")
      .at("dangerous_lc")
      .at("enable")
      .get_to(config.safety.lc_safety.enable);
  jnode.at("safety")
      .at("dangerous_lc")
      .at("max_cost")
      .get_to(config.safety.lc_safety.max_cost);
  jnode.at("safety")
      .at("dangerous_lc")
      .at("min_front_dist")
      .get_to(config.safety.lc_safety.min_front_dist);
  jnode.at("safety")
      .at("dangerous_lc")
      .at("min_back_dist")
      .get_to(config.safety.lc_safety.min_back_dist);
  jnode.at("safety")
      .at("dangerous_lc")
      .at("max_back_dist")
      .get_to(config.safety.lc_safety.max_back_dist);
  jnode.at("safety")
      .at("dangerous_lc")
      .at("delta_v_buf")
      .get_to(config.safety.lc_safety.delta_v_buf);
  jnode.at("safety")
      .at("dangerous_lc")
      .at("delta_v_min")
      .get_to(config.safety.lc_safety.delta_v_min);
  jnode.at("safety")
      .at("dangerous_lc")
      .at("catch_up_t_max")
      .get_to(config.safety.lc_safety.catch_up_t_max);
  // time table
  config.safety.lc_safety.safety_time_table = GetLookupTableConfig(
      jnode.at("safety").at("dangerous_lc").at("safety_time_table"));
  jnode.at("safety")
      .at("dangerous_lc")
      .at("v_coef")
      .get_to(config.safety.lc_safety.v_coef);
  jnode.at("safety")
      .at("dangerous_lc")
      .at("v_coef_velocity")
      .get_to(config.safety.lc_safety.v_coef_velocity);
  jnode.at("safety")
      .at("dangerous_lc")
      .at("filter_front_obs_in_st")
      .get_to(config.safety.lc_safety.filter_front_obs_in_st);
  jnode.at("safety")
      .at("dangerous_lc")
      .at("filter_back_obs_in_st")
      .get_to(config.safety.lc_safety.filter_back_obs_in_st);

  config.safety.safety_cost_table =
      GetLookupTableConfig(jnode.at("safety").at("safety_cost_table"));
  config.safety.safety_lon_cost_table =
      GetLookupTableConfig(jnode.at("safety").at("safety_lon_cost_table"));
  config.safety.safety_lat_cost_table =
      GetLookupTableConfig(jnode.at("safety").at("safety_lat_cost_table"));
  config.safety.collision_time_weight_table = GetLookupTableConfig(
      jnode.at("safety").at("collision_time_weight_table"));
  config.comfort.time_weight_table =
      GetLookupTableConfig(jnode.at("comfort").at("time_weight_table"));
  config.similarity.similarity_cost_table =
      GetLookupTableConfig(jnode.at("similarity").at("similarity_cost_table"));
  double exceed_diverged_pt_cost;
  jnode.at("progress")
      .at("exceed_diverged_pt_cost")
      .get_to(exceed_diverged_pt_cost);
  config.progress.exceed_diverged_pt_cost = exceed_diverged_pt_cost;

  double urgency_cost_weight = 1.0;
  double abortion_cost = 1.0;
  jnode.at("lane_change").at("urgency_cost_weight").get_to(urgency_cost_weight);
  jnode.at("lane_change").at("abortion_cost").get_to(abortion_cost);
  jnode.at("lane_change")
      .at("staging_cost_addition")
      .get_to(config.lane_change.staging_cost_addition);
  jnode.at("lane_change")
      .at("last_lc_finished_cost")
      .get_to(config.lane_change.last_lc_finished_cost);
  jnode.at("lane_change")
      .at("lc_forbidden_cost")
      .get_to(config.lane_change.lc_forbidden_cost);

  config.lane_change.urgency_cost_weight = urgency_cost_weight;
  config.lane_change.abortion_cost = abortion_cost;

  auto &limit = config.cost_limit;
  jnode.at("cost_max_limit").at("safety").get_to(limit.safety_max_cost);
  jnode.at("cost_max_limit").at("progress").get_to(limit.progress_max_cost);
  jnode.at("cost_max_limit").at("comfort").get_to(limit.comfort_max_cost);
  jnode.at("cost_max_limit").at("default").get_to(limit.default_max_cost);

  return config;
}

}  // namespace planning
}  // namespace zark
