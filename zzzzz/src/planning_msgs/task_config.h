/******************************************************************************
 * Copyright 2023 The zpilot Authors Fuhuanhuan. All Rights Reserved.
 *****************************************************************************/
#pragma once

#include <limits>
#include <string>
#include <vector>

namespace zark {
namespace planning {

struct LookupTableConfig {
  std::vector<double> points_x;
  std::vector<double> points_y;
};

struct TrafficRuleDeciderConfig {
  struct TrafficLight {
    bool enable;
  };
  struct Crosswalk {
    bool enable;
  };
  struct Destination {
    bool enable;
  };

  TrafficLight traffic_light;
  Crosswalk crosswalk;
  Destination destination;
};

struct MissionDeciderConfig {
  struct EnvInfo {
    std::string invalid_lane;
    double epsilon;
    int force_lane_block_cnt;
    double default_spd_limit;
    double comf_low_spd_ratio_gate;
    double default_lc_finish_time;
    double default_lc_congestion_coeff;
  };
  struct LCRequest {
    double freeze_time;  // time [s]
    bool enable_mannual_lc;
    bool enable_force_lc;
    bool enable_effi_lc;
    bool enable_hazard_right_lc;
    double forc_distance_buffer;       // dis [m]
    double forc_fast_pass_speed_rate;  // rate []
    double forc_min_pass_speed;        // speed [m/s]
    double confirm_cur_hazard_time;    // time [s]
    double confirm_hazard_clear_time;  // time [s]
    double speed_upper_bound;          // speed [m/s]
    double effi_lc_speed_rate;
    double confirm_effi_lane_time;      // time [s]
    double front_ignore_dis;            // dis [m]
    double front_ignore_time;           // time [s]
    double front_low_spd_confirm_time;  // time [s]
    double urgency_max;
    double urgency_min;
    double waiting_time;  // time [s]
    u_int ramp_size;
    double lc_look_forward_dis;  // dis [m]
  };

  EnvInfo env_info;
  LCRequest lc_request;
};

struct LateralDeciderConfig {
  struct FrenetTrajectorySampler {
    double dt = 0.1;        // sampling time [s]
    double dT = 0.1;        // sampling terminal time [s]
    double t_total = 10.0;  // total trajectory time [s]
    double t_end = 8.0;     // end trajectory time [s]
    struct Cost {
      double k = 1.0;              // total cost scale parameter
      double k_j = 1.0;            // jerk cost parameter
      double k_T = 1.0;            // terminal time cost parameter
      double k_overshoot = 100.0;  // overshoot cost parameter
      double k_deviation = 10.0;   // max deviation cost parameter
    };
    Cost cost_lon;
    Cost cost_lat;
  };

  struct Corridor {
    struct ObstacleCollisionBufferTable {
      double regular_car;
      double large_car;
      double others;
    };

    double t_rear;                // append time behind ego car [s]
    double extra_half_width;      // [m]
    double length_max;            // [m]
    double length_min;            // [m]
    double t_total;               // [s]
    double v_min;                 // [m/s]
    double t_staging = 2.0;       // staging time, must be divisible by dt in
                                  // FrenetTrajectorySampler [s]
    double delta_l_target = 0.5;  // l shift distance for nudge [m]
    uint8_t num_l_target = 2;     // number of nudge targets
    double v_obs_min_to_enable_nudge = 3.0;       // [m/s]
    double v_relative_min_to_enable_nudge = 6.0;  // [m/s]
    double s_distance_thres = 20.0;      // s distance threshold for nudge [m]
    double lane_width_thres = 4.0;       // lane width threshold for nudge [m]
    double obs_collision_buffer = 0.8;   // collision buffer between corridor
                                         // and obstacle [m]
    double t_obs_collision_check = 4.0;  // collision check time [m]
    double sample_dist_min = 2.0;  // minimum distance between samples for read
                                   // and front corridor [m]
    double s_distance_obstacle_need_ignore = 100.0;  // s distancle thres for
                                                     //  ignore obstacle [m]
    double a_need_ignore = -0.5;  // acceleration thres for ignore
                                  // obstacle [m/s^2]
    double t_forward_by_curvature_spd_limit = 3.0;  // forward time for
                                                    // curvature speed limit [s]
    uint8_t hysteresis_cycle_number =
        3;  // hysteresis cycle number for state change
    LookupTableConfig v_target_coeff_table;
    LookupTableConfig a_lat_max_table;
    ObstacleCollisionBufferTable obs_collision_buffer_table;
  };

  FrenetTrajectorySampler frenet_trajectory_sampler_lk;
  FrenetTrajectorySampler frenet_trajectory_sampler_lc;
  FrenetTrajectorySampler frenet_trajectory_sampler_nudge;
  Corridor corridor;
};

struct LongitudinalDeciderConfig {
  struct SpeedLimitConfig {
    LookupTableConfig a_lat_max_table;
    double v_max;
    double t_sliding_window;
  };

  struct STTopologyConfig {
    double alpha;
  };

  struct STProposalConfig {
    int max_num_proposals;
    double w_front;
    double w_rear;
    double a_min_stiff;
    double cost_max;
    double t_start_blocking_max;
    double a_decel_limit;
    LookupTableConfig a_acc_max_table;
    LookupTableConfig narrow_gap_cost_table;
  };

  struct STGraphConfig {
    double lateral_buffer;   //[m]
    double safety_l_buffer;  //[m]
    bool use_obs_sl_for_st_graph;
    double t_plan;                 // [s]
    double fully_blocking_buffer;  //[m]
    int st_boundary_counter_min;
    double delta_t_oncoming_obs;  //[s]
  };

  SpeedLimitConfig speed_limit_config;
  STTopologyConfig st_topology_config;
  STProposalConfig st_proposal_config;
  STGraphConfig st_graph_config;
};

struct LongitudinalOptimizerConfig {
  struct Model {
    double dt;
    int num_steps;
    int num_states;
    int num_ctrls;
    int num_ctrl_rates;
  };

  struct Reference {
    double a_decel_speed_limit;
    double a_decel_obs;
    LookupTableConfig a_accel_table;
  };

  struct Constraint {
    struct StiffPaddingTable {
      double regular_car;
      double large_car;
      double bicycle;
      double pedestrian;
      double virtual_fence;
      double others;
    };
    StiffPaddingTable front_stiff_padding_table;
    StiffPaddingTable rear_stiff_padding_table;
    LookupTableConfig front_padding_speed_tolerance_table;
    LookupTableConfig rear_padding_speed_tolerance_table;
    LookupTableConfig front_padding_relative_speed_multiplier_table;
    LookupTableConfig rear_padding_relative_speed_multiplier_table;
    LookupTableConfig front_soft_padding_table;
    LookupTableConfig rear_soft_padding_table;
    double v_soft_to_stiff;
    LookupTableConfig a_max_stiff_table;
    LookupTableConfig a_max_soft_table;
    double a_min_stiff;
    LookupTableConfig a_min_soft_table;
    double a_decel_speed_limit_violation;
    double j_max_stiff;
    double j_max_soft;
    double j_min_stiff;
    double j_min_soft;
    double constraint_padding_multiplier;
  };

  struct Cost {
    std::vector<double> Q;
    std::vector<double> R;
    std::vector<double> R_dot;

    struct Weight {
      double s;
      double v;
      double a;
      double j;
    };
    Weight stiff_min;
    Weight stiff_max;
    Weight soft_min;
    Weight soft_max;
  };

  struct StopHold {
    double s_tol;  // [m]
    double v_min;  // [m/s]
  };

  Model model;
  Reference ref;
  Constraint con;
  Cost cost;
  StopHold stop_hold;
};

struct LateralOptimizerConfig {
  struct Model {
    double dt;
    int num_steps;
    int num_states;
    int num_ctrls;
    int num_ctrl_rates;
  };
  struct Reference {
    double dl_dt;  // smooth slope [m/s]
  };
  struct Constraint {
    struct StiffPaddingTable {
      double regular_car;
      double large_car;
      double bicycle;
      double pedestrian;
      double curb;
      double others;
    };
    StiffPaddingTable stiff_padding_table;
    LookupTableConfig soft_padding_table;
    double l_stiff_buffer;
    double l_soft_buffer;
  };
  struct Cost {
    std::vector<double> Q;
    std::vector<double> R;
    std::vector<double> R_dot;
    double w_terminal;
    struct Weight {
      double l;
      double l_dot;
      double psi;
      double psi_dot;
      double delta;
      double delta_dot;
    };
    Weight stiff_min;
    Weight stiff_max;
    Weight soft_min;
    Weight soft_max;
  };

  Model model;
  Reference ref;
  Constraint con;
  Cost cost;
};

struct EvaluationDeciderConfig {
  struct SlackWeight {
    std::vector<double> weight_x;
    std::vector<double> weight_u;
    std::vector<double> weight_u_dot;
  };

  struct LCSafety {
    bool enable;
    double max_cost;
    double min_front_dist;
    double min_back_dist;
    double max_back_dist;
    double delta_v_buf;
    double delta_v_min;
    double catch_up_t_max;
    LookupTableConfig safety_time_table;
    double v_coef;
    double v_coef_velocity;
    bool filter_front_obs_in_st;
    bool filter_back_obs_in_st;
  };

  struct Safety {
    bool enable_debug_info;
    double lc_max_collision_dangerous_cost;
    double lk_near_side_front_collision_coef;
    double lk_far_side_front_collision_coef;
    LCSafety lc_safety;
    LookupTableConfig safety_cost_table;
    LookupTableConfig safety_lon_cost_table;
    LookupTableConfig safety_lat_cost_table;
    LookupTableConfig collision_time_weight_table;
    SlackWeight weight_lon_slack;
    SlackWeight weight_lat_slack;
  };
  struct Comfort {
    SlackWeight weight_lon_slack;
    SlackWeight weight_lat_slack;
    LookupTableConfig time_weight_table;
  };
  struct Progress {
    LookupTableConfig progress_cost_table;
    double exceed_diverged_pt_cost;
  };
  struct Similarity {
    LookupTableConfig similarity_cost_table;
  };
  struct LaneChange {
    double urgency_cost_weight;
    double abortion_cost;
    double staging_cost_addition;
    double last_lc_finished_cost;
    double lc_forbidden_cost;
  };

  struct CostMaxLimit {
    double safety_max_cost{10.0};
    double progress_max_cost{1.0};
    double comfort_max_cost{3.0};
    double default_max_cost{10.0};
  };

  CostMaxLimit cost_limit;
  Safety safety;
  Comfort comfort;
  Progress progress;
  Similarity similarity;
  LaneChange lane_change;
};

}  // namespace planning
}  // namespace zark
