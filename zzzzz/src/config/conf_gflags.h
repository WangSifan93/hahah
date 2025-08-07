#pragma once
#include <fstream>
#include <iostream>
#include <string>

#include "nlohmann/json.hpp"

namespace zark {
namespace planning {

using nlohmann::json;

class PlanningGflags {
 public:
  static std::string vehicle_config_path;
  static std::string local_route_path;
  static std::string decision_path;
  static double planning_loop_rate;
  static double trajectory_time_length;
  static bool enable_staging_corridor;
  static bool enable_nudge_corridor;
  static bool enable_use_nudgers_info;
  static bool enable_lat_mpc_in_lane_keep;
  static bool enable_reduce_obs_traj_sample_t;
  static bool enable_lon_ref_v2;
  static bool enable_prediction_time_compensation;
  static bool use_corridor_boundary_blocking_judgement;
  static bool enable_online_map;

 public:
  static void setPlanningConfig(const std::string &config_file);
  static void setPlanningConfig(nlohmann::json jnode);
};

class DecisionGflags {
 public:
  static float alc_forcelc_distlim_valid;
  static float alc_forcelc_speed_default;
  static float alc_forcelc_distlim_split;
  static float alc_forcelc_dist_ign_mergramp;
  static float alc_forcelc_avoidremdistmin;
  static float alc_forcelc_distlim_avoid;
  static float alc_forcelc_spdlim_min;
  static float alc_forcelc_distlim_lc_max;
  static float alc_forcelc_distlim_lc_min;
  static float alc_forcelc_dist_buffer;
  static float alc_forcelc_distbuff_lowconf;
  static float alc_forcelc_dist_offramp_rem;
  static float alc_forcelc_dist_onramp_rem;
  static float alc_onramp_dist_advanced_req;
  static float alc_forcelc_time_lc;
  static float alc_forcelc_coeff_lc;
  static float alc_forcelc_distlim_constr;
  static float alc_takeover_dist_seclvl_toll;
  static float alc_takeover_dist_fstlvl_toll;
  static float alc_speed_limt_first_dis;
  static float alc_speed_limt_merge_main_road_dis;
  static float alc_speed_limt_enter_ramp_dis;
  static float alc_speed_limt_dis_buffer;
  static float alc_flc_dist_onrampend_buffer;
  static float alc_flc_dist_offrampend_buffer;
  static float alc_flc_dist_tollend_buffer;
  static float alc_flc_dist_mergestart_buffer;
  static float alc_takeover_dist_seclvl_ramp;
  static float alc_takeover_dist_fstlvl_ramp;
  static float alc_avoid_dist_rampend_max;
  static float alc_avoid_dist_rampend_min;
  static float alc_takeover_dist_seclvl_RtEnd;
  static float alc_takeover_dist_fstlvl_RtEnd;
  static float alc_inhibit_dist_buffer;
  static float alc_inhibit_keep_distance;
  static float alc_avoid_distlim_lc_rampstart;
  static float alc_forcelc_dist_keep_onramp;
  static float alc_forcelc_dist_reset_onramp;
  static float alc_dist_offramp_ignkeeponramp;
  static uint8_t alc_forcelc_time_hold;
  static float alc_toll_station_range;

 public:
  static void setDecisionConfig(const std::string &config_path);
};

}  // namespace planning
}  // namespace zark
