
#include "conf_gflags.h"

// TODO(zhilong.liu) clean up

namespace zark {
namespace planning {

std::string PlanningGflags::vehicle_config_path;
std::string PlanningGflags::local_route_path;
std::string PlanningGflags::decision_path;
double PlanningGflags::planning_loop_rate;
double PlanningGflags::trajectory_time_length;
bool PlanningGflags::enable_staging_corridor;
bool PlanningGflags::enable_nudge_corridor;
bool PlanningGflags::enable_use_nudgers_info;
bool PlanningGflags::enable_lat_mpc_in_lane_keep;
bool PlanningGflags::enable_reduce_obs_traj_sample_t;
bool PlanningGflags::enable_lon_ref_v2;
bool PlanningGflags::enable_prediction_time_compensation;
bool PlanningGflags::use_corridor_boundary_blocking_judgement;
bool PlanningGflags::enable_online_map;

float DecisionGflags::alc_forcelc_distlim_valid;
float DecisionGflags::alc_forcelc_speed_default;
float DecisionGflags::alc_forcelc_distlim_split;
float DecisionGflags::alc_forcelc_dist_ign_mergramp;
float DecisionGflags::alc_forcelc_avoidremdistmin;
float DecisionGflags::alc_forcelc_distlim_avoid;
float DecisionGflags::alc_forcelc_spdlim_min;
float DecisionGflags::alc_forcelc_distlim_lc_max;
float DecisionGflags::alc_forcelc_distlim_lc_min;
float DecisionGflags::alc_forcelc_dist_buffer;
float DecisionGflags::alc_forcelc_distbuff_lowconf;
float DecisionGflags::alc_forcelc_dist_offramp_rem;
float DecisionGflags::alc_forcelc_dist_onramp_rem;
float DecisionGflags::alc_onramp_dist_advanced_req;
float DecisionGflags::alc_forcelc_time_lc;
float DecisionGflags::alc_forcelc_coeff_lc;
float DecisionGflags::alc_forcelc_distlim_constr;
float DecisionGflags::alc_takeover_dist_seclvl_toll;
float DecisionGflags::alc_takeover_dist_fstlvl_toll;
float DecisionGflags::alc_speed_limt_first_dis;
float DecisionGflags::alc_speed_limt_merge_main_road_dis;
float DecisionGflags::alc_speed_limt_enter_ramp_dis;
float DecisionGflags::alc_speed_limt_dis_buffer;
float DecisionGflags::alc_flc_dist_onrampend_buffer;
float DecisionGflags::alc_flc_dist_offrampend_buffer;
float DecisionGflags::alc_flc_dist_tollend_buffer;
float DecisionGflags::alc_flc_dist_mergestart_buffer;
float DecisionGflags::alc_takeover_dist_seclvl_ramp;
float DecisionGflags::alc_takeover_dist_fstlvl_ramp;
float DecisionGflags::alc_avoid_dist_rampend_max;
float DecisionGflags::alc_avoid_dist_rampend_min;
float DecisionGflags::alc_takeover_dist_seclvl_RtEnd;
float DecisionGflags::alc_takeover_dist_fstlvl_RtEnd;
float DecisionGflags::alc_inhibit_dist_buffer;
float DecisionGflags::alc_inhibit_keep_distance;
float DecisionGflags::alc_avoid_distlim_lc_rampstart;
float DecisionGflags::alc_forcelc_dist_keep_onramp;
float DecisionGflags::alc_forcelc_dist_reset_onramp;
float DecisionGflags::alc_dist_offramp_ignkeeponramp;
uint8_t DecisionGflags::alc_forcelc_time_hold;
float DecisionGflags::alc_toll_station_range;

void PlanningGflags::setPlanningConfig(nlohmann::json jnode) {
  jnode.at("vehicle_config_path").get_to(PlanningGflags::vehicle_config_path);
  jnode.at("local_route_path").get_to(PlanningGflags::local_route_path);
  jnode.at("decision_path").get_to(PlanningGflags::decision_path);
  jnode.at("enable_staging_corridor")
      .get_to(PlanningGflags::enable_staging_corridor);
  jnode.at("planning_loop_rate").get_to(PlanningGflags::planning_loop_rate);
  jnode.at("trajectory_time_length")
      .get_to(PlanningGflags::trajectory_time_length);
  jnode.at("enable_nudge_corridor")
      .get_to(PlanningGflags::enable_nudge_corridor);
  jnode.at("enable_use_nudgers_info")
      .get_to(PlanningGflags::enable_use_nudgers_info);
  jnode.at("enable_lat_mpc_in_lane_keep")
      .get_to(PlanningGflags::enable_lat_mpc_in_lane_keep);
  jnode.at("enable_reduce_obs_traj_sample_t")
      .get_to(PlanningGflags::enable_reduce_obs_traj_sample_t);
  jnode.at("enable_lon_ref_v2").get_to(PlanningGflags::enable_lon_ref_v2);
  jnode.at("enable_prediction_time_compensation")
      .get_to(PlanningGflags::enable_prediction_time_compensation);
  jnode.at("use_corridor_boundary_blocking_judgement")
      .get_to(PlanningGflags::use_corridor_boundary_blocking_judgement);
  jnode.at("enable_online_map").get_to(PlanningGflags::enable_online_map);
}

void PlanningGflags::setPlanningConfig(const std::string &config_file) {
  std::ifstream jdata(config_file);
  if (!jdata.is_open())
    std::cout << "Open " << config_file << " failed" << std::endl;
  nlohmann::json jnode = nlohmann::json::parse(jdata);
  setPlanningConfig(jnode);
}

void DecisionGflags::setDecisionConfig(const std::string &config_path) {
  std::ifstream jdata(config_path);
  if (!jdata.is_open()) {
    std::cout << "Open " << config_path << " failed" << std::endl;
  }
  nlohmann::json jnode = nlohmann::json::parse(jdata);
  jnode.at("alc_forcelc_distlim_valid")
      .get_to(DecisionGflags::alc_forcelc_distlim_valid);
  jnode.at("alc_forcelc_speed_default")
      .get_to(DecisionGflags::alc_forcelc_speed_default);
  jnode.at("alc_forcelc_distlim_split")
      .get_to(DecisionGflags::alc_forcelc_distlim_split);
  jnode.at("alc_forcelc_dist_ign_mergramp")
      .get_to(DecisionGflags::alc_forcelc_dist_ign_mergramp);
  jnode.at("alc_forcelc_avoidremdistmin")
      .get_to(DecisionGflags::alc_forcelc_avoidremdistmin);
  jnode.at("alc_forcelc_distlim_avoid")
      .get_to(DecisionGflags::alc_forcelc_distlim_avoid);
  jnode.at("alc_forcelc_spdlim_min")
      .get_to(DecisionGflags::alc_forcelc_spdlim_min);
  jnode.at("alc_forcelc_distlim_lc_max")
      .get_to(DecisionGflags::alc_forcelc_distlim_lc_max);
  jnode.at("alc_forcelc_distlim_lc_min")
      .get_to(DecisionGflags::alc_forcelc_distlim_lc_min);
  jnode.at("alc_forcelc_dist_buffer")
      .get_to(DecisionGflags::alc_forcelc_dist_buffer);
  jnode.at("alc_forcelc_distbuff_lowconf")
      .get_to(DecisionGflags::alc_forcelc_distbuff_lowconf);
  jnode.at("alc_forcelc_dist_offramp_rem")
      .get_to(DecisionGflags::alc_forcelc_dist_offramp_rem);
  jnode.at("alc_forcelc_dist_onramp_rem")
      .get_to(DecisionGflags::alc_forcelc_dist_onramp_rem);
  jnode.at("alc_forcelc_dist_onramp_rem")
      .get_to(DecisionGflags::alc_onramp_dist_advanced_req);
  jnode.at("alc_forcelc_time_lc").get_to(DecisionGflags::alc_forcelc_time_lc);
  jnode.at("alc_forcelc_coeff_lc").get_to(DecisionGflags::alc_forcelc_coeff_lc);
  jnode.at("alc_forcelc_distlim_constr")
      .get_to(DecisionGflags::alc_forcelc_distlim_constr);
  jnode.at("alc_takeover_dist_seclvl_toll")
      .get_to(DecisionGflags::alc_takeover_dist_seclvl_toll);
  jnode.at("alc_takeover_dist_fstlvl_toll")
      .get_to(DecisionGflags::alc_takeover_dist_fstlvl_toll);
  jnode.at("alc_speed_limt_first_dis")
      .get_to(DecisionGflags::alc_speed_limt_first_dis);
  jnode.at("alc_speed_limt_merge_main_road_dis")
      .get_to(DecisionGflags::alc_speed_limt_merge_main_road_dis);
  jnode.at("alc_speed_limt_enter_ramp_dis")
      .get_to(DecisionGflags::alc_speed_limt_enter_ramp_dis);
  jnode.at("alc_speed_limt_dis_buffer")
      .get_to(DecisionGflags::alc_speed_limt_dis_buffer);
  jnode.at("alc_flc_dist_onrampend_buffer")
      .get_to(DecisionGflags::alc_flc_dist_onrampend_buffer);
  jnode.at("alc_flc_dist_offrampend_buffer")
      .get_to(DecisionGflags::alc_flc_dist_offrampend_buffer);
  jnode.at("alc_flc_dist_tollend_buffer")
      .get_to(DecisionGflags::alc_flc_dist_tollend_buffer);
  jnode.at("alc_flc_dist_mergestart_buffer")
      .get_to(DecisionGflags::alc_flc_dist_mergestart_buffer);
  jnode.at("alc_takeover_dist_seclvl_ramp")
      .get_to(DecisionGflags::alc_takeover_dist_seclvl_ramp);
  jnode.at("alc_takeover_dist_fstlvl_ramp")
      .get_to(DecisionGflags::alc_takeover_dist_fstlvl_ramp);
  jnode.at("alc_avoid_dist_rampend_max")
      .get_to(DecisionGflags::alc_avoid_dist_rampend_max);
  jnode.at("alc_avoid_dist_rampend_min")
      .get_to(DecisionGflags::alc_avoid_dist_rampend_min);
  jnode.at("alc_takeover_dist_seclvl_RtEnd")
      .get_to(DecisionGflags::alc_takeover_dist_seclvl_RtEnd);
  jnode.at("alc_takeover_dist_fstlvl_RtEnd")
      .get_to(DecisionGflags::alc_takeover_dist_fstlvl_RtEnd);
  jnode.at("alc_inhibit_dist_buffer")
      .get_to(DecisionGflags::alc_inhibit_dist_buffer);
  jnode.at("alc_inhibit_keep_distance")
      .get_to(DecisionGflags::alc_inhibit_keep_distance);
  jnode.at("alc_avoid_distlim_lc_rampstart")
      .get_to(DecisionGflags::alc_avoid_distlim_lc_rampstart);

  jnode.at("alc_forcelc_dist_keep_onramp")
      .get_to(DecisionGflags::alc_forcelc_dist_keep_onramp);
  jnode.at("alc_forcelc_dist_reset_onramp")
      .get_to(DecisionGflags::alc_forcelc_dist_reset_onramp);
  jnode.at("alc_dist_offramp_ignkeeponramp")
      .get_to(DecisionGflags::alc_dist_offramp_ignkeeponramp);
  jnode.at("alc_forcelc_time_hold")
      .get_to(DecisionGflags::alc_forcelc_time_hold);
  jnode.at("alc_toll_station_range")
      .get_to(DecisionGflags::alc_toll_station_range);
}

}  // namespace planning
}  // namespace zark
