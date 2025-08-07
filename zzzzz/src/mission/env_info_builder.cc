/******************************************************************************
 * Copyright 2024 The zark Authors. All Rights Reserved.
 *****************************************************************************/

/**
 * @file env_info_builder.cc
 **/

#include "apps/planning/src/mission/env_info_builder.h"

#include "apps/planning/src/common/clock.h"

namespace zark {
namespace planning {

EnvInfoBuilder::EnvInfoBuilder(const MissionDeciderConfig& config) {
  config_ = config;
}

EnvInfoBuilder::~EnvInfoBuilder() {}

EnvInfos EnvInfoBuilder::Build(const common::VehicleState& vehicle_state,
                               const double driver_set_v,
                               const std::vector<const Obstacle*>& obstacles,
                               const LocalRoute& current_local_route) {
  // update Env Infos
  EnvInfos env_infos_res;

  env_infos_res.time_now = zark::common::Clock::NowInSeconds();

  env_infos_res.vehicle_infos = UpdateVehicleInfo(vehicle_state, driver_set_v);

  env_infos_res.lane_infos = UpdateLaneInfo(vehicle_state, current_local_route);

  env_infos_res.obstacle_infos =
      UpdateObstaclesInfo(obstacles, env_infos_res.lane_infos);

  env_infos_res.cur_lane_block_infos = IsCurrentLaneBlock(
      env_infos_res.obstacle_infos, env_infos_res.lane_infos);

  env_infos_res.lane_spd_infos =
      UpdateLaneSpdInfo(env_infos_res.obstacle_infos, env_infos_res.lane_infos);

  env_infos_res.fury_value = UpdateFuryVal(env_infos_res.lane_spd_infos);

  env_infos_res.rc_params_infos = UpdateRouteLCParams(
      env_infos_res.lane_infos, env_infos_res.lane_spd_infos);

  env_infos_res.inhibit_infos =
      UpdateLCInhibInfos(env_infos_res.vehicle_infos, env_infos_res.lane_infos);

  return env_infos_res;
}

VehicleInfos EnvInfoBuilder::UpdateVehicleInfo(
    const VehicleState& vehicle_state, const double& v_set) {
  VehicleInfos vehicle_Infos;
  vehicle_Infos.cur_adc_v = vehicle_state.linear_velocity();
  vehicle_Infos.driver_set_v = v_set;

  return vehicle_Infos;
}

LaneInfos EnvInfoBuilder::UpdateLaneInfo(const VehicleState& vehicle_state,
                                         const LocalRoute& cur_local_route) {
  double cur_lane_ll, cur_lane_rl = 0.0;
  // reset Lane Infos
  LaneInfos laneinfos;
  laneinfos.closer_lane_id = "z_0_0_0";  // set to default id
  laneinfos.current_laneinfo.id = config_.env_info.invalid_lane;
  laneinfos.left_laneinfo.id = config_.env_info.invalid_lane;
  laneinfos.right_laneinfo.id = config_.env_info.invalid_lane;
  laneinfos.connect_lanes_id.clear();
  // TODO : param 3.5
  laneinfos.current_lane_width = 3.5;

  // get ego position
  ::math::Vec2d ego_pose(vehicle_state.x(), vehicle_state.y());

  // get lane infos
  laneinfos.current_laneinfo.id = cur_local_route.Lanes().Id();
  laneinfos.current_laneinfo.local_route = cur_local_route;
  laneinfos.current_laneinfo.route_segment = cur_local_route.Lanes();

  if (cur_local_route.LeftRoute()) {
    laneinfos.left_laneinfo.id = cur_local_route.LeftRoute()->Lanes().Id();
    laneinfos.left_laneinfo.local_route = *cur_local_route.LeftRoute();
    laneinfos.left_laneinfo.route_segment =
        cur_local_route.LeftRoute()->Lanes();
  } else {
    laneinfos.left_laneinfo.id = config_.env_info.invalid_lane;
    AERROR << "NO LEFT LOCALROUTE!!!";
  }

  if (cur_local_route.RightRoute()) {
    laneinfos.right_laneinfo.id = cur_local_route.RightRoute()->Lanes().Id();
    laneinfos.right_laneinfo.local_route = *cur_local_route.RightRoute();
    laneinfos.right_laneinfo.route_segment =
        cur_local_route.RightRoute()->Lanes();
  } else {
    laneinfos.right_laneinfo.id = config_.env_info.invalid_lane;
    AERROR << "NO RIGHT LOCALROUTE!!!";
  }

  laneinfos.current_laneinfo.local_route.XYToSL(ego_pose, laneinfos.current_sl);

  laneinfos.cur_road_type =
      laneinfos.current_laneinfo.local_route.GetRoadClassFromS(
          laneinfos.current_sl.s());

  laneinfos.route_lc_info =
      laneinfos.current_laneinfo.local_route.GetRouteLCInfo();

  if (laneinfos.route_lc_info.dir_next_lc.at(0) ==
      LocalRoute::RouteDirection::LEFT) {
    laneinfos.closer_lane_id = laneinfos.left_laneinfo.local_route.Lanes().Id();
  } else if (laneinfos.route_lc_info.dir_next_lc.at(0) ==
             LocalRoute::RouteDirection::RIGHT) {
    laneinfos.closer_lane_id =
        laneinfos.right_laneinfo.local_route.Lanes().Id();
  } else {
    laneinfos.closer_lane_id = "z_0_0_0";
  }

  if (laneinfos.current_laneinfo.local_route.GetLaneWidth(
          laneinfos.current_sl.s(), cur_lane_ll, cur_lane_rl)) {
    // GetLaneWidth: cur_lane_ll > 0, cur_lane_rl > 0
    laneinfos.current_lane_width = std::fabs(cur_lane_ll + cur_lane_rl);
  } else {
    if (laneinfos.cur_road_type ==
            hdmap_new::Road_RoadClass::Road_RoadClass_RC_EXPRESSWAY ||
        laneinfos.cur_road_type ==
            hdmap_new::Road_RoadClass::Road_RoadClass_RC_NATION_ROAD) {
      // TODO : param 4.0
      laneinfos.current_lane_width = 4.0;
    } else {
      // TODO : param 3.5
      laneinfos.current_lane_width = 3.5;
    }
  }

  return laneinfos;
}

ObstacleInfos EnvInfoBuilder::UpdateObstaclesInfo(
    const std::vector<const zark::planning::Obstacle*>& obstacles,
    const LaneInfos& lane_infos) {
  double cur_lane_start_l = -0.5 * lane_infos.current_lane_width;
  double cur_lane_end_l = 0.5 * lane_infos.current_lane_width;
  double left_lane_start_l = 0.5 * lane_infos.current_lane_width;
  double left_lane_end_l = 1.5 * lane_infos.current_lane_width;
  double right_lane_start_l = -1.5 * lane_infos.current_lane_width;
  double right_lane_end_l = -0.5 * lane_infos.current_lane_width;
  double lleft_lane_start_l = 1.5 * lane_infos.current_lane_width;
  double lleft_lane_end_l = 2.5 * lane_infos.current_lane_width;
  double rright_lane_start_l = -2.5 * lane_infos.current_lane_width;
  double rright_lane_end_l = -1.5 * lane_infos.current_lane_width;

  AINFO << " before process obs size = " << obstacles.size()
        << " adc sl boundary start s = "
        << lane_infos.current_laneinfo.local_route.InitFrenetPoint().s.at(0);

  // init obs result
  ObstacleInfos obs_infos;
  obs_infos.cur_lane_obs.clear();
  obs_infos.left_lane_obs.clear();
  obs_infos.right_lane_obs.clear();
  obs_infos.lleft_lane_obs.clear();
  obs_infos.rright_lane_obs.clear();

  // init temp obs
  std::vector<const Obstacle*> cur_lane_obs;
  std::vector<const Obstacle*> left_lane_obs;
  std::vector<const Obstacle*> right_lane_obs;
  std::vector<const Obstacle*> lleft_lane_obs;
  std::vector<const Obstacle*> rright_lane_obs;
  cur_lane_obs.clear();
  left_lane_obs.clear();
  right_lane_obs.clear();
  lleft_lane_obs.clear();
  rright_lane_obs.clear();

  // get each lane obstacles
  for (size_t i = 0; i < obstacles.size(); i++) {
    std::string obs_id = obstacles.at(i)->Id();

    if (ObstacleNeedIgnore(obs_id)) {
      continue;
    }
    SLBoundary sl_boundary = obstacles.at(i)->PerceptionSLBoundary();
    double center_l = (sl_boundary.start_l() + sl_boundary.end_l()) * 0.5;
    AINFO << " obstacle id = " << obstacles.at(i)->Id()
          << " obstacle s = " << sl_boundary.start_s()
          << " obstacle start l = " << sl_boundary.start_l()
          << " obstacle end l = " << sl_boundary.end_l();
    if (center_l > cur_lane_start_l && center_l < cur_lane_end_l) {
      cur_lane_obs.emplace_back(obstacles.at(i));
    } else if (center_l > left_lane_start_l && center_l < left_lane_end_l) {
      left_lane_obs.emplace_back(obstacles.at(i));
    } else if (center_l > right_lane_start_l && center_l < right_lane_end_l) {
      right_lane_obs.emplace_back(obstacles.at(i));
    } else if (center_l > lleft_lane_start_l && center_l < lleft_lane_end_l) {
      lleft_lane_obs.emplace_back(obstacles.at(i));
    } else if (center_l > rright_lane_start_l && center_l < rright_lane_end_l) {
      rright_lane_obs.emplace_back(obstacles.at(i));
    }
  }

  // get cur lane interest obstacles
  double obs_s_tmp = 0.0;
  // cur lane keep 2 front obj
  if (!cur_lane_obs.empty()) {
    std::sort(cur_lane_obs.begin(), cur_lane_obs.end(), CompareByStarts);

    int8_t cur_lane_nearest_index =
        FindNearestObjIndex(lane_infos.current_sl.s(), cur_lane_obs);
    obs_s_tmp = 0.5 * (cur_lane_obs.at(cur_lane_nearest_index)
                           ->PerceptionSLBoundary()
                           .start_s() +
                       cur_lane_obs.at(cur_lane_nearest_index)
                           ->PerceptionSLBoundary()
                           .end_s());

    if (obs_s_tmp <= lane_infos.current_sl.s()) {
      if (cur_lane_obs.size() >= (cur_lane_nearest_index + 2)) {
        obs_infos.cur_lane_obs.emplace_back(
            cur_lane_obs.at(cur_lane_nearest_index + 1));

        if (cur_lane_obs.size() > (cur_lane_nearest_index + 2)) {
          obs_infos.cur_lane_obs.emplace_back(
              cur_lane_obs.at(cur_lane_nearest_index + 2));
        }
      }
    } else {
      obs_infos.cur_lane_obs.emplace_back(
          cur_lane_obs.at(cur_lane_nearest_index));

      if (cur_lane_obs.size() > (cur_lane_nearest_index + 1)) {
        obs_infos.cur_lane_obs.emplace_back(
            cur_lane_obs.at(cur_lane_nearest_index + 1));
      }
    }
  }
  AINFO << "Cur Lane Obs size = " << obs_infos.cur_lane_obs.size();

  // get left lane interest obstacles
  // left lane keep 2 front obj and 1 rear obj
  if (!left_lane_obs.empty()) {
    std::sort(left_lane_obs.begin(), left_lane_obs.end(), CompareByStarts);

    int8_t left_lane_nearest_index =
        FindNearestObjIndex(lane_infos.current_sl.s(), left_lane_obs);
    obs_s_tmp = 0.5 * (left_lane_obs.at(left_lane_nearest_index)
                           ->PerceptionSLBoundary()
                           .start_s() +
                       left_lane_obs.at(left_lane_nearest_index)
                           ->PerceptionSLBoundary()
                           .end_s());

    if (obs_s_tmp <= lane_infos.current_sl.s()) {
      obs_infos.left_lane_obs.emplace_back(
          left_lane_obs.at(left_lane_nearest_index));

      if (left_lane_obs.size() >= left_lane_nearest_index + 2) {
        obs_infos.left_lane_obs.emplace_back(
            left_lane_obs.at(left_lane_nearest_index + 1));

        if (left_lane_obs.size() > left_lane_nearest_index + 2) {
          obs_infos.left_lane_obs.emplace_back(
              left_lane_obs.at(left_lane_nearest_index + 2));
        }
      }
    } else {
      if (left_lane_nearest_index > 0) {
        obs_infos.left_lane_obs.emplace_back(
            left_lane_obs.at(left_lane_nearest_index - 1));
      }
      obs_infos.left_lane_obs.emplace_back(
          left_lane_obs.at(left_lane_nearest_index));
      if (left_lane_obs.size() >= left_lane_nearest_index + 2) {
        obs_infos.left_lane_obs.emplace_back(
            left_lane_obs.at(left_lane_nearest_index + 1));
      }
    }
  }
  AINFO << "Left Lane Obs size = " << obs_infos.left_lane_obs.size();

  // get right lane interest obstacles
  // right lane keep 2 front obj and 1 rear obj
  if (!right_lane_obs.empty()) {
    std::sort(right_lane_obs.begin(), right_lane_obs.end(), CompareByStarts);

    int8_t right_lane_nearest_index =
        FindNearestObjIndex(lane_infos.current_sl.s(), right_lane_obs);
    obs_s_tmp = 0.5 * (right_lane_obs.at(right_lane_nearest_index)
                           ->PerceptionSLBoundary()
                           .start_s() +
                       right_lane_obs.at(right_lane_nearest_index)
                           ->PerceptionSLBoundary()
                           .end_s());
    if (obs_s_tmp <= lane_infos.current_sl.s()) {
      obs_infos.right_lane_obs.emplace_back(
          right_lane_obs.at(right_lane_nearest_index));

      if (right_lane_obs.size() >= right_lane_nearest_index + 2) {
        obs_infos.right_lane_obs.emplace_back(
            right_lane_obs.at(right_lane_nearest_index + 1));

        if (right_lane_obs.size() > right_lane_nearest_index + 2) {
          obs_infos.right_lane_obs.emplace_back(
              right_lane_obs.at(right_lane_nearest_index + 2));
        }
      }
    } else {
      if (right_lane_nearest_index > 0) {
        obs_infos.right_lane_obs.emplace_back(
            right_lane_obs.at(right_lane_nearest_index - 1));
      }
      obs_infos.right_lane_obs.emplace_back(
          right_lane_obs.at(right_lane_nearest_index));
      if (right_lane_obs.size() >= right_lane_nearest_index + 2) {
        obs_infos.right_lane_obs.emplace_back(
            right_lane_obs.at(right_lane_nearest_index + 1));
      }
    }
  }
  AINFO << "Right Lane Obs size = " << obs_infos.right_lane_obs.size();

  // get left left lane interest obstacles
  // left left lane keep 1 front obj and 1 rear obj
  if (!lleft_lane_obs.empty()) {
    std::sort(lleft_lane_obs.begin(), lleft_lane_obs.end(), CompareByStarts);

    int8_t lleft_lane_nearest_index =
        FindNearestObjIndex(lane_infos.current_sl.s(), lleft_lane_obs);
    obs_s_tmp = 0.5 * (lleft_lane_obs.at(lleft_lane_nearest_index)
                           ->PerceptionSLBoundary()
                           .start_s() +
                       lleft_lane_obs.at(lleft_lane_nearest_index)
                           ->PerceptionSLBoundary()
                           .end_s());
    if (obs_s_tmp <= lane_infos.current_sl.s()) {
      obs_infos.lleft_lane_obs.emplace_back(
          lleft_lane_obs.at(lleft_lane_nearest_index));

      if (lleft_lane_obs.size() >= lleft_lane_nearest_index + 2) {
        obs_infos.lleft_lane_obs.emplace_back(
            lleft_lane_obs.at(lleft_lane_nearest_index + 1));
      }
    } else {
      if (lleft_lane_nearest_index > 0) {
        obs_infos.lleft_lane_obs.emplace_back(
            lleft_lane_obs.at(lleft_lane_nearest_index - 1));
      }
      obs_infos.lleft_lane_obs.emplace_back(
          lleft_lane_obs.at(lleft_lane_nearest_index));
    }
  }
  AINFO << "Left Left Lane Obs size = " << obs_infos.lleft_lane_obs.size();

  // get right right lane interest obstacles
  // right right lane keep 1 front obj and 1 rear obj
  if (!rright_lane_obs.empty()) {
    std::sort(rright_lane_obs.begin(), rright_lane_obs.end(), CompareByStarts);

    int8_t rright_lane_nearest_index =
        FindNearestObjIndex(lane_infos.current_sl.s(), rright_lane_obs);
    obs_s_tmp = 0.5 * (rright_lane_obs.at(rright_lane_nearest_index)
                           ->PerceptionSLBoundary()
                           .start_s() +
                       rright_lane_obs.at(rright_lane_nearest_index)
                           ->PerceptionSLBoundary()
                           .end_s());
    if (obs_s_tmp <= lane_infos.current_sl.s()) {
      obs_infos.rright_lane_obs.emplace_back(
          rright_lane_obs.at(rright_lane_nearest_index));

      if (rright_lane_obs.size() >= rright_lane_nearest_index + 2) {
        obs_infos.rright_lane_obs.emplace_back(
            rright_lane_obs.at(rright_lane_nearest_index + 1));
      }
    } else {
      if (rright_lane_nearest_index > 0) {
        obs_infos.rright_lane_obs.emplace_back(
            rright_lane_obs.at(rright_lane_nearest_index - 1));
      }
      obs_infos.rright_lane_obs.emplace_back(
          rright_lane_obs.at(rright_lane_nearest_index));
    }
  }
  AINFO << "Right Right Lane Obs size = " << obs_infos.rright_lane_obs.size();

  // GET INTEREST OBSTACLE DONE !!!
  return obs_infos;
}

bool EnvInfoBuilder::ObstacleNeedIgnore(const std::string& obs_id) {
  bool res = false;
  if (obs_id == "DEST") {
    return true;
  }
  size_t temp = obs_id.find("_");
  size_t str_length = obs_id.size();
  if (temp != std::string::npos && obs_id.substr(temp + 1, str_length) != "0") {
    return true;
  }
  return res;
}

int8_t EnvInfoBuilder::FindNearestObjIndex(
    const double& adc_s, const std::vector<const Obstacle*>& obj_vec) {
  int8_t nearest_index = 0;
  double dist_tmp = 0.0;
  double nearest_dist = std::numeric_limits<double>::max();

  for (size_t i = 0; i < obj_vec.size(); ++i) {
    SLBoundary sl_boundary = obj_vec.at(i)->PerceptionSLBoundary();
    dist_tmp = fabs(sl_boundary.start_s() - adc_s);

    if (dist_tmp < nearest_dist) {
      nearest_dist = dist_tmp;
      nearest_index = i;
    }
  }
  return nearest_index;
}

CurLaneBlockInfos EnvInfoBuilder::IsCurrentLaneBlock(
    const ObstacleInfos& obs_info, const LaneInfos& lane_infos) {
  // init CurLane Block Infos
  CurLaneBlockInfos cur_lane_block_infos;
  cur_lane_block_infos.is_current_line_blocked = false;
  cur_lane_block_infos.dis_2_frontstaticobj = 0.0;

  bool find_blocking = false;

  // TODO :
  // 将前方障碍物如果是交通锥或者三脚架等处理放在这，如果出现这些障碍物，直接block
  // cur lane 目前，没有Triangle warning sign的类型
  for (const auto& obj : obs_info.cur_lane_obs) {
    if (obj->Perception().sub_type() == perception::SubType::ST_TRAFFICCONE) {
      cur_lane_block_infos.is_current_line_blocked = true;
      cur_lane_block_infos.dis_2_frontstaticobj =
          obs_info.cur_lane_obs.at(0)->PerceptionSLBoundary().start_s() -
          lane_infos.current_sl.s();

      return cur_lane_block_infos;
    } else {
      // TODO : need to split speed with s and l direcition
      if ((obj->Id() != "DEST") && (obj->IsStatic())) {
        find_blocking = true;

        break;
      }
    }
  }

  if (!find_blocking) {
    blocking_cnt_ = 0;
    false_blocking_cnt_++;
    // force_lane_block_cnt : 3
    if (false_blocking_cnt_ >= config_.env_info.force_lane_block_cnt) {
      false_blocking_cnt_ = config_.env_info.force_lane_block_cnt;
      cur_lane_block_infos.is_current_line_blocked = false;
      cur_lane_block_infos.dis_2_frontstaticobj = 0.0;
    }
  } else {  // current lane blocked
    blocking_cnt_++;
    false_blocking_cnt_ = 0;
    if (blocking_cnt_ >= config_.env_info.force_lane_block_cnt) {
      blocking_cnt_ = config_.env_info.force_lane_block_cnt;
      cur_lane_block_infos.is_current_line_blocked = true;
      cur_lane_block_infos.dis_2_frontstaticobj =
          obs_info.cur_lane_obs.at(0)->PerceptionSLBoundary().start_s() -
          lane_infos.current_sl.s();
    }
  }

  return cur_lane_block_infos;
}

LaneSpdInfos EnvInfoBuilder::UpdateLaneSpdInfo(const ObstacleInfos& obs_info,
                                               const LaneInfos& lane_infos) {
  // init Lane Spd Infos
  LaneSpdInfos lane_spd_infos;
  lane_spd_infos.v_cur_lane_limit = 0.0;
  lane_spd_infos.v_left_lane_limit = 0.0;
  lane_spd_infos.v_right_lane_limit = 0.0;
  lane_spd_infos.v_cur_lane_pass = 0.0;
  lane_spd_infos.v_left_lane_pass = 0.0;
  lane_spd_infos.v_right_lane_pass = 0.0;

  if (lane_infos.current_laneinfo.id == config_.env_info.invalid_lane) {
    // default_spd_limit : 16.7 Need to adapt to city
    lane_spd_infos.v_cur_lane_limit = config_.env_info.default_spd_limit;
    lane_spd_infos.v_left_lane_limit = 0.0;
    lane_spd_infos.v_right_lane_limit = 0.0;
  }

  double cur_lane_spd_lim =
      lane_infos.current_laneinfo.local_route.GetSpeedLimitFromS(
          lane_infos.current_sl.s());
  if (cur_lane_spd_lim <= config_.env_info.epsilon) {
    lane_spd_infos.v_cur_lane_limit = config_.env_info.default_spd_limit;
  } else {
    lane_spd_infos.v_cur_lane_limit = cur_lane_spd_lim;
  }

  if (lane_infos.left_laneinfo.id == config_.env_info.invalid_lane) {
    lane_spd_infos.v_left_lane_limit = 0.0;
  } else {
    double left_lane_spd_lim =
        lane_infos.left_laneinfo.local_route.GetSpeedLimitFromS(
            lane_infos.current_sl.s());

    if (left_lane_spd_lim <= config_.env_info.epsilon) {
      lane_spd_infos.v_left_lane_limit = config_.env_info.default_spd_limit;
    } else {
      lane_spd_infos.v_left_lane_limit = left_lane_spd_lim;
    }
  }

  if (lane_infos.right_laneinfo.id == config_.env_info.invalid_lane) {
    lane_spd_infos.v_right_lane_limit = 0.0;
  } else {
    double right_lane_spd_lim =
        lane_infos.right_laneinfo.local_route.GetSpeedLimitFromS(
            lane_infos.current_sl.s());

    if (right_lane_spd_lim <= config_.env_info.epsilon) {
      lane_spd_infos.v_right_lane_limit = config_.env_info.default_spd_limit;
    } else {
      lane_spd_infos.v_right_lane_limit = right_lane_spd_lim;
    }
  }

  // get cur lane pass speed
  if (obs_info.cur_lane_obs.empty()) {
    lane_spd_infos.v_cur_lane_pass = lane_spd_infos.v_cur_lane_limit;
  } else {
    double tmp_v = std::numeric_limits<double>::max();
    for (auto obj : obs_info.cur_lane_obs) {
      if (obj->speed() <= tmp_v) {
        tmp_v = obj->speed();
      }
    }

    lane_spd_infos.v_cur_lane_pass = tmp_v;
  }

  // get left lane pass speed
  if (!obs_info.left_lane_obs.empty()) {
    double tmp_v = std::numeric_limits<double>::max();
    for (auto obj : obs_info.left_lane_obs) {
      if (obj->speed() <= tmp_v) {
        tmp_v = obj->speed();
      }
    }

    lane_spd_infos.v_left_lane_pass = tmp_v;
  } else {
    if (lane_infos.left_laneinfo.id != config_.env_info.invalid_lane) {
      lane_spd_infos.v_left_lane_pass = lane_spd_infos.v_left_lane_limit;
    } else {
      lane_spd_infos.v_left_lane_pass = 0.0;
    }
  }

  // get right lane pass speed
  if (!obs_info.right_lane_obs.empty()) {
    double tmp_v = std::numeric_limits<double>::max();
    for (auto obj : obs_info.right_lane_obs) {
      if (obj->speed() <= tmp_v) {
        tmp_v = obj->speed();
      }
    }

    lane_spd_infos.v_right_lane_pass = tmp_v;
  } else {
    if (lane_infos.right_laneinfo.id != config_.env_info.invalid_lane) {
      lane_spd_infos.v_right_lane_pass = lane_spd_infos.v_right_lane_limit;
    } else {
      lane_spd_infos.v_right_lane_pass = 0.0;
    }
  }

  return lane_spd_infos;
}

double EnvInfoBuilder::UpdateFuryVal(const LaneSpdInfos& lane_spd_info) {
  double fury_value = 0.0;
  double cur_fury = 0.0;
  double low_spd_ratio =
      (lane_spd_info.v_cur_lane_limit - lane_spd_info.v_cur_lane_pass) /
      lane_spd_info.v_cur_lane_limit;

  // calculate current fury value
  // low speed ratio more than 0.05(comf_low_spd_ratio_gate)
  if (low_spd_ratio >= config_.env_info.comf_low_spd_ratio_gate) {
    cur_fury = LinearInterp({0.05, 0.10, 0.15}, {0.5, 1, 10}, low_spd_ratio);
    if (fury_value < 105.0) {
      fury_value = std::min(105.0, (fury_value + cur_fury));
    }
  } else {
    fury_value = 0;
  }

  return fury_value;
}

double EnvInfoBuilder::LinearInterp(const std::vector<double>& x,
                                    const std::vector<double>& y,
                                    const double xi) {
  auto n = x.size();
  if (n != y.size() || n == 0) {
    AERROR << "Invalid input!";
    return 0;
  }
  if (xi <= x[0]) return 0;
  if (xi >= x[n - 1]) return y[n - 1];

  for (std::size_t i = 0; i < n - 1; i++) {
    if (xi >= x[i] && xi <= x[i + 1]) {
      double slope = (y[i + 1] - y[i]) / (x[i + 1] - x[i]);
      double yi = y[i] + slope * (xi - x[i]);
      return yi;
    }
  }

  return 0;
}

RCParamInfos EnvInfoBuilder::UpdateRouteLCParams(
    const LaneInfos& lane_infos, const LaneSpdInfos& lane_spd_info) {
  // init RC Param Infos
  RCParamInfos rc_raram_infos;
  rc_raram_infos.dis_2_ramp_threshold = 0.0;
  rc_raram_infos.reverse_dis_2_ramp_threshold = 0.0;

  // ==================================================
  // TODO : Only Highway Need to add CityRoad
  // ==================================================
  // Functional specifications：d = speed_limit*10s*lc_num*congestion_coeff
  // 10s, congestion_coeff To be calibrated;
  // change_num = std::max(min_num, change_num);
  // default_lc_finish_time : 13.0 s
  double experience_t = config_.env_info.default_lc_finish_time;
  // to do :consider pass_speed; default_lc_congestion_coeff : 1.3s
  double congestion_coeff = config_.env_info.default_lc_congestion_coeff;
  std::vector<int> new_change_num =
      lane_infos.current_laneinfo.local_route.GetRouteLCInfo()
          .num_lc_to_next_lanes;
  rc_raram_infos.dis_2_ramp_threshold = lane_spd_info.v_cur_lane_limit *
                                        experience_t * new_change_num.at(0) *
                                        congestion_coeff;
  // reverse change lane need more distance:
  // change num shoud be neighbor change num;
  // need consider add two change distance;
  rc_raram_infos.reverse_dis_2_ramp_threshold =
      lane_spd_info.v_cur_lane_limit * experience_t *
          (new_change_num.at(0) + 1) * congestion_coeff +
      2 * (lane_spd_info.v_cur_lane_limit * 10);

  AINFO << "  dis_ramp_threshold:  " << rc_raram_infos.dis_2_ramp_threshold
        << "   reverse inhibition dis: "
        << rc_raram_infos.reverse_dis_2_ramp_threshold
        << "  change num: " << new_change_num.at(0)
        << " lane speed : " << lane_spd_info.v_cur_lane_limit;

  // TODO : add city road distance to next 2 intersection and
  // actions(goforward,left,right)
  return rc_raram_infos;
}

InhibitInfos EnvInfoBuilder::UpdateLCInhibInfos(
    const VehicleInfos& vehicle_infos, const LaneInfos& lane_infos) {
  InhibitInfos res;
  res.is_inhibit_left = false;
  res.is_inhibit_right = false;
  res.inhibit_left_type = InhibitInfos::InhibitType::NONE;
  res.inhibit_right_type = InhibitInfos::InhibitType::NONE;

  // 1. main road check
  InhibitInfos main_road_res;
  main_road_res.is_inhibit_left = false;
  main_road_res.is_inhibit_right = false;
  main_road_res.inhibit_left_type = InhibitInfos::InhibitType::NONE;
  main_road_res.inhibit_right_type = InhibitInfos::InhibitType::NONE;
  if (lane_infos.current_laneinfo.id == config_.env_info.invalid_lane) {
    main_road_res.is_inhibit_left = true;
    main_road_res.is_inhibit_right = true;
    main_road_res.inhibit_left_type =
        InhibitInfos::InhibitType::CUR_LANE_INVALID;
    main_road_res.inhibit_right_type =
        InhibitInfos::InhibitType::CUR_LANE_INVALID;

    AERROR << " inhib reason: cur lane is invalid.";
  }

  // 2. cur lane type check
  InhibitInfos cur_lane_type_res;
  cur_lane_type_res.is_inhibit_left = false;
  cur_lane_type_res.is_inhibit_right = false;
  cur_lane_type_res.inhibit_left_type = InhibitInfos::InhibitType::NONE;
  cur_lane_type_res.inhibit_right_type = InhibitInfos::InhibitType::NONE;
  auto cur_lane_type = lane_infos.current_laneinfo.local_route.GetLaneTypeFromS(
      lane_infos.current_sl.s());
  if (cur_lane_type != zark::hdmap_new::Lane_LaneType::
                           Lane_LaneType_LANE_TYPE_ACCELERATION_LANE &&
      cur_lane_type != zark::hdmap_new::Lane_LaneType::
                           Lane_LaneType_LANE_TYPE_DECELERATION_LANE &&
      cur_lane_type !=
          zark::hdmap_new::Lane_LaneType::Lane_LaneType_LANE_TYPE_MIXED_LANE &&
      cur_lane_type !=
          zark::hdmap_new::Lane_LaneType::Lane_LaneType_LANE_TYPE_NORMAL_LANE &&
      cur_lane_type != zark::hdmap_new::Lane_LaneType::
                           Lane_LaneType_LANE_TYPE_VARIABLE_LANE) {
    cur_lane_type_res.is_inhibit_left = true;
    cur_lane_type_res.is_inhibit_right = true;
    cur_lane_type_res.inhibit_left_type =
        InhibitInfos::InhibitType::LANE_TYPE_INVALID;
    cur_lane_type_res.inhibit_right_type =
        InhibitInfos::InhibitType::LANE_TYPE_INVALID;

    AERROR << " cur_lane_type : " << cur_lane_type;
  }

  // 3. curvature check
  InhibitInfos curvature_res;
  curvature_res.is_inhibit_left = false;
  curvature_res.is_inhibit_right = false;
  curvature_res.inhibit_left_type = InhibitInfos::InhibitType::NONE;
  curvature_res.inhibit_right_type = InhibitInfos::InhibitType::NONE;

  double curvature_limit = 0.05;
  double coefficient = 0.1;

  curvature_limit =
      curvature_limit +
      coefficient *
          (config_.lc_request.speed_upper_bound - vehicle_infos.cur_adc_v) /
          config_.lc_request.speed_upper_bound;

  std::vector<LocalRoutePoint> local_route_points =
      lane_infos.current_laneinfo.local_route.GetLocalRoutePoints(
          lane_infos.current_sl.s(),
          lane_infos.current_sl.s() + config_.lc_request.lc_look_forward_dis);

  size_t local_route_points_size = local_route_points.size();
  for (size_t i = 0; i < local_route_points_size; ++i) {
    double curvature = local_route_points.at(i).kappa();
    if (fabs(curvature) > curvature_limit) {
      curvature_res.is_inhibit_left = true;
      curvature_res.is_inhibit_right = true;
      curvature_res.inhibit_left_type =
          InhibitInfos::InhibitType::CURVATURE_INVALID;
      curvature_res.inhibit_right_type =
          InhibitInfos::InhibitType::CURVATURE_INVALID;

      AINFO << "curvature limit exceeded: " << curvature
            << " limit = " << curvature_limit;
    }
  }

  // 4. localroute check
  InhibitInfos localroute_res;
  localroute_res.is_inhibit_left = false;
  localroute_res.is_inhibit_right = false;
  localroute_res.inhibit_left_type = InhibitInfos::InhibitType::NONE;
  localroute_res.inhibit_right_type = InhibitInfos::InhibitType::NONE;
  if (lane_infos.left_laneinfo.id == config_.env_info.invalid_lane) {
    localroute_res.is_inhibit_left = true;
    localroute_res.inhibit_left_type =
        InhibitInfos::InhibitType::LEFT_LANE_INVALID;
    AERROR << "InhibitInfos:Left Lane invalid";
  }

  if (lane_infos.right_laneinfo.id == config_.env_info.invalid_lane) {
    localroute_res.is_inhibit_right = true;
    localroute_res.inhibit_right_type =
        InhibitInfos::InhibitType::RIGHT_LANE_INVALID;
    AERROR << "InhibitInfos:Right Lane invalid";
  }

  // output result
  if (!main_road_res.is_inhibit_left && !cur_lane_type_res.is_inhibit_left &&

      !curvature_res.is_inhibit_left && !localroute_res.is_inhibit_left) {
    res.is_inhibit_left = false;
  } else {
    res.is_inhibit_left = true;
  }

  if (!main_road_res.is_inhibit_right && !cur_lane_type_res.is_inhibit_right &&

      !curvature_res.is_inhibit_right && !localroute_res.is_inhibit_right) {
    res.is_inhibit_right = false;
  } else {
    res.is_inhibit_right = true;
  }

  // output reason
  if (main_road_res.is_inhibit_left) {
    res.inhibit_left_type = InhibitInfos::InhibitType::CUR_LANE_INVALID;
  }
  if (main_road_res.is_inhibit_right) {
    res.inhibit_right_type = InhibitInfos::InhibitType::CUR_LANE_INVALID;
  }

  if (cur_lane_type_res.is_inhibit_left) {
    res.inhibit_left_type = InhibitInfos::InhibitType::LANE_TYPE_INVALID;
  }
  if (cur_lane_type_res.is_inhibit_right) {
    res.inhibit_right_type = InhibitInfos::InhibitType::LANE_TYPE_INVALID;
  }

  if (curvature_res.is_inhibit_left) {
    res.inhibit_left_type = InhibitInfos::InhibitType::CURVATURE_INVALID;
  }
  if (curvature_res.is_inhibit_right) {
    res.inhibit_right_type = InhibitInfos::InhibitType::CURVATURE_INVALID;
  }

  if (localroute_res.is_inhibit_left) {
    res.inhibit_left_type = InhibitInfos::InhibitType::LEFT_LANE_INVALID;
  }
  if (localroute_res.is_inhibit_right) {
    res.inhibit_right_type = InhibitInfos::InhibitType::RIGHT_LANE_INVALID;
  }

  AERROR << "ENVINFO is_inhibit_left:" << res.is_inhibit_left
         << " is_inhibit_right: " << res.is_inhibit_right;
  AERROR << "ENVINFO inhibit_left_type:" << res.inhibit_left_type
         << " inhibit_right_type: " << res.inhibit_right_type;

  return res;
}

}  // namespace planning
}  // namespace zark
