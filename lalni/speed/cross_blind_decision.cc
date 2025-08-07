
#include "speed/cross_blind_decision.h"

#include <algorithm>
#include <cmath>
#include <ostream>
#include <string>
#include <utility>

#include "absl/container/flat_hash_map.h"
#include "absl/strings/str_format.h"
#include "math/piecewise_linear_function.h"
#include "math/util.h"
#include "plan/planner_defs.h"
#include "speed/speed_limit.h"
#include "speed/speed_planning_util.h"
#include "speed/st_close_trajectory.h"
#include "speed/st_graph.h"
#include "speed_planning.pb.h"

namespace e2e_noa::planning {

void CrossCloseDecider(
    const CrossBlindInput& input,
    std::vector<StBoundaryWithDecision>* st_boundaries_with_decision,
    const std::vector<DrivingProcess>& driving_process_seq) {
  if (driving_process_seq.empty()) {
    return;
  }
  const auto& path_points = *input.path;
  const auto driving_zone =
      std::lower_bound(driving_process_seq.begin(), driving_process_seq.end(),
                       path_points.front().s(),
                       [](const DrivingProcess& driving_process, double av_s) {
                         return driving_process.end_s < av_s;
                       });
  if (driving_zone == driving_process_seq.end()) {
    return;
  }
  if (driving_zone->zone_type != InteractionZone::Straight &&
      driving_zone->zone_type != InteractionZone::JunctionStraight) {
    return;
  }
  if (driving_zone->zone_type == InteractionZone::Straight) {
    Log2FG::LogDataV0("CrossCloseDecider", "on Straight");
  }
  if (driving_zone->zone_type == InteractionZone::JunctionStraight) {
    Log2FG::LogDataV0("CrossCloseDecider", "on JunctionStraight");
  }

  const auto& st_traj_mgr = *input.st_traj_mgr;
  const auto& moving_spacetime_objects = st_traj_mgr.moving_object_trajs();

  absl::flat_hash_map<std::string, StBoundaryWithDecision*>
      cross_obs_st_boundary_wd_map;
  std::vector<std::pair<std::string, Segment2d>> fov_lines;
  for (auto& st_boundary_wd : *st_boundaries_with_decision) {
    const auto& st_boundary = *st_boundary_wd.raw_st_boundary();
    auto obj_scene_info = st_boundary.obj_scenario_info();
    if (st_boundary.source_type() != StBoundarySourceTypeProto::ST_OBJECT) {
      continue;
    }
    if (st_boundary.object_type() != StBoundaryProto::VEHICLE) {
      continue;
    }
    const auto& overlap_meta = *st_boundary.overlap_meta();
    if (overlap_meta.pattern() == StOverlapMetaProto::CROSS) {
      if (obj_scene_info.interaction_zone !=
              InteractionZone::JunctionStraight &&
          obj_scene_info.interaction_zone != InteractionZone::Straight) {
        continue;
      }
      CHECK(st_boundary.traj_id().has_value());
      const auto& traj_id = st_boundary.traj_id();
      CHECK(st_boundary.object_id().has_value());
      const auto object_id = st_boundary.object_id().value();
      const auto* traj =
          CHECK_NOTNULL(st_traj_mgr.FindTrajectoryById(*traj_id));

      const auto& overlap_infos = st_boundary.overlap_infos();
      CHECK(!overlap_infos.empty());
      const auto& first_overlap_info = overlap_infos.front();
      const auto* first_overlap_obj_point =
          traj->states()[first_overlap_info.obj_idx].traj_point;
      const auto first_overlap_obj_heading = first_overlap_obj_point->theta();
      const auto first_overlap_av_middle_point =
          path_points[(first_overlap_info.av_start_idx +
                       first_overlap_info.av_end_idx) /
                      2];
      const auto first_overlap_av_middle_heading =
          first_overlap_av_middle_point.theta();
      constexpr double kOnComingThreshold_upper = 10.0 * M_PI / 18.0;
      constexpr double kOnComingThreshold_lower = 8.0 * M_PI / 18.0;

      double heading_diff = std::abs(NormalizeAngle(
          first_overlap_obj_heading - first_overlap_av_middle_heading));

      if (heading_diff < kOnComingThreshold_lower ||
          heading_diff > kOnComingThreshold_upper) {
        continue;
      }

      if (obj_scene_info.delta_heading < kOnComingThreshold_lower ||
          obj_scene_info.delta_heading > kOnComingThreshold_upper) {
        continue;
      }
      double av_time = first_overlap_av_middle_point.s() / input.current_v;
      double obs_time =
          first_overlap_obj_point->s() / traj->planner_object().pose().v();
      if (std::fabs(av_time - obs_time) > 3.0) {
        continue;
      }
      cross_obs_st_boundary_wd_map.emplace(object_id, &st_boundary_wd);
      double frontAxle_x =
          path_points[0].x() +
          input.vehicle_geom->wheel_base() * cos(path_points[0].theta());
      double frontAxle_y =
          path_points[0].y() +
          input.vehicle_geom->wheel_base() * sin(path_points[0].theta());
      fov_lines.emplace_back(std::make_pair(
          object_id, Segment2d(traj->planner_object().pose().pos(),
                               Vec2d(frontAxle_x, frontAxle_y))));
    }
  }

  const auto& stationary_trajectories = st_traj_mgr.stationary_objects();
  for (auto& [id, fov_line_seg] : fov_lines) {
    std::vector<std::string> blind_obs;
    for (size_t i = 0; i < stationary_trajectories.size(); i++) {
      const auto obj = stationary_trajectories.at(i);
      const std::string object_id(obj.object_id);
      const ObjectProto& object_proto = obj.planner_object.object_proto();
      if (object_proto.has_min_z() && object_proto.has_max_z() &&
          object_proto.has_ground_z() &&
          object_proto.max_z() - object_proto.ground_z() < 4.0) {
        continue;
      }
      const Box2d& obj_box = obj.planner_object.bounding_box();
      if (obj_box.area() < 5.0) {
        continue;
      }
      if (obj_box.HasOverlapWithBuffer(fov_line_seg, 1.5, 0.0)) {
        blind_obs.push_back(object_id);
      }
    }

    if (!blind_obs.empty()) {
      auto cross_st_boundary_wd = cross_obs_st_boundary_wd_map[id];
      Log2FG::LogDataV0("CrossCloseDecider", "cross blind:" + id);
      auto& obj_scene_info = cross_st_boundary_wd->mutable_raw_st_boundary()
                                 ->mutable_obj_scenario_info();

      auto& param = obj_scene_info.obj_decision_param;
      {
        param.dp_follow_lead_ratio = 1.5;
        param.pass_time_additional_buffer = 1.5;
        param.yield_time_additional_buffer = 0.5;
        param.agent_reaction_time = 2.0;
        param.geometry_theory_av_follow_distance = 0.5;
        cross_st_boundary_wd->set_follow_standstill_distance(
            cross_st_boundary_wd->follow_standstill_distance() + 3.0);
        continue;
      }
    }
  }
}

}  // namespace e2e_noa::planning
