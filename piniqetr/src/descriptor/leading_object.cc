#include "descriptor/leading_object.h"

#include <float.h>

#include <algorithm>
#include <cmath>
#include <string>
#include <string_view>
#include <utility>

#include "absl/container/flat_hash_set.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"
#include "absl/types/span.h"
#include "descriptor/descriptor_util.h"
#include "lane_path.pb.h"
#include "maps/lane_path.h"
#include "maps/semantic_map_defs.h"
#include "math/util.h"
#include "math/vec.h"
#include "object/planner_object.h"
#include "object/spacetime_object_trajectory.h"
#include "object/spacetime_planner_object_trajectories_filter.h"
#include "plan/trajectorypoint_with_acceleration.h"
#include "util/status_macros.h"

namespace e2e_noa {
namespace planning {

namespace {

using ObjectsOnLane =
    std::vector<std::pair<FrenetBox, const SpacetimeObjectTrajectory*>>;

absl::StatusOr<bool> IsOncomingObjectJudgeByPlanPassage(
    const PlanPassage& passage,
    const TrajectoryPointWithAcceleration& obj_pose) {
  ASSIGN_OR_RETURN(const auto tangent, passage.QueryTangentAt(obj_pose.pos()));
  const double passage_angle = tangent.Angle();
  const double angle_diff =
      std::abs(NormalizeAngle(passage_angle - obj_pose.theta()));

  return angle_diff > M_PI_2;
}

bool IsOncomingObjectJudgeByEgoHeading(
    const ApolloTrajectoryPointProto& plan_start_point,
    const TrajectoryPointWithAcceleration& obj_pose) {
  const double ego_heading_angle = plan_start_point.path_point().theta();
  const double object_heading_angle = obj_pose.theta();
  const double angle_diff =
      std::abs(NormalizeAngle(ego_heading_angle - object_heading_angle));

  return angle_diff > M_PI_2;
}

absl::StatusOr<FrenetBox> FilterObjectViaPlanPassage(
    const PlannerObject& object, const PlanPassage& passage,
    const PathSlBoundary& sl_boundary, const FrenetBox& ego_frenet_box,
    const absl::flat_hash_set<std::string>& stalled_objects,
    absl::flat_hash_set<std::string>& collect_stdlled) {
  ASSIGN_OR_RETURN(const auto object_frenet_box,
                   passage.QueryFrenetBoxAtContour(object.contour()));

  constexpr double kLateralEnterThres = 0.5;
  const auto [boundary_l_max, boundary_l_min] =
      CalcSlBoundaries(sl_boundary, object_frenet_box);

  if (object_frenet_box.l_min > boundary_l_max - kLateralEnterThres ||
      object_frenet_box.l_max < boundary_l_min + kLateralEnterThres) {
    return absl::OutOfRangeError(absl::StrFormat(
        "Object %s out of lateral boundary, l range: (%.2f, %.2f)", object.id(),
        object_frenet_box.l_min, object_frenet_box.l_max));
  }

  if (stalled_objects.find(object.id()) != stalled_objects.end() &&
      ego_frenet_box.center_l() > object_frenet_box.center_l()) {
    collect_stdlled.insert(object.id());
  }

  if (object_frenet_box.s_min < ego_frenet_box.s_max ||
      object_frenet_box.s_min > sl_boundary.end_s()) {
    return absl::OutOfRangeError(absl::StrFormat(
        "Object %s out of longitudinal boundary, s range: ( %.2f, %.2f)",
        object.id(), object_frenet_box.s_min, object_frenet_box.s_max));
  }
  return object_frenet_box;
}

ObjectsOnLane FindFrontObjectsOnLane(
    const PlanPassage& passage, const PathSlBoundary& sl_boundary,
    absl::Span<const SpacetimeObjectTrajectory> st_trajs,
    const FrenetBox& ego_frenet_box,
    absl::flat_hash_set<std::string>& collect_stdlled,
    const absl::flat_hash_set<std::string>& stalled_objects) {
  ObjectsOnLane st_trajs_on_lane;
  st_trajs_on_lane.reserve(st_trajs.size());

  for (const auto& st_traj : st_trajs) {
    if (!IsLeadingObjectType(st_traj.planner_object().type())) {
      continue;
    }

    const auto res =
        IsOncomingObjectJudgeByPlanPassage(passage, st_traj.pose());
    if ((!res.ok() || *res == true) &&
        stalled_objects.find(st_traj.planner_object().id()) ==
            stalled_objects.end()) {
      continue;
    }

    ASSIGN_OR_CONTINUE(const auto obj_fbox,
                       FilterObjectViaPlanPassage(
                           st_traj.planner_object(), passage, sl_boundary,
                           ego_frenet_box, stalled_objects, collect_stdlled));

    st_trajs_on_lane.emplace_back(obj_fbox, &st_traj);
  }

  std::stable_sort(st_trajs_on_lane.begin(), st_trajs_on_lane.end(),
                   [](const auto& a, const auto& b) {
                     return a.first.s_min < b.first.s_min;
                   });

  return st_trajs_on_lane;
}

bool IsUnsafeType(ObjectType type) {
  switch (type) {
    case OT_VEHICLE:
    case OT_LARGE_VEHICLE:
    case OT_MOTORCYCLIST:
    case OT_TRICYCLIST:
      return true;
    case OT_UNKNOWN_MOVABLE:
    case OT_UNKNOWN_STATIC:
    case OT_PEDESTRIAN:
    case OT_CYCLIST:

    case OT_FOD:
    case OT_VEGETATION:
    case OT_BARRIER:
    case OT_CONE:
    case OT_WARNING_TRIANGLE:
      return false;
    default:
      throw std::runtime_error("switch case on enum unexpected");
  }
}

void IsEnvUnsafe(absl::Span<const SpacetimeObjectTrajectory> st_trajs,
                 const PlanPassage& passage, const FrenetBox& ego_frenet_box,
                 std::set<std::string>& l_unsafe_set,
                 std::set<std::string>& r_unsafe_set,
                 const ApolloTrajectoryPointProto& plan_start_point,
                 double lat_thr, const std::string& judge_obj_id) {
  for (const auto& st_traj : st_trajs) {
    const double ego_heading_angle = plan_start_point.path_point().theta();
    const double object_heading_angle = st_traj.pose().theta();
    const std::string obj_id = std::string(st_traj.object_id());
    double obj_v = st_traj.pose().v();
    const double angle_diff =
        NormalizeAngle(ego_heading_angle - object_heading_angle);
    if (!IsUnsafeType(st_traj.planner_object().type()) ||
        obj_id == judge_obj_id)
      continue;
    std::string env_debug = " angle_diff: " + std::to_string(angle_diff) +
                            " lat_thr: " + std::to_string(lat_thr);
    const auto object_frenet_box =
        passage.QueryFrenetBoxAtContour(st_traj.planner_object().contour());
    if (object_frenet_box.ok()) {
      if (std::abs(angle_diff) > M_PI_2) {
        if (((object_frenet_box.value().l_min > ego_frenet_box.l_max &&
              object_frenet_box.value().l_min <
                  ego_frenet_box.l_max + lat_thr))) {
          if (ego_frenet_box.s_min < object_frenet_box.value().s_max &&
              ego_frenet_box.s_max + (plan_start_point.v() + obj_v) >
                  object_frenet_box.value().s_min) {
            l_unsafe_set.insert(st_traj.planner_object().id());
            env_debug += " is_resleft_front_ttc " + obj_id +
                         " v: " + std::to_string(obj_v);
          }
        }
      } else if (((object_frenet_box.value().l_min > ego_frenet_box.l_max &&
                   object_frenet_box.value().l_min <
                       ego_frenet_box.l_max + lat_thr) &&
                  angle_diff > -M_PI / 18) ||
                 ((object_frenet_box.value().l_max < ego_frenet_box.l_min &&
                   object_frenet_box.value().l_max >
                       ego_frenet_box.l_min - lat_thr) &&
                  angle_diff < M_PI / 18)) {
        if (ego_frenet_box.s_max < object_frenet_box.value().s_min &&
            (ego_frenet_box.s_max + 2 * (plan_start_point.v() - obj_v) >
             object_frenet_box.value().s_min)) {
          if (object_frenet_box.value().l_min > ego_frenet_box.l_max) {
            l_unsafe_set.insert(st_traj.planner_object().id());
            env_debug +=
                " is_left_front_ttc " + obj_id + " v: " + std::to_string(obj_v);
          } else if (object_frenet_box.value().l_max < ego_frenet_box.l_min) {
            r_unsafe_set.insert(st_traj.planner_object().id());
            env_debug += " is_right_front_ttc " + obj_id +
                         " v: " + std::to_string(obj_v);
          }
        } else if (ego_frenet_box.s_min > object_frenet_box.value().s_max &&
                   (ego_frenet_box.s_min + 2 * (plan_start_point.v() - obj_v) <
                    object_frenet_box.value().s_max)) {
          if (object_frenet_box.value().l_min > ego_frenet_box.l_max) {
            l_unsafe_set.insert(st_traj.planner_object().id());
            env_debug +=
                " is_left_back_ttc " + obj_id + " v: " + std::to_string(obj_v);
          } else if (object_frenet_box.value().l_max < ego_frenet_box.l_min) {
            r_unsafe_set.insert(st_traj.planner_object().id());
            env_debug +=
                " is_right_back_ttc " + obj_id + " v: " + std::to_string(obj_v);
          }
        } else if ((ego_frenet_box.s_min > object_frenet_box.value().s_min &&
                    ego_frenet_box.s_min < object_frenet_box.value().s_max) ||
                   (ego_frenet_box.s_max < object_frenet_box.value().s_max &&
                    ego_frenet_box.s_max > object_frenet_box.value().s_min)) {
          if (object_frenet_box.value().l_min > ego_frenet_box.l_max) {
            l_unsafe_set.insert(st_traj.planner_object().id());
            env_debug +=
                " is_left_use_pos " + obj_id + " v: " + std::to_string(obj_v);
          } else if (object_frenet_box.value().l_max < ego_frenet_box.l_min) {
            r_unsafe_set.insert(st_traj.planner_object().id());
            env_debug +=
                " is_right_use_pos " + obj_id + " v: " + std::to_string(obj_v);
          }
        }
      } else {
        bool is_lat_enough =
            (object_frenet_box.value().l_min > ego_frenet_box.l_max &&
             object_frenet_box.value().l_min < ego_frenet_box.l_max + lat_thr);
        env_debug +=
            " lat_enough: " + obj_id + " v: " + std::to_string(obj_v) +
            " is_lat_enough: " + std::to_string(is_lat_enough) +
            " obj_l_min: " + std::to_string(object_frenet_box.value().l_min) +
            " ego_frenet_box.l_max: " + std::to_string(ego_frenet_box.l_max);
      }
    } else {
      env_debug += " obj_no_box: " + obj_id + " v: " + std::to_string(obj_v);
    }
  }
  return;
}

void IsEnvUnsafe(absl::Span<const SpacetimeObjectTrajectory> st_trajs,
                 const PlanPassage& passage, const FrenetBox& ego_frenet_box,
                 bool& is_left, bool& is_right, const std::string& obj_id,
                 const ApolloTrajectoryPointProto& plan_start_point) {
  constexpr double kLateralEnterThres = 1.5;
  for (const auto& st_traj : st_trajs) {
    std::string env_debug = "";
    const double obj_v = st_traj.pose().v();
    if (obj_id == st_traj.planner_object().id()) continue;
    const auto object_frenet_box =
        passage.QueryFrenetBoxAtContour(st_traj.planner_object().contour());
    if (object_frenet_box.ok()) {
      if ((object_frenet_box.value().l_min > ego_frenet_box.l_max &&
           object_frenet_box.value().l_min <
               ego_frenet_box.l_max + kLateralEnterThres) ||
          (object_frenet_box.value().l_max < ego_frenet_box.l_min &&
           object_frenet_box.value().l_max >
               ego_frenet_box.l_min - kLateralEnterThres)) {
        if (ego_frenet_box.s_max < object_frenet_box.value().s_min &&
            (ego_frenet_box.s_max + 2 * (plan_start_point.v() - obj_v) >
             object_frenet_box.value().s_min)) {
          if (object_frenet_box.value().l_min > ego_frenet_box.l_max) {
            is_left = true;
            env_debug += " is_left: " + std::to_string(is_left) + " front_ttc";

          } else if (object_frenet_box.value().l_max < ego_frenet_box.l_min) {
            is_right = true;
            env_debug +=
                " is_right: " + std::to_string(is_right) + " front_ttc";
          }
        } else if (ego_frenet_box.s_min > object_frenet_box.value().s_max &&
                   (ego_frenet_box.s_min + 2 * (plan_start_point.v() - obj_v) <
                    object_frenet_box.value().s_max)) {
          if (object_frenet_box.value().l_min > ego_frenet_box.l_max) {
            is_left = true;
            env_debug += " is_left: " + std::to_string(is_left) + " back_ttc";

          } else if (object_frenet_box.value().l_max < ego_frenet_box.l_min) {
            is_right = true;
            env_debug += " is_right: " + std::to_string(is_left) + " back_ttc";
          }
        } else if ((ego_frenet_box.s_min > object_frenet_box.value().s_min &&
                    ego_frenet_box.s_min < object_frenet_box.value().s_max) ||
                   (ego_frenet_box.s_max < object_frenet_box.value().s_max &&
                    ego_frenet_box.s_max > object_frenet_box.value().s_min)) {
          if (object_frenet_box.value().l_min > ego_frenet_box.l_max) {
            is_left = true;
            env_debug += " is_left: " + std::to_string(is_left) + " use_pos";

          } else if (object_frenet_box.value().l_max < ego_frenet_box.l_min) {
            is_right = true;
            env_debug += " is_right: " + std::to_string(is_right) + " use_pos";
          }
        }
      } else {
        env_debug += " lat_enough";
      }
    } else {
      env_debug += " obj_no_box";
    }
    env_debug += " id: " + obj_id;
  }
  return;
}

bool IsObjectAvoidableWithinEnv(
    const PlanPassage& passage, const FrenetBox& obj_frenet_box,
    const std::string obj_id,
    absl::Span<const SpacetimeObjectTrajectory> st_trajs, const bool& l_space,
    const bool& r_space, const NudgeObjectInfo* nudge_object_info,
    const FrenetBox& ego_frenet_box, bool is_stationary) {
  constexpr double kLateralEnterThres = 2.5;
  std::string obj_env_debug = " obj_id: " + obj_id;
  obj_env_debug += " l_space: " + std::to_string(l_space) +
                   " r_space: " + std::to_string(r_space) +
                   " ego_s_min: " + std::to_string(obj_frenet_box.s_min) +
                   "ego_s_max: " + std::to_string(obj_frenet_box.s_max);
  bool UnOverlap = (obj_frenet_box.l_min > ego_frenet_box.l_max ||
                    obj_frenet_box.l_max < ego_frenet_box.l_min);
  if (nudge_object_info || !is_stationary) return true;
  if (UnOverlap) return true;
  bool left_unsafe = false;
  bool right_unsafe = false;
  for (const auto& st_traj : st_trajs) {
    const double obj_v = st_traj.pose().v();
    const std::string other_id = st_traj.planner_object().id();
    if (obj_id == st_traj.planner_object().id() || !st_traj.is_stationary() ||
        obj_v != 0.0)
      continue;
    const auto object_frenet_box =
        passage.QueryFrenetBoxAtContour(st_traj.planner_object().contour());
    if (object_frenet_box.ok()) {
      bool is_lon_unverlap =
          object_frenet_box.value().s_min > obj_frenet_box.s_max + 2.0 ||
          object_frenet_box.value().s_max < obj_frenet_box.s_min - 2.0;
      std::string sl_debug =
          " obj_s_min: " + std::to_string(object_frenet_box.value().s_min) +
          " obj_s_max: " + std::to_string(object_frenet_box.value().s_max);
      if (!is_lon_unverlap) {
        bool LatUnOverlap =
            (obj_frenet_box.l_min > object_frenet_box.value().l_max ||
             obj_frenet_box.l_max < object_frenet_box.value().l_min);
        if (!LatUnOverlap) return true;
        if (object_frenet_box.value().center_l() > obj_frenet_box.center_l() &&
            object_frenet_box.value().l_min <
                obj_frenet_box.l_max + kLateralEnterThres) {
          obj_env_debug += sl_debug;
          obj_env_debug += " left is ununsafe " + other_id;
          left_unsafe = true;
        } else if (object_frenet_box.value().center_l() <
                       obj_frenet_box.center_l() &&
                   object_frenet_box.value().l_max >
                       obj_frenet_box.l_min - kLateralEnterThres) {
          obj_env_debug += sl_debug;
          obj_env_debug += " right is ununsafe " + other_id;
          right_unsafe = true;
        }
      }
    }
  }
  if (r_space && !right_unsafe) return true;
  if (l_space && !left_unsafe) return true;
  return false;
}

absl::flat_hash_set<std::string_view> CollectTrafficWaitingObjectOnCurrentLane(
    const SceneOutputProto& scene_reasoning,
    const mapping::LanePath& lane_path) {
  absl::flat_hash_set<std::string_view> traffic_waiting_objects;
  std::string traffic_wait_debug = " ";
  int size_queue = scene_reasoning.traffic_waiting_queue().size();
  traffic_wait_debug += std::to_string(size_queue);
  for (const auto& traffic_waiting_queue :
       scene_reasoning.traffic_waiting_queue()) {
    traffic_wait_debug += "---------";
    const auto& lane_ids = lane_path.lane_ids();
    if (!traffic_waiting_queue.has_lane_path() ||
        traffic_waiting_queue.lane_path().lane_ids().empty()) {
      continue;
    }
    const mapping::ElementId first_lane_id(
        traffic_waiting_queue.lane_path().lane_ids(0));

    if (std::find(lane_ids.begin(), lane_ids.end(), first_lane_id) !=
        lane_ids.end()) {
      for (const auto& object_id : traffic_waiting_queue.object_id()) {
        traffic_wait_debug += " obj_id " + object_id;
        traffic_waiting_objects.insert(object_id);
      }
    }
  }
  return traffic_waiting_objects;
}

bool IsObjectAvoidableWithinCurbBuffer(
    const PlanPassage& passage, const FrenetBox& obj_frenet_box,
    const PathSlBoundary& sl_boundary, const double& lead_threshold,
    const std::string& obj_id, const double& ego_width, const bool& l_space,
    const bool& r_space) {
  constexpr double kSampleStepAlongS = 1.0;
  double min_left_space = DBL_MAX, min_right_space = DBL_MAX;
  for (double sample_s = obj_frenet_box.s_min; sample_s <= obj_frenet_box.s_max;
       sample_s += kSampleStepAlongS) {
    const auto curb_l = passage.QueryCurbOffsetAtS(sample_s);
    if (!curb_l.ok()) {
      continue;
    }

    const double right_l = curb_l->first + 0.5;
    const double left_l = curb_l->second - 0.5;
    min_left_space = std::clamp(
        left_l - std::max(right_l, obj_frenet_box.l_max), 0.0, min_left_space);
    min_right_space = std::clamp(
        std::min(left_l, obj_frenet_box.l_min) - right_l, 0.0, min_right_space);
  }
  const std::string curb_info =
      "id:" + obj_id + " min_left_space: " + std::to_string(min_left_space) +
      " min_right_space: " + std::to_string(min_right_space) + " l_space; " +
      std::to_string(l_space) + " r_space; " + std::to_string(r_space);

  if (l_space && r_space) {
    return std::max(min_left_space, min_right_space) > ego_width;
  } else if (l_space) {
    return min_left_space > ego_width;
  }
  return min_right_space > ego_width;
}

bool IsObjectAvoidableWithinSlBoundary(
    const PathSlBoundary& sl_boundary, const FrenetBox& obj_frenet_box,
    double ego_width, double lead_threshold, const std::string id,
    bool is_out_boundary, const std::set<std::string>& l_unsafe_set,
    const std::set<std::string>& r_unsafe_set,
    const NudgeObjectInfo* nudge_object_info, const bool& is_lane_change,
    bool judge_type, const FrenetBox& ego_frenet_box) {
  constexpr double kSampleStepAlongS = 1.0;
  double min_left_space = DBL_MAX, min_right_space = DBL_MAX;

  bool UnOverlap = (obj_frenet_box.l_min > ego_frenet_box.l_max ||
                    obj_frenet_box.l_max < ego_frenet_box.l_min);
  for (double sample_s = obj_frenet_box.s_min; sample_s <= obj_frenet_box.s_max;
       sample_s += kSampleStepAlongS) {
    const auto [right_l, left_l] =
        is_out_boundary ? sl_boundary.QueryBoundaryL(sample_s)
                        : sl_boundary.QueryTargetBoundaryL(sample_s);
    min_left_space = std::clamp(
        left_l - std::max(right_l, obj_frenet_box.l_max), 0.0, min_left_space);
    min_right_space = std::clamp(
        std::min(left_l, obj_frenet_box.l_min) - right_l, 0.0, min_right_space);
  }
  const std::string obj_thr =
      "id: " + id + "leading_threshold: " + std::to_string(lead_threshold) +
      " l_unsafe_set_size: " + std::to_string(l_unsafe_set.size()) +
      " r_unsafe_set_size: " + std::to_string(r_unsafe_set.size()) +
      " min_left_space: " + std::to_string(min_left_space) +
      " min_right_space: " + std::to_string(min_right_space);
  " judge_type: " + std::to_string(judge_type);
  if (!is_lane_change && judge_type && !UnOverlap) {
    if (min_left_space > (ego_width + lead_threshold) && !nudge_object_info) {
      if (l_unsafe_set.size() > 1 ||
          (l_unsafe_set.size() == 1 &&
           l_unsafe_set.find(id) == l_unsafe_set.end())) {
        return false;
      }
    } else if (min_right_space > (ego_width + lead_threshold) &&
               !nudge_object_info) {
      if (r_unsafe_set.size() > 1 ||
          (r_unsafe_set.size() == 1 &&
           r_unsafe_set.find(id) == r_unsafe_set.end())) {
        return false;
      }
    }
  }

  return std::max(min_left_space, min_right_space) >
         (ego_width + lead_threshold);
}

bool IsObjectAvoidableWithinSlBoundary(
    const PathSlBoundary& sl_boundary, const FrenetBox& obj_frenet_box,
    double ego_width, double lead_threshold, const std::string id,
    const NudgeObjectInfo* nudge_object_info, const bool& is_lane_change,
    const std::pair<std::string, double>& nearest_obj,
    const PlanPassage& passage, const FrenetBox& ego_frenet_box,
    const ApolloTrajectoryPointProto& plan_start_point,
    const SpacetimeTrajectoryManager& st_traj_mgr, bool& l_space, bool& r_space,
    bool is_stalled, const double& obj_v, bool& is_emergency) {
  bool is_nearest = (id == nearest_obj.first) ? true : false;
  std::set<std::string> l_set, r_set;

  constexpr double kSampleStepAlongS = 1.0;
  double min_left_space = DBL_MAX, min_right_space = DBL_MAX;

  for (double sample_s = obj_frenet_box.s_min; sample_s <= obj_frenet_box.s_max;
       sample_s += kSampleStepAlongS) {
    const auto [right_l, left_l] = sl_boundary.QueryTargetBoundaryL(sample_s);
    min_left_space = std::clamp(
        left_l - std::max(right_l, obj_frenet_box.l_max), 0.0, min_left_space);
    min_right_space = std::clamp(
        std::min(left_l, obj_frenet_box.l_min) - right_l, 0.0, min_right_space);
  }
  if (is_nearest && !is_lane_change && !nudge_object_info) {
    IsEnvUnsafe(st_traj_mgr.trajectories(), passage, ego_frenet_box, l_set,
                r_set, plan_start_point, nearest_obj.second, id);
  }

  std::string obj_thr = "id: " + id +
                        "leading_threshold: " + std::to_string(lead_threshold) +
                        " l_unsafe_set_size: " + std::to_string(l_set.size()) +
                        " r_unsafe_set_size: " + std::to_string(r_set.size()) +
                        " min_left_space: " + std::to_string(min_left_space) +
                        " min_right_space: " + std::to_string(min_right_space);
  if (is_stalled && !is_nearest) {
    l_space = true;
    r_space = true;
    return true;
  }
  if (is_stalled) {
    l_space = true;
    r_space = true;
    if (ego_frenet_box.center_l() >= obj_frenet_box.center_l() &&
        !l_set.empty())
      return false;
    if (ego_frenet_box.center_l() < obj_frenet_box.center_l() && !r_set.empty())
      return false;
    return true;
  }

  l_space = min_left_space > (ego_width + lead_threshold);
  r_space = min_right_space > (ego_width + lead_threshold);

  if (!l_space && !r_space && nudge_object_info) {
    const double comfor_a = -2.0;
    if (obj_v < plan_start_point.v()) {
      double safe_s = (Sqr(obj_v) - Sqr(plan_start_point.v())) / (2 * comfor_a);
      double dist_to_obj = obj_frenet_box.s_min - ego_frenet_box.s_max;
      bool is_comf = dist_to_obj > safe_s;
      obj_thr += " safe_s: " + std::to_string(safe_s) + " dist_to_obj " +
                 std::to_string(dist_to_obj) +
                 " is_comf: " + std::to_string(is_comf);
      if (!is_comf) {
        is_emergency = true;
        return true;
      }
    }
  }

  if (min_left_space > (ego_width + lead_threshold) && !nudge_object_info) {
    if (!l_set.empty()) {
      return false;
    }
  } else if (min_right_space > (ego_width + lead_threshold) &&
             !nudge_object_info) {
    if (!r_set.empty()) {
      return false;
    }
  }

  return std::max(min_left_space, min_right_space) >
         (ego_width + lead_threshold);
}

bool IsObjectAvoidableWithinSlBoundary(
    const PathSlBoundary& sl_boundary, const FrenetBox& obj_frenet_box,
    double ego_width, double lead_threshold, const std::string id,
    bool is_out_boundary, const SpacetimeTrajectoryManager& st_traj_mgr,
    const ApolloTrajectoryPointProto& plan_start_point,
    const PlanPassage& passage, const FrenetBox& ego_frenet_box) {
  bool l_unsafe = false, r_unsafe = false;
  IsEnvUnsafe(st_traj_mgr.trajectories(), passage, ego_frenet_box, l_unsafe,
              r_unsafe, id, plan_start_point);
  constexpr double kSampleStepAlongS = 1.0;
  double min_left_space = DBL_MAX, min_right_space = DBL_MAX;
  for (double sample_s = obj_frenet_box.s_min; sample_s <= obj_frenet_box.s_max;
       sample_s += kSampleStepAlongS) {
    const auto [right_l, left_l] =
        is_out_boundary ? sl_boundary.QueryBoundaryL(sample_s)
                        : sl_boundary.QueryTargetBoundaryL(sample_s);
    min_left_space = std::clamp(
        left_l - std::max(right_l, obj_frenet_box.l_max), 0.0, min_left_space);
    min_right_space = std::clamp(
        std::min(left_l, obj_frenet_box.l_min) - right_l, 0.0, min_right_space);
  }
  const std::string obj_thr =
      "id: " + id + "leading_threshold: " + std::to_string(lead_threshold) +
      " l_unsafe: " + std::to_string(l_unsafe);
  if (min_left_space > (ego_width + lead_threshold) && l_unsafe) return false;
  if (min_left_space > (ego_width + lead_threshold) && r_unsafe) return false;

  return std::max(min_left_space, min_right_space) >
         (ego_width + lead_threshold);
}

bool IsLargeVehicleAvoidableWithinSlBoundary(
    const PlannerSemanticMapManager& psmm, const PlanPassage& passage,
    const ApolloTrajectoryPointProto& plan_start_point,
    const PathSlBoundary& sl_boundary, const FrenetBox& obj_frenet_box,
    const SpacetimeObjectTrajectory& traj_ptr, const double ego_width) {
  if (traj_ptr.is_stationary()) return true;

  const ObjectType obj_type = traj_ptr.planner_object().type();
  if (obj_type != OT_LARGE_VEHICLE) return true;

  if (obj_frenet_box.s_min - plan_start_point.path_point().s() > 15.0) {
    return true;
  }

  if (obj_frenet_box.l_max * obj_frenet_box.l_min < 0.0) return true;
  const auto [right_width, left_width] =
      sl_boundary.QueryTargetBoundaryL(obj_frenet_box.center_s());
  bool is_left = (obj_frenet_box.center_l() > 0.0);
  double target_l_lane = is_left ? left_width : right_width;
  double l_obj_edge = is_left ? obj_frenet_box.l_min : obj_frenet_box.l_max;
  if (std::fabs(l_obj_edge) > std::fabs(target_l_lane)) return true;

  const Vec2d ego_pos = Extract2dVectorFromApolloProto(plan_start_point);
  const auto& cur_lane = psmm.GetNearestLane(ego_pos);
  if (!cur_lane || !(cur_lane->junction_id().empty())) return true;

  const auto& neighbor_lane_id = cur_lane->right_lane_id();
  if (!neighbor_lane_id.empty()) return true;

  const double step_s = 1.0, lead_threshold = 1.4;
  const double preview_s = 10.0;
  double min_left_space = DBL_MAX, min_right_space = DBL_MAX;
  for (double sample_s = obj_frenet_box.s_min;
       sample_s <= obj_frenet_box.s_max + preview_s; sample_s += step_s) {
    const auto curb_offset_or = passage.QueryCurbOffsetAtS(sample_s);
    if (!curb_offset_or.ok()) continue;
    min_left_space =
        std::clamp(curb_offset_or->second -
                       std::max(curb_offset_or->first, obj_frenet_box.l_max),
                   0.0, min_left_space);
    min_right_space =
        std::clamp(std::min(curb_offset_or->second, obj_frenet_box.l_min) -
                       curb_offset_or->first,
                   0.0, min_right_space);
  }
  return std::max(min_left_space, min_right_space) >
         (ego_width + lead_threshold);
}

bool IsObjectWithinTlControlledIntersection(
    const ad_e2e::planning::TrafficLightStatusMap& tl_status_map,
    const PlannerSemanticMapManager& psmm, const FrenetBox& obj_frenet_box,
    const mapping::LanePath& lane_path, double s_offset) {
  const double const_zero = 1e-5;
  const double obj_center_s = obj_frenet_box.center_s();
  const double obj_center_l = obj_frenet_box.center_l();
  for (const auto& seg : lane_path) {
    const auto lane_info_ptr = psmm.map_ptr()->GetLaneById(seg.lane_id);
    if (lane_info_ptr == nullptr) continue;
    if (lane_info_ptr->junction_id().empty()) continue;

    if (!tl_status_map.empty()) {
      const auto& tl_lane_info = tl_status_map.find(seg.lane_id);
      if (tl_status_map.end() == tl_lane_info) {
        return false;
      } else {
        const auto& traffic_light_status = tl_lane_info->second;
        if (traffic_light_status.junction_id.has_value() &&
            ad_e2e::planning::LightStatus::NONE_LIGHT ==
                traffic_light_status.light_status) {
          return false;
        }
      }
    }
    if (lane_info_ptr->left_lane_id().empty() && obj_center_l > 0.5 &&
        lane_info_ptr->turn_type() != TurnType::LEFT_TURN) {
      return false;
    }

    if (seg.end_s + const_zero > obj_center_s && seg.start_s < obj_center_s) {
      double left_width = 0.0, right_width = 0.0;
      lane_info_ptr->GetWidthFromS(obj_center_s - seg.start_s, &left_width,
                                   &right_width);
      if (-right_width < obj_center_l && obj_center_l < left_width) {
        return true;
      }
    }

    if (seg.end_s + s_offset > obj_frenet_box.s_max) break;
  }
  return false;
}

bool IsObjectBlockingRefCenter(const PathSlBoundary& sl_boundary,
                               const FrenetBox& obj_frenet_box,
                               double ego_half_width) {
  const double obj_l_offset =
      sl_boundary.QueryReferenceCenterL(obj_frenet_box.center_s());

  return obj_frenet_box.l_max > obj_l_offset - ego_half_width &&
         obj_frenet_box.l_min < obj_l_offset + ego_half_width;
}

bool IsUnsafe(const FrenetBox& obj_frenet_box, const FrenetBox& ego_frenet_box,
              const SpacetimeObjectTrajectory& traj_ptr,
              const ApolloTrajectoryPointProto& plan_start_point,
              const std::string& obj_id, bool is_nudge,
              const NudgeObjectInfo* nudge_object_info,
              const bool& is_lane_change, const double& ego_width,
              const double& ego_length, const PlanPassage& passage) {
  double lon_dist = obj_frenet_box.s_min - ego_frenet_box.s_max;
  double lat_overlap = 0.0;
  double front_safe_dis = 0.0;
  std::vector<double> lat_threshold = {0.1, 2.0};
  std::vector<double> lon_dis = {0.2, 6.0};
  const double ego_heading_angle = plan_start_point.path_point().theta();
  const double object_heading_angle = traj_ptr.pose().theta();
  const double angle_diff =
      std::abs(NormalizeAngle(ego_heading_angle - object_heading_angle));

  const auto obj_box = traj_ptr.bounding_box();
  const auto ego_pos = Extract2dVectorFromApolloProto(plan_start_point);
  e2e_noa::Box2d ego_box(ego_pos, plan_start_point.path_point().theta(),
                         ego_length, ego_width);
  auto& objects_proto = traj_ptr.planner_object().object_proto();
  e2e_noa::Box2d obj_boxV2({objects_proto.pos().x(), objects_proto.pos().y()},
                           objects_proto.bounding_box().heading(),
                           objects_proto.bounding_box().length(),
                           objects_proto.bounding_box().width());
  const auto right_front_conner = passage.QueryUnboundedFrenetCoordinateAt(
      ego_box.GetCorner(Box2d::Corner::FRONT_RIGHT));

  const auto left_front_conner = passage.QueryUnboundedFrenetCoordinateAt(
      ego_box.GetCorner(Box2d::Corner::FRONT_LEFT));

  const auto right_rear_conner = passage.QueryUnboundedFrenetCoordinateAt(
      obj_box.GetCorner(Box2d::Corner::REAR_RIGHT));

  const auto left_rear_conner = passage.QueryUnboundedFrenetCoordinateAt(
      obj_box.GetCorner(Box2d::Corner::REAR_LEFT));

  const auto right_rear_connerV2 = passage.QueryUnboundedFrenetCoordinateAt(
      obj_boxV2.GetCorner(Box2d::Corner::REAR_RIGHT));

  const auto left_rear_connerv2 = passage.QueryUnboundedFrenetCoordinateAt(
      obj_boxV2.GetCorner(Box2d::Corner::REAR_LEFT));

  double left_front_l = DBL_MAX, right_front_l = DBL_MAX;
  double obj_left_rear_l = DBL_MAX, obj_right_rear_l = DBL_MAX;

  double obj_left_rear_l_V2 = DBL_MAX, obj_right_rear_l_V2 = DBL_MAX;

  if (right_front_conner.ok()) right_front_l = right_front_conner.value().l;
  if (left_front_conner.ok()) left_front_l = left_front_conner.value().l;

  if (right_rear_conner.ok()) obj_right_rear_l = right_rear_conner.value().l;
  if (left_rear_conner.ok()) obj_left_rear_l = left_rear_conner.value().l;

  if (right_rear_connerV2.ok())
    obj_right_rear_l_V2 = right_rear_connerV2.value().l;
  if (left_rear_connerv2.ok())
    obj_left_rear_l_V2 = left_rear_connerv2.value().l;

  if (obj_frenet_box.l_max < ego_frenet_box.l_max &&
      obj_frenet_box.l_max >= ego_frenet_box.l_min) {
    if (right_front_conner.ok() && left_rear_connerv2.ok()) {
      lat_overlap = obj_left_rear_l_V2 - right_front_l;
    }
  } else if (ego_frenet_box.l_max < obj_frenet_box.l_max &&
             ego_frenet_box.l_max >= obj_frenet_box.l_min) {
    if (ego_frenet_box.l_min >= obj_frenet_box.l_min) {
      if (right_front_conner.ok() && left_rear_connerv2.ok()) {
        lat_overlap = obj_left_rear_l_V2 - right_front_l;
      }
    } else {
      if (left_front_conner.ok() && right_rear_connerV2.ok()) {
        lat_overlap = left_front_l - obj_right_rear_l_V2;
      }
    }
  }

  if (lat_overlap == 0.0) return false;
  front_safe_dis =
      ad_e2e::planning::math::interp1_inc(lat_threshold, lon_dis, lat_overlap);
  const std::string nearest =
      "obj_id: " + obj_id + " lon_dist: " + std::to_string(lon_dist) +
      " lat_overlap: " + std::to_string(lat_overlap) +
      " front_safe_dis: " + std::to_string(front_safe_dis) +
      " is_nudge: " + std::to_string(is_nudge) +
      " angle_diff: " + std::to_string(angle_diff) +
      " ego_left_coner: " + std::to_string(left_front_l) +
      " ego_right_coner: " + std::to_string(right_front_l) +
      " obj_left_rear_coner: " + std::to_string(obj_left_rear_l) +
      " obj_left_rear_conerv2 " + std::to_string(obj_left_rear_l_V2) +
      " obj_right_rear_coner: " + std::to_string(obj_right_rear_l) +
      " obj_right_rear_conerv2 " + std::to_string(obj_right_rear_l_V2) +
      " ego_heading_angle: " + std::to_string(ego_heading_angle);

  if (!is_lane_change && !nudge_object_info && lon_dist < front_safe_dis &&
      0.0 < lon_dist)
    return true;
  return false;
}

bool IsObjectBlockingEgoVehicle(const PathSlBoundary& sl_boundary,
                                const FrenetBox& obj_frenet_box,
                                const FrenetBox& ego_frenet_box) {
  const double ego_l_offset =
      sl_boundary.QueryReferenceCenterL(ego_frenet_box.center_s());
  const double obj_l_offset =
      sl_boundary.QueryReferenceCenterL(obj_frenet_box.center_s());

  if (obj_frenet_box.l_max - obj_l_offset <
          ego_frenet_box.l_min - ego_l_offset ||
      obj_frenet_box.l_min - obj_l_offset >
          ego_frenet_box.l_max - ego_l_offset) {
    return false;
  }
  return true;
}

bool SelectLeadingVulnerableRoadUser(
    const double& obj_v, const FrenetBox& obj_frenet_box,
    const ApolloTrajectoryPointProto& plan_start_point,
    const FrenetBox& ego_frenet_box, const bool pre_unlead,
    std::string& vru_debug) {
  if (!pre_unlead || obj_v > plan_start_point.v()) return true;
  const double comfor_a = -1.0;
  double safe_s = (Sqr(obj_v) - Sqr(plan_start_point.v())) / (2 * comfor_a);
  vru_debug += " safe_s: " + std::to_string(safe_s);
  if (obj_frenet_box.s_min - ego_frenet_box.s_max > safe_s) return true;
  return false;
}

bool VruUnsafe(const FrenetBox& obj_frenet_box,
               const SpacetimeObjectTrajectory& traj_ptr, const bool pre_unlead,
               std::string& vru_debug,
               const ApolloTrajectoryPointProto& plan_start_point,
               const FrenetBox& ego_frenet_box, const std::string& obj_id) {
  const double obj_width = traj_ptr.bounding_box().width();
  const double slow_lead_th = pre_unlead ? -std::fmin(obj_width, 0.5) + 0.3
                                         : -std::fmin(obj_width, 0.5);
  const double normal_lead_th = pre_unlead ? 0.3 : 0.0;
  vru_debug +=
      " obj_id: " + obj_id + " pre_unleading" + std::to_string(pre_unlead);
  if (traj_ptr.pose().v() < (10.0 * ad_e2e::planning::Constants::KPH2MPS)) {
    if (obj_frenet_box.center_l() > slow_lead_th &&
        SelectLeadingVulnerableRoadUser(traj_ptr.pose().v(), obj_frenet_box,
                                        plan_start_point, ego_frenet_box,
                                        pre_unlead, vru_debug)) {
      vru_debug += "center_l";
      return true;
    }
  } else if (obj_frenet_box.l_max > normal_lead_th &&
             SelectLeadingVulnerableRoadUser(
                 traj_ptr.pose().v(), obj_frenet_box, plan_start_point,
                 ego_frenet_box, pre_unlead, vru_debug)) {
    vru_debug += "l_max";
    return true;
  }
  return false;
}

std::string find_nearest_stall(
    const SpacetimeTrajectoryManager& st_traj_mgr, const PlanPassage& passage,
    const absl::flat_hash_set<std::string>& stalled_objects,
    const FrenetBox& ego_frenet_box) {
  std::string nearest_obj_id = "";
  if (stalled_objects.empty()) return nearest_obj_id;
  double nearest_stall_s = DBL_MAX;
  for (auto& stall_obj : stalled_objects) {
    const auto temp_nearest_obj = st_traj_mgr.FindObjectByObjectId(stall_obj);
    if (temp_nearest_obj) {
      if (temp_nearest_obj->type() != ObjectType::OT_VEHICLE &&
          temp_nearest_obj->type() != ObjectType::OT_LARGE_VEHICLE)
        continue;
      const auto stall_obj_box =
          passage.QueryFrenetBoxAtContour(temp_nearest_obj->contour());
      if (!stall_obj_box.ok() ||
          stall_obj_box.value().l_max < ego_frenet_box.l_min - 2.0 ||
          stall_obj_box.value().l_min > ego_frenet_box.l_max + 2.0 ||
          stall_obj_box.value().center_s() < ego_frenet_box.center_s() - 20.0)
        continue;
      bool UnOverlap = (stall_obj_box.value().l_min > ego_frenet_box.l_max ||
                        stall_obj_box.value().l_max < ego_frenet_box.l_min);
      if (ego_frenet_box.s_min > stall_obj_box.value().s_max && !UnOverlap)
        continue;

      if (stall_obj_box.value().center_s() < nearest_stall_s) {
        nearest_stall_s = stall_obj_box.value().center_s();
        nearest_obj_id = stall_obj;
      }
    }
  }

  return nearest_obj_id;
}

std::pair<std::string, double> FindNearestStationaryCar(
    const ObjectsOnLane& st_trajs_on_lane, const FrenetBox& ego_frenet_box,
    bool is_vru, const ApolloTrajectoryPointProto& plan_start_point) {
  std::pair<std::string, double> nearest_obj;
  for (const auto& [obj_frenet_box, traj_ptr] : st_trajs_on_lane) {
    const std::string obj_id = traj_ptr->planner_object().id();
    if (traj_ptr->pose().v() > plan_start_point.v()) continue;
    const double comfor_a = -3.0;
    double safe_s = (Sqr(traj_ptr->pose().v()) - Sqr(plan_start_point.v())) /
                    (2 * comfor_a);
    std::string nearest_debug =
        " obj_id: " + obj_id + " safe_s: " + std::to_string(safe_s);
    bool is_comf = obj_frenet_box.s_min - ego_frenet_box.s_max > safe_s;
    nearest_debug += " is_comf: " + std::to_string(is_comf);
    bool JudgeType =
        ((traj_ptr->planner_object().type() == ObjectType::OT_VEHICLE ||
          traj_ptr->planner_object().type() == ObjectType::OT_LARGE_VEHICLE) &&
         is_comf);
    bool UnOverlap = (obj_frenet_box.l_min > ego_frenet_box.l_max ||
                      obj_frenet_box.l_max < ego_frenet_box.l_min);
    if (!JudgeType) continue;

    if (obj_frenet_box.center_l() <= 0.0) {
      if (obj_frenet_box.l_max >= ego_frenet_box.l_min - 0.6) {
        double lat_thr =
            (RoundToInt((obj_frenet_box.l_max + 0.6 - ego_frenet_box.l_min) /
                        0.1) +
             1) *
            0.1;
        nearest_obj = std::make_pair(obj_id, lat_thr);
        return nearest_obj;
      }
    } else {
      if (obj_frenet_box.l_min <= ego_frenet_box.l_max + 0.6) {
        double lat_thr =
            (RoundToInt((ego_frenet_box.l_max - obj_frenet_box.l_min + 0.6) /
                        0.1) +
             1) *
            0.1;
        nearest_obj = std::make_pair(obj_id, lat_thr);
        return nearest_obj;
      }
    }
  }
  return nearest_obj;
}

}  // namespace

std::vector<ConstraintProto::LeadingObjectProto> FindLeadingObjects(
    const ad_e2e::planning::TrafficLightStatusMap& tl_status_map,
    const PlannerSemanticMapManager& psmm, const PlanPassage& passage,
    const PathSlBoundary& sl_boundary, LaneChangeStage lc_stage,
    const SceneOutputProto& scene_reasoning,
    const SpacetimeTrajectoryManager& st_traj_mgr,
    const absl::flat_hash_set<std::string>& stalled_objects,
    const ApolloTrajectoryPointProto& plan_start_point,
    const e2e_noa::VehicleGeometryParamsProto& vehicle_geometry_params,
    const FrenetBox& ego_frenet_box, bool borrow_lane_boundary,
    const ObjectHistoryController* obs_his_manager,
    std::map<std::string, bool>& obj_lead,
    const NudgeObjectInfo* nudge_object_info, const bool& is_lane_change,
    const double& nearest_stop_s, bool* is_first_lead, bool& must_borrow,
    int plan_id) {
  std::vector<ConstraintProto::LeadingObjectProto> leading_objects;

  const auto traffic_waiting_objects = CollectTrafficWaitingObjectOnCurrentLane(
      scene_reasoning, passage.lane_path());

  absl::flat_hash_set<std::string> collect_stalled;

  const auto st_trajs_on_lane =
      FindFrontObjectsOnLane(passage, sl_boundary, st_traj_mgr.trajectories(),
                             ego_frenet_box, collect_stalled, stalled_objects);
  leading_objects.reserve(st_trajs_on_lane.size());

  const auto& lane_info_ptr =
      psmm.FindCurveLaneByIdOrNull(passage.lane_path().front().lane_id());

  std::string is_split_debug = "plan_id: " + std::to_string(plan_id);
  bool is_split = false;
  bool pre_is_split = false;
  double dist_to_split = DBL_MAX, dist_before_split = DBL_MAX;
  const std::size_t curr_ids = lane_info_ptr->lane_ind_in_section();
  const std::vector<std::string> next_lane_ids =
      lane_info_ptr->valid_next_lane_ids();
  if (next_lane_ids.size() > 1) {
    is_split = true;
  }
  if (is_split) {
    dist_to_split =
        (*passage.lane_path().begin()).end_s - ego_frenet_box.center_s();
  }

  if (curr_ids > 1 && lane_info_ptr->pre_lane_ids().size() == 1) {
    pre_is_split = true;
    dist_before_split =
        (*passage.lane_path().begin()).start_s - ego_frenet_box.center_s();
  }

  double split_judge_dist = is_split         ? dist_to_split
                            : (pre_is_split) ? dist_before_split
                                             : DBL_MAX;

  is_split_debug += " is_split: " + std::to_string(is_split) +
                    " curr_ids: " + std::to_string(curr_ids) +
                    " split_judge_dist: " + std::to_string(split_judge_dist) +
                    " dist_before_split: " + std::to_string(dist_before_split) +
                    " dist_to_split: " + std::to_string(dist_to_split);

  bool is_leftmost_lane = false, is_rightmost_lane = false;
  if (lane_info_ptr != nullptr) {
    is_leftmost_lane = lane_info_ptr->left_lane_id().empty();
    is_rightmost_lane = lane_info_ptr->right_lane_id().empty();
  }

  const double front_edge_to_center =
      vehicle_geometry_params.front_edge_to_center();
  const double ego_width = vehicle_geometry_params.width();
  const double ego_half_width = 0.5 * ego_width;
  const bool lc_ongoing = (lc_stage == LaneChangeStage::LCS_EXECUTING ||
                           lc_stage == LaneChangeStage::LCS_RETURN);
  double stdll_s = -20.0;
  std::string stall_obj_debug = "";

  std::string stall_id =
      find_nearest_stall(st_traj_mgr, passage, collect_stalled, ego_frenet_box);
  stall_obj_debug += " stall_id: " + stall_id;
  const auto stll_obj = st_traj_mgr.FindObjectByObjectId(stall_id);
  if (stll_obj) {
    const auto stall_obj_box =
        passage.QueryFrenetBoxAtContour(stll_obj->contour());
    const std::string stall_obj = stll_obj->id();
    stall_obj_debug += " stall_obj_id: " + stall_obj;

    stdll_s = stall_obj_box.value().center_s();
    if (stdll_s > 0.0 && stdll_s < 50.0 && is_rightmost_lane &&
        nudge_object_info && is_leftmost_lane) {
      must_borrow = true;
    }

    stall_obj_debug += " " + std::to_string(stdll_s);
  }

  for (const auto& [obj_frenet_box, traj_ptr] : st_trajs_on_lane) {
    const auto& obj_id = traj_ptr->planner_object().id();
    auto obs_histroy_info = obs_his_manager->GetObjLatestFrame(obj_id);
    bool is_stalled = obs_histroy_info && obs_histroy_info->is_stalled;
    double lead_threshold =
        obj_frenet_box.center_s() > split_judge_dist &&
                obj_frenet_box.center_s() < split_judge_dist + 20
            ? 0.3
            : 0.0;
    bool nudge_obj = obs_histroy_info && obs_histroy_info->is_nudge;
    if (obs_histroy_info && !obs_histroy_info->is_leading)
      lead_threshold = -0.3;

    if (IsOncomingObjectJudgeByEgoHeading(plan_start_point, traj_ptr->pose()) &&
        stalled_objects.find(obj_id) == stalled_objects.end()) {
      obj_lead[obj_id] = false;
      continue;
    }

    bool is_vru_type =
        (traj_ptr->planner_object().type() == ObjectType::OT_CYCLIST ||
         traj_ptr->planner_object().type() == ObjectType::OT_TRICYCLIST)
            ? true
            : false;

    bool JudgeType =
        ((traj_ptr->planner_object().type() == ObjectType::OT_VEHICLE ||
          traj_ptr->planner_object().type() == ObjectType::OT_LARGE_VEHICLE) &&
         traj_ptr->is_stationary());

    bool borrow_consider = false;
    bool is_brake_light = traj_ptr->planner_object()
                              .object_proto()
                              .obstacle_light()
                              .brake_lights() == ObstacleLightType::LIGHT_ON;
    if (nudge_object_info &&
        nudge_object_info->nudge_state == NudgeObjectInfo::NudgeState::BORROW &&
        nudge_object_info->id != obj_id &&
        (is_brake_light || obj_frenet_box.l_max * obj_frenet_box.l_min < 0.0)) {
      borrow_consider = true;
    }

    double lane_width = 0.0;
    if (lane_info_ptr) {
      lane_width = lane_info_ptr->GetWidthAtAccumS(obj_frenet_box.center_s());
    }

    double over_width = std::max(lane_width - 3.6, 0.0);

    const auto& nearest_obj = FindNearestStationaryCar(
        st_trajs_on_lane, ego_frenet_box, is_vru_type, plan_start_point);

    if (IsUnsafe(obj_frenet_box, ego_frenet_box, *traj_ptr, plan_start_point,
                 obj_id, nudge_obj, nudge_object_info, is_lane_change,
                 ego_width, vehicle_geometry_params.length(), passage) &&
        !is_vru_type) {
      leading_objects.push_back(CreateLeadingObject(
          *traj_ptr, passage,
          ConstraintProto::LeadingObjectProto::UNABLE_TO_OVERTAKE));
      obj_lead[obj_id] = true;
      std::string log_reason =
          "Leading reason: UNABLE_TO_OVERTAKE, obj_id: " + obj_id;
      Log2FG::LogDataV0("leading_reason", log_reason);
      continue;
    }

    if (traj_ptr->is_stationary() && stdll_s > -20.0 &&
        obj_frenet_box.center_s() > stdll_s &&
        obj_frenet_box.center_s() < stdll_s + 25.0 && obj_id != stall_id) {
      const auto stall_obj_box =
          passage.QueryFrenetBoxAtContour(stll_obj->contour());
      if (!(stall_obj_box.value().l_max < obj_frenet_box.l_min + 0.5 ||
            stall_obj_box.value().l_min > obj_frenet_box.l_max - 0.5)) {
        obj_lead[obj_id] = false;
        stall_obj_debug += " obj_id: " + obj_id + " obj_pose_s: " +
                           std::to_string(obj_frenet_box.center_s());
        Log2FG::LogDataV2("stall_obj_debug", stall_obj_debug);
        continue;
      }
    }
    stall_obj_debug += " obj_id: " + obj_id;
    Log2FG::LogDataV2("stall_obj_debug", stall_obj_debug);
    stall_obj_debug = "";

    if (IsCutInObjectTrajectory(passage, is_lane_change, plan_start_point.v(),
                                ego_frenet_box, *traj_ptr)) {
      continue;
    }

    bool l_space_enough = false, r_space_enough = false,
         is_emergency_obj = false;
    bool slboundary_check = IsObjectAvoidableWithinSlBoundary(
        sl_boundary, obj_frenet_box, ego_width, lead_threshold, obj_id,
        nudge_object_info, is_lane_change, nearest_obj, passage, ego_frenet_box,
        plan_start_point, st_traj_mgr, l_space_enough, r_space_enough,
        is_stalled, traj_ptr->pose().v(), is_emergency_obj);

    bool is_obj_env = IsObjectAvoidableWithinEnv(
        passage, obj_frenet_box, obj_id, st_traj_mgr.trajectories(),
        l_space_enough, r_space_enough, nudge_object_info, ego_frenet_box,
        traj_ptr->is_stationary());

    if (!is_vru_type && (!slboundary_check || !is_obj_env)) {
      leading_objects.push_back(CreateLeadingObject(
          *traj_ptr, passage,
          ConstraintProto::LeadingObjectProto::UNABLE_TO_OVERTAKE));
      obj_lead[obj_id] = true;
      std::string log_reason =
          "Leading reason: UNABLE_TO_OVERTAKE (SL boundary or environment), "
          "obj_id: " +
          obj_id + "sl_boundary_check:" + std::to_string(slboundary_check) +
          " is_obj_env: " + std::to_string(is_obj_env);
      Log2FG::LogDataV0("leading_reason", log_reason);
      continue;
    }

    if (stalled_objects.contains(obj_id)) {
      obj_lead[obj_id] = false;
      continue;
    }

    bool large_vehicle_check = IsLargeVehicleAvoidableWithinSlBoundary(
        psmm, passage, plan_start_point, sl_boundary, obj_frenet_box, *traj_ptr,
        ego_width);

    if (!is_vru_type && !large_vehicle_check) {
      leading_objects.push_back(CreateLeadingObject(
          *traj_ptr, passage,
          ConstraintProto::LeadingObjectProto::UNABLE_TO_OVERTAKE));
      obj_lead[obj_id] = true;
      std::string log_reason =
          "Leading reason: UNABLE_TO_OVERTAKE (Large vehicle), obj_id: " +
          obj_id;
      Log2FG::LogDataV0("leading_reason", log_reason);
      continue;
    }

    if (!is_vru_type &&
        !IsObjectAvoidableWithinCurbBuffer(passage, obj_frenet_box, sl_boundary,
                                           lead_threshold, obj_id, ego_width,
                                           l_space_enough, r_space_enough) &&
        !is_emergency_obj) {
      leading_objects.push_back(CreateLeadingObject(
          *traj_ptr, passage,
          ConstraintProto::LeadingObjectProto::UNABLE_TO_OVERTAKE));
      obj_lead[obj_id] = true;
      std::string log_reason =
          "Leading reason: UNABLE_TO_OVERTAKE (Curb buffer), obj_id: " + obj_id;
      Log2FG::LogDataV0("leading_reason", log_reason);
      continue;
    }

    if (traffic_waiting_objects.contains(obj_id) && !is_vru_type) {
      std::string traffic_debug =
          "obj_id: " + obj_id + " over_width: " + std::to_string(over_width);
      bool stand_side_ = false;
      if (obj_frenet_box.l_max < 0.0 ||
          obj_frenet_box.l_min - over_width > 0.0) {
        stand_side_ = true;
      }
      if (!stand_side_) {
        leading_objects.push_back(CreateLeadingObject(
            *traj_ptr, passage,
            ConstraintProto::LeadingObjectProto::TRAFFIC_WAITING));
        obj_lead[obj_id] = true;
        std::string log_reason =
            "Leading reason: TRAFFIC_WAITING, obj_id: " + obj_id;
        Log2FG::LogDataV0("leading_reason", log_reason);
        continue;
      }
    }

    bool borrow_debug =
        !lc_ongoing &&
        IsObjectBlockingEgoVehicle(sl_boundary, obj_frenet_box, ego_frenet_box);
    std::string borrow_debug_string =
        std::to_string(borrow_debug) + " " + obj_id;

    if ((lc_ongoing && IsObjectBlockingRefCenter(sl_boundary, obj_frenet_box,
                                                 ego_half_width)) ||
        (!lc_ongoing && IsObjectBlockingEgoVehicle(sl_boundary, obj_frenet_box,
                                                   ego_frenet_box))) {
      std::string vru_debug = " ";
      if (borrow_lane_boundary) {
        if ((OT_CYCLIST == traj_ptr->object_type() ||
             OT_TRICYCLIST == traj_ptr->object_type())) {
          const bool pre_unlead =
              obs_histroy_info && !obs_histroy_info->is_leading;
          std::set<std::string> l_set, r_set;
          double lat_thr = obj_frenet_box.l_max - ego_frenet_box.l_min + 0.6;
          IsEnvUnsafe(st_traj_mgr.trajectories(), passage, ego_frenet_box,
                      l_set, r_set, plan_start_point, lat_thr, obj_id);
          if (VruUnsafe(obj_frenet_box, *traj_ptr, pre_unlead, vru_debug,
                        plan_start_point, ego_frenet_box, obj_id) ||
              ((l_set.size() > 1 ||
                (l_set.size() == 1 && l_set.find(obj_id) == l_set.end())) &&
               !nudge_object_info)) {
            leading_objects.push_back(CreateLeadingObject(
                *traj_ptr, passage,
                ConstraintProto::LeadingObjectProto::UNABLE_TO_OVERTAKE));
            vru_debug +=
                " obj_id: " + obj_id + "-" + std::to_string(pre_unlead);
            obj_lead[obj_id] = true;
          } else {
            obj_lead[obj_id] = false;
          }
          continue;
        } else if (!traj_ptr->is_stationary() || !nudge_object_info ||
                   nudge_object_info->nudge_state ==
                       NudgeObjectInfo::NudgeState::NUDGE ||
                   borrow_consider) {
          leading_objects.push_back(CreateLeadingObject(
              *traj_ptr, passage,
              ConstraintProto::LeadingObjectProto::BORROW_BOUNDARY));
          obj_lead[obj_id] = true;
          std::string log_reason =
              "Leading reason: BORROW_BOUNDARY, obj_id: " + obj_id;
          Log2FG::LogDataV0("leading_reason", log_reason);
          continue;
        } else {
        }
      }

      bool veh_type_condition =
          traj_ptr->planner_object().type() == ObjectType::OT_VEHICLE ||
          traj_ptr->planner_object().type() == ObjectType::OT_LARGE_VEHICLE;
      if (veh_type_condition &&
          IsObjectWithinTlControlledIntersection(
              tl_status_map, psmm, obj_frenet_box, passage.lane_path(),
              passage.lane_path_start_s())) {
        leading_objects.push_back(CreateLeadingObject(
            *traj_ptr, passage,
            ConstraintProto::LeadingObjectProto::INTERSECTION));
        obj_lead[obj_id] = true;
        std::string log_reason =
            "Leading reason: INTERSECTION, obj_id: " + obj_id;
        Log2FG::LogDataV0("leading_reason", log_reason);
        continue;
      }
    }

    obj_lead[obj_id] = false;
  }

  if (nullptr != is_first_lead && !st_trajs_on_lane.empty()) {
    const auto& first_obj_id =
        st_trajs_on_lane.front().second->planner_object().id();
    *is_first_lead =
        (obj_lead.end() != obj_lead.find(first_obj_id) &&
         (obj_lead[first_obj_id] || st_trajs_on_lane.front().first.s_max +
                                            vehicle_geometry_params.length() >
                                        nearest_stop_s));
    if (*is_first_lead && !obj_lead[first_obj_id] &&
        st_trajs_on_lane.front().first.l_max > ego_frenet_box.l_min &&
        st_trajs_on_lane.front().first.l_min < ego_frenet_box.l_max) {
      leading_objects.push_back(CreateLeadingObject(
          *st_trajs_on_lane.front().second, passage,
          ConstraintProto::LeadingObjectProto::AFTER_STOPLINE));
      obj_lead[first_obj_id] = true;
    }
  }

  return leading_objects;
}

}  // namespace planning
}  // namespace e2e_noa
