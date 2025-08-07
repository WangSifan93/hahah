#include "descriptor/crosswalk_descriptor.h"

#include <algorithm>
#include <cmath>
#include <ostream>
#include <string>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_set.h"
#include "absl/strings/str_cat.h"
#include "constraint.pb.h"
#include "container/strong_int.h"
#include "crosswalk_state.pb.h"
#include "descriptor/descriptor_util.h"
#include "glog/logging.h"
#include "maps/lane_path.h"
#include "maps/map_def.h"
#include "maps/map_or_die_macros.h"
#include "maps/semantic_map_defs.h"
#include "math/frenet_common.h"
#include "math/geometry/halfplane.h"
#include "math/geometry/polygon2d.h"
#include "math/geometry/segment2d.h"
#include "math/piecewise_linear_function.h"
#include "math/util.h"
#include "math/vec.h"
#include "object/object_vector.h"
#include "object/planner_object.h"
#include "perception.pb.h"
#include "plan/trajectorypoint_with_acceleration.h"
#include "prediction/predicted_trajectory.h"
#include "prediction/prediction.h"
#include "util/status_macros.h"

namespace e2e_noa {
namespace planning {
namespace {

struct CrosswalkManager {
  ad_e2e::planning::Crosswalk crosswalk;
  double start_s_on_plan_passage;
  double end_s_on_plan_passage;
  bool tl_controlled;
  const CrosswalkStateProto* last_state = nullptr;
};

constexpr double kEpsilon = 1e-5;

constexpr double kStableObjectPredictedTrajectoryTime = 2.0;

constexpr double kCrosswalkAssociationDistance = 1.3;

constexpr double kIgnoreLateralMoveOffBuffer = 1.0;

constexpr double kCrossWalkStandoff = 2.0;

constexpr double kMaxLookAheadDistance = 150;

constexpr double kComfortableAcceleration = 0.5;

constexpr double kStopMinDistance = 3.0;
constexpr double kStopMaxVelocity = 0.3;

constexpr double kMaxYieldingTime = 8.0;

constexpr double kEmergencyStopDeceleration = -3.5;

const PiecewiseLinearFunction kVehicleSpeedComfortableBrakePlf(
    std::vector<double>{0.0, 5.0, 10.0}, std::vector<double>{-3.5, -2.5, -1.5});

const PiecewiseLinearFunction kObjectMovingTimeGainPlf(
    std::vector<double>{0.1, 0.5}, std::vector<double>{0.0, 4.0});

const CrosswalkStateProto* FindCrosswalkStateOrNull(
    mapping::ElementId crosswalk_id,
    const ::google::protobuf::RepeatedPtrField<CrosswalkStateProto>&
        crosswalk_states) {
  for (const auto& state : crosswalk_states) {
    if (state.crosswalk_id() == crosswalk_id) return &state;
  }
  return nullptr;
}

std::vector<CrosswalkManager> FindCrosswalks(
    const PlannerSemanticMapManager& psmm, const PlanPassage& passage,
    const mapping::LanePath& lane_path_from_start,
    const Vec2d& plan_start_pos_xy, const bool has_traffic_light,
    const ::google::protobuf::RepeatedPtrField<CrosswalkStateProto>&
        crosswalk_states) {
  std::vector<CrosswalkManager> crosswalk_managers;

  if (has_traffic_light) {
    return crosswalk_managers;
  }

  for (const auto& seg : lane_path_from_start) {
    SMM_ASSIGN_LANE_OR_CONTINUE(lane_info, psmm, seg.lane_id);
    if (!lane_info.junction_id().empty()) {
      return crosswalk_managers;
    }
    for (const auto& cw_id : lane_info.crosswalks()) {
      const auto cw_ptr = psmm.FindCrosswalkByIdOrNull(cw_id);
      if (cw_ptr == nullptr || !cw_ptr->IsValid()) {
        continue;
      }
      const auto& cw_info = *cw_ptr;
      const auto frenet_box =
          passage.QueryFrenetBoxAtContour(cw_info.polygon());
      if (!frenet_box.ok()) {
        continue;
      }
      const auto plan_start_sl =
          passage.QueryFrenetCoordinateAt(plan_start_pos_xy);
      if (!plan_start_sl.ok() || plan_start_sl->s > frenet_box->s_max ||
          plan_start_sl->s + kMaxLookAheadDistance < frenet_box->s_min) {
        continue;
      }

      crosswalk_managers.emplace_back(CrosswalkManager{
          .crosswalk = cw_info,
          .start_s_on_plan_passage = frenet_box->s_min,
          .end_s_on_plan_passage = frenet_box->s_max,
          .tl_controlled = false,
          .last_state = FindCrosswalkStateOrNull(cw_id, crosswalk_states)});
    }
  }

  return crosswalk_managers;
}

bool IsComfortableDecelerationForCrosswalk(double speed, double av_front_edge_s,
                                           double crosswalk_start_s) {
  if (crosswalk_start_s < av_front_edge_s) return false;

  const double comfortable_deceleration_dist =
      0.5 * speed * speed / std::fabs(kVehicleSpeedComfortableBrakePlf(speed));

  return crosswalk_start_s > av_front_edge_s + comfortable_deceleration_dist;
}

double UncontrolledCrosswalkTimeBenefit(ObjectType type) {
  switch (type) {
    case OT_PEDESTRIAN:
      return 4.0;
    case OT_CYCLIST:
    case OT_TRICYCLIST:
      return 0.6;
    case OT_UNKNOWN_STATIC:
    case OT_VEHICLE:
    case OT_LARGE_VEHICLE:
    case OT_MOTORCYCLIST:
    case OT_FOD:
    case OT_UNKNOWN_MOVABLE:
    case OT_VEGETATION:
    case OT_BARRIER:
    case OT_CONE:
    case OT_WARNING_TRIANGLE:
      return 0.0;
    default:
      throw std::runtime_error("switch case on enum unexpected");
  }
}
bool IsConsideredType(ObjectType type) {
  switch (type) {
    case OT_PEDESTRIAN:
    case OT_CYCLIST:
    case OT_TRICYCLIST:
      return true;
    case OT_UNKNOWN_STATIC:
    case OT_VEHICLE:
    case OT_LARGE_VEHICLE:
    case OT_MOTORCYCLIST:
    case OT_FOD:
    case OT_UNKNOWN_MOVABLE:
    case OT_VEGETATION:
    case OT_BARRIER:
    case OT_CONE:
    case OT_WARNING_TRIANGLE:
      return false;
    default:
      throw std::runtime_error("switch case on enum unexpected");
  }
}

double GetIgnoreLateralTowardBuffer(ObjectType type) {
  switch (type) {
    case OT_PEDESTRIAN:
      return 5.0;
    case OT_CYCLIST:
    case OT_TRICYCLIST:
      return 2.0;
    case OT_UNKNOWN_STATIC:
    case OT_VEHICLE:
    case OT_LARGE_VEHICLE:
    case OT_MOTORCYCLIST:
    case OT_FOD:
    case OT_UNKNOWN_MOVABLE:
    case OT_VEGETATION:
    case OT_BARRIER:
    case OT_CONE:
    case OT_WARNING_TRIANGLE:
      return 0.0;
    default:
      throw std::runtime_error("switch case on enum unexpected");
  }
}

bool IsObjectTrajectoryOnCrosswalk(const CrosswalkManager& cw_manager,
                                   const PlannerObject& object) {
  const auto& cw_info = cw_manager.crosswalk;
  const auto& trajs = object.prediction().trajectories();
  for (const auto& pred_traj : trajs) {
    const auto& pred_traj_points = pred_traj.points();
    for (const auto& pt : pred_traj_points) {
      if (pt.t() > kStableObjectPredictedTrajectoryTime) break;

      if (cw_info.polygon().DistanceSquareTo(pt.pos()) < Sqr(kEpsilon))
        return true;
    }
  }
  return false;
}

bool IsObjectNearCrosswalk(const CrosswalkManager& cw_manager,
                           const PlannerObject& object, double distance) {
  const auto& cw_info = cw_manager.crosswalk;

  return cw_info.polygon().DistanceSquareTo(object.pose().pos()) <
         Sqr(distance);
}

absl::StatusOr<ConstraintProto::StopLineProto> GenerateCrosswalkConstraints(
    const CrosswalkManager& cw_manager, const PlanPassage& passage) {
  ASSIGN_OR_RETURN(const auto curbs, passage.QueryCurbPointAtS(
                                         cw_manager.start_s_on_plan_passage));

  ConstraintProto::StopLineProto stop_line;
  stop_line.set_s(cw_manager.start_s_on_plan_passage);
  stop_line.set_standoff(kCrossWalkStandoff);
  stop_line.set_time(0.0);
  HalfPlane halfplane(curbs.first, curbs.second);
  halfplane.ToProto(stop_line.mutable_half_plane());
  stop_line.set_id(absl::StrCat("crosswalk_", cw_manager.crosswalk.id()));
  stop_line.mutable_source()->mutable_crosswalk()->set_crosswalk_id(
      cw_manager.crosswalk.id());
  return stop_line;
}

double CalculateAccelerationTime(double dist, double vel_begin, double acc) {
  return (std::sqrt(2 * acc * dist + Sqr(vel_begin)) - vel_begin) / acc;
}

CrosswalkStateProto CreateApproachingState(mapping ::ElementId crosswalk_id) {
  CrosswalkStateProto new_state;
  new_state.set_crosswalk_id(crosswalk_id);

  new_state.set_stop_state(CrosswalkStateProto::APPROACHING);
  return new_state;
}

CrosswalkStateProto CreatePassedState(mapping ::ElementId crosswalk_id) {
  CrosswalkStateProto new_state;

  new_state.set_crosswalk_id(crosswalk_id);

  new_state.set_stop_state(CrosswalkStateProto::PASSED);
  return new_state;
}

CrosswalkStateProto CreateFirstStoppedState(mapping ::ElementId crosswalk_id,
                                            double now_in_seconds) {
  CrosswalkStateProto new_state;

  new_state.set_crosswalk_id(crosswalk_id);

  new_state.set_stop_time(now_in_seconds);
  new_state.set_stop_state(CrosswalkStateProto::STOPPED);
  return new_state;
}

CrosswalkStateProto UpdateCrosswalkState(double av_front_edge_s,
                                         double plan_start_v,
                                         double now_in_seconds,
                                         const CrosswalkManager& cw_manager) {
  const auto& crosswalk_id = cw_manager.crosswalk.id();
  if (cw_manager.last_state == nullptr) {
    return CreateApproachingState(crosswalk_id);
  }

  const auto& last_state = *cw_manager.last_state;

  if (last_state.stop_state() == CrosswalkStateProto::PASSED)
    return CreatePassedState(crosswalk_id);

  if (last_state.stop_state() == CrosswalkStateProto::APPROACHING) {
    if (av_front_edge_s >
            cw_manager.start_s_on_plan_passage - kStopMinDistance &&
        plan_start_v < kStopMaxVelocity) {
      return CreateFirstStoppedState(crosswalk_id, now_in_seconds);
    } else if (av_front_edge_s > cw_manager.end_s_on_plan_passage) {
      return CreatePassedState(crosswalk_id);
    } else {
      return CreateApproachingState(crosswalk_id);
    }
  }

  const bool exceed_yield_time =
      now_in_seconds - last_state.stop_time() > kMaxYieldingTime;
  if (exceed_yield_time || av_front_edge_s > cw_manager.end_s_on_plan_passage) {
    return CreatePassedState(crosswalk_id);
  }

  CrosswalkStateProto new_state;
  new_state.set_crosswalk_id(crosswalk_id);

  new_state.set_stop_time(last_state.stop_time());
  new_state.set_stop_state(CrosswalkStateProto::STOPPED);
  return new_state;
}

double CalculateObjectAcrossRatio(const PlannerObject& object,
                                  const CrosswalkManager& cw_manager) {
  const auto& cw_segment = cw_manager.crosswalk.bone_axis_smooth();
  const auto& cw_unit_dir = cw_segment.unit_direction();

  const double object_signed_offset =
      (object.pose().pos() - cw_segment.start()).dot(cw_unit_dir);
  double across_ratio =
      std::clamp(object_signed_offset / cw_segment.length(), 0.0, 1.0);

  const Vec2d& object_vel = object.velocity();

  if (object_vel.dot(cw_unit_dir) < kEpsilon) {
    across_ratio = 1.0 - across_ratio;
  }
  return across_ratio;
}

}  // namespace

absl::StatusOr<CrosswalkDescriptor> BuildCrosswalkConstraints(
    const CrosswalkDescriptorInput& input) {
  const auto& vehicle_geometry_params = *input.vehicle_geometry_params;
  const auto& psmm = *input.psmm;
  const auto& plan_start_point = *input.plan_start_point;
  const auto& passage = *input.passage;
  const auto& lane_path_from_start = *input.lane_path_from_start;
  const auto& obj_mgr = *input.obj_mgr;
  const auto& last_crosswalk_states = *input.last_crosswalk_states;
  const double now_in_seconds = input.now_in_seconds;
  const double s_offset = input.s_offset;
  const bool has_traffic_light = input.has_traffic_light;

  std::vector<ConstraintProto::StopLineProto> cw_stop_lines;
  std::vector<ConstraintProto::SpeedRegionProto> cw_speed_regions;
  std::vector<CrosswalkStateProto> cw_states;

  const Vec2d plan_start_pos_xy(plan_start_point.path_point().x(),
                                plan_start_point.path_point().y());

  const auto crosswalk_managers =
      FindCrosswalks(psmm, passage, lane_path_from_start, plan_start_pos_xy,
                     has_traffic_light, last_crosswalk_states);

  const auto plan_start_pos_sl =
      passage.QueryFrenetCoordinateAt(plan_start_pos_xy);
  if (!plan_start_pos_sl.ok()) {
    return plan_start_pos_sl.status();
  }

  const auto av_front_edge_s =
      vehicle_geometry_params.front_edge_to_center() + plan_start_pos_sl->s;

  const double plan_start_v = std::max(plan_start_point.v(), kEpsilon);

  const auto av_front_edge_xy =
      passage.QueryPointXYAtSL(av_front_edge_s, plan_start_pos_sl->l);
  if (!av_front_edge_xy.ok()) return av_front_edge_xy.status();

  for (const auto& cw_manager : crosswalk_managers) {
    auto cw_state = UpdateCrosswalkState(av_front_edge_s, plan_start_v,
                                         now_in_seconds, cw_manager);

    absl::flat_hash_set<std::string> last_must_yield_objects;
    if (cw_manager.last_state != nullptr) {
      const auto& last_state = *cw_manager.last_state;
      const int num_objects = last_state.must_yield_objects_size();
      for (int i = 0; i < num_objects; ++i) {
        last_must_yield_objects.insert(last_state.must_yield_objects(i));
      }
    }

    const auto cw_start_xy = passage.QueryPointXYAtSL(
        cw_manager.start_s_on_plan_passage, plan_start_pos_sl->l);
    const auto cw_end_xy = passage.QueryPointXYAtSL(
        cw_manager.end_s_on_plan_passage, plan_start_pos_sl->l);
    if (!cw_start_xy.ok() || !cw_end_xy.ok()) continue;

    VLOG(3) << " * * * * *cw:\t" << cw_manager.crosswalk.id() << " |start:\t"
            << cw_manager.start_s_on_plan_passage << " |end:\t"
            << cw_manager.end_s_on_plan_passage << " |tlc:\t"
            << cw_manager.tl_controlled;

    std::vector<std::string> new_must_yield_objects;

    std::vector<std::string> all_yield_objects;

    for (const auto& object : obj_mgr.planner_objects()) {
      bool must_yield_to_object = false;

      if (!IsConsideredType(object.type())) continue;

      const bool object_near_cw = IsObjectNearCrosswalk(
          cw_manager, object, kCrosswalkAssociationDistance);

      if (!object_near_cw &&
          !IsObjectTrajectoryOnCrosswalk(cw_manager, object)) {
        continue;
      }

      const Vec2d cw_axis_unit_dir =
          cw_manager.crosswalk.bone_axis_smooth().unit_direction();

      const Vec2d object_velocity = object.velocity();
      const double object_signed_velocity_proj_on_cw =
          object_velocity.dot(cw_axis_unit_dir);

      const bool av_already_on_crosswalk =
          cw_manager.start_s_on_plan_passage < av_front_edge_s;

      const Vec2d origin =
          av_already_on_crosswalk ? *av_front_edge_xy : *cw_start_xy;
      const double object_signed_offset_to_path =
          (object.pose().pos() - origin).dot(cw_axis_unit_dir);

      const bool same_side =
          object_signed_offset_to_path * object_signed_velocity_proj_on_cw >
          0.0;

      const double av_half_width =
          (vehicle_geometry_params.left_edge_to_center() +
           vehicle_geometry_params.right_edge_to_center()) *
          0.5;

      if (same_side && std::abs(object_signed_offset_to_path) >
                           kIgnoreLateralMoveOffBuffer + av_half_width) {
        continue;
      }

      if (std::abs(object_velocity.CrossProd(cw_axis_unit_dir)) >
          std::abs(object_signed_velocity_proj_on_cw)) {
        continue;
      }

      const Vec2d cw_axis_perp_unit_dir = (*cw_end_xy - *cw_start_xy).Unit();
      const double object_offset_to_cw =
          (object.pose().pos() - *cw_start_xy).dot(cw_axis_perp_unit_dir);
      const double av_offset_to_cw =
          (*av_front_edge_xy - *cw_start_xy).dot(cw_axis_perp_unit_dir);
      const double object_lead_av_dist = object_offset_to_cw - av_offset_to_cw;

      if (av_already_on_crosswalk && object_lead_av_dist < kEpsilon) {
        continue;
      }

      const bool cw_type_valid_yield_object = true;

      const bool comfortable_deceleration =
          IsComfortableDecelerationForCrosswalk(
              plan_start_v, av_front_edge_s,
              cw_manager.start_s_on_plan_passage);

      if (object_near_cw && cw_type_valid_yield_object &&
          comfortable_deceleration) {
        switch (cw_state.stop_state()) {
          case CrosswalkStateProto::APPROACHING:
          case CrosswalkStateProto::STOPPED:

            new_must_yield_objects.push_back(object.id());
            must_yield_to_object = true;
            break;
          case CrosswalkStateProto::PASSED:

            if (!object.is_stationary() &&
                last_must_yield_objects.contains(object.id())) {
              new_must_yield_objects.push_back(object.id());
              must_yield_to_object = true;
            }
            break;
        }
      }

      const double distance_to_crosswalk =
          std::max(object_lead_av_dist,
                   cw_manager.start_s_on_plan_passage - av_front_edge_s);

      double av_t_at_cw = distance_to_crosswalk / plan_start_v;
      double av_t_at_cw_acceleration = CalculateAccelerationTime(
          distance_to_crosswalk, plan_start_v, kComfortableAcceleration);

      double object_time_gain = UncontrolledCrosswalkTimeBenefit(object.type());
      if (object.type() == OT_PEDESTRIAN) {
        object_time_gain += kObjectMovingTimeGainPlf(
            CalculateObjectAcrossRatio(object, cw_manager));
      }

      const double object_ahead_of_av_dist_with_time_gain =
          std::abs(object_signed_velocity_proj_on_cw) *
              (av_t_at_cw + object_time_gain) +
          av_half_width - std::abs(object_signed_offset_to_path);
      const bool object_ahead_of_av_with_time_gain =
          object_ahead_of_av_dist_with_time_gain > kEpsilon;

      const double object_near_av_dist_without_time_gain =
          std::abs(object_signed_velocity_proj_on_cw) *
              av_t_at_cw_acceleration +
          GetIgnoreLateralTowardBuffer(object.type()) + av_half_width -
          std::abs(object_signed_offset_to_path);
      const bool object_near_av_without_time_gain =
          object_near_av_dist_without_time_gain > kEpsilon;

      const double deceleration =
          -0.5 * Sqr(plan_start_v) / distance_to_crosswalk;
      const bool av_stop_comfortablely =
          deceleration > kEmergencyStopDeceleration;

      if ((object_ahead_of_av_with_time_gain &&
           object_near_av_without_time_gain && av_stop_comfortablely) ||
          must_yield_to_object) {
        all_yield_objects.push_back(object.id());
      }
    }

    for (auto& id : new_must_yield_objects) {
      cw_state.add_must_yield_objects(std::move(id));
    }
    cw_states.push_back(std::move(cw_state));

    if (!all_yield_objects.empty()) {
      ASSIGN_OR_CONTINUE(auto stop_line,
                         GenerateCrosswalkConstraints(cw_manager, passage));

      for (auto& id : all_yield_objects) {
        stop_line.mutable_source()->mutable_crosswalk()->add_object_id(
            std::move(id));
      }

      cw_stop_lines.push_back(std::move(stop_line));
    }
  }

  return CrosswalkDescriptor{.stop_lines = std::move(cw_stop_lines),
                             .speed_regions = std::move(cw_speed_regions),
                             .crosswalk_states = std::move(cw_states)};
}
}  // namespace planning
}  // namespace e2e_noa
