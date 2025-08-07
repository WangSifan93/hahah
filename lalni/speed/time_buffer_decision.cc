#include "speed/time_buffer_decision.h"

#include <algorithm>
#include <cmath>
#include <optional>
#include <ostream>
#include <string>
#include <vector>

#include "math/piecewise_linear_function.h"
#include "object/planner_object.h"
#include "object/spacetime_object_trajectory.h"
#include "plan/trajectorypoint_with_acceleration.h"
#include "speed/st_boundary.h"
#include "speed/st_point.h"
#include "speed_planning.pb.h"

namespace e2e_noa {
namespace planning {

namespace {

double ComputePassTime(const StBoundary& st_boundary, double init_v,
                       const VehicleGeometryParamsProto& vehicle_params,
                       const SpacetimeTrajectoryManager& st_traj_mgr) {
  constexpr double kPassTimeForPedBase = 1.5;
  constexpr double kPassTimeForCyclistLowPriority = 0.9;
  constexpr double kPassTimeForCyclistHighPriority = 0.4;
  constexpr double kPassTimeForVehicleLowPriority = 0.7;
  constexpr double kPassTimeForVehicleHighPriority = 0.2;
  constexpr double kExtraPassTimeForFrontMerging = 1.0;

  constexpr double kMaxPassDisForCyclistLowPriority = 7.0;
  constexpr double kMaxPassDisForCyclistHighPriority = 4.0;
  constexpr double kMaxPassDisForVehicleLowPriority = 5.0;
  constexpr double kMaxPassDisForVehicleHighPriority = 2.0;
  constexpr double kMaxPassDisForAggresiveMerging = 1.0;
  constexpr double kEps = 1e-3;

  const auto& overlap_meta = *st_boundary.overlap_meta();
  const auto get_pass_time_by_overlap_priority =
      [](StOverlapMetaProto::OverlapPriority priority,
         double pass_time_for_high_priority, double pass_time_for_low_priority,
         double max_dis_for_high_priority, double max_dis_for_low_priority,
         double init_v) {
        if (priority == StOverlapMetaProto::UNKNOWN_PRIORITY) {
          return 0.0;
        } else if (priority == StOverlapMetaProto::LOW ||
                   priority == StOverlapMetaProto::EQUAL) {
          double max_time =
              max_dis_for_low_priority / (std::abs(init_v) + kEps);
          return std::min(pass_time_for_low_priority, max_time);
        } else {
          double max_time =
              max_dis_for_high_priority / (std::abs(init_v) + kEps);
          return std::min(pass_time_for_high_priority, max_time);
        }
      };

  double res = 0.0;
  const PiecewiseLinearFunction<double> time_buffer_min_t_s_factor(
      {0.0, 5.0, 7.0}, {0.5, 0.5, 0.0});
  const double first_overlap_average_s =
      0.5 * (st_boundary.bottom_left_point().s() +
             st_boundary.upper_left_point().s());
  const double buffer_min_t = std::min(
      st_boundary.min_t(), time_buffer_min_t_s_factor(first_overlap_average_s));
  if (st_boundary.object_type() == StBoundaryProto::PEDESTRIAN) {
    const PiecewiseLinearFunction<double> ped_pass_time_s_factor(
        {0.0, 10.0, 15.0, 30.0, 60.0, 90.0}, {0.5, 0.75, 1.0, 1.1, 1.2, 1.3});
    const PiecewiseLinearFunction<double> ped_pass_time_min_t_factor(
        {0.0, 1.0, 3.0, 5.0, 10.0}, {1.0, 1.1, 1.2, 1.3, 1.5});
    res = kPassTimeForPedBase *
          ped_pass_time_s_factor(first_overlap_average_s) *
          ped_pass_time_min_t_factor(st_boundary.min_t());
  } else if (st_boundary.object_type() == StBoundaryProto::CYCLIST) {
    res = get_pass_time_by_overlap_priority(
        overlap_meta.priority(), kPassTimeForCyclistHighPriority,
        kPassTimeForCyclistLowPriority, kMaxPassDisForCyclistHighPriority,
        kMaxPassDisForCyclistLowPriority, init_v);
  } else {
    res = get_pass_time_by_overlap_priority(
        overlap_meta.priority(), kPassTimeForVehicleHighPriority,
        kPassTimeForVehicleLowPriority, kMaxPassDisForVehicleHighPriority,
        kMaxPassDisForVehicleLowPriority, init_v);
    if (overlap_meta.source() == StOverlapMetaProto::LANE_MERGE &&
        st_boundary.overlap_meta()->has_front_most_projection_distance()) {
      const bool is_in_front_of_av =
          st_boundary.overlap_meta()->front_most_projection_distance() >
          vehicle_params.front_edge_to_center();
      const bool is_behind_av =
          st_boundary.overlap_meta()->front_most_projection_distance() <
          vehicle_params.front_edge_to_center() - vehicle_params.length() / 2.0;
      CHECK(st_boundary.traj_id().has_value());
      const auto& traj_id = st_boundary.traj_id();
      const auto* traj =
          CHECK_NOTNULL(st_traj_mgr.FindTrajectoryById(*traj_id));
      if (is_in_front_of_av) {
        const PiecewiseLinearFunction<double> obj_acc_extra_time_factor(
            {-1.0, -0.3, 0.0, 0.3, 0.5}, {0.0, 0.1, 0.3, 0.6, 1.0});
        res += kExtraPassTimeForFrontMerging *
               obj_acc_extra_time_factor(traj->planner_object().pose().a());
      } else if (is_behind_av && st_boundary.overlap_meta()->has_theta_diff()) {
        constexpr double kParallelHeadingThreshold = M_PI / 18.0;
        constexpr double kMaxRelVelocityThreshold = 1.0;
        const double rel_vel = traj->planner_object().pose().v() - init_v;
        if (rel_vel < kMaxRelVelocityThreshold &&
            std::abs(st_boundary.overlap_meta()->theta_diff()) <
                kParallelHeadingThreshold) {
          const double max_time =
              kMaxPassDisForAggresiveMerging / (std::abs(init_v) + kEps);
          res = std::min(res, max_time);
        }
      }
    }
  }

  return std::min(st_boundary.min_t() - buffer_min_t, res);
}

double ComputeYieldTime(const StBoundary& st_boundary,
                        const SpacetimeTrajectoryManager& st_traj_mgr) {
  constexpr double kYieldTimeForPedBase = 1.0;
  constexpr double kYieldTimeForCyclistLowPriority = 1.2;
  constexpr double kYieldTimeForCyclistHighPriority = 0.7;
  constexpr double kYieldTimeForVehicleLowPriority = 1.0;
  constexpr double kYieldTimeForVehicleHighPriority = 0.5;
  constexpr double kExtraYieldTimeForLeaveAndCross = 1.0;

  const auto& overlap_meta = *st_boundary.overlap_meta();
  const auto get_yield_time_by_overlap_priority_and_pattern =
      [](StOverlapMetaProto::OverlapPattern pattern,
         StOverlapMetaProto::OverlapPriority priority,
         double yield_time_for_high_priority,
         double yield_time_for_low_priority) {
        if (priority == StOverlapMetaProto::UNKNOWN_PRIORITY) {
          if (pattern == StOverlapMetaProto::LEAVE ||
              pattern == StOverlapMetaProto::ENTER ||
              pattern == StOverlapMetaProto::STAY) {
            return yield_time_for_low_priority;
          } else {
            return 0.0;
          }
        } else if (priority == StOverlapMetaProto::LOW ||
                   priority == StOverlapMetaProto::EQUAL) {
          return yield_time_for_low_priority;
        } else {
          return yield_time_for_high_priority;
        }
      };

  double res = 0.0;
  if (st_boundary.object_type() == StBoundaryProto::PEDESTRIAN) {
    const double last_overlap_average_s =
        0.5 * (st_boundary.bottom_right_point().s() +
               st_boundary.upper_right_point().s());
    const PiecewiseLinearFunction<double> ped_yield_time_s_factor(
        {0.0, 30.0, 60.0, 90.0}, {1.0, 1.1, 1.2, 1.3});
    const PiecewiseLinearFunction<double> ped_yield_time_min_t_factor(
        {0.0, 1.0, 3.0, 5.0, 10.0}, {1.0, 1.1, 1.2, 1.3, 1.5});
    res = kYieldTimeForPedBase *
          ped_yield_time_s_factor(last_overlap_average_s) *
          ped_yield_time_min_t_factor(st_boundary.max_t());
  } else if (st_boundary.object_type() == StBoundaryProto::CYCLIST) {
    res = get_yield_time_by_overlap_priority_and_pattern(
        overlap_meta.pattern(), overlap_meta.priority(),
        kYieldTimeForCyclistHighPriority, kYieldTimeForCyclistLowPriority);
  } else {
    res = get_yield_time_by_overlap_priority_and_pattern(
        overlap_meta.pattern(), overlap_meta.priority(),
        kYieldTimeForVehicleHighPriority, kYieldTimeForVehicleLowPriority);

    const double leave_speed = st_boundary.speed_points().back().v();
    const PiecewiseLinearFunction<double> vehicle_yield_time_speed_factor(
        {0.0, 10.0, 20.0, 30.0}, {1.0, 0.4, 0.2, 0.1});
    res *= vehicle_yield_time_speed_factor(leave_speed);
  }
  if (st_boundary.object_type() != StBoundaryProto::PEDESTRIAN &&
      (overlap_meta.pattern() == StOverlapMetaProto::LEAVE ||
       overlap_meta.pattern() == StOverlapMetaProto::CROSS)) {
    CHECK(st_boundary.traj_id().has_value());
    const auto& traj_id = st_boundary.traj_id();
    const auto* traj = CHECK_NOTNULL(st_traj_mgr.FindTrajectoryById(*traj_id));
    const PiecewiseLinearFunction<double> obj_vel_extra_time_factor(
        {0.0, 3.0, 5.0}, {1.0, 1.0, 0.0});
    res += kExtraYieldTimeForLeaveAndCross *
           obj_vel_extra_time_factor(traj->planner_object().pose().v());
  }
  return res;
}

}  // namespace

void DecideTimeBuffersForStBoundary(
    StBoundaryWithDecision* st_boundary_wd, double init_v,
    const VehicleGeometryParamsProto& vehicle_params,
    const SpacetimeTrajectoryManager& st_traj_mgr,
    bool disable_pass_time_buffer) {
  CHECK_NOTNULL(st_boundary_wd);

  const auto& st_boundary = *st_boundary_wd->raw_st_boundary();

  if (!st_boundary.overlap_meta().has_value()) return;
  const auto& overlap_meta = *st_boundary.overlap_meta();

  if (st_boundary_wd->decision_type() == StBoundaryProto::IGNORE ||
      st_boundary.is_protective()) {
    return;
  }

  CHECK(st_boundary.object_type() == StBoundaryProto::VEHICLE ||
        st_boundary.object_type() == StBoundaryProto::CYCLIST ||
        st_boundary.object_type() == StBoundaryProto::PEDESTRIAN)
      << StBoundaryProto::ObjectType_Name(st_boundary.object_type());

  double pass_time = 0.0;
  double yield_time = 0.0;

  if (!disable_pass_time_buffer &&
      (overlap_meta.pattern() == StOverlapMetaProto::ENTER ||
       overlap_meta.pattern() == StOverlapMetaProto::CROSS ||
       overlap_meta.pattern() == StOverlapMetaProto::INTERFERE)) {
    pass_time =
        ComputePassTime(st_boundary, init_v, vehicle_params, st_traj_mgr);
  }

  if (overlap_meta.pattern() == StOverlapMetaProto::ENTER ||
      overlap_meta.pattern() == StOverlapMetaProto::STAY ||
      overlap_meta.pattern() == StOverlapMetaProto::LEAVE ||
      overlap_meta.pattern() == StOverlapMetaProto::CROSS ||
      overlap_meta.pattern() == StOverlapMetaProto::INTERFERE) {
    yield_time = ComputeYieldTime(st_boundary, st_traj_mgr);
  }

  const auto decision_param = st_boundary_wd->decision_param();
  pass_time = pass_time * decision_param.pass_time_factor +
              decision_param.pass_time_additional_buffer;
  yield_time = yield_time * decision_param.yield_time_factor +
               decision_param.yield_time_additional_buffer;
  st_boundary_wd->SetTimeBuffer(pass_time, yield_time);
}

}  // namespace planning
}  // namespace e2e_noa
