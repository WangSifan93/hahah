#pragma once

#include <array>
#include <vector>

#include "common/spt/solver/utils.h"

namespace e2e_noa {
namespace spt {
void RunSptTest();
// SOLVER_TYPES(STATE_SIZE, INPUT_SIZE)
struct TrajectoryPoint {
  double x;
  double y;
  double theta;
  double s;
  double v;
  double a;
  double jerk;
  double t;
  double kappa;
  double dkappa;
};

struct EgoParams {
  double width = 2.0;
  double length = 5.0;
  double center_to_rear_axle = 1.5;
  double wheel_base = 3.0;
};

struct SptParams {
  double ref_path_cost_weight = 50.0;
  double ref_v_cost_weight = 10.0;
  double ddkappa_weight = 2000.0;
  double djerk_weight = 5.0;
  double heading_weight = 1.0;
  double ref_a_weight = 1.0;
  double ref_s_cost_weight = 10.0;
  double v_limit_ub_weight = 200.0;
  double v_limit_lb_weight = 100.0;
  double jerk_long_weight = 10.0;
  double jerk_long_ub_weight = 10.0;
  double jerk_long_lb_weight = 10.0;
  double jerk_long_boundary_upper = 2.0;
  double jerk_long_boundary_lower = -5.0;
  std::vector<double> v_list{2.778, 8.333, 16.667, 25.000, 33.333};
  std::vector<double> front_angle_list{30.00, 6.19, 1.90, 1.00, 0.90};
  std::vector<double> front_angle_rate_list{30.00, 9.52, 2.90, 2.30, 1.90};
};

struct LatPlannerInput {
  EgoParams ego_params;
  TrajectoryPoint init_point;
  SptParams spt_params;
};

struct LatPlannerOutput {};

struct RefPoint {
  double x;
  double y;
  double theta;
  double kappa;
  double dkappa;
  double s;
  double v;
  double a;
};
}  // namespace spt
}  // namespace e2e_noa