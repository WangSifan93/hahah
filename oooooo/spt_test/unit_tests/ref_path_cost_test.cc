#include <gtest/gtest.h>

#include <iostream>

#include "common/spt/lat_planner/lat_planner_interface/lat_planner_interface.h"
#include "common/spt/lat_planner/lat_planner_interface/model/spt_lat_model.h"
#include "common/spt/lat_planner/lateral_opt.h"
#include "common/spt/solver/utils.h"
namespace e2e_noa {
namespace spt {
namespace test {
namespace {
enum TestStateIndex { X = 0, Y, THETA, KAPPA, DKAPPA, V, ACC, S, STATE_SIZE };
enum TestControlIndex { DDKAPPA = 0, JERK = 1, INPUT_SIZE };
typedef Eigen::Matrix<double, TestStateIndex::STATE_SIZE, 1> TestState;
typedef Eigen::Matrix<double, TestControlIndex::INPUT_SIZE, 1> TestControl;

TestState UpdateDynamicsWithDt(const TestState &x, const TestControl &u,
                               const double dt) {
  TestState next_state;

  const double pos_x = x[TestStateIndex::X];
  const double pos_y = x[TestStateIndex::Y];
  const double theta = x[TestStateIndex::THETA];
  const double v = x[TestStateIndex::V];
  const double kappa = x[TestStateIndex::KAPPA];
  const double a = x[TestStateIndex::ACC];
  const double s = x[TestStateIndex::S];
  const double psi = x[TestStateIndex::DKAPPA];
  const double chi = u[TestControlIndex::DDKAPPA];
  const double j = u[TestControlIndex::DDKAPPA];

  const double half_da = 0.5 * j * dt;
  const double halfway_a = a + half_da;
  const double half_dv = 0.5 * halfway_a * dt;
  const double halfway_v = v + half_dv;
  const double half_dpsi = 0.5 * chi * dt;
  const double halfway_psi = psi + half_dpsi;
  const double half_dkappa = 0.5 * psi * dt + 0.25 * chi * Sqr(dt);
  const double halfway_kappa = kappa + half_dkappa;
  const double half_dtheta = 0.5 * halfway_v * halfway_kappa * dt;
  const double halfway_theta = theta + half_dtheta;
  const double cos_halfway_theta = std::cos(halfway_theta);
  const double sin_halfway_theta = std::sin(halfway_theta);

  constexpr double kMaxValue = 1e50;

  const double next_pos_x = std::clamp(
      pos_x + halfway_v * cos_halfway_theta * dt, -kMaxValue, kMaxValue);
  const double next_pos_y = std::clamp(
      pos_y + halfway_v * sin_halfway_theta * dt, -kMaxValue, kMaxValue);
  const double next_theta =
      std::clamp(halfway_theta + half_dtheta, -kMaxValue, kMaxValue);
  const double next_v = std::clamp(halfway_v + half_dv, -kMaxValue, kMaxValue);
  const double next_kappa =
      std::clamp(halfway_kappa + half_dkappa, -kMaxValue, kMaxValue);
  const double next_psi =
      std::clamp(halfway_psi + half_dpsi, -kMaxValue, kMaxValue);
  const double next_a = std::clamp(halfway_a + half_da, -kMaxValue, kMaxValue);
  const double next_s =
      std::clamp(s + v * dt + 0.5 * a * Sqr(dt) + j * Cube(dt) / 6.0,
                 -kMaxValue, kMaxValue);

  next_state[TestStateIndex::X] = next_pos_x;
  next_state[TestStateIndex::Y] = next_pos_y;
  next_state[TestStateIndex::THETA] = next_theta;
  next_state[TestStateIndex::KAPPA] = next_kappa;
  next_state[TestStateIndex::DKAPPA] = next_psi;
  next_state[TestStateIndex::V] = next_v;
  next_state[TestStateIndex::ACC] = next_a;
  next_state[TestStateIndex::S] = next_s;

  return next_state;
}

std::vector<RefPoint> GenerateRefLine(const int horizon) {
  std::vector<RefPoint> ref_path;
  int size = 3 * horizon;
  TestState init_state{0.0, 0.0, 0.0, 0.0, 0.0, 3.0, 0.0, 0.0};
  TestControl init_control{0.0, 0.0};
  double dt = 0.2;
  TestState state = init_state;

  ref_path.reserve(size);
  RefPoint init_ref_point;
  init_ref_point.x = init_state[TestStateIndex::X];
  init_ref_point.y = init_state[TestStateIndex::Y];
  init_ref_point.theta = init_state[TestStateIndex::THETA];
  init_ref_point.s = 0.0;
  init_ref_point.kappa = init_state[TestStateIndex::KAPPA];
  ref_path.push_back(std::move(init_ref_point));

  for (int i = 0; i < size; ++i) {
    RefPoint temp_point;
    state = UpdateDynamicsWithDt(state, init_control, dt);
    temp_point.x = state[TestStateIndex::X];
    temp_point.y = state[TestStateIndex::Y];
    temp_point.theta = state[TestStateIndex::THETA];
    temp_point.kappa = state[TestStateIndex::KAPPA];
    temp_point.dkappa = state[TestStateIndex::DKAPPA];
    temp_point.s = state[TestStateIndex::S];
    temp_point.v = state[TestStateIndex::V];
    temp_point.a = state[TestStateIndex::ACC];

    ref_path.push_back(std::move(temp_point));
  }

  return ref_path;
}
}  // namespace

TEST(Spt, ref_path_cost_test) {
  e2e_noa::spt::LatPlannerInput input;
  e2e_noa::spt::LatPlannerOutput output;
  input.init_point = {20.0, 2.0, 0.0, 0.0, 3.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  auto &lat_planner = e2e_noa::spt::LatPlannerInterface::get_instance();
  const auto solver_config = *lat_planner.GetSolverConfig();

  std::vector<RefPoint> ref_path = GenerateRefLine(solver_config.horizon);
  // for (const auto &point : ref_path) {
  //   std::cout << "x: " << point.x << std::endl;
  //   std::cout << "y: " << point.y << std::endl;
  //   std::cout << "theta: " << point.theta << std::endl;
  //   std::cout << "kappa: " << point.kappa << std::endl;
  //   std::cout << "dkappa: " << point.dkappa << std::endl;
  //   std::cout << "s: " << point.s << std::endl;
  //   std::cout << "v: " << point.v << std::endl;
  //   std::cout << "acc: " << point.a << std::endl;
  //   std::cout << "----------------------" << std::endl;
  // }
  lat_planner.Reset();
  lat_planner.ResizeTimestep(solver_config.horizon + 1);
  for (int i = 0; i < solver_config.horizon; ++i) {
    lat_planner.SetTimeStep(0.2, i);
  }
  lat_planner.Update(ref_path, &input, &output);
}

}  // namespace test
}  // namespace spt
}  // namespace e2e_noa