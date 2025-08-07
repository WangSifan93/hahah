#include "common/spt/lat_planner/lat_planner_interface/cost/long_jerk_cost.h"

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
enum TestStateIndex { X = 0, Y, THETA, KAPPA, DKAPPA, V, ACC, JERK, STATE_SIZE };
enum TestControlIndex { DDKAPPA = 0, DJERK = 1, INPUT_SIZE };
}  // namespace

TEST(Spt, lon_acc_get_cost_test) {
  auto spt_data = std::make_unique<LatSptDataCache>();
  spt_data->ResizeStepData(10);
  const double jerk_long_weight = 10.0;
  const double jerk_long_ub_weight = 1.0;
  const double jerk_long_lb_weight = 1.0;
  const double jerk_long_boundary_upper = 2.0;
  const double jerk_long_boundary_lower = -5.0;
  const int idx = 0;
  spt_data->SetSptStepData(idx, SptStepData::JERK_LONG_WEIGHT, jerk_long_weight);
  spt_data->SetSptStepData(idx, SptStepData::JERK_LONG_UB_WEIGHT,
                           jerk_long_ub_weight);
  spt_data->SetSptStepData(idx, SptStepData::JERK_LONG_LB_WEIGHT,
                           jerk_long_lb_weight);
  spt_data->SetSptStepData(idx, SptStepData::JERK_LONG_BOUNDARY_UPPER,
                           jerk_long_boundary_upper);
  spt_data->SetSptStepData(idx, SptStepData::JERK_LONG_BOUNDARY_LOWER,
                           jerk_long_boundary_lower);
  auto cost = std::make_unique<LONGJerkCost>(*spt_data);
  CONSTRAIN_TYPES(8, 2, 3)
  State x = State::Zero();
  x[StateIndex::JERK] = 1.0;
  double result = cost->GetCost(x, Control::Zero(), idx);
  EXPECT_NEAR(result, 5.0, 1e-6);

  x[StateIndex::JERK] = 3.0;
  result = cost->GetCost(x, Control::Zero(), idx);
  EXPECT_NEAR(result, 45.5, 1e-6);
  x[StateIndex::JERK] = -7.0;
  result = cost->GetCost(x, Control::Zero(), idx);
  EXPECT_NEAR(result, 247, 1e-6);
}

TEST(Spt, lon_jerk_cost_gardient_and_hessian_test) {
  auto spt_data = std::make_unique<LatSptDataCache>();
  spt_data->ResizeStepData(10);
  const double jerk_long_weight = 10.0;
  const double jerk_long_ub_weight = 1.0;
  const double jerk_long_lb_weight = 1.0;
  const double jerk_long_boundary_upper = 2.0;
  const double jerk_long_boundary_lower = -5.0;
  const int idx = 0;
  spt_data->SetSptStepData(idx, SptStepData::JERK_LONG_WEIGHT, jerk_long_weight);
  spt_data->SetSptStepData(idx, SptStepData::JERK_LONG_UB_WEIGHT,
                           jerk_long_ub_weight);
  spt_data->SetSptStepData(idx, SptStepData::JERK_LONG_LB_WEIGHT,
                           jerk_long_lb_weight);
  spt_data->SetSptStepData(idx, SptStepData::JERK_LONG_BOUNDARY_UPPER,
                           jerk_long_boundary_upper);
  spt_data->SetSptStepData(idx, SptStepData::JERK_LONG_BOUNDARY_LOWER,
                           jerk_long_boundary_lower);
  auto cost = std::make_unique<LONGJerkCost>(*spt_data);
  CONSTRAIN_TYPES(8, 2, 3)
  State x = State::Zero();
  x[StateIndex::JERK] = 1.0;
  Control u = Control::Zero();
  Lx lx = Lx::Zero();
  Lu lu = Lu::Zero();
  Lxx lxx = Lxx::Zero();
  Lxu lxu = Lxu::Zero();
  Luu luu = Luu::Zero();
  cost->GetGradientAndHessian(x, u, idx, &lx, &lu, &lxx, &lxu, &luu);
  EXPECT_NEAR(lx(TestStateIndex::JERK), 10.0, 1e-6);
  EXPECT_NEAR(lxx(TestStateIndex::JERK, TestStateIndex::JERK), 10.0, 1e-6);

  x[StateIndex::JERK] = 3.0;
  lx = Lx::Zero();
  cost->GetGradientAndHessian(x, u, idx, &lx, &lu, &lxx, &lxu, &luu);
  EXPECT_NEAR(lx(TestStateIndex::JERK), 31.0, 1e-6);
  EXPECT_NEAR(lxx(TestStateIndex::JERK, TestStateIndex::JERK), 21.0, 1e-6);
  x[StateIndex::JERK] = -7.0;
  lx = Lx::Zero();
  cost->GetGradientAndHessian(x, u, idx, &lx, &lu, &lxx, &lxu, &luu);
  EXPECT_NEAR(lx(TestStateIndex::JERK), -72.0, 1e-6);
  EXPECT_NEAR(lxx(TestStateIndex::JERK, TestStateIndex::JERK), 32.0, 1e-6);
}

}  // namespace test
}  // namespace spt
}  // namespace e2e_noa
