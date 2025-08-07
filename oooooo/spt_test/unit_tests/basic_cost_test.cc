#include <gtest/gtest.h>

#include <iostream>

#include "common/spt/lat_planner/lat_planner_interface/cost/heading_cost.h"
#include "common/spt/lat_planner/lat_planner_interface/cost/ref_a_cost.h"
#include "common/spt/lat_planner/lat_planner_interface/cost/ref_s_cost.h"
#include "common/spt/lat_planner/lat_planner_interface/cost/v_limit_cost.h"
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
}  // namespace
// begin heading cost test
TEST(Spt, heading_get_cost_test) {
  auto spt_data = std::make_unique<LatSptDataCache>();
  spt_data->ResizeStepData(10);
  const double heading_weight = 2.0;

  const int idx = 0;
  spt_data->SetSptStepData(idx, SptStepData::HEADING_WEIGHT, heading_weight);
  CircleModelInfo circle_info = CircleModelInfo();
  circle_info.Initialize();
  circle_info.SetCircleData(SptCircleData::MATCHED_REF_ThETA, 1.0);
  const std::vector<CircleModelInfo> val = {
      circle_info,       CircleModelInfo(), CircleModelInfo(),
      CircleModelInfo(), CircleModelInfo(), CircleModelInfo(),
      CircleModelInfo(), CircleModelInfo(), CircleModelInfo(),
      CircleModelInfo()};
  spt_data->SetCircleModelInfo(idx, val);

  auto cost = std::make_unique<HeadingCost>(*spt_data);
  CONSTRAIN_TYPES(8, 2, 3)
  State x = State::Zero();
  x[StateIndex::THETA] = 1.5;
  double result = cost->GetCost(x, Control::Zero(), idx);
  EXPECT_NEAR(result, 0.25, 1e-6);

  x[StateIndex::THETA] = 3.0;
  result = cost->GetCost(x, Control::Zero(), idx);
  EXPECT_NEAR(result, 4.0, 1e-6);
  x[StateIndex::THETA] = -7.0;
  result = cost->GetCost(x, Control::Zero(), idx);
  EXPECT_NEAR(result, 64.0, 1e-6);
}

TEST(Spt, heading_cost_gardient_and_hessian_test) {
  auto spt_data = std::make_unique<LatSptDataCache>();
  spt_data->ResizeStepData(10);
  const double heading_weight = 10.0;

  const int idx = 0;
  spt_data->SetSptStepData(idx, SptStepData::HEADING_WEIGHT, heading_weight);
  CircleModelInfo circle_info = CircleModelInfo();
  circle_info.Initialize();
  circle_info.SetCircleData(SptCircleData::MATCHED_REF_ThETA, 1.0);
  const std::vector<CircleModelInfo> val = {
      circle_info,       CircleModelInfo(), CircleModelInfo(),
      CircleModelInfo(), CircleModelInfo(), CircleModelInfo(),
      CircleModelInfo(), CircleModelInfo(), CircleModelInfo(),
      CircleModelInfo()};
  spt_data->SetCircleModelInfo(idx, val);

  auto cost = std::make_unique<HeadingCost>(*spt_data);
  CONSTRAIN_TYPES(8, 2, 3)
  State x = State::Zero();
  x[StateIndex::THETA] = 1.0;
  Control u = Control::Zero();
  Lx lx = Lx::Zero();
  Lu lu = Lu::Zero();
  Lxx lxx = Lxx::Zero();
  Lxu lxu = Lxu::Zero();
  Luu luu = Luu::Zero();
  cost->GetGradientAndHessian(x, u, idx, &lx, &lu, &lxx, &lxu, &luu);
  EXPECT_NEAR(lx(TestStateIndex::THETA), 0.0, 1e-6);
  EXPECT_NEAR(lxx(TestStateIndex::THETA, TestStateIndex::THETA), 10.0, 1e-6);

  x[StateIndex::THETA] = 3.0;
  lx = Lx::Zero();
  cost->GetGradientAndHessian(x, u, idx, &lx, &lu, &lxx, &lxu, &luu);
  EXPECT_NEAR(lx(TestStateIndex::THETA), 20.0, 1e-6);
  EXPECT_NEAR(lxx(TestStateIndex::THETA, TestStateIndex::THETA), 20.0, 1e-6);
  x[StateIndex::THETA] = -7.0;
  lx = Lx::Zero();
  cost->GetGradientAndHessian(x, u, idx, &lx, &lu, &lxx, &lxu, &luu);
  EXPECT_NEAR(lx(TestStateIndex::THETA), -80.0, 1e-6);
  EXPECT_NEAR(lxx(TestStateIndex::THETA, TestStateIndex::THETA), 30.0, 1e-6);
}
// end heading cost test
// begin ref a cost test
TEST(Spt, ref_a_get_cost_test) {
  auto spt_data = std::make_unique<LatSptDataCache>();
  spt_data->ResizeStepData(10);
  const double ref_a_weight = 2.0;

  const int idx = 0;
  spt_data->SetSptStepData(idx, SptStepData::REF_A_WEIGHT, ref_a_weight);
  CircleModelInfo circle_info = CircleModelInfo();
  circle_info.Initialize();
  circle_info.SetCircleData(SptCircleData::MATCHED_REF_A, 1.0);
  const std::vector<CircleModelInfo> val = {
      circle_info,       CircleModelInfo(), CircleModelInfo(),
      CircleModelInfo(), CircleModelInfo(), CircleModelInfo(),
      CircleModelInfo(), CircleModelInfo(), CircleModelInfo(),
      CircleModelInfo()};
  spt_data->SetCircleModelInfo(idx, val);

  auto cost = std::make_unique<RefACost>(*spt_data);
  CONSTRAIN_TYPES(8, 2, 3)
  State x = State::Zero();
  x[StateIndex::ACC] = 1.5;
  double result = cost->GetCost(x, Control::Zero(), idx);
  EXPECT_NEAR(result, 0.25, 1e-6);

  x[StateIndex::ACC] = 3.0;
  result = cost->GetCost(x, Control::Zero(), idx);
  EXPECT_NEAR(result, 4.0, 1e-6);
  x[StateIndex::ACC] = -10.0;
  result = cost->GetCost(x, Control::Zero(), idx);
  EXPECT_NEAR(result, 121.0, 1e-6);
}

TEST(Spt, ref_a_cost_gardient_and_hessian_test) {
  auto spt_data = std::make_unique<LatSptDataCache>();
  spt_data->ResizeStepData(10);
  const double ref_a_weight = 2.0;

  const int idx = 0;
  spt_data->SetSptStepData(idx, SptStepData::REF_A_WEIGHT, ref_a_weight);
  CircleModelInfo circle_info = CircleModelInfo();
  circle_info.Initialize();
  circle_info.SetCircleData(SptCircleData::MATCHED_REF_A, 1.0);
  const std::vector<CircleModelInfo> val = {
      circle_info,       CircleModelInfo(), CircleModelInfo(),
      CircleModelInfo(), CircleModelInfo(), CircleModelInfo(),
      CircleModelInfo(), CircleModelInfo(), CircleModelInfo(),
      CircleModelInfo()};
  spt_data->SetCircleModelInfo(idx, val);

  auto cost = std::make_unique<RefACost>(*spt_data);
  CONSTRAIN_TYPES(8, 2, 3)
  State x = State::Zero();
  x[StateIndex::ACC] = 1.0;
  Control u = Control::Zero();
  Lx lx = Lx::Zero();
  Lu lu = Lu::Zero();
  Lxx lxx = Lxx::Zero();
  Lxu lxu = Lxu::Zero();
  Luu luu = Luu::Zero();
  cost->GetGradientAndHessian(x, u, idx, &lx, &lu, &lxx, &lxu, &luu);
  EXPECT_NEAR(lx(TestStateIndex::ACC), 0.0, 1e-6);
  EXPECT_NEAR(lxx(TestStateIndex::ACC, TestStateIndex::ACC), 2.0, 1e-6);

  x[StateIndex::ACC] = 3.0;
  lx = Lx::Zero();
  cost->GetGradientAndHessian(x, u, idx, &lx, &lu, &lxx, &lxu, &luu);
  EXPECT_NEAR(lx(TestStateIndex::ACC), 4.0, 1e-6);
  EXPECT_NEAR(lxx(TestStateIndex::ACC, TestStateIndex::ACC), 4.0, 1e-6);
  x[StateIndex::ACC] = -7.0;
  lx = Lx::Zero();
  cost->GetGradientAndHessian(x, u, idx, &lx, &lu, &lxx, &lxu, &luu);
  EXPECT_NEAR(lx(TestStateIndex::ACC), -16.0, 1e-6);
  EXPECT_NEAR(lxx(TestStateIndex::ACC, TestStateIndex::ACC), 6.0, 1e-6);
}
// end ref a cost test
// begin ref s cost test
TEST(Spt, ref_s_get_cost_test) {
  auto spt_data = std::make_unique<LatSptDataCache>();
  spt_data->ResizeStepData(10);
  const double ref_s_cost_weight = 2.0;

  const int idx = 0;
  spt_data->SetSptConstant(SptConstant::REF_S_COST_HUBER_THRESHOLD, 2.0);
  spt_data->SetSptStepData(idx, SptStepData::REF_S_COST_WEIGHT,
                           ref_s_cost_weight);

  CircleModelInfo circle_info = CircleModelInfo();
  circle_info.Initialize();
  circle_info.SetLength(4.5);
  circle_info.SetWidth(2.0);
  circle_info.SetCircleData(SptCircleData::S_OFFSET, 1.0);
  circle_info.SetCircleData(SptCircleData::MATCHED_REF_COS_THETA, 0.0);
  circle_info.SetCircleData(SptCircleData::MATCHED_REF_SIN_THETA, 1.0);
  const std::vector<CircleModelInfo> val = {
      circle_info,       CircleModelInfo(), CircleModelInfo(),
      CircleModelInfo(), CircleModelInfo(), CircleModelInfo(),
      CircleModelInfo(), CircleModelInfo(), CircleModelInfo(),
      CircleModelInfo()};
  spt_data->SetCircleModelInfo(idx, val);

  auto cost = std::make_unique<RefSCost>(*spt_data);
  CONSTRAIN_TYPES(8, 2, 3)
  State x = State::Zero();

  spt_data->SetSptStepData(idx, SptStepData::SIN_THETA, 1);
  spt_data->SetSptStepData(idx, SptStepData::COS_THETA, 0);
  double result = cost->GetCost(x, Control::Zero(), idx);
  EXPECT_NEAR(result, 1.0, 1e-6);

  spt_data->SetSptStepData(idx, SptStepData::REF_S_COST_WEIGHT, 4.0);
  result = cost->GetCost(x, Control::Zero(), idx);
  EXPECT_NEAR(result, 2.0, 1e-6);
  auto circle_model_info = spt_data->GetMutableCircleModelInfo(idx);
  circle_model_info->at(0).SetCircleData(SptCircleData::S_OFFSET, 2.5);
  result = cost->GetCost(x, Control::Zero(), idx);
  EXPECT_NEAR(result, 12.0, 1e-6);
}

TEST(Spt, ref_s_cost_gardient_and_hessian_test) {
  auto spt_data = std::make_unique<LatSptDataCache>();
  spt_data->ResizeStepData(10);
  const double ref_s_cost_weight = 2.0;

  const int idx = 0;
  spt_data->SetSptConstant(SptConstant::REF_S_COST_HUBER_THRESHOLD, 2.0);
  spt_data->SetSptStepData(idx, SptStepData::REF_S_COST_WEIGHT,
                           ref_s_cost_weight);
  spt_data->SetSptStepData(idx, SptStepData::SIN_THETA, 1);
  spt_data->SetSptStepData(idx, SptStepData::COS_THETA, 0);
  CircleModelInfo circle_info = CircleModelInfo();
  circle_info.Initialize();
  circle_info.SetLength(4.5);
  circle_info.SetWidth(2.0);
  circle_info.SetCircleData(SptCircleData::S_OFFSET, 1.0);
  circle_info.SetCircleData(SptCircleData::MATCHED_REF_COS_THETA, 0.0);
  circle_info.SetCircleData(SptCircleData::MATCHED_REF_SIN_THETA, 1.0);
  const std::vector<CircleModelInfo> val = {
      circle_info,       CircleModelInfo(), CircleModelInfo(),
      CircleModelInfo(), CircleModelInfo(), CircleModelInfo(),
      CircleModelInfo(), CircleModelInfo(), CircleModelInfo(),
      CircleModelInfo()};
  spt_data->SetCircleModelInfo(idx, val);

  auto cost = std::make_unique<RefSCost>(*spt_data);
  CONSTRAIN_TYPES(8, 2, 3)
  State x = State::Zero();
  spt_data->SetSptStepData(idx, SptStepData::SIN_THETA, 1);
  spt_data->SetSptStepData(idx, SptStepData::COS_THETA, 0);
  Control u = Control::Zero();
  Lx lx = Lx::Zero();
  Lu lu = Lu::Zero();
  Lxx lxx = Lxx::Zero();
  Lxu lxu = Lxu::Zero();
  Luu luu = Luu::Zero();
  cost->GetGradientAndHessian(x, u, idx, &lx, &lu, &lxx, &lxu, &luu);
  std::cout << "lx: " << lx << std::endl;
  std::cout << "lxx: " << lxx << std::endl;
  std::cout << "REF_S_COST_HUBER_THRESHOLD: "
            << spt_data->GetSptConstant(SptConstant::REF_S_COST_HUBER_THRESHOLD)
            << std::endl;
  std::cout << "SptStepData::REF_S_COST_WEIGHT : "
            << spt_data->GetSptStepData(idx, SptStepData::REF_S_COST_WEIGHT)
            << std::endl;
  EXPECT_NEAR(lx(TestStateIndex::X), 0.0, 1e-6);
  EXPECT_NEAR(lx(TestStateIndex::Y), 2.0, 1e-6);
  EXPECT_NEAR(lx(TestStateIndex::THETA), 4.0, 1e-6);
  EXPECT_NEAR(lxx(TestStateIndex::Y, TestStateIndex::Y), 2.0, 1e-6);
  EXPECT_NEAR(lxx(TestStateIndex::THETA, TestStateIndex::THETA), -1.0, 1e-6);

  auto circle_model_info = spt_data->GetMutableCircleModelInfo(idx);
  circle_model_info->at(0).SetCircleData(SptCircleData::S_OFFSET, 3.0);
  lx = Lx::Zero();
  lu = Lu::Zero();
  lxx = Lxx::Zero();
  lxu = Lxu::Zero();
  luu = Luu::Zero();
  cost->GetGradientAndHessian(x, u, idx, &lx, &lu, &lxx, &lxu, &luu);
  std::cout << "lx: " << lx << std::endl;
  std::cout << "lxx: " << lxx << std::endl;
  EXPECT_NEAR(lx(TestStateIndex::X), 0.0, 1e-6);
  EXPECT_NEAR(lx(TestStateIndex::Y), 4.0, 1e-6);
  EXPECT_NEAR(lx(TestStateIndex::THETA), 8.0, 1e-6);
  EXPECT_NEAR(lxx(TestStateIndex::Y, TestStateIndex::Y), 0.0, 1e-6);
  EXPECT_NEAR(lxx(TestStateIndex::THETA, TestStateIndex::THETA), -18.0, 1e-6);

  spt_data->SetSptStepData(idx, SptStepData::SIN_THETA, 0.9);
  spt_data->SetSptStepData(idx, SptStepData::COS_THETA, 0.4);
  lx = Lx::Zero();
  lu = Lu::Zero();
  lxx = Lxx::Zero();
  lxu = Lxu::Zero();
  luu = Luu::Zero();
  cost->GetGradientAndHessian(x, u, idx, &lx, &lu, &lxx, &lxu, &luu);
  std::cout << "lx: " << lx << std::endl;
  std::cout << "lxx: " << lxx << std::endl;
  EXPECT_NEAR(lx(TestStateIndex::X), 0.0, 1e-6);
  EXPECT_NEAR(lx(TestStateIndex::Y), 4.0, 1e-6);
  EXPECT_NEAR(lx(TestStateIndex::THETA), 14.4, 1e-6);
  EXPECT_NEAR(lxx(TestStateIndex::X, TestStateIndex::X), 0.0, 1e-6);
  EXPECT_NEAR(lxx(TestStateIndex::Y, TestStateIndex::Y), 0.0, 1e-6);
  EXPECT_NEAR(lxx(TestStateIndex::THETA, TestStateIndex::THETA), -13.0, 1e-6);
}
// end ref s cost test
// begin vlimit cost test
TEST(Spt, vlimit_get_cost_test) {
  auto spt_data = std::make_unique<LatSptDataCache>();
  spt_data->ResizeStepData(10);
  const double v_limit_ub_weight = 200.0;
  const double v_limit_lb_weight = 100.0;
  const int idx = 0;
  spt_data->SetSptStepData(idx, SptStepData::V_LIMIT_UB_WEIGHT,
                           v_limit_ub_weight);
  spt_data->SetSptStepData(idx, SptStepData::V_LIMIT_LB_WEIGHT,
                           v_limit_lb_weight);
  auto cost = std::make_unique<VLimitCost>(*spt_data);
  CONSTRAIN_TYPES(8, 2, 3)
  State x = State::Zero();
  x[StateIndex::V] = 11.0;
  double result = cost->GetCost(x, Control::Zero(), idx);
  EXPECT_NEAR(result, 100.0, 1e-6);

  x[StateIndex::V] = 3.0;
  result = cost->GetCost(x, Control::Zero(), idx);
  EXPECT_NEAR(result, 200.0, 1e-6);
}

TEST(Spt, vlimit_cost_gardient_and_hessian_test) {
  auto spt_data = std::make_unique<LatSptDataCache>();
  spt_data->ResizeStepData(10);
  const double v_limit_ub_weight = 200.0;
  const double v_limit_lb_weight = 100.0;
  const int idx = 0;
  spt_data->SetSptStepData(idx, SptStepData::V_LIMIT_UB_WEIGHT,
                           v_limit_ub_weight);
  spt_data->SetSptStepData(idx, SptStepData::V_LIMIT_LB_WEIGHT,
                           v_limit_lb_weight);
  auto cost = std::make_unique<VLimitCost>(*spt_data);
  CONSTRAIN_TYPES(8, 2, 3)
  State x = State::Zero();
  x[StateIndex::V] = 1.0;
  Control u = Control::Zero();
  Lx lx = Lx::Zero();
  Lu lu = Lu::Zero();
  Lxx lxx = Lxx::Zero();
  Lxu lxu = Lxu::Zero();
  Luu luu = Luu::Zero();
  cost->GetGradientAndHessian(x, u, idx, &lx, &lu, &lxx, &lxu, &luu);
  EXPECT_NEAR(lx(TestStateIndex::V), -400.0, 1e-6);
  EXPECT_NEAR(lxx(TestStateIndex::V, TestStateIndex::V), 100.0, 1e-6);

  x[StateIndex::V] = 13.0;
  lx = Lx::Zero();
  lxx = Lxx::Zero();
  cost->GetGradientAndHessian(x, u, idx, &lx, &lu, &lxx, &lxu, &luu);
  EXPECT_NEAR(lx(TestStateIndex::V), 600.0, 1e-6);
  EXPECT_NEAR(lxx(TestStateIndex::V, TestStateIndex::V), 200.0, 1e-6);
  x[StateIndex::V] = -7.0;
  lx = Lx::Zero();
  lxx = Lxx::Zero();
  cost->GetGradientAndHessian(x, u, idx, &lx, &lu, &lxx, &lxu, &luu);
  EXPECT_NEAR(lx(TestStateIndex::V), -1200.0, 1e-6);
  EXPECT_NEAR(lxx(TestStateIndex::V, TestStateIndex::V), 100.0, 1e-6);
}
// end vlimit cost test
}  // namespace test
}  // namespace spt
}  // namespace e2e_noa