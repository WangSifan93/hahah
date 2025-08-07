#include "ilq_games/include/examples/muti_vehicle_game.h"

#include <glog/logging.h>

#include "ilq_games/include/cost/final_time_cost.h"
#include "ilq_games/include/cost/proximity_cost.h"
#include "ilq_games/include/cost/quadratic_cost.h"
#include "ilq_games/include/cost/quadratic_polyline2_cost.h"
#include "ilq_games/include/cost/semiquadratic_cost.h"
#include "ilq_games/include/cost/semiquadratic_polyline2_cost.h"
#include "ilq_games/include/solver/augmented_lagrangian_solver.h"

namespace e2e_noa::planning {
using DynamicsList = std::vector<std::shared_ptr<SinglePlayerDynamicalSystem>>;
using LaneKeepCost = std::shared_ptr<QuadraticPolyline2Cost>;
using LaneBoundryCost = std::shared_ptr<SemiquadraticPolyline2Cost>;
using SLog = std::shared_ptr<const SolverLog>;
using Car6D = SinglePlayerCar6D;

MultiVehicleGame::MultiVehicleGame(
    const std::vector<VehicleStaticData> &vehicles_static_dataset,
    const std::vector<VehicleDynamicData> &vehicles_dynamic_dataset)
    : Problem(),
      vehicles_static_dataset_(vehicles_static_dataset),
      vehicles_dynamic_dataset_(vehicles_dynamic_dataset) {
  // 预计算每辆车的状态起始索引
  Dimension x_start = 0;
  Dimension u_start = 0;
  for (const auto &vehicle_static_data : vehicles_static_dataset) {
    x_starts_.emplace_back(x_start);
    x_start += Car6D::kNumXDims;
  }
}

void MultiVehicleGame::ConstructDynamics() {
  // 合并所有车辆的系统动态
  DynamicsList dynamicslist;
  for (const auto &vehicle_static_data : vehicles_static_dataset_) {
    dynamicslist.emplace_back(
        std::make_shared<Car6D>(vehicle_static_data.inter_axle_distance));
  }
  dynamics_.reset(new ConcatenatedDynamicalSystem(dynamicslist));
}
// 构建多车的系统初始状态
void MultiVehicleGame::ConstructInitialState() {
  x0_ = VectorXf::Zero(dynamics_->XDim());
  // 多车系统的状态量初始值赋值
  for (size_t i = 0; i < vehicles_dynamic_dataset_.size(); ++i) {
    const auto &vehicle_dynamic_data = vehicles_dynamic_dataset_[i];
    const Dimension x_start = x_starts_[i];
    x0_(x_start + Car6D::kPxIdx) = vehicle_dynamic_data.init_x;
    x0_(x_start + Car6D::kPyIdx) = vehicle_dynamic_data.init_y;
    x0_(x_start + Car6D::kThetaIdx) = vehicle_dynamic_data.init_heading;
    x0_(x_start + Car6D::kVIdx) = vehicle_dynamic_data.init_v;
  }
}

void MultiVehicleGame::ConstructPlayerCosts() {
  const size_t num_players = vehicles_static_dataset_.size();
  player_costs_.reserve(num_players);

  // 正则化权重
  for (size_t i = 0; i < num_players; ++i) {
    const auto &vehicle_static_data = vehicles_static_dataset_[i];
    player_costs_.emplace_back("Player" + std::to_string(i + 1),
                               vehicle_static_data.state_regularization,
                               vehicle_static_data.control_regularization);
  }
  const float kFinalTimeWindow = 1.0;
  // 为每个车辆构建成本项
  for (size_t i = 0; i < num_players; ++i) {
    const auto &vehicle_static_data = vehicles_static_dataset_[i];
    const auto &vehicle_dynamic_data = vehicles_dynamic_dataset_[i];
    auto &cost = player_costs_[i];
    const Dimension x_start = x_starts_[i];
    // 获取车辆状态与控制量索引
    const Dimension px_idx = x_start + Car6D::kPxIdx;
    const Dimension py_idx = x_start + Car6D::kPyIdx;
    const Dimension v_idx = x_start + Car6D::kVIdx;
    const Dimension theta_idx = x_start + Car6D::kThetaIdx;
    const Dimension phi_idx = x_start + Car6D::kPhiIdx;
    const Dimension a_idx = x_start + Car6D::kAIdx;
    const Dimension omega_idx = Car6D::kOmegaIdx;
    const Dimension jerk_idx = Car6D::kJerkIdx;

    // 车道保持成本
    const LaneKeepCost lane_cost(new QuadraticPolyline2Cost(
        vehicle_static_data.lane_cost_w, Polyline2(vehicle_dynamic_data.lane),
        {px_idx, py_idx}, "LaneCenter"));

    cost.AddStateCost(lane_cost);

    // 车道边界软约束
    const LaneBoundryCost right_boundary_cost(new SemiquadraticPolyline2Cost(
        vehicle_static_data.lane_boundary_cost_w,
        Polyline2(vehicle_dynamic_data.lane), {px_idx, py_idx},
        vehicle_dynamic_data.lane_half_width, true,
        "LaneRightBoundary"));  // 右边取正
    const LaneBoundryCost left_boundary_cost(new SemiquadraticPolyline2Cost(
        vehicle_static_data.lane_boundary_cost_w,
        Polyline2(vehicle_dynamic_data.lane), {px_idx, py_idx},
        -vehicle_dynamic_data.lane_half_width, false,
        "LaneLeftBoundary"));  // 左边取负
    cost.AddStateCost(right_boundary_cost);
    cost.AddStateCost(left_boundary_cost);

    // 终点位置成本
    const auto goal_x_cost = std::make_shared<FinalTimeCost>(
        std::make_shared<QuadraticCost>(vehicle_static_data.goal_x_cost_w,
                                        px_idx, vehicle_dynamic_data.goal_x),
        time::getTimeHorizon() - kFinalTimeWindow, "goal_x");
    const auto goal_y_cost = std::make_shared<FinalTimeCost>(
        std::make_shared<QuadraticCost>(vehicle_static_data.goal_y_cost_w,
                                        py_idx, vehicle_dynamic_data.goal_y),
        time::getTimeHorizon() - kFinalTimeWindow, "goal_y");
    cost.AddStateCost(goal_x_cost);
    cost.AddStateCost(goal_y_cost);

    // 期望速度成本
    const auto nominal_v_cost =
        std::make_shared<QuadraticCost>(vehicle_static_data.tar_v_cost_w, v_idx,
                                        vehicle_dynamic_data.tar_v, "tar_v");
    cost.AddStateCost(nominal_v_cost);

    // 最大速度软约束
    const auto max_v_cost = std::make_shared<SemiquadraticCost>(
        vehicle_static_data.vlimit_cost_w, v_idx, vehicle_static_data.max_v,
        true, "max_v");
    cost.AddStateCost(max_v_cost);

    // 加速度上下限软约束
    const auto max_a_cost = std::make_shared<SemiquadraticCost>(
        vehicle_static_data.alimit_cost_w, a_idx, vehicle_static_data.max_a,
        true, "max_a");
    const auto min_a_cost = std::make_shared<SemiquadraticCost>(
        vehicle_static_data.alimit_cost_w, a_idx, vehicle_static_data.min_a,
        false, "min_a");
    cost.AddStateCost(max_a_cost);
    cost.AddStateCost(min_a_cost);

    // 前轮偏角上下限软约束
    const auto max_phi_cost = std::make_shared<SemiquadraticCost>(
        vehicle_static_data.phi_limit_cost_w, phi_idx,
        vehicle_static_data.max_phi, true, "max_phi");
    const auto min_phi_cost = std::make_shared<SemiquadraticCost>(
        vehicle_static_data.phi_limit_cost_w, phi_idx,
        -vehicle_static_data.max_phi, false, "MinPhi");
    cost.AddStateCost(max_phi_cost);
    cost.AddStateCost(min_phi_cost);

    // 控制成本
    const auto omega_cost = std::make_shared<QuadraticCost>(
        vehicle_static_data.omega_cost_w, omega_idx, 0.0, "Steering");
    const auto jerk_cost = std::make_shared<QuadraticCost>(
        vehicle_static_data.jerk_cost_w, jerk_idx, 0.0, "Jerk");
    cost.AddControlCost(i, omega_cost);
    cost.AddControlCost(i, jerk_cost);

    // 车辆间碰撞避免约束
    for (size_t j = 0; j < num_players; ++j) {
      if (i == j) continue;
      const auto other_x_start = x_starts_[j];
      const auto other_px_idx = other_x_start + Car6D::kPxIdx;
      const auto other_py_idx = other_x_start + Car6D::kPyIdx;
      const std::shared_ptr<ProximityCost> proximity_cost(new ProximityCost(
          vehicle_static_data.proximity_cost_w, {px_idx, py_idx},
          {other_px_idx, other_py_idx}, vehicle_static_data.min_proximity,
          "Proximity_" + std::to_string(i) + "_" + std::to_string(j)));
      cost.AddStateCost(proximity_cost);
    }
  }
}

void MutiVehicleGameSolve::GameProblemSolve(std::string env_name,
                                            const uint64_t seq_num) {
  // 问题初始化
  problem_->Initialize();
  AugmentedLagrangianSolver solver(problem_, params_);
  // 问题求解
  const auto start = std::chrono::system_clock::now();
  SLog log = solver.Solve();
  const std::vector<SLog> logs = {log};
  std::cout << "Solver time cost: "
            << std::chrono::duration<Time>(std::chrono::system_clock::now() -
                                           start)
                   .count()
            << " seconds." << std::endl;
  const auto &static_data = problem_->GetVehicleStaticData();
  const auto &dynamic_data = problem_->GetVehicleDynamicData();
  log->Save(static_data, dynamic_data, env_name, seq_num,
            params_.enable_online_log, params_.enable_offline_log);
  final_operating_point_ = solver.GetFinalOperatorPoint();
}
}  // namespace e2e_noa::planning