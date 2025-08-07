///////////////////////////////////////////////////////////////////////////////
//
// Generic multi-vehicle game.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef ILQGAMES_EXAMPLES_MULTI_VEHICLE_GAME_H
#define ILQGAMES_EXAMPLES_MULTI_VEHICLE_GAME_H

#include <memory>
#include <vector>

#include "ilq_games/include/dynamics/concatenated_dynamical_system.h"
#include "ilq_games/include/dynamics/single_player_car_6d.h"
#include "ilq_games/include/geometry/polyline2.h"
#include "ilq_games/include/solver/problem.h"
#include "ilq_games/include/utils/ilq_types.h"

namespace e2e_noa::planning {

struct VehicleStaticData {
  int id = 0;
  float inter_axle_distance = 4.0;
  float width = 1.8;
  float length = 4.5;
  std::string type = "car";
  // 上下限参数
  float max_v;
  float max_a = 4;
  float min_a = -6;
  float max_phi = 0.8;
  float min_proximity = 6;

  // Cost权重
  float state_regularization = 1;
  float control_regularization = 5;
  float lane_cost_w = 25;
  float goal_x_cost_w = 100;
  float goal_y_cost_w = 100;
  float tar_v_cost_w = 100;
  float omega_cost_w = 0.1;
  float jerk_cost_w = 0.1;
  float lane_boundary_cost_w = 25;
  float vlimit_cost_w = 1;
  float alimit_cost_w = 1;
  float phi_limit_cost_w = 1;
  float proximity_cost_w = 20;
};

struct VehicleDynamicData {
  // 初始状态
  float init_x;
  float init_y;
  float init_heading;
  float init_v;
  // 目标终点
  float goal_x;
  float goal_y;
  // 期望速度
  float tar_v;
  // 中心线参数
  PointList2 lane;
  float lane_half_width = 1.75;
};

struct SolverParams {
  // Consider a solution converged once max elementwise difference is below this
  // tolerance or solver has exceeded a maximum number of iterations.
  float convergence_tolerance = 1e-1;
  size_t max_solver_iters = 100;

  // Linesearch parameters. If flag is set 'true', then applied initial alpha
  // scaling to all strategies and backs off geometrically at the given rate for
  // the specified number of steps.
  bool linesearch = true;
  float initial_alpha_scaling = 0.5;
  float geometric_alpha_scaling = 0.5;
  size_t max_backtracking_steps = 10;
  float expected_decrease_fraction = 0.1;

  // State and control regularization.
  float state_regularization = 0.0;
  float control_regularization = 0.0;

  // Augmented Lagrangian parameters.
  size_t unconstrained_solver_max_iters = 10;

  // log
  bool enable_online_log = false;
  bool enable_offline_log = false;
};  // struct SolverParams

struct TimeSet {
  Time time_step = 0.2;
  Time time_horizon = 8.0;
};

// 多智能体博弈建模
class MultiVehicleGame : public Problem {
 public:
  MultiVehicleGame(
      const std::vector<VehicleStaticData> &vehicles_static_dataset,
      const std::vector<VehicleDynamicData> &vehicles_dynamic_dataset);
  ~MultiVehicleGame() {}
  // Construct dynamics, initial state, and player costs.
  void ConstructDynamics();
  void ConstructInitialState();
  void ConstructPlayerCosts();
  const std::vector<VehicleStaticData> &GetVehicleStaticData() const {
    return vehicles_static_dataset_;
  }
  const std::vector<VehicleDynamicData> &GetVehicleDynamicData() const {
    return vehicles_dynamic_dataset_;
  }

 private:
  const std::vector<VehicleStaticData> vehicles_static_dataset_;
  const std::vector<VehicleDynamicData> vehicles_dynamic_dataset_;
  std::vector<Dimension> x_starts_;
};

// 多智能体博弈问题求解
class MutiVehicleGameSolve {
 public:
  MutiVehicleGameSolve(
      const SolverParams &input_solvers_params,
      const std::vector<VehicleStaticData> &vehicles_static_dataset,
      const std::vector<VehicleDynamicData> &vehicles_dynamic_dataset)
      : params_(input_solvers_params),
        problem_(std::make_shared<MultiVehicleGame>(
            vehicles_static_dataset, vehicles_dynamic_dataset)){};

  ~MutiVehicleGameSolve(){};
  void GameProblemSolve(std::string env_name, const uint64_t seq_num = 0);
  OperatingPoint GetFinalOperatorPoint() const {
    return final_operating_point_;
  }

 private:
  SolverParams params_;
  std::shared_ptr<MultiVehicleGame> problem_;
  OperatingPoint final_operating_point_;
};

}  // namespace e2e_noa::planning

#endif  // MULTI_VEHICLE_GAME_H