#ifndef PLANNER_SPEED_GAME_THEORY_ENVIRONMENT_H_
#define PLANNER_SPEED_GAME_THEORY_ENVIRONMENT_H_

#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

#include "../interactive_agent.h"
#include "absl/status/status.h"
#include "async/thread_pool.h"
#include "game_theory_common.h"
#include "game_theory_cost.h"
#include "object/spacetime_object_trajectory.h"
#include "object/spacetime_trajectory_manager.h"
#include "plan/discretized_path.h"
#include "planner_params.pb.h"
#include "speed/speed_limit_provider.h"
#include "speed/speed_vector.h"
#include "speed/st_boundary_with_decision.h"
#include "speed/st_graph.h"
#include "speed_planning.pb.h"
#include "speed_planning_params.pb.h"
#include "vehicle.pb.h"
namespace e2e_noa {
namespace planning {

struct GT_Interaction_Matrix {
  GT_Interaction_Data Matrix[MAX_GT_ACC_NUM][MAX_GT_ACC_NUM];
  int col;
  int row;
};

typedef struct GT_Result {
  bool is_valid;
  bool is_solution;

  GT_Interaction_Matrix gt_interaction_matrix;

} gt_result_t;

typedef struct GT_Static_Params {
  double ego_sampling_interval;
  double agent_sampling_interval;
  double time_step;
  double max_simulation_time;
  double max_simulation_dist;
  double ego_acc_to_acc_delay_time;
  double ego_acc_to_dec_delay_time;
  double ego_dec_to_acc_delay_time;
  double ego_dec_to_dec_delay_time;
  double ego_acc_default_delay_time;
  bool consider_ego_jerk;
  double ego_max_acc_jerk;
  double ego_max_dec_jerk;
} gt_static_params_t;

class Game_Theory_Interaction_Environment {
 public:
  Game_Theory_Interaction_Environment();

  void BuildUpGameEnvironment();

  void StartStackelbergGameProcess(gt_result_t *result);

  void InitGameTheoryEnvironment(InteractiveAgent *ego_interactive_info,
                                 InteractiveAgent *obj_interactive_info);

  ~Game_Theory_Interaction_Environment() = default;

  gt_static_params_t static_params_;
  bool is_valid;

 private:
  std::vector<double> agent_array_, ego_array_;
  std::unordered_map<double, gt_ego_sim_state_t> ego_traj_sim_state_;
  std::unordered_map<double, gt_obj_sim_state_t> obj_traj_sim_state_;
  GT_Interaction_Matrix gt_inter_matrix_;
  InteractiveAgent *ego_interactive_info_;
  InteractiveAgent *obj_interactive_info_;
  gt_ego_info_t gt_start_ego_info_;
  gt_agent_info_t gt_start_agent_info_;
  TurnLeftCrossCost turn_left_cost_;

  void UpdateAccArrayForBothPlayer(std::vector<double> &agent_array,
                                   std::vector<double> &ego_array);

  void SetUpEgoInitState(gt_ego_sim_state_t &gt_ego_sim_state,
                         const InteractiveAgent *ego_interactive_info);

  void SetUpObjInitState(gt_obj_sim_state_t &obj_state,
                         const InteractiveAgent *obj_interactive_info);

  void SetUpInteractionMatrix();

  void FindSimPoseFromEgoTraj(gt_ego_state_t *next_state,
                              const gt_ego_state_t *curr_state);

  std::optional<gt_ego_sim_state_t> UpdateEgoActionSimState(double ego_acc);

  void UpdateEgoAllActionSimState();

  void FindSimPoseFromObjTraj(gt_agent_state_t *next_state,
                              const gt_agent_state_t *curr_state);

  std::optional<gt_obj_sim_state_t> UpdateObjActionSimState(double obj_acc);

  void UpdateObjAllActionSimState();

  void IgnoreUnrealizedStrategy(bool *is_realizable_strategy,
                                double ego_strategy_acc,
                                double obj_strategy_acc);
  void UpdateDecisionResult(GT_Interaction_Data *inter_data);
  void UpdateGtSimStateByAction(GT_Interaction_Data *inter_data);

  void SelectedCostFuncUseScenario(
      gt_agent_state_t *obj_state, gt_ego_state_t *ego_state,
      const game_theory_scenario_t game_theory_scenario,
      const StBoundaryProto::DecisionType decision_type,
      const double ego_sampling_acc, const double obj_sampling_acc,
      const int step, const bool is_debug_cost);
  bool IsSimulationStop(const gt_ego_state_t *ego_state,
                        const gt_agent_state_t *obj_state);

  void ComputeExptAcc(double *acc, double start_vel, double end_vel,
                      double delta_dist);
  void CalcEgoMaxAccBySpeedLimit(double *ego_max_acc, const double ego_init_vel,
                                 const double ego_enter_collision_max_vel,
                                 const double ego_leave_collision_max_vel);
  void DumpCostMatrixSimple(const GT_Interaction_Matrix *gt_interaction_matrix);
};

}  // namespace planning
}  // namespace e2e_noa
#endif
