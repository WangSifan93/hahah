#ifndef PLANNER_SPEED_GAME_THEORY_COMMON_H_
#define PLANNER_SPEED_GAME_THEORY_COMMON_H_

#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

#include "../interactive_agent.h"
#include "absl/status/status.h"
#include "async/thread_pool.h"
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

#define MAX_STATE_NUM (50)
#define MAX_GT_ACC_NUM (50)

typedef struct GT_Ego_State {
  int traj_point_idx;
  TrajectoryPointWithAcceleration sl_center_pose;
  FrenetBox frenet_box;
  frenet_vel_t sl_vel;
  TrajectoryPointWithAcceleration global_center_pose;
  Polygon2d global_polygon;

  double vel;
  double acc;
  double theta;
  double time_stamp;
  double dist = 0.0;
  double wait_time = 0.0;

  double safety_cost;
  double law_cost;
  double acc_cost;
  bool is_legal = true;
} gt_ego_state_t;

typedef struct GT_Ego_Info {
  double max_acc;
  double min_acc;
  double acc_interval;
  double max_vel;
  double safety_cost;
  double law_cost;
  double acc_cost;
  double total_cost;
  double average_cost;

  int size;
  std::vector<gt_ego_state_t*> states;

  void ResetGtEgoInfo();
  bool is_legal = true;

} gt_ego_info_t;

typedef struct GT_Agent_State {
  int traj_point_idx;
  TrajectoryPointWithAcceleration sl_center_pose;
  FrenetBox frenet_box;
  frenet_vel_t sl_vel;
  TrajectoryPointWithAcceleration global_center_pose;
  Polygon2d global_polygon;

  double vel;
  double acc;
  double theta;
  double time_stamp;
  double dist = 0.0;
  double wait_time = 0.0;

  double safety_cost;
  double law_cost;
  double acc_cost;
  bool is_legal = true;
} gt_agent_state_t;

typedef struct GT_Agent_Info {
  const InteractiveAgent* obstacle_;
  double max_acc;
  double min_acc;
  double acc_interval;
  double max_vel;

  double safety_cost;
  double law_cost;
  double acc_cost;
  double total_cost;
  double average_cost;

  Polygon2d base_polygon;

  int size;
  std::vector<gt_agent_state_t*> states;

  void ResetGtAgentInfo();
  bool is_legal = true;

} gt_agent_info_t;

typedef struct GtEgoSimState {
  std::vector<gt_ego_state_t> states;
} gt_ego_sim_state_t;

typedef struct GtObjSimState {
  std::vector<gt_agent_state_t> states;
} gt_obj_sim_state_t;

struct GT_Interaction_Data {
  double ego_acc;
  double agent_acc;

  bool is_success;
  bool is_collision;

  gt_agent_info_t obj_info;
  gt_ego_info_t ego_info;
  StBoundaryProto::DecisionType decision_type;
};

}  // namespace planning
}  // namespace e2e_noa

#endif
