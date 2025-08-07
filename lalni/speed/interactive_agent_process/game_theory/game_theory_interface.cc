#include "game_theory_interface.h"

#include <algorithm>
#include <cmath>
#include <initializer_list>
#include <iterator>
#include <limits>
#include <optional>
#include <ostream>
#include <string>
#include <string_view>
#include <utility>

#include "../interactive_agent.h"
#include "absl/container/flat_hash_map.h"
#include "absl/container/flat_hash_set.h"
#include "absl/meta/type_traits.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/time/clock.h"
#include "absl/time/time.h"
#include "absl/types/span.h"
#include "common/timer.h"
#include "game_theory_environment.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "math/fast_math.h"
#include "math/frenet_common.h"
#include "math/frenet_frame.h"
#include "math/geometry/box2d.h"
#include "math/geometry/halfplane.h"
#include "math/geometry/polygon2d.h"
#include "math/intelligent_driver_model.h"
#include "math/piecewise_linear_function.h"
#include "math/util.h"
#include "math/vec.h"
#include "object/planner_object.h"
#include "object/spacetime_object_state.h"
#include "plan/planner_defs.h"
#include "plan/speed_profile.h"
#include "plan/trajectory_util.h"
#include "plan/trajectorypoint_with_acceleration.h"
#include "prediction/predicted_trajectory.h"
#include "prediction/prediction_defs.h"
#include "speed/decision/interaction_util.h"
#include "speed/decision/post_st_boundary_modifier.h"
#include "speed/decision/pre_brake_decision.h"
#include "speed/decision/st_boundary_modifier_util.h"
#include "speed/empty_road_speed.h"
#include "speed/gridded_svt_graph.h"
#include "speed/overlap_info.h"
#include "speed/speed_point.h"
#include "speed/st_boundary.h"
#include "speed/st_graph_data.h"
#include "speed/st_point.h"
#include "speed_planning_params.pb.h"
#include "trajectory_point.pb.h"
#include "util/status_macros.h"
#include "util/vehicle_geometry_util.h"

namespace e2e_noa::planning {

void reset_game_theory_interaction_result(gt_result_t* gt_result) {
  if (nullptr == gt_result) return;

  gt_result->is_valid = false;
  gt_result->is_solution = false;
  gt_result->gt_interaction_matrix.col = 0;
  gt_result->gt_interaction_matrix.row = 0;
}

void GameTheoryEntry(std::vector<gt_result_t>& gt_results,
                     std::vector<InteractiveAgent>& interactive_agents,
                     InteractiveAgent& ego_interactive_info) {
  std::vector<Game_Theory_Interaction_Environment> gt_env;

  for (int i = 0; i < interactive_agents.size(); i++) {
    Game_Theory_Interaction_Environment tmp_gt_env;
    gt_env.emplace_back(tmp_gt_env);
  }

  for (int i = 0; i < interactive_agents.size(); i++) {
    auto& curr_gt_env = gt_env[i];
    curr_gt_env.InitGameTheoryEnvironment(&ego_interactive_info,
                                          &interactive_agents[i]);
    curr_gt_env.BuildUpGameEnvironment();
    gt_result_t tmp_gt_result;
    curr_gt_env.StartStackelbergGameProcess(&tmp_gt_result);
    gt_results.emplace_back(tmp_gt_result);
  }
}
}  // namespace e2e_noa::planning
