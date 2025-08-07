#include "mcts_game_manager.h"

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

#include "absl/container/flat_hash_map.h"
#include "absl/container/flat_hash_set.h"
#include "absl/meta/type_traits.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/time/clock.h"
#include "absl/time/time.h"
#include "absl/types/span.h"
#include "async/async_util.h"
#include "async/parallel_for.h"
#include "common/timer.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "lon_game_manager.h"
#include "math/fast_math.h"
#include "math/frenet_common.h"
#include "math/frenet_frame.h"
#include "math/geometry/box2d.h"
#include "math/geometry/halfplane.h"
#include "math/geometry/polygon2d.h"
#include "math/piecewise_linear_function.h"
#include "math/util.h"
#include "math/vec.h"
#include "mcts_longitudinal.h"
#include "mcts_spatiotemporal.h"
#include "object/planner_object.h"
#include "object/spacetime_object_state.h"
#include "plan/planner_defs.h"
#include "plan/speed_profile.h"
#include "plan/trajectory_util.h"
#include "prediction/predicted_trajectory.h"
#include "prediction/prediction_defs.h"
#include "speed/decision/st_boundary_modifier_util.h"
#include "speed/empty_road_speed.h"
#include "speed/overlap_info.h"
#include "speed/speed_point.h"
#include "speed/st_boundary.h"
#include "speed/st_graph_data.h"
#include "speed/st_point.h"
#include "speed_planning_params.pb.h"
#include "trajectory_point.pb.h"
#include "util/status_macros.h"
#include "util/vehicle_geometry_util.h"
#include "vru_game_manager.h"

namespace e2e_noa::planning {

void ProcessMCTSGame(
    const std::vector<InteractiveInput> &interactive_input_infos,
    const SpacetimeTrajectoryManager &st_traj_mgr, const DiscretizedPath &path,
    const double av_velocity, const double av_acc, const uint64_t seq_num,
    const SpeedPlanningParamsProto *params, WorkerThreadManager *thread_pool,
    std::vector<MCTSInteractiveResult> &interactive_results) {
  if (params == nullptr) {
    return;
  }
  interactive_results.clear();
  const int interactive_obj_num = interactive_input_infos.size();
  std::vector<MCTSInteractiveResult> mcts_result_for_workers(
      interactive_obj_num);

  ParallelFor(
      0, interactive_obj_num, thread_pool,
      [&](parallel_for::WorkerIndex worker_index, int i) {
        const auto &input_info = interactive_input_infos[i];
        if (input_info.boundary_with_decision != nullptr &&
            input_info.boundary_with_decision->traj_id().has_value() &&
            input_info.boundary_with_decision->st_boundary() != nullptr) {
          const auto traj_id = input_info.boundary_with_decision->traj_id();
          const auto *spacetime_obj = st_traj_mgr.FindTrajectoryById(*traj_id);
          if (spacetime_obj != nullptr) {
            auto obj_scenario = input_info.boundary_with_decision->st_boundary()
                                    ->obj_scenario_info();
            if (input_info.is_hrvo && params->enable_mcts_vru()) {
              ProcessMCTSVruGame(
                  *input_info.boundary_with_decision, *spacetime_obj, path,
                  av_velocity, av_acc, obj_scenario, traj_id.value(),
                  &params->mcts_vru_params(), mcts_result_for_workers[i]);
            } else {
              if (params->has_mcts_lon_params() &&
                  params->mcts_lon_params().enable_lon_mcts_game()) {
                ProcessMCTSLonGame(
                    *input_info.boundary_with_decision, *spacetime_obj, path,
                    av_velocity, av_acc, obj_scenario, traj_id.value(),
                    &params->mcts_lon_params(), mcts_result_for_workers[i]);
              }
            }
          }
        }
      });
  interactive_results.reserve(interactive_obj_num);
  for (auto &single_worker : mcts_result_for_workers) {
    if (!single_worker.mcts_result.empty()) {
      interactive_results.emplace_back(std::move(single_worker));
    }
  }
}
}  // namespace e2e_noa::planning
