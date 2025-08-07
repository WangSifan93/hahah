#pragma once

#include <algorithm>
#include <cmath>
#include <string>
#include <utility>
#include <vector>

#include "math/vec.h"
#include "object/spacetime_object_trajectory.h"
#include "plan/discretized_path.h"
#include "speed/mcts_game_process/mcts_data_type.h"

namespace e2e_noa::planning {

struct IntersectionResult {
  bool has_intersection = false;
  double dis_leader = 0.0;
  double dis_follower = 0.0;
  double t_leader_arrive = 0.0;
  double t_follower_arrive = 0.0;
};

bool UpdateVRUStatus(
    const AgentStatus &leader_status, const AgentStatus &vru_parent,
    const Vec2d &goal_position, const double goal_speed,
    const double goal_heading, const double original_v,
    const double current_time, const double time_step,
    const SpeedPlanningParamsProto::MCTSVruConfigProto &params,
    const StOverlapMetaProto::OverlapSource &overlap_source,
    const StBoundaryProto::ObjectType &vru_type,
    const std::vector<const SpacetimeObjectTrajectory *> &other_agents,
    const double follower_length, const double follower_width,
    const double heading_diff, AgentStatus &vru_child_status,
    const InteractionZone interaction_zone, const DiscretizedPath &ego_path,
    std::shared_ptr<SptDebug> spt_debug);

}  // namespace e2e_noa::planning
