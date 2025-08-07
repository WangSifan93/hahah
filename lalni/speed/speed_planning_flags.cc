#include "speed/speed_planning_flags.h"

DEFINE_bool(planner_send_speed_optimizer_debug, false,
            "Whether to send speed optimizer debug.");

DEFINE_bool(planner_enable_moving_close_traj_speed_limit, true,
            "Whether to enable moving close object speed_limit.");

DEFINE_bool(planner_enable_right_turn_close_speed_limit, true,
            "Whether to enable right turn close close object speed_limit.");

DEFINE_bool(planner_enable_cross_blind_close_decision, true,
            "Whether to enable planner_enable_cross_blind_close_decision.");

DEFINE_bool(enable_lanechanging_limit, true,
            "Whether to enable lanechanging speed_limit.");
