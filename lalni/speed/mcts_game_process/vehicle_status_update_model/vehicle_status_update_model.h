
#ifndef PLANNER_SPEED_MCTS_VEHICLE_STATUS_UPDATE_MODEL_H_
#define PLANNER_SPEED_MCTS_VEHICLE_STATUS_UPDATE_MODEL_H_
#include <gtest/gtest.h>

#include "../mcts_data_type.h"
#include "math/double.h"
#include "vehicle_simulation_model.h"
namespace e2e_noa {
namespace planning {

bool UpdateFollowerState(
    const MCTSNodeConstPtr &parent, const double leader_time,
    const double time_step, const double follower_cur_v,
    const SpeedPlanningParamsProto::VehicleStatusUpdateModelParamsProto &params,
    VehicleSimulationModel &follower_model, NodeState &state,
    const std::shared_ptr<LonDebug> &lon_debug);

}
}  // namespace e2e_noa
#endif
