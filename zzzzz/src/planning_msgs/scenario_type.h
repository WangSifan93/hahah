#pragma once

namespace zark {
namespace planning {
enum ScenarioType { LANE_FOLLOW_V2 = 1 };

// StageType is a superset of stages from all scenarios.
// It is created to keep different scenarios have uniform config interface
enum StageType { NO_STAGE = 0, LANE_FOLLOW_V2_DEFAULT_STAGE = 1 };

}  // namespace planning
}  // namespace zark
