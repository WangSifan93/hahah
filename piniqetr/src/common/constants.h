#ifndef PLANNING_COMMON_CONSTANTS_H
#define PLANNING_COMMON_CONSTANTS_H
#include <string>
namespace ad_e2e {
namespace planning {
namespace Constants {

static constexpr double ZERO = 1e-5;
static constexpr double SPEED_MIN = 0.01;
static constexpr double SOFT_ACC = 0.5;
static constexpr double SOFT_BRAKE = -0.5;
static constexpr double PROPER_ACC = 1.0;
static constexpr double PROPER_BRAKE = -1.0;
static constexpr double MRM_BRAKE = -1.5;
static constexpr double OBSTACLE_MAX_SPEED = 40.0;
static constexpr double OBSTACLE_TIME_BUFFER = 3.0;
static constexpr double MIN_HALF_LANE_WIDTH = 1.5;
static constexpr double DEFAULT_LANE_WIDTH = 3.75;
static constexpr double TRAJECTORY_MIN_LENGTH = 5.0;
static constexpr double LANE_KEEPING_THRESHOLD = 0.3;
static constexpr double OPPOSITE_SPEED = -2.0;
static constexpr int MAX_THREAD_NUM = 3;

static constexpr double MATH_PI = 3.1415926535897932384626433832795028841971693;
static constexpr double DEG2RAD = MATH_PI / 180;
static constexpr double RAD2DEG = 180 / MATH_PI;
static constexpr double MPS2KPH = 3.6;
static constexpr double KPH2MPS = 1.0 / 3.6;

static constexpr double LOCALIZATION_HISTORY_LENGTH = 0.5;

static constexpr double PLANNING_PREVIEW_TIME = 0.1;
static constexpr double PLANNING_MAX_LAT_DIS = 0.5;

static const std::string VIRTUAL_LANE_ID = "virtual_lane";
static const std::string VIRTUAL_LANE_ID_WITH_MAP = "virtual_lane_with_map";

static constexpr int MSG_TIMEOUT_COUNTER = 3;
static constexpr int PLAN_FAIL_COUNTER = 6;

}  // namespace Constants
}  // namespace planning
}  // namespace ad_e2e
#endif
