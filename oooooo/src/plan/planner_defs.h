#ifndef ONBOARD_PLANNER_PLANNER_DEFS_H_
#define ONBOARD_PLANNER_PLANNER_DEFS_H_

#include "absl/time/time.h"
#include "math/util.h"

namespace e2e_noa {
namespace planning {

#ifdef __X9HP__
constexpr int kTrajectorySteps = 80;
constexpr int kInitializationTrajectorySteps = 80;
constexpr int kInitializationCostEvalStep = 3;
constexpr double kPlanningTimeHorizon = 13.0;
#else
constexpr int kTrajectorySteps = 80;
constexpr int kInitializationTrajectorySteps = 80;
constexpr int kInitializationCostEvalStep = 3;
constexpr double kPlanningTimeHorizon = 16.0;
#endif

constexpr absl::Duration kPlannerMainLoopInterval = absl::Milliseconds(100);

constexpr double kTrajectoryTimeStep = 0.1;
constexpr double kTrajectoryTimeHorizon =
    (kTrajectorySteps - 1) * kTrajectoryTimeStep;
constexpr double kMaxLatAccCheckTime = 3.0;
constexpr double kInitializationTrajectoryTimeHorizon =
    (kInitializationTrajectorySteps - 1) * kTrajectoryTimeStep;

constexpr int kAccTrajectorySteps = 80;
constexpr double kAccTrajectoryTimeHorizon =
    (kAccTrajectorySteps - 1) * kTrajectoryTimeStep;
constexpr double kSpacetimePlannerTrajectoryHorizon =
    kTrajectoryTimeStep * kTrajectorySteps;

constexpr double kSpacetimePlannerBehindCarTrajectoryHorizon = 1.0;
constexpr double kSpacetimePlannerVRUTrajectoryHorizon = 2.0;

constexpr double kSpacetimePlannerInCrossingTrajectoryHorizon = 3.0;

constexpr double kPathSampleInterval = 0.2;
constexpr int kSpeedPlanningMaxTrajectorySteps = 80;

constexpr int kMaxPastPointNum = 50;
constexpr int kMaxAccPastPointNum = 5;

constexpr double kSpaceTimeVisualizationDefaultTimeScale = 10.0;

constexpr double kMinLCSpeed = 5.0 / 3.6;
constexpr double kMinLcLaneLength = 20.0;

constexpr double kDefaultLaneWidth = 3.5;
constexpr double kDefaultHalfLaneWidth = 0.5 * kDefaultLaneWidth;
constexpr double kMaxHalfLaneWidth = 2.7;
constexpr double kMinLaneWidth = 2.6;
constexpr double kMinHalfLaneWidth = kMinLaneWidth * 0.5;

constexpr double kMaxLateralOffset = 10.0;
constexpr double kMaxLaneKeepLateralOffset = 0.4;

constexpr double kRouteStationUnitStep = 1.0;

constexpr double kPlanPassageKeepBehindLength = 50.0;

constexpr double kLaneChangeCheckForwardLength = 180.0;
constexpr double kLaneChangeCheckBackwardLength = 100.0;

constexpr double kMaxTravelDistanceBetweenFrames = 200.0;

constexpr double kSpacetimeSearchMinFollowDistance = 3.0;

constexpr double kPlannerLaneGraphLength = 200.0;

constexpr double kCurvatureLimitRange = 6.0;

constexpr double kUTurnCurbGain = 0.3;

constexpr double kCurbGain = 1.0;

constexpr double kNormalToVirtual = 20.0;
constexpr double kVirtualToNormal = 50.0;

const char SoftNameString[] = "Soft";
const char HardNameString[] = "Hard";

constexpr double kAlternateRouteAllowRoundaboutDist = 2000.0;

const std::string kInvalidOnlineMapId = "";

constexpr int kAsyncCounterInitVal = -1;

constexpr bool kSimEnable = false;
}  // namespace planning
}  // namespace e2e_noa

#endif
