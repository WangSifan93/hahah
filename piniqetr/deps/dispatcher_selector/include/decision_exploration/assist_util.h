#ifndef E2E_NOA_PLANNING_ASSIST_ASSIST_UTIL
#define E2E_NOA_PLANNING_ASSIST_ASSIST_UTIL

#include <utility>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "autonomy_state.pb.h"
#include "common/driving_map_topo.h"
#include "common/path_sl_boundary.h"
#include "lane_change.pb.h"
#include "maps/lane_path.h"
#include "plan/planner_semantic_map_manager.h"
#include "positioning.pb.h"
#include "alc.pb.h"
#include "router/plan_passage.h"
#include "speed_planning_params.pb.h"
#include "trajectory_point.pb.h"
#include "vehicle.pb.h"

namespace e2e_noa::planning {

class BoundaryInterval;
std::vector<BoundaryInterval> FindSolidBoundaryIntervals(
    const PlanPassage& plan_passage, const FrenetCoordinate& first_point_sl,
    double cutoff_s);

absl::StatusOr<bool> CrossedBoundary(const PlanPassage& dp,
                                     const Vec2d& ego_pos);

absl::StatusOr<bool> HasTrajectoryCrossedSolidBoundary(
    const PlanPassage& plan_passage, const PathSlBoundary& sl_boundary,
    const std::vector<ApolloTrajectoryPointProto>& traj_pts,
    const VehicleGeometryParamsProto& vehicle_geom, bool lc_pause);

absl::StatusOr<ALCState> UpdateAlcState(ALCState state,
                                         const Vec2d& preview_pos,
                                         const Vec2d& ego_pos,
                                         double ego_heading,
                                         const PlanPassage& plan_passage);
}  

#endif
