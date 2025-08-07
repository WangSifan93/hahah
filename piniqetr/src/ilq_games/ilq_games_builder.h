///////////////////////////////////////////////////////////////////////////
//
// Base class for all multi-player dynamical systems. Supports (discrete-time)
// linearization and integration.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef ILQ_GAMES_BUILDER_H
#define ILQ_GAMES_BUILDER_H

#include "absl/types/span.h"
#include "common/path_sl_boundary.h"
#include "constraint.pb.h"
#include "object/spacetime_object_trajectory.h"
#include "object/spacetime_planner_object_trajectories.h"
#include "object/spacetime_trajectory_manager.h"
#include "plan/planner_semantic_map_manager.h"
#include "planner_object.pb.h"
#include "planner_params.pb.h"
#include "router/plan_passage.h"
#include "trajectory_point.pb.h"
#include "util/hmi_content_util.h"
#include "vehicle.pb.h"

#include <vector>

namespace e2e_noa::planning {

enum class LaneSemantic {
  NONE = 0,
  ROAD = 1,
  INTERSECTION_LEFT_TURN = 2,
  INTERSECTION_RIGHT_TURN = 3,
  INTERSECTION_STRAIGHT = 4,
  INTERSECTION_UTURN = 5
};

struct RouteLaneInfo {
  ad_e2e::planning::LaneConstPtr lane = nullptr;
  double start_fraction = 0.0;
  double end_fraction = 1.0;
};
struct ILQGAMESBuilderInput {
  const PlannerSemanticMapManager *psmm;
  const PlanPassage *passage;
  const LaneChangeStage &lc_stage;
  const VehicleGeometryParamsProto *veh_geom;
  const ApolloTrajectoryPointProto *plan_start_point;
  const SpacetimeTrajectoryManager *st_traj_mgr;
  const ILQGamesParamsProto *ilq_games_params;
  const uint64_t seq_num = 0;
};

SpacetimeTrajectoryManager BuildILQGames(const ILQGAMESBuilderInput &input);

} // namespace e2e_noa::planning

#endif
