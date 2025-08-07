#ifndef ONBOARD_PLANNER_ROUTER_ROUTE_SECTIONS_UTIL_H_
#define ONBOARD_PLANNER_ROUTER_ROUTE_SECTIONS_UTIL_H_

#include <vector>

#include "absl/status/statusor.h"
#include "maps/lane_path.h"
#include "maps/semantic_map_defs.h"
#include "math/vec.h"
#include "plan/composite_lane_path.h"
#include "plan/planner_defs.h"
#include "plan/planner_semantic_map_manager.h"
#include "router/route_sections.h"
#include "router/route_sections_info.h"

namespace e2e_noa::planning {

struct PointOnRouteSections {
  double accum_s;
  int section_idx;
  double fraction;
  mapping::ElementId lane_id;
};

absl::StatusOr<RouteSections> AlignRouteSections(
    const RouteSections& global_sections, const RouteSections& local_sections);

absl::StatusOr<RouteSections> SpliceRouteSections(
    const RouteSections& origin_sections, const RouteSections& target_sections);

absl::StatusOr<RouteSections> AppendRouteSectionsToTail(
    const RouteSections& origin_sections, const RouteSections& global_sections);

RouteSections RouteSectionsFromCompositeLanePath(
    const ad_e2e::planning::Map& smm, const CompositeLanePath& clp);

RouteSections RouteSectionsFromCompositeLanePath(
    const PlannerSemanticMapManager& psmm, const CompositeLanePath& clp);

absl::StatusOr<PointOnRouteSections>
FindSmoothPointOnRouteSectionsByLateralOffset(
    const PlannerSemanticMapManager& psmm,
    const RouteSectionsInfo& sections_info, const Vec2d& query_point,
    double lat_dist_thres = kMaxHalfLaneWidth);

absl::StatusOr<mapping::LanePath>
FindClosestLanePathOnRouteSectionsToSmoothPoint(
    const PlannerSemanticMapManager& psmm, const RouteSections& sections,
    const Vec2d& query_point, double* proj_s = nullptr);

absl::StatusOr<RouteSections> ClampRouteSectionsBeforeArcLength(
    const PlannerSemanticMapManager& psmm,
    const RouteSections& raw_route_sections, double len);

absl::StatusOr<RouteSections> ClampRouteSectionsAfterArcLength(
    const PlannerSemanticMapManager& psmm,
    const RouteSections& raw_route_sections, double len);

absl::StatusOr<std::vector<mapping::LanePath>>
CollectAllLanePathOnRouteSectionsFromStart(
    const PlannerSemanticMapManager& psmm, const RouteSections& route_sections);

absl::StatusOr<std::vector<mapping::LanePath>>
CollectAllLanePathOnRouteSections(const PlannerSemanticMapManager& psmm,
                                  const RouteSections& route_sections);

RouteSections BackwardExtendRouteSections(const PlannerSemanticMapManager& psmm,
                                          const RouteSections& raw_sections,
                                          double extend_len);

absl::StatusOr<RouteSections> BackwardExtendRouteSectionsFromPos(
    const PlannerSemanticMapManager& psmm, const RouteSections& raw_sections,
    const Vec2d& pos, double extend_len);

absl::StatusOr<mapping::LanePath> ForwardExtendLanePathOnRouteSections(
    const PlannerSemanticMapManager& psmm, const RouteSections& route_sections,
    const mapping::LanePath& raw_lane_path, double extend_len);

absl::StatusOr<mapping::LanePath> BackwardExtendLanePathOnRouteSections(
    const PlannerSemanticMapManager& psmm, const RouteSections& route_sections,
    const mapping::LanePath& raw_lane_path, double extend_len);

absl::StatusOr<mapping::LanePath> FindClosestTargetLanePathOnReset(
    const PlannerSemanticMapManager& psmm, const RouteSections& prev_sections,
    const Vec2d& ego_pos);

}  // namespace e2e_noa::planning

#endif
