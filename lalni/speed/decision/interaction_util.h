

#ifndef PLANNER_SPEED_DECIDER_INTERACTION_UTIL_H_
#define PLANNER_SPEED_DECIDER_INTERACTION_UTIL_H_

#include <vector>

#include "object/planner_object.h"
#include "object/spacetime_object_trajectory.h"
#include "plan/discretized_path.h"
#include "speed/overlap_info.h"
#include "speed/speed_vector.h"
#include "speed_planning.pb.h"
#include "trajectory_point.pb.h"
#include "vehicle.pb.h"

namespace e2e_noa {
namespace planning {

struct TtcInfo {
  double ttc_upper_limit = 0.0;
  double ttc_lower_limit = 0.0;
};

TtcInfo GetAvOverlapTtcInformation(
    const std::vector<OverlapInfo>& overlap_infos,
    const StOverlapMetaProto& overlap_meta, const DiscretizedPath& path,
    const SpeedVector& preliminary_speed);

TtcInfo GetObjectOverlapTtcInfo(const OverlapInfo& fo_info,
                                const SpacetimeObjectTrajectory& spacetime_obj,
                                const std::optional<double>& reaction_time);

bool IsAvInObjectFov(const PathPoint& current_path_point,
                     const PlannerObject& planner_object,
                     const VehicleGeometryParamsProto& vehicle_geo_params);

bool IsAvCompletelyInObjectFov(
    const PathPoint& current_path_point, const PlannerObject& planner_object,
    const VehicleGeometryParamsProto& vehicle_geo_params);

bool IsObjectInAvFov(const PathPoint& current_path_point,
                     const PlannerObject& planner_object);

double CalcObjectYieldingTime(const std::vector<OverlapInfo>& overlap_infos,
                              const StOverlapMetaProto& overlap_meta,
                              const DiscretizedPath& path,
                              const SpeedVector& preliminary_speed);

bool IsParallelMerging(const StOverlapMetaProto& overlap_meta);

bool HasYieldingIntentionToFrontAv(
    const PathPoint& current_path_point, const PlannerObject& planner_object,
    const VehicleGeometryParamsProto& vehicle_geo_params,
    const StOverlapMetaProto& overlap_meta,
    const SpeedVector& preliminary_speed, double first_overlap_time);

bool IsAggressiveLeading(const SpeedVector& preliminary_speed,
                         double first_overlap_time);

}  // namespace planning
}  // namespace e2e_noa
#endif
