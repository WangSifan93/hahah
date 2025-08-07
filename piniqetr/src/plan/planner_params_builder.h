#ifndef ST_PLANNING_PLANNER_PARAMS_BUILDER
#define ST_PLANNING_PLANNER_PARAMS_BUILDER

#include "absl/status/statusor.h"
#include "planner_params.pb.h"
#include "vehicle.pb.h"

namespace e2e_noa {
namespace planning {

absl::StatusOr<PlannerParamsProto> BuildPlannerParams(
    const std::string& params_dir,
    const VehicleGeometryParamsProto& vehicle_geo_params);

}
}  

#endif
