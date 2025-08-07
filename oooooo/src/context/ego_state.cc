#include "context/ego_state.h"

#include "math/geometry/util.h"
#include "util/vehicle_geometry_util.h"

namespace e2e_noa::planning {
EgoState::EgoState() {}

void EgoState::update(const PlannerWorldInput &input) {
  start_point_ = input.start_point_info->start_point;
  start_point_velocity_ = start_point_.v();
  ego_pos_ =
      Extract2dVectorFromApolloProto(input.start_point_info->start_point);
  vehicle_geometry_ = input.vehicle_params->vehicle_geometry_params();

  ego_theta_ = input.start_point_info->start_point.path_point().theta();

  ego_box_ = ComputeAvBox(ego_pos_, ego_theta_, vehicle_geometry_);
}
}  // namespace e2e_noa::planning