#include "common/spt/lat_planner/lateral_opt.h"

#include "common/spt/lat_planner/lat_planner_interface/lat_planner_interface.h"
namespace e2e_noa {
namespace spt {
void RunSptTest() {
  std::vector<RefPoint> ref_path;

  LatPlannerInput input;
  LatPlannerOutput output;
  auto& lat_planner = e2e_noa::spt::LatPlannerInterface::get_instance();
  const auto solver_config = *lat_planner.GetSolverConfig();
  lat_planner.Reset();
  lat_planner.ResizeTimestep(solver_config.horizon + 1);
  for (int i = 0; i < solver_config.horizon; ++i) {
    lat_planner.SetTimeStep(0.2, i);
  }
  lat_planner.Update(ref_path, &input, &output);
}
}  // namespace spt
}  // namespace e2e_noa