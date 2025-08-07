#ifndef AD_E2E_PLANNING_NODES_E2E_PLANNING_CORE_H
#define AD_E2E_PLANNING_NODES_E2E_PLANNING_CORE_H

#include <chrono>
#include <memory>

#include "common/gflags.h"
#include "common/input_frame.h"
#include "common/planning_macros.h"
#include "nodes/planner/city/city_planner.h"
#include "planning_config.pb.h"

namespace ad_e2e {
namespace planning {

class E2EPlanningCore {
 public:
  DECLARE_PTR(E2EPlanningCore);
  E2EPlanningCore() = default;
  virtual ~E2EPlanningCore() = default;

  bool Init();
  bool Exit();

  std::shared_ptr<planning_result_type> PlanningCallback(
      const std::pair<PlanResult, PlanningInputFrame::Ptr>& input_frame);

  bool planning_enabled() const { return planning_enabled_; }
  void set_planning_enabled(bool b) { planning_enabled_ = true; }

 private:
  bool InitGflags() const;
  e2e_noa::planning::planning_config::PlanningConfig planning_config_;

  ad_e2e::planning::CityPlanner::UPtr city_planner_;

  bool is_city_{true};
  bool planning_enabled_{true};
};

}  // namespace planning
}  // namespace ad_e2e

#endif
