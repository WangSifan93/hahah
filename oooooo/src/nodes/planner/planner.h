#ifndef AD_E2E_PLANNING_NODES_PLANNER_PLANNER_H
#define AD_E2E_PLANNING_NODES_PLANNER_PLANNER_H
#include "common/input_frame.h"
#include "common/planning_macros.h"

namespace ad_e2e {
namespace planning {

class Planner {
 public:
  DECLARE_PTR(Planner);

  virtual ~Planner() = default;

  virtual void Run(const PlanningInputFrame* input_frame) = 0;
  virtual void Reset() = 0;
  virtual std::shared_ptr<planning_result_type> GetPlanningResult() const = 0;
};

}  // namespace planning
}  // namespace ad_e2e

#endif
