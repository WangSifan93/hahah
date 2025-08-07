#ifndef AD_E2E_PLANNING_NODES_PLANNER_CITY_CITY_PLANNER_H
#define AD_E2E_PLANNING_NODES_PLANNER_CITY_CITY_PLANNER_H
#include "async/thread_pool.h"
#include "nodes/planner/planner.h"
#include "plan/main_planner.h"
#include "plan/planner_state.h"
#include "plan/planner_world.h"

#define ZARK_OUTPUT_TRAJ_SIZE (17)

namespace ad_e2e {

namespace planning {

class CityPlanner : public ad_e2e::planning::Planner {
 public:
  DECLARE_PTR(CityPlanner);

  CityPlanner(int num_workers);

  bool Init(const std::string& params_dir);

  virtual void Run(
      const ad_e2e::planning::PlanningInputFrame* input_frame) override;
  virtual void Reset() override;
  virtual std::shared_ptr<ad_e2e::planning::planning_result_type>
  GetPlanningResult() const override;

 private:
  void UpdatePlannerState(
      const e2e_noa::planning::PlannerWorldInput* planner_input,
      const e2e_noa::planning::PlannerWorldOutput* planner_output,
      const absl::Time predicted_plan_time);
  void PlanningResultPub(
      const ad_e2e::planning::PlanningInputFrame* f,
      const e2e_noa::planning::PlannerWorldInput* planner_input,
      const e2e_noa::planning::PlannerWorldOutput* planner_output);
  bool InitParams(const std::string& params_dir);
  bool ComputerFailCodeHoldCounter(
      const e2e_noa::planning::PlannerStatusProto::PlannerStatusCode&
          status_code) const;

  void AddTrafficLightInfo(const ad_e2e::planning::MapPtr& map,
                           zark::e2e_noa::PlanningResult& plan_res);

  e2e_noa::planning::E2EPlanner e2e_planner_;
  std::unique_ptr<e2e_noa::WorkerThreadManager> thread_pool_;
  std::unique_ptr<e2e_noa::PlannerParamsProto> planner_params_;
  std::unique_ptr<e2e_noa::VehicleParamsProto> vehicle_params_;

  e2e_noa::planning::PlannerStatus planner_status_;
  std::unique_ptr<e2e_noa::planning::PlannerState> planner_state_;

  std::shared_ptr<planning_result_type> planning_result_msg_;
  mutable uint8_t fail_number_ = 10;
};

}  // namespace planning

}  // namespace ad_e2e

#endif
