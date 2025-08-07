#pragma once

#include "common/spt/lat_planner/lat_planner_interface/constraint/constraint_common.h"
#include "common/spt/lat_planner/lat_planner_interface/cost/cost_common.h"
#include "common/spt/lat_planner/lat_planner_interface/data/spt_data.h"
#include "common/spt/lat_planner/lat_planner_interface/model/spt_lat_model.h"
#include "common/spt/lat_planner/lateral_opt.h"
#include "common/spt/solver/core.h"
#include "common/spt/solver/model_base.h"
#include "common/spt/solver/utils.h"
namespace e2e_noa {
namespace spt {

SOLVER_TYPES(STATE_SIZE, INPUT_SIZE)
using e2e_noa::spt::Constraint;
using e2e_noa::spt::CostBase;
using e2e_noa::spt::ILqr;
using e2e_noa::spt::SolverConfig;
struct OutputInfo {
  StateVec x_vec;
  ControlVec u_vec;
};

static constexpr double kConstraintExpandScale = 3.0;

class LatPlannerInterface {
  SOLVER_TYPES(STATE_SIZE, INPUT_SIZE)

 public:
  static LatPlannerInterface &get_instance() {
    static LatPlannerInterface instance;
    return instance;
  }
  ~LatPlannerInterface() = default;
  LatPlannerInterface(const LatPlannerInterface &) = delete;
  LatPlannerInterface &operator=(const LatPlannerInterface &) = delete;
  void Reset();
  void Init();
  void Update(std::vector<RefPoint> &ref_points, LatPlannerInput *input,
              LatPlannerOutput *output);

  void InitConstrainConfig();
  const std::vector<double> &GetTimeStep() const { return dt_vec_; }
  e2e_noa::spt::SolverConfig *GetSolverConfig() const {
    return solver_config_ptr_.get();
  }
  double GetTimeStep(size_t step) const { return dt_vec_.at(step); }
  void ResizeTimestep(const size_t size) { dt_vec_.resize(size); }
  void SetTimeStep(const double dt, const size_t step) {
    if (step >= dt_vec_.size()) {
      return;
    }
    dt_vec_[step] = dt;
  }

 private:
  LatPlannerInterface();
  void Solve(OutputInfo *output) const;

  void UpdateInitState(const TrajectoryPoint &init_point, State *x) const;
  void InitSolverConfig();
  void InitCostStack();
  void UpdateRefPathInfo(const RefPoint &ref_point, const size_t step) const;
  void UpdateRefPathSegmentInfo(const RefPoint &s, const RefPoint &e,
                                const size_t step) const;
  void UpdateConstantInfo(const LatPlannerInput &input, const size_t size);
  void UpdateStepInfo(const LatPlannerInput &input);
  void BuildCircleModel(const LatPlannerInput &input);

  void UpdateConstraintBound(const LatPlannerInput &input) const;
  inline size_t Horizon() const {
    return static_cast<unsigned long>(solver_config_ptr_->horizon);
  }
  inline bool IsDebugMode() const { return solver_config_ptr_->is_debug_mode; }
  inline size_t GetInputSize() const { return solver_config_ptr_->input_size; }
  inline size_t GetStateSize() const { return solver_config_ptr_->state_size; }

 public:
  void UpdateCostConfig(const std::vector<RefPoint> &ref_path,
                        const LatPlannerInput &input);
  void UpdateStepTime() const;

 private:
  StateVec x0_{};
  std::vector<double> dt_vec_{};
  std::unique_ptr<SolverConfig> solver_config_ptr_{nullptr};
  std::unique_ptr<ILqr<STATE_SIZE, INPUT_SIZE, CONSTRAINT_SIZE>> ilqr_core_ptr_{
      nullptr};
  std::unique_ptr<LatSptDataCache> spt_data_{nullptr};

  std::vector<std::unique_ptr<CostBase<STATE_SIZE, INPUT_SIZE>>> cost_vec_;
  std::vector<std::unique_ptr<Constraint<STATE_SIZE, INPUT_SIZE, 2>>>
      constraint_vec_;
  std::unique_ptr<LatCostCommon> cost_common_ptr_{nullptr};
  std::unique_ptr<SptLatModel> lat_model_{nullptr};

  OutputInfo output_{};
};
}  // namespace spt
}  // namespace e2e_noa