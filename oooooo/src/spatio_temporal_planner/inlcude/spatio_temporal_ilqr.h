#pragma once

#include "ilqr_model.h"
#include "lon_ilqr_data.pb.h"
#include "simplify_speed_planner.h"
#include "spatio_temporal_constraint.h"
#include "spatio_temporal_cost.h"
#include "spatio_temporal_model.h"
namespace e2e_noa {
namespace spt {

enum SolutionId : std::uint8_t {
  kPrePlan = 0,
  kForwardSim,
  kBackUpInit,
  kNormal,
  kSolutionSize
};

struct OutputInfo {
  StateVec x_vec;
  ControlVec u_vec;
  bool is_turn_circle{false};
  bool is_rb_collision{false};
  bool is_valid_traj{false};
};

static constexpr double kConstraintExpandScale = 5.0;

class ILQRInterface {
  using ProtoRefPoints = google::protobuf::RepeatedPtrField<RefPoint>;

 public:
  static ILQRInterface &get_instance() {
    static ILQRInterface instance;
    return instance;
  }
  ~ILQRInterface() = default;
  ILQRInterface(const ILQRInterface &) = delete;
  ILQRInterface &operator=(const ILQRInterface &) = delete;
  void Reset();
  void Init();
  void Update(const ProtoRefPoints &fcs_refpath,
              e2e_noa::sptLatMotionPlanInput *input,
              e2e_noa::sptLatMotionPlanOutput *output,
              std::vector<State> *dense_output);

  double GetTimeStep(size_t step) const { return dts_.at(step); }
  void ResizeTimestep(size_t size) { dts_.resize(size); }
  void SetTimeStep(const double dt, const size_t step) {
    if (step >= dts_.size()) {
      return;
    }
    dts_[step] = dt;
  }
  void UpdateSolverConfig(const ilqr_solver::SolverConfig &solver_config) {
    ilqr_solver_config_ptr_->mutable_ilqr_config()->set_max_iter(
        solver_config.ilqr_config().max_iter());
    ilqr_solver_config_ptr_->mutable_cilqr_config()->set_max_outer_iterations(
        solver_config.cilqr_config().max_outer_iterations());
    ilqr_solver_config_ptr_->set_enable_cilqr(solver_config.enable_cilqr());
    ilqr_solver_config_ptr_->set_is_debug_mode(solver_config.is_debug_mode());
  }

 private:
  ILQRInterface();
  void Solve(OutputInfo *output) const;
  int UpdateOutput(const std::vector<OutputInfo> &output_candidates,
                   const double wheel_base,
                   e2e_noa::sptLatMotionPlanOutput *output) const;
  void IsInteractState(
      ::google::protobuf::RepeatedPtrField<::micro_decider::DecisionInfo> dec,
      e2e_noa::sptLatMotionPlanOutput *output);
  void TransferOutput(const double wheel_base,
                      std::vector<OutputInfo> *output_candidates);
  void ParamsOnlyLaneKeep(const e2e_noa::sptLatMotionPlanInput &input);
  void TransferOutputToProtoData(const OutputInfo &out_info,
                                 const double wheel_base,
                                 e2e_noa::sptOutput *out_proto) const;
  void ClearAndResizeOutput(e2e_noa::sptOutput *output) const;
  void UpdateInitState(const State &init_state, State *x) const;
  void InitSolverConfig();
  void InitCost();
  inline size_t Horizon() const {
    return static_cast<unsigned long>(
        ilqr_solver_config_ptr_->ilqr_config().horizon());
  }
  inline bool EnableCilqr() const {
    return ilqr_solver_config_ptr_->enable_cilqr();
  }
  inline bool EnableCilqrTest() const {
    return ilqr_solver_config_ptr_->cilqr_config().enable_constraint_test();
  }
  inline bool IsDebugMode() const {
    return ilqr_solver_config_ptr_->is_debug_mode();
  }
  inline size_t GetInputSize() const {
    return static_cast<unsigned long>(
        ilqr_solver_config_ptr_->ilqr_config().data_input());
  }
  inline size_t GetStateSize() const {
    return static_cast<unsigned long>(
        ilqr_solver_config_ptr_->ilqr_config().state());
  }

  void GenInitSeed(const e2e_noa::sptLatMotionPlanInput &input,
                   OutputInfo *output) const;
  void GetOutputFromControl(const ControlVec &u_vec, OutputInfo *output) const;
  void UpdateWarmStartStatus(const e2e_noa::sptLatMotionPlanInput &input);
  void SoftBoundParams(const e2e_noa::sptLatMotionPlanInput &input);
  void FixParams(const e2e_noa::sptLatMotionPlanInput &input);
  void StepRelatedParams(const e2e_noa::sptLatMotionPlanInput &input);
  void ConsistencyTimeParams(const e2e_noa::sptLatMotionPlanInput &input);
  double ClampDirection(const double buff, const double center,
                        const double target) const;
  void BoundInfo(const micro_decider::DecisionInfo &bnd,
                 const RefPoint &ref_point, const double init_theta,
                 const size_t step) const;
  void CoarseRefpathInfo(const RefPoint &ref_point, const double init_theta,
                         const size_t step) const;
  void CoarseRefpathSegInfo(const RefPoint &start_pt, const RefPoint &end_pt,
                            const size_t step) const;
  void GenDenseTraj(const double wheel_base, const StateVec &xs,
                    const ControlVec &us, std::vector<State> *dense_xs) const;

  template <typename T>
  void AddConstraint(
      std::vector<ilqr_solver::BoundConstraint<state, data_input>>
          *constraint_vec,
      double tolerance);

 public:
  void UpdateCostConfig(const ProtoRefPoints &fcs_refpath,
                        const e2e_noa::sptLatMotionPlanInput &input);
  void ClearCandidatesOuput(std::vector<OutputInfo> *output_candidates) const;
  void RestHardBoundScaleCost();
  void GetCarCircle(const e2e_noa::sptLatMotionPlanInput &input);
  void HardBoundScaleAndShiftRefCenteringCost(
      const e2e_noa::sptLatMotionPlanInput &input) const;
  void ExtraContext(const double wheel_base) const;

 private:
  StateVec x0_{};
  std::vector<double> dts_{};
  std::vector<OutputInfo> output_candidates_{};

  std::unique_ptr<ilqr_solver::SolverConfig> ilqr_solver_config_ptr_{nullptr};
  std::unique_ptr<ilqr_solver::ILqr<state, data_input, constraint>>
      ilqr_solver_{nullptr};
  std::unique_ptr<SptDataCache> data_cache_{nullptr};

  std::vector<std::unique_ptr<ilqr_solver::BaseCostTerm<state, data_input>>>
      cost_stack_vec_;
  std::vector<std::unique_ptr<ilqr_solver::Constraint<state, data_input, 2>>>
      constraint_vec_;
  std::unique_ptr<LatIlqrCommonTerm> common_term_calculator_{nullptr};
  std::unique_ptr<ILqrLatModel> lat_model_{nullptr};
  ControlVec ssp_result_u_{};
  StateVec ssp_result_x_{};
};

}  // namespace spt
}  // namespace e2e_noa