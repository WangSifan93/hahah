#include "spatio_temporal_ilqr.h"

#include <cmath>
#include <vector>

#include "ilqr_model.h"
#include "lon_ilqr_data.pb.h"
#include "motion_plan_common.pb.h"
#include "simplify_path_planner.h"
#include "spatio_temporal_constraint.h"
#include "spatio_temporal_cost.h"
#include "spatio_temporal_data.h"
#include "spatio_temporal_model.h"
namespace {
static constexpr double kConstraintHorizonRatio = 0.25;
static constexpr double kConstraintDefault = 10.0;
static constexpr double kFurtherConstraintRatio = 0.8;
static constexpr double kLonAccBndBuffer = 0.5;
static constexpr double kLonAccLimitBuffer = 0.3;
static constexpr double kVelBndBuffer = 0.3;
constexpr int kScaleDecayIndex = 20;
constexpr double kScaleDecaySRange = 50.0;
}  // namespace
namespace e2e_noa {
namespace spt {

ILQRInterface::ILQRInterface() { Init(); }
void ILQRInterface::Init() {
  data_cache_ = std::make_unique<SptDataCache>();
  ilqr_solver_ =
      std::make_unique<ilqr_solver::ILqr<state, data_input, constraint>>();
  common_term_calculator_ =
      std::make_unique<LatIlqrCommonTerm>(data_cache_.get());
  lat_model_ = std::make_unique<ILqrLatModel>(*data_cache_);
  InitSolverConfig();
  ilqr_solver_->Init(lat_model_.get(), ilqr_solver_config_ptr_.get(),
                     common_term_calculator_.get());
  InitCost();
}

void ILQRInterface::InitCost() {
  costs.emplace_back(std::make_unique<RefpathCostT>(*data_cache_));
  costs.emplace_back(std::make_unique<HeadingCost>(*data_cache_));
  costs.emplace_back(std::make_unique<SRefCost>(*data_cache_));
  costs.emplace_back(std::make_unique<VRefCost>(*data_cache_));
  costs.emplace_back(std::make_unique<ARefCost>(*data_cache_));
  costs.emplace_back(std::make_unique<VLimitCost>(*data_cache_));
  costs.emplace_back(std::make_unique<CurvatureCost>(*data_cache_));
  costs.emplace_back(std::make_unique<CurvatureRefCost>(*data_cache_));
  costs.emplace_back(std::make_unique<DkappaCost>(*data_cache_));
  costs.emplace_back(std::make_unique<JerkCost>(*data_cache_));
  costs.emplace_back(std::make_unique<DdkappaCost>(*data_cache_));
  costs.emplace_back(std::make_unique<DjerkCost>(*data_cache_));
  costs.emplace_back(std::make_unique<LatAccCost>(*data_cache_));
  costs.emplace_back(std::make_unique<LatJerkCost>(*data_cache_));
  costs.emplace_back(std::make_unique<LonAccCost>(*data_cache_));
  costs.emplace_back(std::make_unique<LonJerkCost>(*data_cache_));
  costs.emplace_back(std::make_unique<ConsistencyCost>(*data_cache_));
  costs.emplace_back(std::make_unique<RoadBoundatyHardCost>(*data_cache_));
  costs.emplace_back(std::make_unique<RoadBoundatySoftCost>(*data_cache_));
  costs.emplace_back(std::make_unique<ObstacleSoftCost>(*data_cache_));
  costs.emplace_back(std::make_unique<ObstacleHardCost>(*data_cache_));

  for (const auto &cost_ptr : costs) {
    ilqr_solver_->AddCost(cost_ptr.get());
  }
}

void ILQRInterface::Reset() {
  data_cache_->Reset();
  ilqr_solver_->Reset();
  ResizeAndResetEigenVec(x0_, Horizon() + 1U, GetStateSize());
}

template <typename T>
void ILQRInterface::AddConstraint(
    std::vector<ilqr_solver::BoundConstraint<state, data_input>>
        *constraint_vec,
    double tolerance) {
  (void)constraint_vec_.emplace_back(std::make_unique<T>(*data_cache_));
  auto constraint = ilqr_solver::BoundConstraint<state, data_input>();
  constraint.Init(constraint_vec_.back().get(), tolerance,
                  ilqr_solver_config_ptr_.get());
  constraint_vec->emplace_back(constraint);
}

void ILQRInterface::InitConstrainConfig(
    const SolverConfig *const /*ilqr_solver_config_ptr*/) {
  static constexpr double kKappaTolerance = 1e-3;
  static constexpr double kDKappaTolerance = 1e-3;
  static constexpr double kAccelTolerance = 1e-1;
  static constexpr double kJerklTolerance = 1e-1;
  static constexpr double kVlowerTolerance = 2e-1;

  for (size_t step = 0U; step < Horizon() + 1U; step++) {
    std::vector<BoundConstraint<state, data_input>> constraint_vec;
    constraint_vec.reserve(ILQRCostraintId::constraint);

    AddConstraint<CurvatureConstraint>(&constraint_vec, kKappaTolerance);
    AddConstraint<DkappaConstraint>(&constraint_vec, kDKappaTolerance);
    AddConstraint<VLowerConstraint>(&constraint_vec, kVlowerTolerance);
    AddConstraint<LonAccConstraint>(&constraint_vec, kAccelTolerance);
    AddConstraint<LonJerkConstraint>(&constraint_vec, kJerklTolerance);
    ilqr_solver_->AddConstraint(step, constraint_vec);
  }
}

bool ILQRInterface::RbCollisionChecker(const StateVec & /*x_vec*/) const {
  return false;
}

void ILQRInterface::GetCarCircle(const e2e_noa::sptLatMotionPlanInput &input) {
  const auto &agent_params = input.input().agent_params();
  const auto &dis_center_to_rear_axle = agent_params.dis_center_to_rear_axle();
  const auto &length = agent_params.length();
  const auto &HalfWidth = agent_params.width() * 0.5;
  const double KFrontLength = 0.5 * length + dis_center_to_rear_axle;
  const double KRearLength = 0.5 * length - dis_center_to_rear_axle;
  const double KSideMirrorWidth = 1.0845;
  const double KSideMirrorFrontLength = 2.3;
  const double KSideMirrorRearLength = 2.0;

  double r0 = HalfWidth + 0.15;
  double r3 = (KSideMirrorFrontLength - KSideMirrorRearLength) * 0.5 + 0.1;

  const double x0 = 0.0;
  const double y0 = 0.0;
  std::array<double, 3> circle0 = {x0, y0, r0};

  const double x1 = (KFrontLength - KRearLength) * 0.5;
  const double y1 = 0.0;
  std::array<double, 3> circle1 = {x1, y1, r0};

  const double x2 = KFrontLength - r0;
  const double y2 = 0.0;
  std::array<double, 3> circle2 = {x2, y2, r0};

  const double x3 = (KSideMirrorFrontLength + KSideMirrorRearLength) * 0.5;
  const double y3 = (KSideMirrorWidth + HalfWidth) * 0.5;
  std::array<double, 3> circle3 = {x3, y3, r3};

  const double x4 = (KSideMirrorFrontLength + KSideMirrorRearLength) * 0.5;
  const double y4 = -(KSideMirrorWidth + HalfWidth) * 0.5;
  std::array<double, 3> circle4 = {x4, y4, r3};

  const double x5 = KFrontLength - std::sqrt(r0 * r0 - HalfWidth * HalfWidth);
  const double y5 = 0.0;
  std::array<double, 3> circle5 = {x5, y5, r0};

  std::array<std::array<double, 3>, 6> car_circle = {circle0, circle1, circle2,
                                                     circle3, circle4, circle5};
}

void ILQRInterface::Update(const ProtoRefPoints &fcs_refpath,
                           e2e_noa::sptLatMotionPlanInput *input,
                           e2e_noa::sptLatMotionPlanOutput *output,
                           std::vector<State> *dense_output) {
  auto *input = input->mutable_input();
  const auto &dec = input->micro_decider_output().dec();
  bool is_enable_rear_steering =
      input->ppprocessor_config().is_enable_rear_steering();
  auto *agent_params = input->mutable_agent_params();
  const double wheel_base = agent_params->wheel_base();
  if (is_enable_rear_steering) {
    agent_params->set_wheel_base(wheel_base * kWheelBaseScale);
  }
  ClearCandidatesOuput(&output_candidates_);
  UpdateInitState(input->input().refine_init_state(), &(x0_.at(0U)));
  UpdateCostConfig(fcs_refpath, *input);
  Solve(&output_candidates_.at(kNormal));
  UpdateOutputSolverInfo(*ilqr_solver_->GetSolverInfoPtr(), output);
  UpdateCilqrSolverInfo(*ilqr_solver_->GetCilqrInfoPtr(), output);
  if (is_enable_rear_steering) {
    agent_params->set_wheel_base(static_cast<float>(wheel_base));
    TransferOutput(wheel_base, &output_candidates_);
  }
  const int final_traj_index =
      UpdateOutput(output_candidates_, wheel_base, output);
}

void ILQRInterface::TransferOutput(const double wheel_base,
                                   std::vector<OutputInfo> *output_candidates) {
  if (output_candidates == nullptr) {
    return;
  }
  for (int i = 0; i < kSolutionSize; i++) {
    auto &cur_output = (*output_candidates).at(static_cast<unsigned long>(i));
    if (!cur_output.is_valid_traj) {
      break;
    }
    double rear_wheel_base = (1.0 - kWheelBaseScale) * wheel_base;
    auto &result_xk = cur_output.x_vec;
    for (size_t j = 0U; j < result_xk.size(); ++j) {
      result_xk[j][X] =
          result_xk[j][X] - rear_wheel_base * std::cos(result_xk[j][THETA]);
      result_xk[j][Y] =
          result_xk[j][Y] - rear_wheel_base * std::sin(result_xk[j][THETA]);
    }
  }
}

double ILQRInterface::Direction_Select(const double buff, const double center,
                                       const double target) const {
  double delta = NormalizeAngle(target - center);
  if ((delta >= buff) && (delta < M_PI - buff)) {
    if (delta < M_PI_2) {
      delta = buff;
    } else {
      delta = M_PI - buff;
    }
  } else if ((delta >= buff - M_PI) && (delta < -buff)) {
    if (delta >= -M_PI_2) {
      delta = -buff;
    } else {
      delta = buff - M_PI;
    }
  }
  return (center + delta);
}

int ILQRInterface::UpdateOutput(
    const std::vector<OutputInfo> &output_candidates, const double wheel_base,
    e2e_noa::sptLatMotionPlanOutput *output) const {
  auto *out_candidates_pb = output->mutable_output_candidates();

  const int init_seed_index = ilqr_solver_->GetInitSeedIndex();
  out_candidates_pb->set_init_seed_type(
      static_cast<e2e_noa::sptCandidateType>(init_seed_index));
  int final_output_type = 0;
  for (int i = kSolutionSize - 1; i >= 0; i--) {
    if (output_candidates[static_cast<unsigned long>(i)].is_turn_circle ||
        !output_candidates[static_cast<unsigned long>(i)].is_valid_traj) {
      continue;
    }
    final_output_type = i;
    out_candidates_pb->set_is_final_traj_rb_collision(
        output_candidates[static_cast<unsigned long>(i)].is_rb_collision);
    break;
  }
  out_candidates_pb->set_final_output_type(
      static_cast<e2e_noa::sptCandidateType>(final_output_type));

  return static_cast<int>(out_candidates_pb->final_output_type());
}

void ILQRInterface::UpdateInitState(const State &init_state, State *x) const {
  *x << init_state.x(), init_state.y(), init_state.theta(), init_state.kappa(),
      init_state.dkappa(), init_state.v(), init_state.a(), init_state.jerk();
}

void ILQRInterface::ClearCandidatesOuput(
    std::vector<OutputInfo> *output_candidates) const {
  output_candidates->clear();
  output_candidates->resize(kSolutionSize);
}

void ILQRInterface::GetOutputFromControl(const ControlVec &u_vec,
                                         OutputInfo *output) const {
  StateVec x_vec{};
  ResizeAndResetEigenVec(x_vec, Horizon() + 1U, GetStateSize());
  x_vec.at(0U) = x0_.at(0U);
  for (size_t i = 0U; i < u_vec.size(); i++) {
    x_vec.at(i + 1U) = ILqrLatModel::UpdateDynamicsWithDt(
        x_vec.at(i), u_vec.at(i), GetTimeStep(i));
  }

  *output = {x_vec, u_vec, PathCircleChecker(x_vec), RbCollisionChecker(x_vec),
             true};
}

void ILQRInterface::UpdateWarmStartStatus(
    const e2e_noa::sptLatMotionPlanInput &input) {
  ControlVec prev_plan_u{};
  ResizeAndResetEigenVec(prev_plan_u, Horizon(), GetInputSize());
  const auto &warm_start_data = input.warm_start_data();
  for (int i = 0; i < warm_start_data.stp_controls_size() - 1; i++) {
    prev_plan_u.at(static_cast<unsigned long>(i))(0) =
        warm_start_data.stp_controls(i).ddkappa();
    prev_plan_u.at(static_cast<unsigned long>(i))(1) =
        warm_start_data.stp_controls(i).djerk();
  }
  GetOutputFromControl(prev_plan_u, &output_candidates_.at(kPrePlan));

  ControlVec forward_sim_u{};
  StateVec forward_sim_x{};
  ForwardSimulationNew forward_sim{};
  const int ssp_result_size = input.input().ssp_results_size();
  ResizeAndResetEigenVec(ssp_result_u_,
                         static_cast<unsigned long>(ssp_result_size), 1U);
  for (int i = 0; i < ssp_result_size; ++i) {
    const auto &ssp_result = input.input().ssp_results(i);
    ssp_result_u_.at(static_cast<unsigned long>(i))(
        simplify_speed_planner::LonControlId::DJERK) = ssp_result.djerk();
  }
  (void)forward_sim.Simulation(data_cache_, x0_.at(0U), ssp_result_u_,
                               &forward_sim_x, &forward_sim_u);
  for (size_t i = 0U; i < forward_sim_x.size(); i++) {
    SppType::State tmp_spp_state{};
    const auto &cur_forward_sim_x = forward_sim_x[i];
    tmp_spp_state << cur_forward_sim_x(0), cur_forward_sim_x(1),
        cur_forward_sim_x(2), cur_forward_sim_x(3), cur_forward_sim_x(4);
    spp_input_x_list.emplace_back(tmp_spp_state);
  }
  spp_input_u_list.reserve(forward_sim_u.size());
  for (size_t i = 0U; i < forward_sim_u.size(); i++) {
    SppType::Control tmp_spp_control{};
    tmp_spp_control << forward_sim_u[i](0);
    spp_input_u_list.emplace_back(tmp_spp_control);
  }
  spp_init_x << x0_[0U](0), x0_[0U](1), x0_[0U](2), x0_[0U](3), x0_[0U](4);

  spp.RunSimplifyPathPlanner(spp_init_x, spp_input_x_list, spp_input_u_list,
                             Horizon(), data_cache_.get(), &spp_result_x,
                             &spp_result_u);
  ControlVec ssp_spp_warm_start_u{};
  ResizeAndResetEigenVec(ssp_spp_warm_start_u, Horizon(), GetInputSize());
  for (int i = 0; i < static_cast<int>(ssp_spp_warm_start_u.size()); i++) {
    ssp_spp_warm_start_u[static_cast<unsigned long>(i)](0) =
        spp_result_u.at(static_cast<unsigned long>(i))(0);
    ssp_spp_warm_start_u[static_cast<unsigned long>(i)](1) =
        ssp_result_u_.at(static_cast<unsigned long>(i))(0);
  }
  GetOutputFromControl(ssp_spp_warm_start_u,
                       &output_candidates_.at(kBackUpInit));

  ilqr_solver_->ResetU();

  ilqr_solver_->UpdateWarmStart({prev_plan_u, ssp_spp_warm_start_u});
}

void ILQRInterface::ClearAndResizeOutput(e2e_noa::sptOutput *output) const {
  auto states = output->mutable_states();
  states->Clear();
  states->Reserve(static_cast<int>(Horizon()) + 1);
  auto controls = output->mutable_stp_controls();
  controls->Clear();
  controls->Reserve(static_cast<int>(Horizon()) + 1);
}

void ILQRInterface::Solve(OutputInfo *output) const {
  ilqr_solver_->Solve(x0_.at(0U));

  *output = {*ilqr_solver_->GetStateResultPtr(),
             *ilqr_solver_->GetControlResultPtr(),
             PathCircleChecker(*ilqr_solver_->GetStateResultPtr()),
             RbCollisionChecker(*ilqr_solver_->GetStateResultPtr()), true};
}

}  // namespace spt
}  // namespace e2e_noa