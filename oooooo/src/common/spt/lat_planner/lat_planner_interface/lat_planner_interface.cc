#include "common/spt/lat_planner/lat_planner_interface/lat_planner_interface.h"

#include <cmath>
#include <vector>

#include "common/spt/lat_planner/lat_planner_interface/constraint/constraint_common.h"
#include "common/spt/lat_planner/lat_planner_interface/cost/ddkappa_cost.h"
#include "common/spt/lat_planner/lat_planner_interface/cost/djerk_cost.h"
#include "common/spt/lat_planner/lat_planner_interface/cost/heading_cost.h"
#include "common/spt/lat_planner/lat_planner_interface/cost/ref_a_cost.h"
#include "common/spt/lat_planner/lat_planner_interface/cost/ref_path_cost.h"
#include "common/spt/lat_planner/lat_planner_interface/cost/ref_s_cost.h"
#include "common/spt/lat_planner/lat_planner_interface/cost/ref_v_cost.h"
#include "common/spt/lat_planner/lat_planner_interface/cost/v_limit_cost.h"
#include "common/spt/lat_planner/lat_planner_interface/cost/long_jerk_cost.h"
#include "math/piecewise_linear_function.h"

namespace e2e_noa {
namespace spt {

static constexpr double kConstraintHorizonRatio = 0.25;
static constexpr double kConstraintDefault = 10.0;
static constexpr double kFurtherConstraintRatio = 0.8;
static constexpr double kLonAccBndBuffer = 0.5;
static constexpr double kLonAccLimitBuffer = 0.3;
static constexpr double kVelBndBuffer = 0.3;
constexpr int kScaleDecayIndex = 20;
constexpr double kScaleDecaySRange = 50.0;

LatPlannerInterface::LatPlannerInterface() { Init(); }
void LatPlannerInterface::Init() {
  spt_data_ = std::make_unique<LatSptDataCache>();
  ilqr_core_ptr_ =
      std::make_unique<ILqr<STATE_SIZE, INPUT_SIZE, CONSTRAINT_SIZE>>();
  lat_model_ = std::make_unique<SptLatModel>(*spt_data_);
  InitSolverConfig();
  cost_common_ptr_ = std::make_unique<LatCostCommon>(spt_data_.get());
  ilqr_core_ptr_->Init(lat_model_.get(), solver_config_ptr_.get(),
                       cost_common_ptr_.get());
  InitCostStack();
  // InitConstrainConfig(solver_config_ptr_.get());
  spt_data_->ResizeStepData(Horizon() + 1);
}

void LatPlannerInterface::InitCostStack() {
  cost_vec_.emplace_back(std::make_unique<RefpathCost>(*spt_data_));
  cost_vec_.emplace_back(std::make_unique<RefVCost>(*spt_data_));
  cost_vec_.emplace_back(std::make_unique<DDKappaCost>(*spt_data_));
  cost_vec_.emplace_back(std::make_unique<DJerkCost>(*spt_data_));
  cost_vec_.emplace_back(std::make_unique<HeadingCost>(*spt_data_));
  cost_vec_.emplace_back(std::make_unique<RefACost>(*spt_data_));
  cost_vec_.emplace_back(std::make_unique<RefSCost>(*spt_data_));
  cost_vec_.emplace_back(std::make_unique<VLimitCost>(*spt_data_));
  cost_vec_.emplace_back(std::make_unique<LONGJerkCost>(*spt_data_));

  for (const auto &cost_ptr : cost_vec_) {
    ilqr_core_ptr_->AddCost(cost_ptr.get());
  }
}

void LatPlannerInterface::InitSolverConfig() {
  solver_config_ptr_ = std::make_unique<SolverConfig>();
  const double kControlPercentTol = 1e-3;
  const double kControlTol = 1e-2;
  const int kHorizon = 100;
  const int kInputSize = 2;
  const int kStateSize = 8;
  const int kMaxIter = 50;
  const int kMaxOuterIter = 10;
  const double kPenaltyFactor = 20.0;
  const double kInitRho = 1000.0;
  solver_config_ptr_->is_debug_mode = false;
  solver_config_ptr_->cost_percent_tol = kControlPercentTol;
  solver_config_ptr_->cost_tol = kControlTol;
  solver_config_ptr_->horizon = kHorizon;
  solver_config_ptr_->input_size = kInputSize;
  solver_config_ptr_->state_size = kStateSize;
  solver_config_ptr_->max_iter = kMaxIter;
  solver_config_ptr_->variable_cost_percent_tol = true;
  solver_config_ptr_->max_outer_iterations = kMaxOuterIter;
  solver_config_ptr_->penalty_factor = kPenaltyFactor;
  solver_config_ptr_->init_rho = kInitRho;
}

void LatPlannerInterface::Reset() {
  spt_data_->Reset();
  ilqr_core_ptr_->Reset();
  ResizeAndResetEigenVec(x0_, Horizon() + 1, GetStateSize());
}

void LatPlannerInterface::InitConstrainConfig() {}

void LatPlannerInterface::BuildCircleModel(const LatPlannerInput &input) {
  const auto &ego_params = input.ego_params;
  const double kHalfWidth = ego_params.width * 0.5;
  const double kHalfLength = ego_params.length * 0.5;
  const double center_to_rear_axle = ego_params.center_to_rear_axle;

  const double r = kHalfWidth + 0.15;

  const double x0 = 0.0;
  const double y0 = 0.0;
  std::array<double, 3> cir0 = {x0, y0, r};

  const double x1 = center_to_rear_axle * 2.0;
  const double y1 = 0.0;
  std::array<double, 3> cir1 = {x1, y1, r};

  const double x2 = kHalfLength + center_to_rear_axle - r;
  const double y2 = 0.0;
  std::array<double, 3> cir2 = {x2, y2, r};

  std::array<std::array<double, 3>, 3> cirs_info = {cir0, cir1, cir2};

  for (size_t i = 0; i < Horizon() + 1; i++) {
    std::vector<CircleModelInfo> circle_model_info;
    circle_model_info.reserve(cirs_info.size());
    for (size_t j = 0; j < cirs_info.size(); j++) {
      CircleModelInfo temp;
      temp.SetLength(cirs_info[j][0]);
      temp.SetWidth(cirs_info[j][1]);
      temp.SetRadius(cirs_info[j][2]);
      circle_model_info.push_back(std::move(temp));
    }
    spt_data_->SetCircleModelInfo(i, std::move(circle_model_info));
  }
}

void LatPlannerInterface::Update(std::vector<RefPoint> &ref_path,
                                 LatPlannerInput *input,
                                 LatPlannerOutput *output) {
  const auto &ego_params = input->ego_params;
  const double wheel_base = ego_params.wheel_base;

  UpdateInitState(input->init_point, &(x0_.at(0)));
  UpdateCostConfig(ref_path, *input);
  // UpdateConstraintBound(*input);

  Solve(&output_);
}

void LatPlannerInterface::UpdateConstraintBound(
    const LatPlannerInput &input) const {
  PiecewiseLinearFunction<double, double> front_angle_plf(
      input.spt_params.v_list, input.spt_params.front_angle_list);
  PiecewiseLinearFunction<double, double> front_angle_rate_plf(
      input.spt_params.v_list, input.spt_params.front_angle_rate_list);
  const double wheel_base = input.ego_params.wheel_base;
  constexpr double kDecay = 0.25;
  auto get_dkappa_bound = [&front_angle_plf, &front_angle_rate_plf,
                           &wheel_base](double v) {
    const double front_angle = front_angle_plf(v);
    const double front_angle_rate = front_angle_rate_plf(v);
    const double dkappa_bound =
        (1.0 + Sqr(tan(front_angle))) * front_angle_rate / wheel_base;
    return dkappa_bound;
  };

  const double default_dkappa_bound = get_dkappa_bound(0.0);

  for (int i = 0; i < Horizon() + 1; ++i) {
    spt_data_->SetSptStepData(
        i,
        SptStepData::DKAPPA_BOUNDARY_UPPER,  // to do get dkappa with target v
        default_dkappa_bound);
    spt_data_->SetSptStepData(
        i,
        SptStepData::DKAPPA_BOUNDARY_LOWER,  // to do get dkappa with target v
        -default_dkappa_bound);
  }
}

void LatPlannerInterface::UpdateCostConfig(
    const std::vector<RefPoint> &ref_path, const LatPlannerInput &input) {
  const int ref_path_size = ref_path.size();
  spt_data_->ResizeRefpath(ref_path_size);
  for (int i = 0; i < ref_path_size; ++i) {
    UpdateRefPathInfo(ref_path[i], i);
    if (i < ref_path_size - 1) {
      UpdateRefPathSegmentInfo(ref_path[i], ref_path[i + 1], i);
    }
  }
  UpdateStepTime();
  BuildCircleModel(input);
  UpdateConstantInfo(input, ref_path_size);
  UpdateStepInfo(input);

  spt_data_->SetSptStepData(Horizon(), TERMINAL_FLAG, 1.0);
}

void LatPlannerInterface::UpdateRefPathInfo(const RefPoint &ref_point,
                                            const size_t step) const {
  spt_data_->SetSptRefPath(step, SptRefPath::REF_X, ref_point.x);
  spt_data_->SetSptRefPath(step, SptRefPath::REF_Y, ref_point.y);
  spt_data_->SetSptRefPath(step, SptRefPath::REF_S, ref_point.s);
  spt_data_->SetSptRefPath(step, SptRefPath::REF_THETA,
                           ref_point.theta);  // normalize
  spt_data_->SetSptRefPath(step, SptRefPath::REF_COS_THETA,
                           std::cos(ref_point.theta));
  spt_data_->SetSptRefPath(step, SptRefPath::REF_SIN_THETA,
                           std::sin(ref_point.theta));
  spt_data_->SetSptRefPath(step, SptRefPath::REF_KAPPA, ref_point.kappa);
  spt_data_->SetSptRefPath(step, SptRefPath::REF_V, ref_point.v);
  spt_data_->SetSptRefPath(step, SptRefPath::REF_A, ref_point.a);
}
void LatPlannerInterface::UpdateRefPathSegmentInfo(const RefPoint &s,
                                                   const RefPoint &e,
                                                   const size_t step) const {
  Segment2d seg(Vec2d(s.x, s.y), Vec2d(e.x, e.y));
  spt_data_->SetSptRefPathSegment(step, std::move(seg));
}

void LatPlannerInterface::UpdateConstantInfo(const LatPlannerInput &input,
                                             const size_t size) {
  const double kRefLaneCostHuberThreshold = 4.0;
  spt_data_->SetSptConstant(SptConstant::REF_PATH_COST_HUBER_THRESHOLD,
                            kRefLaneCostHuberThreshold);
  spt_data_->SetSptConstant(SptConstant::REF_PATH_START_INDEX, 0.0);
  spt_data_->SetSptConstant(SptConstant::REF_PATH_END_INDEX, size);
  spt_data_->SetSptConstant(SptConstant::REF_S_COST_HUBER_THRESHOLD, 2.0);
}

void LatPlannerInterface::UpdateStepInfo(const LatPlannerInput &input) {
  constexpr double kDmapHorizon = 20;
  int damp_index = -1;
  for (size_t i = 0; i < Horizon() + 1; i++) {
    double ref_path_cost_ratio = 1.0;
    if (i > kDmapHorizon && damp_index == -1) {
      damp_index = i;
    }
    if (damp_index != -1) {
      ref_path_cost_ratio = Logistic(i - damp_index, -0.5, 0.1,
                                     (Horizon() - damp_index) / 2.0, 1.0);
    }
    spt_data_->SetSptStepData(
        i, SptStepData::REF_PATH_COST_WEIGHT,
        input.spt_params.ref_path_cost_weight * ref_path_cost_ratio);
    spt_data_->SetSptStepData(i, SptStepData::REF_V_COST_WEIGHT,
                              input.spt_params.ref_v_cost_weight);
    spt_data_->SetSptStepData(i, SptStepData::DDKAPPA_WEIGHT,
                              input.spt_params.ddkappa_weight);
    spt_data_->SetSptStepData(i, SptStepData::DJERK_WEIGHT,
                              input.spt_params.djerk_weight);
    spt_data_->SetSptStepData(i, SptStepData::HEADING_WEIGHT,
                              input.spt_params.heading_weight);
    spt_data_->SetSptStepData(i, SptStepData::REF_A_WEIGHT,
                              input.spt_params.ref_a_weight);
    spt_data_->SetSptStepData(i, SptStepData::REF_S_COST_WEIGHT,
                              input.spt_params.ref_s_cost_weight);
    spt_data_->SetSptStepData(i, SptStepData::V_LIMIT_UB_WEIGHT,
                              input.spt_params.v_limit_ub_weight);
    spt_data_->SetSptStepData(i, SptStepData::V_LIMIT_LB_WEIGHT,
                              input.spt_params.v_limit_lb_weight);

    spt_data_->SetSptStepData(i, SptStepData::JERK_LONG_WEIGHT,
                              input.spt_params.jerk_long_weight);
    spt_data_->SetSptStepData(i, SptStepData::JERK_LONG_UB_WEIGHT,
                              input.spt_params.jerk_long_ub_weight);
    spt_data_->SetSptStepData(i, SptStepData::JERK_LONG_LB_WEIGHT,
                              input.spt_params.jerk_long_lb_weight);
    spt_data_->SetSptStepData(i, SptStepData::JERK_LONG_BOUNDARY_UPPER,
                              input.spt_params.jerk_long_boundary_upper);
    spt_data_->SetSptStepData(i, SptStepData::JERK_LONG_BOUNDARY_LOWER,
                              input.spt_params.jerk_long_boundary_lower);
  }
}

void LatPlannerInterface::UpdateStepTime() const {
  auto SetStepData = [&](const int step, const SptStepData index,
                         const double val) {
    spt_data_->SetSptStepData(step, index, val);
  };
  for (size_t i = 0; i < Horizon(); i++) {
    SetStepData(i, SptStepData::DT, GetTimeStep(i));
    if (i == 0) {
      SetStepData(i, SptStepData::DT, 0.0);
      continue;
    }
    SetStepData(
        i, SptStepData::T,
        spt_data_->GetSptStepData(i - 1, SptStepData::T) + GetTimeStep(i - 1));
  }
  SetStepData(Horizon(), SptStepData::DT,
              spt_data_->GetSptStepData(Horizon() - 1, SptStepData::T) +
                  GetTimeStep(Horizon() - 1));
}

void LatPlannerInterface::UpdateInitState(const TrajectoryPoint &init_point,
                                          State *x) const {
  *x << init_point.x, init_point.y, init_point.theta, init_point.kappa,
      init_point.dkappa, init_point.v, init_point.a, init_point.jerk;
}

void LatPlannerInterface::Solve(OutputInfo *output) const {
  ilqr_core_ptr_->Solve(x0_.at(0));

  *output = {*ilqr_core_ptr_->GetStateResultPtr(),
             *ilqr_core_ptr_->GetControlResultPtr()};
  int size = output->x_vec.size();
  const auto &x_vec = output->x_vec;
  const auto &u_vec = output->u_vec;
  for (int i = 0; i < size; ++i) {
    std::cout << "horizon: " << i << std::endl;
    std::cout << "x:" << x_vec[i][StateIndex::X] << std::endl;
    std::cout << "y:" << x_vec[i][StateIndex::Y] << std::endl;
    std::cout << "v:" << x_vec[i][StateIndex::V] << std::endl;
    std::cout << "theta:" << x_vec[i][StateIndex::THETA] << std::endl;
    std::cout << "kappa:" << x_vec[i][StateIndex::KAPPA] << std::endl;
    std::cout << "dkappa:" << x_vec[i][StateIndex::DKAPPA] << std::endl;
    std::cout << "acc:" << x_vec[i][StateIndex::ACC] << std::endl;
    std::cout << "jerk:" << x_vec[i][StateIndex::JERK] << std::endl;
    std::cout << "ddkappa:" << u_vec[i][ControlIndex::DDKAPPA] << std::endl;
    std::cout << "djerk:" << u_vec[i][ControlIndex::DJERK] << std::endl;

    // std::cout << "rear_ref_x: "
    //           << spt_data_->GetCircleModelInfo(i)[0].GetCircleData(
    //                  SptCircleData::MATCHED_REF_X)
    //           << std::endl;
    // std::cout << "rear_ref_y: "
    //           << spt_data_->GetCircleModelInfo(i)[0].GetCircleData(
    //                  SptCircleData::MATCHED_REF_Y)
    //           << std::endl;
    // std::cout << "rear_lat_offset: "
    //           << spt_data_->GetCircleModelInfo(i)[0].GetCircleData(
    //                  SptCircleData::LAT_OFFSET)
    //           << std::endl;
    // std::cout << "front_ref_x: "
    //           << spt_data_->GetCircleModelInfo(i)[2].GetCircleData(
    //                  SptCircleData::MATCHED_REF_X)
    //           << std::endl;
    // std::cout << "front_ref_y: "
    //           << spt_data_->GetCircleModelInfo(i)[2].GetCircleData(
    //                  SptCircleData::MATCHED_REF_Y)
    //           << std::endl;
    // std::cout << "front_lat_offset: "
    //           << spt_data_->GetCircleModelInfo(i)[0].GetCircleData(
    //                  SptCircleData::LAT_OFFSET)
    //           << std::endl;
    std::cout << "-------------------------------------" << std::endl;
  }
}

}  // namespace spt
}  // namespace e2e_noa
