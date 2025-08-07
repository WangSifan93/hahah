#include "common/spt/lat_planner/lat_planner_interface/cost/cost_common.h"

#include <iostream>

#include "common/spt/lat_planner/lat_planner_interface/data/spt_data.h"
#include "common/spt/solver/utils.h"

namespace e2e_noa {
namespace spt {
void LatCostCommon::UpdateCostCommon(const State &x, const Control & /*u*/,
                                     const size_t step) {
  auto GetStepData = [&](const SptStepData index) {
    return spt_data_->GetSptStepData(step, index);
  };
  auto SetStepData = [&](const SptStepData index, const double val) {
    spt_data_->SetSptStepData(step, index, val);
  };
  auto GetRefPathData = [&](const SptRefPath index) {
    return spt_data_->GetRefPath(step, index);
  };

  SetStepData(SptStepData::V_SQUARE, x[StateIndex::V] * x[StateIndex::V]);
  SetStepData(SptStepData::LAT_ACC,
              GetStepData(SptStepData::V_SQUARE) * x[StateIndex::KAPPA]);
  SetStepData(
      SptStepData::LAT_JERK,
      3.0 * x[StateIndex::V] * x[StateIndex::ACC] * x[StateIndex::KAPPA] +
          GetStepData(SptStepData::V_SQUARE) * x[StateIndex::DKAPPA]);
  SetStepData(SptStepData::LON_JERK,
              x[StateIndex::JERK] -
                  GetStepData(SptStepData::V_SQUARE) * x[StateIndex::V] *
                      x[StateIndex::KAPPA] * x[StateIndex::KAPPA]);
  SetStepData(SptStepData::SIN_THETA, std::sin(x[StateIndex::THETA]));
  SetStepData(SptStepData::COS_THETA, std::cos(x[StateIndex::THETA]));

  auto *circle_model_info = spt_data_->GetMutableCircleModelInfo(step);
  ComputeCirclePos(x, step, circle_model_info);
  const int circle_size = spt_data_->GetCircleModelInfo(step).size();

  const auto &ref_path_segment = spt_data_->GetRefPathSegment();
  const auto start_index =
      spt_data_->GetSptConstant(SptConstant::REF_PATH_START_INDEX);
  const auto end_index =
      spt_data_->GetSptConstant(SptConstant::REF_PATH_END_INDEX);
  const int last_rear_circle_match_index =
      (step == 0) ? start_index
                  : spt_data_->GetSptStepData(
                        step - 1, SptStepData::LAST_REAR_CIRCLE_MATCH_INDEX);
  const int kProtectIndex = 5;
  const int search_start_index =
      std::max(static_cast<int>(start_index),
               last_rear_circle_match_index - kProtectIndex);
  const int search_end_index =
      std::min(static_cast<int>(end_index),
               static_cast<int>(spt_data_->GetRefPathSegment().size()));
  int rear_circle_match_index = search_start_index;
  for (int i = 0; i < circle_size; ++i) {
    auto *circle_info = &(circle_model_info->at(i));
    const double circle_x = circle_info->X();
    const double circle_y = circle_info->Y();
    double min_distance = std::numeric_limits<double>::max();
    int match_ref_path_index = 0;
    const int search_start_index_consider_rear_res =
        std::max(static_cast<int>(start_index), rear_circle_match_index);

    for (int j = search_start_index_consider_rear_res; j < search_end_index;
         ++j) {
      const double temp_distance =
          ref_path_segment.at(j).DistanceSquareTo(Vec2d(circle_x, circle_y));
      if (temp_distance < min_distance) {
        min_distance = temp_distance;
        match_ref_path_index = j;
      }
    }
    const auto &matched_ref_segment = ref_path_segment.at(match_ref_path_index);
    double ds = matched_ref_segment.ProjectOntoUnit(Vec2d(circle_x, circle_y));

    const double ref_s = ds;

    const double lat_offset =
        -GetRefPathData(SptRefPath::REF_SIN_THETA) *
            (circle_x - GetRefPathData(SptRefPath::REF_X)) +
        GetRefPathData(SptRefPath::REF_COS_THETA) *
            (circle_y - GetRefPathData(SptRefPath::REF_Y));
    circle_info->SetCircleData(SptCircleData::LAT_OFFSET, lat_offset);
    circle_info->SetCircleData(
        SptCircleData::MATCHED_REF_X,
        spt_data_->GetRefPath(match_ref_path_index, SptRefPath::REF_X));
    circle_info->SetCircleData(
        SptCircleData::MATCHED_REF_Y,
        spt_data_->GetRefPath(match_ref_path_index, SptRefPath::REF_Y));
    circle_info->SetCircleData(
        SptCircleData::MATCHED_REF_ThETA,
        spt_data_->GetRefPath(match_ref_path_index, SptRefPath::REF_THETA));
    circle_info->SetCircleData(
        SptCircleData::MATCHED_REF_KAPPA,
        spt_data_->GetRefPath(match_ref_path_index, SptRefPath::REF_KAPPA));
    circle_info->SetCircleData(
        SptCircleData::MATCHED_REF_SIN_THETA,
        spt_data_->GetRefPath(match_ref_path_index, SptRefPath::REF_SIN_THETA));
    circle_info->SetCircleData(
        SptCircleData::MATCHED_REF_COS_THETA,
        spt_data_->GetRefPath(match_ref_path_index, SptRefPath::REF_COS_THETA));
    circle_info->SetCircleData(
        SptCircleData::MATCHED_REF_V,
        spt_data_->GetRefPath(match_ref_path_index, SptRefPath::REF_V));
    circle_info->SetCircleData(
        SptCircleData::MATCHED_REF_A,
        spt_data_->GetRefPath(match_ref_path_index, SptRefPath::REF_A));
    circle_info->SetCircleData(SptCircleData::S_OFFSET, ref_s);
  }

  SetStepData(SptStepData::LAST_REAR_CIRCLE_MATCH_INDEX,
              rear_circle_match_index);
}

double LatCostCommon::GetCostOneStep(const double weight,
                                     const double threshold,
                                     const double offset, const double upper,
                                     const double lower) {
  if ((upper < lower) || ((offset < upper) && (offset > lower))) {
    return 0.0;
  }
  const double diff_offset = offset > upper ? offset - upper : offset - lower;
  return HuberLoss(weight, diff_offset, threshold);
}

double LatCostCommon::GetCostOneStep(const double weight,
                                     const double threshold,
                                     const double offset) {
  return HuberLoss(weight, offset, threshold);
}

void LatCostCommon::GetGradientAndHessianOneStep(
    const double weight, const double threshold, const double offset,
    const double upper, const double lower, const double cos_theta,
    const double sin_theta, const double length, const double width,
    const double ref_cos_theta, const double ref_sin_theta, Lx *const lx,
    Lxx *const lxx) {
  if ((upper < lower) || ((offset < upper) && (offset > lower))) {
    return;
  }
  const double diff_offset = offset > upper ? offset - upper : offset - lower;

  const double grad_x = -ref_sin_theta;
  const double grad_y = ref_cos_theta;
  const double grad_theta =
      length * (sin_theta * ref_sin_theta + cos_theta * ref_cos_theta) +
      width * (cos_theta * ref_sin_theta - sin_theta * ref_cos_theta);
  (*lx)(StateIndex::X) +=
      HuberLossGrad1D(weight, diff_offset, threshold, grad_x);
  (*lx)(StateIndex::Y) +=
      HuberLossGrad1D(weight, diff_offset, threshold, grad_y);
  (*lx)(StateIndex::THETA) +=
      HuberLossGrad1D(weight, diff_offset, threshold, grad_theta);

  const double hess_x_x = 0.0;
  const double hess_x_y = 0.0;
  const double hess_x_theta = 0.0;
  const double hess_y_y = 0.0;
  const double hess_y_theta = 0.0;
  const double hess_theta_theta =
      length * (cos_theta * ref_sin_theta - sin_theta * ref_cos_theta) +
      width * (-sin_theta * ref_sin_theta - cos_theta * ref_cos_theta);
  (*lxx)(StateIndex::X, StateIndex::X) +=
      HuberLossHess1D(weight, diff_offset, threshold, grad_x, grad_x, hess_x_x);
  (*lxx)(StateIndex::Y, StateIndex::Y) +=
      HuberLossHess1D(weight, diff_offset, threshold, grad_y, grad_y, hess_y_y);
  (*lxx)(StateIndex::THETA, StateIndex::THETA) += HuberLossHess1D(
      weight, diff_offset, threshold, grad_theta, grad_theta, hess_theta_theta);
  const double lxx_x_y_temp =
      HuberLossHess1D(weight, diff_offset, threshold, grad_x, grad_y, hess_x_y);
  (*lxx)(StateIndex::X, StateIndex::Y) += lxx_x_y_temp;
  (*lxx)(StateIndex::Y, StateIndex::X) += lxx_x_y_temp;
  const double lxx_x_theta_temp = HuberLossHess1D(
      weight, diff_offset, threshold, grad_x, grad_theta, hess_x_theta);
  (*lxx)(StateIndex::X, StateIndex::THETA) += lxx_x_theta_temp;
  (*lxx)(StateIndex::THETA, StateIndex::X) += lxx_x_theta_temp;
  const double lxx_y_theta_temp = HuberLossHess1D(
      weight, diff_offset, threshold, grad_y, grad_theta, hess_y_theta);
  (*lxx)(StateIndex::Y, StateIndex::THETA) += lxx_y_theta_temp;
  (*lxx)(StateIndex::THETA, StateIndex::Y) += lxx_y_theta_temp;
}

void LatCostCommon::GetGradientAndHessianOneStep(
    const double weight, const double threshold, const double offset,
    const double cos_theta, const double sin_theta, const double length,
    const double width, const double ref_cos_theta, const double ref_sin_theta,
    Lx *const lx, Lxx *const lxx) {
  const double grad_x = -ref_sin_theta;
  const double grad_y = ref_cos_theta;
  const double grad_theta =
      length * (sin_theta * ref_sin_theta + cos_theta * ref_cos_theta) +
      width * (cos_theta * ref_sin_theta - sin_theta * ref_cos_theta);
  (*lx)(StateIndex::X) += HuberLossGrad1D(weight, offset, threshold, grad_x);
  (*lx)(StateIndex::Y) += HuberLossGrad1D(weight, offset, threshold, grad_y);
  (*lx)(StateIndex::THETA) +=
      HuberLossGrad1D(weight, offset, threshold, grad_theta);

  const double hess_x_x = 0.0;
  const double hess_x_y = 0.0;
  const double hess_x_theta = 0.0;
  const double hess_y_y = 0.0;
  const double hess_y_theta = 0.0;
  const double hess_theta_theta =
      length * (cos_theta * ref_sin_theta - sin_theta * ref_cos_theta) +
      width * (-sin_theta * ref_sin_theta - cos_theta * ref_cos_theta);
  (*lxx)(StateIndex::X, StateIndex::X) +=
      HuberLossHess1D(weight, offset, threshold, grad_x, grad_x, hess_x_x);
  (*lxx)(StateIndex::Y, StateIndex::Y) +=
      HuberLossHess1D(weight, offset, threshold, grad_y, grad_y, hess_y_y);
  (*lxx)(StateIndex::THETA, StateIndex::THETA) += HuberLossHess1D(
      weight, offset, threshold, grad_theta, grad_theta, hess_theta_theta);
  const double lxx_x_y_temp =
      HuberLossHess1D(weight, offset, threshold, grad_x, grad_y, hess_x_y);
  (*lxx)(StateIndex::X, StateIndex::Y) += lxx_x_y_temp;
  (*lxx)(StateIndex::Y, StateIndex::X) += lxx_x_y_temp;
  const double lxx_x_theta_temp = HuberLossHess1D(
      weight, offset, threshold, grad_x, grad_theta, hess_x_theta);
  (*lxx)(StateIndex::X, StateIndex::THETA) += lxx_x_theta_temp;
  (*lxx)(StateIndex::THETA, StateIndex::X) += lxx_x_theta_temp;
  const double lxx_y_theta_temp = HuberLossHess1D(
      weight, offset, threshold, grad_y, grad_theta, hess_y_theta);
  (*lxx)(StateIndex::Y, StateIndex::THETA) += lxx_y_theta_temp;
  (*lxx)(StateIndex::THETA, StateIndex::Y) += lxx_y_theta_temp;
}
void LatCostCommon::GetGradientAndHessianRefDsOneStep(
    const double weight, const double threshold, const double ref_ds,
    const double cos_theta, const double sin_theta, const double length,
    const double width, const double ref_cos_theta, const double ref_sin_theta,
    Lx *const lx, Lxx *const lxx) {
  // ref_ds的梯度计算
  // ref_ds是圆心在参考路径方向上的投影，其梯度为参考路径的单位方向向量
  const double grad_x = ref_cos_theta;
  const double grad_y = ref_sin_theta;

  // 对于theta的梯度，需要考虑车辆旋转对圆心位置的影响
  const double grad_theta =
      -length * (sin_theta * ref_cos_theta - cos_theta * ref_sin_theta) +
      width * (cos_theta * ref_cos_theta + sin_theta * ref_sin_theta);

  // 更新梯度
  (*lx)(StateIndex::X) += HuberLossGrad1D(weight, ref_ds, threshold, grad_x);
  (*lx)(StateIndex::Y) += HuberLossGrad1D(weight, ref_ds, threshold, grad_y);
  (*lx)(StateIndex::THETA) +=
      HuberLossGrad1D(weight, ref_ds, threshold, grad_theta);

  // 计算海森矩阵元素
  const double hess_x_x = 0.0;
  const double hess_x_y = 0.0;
  const double hess_x_theta = 0.0;
  const double hess_y_y = 0.0;
  const double hess_y_theta = 0.0;

  const double hess_theta_theta =
      -length * (cos_theta * ref_cos_theta + sin_theta * ref_sin_theta) +
      width * (-sin_theta * ref_cos_theta + cos_theta * ref_sin_theta);

  // 更新海森矩阵
  (*lxx)(StateIndex::X, StateIndex::X) +=
      HuberLossHess1D(weight, ref_ds, threshold, grad_x, grad_x, hess_x_x);
  (*lxx)(StateIndex::Y, StateIndex::Y) +=
      HuberLossHess1D(weight, ref_ds, threshold, grad_y, grad_y, hess_y_y);
  (*lxx)(StateIndex::THETA, StateIndex::THETA) += HuberLossHess1D(
      weight, ref_ds, threshold, grad_theta, grad_theta, hess_theta_theta);

  const double lxx_x_y_temp =
      HuberLossHess1D(weight, ref_ds, threshold, grad_x, grad_y, hess_x_y);
  (*lxx)(StateIndex::X, StateIndex::Y) += lxx_x_y_temp;
  (*lxx)(StateIndex::Y, StateIndex::X) += lxx_x_y_temp;

  const double lxx_x_theta_temp = HuberLossHess1D(
      weight, ref_ds, threshold, grad_x, grad_theta, hess_x_theta);
  (*lxx)(StateIndex::X, StateIndex::THETA) += lxx_x_theta_temp;
  (*lxx)(StateIndex::THETA, StateIndex::X) += lxx_x_theta_temp;

  const double lxx_y_theta_temp = HuberLossHess1D(
      weight, ref_ds, threshold, grad_y, grad_theta, hess_y_theta);
  (*lxx)(StateIndex::Y, StateIndex::THETA) += lxx_y_theta_temp;
  (*lxx)(StateIndex::THETA, StateIndex::Y) += lxx_y_theta_temp;
}
void LatCostCommon::ComputeCirclePos(
    const State &x, const size_t step,
    std::vector<CircleModelInfo> *circle_infos) {
  for (auto &cir : *circle_infos) {
    cir.SetX(
        x[StateIndex::X] +
        spt_data_->GetSptStepData(step, SptStepData::COS_THETA) * cir.Length() -
        spt_data_->GetSptStepData(step, SptStepData::SIN_THETA) * cir.Width());
    cir.SetY(
        x[StateIndex::Y] +
        spt_data_->GetSptStepData(step, SptStepData::SIN_THETA) * cir.Length() +
        spt_data_->GetSptStepData(step, SptStepData::COS_THETA) * cir.Width());
  }
}
}  // namespace spt
}  // namespace e2e_noa
