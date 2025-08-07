#pragma once

#include <cstddef>

#include "common/spt/lat_planner/lat_planner_interface/data/spt_data.h"
#include "common/spt/solver/cost_base.h"
namespace e2e_noa {
namespace spt {

enum CostList : uint8_t {
  REF_PATH_COST = 0,
  REF_V_COST,
  DDKAPPA_COST,
  DJERK_COST,
  HEADING_COST,
  REF_A_COST,
  REF_S_COST,
  V_LIMIT_COST,
  LONG_JERK_COST,
  COST_SIZE,
};

static constexpr double kHalf = 0.5;
static constexpr double kTwo = 2.0;
static constexpr double kThree = 3.0;
static constexpr double kSix = 6.0;
constexpr double kOneThird = 1.0 / kThree;
static inline double Square(double x) { return x * x; }
static inline double QuadraticCost(const double weight, const double func) {
  return kHalf * weight * Square(func);
}
static inline double HuberLoss(const double weight, const double func,
                               const double threshold) {
  return weight * (std::fabs(func) <= threshold
                       ? kHalf * Square(func)
                       : threshold * (std::fabs(func) - kHalf * threshold));
}
static inline double HuberLossGrad1D(const double weight, const double func,
                                     const double threshold,
                                     const double grad) {
  double huber_grad{0.0};
  if (func < -threshold) {
    huber_grad = weight * -threshold * grad;
  } else if (func < threshold) {
    huber_grad = weight * func * grad;
  } else {
    huber_grad = weight * threshold * grad;
  }
  return huber_grad;
}
static inline double QuadraticGrad1D(const double weight, const double func,
                                     const double grad) {
  return weight * func * grad;
}
static inline double HuberLossHess1D(const double weight, const double func,
                                     const double threshold,
                                     const double grad_1, const double grad_2,
                                     const double hess) {
  double huber_hess{0.0};
  if (func < -threshold) {
    huber_hess = weight * -threshold * hess;
  } else if (func < threshold) {
    huber_hess = weight * grad_1 * grad_2 + weight * func * hess;
  } else {
    huber_hess = weight * threshold * hess;
  }
  return huber_hess;
}
static inline double QuadraticHess1D(const double weight, const double func,
                                     const double grad_1, const double grad_2,
                                     const double hess) {
  return weight * grad_1 * grad_2 + weight * func * hess;
}

class LatCostCommon : public e2e_noa::spt::CostBase<STATE_SIZE, INPUT_SIZE> {
 public:
  explicit LatCostCommon(LatSptDataCache *data) { spt_data_ = data; }
  void UpdateCostCommon(const State &x, const Control & /*u*/,
                        const size_t step) override;
  void ComputeCirclePos(const State &x, const size_t step,
                        std::vector<CircleModelInfo> *circle_infos);
  static double GetCostOneStep(const double weight, const double threshold,
                               const double offset, const double upper,
                               const double lower);

  static void GetGradientAndHessianOneStep(
      const double weight, const double threshold, const double offset,
      const double upper, const double lower, const double cos_theta,
      const double sin_theta, const double length, const double width,
      const double ref_cos_theta, const double ref_sin_theta, Lx *const lx,
      Lxx *const lxx);
  static double GetCostOneStep(const double weight, const double threshold,
                               const double offset);
  static void GetGradientAndHessianOneStep(
      const double weight, const double threshold, const double offset,
      const double cos_theta, const double sin_theta, const double length,
      const double width, const double ref_cos_theta,
      const double ref_sin_theta, Lx *const lx, Lxx *const lxx);
  static void GetGradientAndHessianRefDsOneStep(
      const double weight, const double threshold, const double ref_ds,
      const double cos_theta, const double sin_theta, const double length,
      const double width, const double ref_cos_theta,
      const double ref_sin_theta, Lx *const lx, Lxx *const lxx);

 private:
  LatSptDataCache *spt_data_;
};

}  // namespace spt
}  // namespace e2e_noa
