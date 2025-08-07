#pragma once

#include <cstddef>

#include "ilqr_cost.h"
#include "spatio_temporal_data.h"
namespace e2e_noa {
namespace spt {

static inline double Square(double x) { return x * x; }
static inline double QuadraticCost(const double weight, const double func) {
  return kHalf * weight * Square(func);
}
static inline double HuberLoss(const double weight, const double func,
                               const double threshold);
static inline double HuberLossGrad1D(const double weight, const double func,
                                     const double threshold, const double grad);
static inline double QuadraticGrad1D(const double weight, const double func,
                                     const double grad);
static inline double HuberLossHess1D(const double weight, const double func,
                                     const double threshold,
                                     const double grad_1, const double grad_2,
                                     const double hess);
static inline double QuadraticHess1D(const double weight, const double func,
                                     const double grad_1, const double grad_2,
                                     const double hess);

// Reference path tracking cost
class RefpathCost : public ilqr_solver::CostBase<state, input> {
 public:
  explicit RefpathCost(const SptDataCache &data_cache)
      : data_cache_(data_cache) {}
  double GetCost(const State &x, const Control & /*u*/,
                 const size_t step) const override;
  void GetGradientHessian(const State &x, const Control & /*u*/,
                          const size_t step, dx *const dx, du *const /*du*/,
                          ddx *const ddx, dxu *const /*dxu*/,
                          ddu *const /*ddu*/) const override;
  std::uint8_t GetId() const override { return REFPATH_COST; }

 private:
  const SptDataCache &data_cache_;
};

// Reference heading tracking cost
class HeadingCost : public ilqr_solver::CostBase<state, input> {
 public:
  explicit HeadingCost(const SptDataCache &data_cache)
      : data_cache_(data_cache) {}
  double GetCost(const State &x, const Control & /*u*/,
                 const size_t step) const override;
  void GetGradientHessian(const State &x, const Control & /*u*/,
                          const size_t step, dx *const dx, du *const /*du*/,
                          ddx *const ddx, dxu *const /*dxu*/,
                          ddu *const /*ddu*/) const override;
  std::uint8_t GetId() const override { return HEADING_COST; }

 private:
  const SptDataCache &data_cache_;
};

// Reference s tracking cost
class SRefCost : public ilqr_solver::CostBase<state, input> {
 public:
  explicit SRefCost(const SptDataCache &data_cache) : data_cache_(data_cache) {}
  double GetCost(const State &x, const Control & /*u*/,
                 const size_t step) const override;
  void GetGradientHessian(const State &x, const Control & /*u*/,
                          const size_t step, dx *const dx, du *const /*du*/,
                          ddx *const ddx, dxu *const /*dxu*/,
                          ddu *const /*ddu*/) const override;
  std::uint8_t GetId() const override {
    return static_cast<std::uint8_t>(SREF_COST);
  }

 private:
  const SptDataCache &data_cache_;
};

// Reference velocity tracking cost
class VRefCost : public ilqr_solver::CostBase<state, input> {
 public:
  explicit VRefCost(const SptDataCache &data_cache) : data_cache_(data_cache) {}
  double GetCost(const State &x, const Control & /*u*/,
                 const size_t step) const override;
  void GetGradientHessian(const State &x, const Control & /*u*/,
                          const size_t step, dx *const dx, du *const /*du*/,
                          ddx *const ddx, dxu *const /*dxu*/,
                          ddu *const /*ddu*/) const override;
  std::uint8_t GetId() const override {
    return static_cast<std::uint8_t>(VREF_COST);
  }

 private:
  const SptDataCache &data_cache_;
};

class ARefCost : public ilqr_solver::CostBase<state, input> {
 public:
  explicit ARefCost(const SptDataCache &data_cache) : data_cache_(data_cache) {}
  double GetCost(const State &x, const Control & /*u*/,
                 const size_t step) const override;
  void GetGradientHessian(const State &x, const Control & /*u*/,
                          const size_t step, dx *const dx, du *const /*du*/,
                          ddx *const ddx, dxu *const /*dxu*/,
                          ddu *const /*ddu*/) const override;
  std::uint8_t GetId() const override {
    return static_cast<std::uint8_t>(AREF_COST);
  }

 private:
  const SptDataCache &data_cache_;
};

// Velocity limit cost
class VLimitCost : public ilqr_solver::CostBase<state, input> {
 public:
  explicit VLimitCost(const SptDataCache &data_cache)
      : data_cache_(data_cache) {}
  double GetCost(const State &x, const Control & /*u*/,
                 const size_t step) const override;
  void GetGradientHessian(const State &x, const Control & /*u*/,
                          const size_t step, dx *const dx, du *const /*du*/,
                          ddx *const ddx, dxu *const /*dxu*/,
                          ddu *const /*ddu*/) const override;
  std::uint8_t GetId() const override { return VLIMIT_COST; }

 private:
  const SptDataCache &data_cache_;
};

// Curvature cost
class CurvatureCost : public ilqr_solver::CostBase<state, input> {
 public:
  explicit CurvatureCost(const SptDataCache &data_cache)
      : data_cache_(data_cache) {}
  double GetCost(const State &x, const Control & /*u*/,
                 const size_t step) const override;
  void GetGradientHessian(const State &x, const Control & /*u*/,
                          const size_t step, dx *const dx, du *const /*du*/,
                          ddx *const ddx, dxu *const /*dxu*/,
                          ddu *const /*ddu*/) const override;
  std::uint8_t GetId() const override { return CURVATURE_COST; }

 private:
  const SptDataCache &data_cache_;
};
// Ddkappa cost
class DdkappaCost : public ilqr_solver::CostBase<state, input> {
 public:
  explicit DdkappaCost(const SptDataCache &data_cache)
      : data_cache_(data_cache) {}
  double GetCost(const State & /*x*/, const Control &u,
                 const size_t step) const override;
  void GetGradientHessian(const State & /*x*/, const Control &u,
                          const size_t step, dx *const /*dx*/, du *const du,
                          ddx *const /*ddx*/, dxu *const /*dxu*/,
                          ddu *const ddu) const override;
  std::uint8_t GetId() const override { return DDKAPPA_COST; }

 private:
  const SptDataCache &data_cache_;
};

// Jerk cost
class JerkCost : public ilqr_solver::CostBase<state, input> {
 public:
  explicit JerkCost(const SptDataCache &data_cache) : data_cache_(data_cache) {}
  double GetCost(const State &x, const Control & /*u*/,
                 const size_t /*step*/) const override;
  void GetGradientHessian(const State &x, const Control & /*u*/,
                          const size_t /*step*/, dx *const dx, du *const /*du*/,
                          ddx *const ddx, dxu *const /*dxu*/,
                          ddu *const /*ddu*/) const override;
  std::uint8_t GetId() const override { return JERK_COST; }

 private:
  const SptDataCache &data_cache_;
};

// Djerk cost
class DjerkCost : public ilqr_solver::CostBase<state, input> {
 public:
  explicit DjerkCost(const SptDataCache &data_cache)
      : data_cache_(data_cache) {}
  double GetCost(const State & /*x*/, const Control &u,
                 const size_t step) const override;
  void GetGradientHessian(const State & /*x*/, const Control &u,
                          const size_t step, dx *const /*dx*/, du *const du,
                          ddx *const /*ddx*/, dxu *const /*dxu*/,
                          ddu *const ddu) const override;
  std::uint8_t GetId() const override { return DJERK_COST; }

 private:
  const SptDataCache &data_cache_;
};

// Lateral acceleration cost
class LatAccCost : public ilqr_solver::CostBase<state, input> {
 public:
  explicit LatAccCost(const SptDataCache &data_cache)
      : data_cache_(data_cache) {}
  double GetCost(const State &x, const Control & /*u*/,
                 const size_t step) const override;
  void GetGradientHessian(const State &x, const Control & /*u*/,
                          const size_t step, dx *const dx, du *const /*du*/,
                          ddx *const ddx, dxu *cosnt /*dxu*/,
                          ddu *const /*ddu*/) const override;
  std::uint8_t GetId() const override { return LAT_ACC_COST; }

 private:
  void GetGradientHessianElementwise(const State &x, const size_t step,
                                     const double lat_acc, const double scale,
                                     const double threshold, dx *const dx,
                                     ddx *const ddx) const;
  const SptDataCache &data_cache_;
};

// Lateral jerk cost
class LatJerkCost : public ilqr_solver::CostBase<state, input> {
 public:
  explicit LatJerkCost(const SptDataCache &data_cache)
      : data_cache_(data_cache) {}
  double GetCost(const State &x, const Control & /*u*/,
                 const size_t step) const override;
  void GetGradientHessian(const State &x, const Control & /*u*/,
                          const size_t step, dx *const dx, du *const /*du*/,
                          ddx *const ddx, dxu *cosnt /*dxu*/,
                          ddu *const /*ddu*/) const override;
  std::uint8_t GetId() const override { return LAT_JERK_COST; }

 private:
  void GetGradientHessianElementwise(const State &x, const size_t step,
                                     const double lat_jerk, const double scale,
                                     const double threshold, dx *const dx,
                                     ddx *const ddx) const;
  const SptDataCache &data_cache_;
};

// Longitudinal acceleration cost
class LonAccCost : public ilqr_solver::CostBase<state, input> {
 public:
  explicit LonAccCost(const SptDataCache &data_cache)
      : data_cache_(data_cache) {}
  double GetCost(const State &x, const Control & /*u*/,
                 const size_t step) const override;
  void GetGradientHessian(const State &x, const Control & /*u*/,
                          const size_t step, dx *const dx, du *const /*du*/,
                          ddx *const ddx, dxu *cosnt /*dxu*/,
                          ddu *const /*ddu*/) const override;
  std::uint8_t GetId() const override { return LON_ACC_COST; }

 private:
  const SptDataCache &data_cache_;
};

// Longitudinal jerk cost
class LonJerkCost : public ilqr_solver::CostBase<state, input> {
 public:
  explicit LonJerkCost(const SptDataCache &data_cache)
      : data_cache_(data_cache) {}
  double GetCost(const State &x, const Control & /*u*/,
                 const size_t step) const override;
  void GetGradientHessian(const State &x, const Control & /*u*/,
                          const size_t step, dx *const dx, du *const /*du*/,
                          ddx *const ddx, dxu *cosnt /*dxu*/,
                          ddu *const /*ddu*/) const override;
  std::uint8_t GetId() const override { return LON_JERK_COST; }

 private:
  void GetGradientHessianElementwise(const State &x, const size_t step,
                                     const double lon_jerk, const double scale,
                                     dx *const dx, ddx *const ddx) const;
  const SptDataCache &data_cache_;
};

// State consistency cost
class ConsistencyCost : public ilqr_solver::CostBase<state, input> {
 public:
  explicit ConsistencyCost(const SptDataCache &data_cache)
      : data_cache_(data_cache) {}
  double GetCost(const State &x, const Control & /*u*/,
                 const size_t step) const override;
  void GetGradientHessian(const State &x, const Control & /*u*/,
                          const size_t step, dx *const dx, du *const /*du*/,
                          ddx *const ddx, dxu *cosnt /*dxu*/,
                          ddu *const /*ddu*/) const override;
  std::uint8_t GetId() const override { return CONSISTENCY_COST; }

 private:
  const SptDataCache &data_cache_;
};

// CurvatureRef cost
class CurvatureRefCost : public ilqr_solver::CostBase<state, input> {
 public:
  explicit CurvatureRefCost(const SptDataCache &data_cache)
      : data_cache_(data_cache) {}
  double GetCost(const State &x, const Control & /*u*/,
                 const size_t step) const override;
  void GetGradientHessian(const State &x, const Control & /*u*/,
                          const size_t step, dx *const dx, du *const /*du*/,
                          ddx *const ddx, dxu *const /*dxu*/,
                          ddu *const /*ddu*/) const override;
  std::uint8_t GetId() const override { return CURVATURE_REF_COST; }

 private:
  const SptDataCache &data_cache_;
};

// Curvature change rate cost
class DkappaCost : public ilqr_solver::CostBase<state, input> {
 public:
  explicit DkappaCost(const SptDataCache &data_cache)
      : data_cache_(data_cache) {}
  double GetCost(const State &x, const Control & /*u*/,
                 const size_t step) const override;
  void GetGradientHessian(const State &x, const Control & /*u*/,
                          const size_t step, dx *const dx, du *const /*du*/,
                          ddx *const ddx, dxu *const /*dxu*/,
                          ddu *const /*ddu*/) const override;
  std::uint8_t GetId() const override { return DKAPPA_COST; }

 private:
  const SptDataCache &data_cache_;
};

}  // namespace spt
}  // namespace e2e_noa
