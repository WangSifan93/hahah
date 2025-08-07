#include "spatio_temporal_cost.h"

#include "ilqr_model.h"
#include "spatio_temporal_data.h"
#include "spatio_temporal_model.h"

namespace e2e_noa {
namespace spt {

using npp::pnc::NormalizeAngle;

void LatIlqrCommonTerm::ComputeCircleInfo(
    const State &x, const size_t step,
    std::vector<CircleAndMatchBoundaryInfo> *circle_infos) {
  for (auto &cir : *circle_infos) {
    cir.SetX(x[X] + (*data_cache_)(COS_THETA, step) * cir.Length() -
             (*data_cache_)(SIN_THETA, step) * cir.Width());
    cir.SetY(x[Y] + (*data_cache_)(SIN_THETA, step) * cir.Length() +
             (*data_cache_)(COS_THETA, step) * cir.Width());
  }
}

double LatIlqrCommonTerm::GetCostEachPoint(const double weight,
                                           const double threshold,
                                           const double offset) {
  return HuberLoss(weight, offset, threshold);
}

double HeadingCost::GetCost(const State & /*x*/, const Control & /*u*/,
                            const size_t step) const {
  const double scale =
      data_cache_(DYNAMIC_HEADING_SCALE) * Square(data_cache_(TARGET_V, step)) +
      data_cache_(BASE_HEADING_SCALE);
  const auto &rear_circle_data = data_cache_.RearCircleAndBoundary(step);
  return HuberLoss(scale, rear_circle_data(DESIRE_HEADING_DIFF),
                   data_cache_(HEADING_HUBER_THRESHOLD));
}
void HeadingCost::GetGradientHessian(const State & /*x*/, const Control & /*u*/,
                                     const size_t step, dx *const dx,
                                     du *const /*du*/, ddx *const ddx,
                                     dxu *const /*dxu*/,
                                     ddu *const /*ddu*/) const {
  const double scale =
      data_cache_(DYNAMIC_HEADING_SCALE) * Square(data_cache_(TARGET_V, step)) +
      data_cache_(BASE_HEADING_SCALE);
  const auto &rear_circle_data = data_cache_.RearCircleAndBoundary(step);
  const double grad = 1.0;
  const double hess = 0.0;
  (*dx)(THETA) += HuberLossGrad1D(scale, rear_circle_data(DESIRE_HEADING_DIFF),
                                  data_cache_(HEADING_HUBER_THRESHOLD), grad);

  (*ddx)(THETA, THETA) +=
      HuberLossHess1D(scale, rear_circle_data(DESIRE_HEADING_DIFF),
                      data_cache_(HEADING_HUBER_THRESHOLD), grad, grad, hess);
}

double SRefCost::GetCost(const State & /*x*/, const Control & /*u*/,
                         const size_t step) const {
  const auto &rear_circle_data = data_cache_.RearCircleAndBoundary(step);
  return QuadraticCost(data_cache_(S_REF_SCALE), rear_circle_data(S_DIFF));
}
void SRefCost::GetGradientHessian(const State & /*x*/, const Control & /*u*/,
                                  const size_t step, dx *const dx,
                                  du *const /*du*/, ddx *const ddx,
                                  dxu *const /*dxu*/,
                                  ddu *const /*ddu*/) const {
  const auto &rear_circle_data = data_cache_.RearCircleAndBoundary(step);
  const double scale = data_cache_(S_REF_SCALE);
  const double ref_sin_theta = rear_circle_data(M_REF_SIN_THETA);
  const double ref_cos_theta = rear_circle_data(M_REF_COS_THETA);
  const double s_diff = rear_circle_data(S_DIFF);

  (*dx)(X) += ref_cos_theta * s_diff * scale;
  (*dx)(Y) += ref_sin_theta * s_diff * scale;

  const double lxx_x_y_temp = ref_cos_theta * ref_sin_theta * scale;

  (*ddx)(X, X) += Square(ref_cos_theta) * scale;
  (*ddx)(Y, Y) += Square(ref_sin_theta) * scale;
  (*ddx)(X, Y) += lxx_x_y_temp;
  (*ddx)(Y, X) += lxx_x_y_temp;
}

double VRefCost::GetCost(const State &x, const Control & /*u*/,
                         const size_t step) const {
  const double v_diff = x[V] - data_cache_(TARGET_V, step);
  return QuadraticCost(data_cache_(V_REF_SCALE, step), v_diff);
}
void VRefCost::GetGradientHessian(const State &x, const Control & /*u*/,
                                  const size_t step, dx *const dx,
                                  du *const /*du*/, ddx *const ddx,
                                  dxu *const /*dxu*/,
                                  ddu *const /*ddu*/) const {
  const double scale = data_cache_(V_REF_SCALE, step);
  const double v_diff = x[V] - data_cache_(TARGET_V, step);
  (*dx)(V) += scale * v_diff;
  (*ddx)(V, V) += scale;
}

double ARefCost::GetCost(const State &x, const Control & /*u*/,
                         const size_t step) const {
  const double a_diff = x[ACC] - data_cache_(TARGET_A, step);
  return QuadraticCost(data_cache_(LON_ACC_REF_SCALE, step), a_diff);
}
void ARefCost::GetGradientHessian(const State &x, const Control & /*u*/,
                                  const size_t step, dx *const dx,
                                  du *const /*du*/, ddx *const ddx,
                                  dxu *const /*dxu*/,
                                  ddu *const /*ddu*/) const {
  const double scale = data_cache_(LON_ACC_REF_SCALE, step);
  const double a_diff = x[ACC] - data_cache_(TARGET_A, step);
  (*dx)(ACC) += scale * a_diff;
  (*ddx)(ACC, ACC) += scale;
}

double VLimitCost::GetCost(const State &x, const Control & /*u*/,
                           const size_t step) const {
  double cost = 0.0;
  const double v = x[V];
  const double limit_scale = data_cache_(V_LIMIT_SCALE);
  const double threshold = data_cache_(V_LIMIT_HUBER_THRESHOLD);
  if (v > data_cache_(V_LIMIT_UPPER, step)) {
    const double v_diff = v - data_cache_(V_LIMIT_UPPER, step);
    cost += HuberLoss(limit_scale, v_diff, threshold);
  } else if (v < data_cache_(V_LIMIT_LOWER, step)) {
    const double v_diff = v - data_cache_(V_LIMIT_LOWER, step);
    cost += HuberLoss(limit_scale, v_diff, threshold);
  }
  return cost;
}
void VLimitCost::GetGradientHessian(const State &x, const Control & /*u*/,
                                    const size_t step, dx *const dx,
                                    du *const /*du*/, ddx *const ddx,
                                    dxu *const /*dxu*/,
                                    ddu *const /*ddu*/) const {
  const double v = x[V];
  const double limit_scale = data_cache_(V_LIMIT_SCALE);
  if (v > data_cache_(V_LIMIT_UPPER, step)) {
    const double v_diff = v - data_cache_(V_LIMIT_UPPER, step);
    const double grad_v = 1.0;
    const double hess_v_v = 0.0;
    (*dx)(V) += HuberLossGrad1D(limit_scale, v_diff,
                                data_cache_(V_LIMIT_HUBER_THRESHOLD), grad_v);
    (*ddx)(V, V) += HuberLossHess1D(limit_scale, v_diff,
                                    data_cache_(V_LIMIT_HUBER_THRESHOLD),
                                    grad_v, grad_v, hess_v_v);
  } else if (v < data_cache_(V_LIMIT_LOWER, step)) {
    const double v_diff = v - data_cache_(V_LIMIT_LOWER, step);
    const double grad_v = 1.0;
    const double hess_v_v = 0.0;
    (*dx)(V) += HuberLossGrad1D(limit_scale, v_diff,
                                data_cache_(V_LIMIT_HUBER_THRESHOLD), grad_v);
    (*ddx)(V, V) += HuberLossHess1D(limit_scale, v_diff,
                                    data_cache_(V_LIMIT_HUBER_THRESHOLD),
                                    grad_v, grad_v, hess_v_v);
  }
}

double CurvatureCost::GetCost(const State &x, const Control & /*u*/,
                              const size_t step) const {
  double cost = 0.0;
  const double kappa = x[KAPPA];
  cost += QuadraticCost(data_cache_(KAPPA_SCALE), kappa);

  const double limit_scale = data_cache_(KAPPA_LIMIT_SCALE);
  if (kappa > data_cache_(KAPPA_LIMIT_UPPER, step)) {
    const double kappa_diff = kappa - data_cache_(KAPPA_LIMIT_UPPER, step);
    cost += QuadraticCost(limit_scale, kappa_diff);
  } else if (kappa < data_cache_(KAPPA_LIMIT_LOWER, step)) {
    const double kappa_diff = kappa - data_cache_(KAPPA_LIMIT_LOWER, step);
    cost += QuadraticCost(limit_scale, kappa_diff);
  }
  return cost;
}
void CurvatureCost::GetGradientHessian(const State &x, const Control & /*u*/,
                                       const size_t step, dx *const dx,
                                       du *const /*du*/, ddx *const ddx,
                                       dxu *const /*dxu*/,
                                       ddu *const /*ddu*/) const {
  const double kappa = x[KAPPA];
  const double scale = data_cache_(KAPPA_SCALE);
  (*dx)(KAPPA) += scale * kappa;
  (*ddx)(KAPPA, KAPPA) += scale;

  const double limit_scale = data_cache_(KAPPA_LIMIT_SCALE);
  if (kappa > data_cache_(KAPPA_LIMIT_UPPER, step)) {
    const double kappa_diff = kappa - data_cache_(KAPPA_LIMIT_UPPER, step);
    (*dx)(KAPPA) += limit_scale * kappa_diff;
    (*ddx)(KAPPA, KAPPA) += limit_scale;
  } else if (kappa < data_cache_(KAPPA_LIMIT_LOWER, step)) {
    const double kappa_diff = kappa - data_cache_(KAPPA_LIMIT_LOWER, step);
    (*dx)(KAPPA) += limit_scale * kappa_diff;
    (*ddx)(KAPPA, KAPPA) += limit_scale;
  }
}

double CurvatureRefCost::GetCost(const State &x, const Control & /*u*/,
                                 const size_t step) const {
  double cost = 0.0;
  const auto &rear_circle_data = data_cache_.RearCircleAndBoundary(step);
  const double kappa = x[KAPPA] - rear_circle_data(M_REF_KAPPA);
  cost += QuadraticCost(data_cache_(KAPPA_REF_SCALE), kappa);
  return cost;
}

void CurvatureRefCost::GetGradientHessian(const State &x, const Control & /*u*/,
                                          const size_t step, dx *const dx,
                                          du *const /*du*/, ddx *const ddx,
                                          dxu *const /*dxu*/,
                                          ddu *const /*ddu*/) const {
  const auto &rear_circle_data = data_cache_.RearCircleAndBoundary(step);
  const double kappa = x[KAPPA] - rear_circle_data(M_REF_KAPPA);
  const double scale = data_cache_(KAPPA_REF_SCALE);
  (*dx)(KAPPA) += scale * kappa;
  (*ddx)(KAPPA, KAPPA) += scale;
}

double DkappaCost::GetCost(const State &x, const Control & /*u*/,
                           const size_t step) const {
  double cost = 0.0;
  const double dkappa = x[DKAPPA];
  cost += QuadraticCost(data_cache_(DKAPPA_SCALE), dkappa);

  const double limit_scale = data_cache_(DKAPPA_LIMIT_SCALE);
  if (dkappa > data_cache_(DKAPPA_LIMIT_UPPER, step)) {
    const double dkappa_diff = dkappa - data_cache_(DKAPPA_LIMIT_UPPER, step);
    cost += QuadraticCost(limit_scale, dkappa_diff);
  } else if (dkappa < data_cache_(DKAPPA_LIMIT_LOWER, step)) {
    const double dkappa_diff = dkappa - data_cache_(DKAPPA_LIMIT_LOWER, step);
    cost += QuadraticCost(limit_scale, dkappa_diff);
  }
  return cost;
}
void DkappaCost::GetGradientHessian(const State &x, const Control & /*u*/,
                                    const size_t step, dx *const dx,
                                    du *const /*du*/, ddx *const ddx,
                                    dxu *const /*dxu*/,
                                    ddu *const /*ddu*/) const {
  const double dkappa = x[DKAPPA];
  const double scale = data_cache_(DKAPPA_SCALE);
  (*dx)(DKAPPA) += scale * dkappa;
  (*ddx)(DKAPPA, DKAPPA) += scale;

  const double limit_scale = data_cache_(DKAPPA_LIMIT_SCALE);
  if (dkappa > data_cache_(DKAPPA_LIMIT_UPPER, step)) {
    const double dkappa_diff = dkappa - data_cache_(DKAPPA_LIMIT_UPPER, step);
    (*dx)(DKAPPA) += limit_scale * dkappa_diff;
    (*ddx)(DKAPPA, DKAPPA) += limit_scale;
  } else if (dkappa < data_cache_(DKAPPA_LIMIT_LOWER, step)) {
    const double dkappa_diff = dkappa - data_cache_(DKAPPA_LIMIT_LOWER, step);
    (*dx)(DKAPPA) += limit_scale * dkappa_diff;
    (*ddx)(DKAPPA, DKAPPA) += limit_scale;
  }
}

double DdkappaCost::GetCost(const State & /*x*/, const Control &u,
                            const size_t step) const {
  if (data_cache_(TERMINAL_FLAG, step) > 0.) {
    return 0.0;
  }

  double cost = 0.0;
  const double ddkappa = u[DDKAPPA];
  cost += QuadraticCost(data_cache_(DDKAPPA_SCALE, step), ddkappa);

  const double ddkappa_limit = data_cache_(DDKAPPA_LIMIT, step);
  const double limit_scale = data_cache_(DDKAPPA_LIMIT_SCALE);
  if (ddkappa > ddkappa_limit) {
    const double ddkappa_diff = ddkappa - ddkappa_limit;
    cost += QuadraticCost(limit_scale, ddkappa_diff);
  } else if (ddkappa < -ddkappa_limit) {
    const double ddkappa_diff = ddkappa + ddkappa_limit;
    cost += QuadraticCost(limit_scale, ddkappa_diff);
  }
  return cost;
}
void DdkappaCost::GetGradientHessian(const State & /*x*/, const Control &u,
                                     const size_t step, dx *const /*dx*/,
                                     du *const du, ddx *const /*ddx*/,
                                     dxu *const /*dxu*/, ddu *const ddu) const {
  if (data_cache_(TERMINAL_FLAG, step) > 0.) {
    return;
  }

  const double ddkappa = u[DDKAPPA];
  const double scale = data_cache_(DDKAPPA_SCALE, step);
  (*du)(DDKAPPA) += scale * ddkappa;
  (*ddu)(DDKAPPA, DDKAPPA) += scale;

  const double ddkappa_limit = data_cache_(DDKAPPA_LIMIT, step);
  const double limit_scale = data_cache_(DDKAPPA_LIMIT_SCALE);
  if (ddkappa > ddkappa_limit) {
    const double ddkappa_diff = ddkappa - ddkappa_limit;
    (*du)(DDKAPPA) += limit_scale * ddkappa_diff;
    (*ddu)(DDKAPPA, DDKAPPA) += limit_scale;
  } else if (ddkappa < -ddkappa_limit) {
    const double ddkappa_diff = ddkappa + ddkappa_limit;
    (*du)(DDKAPPA) += limit_scale * ddkappa_diff;
    (*ddu)(DDKAPPA, DDKAPPA) += limit_scale;
  }
}

double JerkCost::GetCost(const State &x, const Control & /*u*/,
                         const size_t /*step*/) const {
  double cost = 0.0;
  const double jerk = x[JERK];
  cost += QuadraticCost(data_cache_(JERK_SCALE), jerk);

  const double limit_scale = data_cache_(JERK_LIMIT_SCALE);
  if (jerk > data_cache_(LON_JERK_LIMIT_UPPER)) {
    const double jerk_diff = jerk - data_cache_(LON_JERK_LIMIT_UPPER);
    cost += QuadraticCost(limit_scale, jerk_diff);
  } else if (jerk < data_cache_(LON_JERK_LIMIT_LOWER)) {
    const double jerk_diff = jerk - data_cache_(LON_JERK_LIMIT_LOWER);
    cost += QuadraticCost(limit_scale, jerk_diff);
  }
  return cost;
}
void JerkCost::GetGradientHessian(const State &x, const Control & /*u*/,
                                  const size_t /*step*/, dx *const dx,
                                  du *const /*du*/, ddx *const ddx,
                                  dxu *const /*dxu*/,
                                  ddu *const /*ddu*/) const {
  const double jerk = x[JERK];
  const double scale = data_cache_(JERK_SCALE);
  (*dx)(JERK) += scale * jerk;
  (*ddx)(JERK, JERK) += scale;

  const double limit_scale = data_cache_(JERK_LIMIT_SCALE);
  if (jerk > data_cache_(LON_JERK_LIMIT_UPPER)) {
    const double jerk_diff = jerk - data_cache_(LON_JERK_LIMIT_UPPER);
    (*dx)(JERK) += limit_scale * jerk_diff;
    (*ddx)(JERK, JERK) += limit_scale;
  } else if (jerk < data_cache_(LON_JERK_LIMIT_LOWER)) {
    const double jerk_diff = jerk - data_cache_(LON_JERK_LIMIT_LOWER);
    (*dx)(JERK) += limit_scale * jerk_diff;
    (*ddx)(JERK, JERK) += limit_scale;
  }
}

double DjerkCost::GetCost(const State & /*x*/, const Control &u,
                          const size_t step) const {
  if (data_cache_(TERMINAL_FLAG, step) > 0.) {
    return 0.0;
  }

  double cost = 0.0;
  const double djerk = u[DJERK];
  cost += QuadraticCost(data_cache_(DJERK_SCALE), djerk);

  const double djerk_limit = data_cache_(DJERK_LIMIT);
  const double limit_scale = data_cache_(DJERK_LIMIT_SCALE);
  if (djerk > djerk_limit) {
    const double djerk_diff = djerk - djerk_limit;
    cost += QuadraticCost(limit_scale, djerk_diff);
  } else if (djerk < -djerk_limit) {
    const double djerk_diff = djerk + djerk_limit;
    cost += QuadraticCost(limit_scale, djerk_diff);
  }
  return cost;
}
void DjerkCost::GetGradientHessian(const State & /*x*/, const Control &u,
                                   const size_t step, dx *const /*dx*/,
                                   du *const du, ddx *const /*ddx*/,
                                   dxu *const /*dxu*/, ddu *const ddu) const {
  if (data_cache_(TERMINAL_FLAG, step) > 0.) {
    return;
  }

  const double djerk = u[DJERK];
  const double scale = data_cache_(DJERK_SCALE);
  (*du)(DJERK) += scale * djerk;
  (*ddu)(DJERK, DJERK) += scale;

  const double djerk_limit = data_cache_(DJERK_LIMIT);
  const double limit_scale = data_cache_(DJERK_LIMIT_SCALE);
  if (djerk > djerk_limit) {
    const double djerk_diff = djerk - djerk_limit;
    (*du)(DJERK) += limit_scale * djerk_diff;
    (*ddu)(DJERK, DJERK) += limit_scale;
  } else if (djerk < -djerk_limit) {
    const double djerk_diff = djerk + djerk_limit;
    (*du)(DJERK) += limit_scale * djerk_diff;
    (*ddu)(DJERK, DJERK) += limit_scale;
  }
}

}  // namespace spt
}  // namespace e2e_noa
