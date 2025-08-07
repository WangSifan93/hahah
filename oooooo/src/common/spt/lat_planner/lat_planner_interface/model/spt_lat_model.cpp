#include "common/spt/lat_planner/lat_planner_interface/model/spt_lat_model.h"

namespace e2e_noa {
namespace spt {

constexpr double kHalf = 0.5;
constexpr double kTwo = 2.0;
constexpr double kThree = 3.0;
constexpr double kOneThird = 1.0 / kThree;
using e2e_noa::spt::ResizeAndResetEigen;

SptLatModel::State SptLatModel::UpdateDynamicsOneStep(
    const State &x, const Control &u, const std::size_t step) const {
  const double dt = spt_data_.GetSptStepData(step, SptStepData::DT);
  return UpdateDynamicsWithDt(x, u, dt);
}

SptLatModel::State SptLatModel::UpdateDynamicsWithDt(const State &x,
                                                     const Control &u,
                                                     const double dt) {
  std::array<double, STATE_SIZE> k1{};
  std::array<double, STATE_SIZE> k2{};
  std::array<double, STATE_SIZE> k3{};
  std::array<double, STATE_SIZE> k4{};
  k1.fill(0.0), k2.fill(0.0), k3.fill(0.0), k4.fill(0.0);

  const double dt_2 = dt * kHalf;
  const double dt_3 = dt * kOneThird;
  const double dt_6 = dt_3 * kHalf;

  const double theta = x[StateIndex::THETA];
  const double kappa = x[StateIndex::KAPPA];
  const double dkappa = x[StateIndex::DKAPPA];
  const double v = x[StateIndex::V];
  const double a = x[StateIndex::ACC];
  const double jerk = x[StateIndex::JERK];
  const double ddkappa = u[ControlIndex::DDKAPPA];
  const double djerk = u[ControlIndex::DJERK];

  k1[X] = v * std::cos(theta);
  k1[Y] = v * std::sin(theta);
  k1[THETA] = v * kappa;
  k1[KAPPA] = dkappa;
  k1[DKAPPA] = ddkappa;
  k1[V] = a;
  k1[ACC] = jerk;
  k1[JERK] = djerk;

  const double theta1 = theta + dt_2 * k1[THETA];
  const double kappa1 = kappa + dt_2 * k1[KAPPA];
  const double dkappa1 = dkappa + dt_2 * k1[DKAPPA];
  const double v1 = v + dt_2 * k1[V];
  const double a1 = a + dt_2 * k1[ACC];
  const double jerk1 = jerk + dt_2 * k1[JERK];
  k2[X] = v1 * std::cos(theta1);
  k2[Y] = v1 * std::sin(theta1);
  k2[THETA] = v1 * kappa1;
  k2[KAPPA] = dkappa1;
  k2[DKAPPA] = ddkappa;
  k2[V] = a1;
  k2[ACC] = jerk1;
  k2[JERK] = djerk;

  const double theta2 = theta + dt_2 * k2[THETA];
  const double kappa2 = kappa + dt_2 * k2[KAPPA];
  const double dkappa2 = dkappa + dt_2 * k2[DKAPPA];
  const double v2 = v + dt_2 * k2[V];
  const double a2 = a + dt_2 * k2[ACC];
  const double jerk2 = jerk + dt_2 * k2[JERK];
  k3[X] = v2 * std::cos(theta2);
  k3[Y] = v2 * std::sin(theta2);
  k3[THETA] = v2 * kappa2;
  k3[KAPPA] = dkappa2;
  k3[DKAPPA] = ddkappa;
  k3[V] = a2;
  k3[ACC] = jerk2;
  k3[JERK] = djerk;

  const double theta3 = theta + dt * k3[THETA];
  const double kappa3 = kappa + dt * k3[KAPPA];
  const double dkappa3 = dkappa + dt * k3[DKAPPA];
  const double v3 = v + dt * k3[V];
  const double a3 = a + dt * k3[ACC];
  const double jerk3 = jerk + dt * k3[JERK];
  k4[X] = v3 * std::cos(theta3);
  k4[Y] = v3 * std::sin(theta3);
  k4[THETA] = v3 * kappa3;
  k4[KAPPA] = dkappa3;
  k4[DKAPPA] = ddkappa;
  k4[V] = a3;
  k4[ACC] = jerk3;
  k4[JERK] = djerk;

  State output_state = x;
  for (std::size_t i = 0U; i < STATE_SIZE; ++i) {
    output_state[static_cast<long>(i)] +=
        (k1[i] + kTwo * k2[i] + kTwo * k3[i] + k4[i]) * dt_6;
  }
  return output_state;
}

void SptLatModel::CalRk4AndDynamicsDerivatives(const State &x, const Control &u,
                                               const double dt, Fx *f_x,
                                               Fu *f_u) {
  const double dt_2 = dt * kHalf;
  const double dt_3 = dt * kOneThird;
  const double dt_6 = dt_3 * kHalf;

  std::array<double, STATE_SIZE> k1{};
  std::array<double, STATE_SIZE> k2{};
  std::array<double, STATE_SIZE> k3{};
  std::array<double, STATE_SIZE> k4{};
  k1.fill(0.0), k2.fill(0.0), k3.fill(0.0), k4.fill(0.0);

  const double theta = x[StateIndex::THETA];
  const double kappa = x[StateIndex::KAPPA];
  const double dkappa = x[StateIndex::DKAPPA];
  const double v = x[StateIndex::V];
  const double a = x[StateIndex::ACC];
  const double jerk = x[StateIndex::JERK];
  const double ddkappa = u[ControlIndex::DDKAPPA];
  const double djerk = u[ControlIndex::DJERK];
  const double cos_theta = std::cos(theta);
  const double sin_theta = std::sin(theta);

  k1[X] = v * cos_theta;
  k1[Y] = v * sin_theta;
  k1[THETA] = v * kappa;
  k1[KAPPA] = dkappa;
  k1[DKAPPA] = ddkappa;
  k1[V] = a;
  k1[ACC] = jerk;
  k1[JERK] = djerk;

  const double theta1 = theta + dt_2 * k1[THETA];
  const double kappa1 = kappa + dt_2 * k1[KAPPA];
  const double dkappa1 = dkappa + dt_2 * k1[DKAPPA];
  const double v1 = v + dt_2 * k1[V];
  const double a1 = a + dt_2 * k1[ACC];
  const double jerk1 = jerk + dt_2 * k1[JERK];
  const double cos_theta1 = std::cos(theta1);
  const double sin_theta1 = std::sin(theta1);
  k2[X] = v1 * cos_theta1;
  k2[Y] = v1 * sin_theta1;
  k2[THETA] = v1 * kappa1;
  k2[KAPPA] = dkappa1;
  k2[DKAPPA] = ddkappa;
  k2[V] = a1;
  k2[ACC] = jerk1;
  k2[JERK] = djerk;

  const double theta2 = theta + dt_2 * k2[THETA];
  const double kappa2 = kappa + dt_2 * k2[KAPPA];
  const double dkappa2 = dkappa + dt_2 * k2[DKAPPA];
  const double v2 = v + dt_2 * k2[V];
  const double a2 = a + dt_2 * k2[ACC];
  const double jerk2 = jerk + dt_2 * k2[JERK];
  const double cos_theta2 = std::cos(theta2);
  const double sin_theta2 = std::sin(theta2);
  k3[X] = v2 * cos_theta2;
  k3[Y] = v2 * sin_theta2;
  k3[THETA] = v2 * kappa2;
  k3[KAPPA] = dkappa2;
  k3[DKAPPA] = ddkappa;
  k3[V] = a2;
  k3[ACC] = jerk2;
  k3[JERK] = djerk;

  const double theta3 = theta + dt * k3[THETA];
  const double kappa3 = kappa + dt * k3[KAPPA];
  const double dkappa3 = dkappa + dt * k3[DKAPPA];
  const double v3 = v + dt * k3[V];
  const double a3 = a + dt * k3[ACC];
  const double jerk3 = jerk + dt * k3[JERK];
  const double cos_theta3 = std::cos(theta3);
  const double sin_theta3 = std::sin(theta3);
  k4[X] = v3 * cos_theta3;
  k4[Y] = v3 * sin_theta3;
  k4[THETA] = v3 * kappa3;
  k4[KAPPA] = dkappa3;
  k4[DKAPPA] = ddkappa;
  k4[V] = a3;
  k4[ACC] = jerk3;
  k4[JERK] = djerk;

  (void)f_x->setIdentity();
  (void)f_u->setZero();
  (*f_x)(X, THETA) += -k1[Y] * dt_6;
  (*f_x)(X, V) += cos_theta * dt_6;
  (*f_x)(Y, THETA) += k1[X] * dt_6;
  (*f_x)(Y, V) += sin_theta * dt_6;
  (*f_x)(THETA, KAPPA) += v * dt_6;
  (*f_x)(THETA, V) += kappa * dt_6;
  (*f_x)(KAPPA, DKAPPA) += dt_6;
  (*f_x)(V, ACC) += dt_6;
  (*f_x)(ACC, JERK) += dt_6;

  (*f_u)(DKAPPA, DDKAPPA) += dt_6;
  (*f_u)(JERK, DJERK) += dt_6;

  const double d_theta1_d_kappa = dt_2 * v;
  const double d_theta1_d_v = dt_2 * kappa;
  const double dt2_6 = dt_2 * dt_3;
  (*f_x)(X, THETA) += -k2[Y] * dt_3;
  (*f_x)(X, KAPPA) += -k2[Y] * d_theta1_d_kappa * dt_3;
  (*f_x)(X, V) += (cos_theta1 - k2[Y] * d_theta1_d_v) * dt_3;
  (*f_x)(X, ACC) += cos_theta1 * dt2_6;
  (*f_x)(Y, THETA) += k2[X] * dt_3;
  (*f_x)(Y, KAPPA) += k2[X] * d_theta1_d_kappa * dt_3;
  (*f_x)(Y, V) += (sin_theta1 + k2[X] * d_theta1_d_v) * dt_3;
  (*f_x)(Y, ACC) += sin_theta1 * dt2_6;
  (*f_x)(THETA, KAPPA) += v1 * dt_3;
  (*f_x)(THETA, DKAPPA) += v1 * dt2_6;
  (*f_x)(THETA, V) += kappa1 * dt_3;
  (*f_x)(THETA, ACC) += kappa1 * dt2_6;
  (*f_x)(KAPPA, DKAPPA) += dt_3;
  (*f_x)(V, ACC) += dt_3;
  (*f_x)(V, JERK) += dt2_6;
  (*f_x)(ACC, JERK) += dt_3;

  (*f_u)(KAPPA, DDKAPPA) += dt2_6;
  (*f_u)(DKAPPA, DDKAPPA) += dt_3;
  (*f_u)(ACC, DJERK) += dt2_6;
  (*f_u)(JERK, DJERK) += dt_3;

  const double d_theta2_d_kappa = dt_2 * v1;
  const double d_theta2_d_dkappa = dt_2 * d_theta2_d_kappa;
  const double d_theta2_d_v = dt_2 * kappa1;
  const double d_theta2_d_a = dt_2 * d_theta2_d_v;
  const double dt3_12 = dt_2 * dt2_6;
  (*f_x)(X, THETA) += -k3[Y] * dt_3;
  (*f_x)(X, KAPPA) += -k3[Y] * d_theta2_d_kappa * dt_3;
  (*f_x)(X, DKAPPA) += -k3[Y] * d_theta2_d_dkappa * dt_3;
  (*f_x)(X, V) += (cos_theta2 - k3[Y] * d_theta2_d_v) * dt_3;
  (*f_x)(X, ACC) += (cos_theta2 * dt_2 - k3[Y] * d_theta2_d_a) * dt_3;
  (*f_x)(X, JERK) += cos_theta2 * dt3_12;
  (*f_x)(Y, THETA) += k3[X] * dt_3;
  (*f_x)(Y, KAPPA) += k3[X] * d_theta2_d_kappa * dt_3;
  (*f_x)(Y, DKAPPA) += k3[X] * d_theta2_d_dkappa * dt_3;
  (*f_x)(Y, V) += (sin_theta2 + k3[X] * d_theta2_d_v) * dt_3;
  (*f_x)(Y, ACC) += (sin_theta2 * dt_2 + k3[X] * d_theta2_d_a) * dt_3;
  (*f_x)(Y, JERK) += sin_theta2 * dt3_12;
  (*f_x)(THETA, KAPPA) += v2 * dt_3;
  (*f_x)(THETA, DKAPPA) += v2 * dt2_6;
  (*f_x)(THETA, V) += kappa2 * dt_3;
  (*f_x)(THETA, ACC) += kappa2 * dt2_6;
  (*f_x)(THETA, JERK) += kappa2 * dt3_12;
  (*f_x)(KAPPA, DKAPPA) += dt_3;
  (*f_x)(V, ACC) += dt_3;
  (*f_x)(V, JERK) += dt2_6;
  (*f_x)(ACC, JERK) += dt_3;

  (*f_u)(THETA, DDKAPPA) += v2 * dt3_12;
  (*f_u)(KAPPA, DDKAPPA) += dt2_6;
  (*f_u)(DKAPPA, DDKAPPA) += dt_3;
  (*f_u)(V, DJERK) += dt3_12;
  (*f_u)(ACC, DJERK) += dt2_6;
  (*f_u)(JERK, DJERK) += dt_3;

  const double d_theta3_d_kappa = dt * v2;
  const double d_theta3_d_dkappa = dt_2 * d_theta3_d_kappa;
  const double d_theta3_d_ddkappa = dt_2 * d_theta3_d_dkappa;
  const double d_theta3_d_v = dt * kappa2;
  const double d_theta3_d_a = dt_2 * d_theta3_d_v;
  const double d_theta3_d_jerk = dt_2 * d_theta3_d_a;
  const double dt2_2 = dt * dt_2;
  (*f_x)(X, THETA) += -k4[Y] * dt_6;
  (*f_x)(X, KAPPA) += -k4[Y] * d_theta3_d_kappa * dt_6;
  (*f_x)(X, DKAPPA) += -k4[Y] * d_theta3_d_dkappa * dt_6;
  (*f_x)(X, V) += (cos_theta3 - k4[Y] * d_theta3_d_v) * dt_6;
  (*f_x)(X, ACC) += (cos_theta3 * dt - k4[Y] * d_theta3_d_a) * dt_6;
  (*f_x)(X, JERK) += (cos_theta3 * dt2_2 - k4[Y] * d_theta3_d_jerk) * dt_6;
  (*f_x)(Y, THETA) += k4[X] * dt_6;
  (*f_x)(Y, KAPPA) += k4[X] * d_theta3_d_kappa * dt_6;
  (*f_x)(Y, DKAPPA) += k4[X] * d_theta3_d_dkappa * dt_6;
  (*f_x)(Y, V) += (sin_theta3 + k4[X] * d_theta3_d_v) * dt_6;
  (*f_x)(Y, ACC) += (sin_theta3 * dt + k4[X] * d_theta3_d_a) * dt_6;
  (*f_x)(Y, JERK) += (sin_theta3 * dt2_2 + k4[X] * d_theta3_d_jerk) * dt_6;
  (*f_x)(THETA, KAPPA) += v3 * dt_6;
  (*f_x)(THETA, DKAPPA) += v3 * dt2_6;
  (*f_x)(THETA, V) += kappa3 * dt_6;
  (*f_x)(THETA, ACC) += kappa3 * dt2_6;
  (*f_x)(THETA, JERK) += kappa3 * dt3_12;
  (*f_x)(KAPPA, DKAPPA) += dt_6;
  (*f_x)(V, ACC) += dt_6;
  (*f_x)(V, JERK) += dt2_6;
  (*f_x)(ACC, JERK) += dt_6;

  const double dt4_24 = dt3_12 * dt_2;
  (*f_u)(X, DDKAPPA) += -k4[Y] * d_theta3_d_ddkappa * dt_6;
  (*f_u)(Y, DDKAPPA) += k4[X] * d_theta3_d_ddkappa * dt_6;
  (*f_u)(THETA, DDKAPPA) += v3 * dt3_12;
  (*f_u)(KAPPA, DDKAPPA) += dt2_6;
  (*f_u)(DKAPPA, DDKAPPA) += dt_6;
  (*f_u)(X, DJERK) += cos_theta3 * dt4_24;
  (*f_u)(Y, DJERK) += sin_theta3 * dt4_24;
  (*f_u)(THETA, DJERK) += kappa3 * dt4_24;
  (*f_u)(V, DJERK) += dt3_12;
  (*f_u)(ACC, DJERK) += dt2_6;
  (*f_u)(JERK, DJERK) += dt_6;
}

void SptLatModel::GetDynamicsDerivatives(const State &x, const Control &u,
                                         const std::size_t step, Fx *const f_x,
                                         Fu *const f_u) const {
  const double dt = spt_data_.GetSptStepData(step, SptStepData::DT);
  CalRk4AndDynamicsDerivatives(x, u, dt, f_x, f_u);
}

}  // namespace spt
}  // namespace e2e_noa
