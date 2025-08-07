/******************************************************************************
 * Copyright 2024 The zark Authors. All Rights Reserved.
 *****************************************************************************/

#include "apps/planning/src/common/math/polynomial_fit/polynomial_fit.h"

#include "apps/planning/src/common/log.h"
#include "vec2d.h"

namespace zark {
namespace planning {

PolynomialFit::PolynomialFit() {}
PolynomialFit::~PolynomialFit() {}

PolynomialFit::TrajPolynomial PolynomialFit::TrajPointsPolyFit(
    const DiscretizedTrajectory& traj_points,
    const zark::planning::common::VehicleState& vehicle_state) {
  PolynomialFit::TrajPolynomial res;
  res.Reset();

  // determinate polyfit order
  if (traj_points.empty()) {
    AERROR << "NO TrajPoints";
    return res;
  }

  const int8_t lon_polyfit_order = 5;
  int8_t lat_polyfit_order = 0;
  double traj_length = traj_points.back().path_point().s() -
                       traj_points.front().path_point().s();

  if (traj_points.size() >= 6 && traj_length > 3) {
    // Normal Traj Points
    lat_polyfit_order = 5;
  } else {
    // size < 6 or too short length, set order to 1
    lat_polyfit_order = 1;
  }

  std::vector<math::Vec2d> tmp_points;
  tmp_points.clear();
  tmp_points.reserve(traj_points.size());

  std::vector<double> a;
  std::vector<double> t;
  a.clear();
  t.clear();
  a.reserve(traj_points.size());
  t.reserve(traj_points.size());

  // 1. convert to vehicle frame and get a, t
  double tmp_x, tmp_y;
  for (auto& p : traj_points) {
    LocalCoordinatesToVehicleBody(vehicle_state, p.path_point().x(),
                                  p.path_point().y(), tmp_x, tmp_y);

    tmp_points.emplace_back(tmp_x, tmp_y);
    a.emplace_back(p.a());
    t.emplace_back(p.relative_time());
  }

  // 2. lat polynomial fit
  LatPolyFit(tmp_points, lat_polyfit_order, res);
  if (res.lat_coefficients.size() != (lat_polyfit_order + 1)) {
    res.Reset();

    AERROR << "ERROR: Polynomial Fit Failed.";
    return res;
  }

  // 3. lon polynomial fit
  LonPolyFit(a, t, lon_polyfit_order, res);

  if (res.lon_coefficients.size() != (lon_polyfit_order + 1)) {
    res.Reset();

    AERROR << "ERROR:Polynomial Fit Failed.";
    return res;
  }

  // from acc to s by integrate
  Eigen::VectorXd lon_poly_tmp;
  ConvAccFit2SFit(res.lon_coefficients, traj_points.front().relative_time(),
                  traj_points.front().v(), traj_points.front().path_point().s(),
                  lon_poly_tmp);

  // update lon_coefficients
  res.lon_coefficients = lon_poly_tmp;
  const double kDtPreview = 0.1;  // TODO(chaoqun) make it a parameter
  const ::common::TrajectoryPoint traj_pt_preview =
      traj_points.EvaluateByT(kDtPreview);

  if (res.lon_coefficients.size() > 2) {
    res.lon_coefficients(0) = 0;                           // s
    res.lon_coefficients(1) = traj_pt_preview.v();         // v
    res.lon_coefficients(2) = traj_pt_preview.a() / 2.0;   // a/2.
    res.lon_coefficients(3) = traj_pt_preview.da() / 6.0;  // j/6.
    res.lon_coefficients(4) = 0.0;
    res.lon_coefficients(5) = 0.0;
  } else {
    res.lon_coefficients.resize(lon_polyfit_order + 1);
    res.lon_coefficients(0) = 0;                           // s
    res.lon_coefficients(1) = traj_pt_preview.v();         // v
    res.lon_coefficients(2) = traj_pt_preview.a() / 2.0;   // a/2.
    res.lon_coefficients(3) = traj_pt_preview.da() / 6.0;  // j/6.
    res.lon_coefficients(4) = 0.0;
    res.lon_coefficients(5) = 0.0;
  }

  // 4. get total length
  res.length = traj_length;
  res.is_valid = true;

  return res;
}

void PolynomialFit::LocalCoordinatesToVehicleBody(
    const VehicleState& vehicle_state, const double local_x,
    const double local_y, double& body_frame_x, double& body_frame_y) {
  body_frame_x =
      (local_x - vehicle_state.x()) * std::cos(vehicle_state.heading()) +
      (local_y - vehicle_state.y()) * std::sin(vehicle_state.heading());
  body_frame_y =
      (local_y - vehicle_state.y()) * std::cos(vehicle_state.heading()) -
      (local_x - vehicle_state.x()) * std::sin(vehicle_state.heading());
}

void PolynomialFit::LatPolyFit(std::vector<::math::Vec2d> tmp_points,
                               const int order, TrajPolynomial& traj_poly_fit) {
  int coff_num = order + 1;
  uint16_t n = tmp_points.size();
  const int64_t lat_fit_start_time =
      std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::system_clock::now().time_since_epoch())
          .count();

  Eigen::MatrixXd A(n, coff_num);
  Eigen::MatrixXd B(n, 1);
  Eigen::MatrixXd Weight(n, n);
  Weight.setZero();

  // construct Matrix
  double a, b;
  uint16_t i = 0;
  for (const auto& point : tmp_points) {
    // the more far away distance, the smaller weight;
    Weight(i, i) = std::max(1, (n - i) * order);
    a = point.x();
    b = point.y();
    B(i, 0) = b;
    A(i, 0) = 1;
    for (int j = 1; j < order + 1; ++j) {
      A(i, j) = std::pow(a, j);
    }
    i++;
  }

  // fitted_line.polynomial_coefficients_ =
  //     (A.transpose() * Weight.transpose() * Weight * A).inverse() *
  //     A.transpose() * Weight.transpose() * Weight * B;

  Eigen::MatrixXd C = A.transpose() * Weight.transpose() * Weight * A;
  Eigen::VectorXd d = A.transpose() * Weight.transpose() * Weight * B;
  Eigen::PartialPivLU<Eigen::MatrixXd> lu(C);
  traj_poly_fit.lat_coefficients = lu.solve(d);

  const int64_t lat_fit_end_time =
      std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::system_clock::now().time_since_epoch())
          .count();
  AINFO << " calculate polynonimal coeff time diff ms: "
        << (lat_fit_end_time - lat_fit_start_time);
}

void PolynomialFit::LonPolyFit(std::vector<double> accel, std::vector<double> t,
                               const int order, TrajPolynomial& traj_poly_fit) {
  int coff_num = order + 1;

  uint16_t n = accel.size();
  const int64_t lon_fit_start_time =
      std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::system_clock::now().time_since_epoch())
          .count();

  Eigen::MatrixXd A(n, coff_num);
  Eigen::MatrixXd B(n, 1);

  // construct Matrix
  // s = sum(coeff * t) , so a -> t, b -> s
  double a, b;
  for (uint i = 0; i < accel.size(); i++) {
    a = t.at(i);
    b = accel.at(i);
    A(i, 0) = 1;
    B(i, 0) = b;
    for (int j = 1; j < order + 1; ++j) {
      A(i, j) = std::pow(a, j);
    }
  }

  Eigen::MatrixXd C = A.transpose() * A;
  Eigen::VectorXd d = A.transpose() * B;
  Eigen::PartialPivLU<Eigen::MatrixXd> lu(C);
  traj_poly_fit.lon_coefficients = lu.solve(d);

  const int64_t lon_fit_end_time =
      std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::system_clock::now().time_since_epoch())
          .count();
  AINFO << " calculate polynonimal coeff time diff ms: "
        << (lon_fit_end_time - lon_fit_start_time);
}

void PolynomialFit::ConvAccFit2SFit(const Eigen::VectorXd& acc_fit,
                                    const double& t, const double& v,
                                    const double& s, Eigen::VectorXd& s_fit) {
  double coe_v = v;
  double coe_s = s;
  std::deque<double> coeffi_v;
  coeffi_v.clear();
  std::deque<double> coeffi_s;
  coeffi_s.clear();

  std::deque<double> deque_acc_fit;
  deque_acc_fit.clear();
  // conver VectorXd to deque
  for (auto t : acc_fit) {
    deque_acc_fit.push_back(t);
  }

  // caculate coeffi_v accord to acc_fit
  coeffi_v = IntegratePolynomial(deque_acc_fit);
  for (int i = 0; i < static_cast<int>(coeffi_v.size()); i++) {
    coe_v -= coeffi_v.at(i) * std::pow(t, i);
  }
  coeffi_v.push_front(coe_v);

  // caculate coeffi_s accord to coeffi_v
  coeffi_s = IntegratePolynomial(coeffi_v);
  for (int i = 0; i < static_cast<int>(coeffi_s.size()); i++) {
    coe_s -= coeffi_s.at(i) * std::pow(t, i);
  }
  coeffi_s.push_front(coe_s);

  // rm redundant coeffie
  while (coeffi_s.size() > acc_fit.size()) {
    coeffi_s.pop_back();
  }

  s_fit.resize(coeffi_s.size());
  for (int i = 0; i < static_cast<int>(coeffi_s.size()); i++) {
    s_fit(i) = coeffi_s.at(i);
  }
}

// order: 0 -> max
std::deque<double> PolynomialFit::IntegratePolynomial(
    const std::deque<double>& polynomial) {
  std::deque<double> integrated;
  integrated.clear();

  for (size_t i = 0; i < polynomial.size(); ++i) {
    if (polynomial[i] != 0) {
      integrated.emplace_back(polynomial[i] / (i + 1));
    }
  }

  return integrated;
}

}  // namespace planning
}  // namespace zark
