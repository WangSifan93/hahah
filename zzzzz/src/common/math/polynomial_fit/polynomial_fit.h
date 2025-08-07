/******************************************************************************
 * Copyright 2024 The zark Authors. All Rights Reserved.
 *****************************************************************************/

/**
 * @file traj_poly_fit.h
 **/

#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>
#include "vec2d.h"
#include "pnc_point.h"
#include "apps/planning/src/common/trajectory/discretized_trajectory.h"
#include "apps/planning/src/common/vehicle_state/proto/vehicle_state.h"
#include "gtest/gtest.h"

namespace zark {
namespace planning {

using VehicleState = zark::planning::common::VehicleState;

class PolynomialFit {
 public:
  PolynomialFit();
  ~PolynomialFit();

  /**
   * @brief polynomial fit output
   *
   */
  struct TrajPolynomial {
    Eigen::VectorXd lat_coefficients;
    Eigen::VectorXd lon_coefficients;
    double length;
    bool is_valid;

    void Reset() {
      this->lat_coefficients.resize(6);
      this->lon_coefficients.resize(6);

      this->lat_coefficients << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
      this->lon_coefficients << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
      this->length = 0.0;
      this->is_valid = false;
    }
  };

  /**
   * @brief Polynomial fit traj_points to vehicle frame
   *
   * @param traj_points
   * @param vehicle_state
   * @param fit_order
   * @return TrajPolyFitOut
   */
  TrajPolynomial TrajPointsPolyFit(const DiscretizedTrajectory& traj_interp,
                                   const VehicleState& vehicle_state);

 private:
  /**
   * @brief transform local coordinate points to vehicle coordinate
   *
   * @param vehicle_state
   * @param local_x
   * @param local_y
   * @param body_frame_x
   * @param body_frame_y
   */
  void LocalCoordinatesToVehicleBody(const VehicleState& vehicle_state,
                                     const double local_x, const double local_y,
                                     double& body_frame_x,
                                     double& body_frame_y);

  /**
   * @brief Lat Polynomial fit traj_points to vehicle frame
   *
   * @param tmp_points
   * @param order
   * @param traj_poly_fit
   */
  void LatPolyFit(std::vector<::math::Vec2d> tmp_points, const int order,
                  TrajPolynomial& traj_poly_fit);

  /**
   * @brief Lon Polynomial fit traj_points to vehicle frame
   *
   * @param tmp_points
   * @param order
   * @param traj_poly_fit
   */
  void LonPolyFit(std::vector<double> acc, std::vector<double> t,
                  const int order, TrajPolynomial& traj_poly_fit);

  /**
   * @brief convert acc_fit 2 s_fit
   *
   * @param acc_fit
   * @param s_fit
   */
  void ConvAccFit2SFit(const Eigen::VectorXd& acc_fit, const double& t,
                       const double& v, const double& s,
                       Eigen::VectorXd& s_fit);

  /**
   * @brief IntegratePolynomial
   *
   * @param polynomial
   * @return std::deque<double> order: 0 -> max
   */
  std::deque<double> IntegratePolynomial(const std::deque<double>& polynomial);

 private:
  FRIEND_TEST(TrajPolynomialTest, TestLocalCoordinatesToVehicleBody);
  FRIEND_TEST(TrajPolynomialTest, TestLatPolyFit);
  FRIEND_TEST(TrajPolynomialTest, TestLonPolyFit);
};

}  // namespace planning
}  // namespace zark
