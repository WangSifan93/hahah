//
// Copyright 2024 ZDrive.AI. All Rights Reserved.
//

#pragma once

#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <algorithm>

#include "apps/planning/src/common/vehicle_state/proto/vehicle_state.h"
#include "apps/planning/src/common/local_route/local_route.h"
#include "apps/planning/src/config/conf_gflags.h"
#include "messages/map_service/all_map_new.pb.h"

namespace zark {
namespace planning {

constexpr double kCentimeterToMetre = 0.01;
constexpr double kMinValidLength = 10.0;    //[m]
constexpr double kBoundaryBackDist = 15.0;  //[m]
constexpr double kTimeGapDisbuffer = 5.0;   //[m]
constexpr double kFitStepDist = 5.0;
constexpr int8_t kPolynomialDegree = 5;

/**
 * @class MapPolynomialFit
 * @brief The class of MapPolynomialFit.
 *        It generate reference lines based on local hd map.
 */
class MapPolynomialFit {
 public:
  MapPolynomialFit() : vehicle_state_(){};
  ~MapPolynomialFit(){};

  struct FittedLine {
    FittedLine() : polynomial_coefficients_(), valid_length_(0.0) {}

    Eigen::VectorXd polynomial_coefficients_;
    double valid_length_;
  };

  void SetVehicleState(const common::VehicleState& state) {
    vehicle_state_ = state;
    ego_point_ = ::math::Vec2d(state.x(), state.y());
  }
  const common::VehicleState GetVehicleState() const { return vehicle_state_; }
  const ::math::Vec2d GetEgoPoint() const { return ego_point_; }

  /**
   * @brief Fits a polynomial curve to a set of discrete points using the
   * weighted least squares method.
   *
   * @param points Vector containing the discrete points to be fitted.
   * @param line  the fitted polynomial curve info.
   *
   * @note The polynomial curve to be fitted is of the form:
   *       y = a0 + a1*x + a2*x^2 + a3*x^3 + a4*x^4 + a5*x^5;
   *       The function minimizes the error using the equation AX = B, where A
   * Matrix representing the design matrix derived from the input points; B
   * Matrix representing the target values derived from the input points; X
   * Matrix represents the coefficients of the polynomial.
   *
   * @b Algorithm:
   *   X = (A.t() * W.t() * W * A).inverse() * A.t() * W.t() * W * B;
   * The weight matrix W assigns higher weights to points closer to the curve
   * and lower weights to points farther away.
   */
  void DiscretePointsFit(const std::vector<LocalRoutePoint>& points, int order,
                         FittedLine& line,
                         const bool& need_convert_frame = true);

  void VirtualLineProcess(const int& order, const FittedLine& center_line,
                          const FittedLine& other_side_line,
                          FittedLine& virtual_line);

  inline void LocalCoordinatesToVehicleBody(
      const common::VehicleState& vehicle_state, const double local_x,
      const double local_y, double& body_frame_x, double& body_frame_y) {
    body_frame_x =
        (local_x - vehicle_state.x()) * std::cos(vehicle_state.heading()) +
        (local_y - vehicle_state.y()) * std::sin(vehicle_state.heading());
    body_frame_y =
        (local_y - vehicle_state.y()) * std::cos(vehicle_state.heading()) -
        (local_x - vehicle_state.x()) * std::sin(vehicle_state.heading());
  }

  inline void BodyCoordinatesToLocal(const common::VehicleState& vehicle_state,
                                     const double body_frame_x,
                                     const double body_frame_y, double& local_x,
                                     double& local_y) {
    local_x = vehicle_state.x() + body_frame_x * cos(vehicle_state.heading()) -
              body_frame_y * sin(vehicle_state.heading());
    local_y = vehicle_state.y() + body_frame_x * sin(vehicle_state.heading()) +
              body_frame_y * cos(vehicle_state.heading());
  }

  void SolvePolynomialCoefficients(const double t1, const double vel,
                                   const LocalRoutePoint& start_point,
                                   const LocalRoutePoint& tartget_point,
                                   std::vector<LocalRoutePoint>& fix_noa_line);

 private:
  /**
   * @brief Sets the maximum allowable error value to determine the effective
   * fitting length.
   *
   * @param points Vector of Vec2d points representing the original data points.
   * @param coeff Eigen::VectorXd object containing the coefficients for
   * fitting.
   * @return The determined length after checking for maximum allowable error.
   */
  inline bool LengthCheck(const std::list<Vec2d>& points, const int order,
                          FittedLine& line) {
    const auto coeff_size = line.polynomial_coefficients_.size();
    if (coeff_size != order + 1) {
      AERROR << "LengthCheck line.polynomial_coefficients_.size not equal to "
                "order, "
             << coeff_size << " != " << order;
      return false;
    }
    const float kMaxErrorValue = tolerance_increase_ ? 0.30 : 0.20;
    const float kValidError = 0.08;
    const float kValidRatio = 0.25;
    float length = 0.0;
    if (static_cast<int>(points.size()) < order + 1) {
      return false;
    }
    double error_value = 0.0;
    uint16_t i = 0;
    uint16_t j = 0;
    for (const auto& point : points) {
      double poly_y =
          CallPolyfitValue(line.polynomial_coefficients_, order, point.x());
      error_value = sqrt((poly_y - point.y()) * (poly_y - point.y()));
      if (error_value > kMaxErrorValue) {
        AERROR << " index: " << i << "  fit error: " << error_value
               << " point size: " << points.size()
               << "  curve length: " << length << "  x: " << point.x()
               << "  original y: " << point.y() << "  fitted y: " << poly_y;
        ADEBUG << " coeff : " << line.polynomial_coefficients_;
        break;
      }
      if (error_value > kValidError) {
        j++;
      }
      length = point.x();
      if (i == points.size() - 1) {
        AERROR << " index: " << i << "  fit error: " << error_value
               << " point size: " << points.size()
               << "  curve length: " << length << "  x: " << point.x()
               << "  original y: " << point.y() << "  fitted y: " << poly_y;
        ADEBUG << " coeff : " << line.polynomial_coefficients_;
      }
      i++;
    }
    line.valid_length_ = length;
    float error_ratio = i > 0 ? j / i : 1.0;
    if (error_ratio < kValidRatio && error_value < kMaxErrorValue) {
      return true;
    } else {
      AERROR << "fit error more than 0.1 ration: " << error_ratio
             << " error point num: " << j;
      return false;
    }
  }

  bool LinearInterpolation(std::vector<::math::Vec2d>& points);

  inline const double CallPolyfitValue(const Eigen::VectorXd& coefficients,
                                       const int order, const double x_value) {
    double fit_value = 0.0;
    for (int i = 0; i <= order; i++) {
      fit_value += coefficients(i) * std::pow(x_value, i);
    }
    return fit_value;
  }

  void FittingLine(std::list<::math::Vec2d>& tmp_points, const int order,
                   FittedLine& fitted_line);

 private:
  common::VehicleState vehicle_state_;
  ::math::Vec2d ego_point_;
  bool tolerance_increase_ = false;

 private:
  FRIEND_TEST(PolynomialFitTest, TestLocalCoordinatesToVehicleBody);
  FRIEND_TEST(PolynomialFitTest, TestFittingLine);
};

}  // namespace planning
}  // namespace zark
