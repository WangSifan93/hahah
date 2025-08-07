#include <queue>

#include "apps/planning/src/common/math/discrete_points_math.h"
#include "apps/planning/src/reference_line_provider/map_polynomial_fit/map_polynomial_fit.h"

namespace zark {
namespace planning {

void MapPolynomialFit::DiscretePointsFit(
    const std::vector<LocalRoutePoint>& points, int order, FittedLine& line,
    const bool& need_convert_frame) {
  tolerance_increase_ = !need_convert_frame;
  const int64_t fit_start_time =
      std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::system_clock::now().time_since_epoch())
          .count();
  AINFO << "DiscretePointsFit original points size: " << points.size();
  std::list<Vec2d> tmp_points;
  tmp_points.clear();
  const double kValidEgoSpeed = 10.0;
  const double kFitForwradTimeSec = 6.0;
  double body_frame_x = 0.0;
  double body_frame_y = 0.0;
  double last_x = -1000.0;
  double valid_speed = fmax(kValidEgoSpeed, vehicle_state_.linear_velocity());
  double forward_distance = valid_speed * kFitForwradTimeSec;

  // filter ReferencePoints
  for (const auto& tmp : points) {
    if (need_convert_frame) {
      LocalCoordinatesToVehicleBody(vehicle_state_, tmp.x(), tmp.y(),
                                    body_frame_x, body_frame_y);
    } else {
      body_frame_x = tmp.x();
      body_frame_y = tmp.y();
    }
    if (body_frame_x + kBoundaryBackDist < 0) {
      continue;
    }

    if (body_frame_x < last_x) {
      continue;
    }

    if (body_frame_x > forward_distance) {
      AERROR << "meet max forward distance body_frame_x: " << body_frame_x
             << " valid_speed: " << valid_speed
             << " forward_distance: " << forward_distance;
      break;
    }

    tmp_points.emplace_back(std::move(Vec2d(body_frame_x, body_frame_y)));
    last_x = body_frame_x;
  }

  if (static_cast<int>(tmp_points.size()) >= order + 1) {
    FittingLine(tmp_points, order, line);
  } else {
    AERROR << "filter points size errror: " << tmp_points.size();
  }
  const int64_t end_time =
      std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::system_clock::now().time_since_epoch())
          .count();
  AINFO << " single line fitted time diff ms: " << (end_time - fit_start_time);
}

void MapPolynomialFit::VirtualLineProcess(const int& order,
                                          const FittedLine& center_line,
                                          const FittedLine& other_side_line,
                                          FittedLine& virtual_line) {
  if (order <= 0) {
    AERROR << "order error: " << order;
    return;
  }
  const auto center_coeff_num = center_line.polynomial_coefficients_.size();
  const auto sider_coeff_num = other_side_line.polynomial_coefficients_.size();
  if (center_coeff_num != order + 1) {
    AERROR << "center_line.polynomial_coefficients not equal to order, "
           << center_coeff_num << " != " << order;
    return;
  }
  if (sider_coeff_num != order + 1) {
    AERROR << "other_side_line.polynomial_coefficients not equal to order, "
           << sider_coeff_num << " != " << order;
    return;
  }
  double min_length =
      std::fmin(center_line.valid_length_, other_side_line.valid_length_);
  const double length_limit = 0.001;
  if (min_length < length_limit) {
    return;
  }
  const uint8_t kVirtualPointSize = 20;
  std::list<::math::Vec2d> tmp_points;
  double distance_diff = min_length / kVirtualPointSize;
  double accumulate_dis = 0.0;
  double lateral_value = 0.0;
  while (accumulate_dis <= min_length) {
    lateral_value = 2.0 * CallPolyfitValue(center_line.polynomial_coefficients_,
                                           order, accumulate_dis) -
                    CallPolyfitValue(other_side_line.polynomial_coefficients_,
                                     order, accumulate_dis);
    tmp_points.emplace_back(std::move(Vec2d(accumulate_dis, lateral_value)));
    accumulate_dis += distance_diff;
  }

  FittingLine(tmp_points, order, virtual_line);
}

bool MapPolynomialFit::LinearInterpolation(std::vector<Vec2d>& points) {
  const float kPointsMinDistance = 0.5;      //[m]
  const double kPointsValidDistance = 50.0;  //[m]
  size_t i = 1;
  while (i < points.size()) {
    const double dis = fabs(points.at(i).x() - points.at(i - 1).x());
    if (dis >= kPointsValidDistance) {
      AERROR << " points distance error: " << dis;
      return false;
    }
    if (dis > kPointsMinDistance) {
      double interpolate_length = 0.0;
      double k = (points.at(i).y() - points.at(i - 1).y()) / dis;
      int j = 1;
      std::vector<Vec2d> interpolate_points;
      while (j * kPointsMinDistance < dis) {
        interpolate_length = j * kPointsMinDistance;
        interpolate_points.emplace_back(
            std::move(Vec2d(points.at(i - 1).x() + interpolate_length,
                            points.at(i - 1).y() + k * interpolate_length)));
        j++;
      }
      std::vector<Vec2d>::iterator iter = points.begin() + i;
      points.insert(iter, interpolate_points.begin(), interpolate_points.end());
      i += interpolate_points.size();
    } else {
      i++;
    }
  }
  return true;
}

void MapPolynomialFit::FittingLine(std::list<::math::Vec2d>& tmp_points,
                                   const int order, FittedLine& fitted_line) {
  const uint8_t kMaxRecycleCnt = 10;
  uint8_t k = 0;
  int coff_num = order + 1;
  uint16_t n = tmp_points.size();
  const int64_t fit_start_time =
      std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::system_clock::now().time_since_epoch())
          .count();
  while (k < kMaxRecycleCnt && n >= coff_num) {
    Eigen::MatrixXd A(n, coff_num);
    Eigen::MatrixXd B(n, 1);
    Eigen::MatrixXd Weight(n, n);
    Weight.setZero();

    // construct Matrix
    double a, b;
    uint16_t i = 0;
    for (const auto& point : tmp_points) {
      // the more far away distance, the smaller weight;
      Weight(i, i) = 1.0;  // std::max(1, (n - i) * order);
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
    fitted_line.polynomial_coefficients_ = lu.solve(d);

    ADEBUG << " FIT COEFF: " << fitted_line.polynomial_coefficients_;

    const int64_t fit_end_time =
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch())
            .count();
    AINFO << " calculate polynonimal coeff time diff ms: "
          << (fit_end_time - fit_start_time);
    if (LengthCheck(tmp_points, order, fitted_line)) {
      break;
    } else {
      if (fitted_line.valid_length_ <= kMinValidLength) {
        AERROR << "fit error ,line valid length: " << fitted_line.valid_length_;
        // break;
      }
    }
    k++;
    double distance = 0.0;
    uint8_t cycle_cnt = 0;
    while (distance <= kFitStepDist && n >= coff_num &&
           cycle_cnt < kMaxRecycleCnt) {
      auto iterator = tmp_points.rbegin();
      auto last_it = std::next(iterator);
      if (last_it == tmp_points.rend()) {
        break;
      }
      distance += iterator->DistanceTo(*last_it);
      tmp_points.pop_back();
      n = tmp_points.size();
      cycle_cnt++;
    }
  }
  AERROR << "RANGE Fit NUM: " << int(k);
}

void MapPolynomialFit::SolvePolynomialCoefficients(
    const double t1, const double vel, const LocalRoutePoint& start_point,
    const LocalRoutePoint& tartget_point,
    std::vector<LocalRoutePoint>& fix_noa_line) {
  Eigen::MatrixXd T(6, 6);
  Eigen::VectorXd X(6), Y(6);

  double x0, v0x, a0x, y0, v0y, a0y, x1, v1x, a1x, y1, v1y, a1y;

  Eigen::VectorXd coefficients_x;
  Eigen::VectorXd coefficients_y;

  x0 = start_point.x();
  v0x = vel * std::cos(start_point.heading());
  a0x = 0.0;

  y0 = start_point.y();
  v0y = vel * std::sin(start_point.heading());
  a0y = 0.0;

  x1 = tartget_point.x();
  y1 = tartget_point.y();
  v1x = vel * std::cos(tartget_point.heading());
  v1y = vel * std::sin(tartget_point.heading());
  a1x = a0x;
  a1y = a0y;

  double t0 = 0.0;

  // 构建T矩阵
  T << 1, t0, pow(t0, 2), pow(t0, 3), pow(t0, 4), pow(t0, 5), 0, 1, 2 * t0,
      3 * pow(t0, 2), 4 * pow(t0, 3), 5 * pow(t0, 4), 0, 0, 2, 6 * t0,
      12 * pow(t0, 2), 20 * pow(t0, 3), 1, t1, pow(t1, 2), pow(t1, 3),
      pow(t1, 4), pow(t1, 5), 0, 1, 2 * t1, 3 * pow(t1, 2), 4 * pow(t1, 3),
      5 * pow(t1, 4), 0, 0, 2, 6 * t1, 12 * pow(t1, 2), 20 * pow(t1, 3);

  // 构建X和Y向量
  X << x0, v0x, a0x, x1, v1x, a1x;
  Y << y0, v0y, a0y, y1, v1y, a1y;

  // 计算x和y方向的多项式系数

  coefficients_x = T.inverse() * X;
  coefficients_y = T.inverse() * Y;
  coefficients_x.reverse();
  coefficients_y.reverse();

  ADEBUG << "  coefficients_x : " << coefficients_x;
  ADEBUG << "  coefficients_Y : " << coefficients_y;

  fix_noa_line.clear();
  const double kPointSampleDist = 1.5;
  double length = start_point.DistanceTo(tartget_point);
  double dist_interval = length / kPointSampleDist;
  double t_inter_val = t1 / dist_interval;
  for (double i = 0.0; i <= t1; i += t_inter_val) {
    double x = CallPolyfitValue(coefficients_x, 5, i);
    double y = CallPolyfitValue(coefficients_y, 5, i);
    fix_noa_line.emplace_back(std::move(LocalRoutePoint(
        std::move(hdmap::MapPathPoint(std::move(::math::Vec2d(x, y)), 0.0)),
        0.0, 0.0)));
  }

  // heading calculate;
  if (fix_noa_line.size() > 1) {
    for (auto point_iter = fix_noa_line.begin();
         point_iter != fix_noa_line.end(); point_iter++) {
      double heading = 0.0;
      if (point_iter == fix_noa_line.begin()) {
        auto next_point = std::next(point_iter);
        heading = std::atan2(next_point->y() - point_iter->y(),
                             next_point->x() - point_iter->x());
      } else {
        auto prve_point = std::prev(point_iter);
        heading = std::atan2(point_iter->y() - prve_point->y(),
                             point_iter->x() - prve_point->x());
      }
      point_iter->set_heading(heading);
    }
  }
}

}  // namespace planning
}  // namespace zark
