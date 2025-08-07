#ifndef AD_E2E_PLANNING_MATH_MATH_UTIL_H
#define AD_E2E_PLANNING_MATH_MATH_UTIL_H

#include <cstdint>
#include <limits>
#include <list>
#include <utility>
#include <vector>

#include "common/type_def.h"
#include "math/vec2d.h"

namespace ad_e2e {
namespace planning {
namespace math {
double Sqr(const double x);

double CrossProd(const Vec2d &start_point, const Vec2d &end_point_1,
                 const Vec2d &end_point_2);

double InnerProd(const Vec2d &start_point, const Vec2d &end_point_1,
                 const Vec2d &end_point_2);

double CrossProd(const double x0, const double y0, const double x1,
                 const double y1);

double InnerProd(const double x0, const double y0, const double x1,
                 const double y1);

double WrapAngle(const double angle);

double NormalizeAngle(const double angle);

double AngleDiff(const double a1, const double a2);

int RandomInt(const int s, const int t, unsigned int rand_seed = 1);

double RandomDouble(const double s, const double t, unsigned int rand_seed = 1);

TurnType GetTurnTypeByHeading(const double &entry_heading,
                              const double &exit_heading);

template <typename T>
inline T Square(const T value) {
  return value * value;
}

template <typename T>
T Clamp(const T value, T bound1, T bound2) {
  if (bound1 > bound2) {
    std::swap(bound1, bound2);
  }

  if (value < bound1) {
    return bound1;
  } else if (value > bound2) {
    return bound2;
  }
  return value;
}

double Gaussian(const double u, const double std, const double x);

inline double Sigmoid(const double x) { return 1.0 / (1.0 + std::exp(-x)); }

std::pair<double, double> RotateVector2d(const double x, const double y,
                                         const double theta);

Vec2d RotateVector2d(const Vec2d &vec, const double &theta);

inline std::pair<double, double> RFUToFLU(const double x, const double y) {
  return std::make_pair(y, -x);
}

inline std::pair<double, double> FLUToRFU(const double x, const double y) {
  return std::make_pair(-y, x);
}

inline void L2Norm(int feat_dim, float *feat_data) {
  if (feat_dim == 0) {
    return;
  }

  float l2norm = 0.0f;
  for (int i = 0; i < feat_dim; ++i) {
    l2norm += feat_data[i] * feat_data[i];
  }
  if (l2norm == 0) {
    float val = 1.f / std::sqrt(static_cast<float>(feat_dim));
    for (int i = 0; i < feat_dim; ++i) {
      feat_data[i] = val;
    }
  } else {
    l2norm = std::sqrt(l2norm);
    for (int i = 0; i < feat_dim; ++i) {
      feat_data[i] /= l2norm;
    }
  }
}

std::pair<double, double> Cartesian2Polar(double x, double y);

template <typename T>
T interp1_inc(const std::vector<T> &X, const std::vector<T> &Y, T xp) {
  T temp1 = 0;
  uint16_t i;

  if (X.size() != Y.size()) {
    return 0;
  }

  if (xp <= X.front()) {
    temp1 = Y.front();
  } else if (xp >= X.back()) {
    temp1 = Y.back();
  } else {
    for (i = 0; i < X.size() - 1; ++i) {
      if (xp >= X.at(i) && xp < X.at(i + 1)) {
        break;
      }
    }
    if (i == X.size() - 1) {
      i = i - 1;
    }
    temp1 = Y.at(i) +
            (Y.at(i + 1) - Y.at(i)) / (X.at(i + 1) - X.at(i)) * (xp - X.at(i));
  }
  return temp1;
}

template <typename T>
T interp1_dec(std::vector<T> &X, std::vector<T> &Y, T xp) {
  T temp1 = 0;
  uint16_t i;

  if (X.size() != Y.size()) {
    return 0;
  }

  if (xp >= X.front()) {
    temp1 = Y.front();
  } else if (xp <= X.back()) {
    temp1 = Y.back();
  } else {
    for (i = 0; i < X.size() - 1; ++i) {
      if (xp < X.at(i) && xp >= X.at(i + 1)) {
        break;
      }
    }
    if (i == X.size() - 1) {
      i = i - 1;
    }
    temp1 = Y.at(i) +
            (Y.at(i + 1) - Y.at(i)) / (X.at(i + 1) - X.at(i)) * (xp - X.at(i));
  }
  return temp1;
}

template <typename T>
inline T RateLmt(T CurrentValue, const T &LastValue, T &MinLmt, T &MaxLmt) {
  if (MinLmt > MaxLmt) {
    std::swap(MinLmt, MaxLmt);
  }

  if (CurrentValue - LastValue > MaxLmt) {
    CurrentValue = LastValue + MaxLmt;
  } else if (CurrentValue - LastValue < MinLmt) {
    CurrentValue = LastValue + MinLmt;
  }
  return CurrentValue;
}

template <typename T>
inline T Cubic(const T value) {
  return value * value * value;
}

template <typename T>
T interp2_inc(std::vector<T> &X, std::vector<T> &Y,
              std::vector<std::vector<T>> &Z, T xp, T yp) {
  std::vector<T> temp_z_line;
  std::vector<T> temp_y_line = Z.front();
  uint16_t size_x = X.size();
  uint16_t size_y = Y.size();
  if (size_y != Z.size()) {
    return 0;
  }

  for (uint16_t iy = 0; iy < size_y; iy++) {
    temp_y_line = Z[iy];
    if (size_x != temp_y_line.size()) {
      return 0;
    }
    temp_z_line.push_back(interp1_inc(X, temp_y_line, xp));
  }
  return interp1_inc(Y, temp_z_line, yp);
}

template <typename T>
inline T SafeDivide(const T &divisor, T dividend) {
  const T eps = std::numeric_limits<T>::epsilon();
  if (std::abs(dividend) < eps) {
    dividend = eps;
  }
  return dividend == static_cast<T>(0) ? std::numeric_limits<T>::max()
                                       : divisor / dividend;
}

template <typename T>
inline T QuaternionToYaw(T w, T x, T y, T z) {
  return atan2(static_cast<T>(2.0) * (x * y + w * z),
               static_cast<T>(1.0) - static_cast<T>(2.0) * (y * y + z * z));
}
}  // namespace math
}  // namespace planning
}  // namespace ad_e2e
#endif
