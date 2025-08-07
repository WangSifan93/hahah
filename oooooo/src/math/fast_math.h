#ifndef MATH_FAST_MATH_H_
#define MATH_FAST_MATH_H_

#include <cmath>
#include <cstdint>
#include <ostream>

#include "glog/logging.h"
#include "math/cosine_approximation.h"
#include "math/util.h"

namespace e2e_noa::fast_math {

template <typename T>
inline T Atan2(T y, T x) {
  const T abs_x = std::abs(x);
  const T abs_y = std::abs(y);
  T min_val, max_val;
  const bool x_lt_y = abs_x < abs_y;
  if (x_lt_y) {
    min_val = abs_x;
    max_val = abs_y;
  } else {
    min_val = abs_y;
    max_val = abs_x;
  }
  if (max_val == T(0.0)) return 0.0;
  const T a = min_val / max_val;
  const T s = a * a;
  T r =
      ((T(-0.0464964749) * s + T(0.15931422)) * s - T(0.327622764)) * s * a + a;
  if (x_lt_y) r = T(M_PI_2) - r;
  if (x < 0) r = T(M_PI) - r;
  if (y < 0) r = -r;
  return r;
}

template <int N = 7>
inline double CosNormalized(double x) {
  constexpr double kTwoOverPi = 2.0 * M_1_PI;
  const int quad = FloorToInt(x * kTwoOverPi) + 2;
  switch (quad) {
    case 0:
      return -CosPi2<N>(x + M_PI);
    case 1:
      return CosPi2<N>(-x);
    case 2:
      return CosPi2<N>(x);
    case 3:
      return -CosPi2<N>(M_PI - x);
  }
  LOG(FATAL) << "Invalid quad " << quad << " with input " << x;
  return -1;
}
template <int N = 7>
inline double SinNormalized(double angle) {
  if (angle <= -M_PI_2) {
    return -CosPi2<N>(-angle - M_PI_2);
  } else {
    return CosNormalized<N>(NormalizeAngle(M_PI_2 - angle));
  }
}

inline double Sin(double angle) {
  return CosNormalized<7>(NormalizeAngle(M_PI_2 - angle));
}

inline double Cos(double angle) {
  return CosNormalized<7>(NormalizeAngle(angle));
}

template <int N = 7>
inline double SinN(double angle) {
  return CosNormalized<N>(NormalizeAngle(M_PI_2 - angle));
}

template <int N = 7>
inline double CosN(double angle) {
  return CosNormalized<N>(NormalizeAngle(angle));
}

inline constexpr float Log2(float x) {
  union {
    float f;
    uint32_t i;
  } vx = {x};
  union {
    uint32_t i;
    float f;
  } mx = {(vx.i & 0x007FFFFF) | 0x3f000000};
  float y = vx.i;
  y *= 1.1920928955078125e-7f;

  return y - 124.22551499f - 1.498030302f * mx.f -
         1.72587999f / (0.3520887068f + mx.f);
}

inline constexpr float Log(float p) { return 0.69314718f * Log2(p); }

inline float FastExpSchraudolph(float x) {
  constexpr float kA = (1 << 23) / 0.69314718f;
  constexpr float kB = (1 << 23) * (127 - 0.043677448f);
  float res = kA * x + kB;

  constexpr float kC = (1 << 23);
  constexpr float kD = (1 << 23) * 255;
  if (res < kC || res > kD) {
    res = (res < kC) ? 0.0f : kD;
  }

  uint32_t n = static_cast<uint32_t>(res);
  memcpy(&res, &n, 4);
  return res;
}

inline double FastExpSchraudolph(double x) {
  constexpr double kA = (1ll << 52) / 0.6931471805599453;
  constexpr double kB = (1ll << 52) * (1023 - 0.04367744890362246);
  double res = kA * x + kB;

  constexpr double kC = (1ll << 52);
  constexpr double kD = (1ll << 52) * 2047;
  if (res < kC || res > kD) {
    res = (res < kC) ? 0.0 : kD;
  }

  uint64_t n = static_cast<uint64_t>(res);
  memcpy(&res, &n, 8);
  return res;
}
}  // namespace e2e_noa::fast_math

#endif
