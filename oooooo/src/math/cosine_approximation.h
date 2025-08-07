#ifndef ONBOARD_MATH_COSINE_APPROXIMATION_H_
#define ONBOARD_MATH_COSINE_APPROXIMATION_H_

namespace e2e_noa::fast_math {

inline double Cos_52(double x) {
  const double c1 = 0.9999932946;
  const double c2 = -0.4999124376;
  const double c3 = 0.0414877472;
  const double c4 = -0.0012712095;
  const double x2 = x * x;
  return (c1 + x2 * (c2 + x2 * (c3 + c4 * x2)));
}

inline double Cos_73(double x) {
  constexpr double c1 = 0.999999953464;
  constexpr double c2 = -0.499999053455;
  constexpr double c3 = 0.0416635846769;
  constexpr double c4 = -0.0013853704264;
  constexpr double c5 = 0.00002315393167;

  const double x2 = x * x;
  return (c1 + x2 * (c2 + x2 * (c3 + x2 * (c4 + c5 * x2))));
}

inline double Cos_121(double x) {
  constexpr double c1 = 0.99999999999925182;
  constexpr double c2 = -0.49999999997024012;
  constexpr double c3 = 0.041666666473384543;
  constexpr double c4 = -0.001388888418000423;
  constexpr double c5 = 0.0000248010406484558;
  constexpr double c6 = -0.0000002752469638432;
  constexpr double c7 = 0.0000000019907856854;

  const double x2 = x * x;
  return (c1 +
          x2 * (c2 + x2 * (c3 + x2 * (c4 + x2 * (c5 + x2 * (c6 + c7 * x2))))));
}

template <int N = 7>
inline double CosPi2(double angle) {
  switch (N) {
    case 5:
      return Cos_52(angle);
    case 7:
      return Cos_73(angle);
    case 12:
      return Cos_121(angle);
  }
}

}  // namespace e2e_noa::fast_math

#endif
