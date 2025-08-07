#ifndef AD_E2E_PLANNING_MATH_DOUBLE_H
#define AD_E2E_PLANNING_MATH_DOUBLE_H

namespace ad_e2e {
namespace planning {
namespace math {

class Double {
 public:
  enum class CompareType { EQUAL = 0, LESS = -1, GREATER = 1 };
  Double() = default;
  ~Double() = default;
  static CompareType Compare(const double& a, const double& b);
  static CompareType Compare(const double& a, const double& b,
                             const double& epsilon);

 private:
  static bool LessThan(const double& a, const double& b, const double& epsilon);
  static bool GreaterThan(const double& a, const double& b,
                          const double& epsilon);

  static double epsilon_;
};

}  // namespace math
}  // namespace planning
}  // namespace ad_e2e
#endif
