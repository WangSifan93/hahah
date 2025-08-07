#ifndef ONBOARD_PLANNER_MATH_SPIRAL_H_
#define ONBOARD_PLANNER_MATH_SPIRAL_H_

#include <cmath>
#include <string>
#include <vector>

#include "absl/strings/str_cat.h"
#include "absl/types/span.h"

namespace e2e_noa::planning {
struct SpiralPoint {
  double x;
  double y;
  double theta;
  double k;
  double s{0};
  double dk{0};
  double ddk{0};
  std::string DebugString() const {
    return absl::StrCat("x: ", x, "y: ", y, " theta: ", theta, " k: ", k,
                        "s:", s, "dk: ", dk, "ddk: ", ddk);
  }

  double DistanceTo(const SpiralPoint& p) const {
    return std::sqrt((p.x - this->x) * (p.x - this->x) +
                     (p.y - this->y) * (p.y - this->y));
  }
};

class Spiral {
 public:
  Spiral(double x, double y, double theta, double sg,
         absl::Span<const double> params);

  SpiralPoint Eval(double s, int num_steps) const;

  std::vector<SpiralPoint> BatchEval(double s, int num_steps) const;
  int order() const { return static_cast<int>(params_.size()) - 1; }
  double length() const { return sg_; }

 private:
  SpiralPoint IntegrateOneStep(const SpiralPoint& sp0, double dx0, double dy0,
                               int step, double ds, double* pdx1,
                               double* pdy1) const;
  double x_;
  double y_;
  double theta_;
  double sg_;
  std::vector<double> params_;
  std::vector<double> theta_params_;
};
}  // namespace e2e_noa::planning
#endif
