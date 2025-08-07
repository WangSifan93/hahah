#ifndef ONBOARD_MATH_LINE_FITTER_H_
#define ONBOARD_MATH_LINE_FITTER_H_

#include <limits>
#include <utility>
#include <vector>

#include "math/vec.h"

namespace e2e_noa {

enum FITTER {
  DEMING = 0,
};

class LineFitter {
 public:
  LineFitter()
      : param_(Vec3d(0.0, 0.0, 0.0)),
        mse_(std::numeric_limits<double>::infinity()),
        debug_(false) {}
  LineFitter(std::vector<Vec2d> data, std::vector<double> weights,
             bool debug = false)
      : data_(std::move(data)),
        weights_(std::move(weights)),
        param_(Vec3d(0.0, 0.0, 0.0)),
        mse_(std::numeric_limits<double>::infinity()),
        debug_(debug) {
    CheckData();
  }

  void CheckData() const;

  void FitData(FITTER fitter = FITTER::DEMING, bool normalize = true,
               bool compute_mse = false);
  double FitPointError(const Vec2d& point) const {
    return param_(0) * point(0) + param_(1) * point(1) + param_(2);
  }
  const Vec3d& param() const { return param_; }
  Vec2d tangent() const { return Vec2d(param_(1), -param_(0)); }
  double mse() const { return mse_; }

 private:
  std::vector<e2e_noa::Vec2d> data_;
  std::vector<double> weights_;
  Vec3d param_;
  double mse_;
  bool debug_;
};

}  // namespace e2e_noa

#endif
