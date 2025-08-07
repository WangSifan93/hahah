#ifndef ONBOARD_MATH_CIRCLE_FITTER_H_
#define ONBOARD_MATH_CIRCLE_FITTER_H_

#include <vector>

#include "absl/status/statusor.h"
#include "math/circle.h"
#include "math/vec.h"

namespace e2e_noa {

enum LS_SOLVER {
  SVD = 0,
  QR = 1,
  PINV = 2,
};

absl::StatusOr<Circle> FitCircleToData(
    const std::vector<Vec2d>& data,
    const std::vector<double>& weights = std::vector<double>(),
    LS_SOLVER solver = SVD, double* mse = nullptr);

}  // namespace e2e_noa

#endif
