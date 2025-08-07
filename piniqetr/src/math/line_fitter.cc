#include "math/line_fitter.h"

#include <algorithm>
#include <numeric>
#include <ostream>
#include <string>

#include "Eigen/Jacobi"
#include "Eigen/SVD"
#include "glog/logging.h"
#include "math/stats.h"
#include "math/util.h"

namespace e2e_noa {

void LineFitter::CheckData() const {
  const int nd = data_.size();
  const int nw = weights_.size();
  CHECK(nw == 0 || nw == nd);
  for (const double& w : weights_) {
    CHECK_GE(w, 0.0);
  }
}

void LineFitter::FitData(FITTER fitter, bool normalize, bool compute_mse) {
  Vec2d sample_mean;
  const auto sample_cov =
      ComputeWeighted2dSampleStatistics(data_, &sample_mean, weights_);

  if (debug_) {
    VLOG(2) << "******** line fitter stats ********";
    VLOG(2) << " line fitter data mean: " << sample_mean.transpose();
    VLOG(2) << " line fitter data cov: " << sample_cov;
    VLOG(2) << "******** line fitter stats ********";
  }

  if (fitter == FITTER::DEMING) {
    const Eigen::JacobiSVD<Eigen::Matrix2d> svd(
        sample_cov, Eigen::ComputeFullU | Eigen::ComputeFullV);

    const auto line_normal = svd.matrixU().block<2, 1>(0, 1);
    const double bias = sample_mean.dot(line_normal);
    param_ = Vec3d(line_normal(0), line_normal(1), bias);
  } else {
    CHECK(false);
  }

  if (normalize) {
    param_ /= param_.head(2).norm();
    if (debug_) {
      VLOG(3) << "  normalized param = " << param_.transpose();
    }
  }

  if (compute_mse) {
    mse_ = std::accumulate(
        data_.begin(), data_.end(), 0.0,
        [this](const double& sum, const Vec2d& sample) {
          return sum +
                 Sqr(param_(0) * sample(0) + param_(1) * sample(1) + param_(2));
        });
  }
}

}  // namespace e2e_noa
