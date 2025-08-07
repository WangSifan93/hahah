#ifndef ONBOARD_MATH_PIECEWISE_LINEAR_FUNCTION_H_
#define ONBOARD_MATH_PIECEWISE_LINEAR_FUNCTION_H_

#include <algorithm>
#include <utility>
#include <vector>

#include "glog/logging.h"
#include "math/eigen.h"
#include "math/geometry/util.h"
#include "math/util.h"
#include "math/vec.h"
#include "piecewise_linear_function.pb.h"

namespace e2e_noa {

template <typename T, typename TS>
class Lerper {
 public:
  T operator()(T a, T b, TS alpha) const { return Lerp(a, b, alpha); }
};

template <typename T, typename TS>
class AngleLerper {
 public:
  T operator()(T a, T b, TS alpha) const { return LerpAngle(a, b, alpha); }
};

template <typename T, typename TS>
class NormalizedAngleLerper {
 public:
  T operator()(T a, T b, TS alpha) const {
    return NormalizeAngle(LerpAngle(a, b, alpha));
  }
};

template <typename TY, typename TX = double, typename LERPER = Lerper<TY, TX>>
class PiecewiseLinearFunction final {
 public:
  PiecewiseLinearFunction() = default;
  PiecewiseLinearFunction(std::vector<TX> x, std::vector<TY> y)
      : x_(std::move(x)), y_(std::move(y)), lerper_(LERPER()) {
    CHECK_EQ(x_.size(), y_.size());
    CHECK_GT(x_.size(), 1);
    x_slope_.reserve(x_.size() - 1);
    for (size_t i = 1; i < x_.size(); ++i) {
      x_slope_.push_back(TX(1) / (x_[i] - x_[i - 1]));
    }
  }

  TY operator()(TX x) const { return Evaluate(x); }
  TY Evaluate(TX x) const {
    const int index = std::upper_bound(x_.begin(), x_.end(), x) - x_.begin();
    if (index == 0) return y_.front();
    if (index == static_cast<int>(x_.size())) return y_.back();
    DCHECK_GT(x_[index], x_[index - 1]);
    const TX alpha = (x - x_[index - 1]) * x_slope_[index - 1];
    return lerper_(y_[index - 1], y_[index], alpha);
  }

  TY EvaluateWithExtrapolation(TX x) const {
    int index = std::upper_bound(x_.begin(), x_.end(), x) - x_.begin();
    if (index == 0) {
      if (x_.size() <= 1) return y_.front();
      index = 1;
    }
    if (index == static_cast<int>(x_.size())) {
      if (index <= 1) return y_.front();
      index = x_.size() - 1;
    }
    DCHECK_GT(x_[index], x_[index - 1]);
    const TX alpha = (x - x_[index - 1]) * x_slope_[index - 1];
    return lerper_(y_[index - 1], y_[index], alpha);
  }

  std::vector<TY> Evaluate(const std::vector<TX>& x) const {
    DCHECK(!x.empty());
    DCHECK(std::is_sorted(x.begin(), x.end()));
    std::vector<TY> y;
    y.reserve(x.size());
    int index = std::upper_bound(x_.begin(), x_.end(), x.front()) - x_.begin();
    for (int i = 0; i < x.size(); ++i) {
      while (index < x_.size() && x_[index] <= x[i]) ++index;
      if (index == static_cast<int>(x_.size())) {
        y.insert(y.end(), x.size() - y.size(), y_.back());
        return y;
      }
      if (index == 0) {
        y.push_back(y_.front());
        continue;
      }
      DCHECK_GT(x_[index], x_[index - 1]);
      const TX alpha = (x[i] - x_[index - 1]) * x_slope_[index - 1];
      y.push_back(lerper_(y_[index - 1], y_[index], alpha));
    }
    return y;
  }

  TY EvaluateSlope(TX x) const {
    int index = std::upper_bound(x_.begin(), x_.end(), x) - x_.begin();
    if (index == 0) return TY();
    if (index == static_cast<int>(x_.size())) {
      if (x > x_.back()) {
        return TY();
      } else {
        index--;
      }
    }
    DCHECK_GT(x_[index], x_[index - 1]);
    return (y_[index] - y_[index - 1]) * x_slope_[index - 1];
  }

  const std::vector<TX>& x() const { return x_; }
  const std::vector<TY>& y() const { return y_; }

 private:
  std::vector<TX> x_;
  std::vector<TY> y_;
  std::vector<TX> x_slope_;
  LERPER lerper_;
};

template <typename LERPER = Lerper<double, double>>
PiecewiseLinearFunction<double, double, LERPER>
PiecewiseLinearFunctionFromProto(
    const PiecewiseLinearFunctionDoubleProto& proto) {
  CHECK_EQ(proto.x_size(), proto.y_size());
  std::vector<double> x(proto.x().begin(), proto.x().end());
  std::vector<double> y(proto.y().begin(), proto.y().end());
  return PiecewiseLinearFunction<double, double, LERPER>(std::move(x),
                                                         std::move(y));
}

template <typename LERPER = Lerper<double, double>>
PiecewiseLinearFunction<double, double, LERPER>
PiecewiseLinearFunctionFromProtocolScale(
    const PiecewiseLinearFunctionDoubleProto& proto, double scale) {
  CHECK_EQ(proto.x_size(), proto.y_size());
  std::vector<double> x(proto.x().begin(), proto.x().end());
  std::vector<double> y(proto.y().begin(), proto.y().end());
  std::transform(y.begin(), y.end(), y.begin(),
                 [&](double x) { return x * scale; });
  return PiecewiseLinearFunction<double, double, LERPER>(std::move(x),
                                                         std::move(y));
}

template <typename LERPER = Lerper<Vec2d, double>>
PiecewiseLinearFunction<Vec2d, double, LERPER> PiecewiseLinearFunctionFromProto(
    const PiecewiseLinearFunctionVec2dProto& proto) {
  CHECK_EQ(proto.x_size(), proto.y_size());
  std::vector<double> x(proto.x().begin(), proto.x().end());
  std::vector<Vec2d> y;
  for (const auto& proto_y : proto.y()) y.push_back(Vec2dFromProto(proto_y));
  return PiecewiseLinearFunction<Vec2d, double, LERPER>(std::move(x),
                                                        std::move(y));
}

template <typename LERPER = Lerper<Vec3d, double>>
PiecewiseLinearFunction<Vec3d, double, LERPER> PiecewiseLinearFunctionFromProto(
    const PiecewiseLinearFunctionVec3dProto& proto) {
  CHECK_EQ(proto.x_size(), proto.y_size());
  std::vector<double> x(proto.x().begin(), proto.x().end());
  std::vector<Vec3d> y;
  for (const auto& proto_y : proto.y()) y.push_back(Vec3dFromProto(proto_y));
  return PiecewiseLinearFunction<Vec3d, double, LERPER>(std::move(x),
                                                        std::move(y));
}

template <typename LERPER = Lerper<Vec4d, double>>
PiecewiseLinearFunction<Vec4d, double, LERPER> PiecewiseLinearFunctionFromProto(
    const PiecewiseLinearFunctionVec4dProto& proto) {
  CHECK_EQ(proto.x_size(), proto.y_size());
  std::vector<double> x(proto.x().begin(), proto.x().end());
  std::vector<Vec4d> y;
  for (const auto& proto_y : proto.y()) y.push_back(Vec4dFromProto(proto_y));
  return PiecewiseLinearFunction<Vec4d, double, LERPER>(std::move(x),
                                                        std::move(y));
}

template <typename LERPER = Lerper<double, double>>
void PiecewiseLinearFunctionToProto(
    const PiecewiseLinearFunction<double, double, LERPER>& plf,
    PiecewiseLinearFunctionDoubleProto* proto) {
  proto->Clear();
  for (const double x : plf.x()) proto->add_x(x);
  for (const double y : plf.y()) proto->add_y(y);
}

template <typename LERPER = Lerper<Vec2d, double>>
void PiecewiseLinearFunctionToProto(
    const PiecewiseLinearFunction<Vec2d, double, LERPER>& plf,
    PiecewiseLinearFunctionVec2dProto* proto) {
  proto->Clear();
  for (const double x : plf.x()) proto->add_x(x);
  for (const Vec2d& y : plf.y()) Vec2dToProto(y, proto->add_y());
}

template <typename LERPER = Lerper<Vec3d, double>>
void PiecewiseLinearFunctionToProto(
    const PiecewiseLinearFunction<Vec3d, double, LERPER>& plf,
    PiecewiseLinearFunctionVec3dProto* proto) {
  proto->Clear();
  for (const double x : plf.x()) proto->add_x(x);
  for (const Vec3d& y : plf.y()) Vec3dToProto(y, proto->add_y());
}

template <typename LERPER = Lerper<Vec4d, double>>
void PiecewiseLinearFunctionToProto(
    const PiecewiseLinearFunction<Vec4d, double, LERPER>& plf,
    PiecewiseLinearFunctionVec4dProto* proto) {
  proto->Clear();
  for (const double x : plf.x()) proto->add_x(x);
  for (const Vec4d& y : plf.y()) Vec4dToProto(y, proto->add_y());
}

template <typename T, typename TS>
class SqrtInterpolator {
 public:
  T operator()(T a, T b, TS alpha) const {
    return std::sqrt(Lerp(Sqr(a), Sqr(b), alpha));
  }
};

template <typename TY, typename TX = double>
using PiecewiseSqrtFunction =
    PiecewiseLinearFunction<TY, TX, SqrtInterpolator<TY, TX>>;

}  // namespace e2e_noa

#endif
