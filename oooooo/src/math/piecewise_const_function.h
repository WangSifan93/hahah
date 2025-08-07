#ifndef ONBOARD_MATH_PIECEWISE_CONST_FUNCTION_H_
#define ONBOARD_MATH_PIECEWISE_CONST_FUNCTION_H_

#include <glog/logging.h>

#include <algorithm>
#include <utility>
#include <vector>

#include "piecewise_const_function.pb.h"

namespace e2e_noa {

template <typename TX, typename TY>
class PiecewiseConstFunction final {
 public:
  PiecewiseConstFunction(std::vector<TX> x, std::vector<TY> y)
      : x_(std::move(x)), y_(std::move(y)) {
    CHECK_EQ(x_.size(), y_.size() + 1);
    CHECK_GT(x_.size(), 1);
  }

  TY operator()(TX x) const { return Evaluate(x); }

  TY Evaluate(TX x) const {
    const auto it = std::upper_bound(x_.begin(), x_.end(), x);
    if (it == x_.end()) {
      return y_.back();
    }
    if (it == x_.begin()) {
      return y_.front();
    }
    return y_[it - x_.begin() - 1];
  }

 private:
  std::vector<TX> x_;
  std::vector<TY> y_;
};

PiecewiseConstFunction<double, double> PiecewiseConstFunctionFromProtocol(
    const PiecewiseConstFunctionDoubleProto& proto);

}  // namespace e2e_noa

#endif
