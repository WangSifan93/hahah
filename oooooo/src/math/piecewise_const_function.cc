#include "math/piecewise_const_function.h"

#include <vector>

#include "piecewise_const_function.pb.h"

namespace e2e_noa {

PiecewiseConstFunction<double, double> PiecewiseConstFunctionFromProtocol(
    const PiecewiseConstFunctionDoubleProto& proto) {
  CHECK_EQ(proto.x_size(), proto.y_size() + 1);
  std::vector<double> x(proto.x().begin(), proto.x().end());
  std::vector<double> y(proto.y().begin(), proto.y().end());
  return PiecewiseConstFunction<double, double>(std::move(x), std::move(y));
}

}  // namespace e2e_noa
