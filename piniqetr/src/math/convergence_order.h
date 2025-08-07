#ifndef ONBOARD_MATH_CONVERGENCE_ORDER_H_
#define ONBOARD_MATH_CONVERGENCE_ORDER_H_

#include <functional>

namespace e2e_noa {

int AssessConvergenceOrder(const std::function<double(double)>& f);

}

#endif
