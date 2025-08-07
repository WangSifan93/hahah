///////////////////////////////////////////////////////////////////////////////
//
// Base class for all multi-player dynamical systems. Supports (discrete-time)
// linearization and integration.
//
///////////////////////////////////////////////////////////////////////////////

#include "ilq_games/include/dynamics/multi_player_dynamical_system.h"

#include <glog/logging.h>

#include "ilq_games/include/utils/ilq_types.h"
#include "ilq_games/include/utils/linear_dynamics_approximation.h"

namespace e2e_noa::planning {

VectorXf MultiPlayerDynamicalSystem::Integrate(
    Time t0, Time time_interval, const VectorXf& x0,
    const std::vector<VectorXf>& us) const {
  VectorXf x(x0);

  if (integrate_using_euler_) {
    x += time_interval * Evaluate(t0, x0, us);
  } else {
    // Number of integration steps and corresponding time step.
    constexpr size_t kNumIntegrationSteps = 2;
    const double dt = time_interval / static_cast<Time>(kNumIntegrationSteps);

    // RK4 integration. See https://en.wikipedia.org/wiki/Runge-Kutta_methods
    // for further details.
    for (Time t = t0; t < t0 + time_interval - 0.5 * dt; t += dt) {
      const VectorXf k1 = dt * Evaluate(t, x, us);
      const VectorXf k2 = dt * Evaluate(t + 0.5 * dt, x + 0.5 * k1, us);
      const VectorXf k3 = dt * Evaluate(t + 0.5 * dt, x + 0.5 * k2, us);
      const VectorXf k4 = dt * Evaluate(t + dt, x + k3, us);

      x += (k1 + 2.0 * (k2 + k3) + k4) / 6.0;
    }
  }

  return x;
}

}  // namespace e2e_noa::planning
