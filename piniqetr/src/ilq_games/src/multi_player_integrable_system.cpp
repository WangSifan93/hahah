///////////////////////////////////////////////////////////////////////////////
//
// Base class for all multi-player *integrable* dynamical systems.
// Supports (discrete-time) linearization and integration.
//
///////////////////////////////////////////////////////////////////////////////

#include "ilq_games/include/dynamics/multi_player_integrable_system.h"

#include <vector>

#include "ilq_games/include/utils/ilq_types.h"
#include "ilq_games/include/utils/operating_point.h"
#include "ilq_games/include/utils/strategy.h"

namespace e2e_noa::planning {

bool MultiPlayerIntegrableSystem::integrate_using_euler_ = false;

VectorXf MultiPlayerIntegrableSystem::Integrate(
    Time t0, Time t, const VectorXf& x0, const OperatingPoint& operating_point,
    const std::vector<Strategy>& strategies) const {
  CHECK_GE(t, t0);
  CHECK_GE(t0, operating_point.t0);
  CHECK_EQ(strategies.size(), NumPlayers());

  std::vector<VectorXf> us(NumPlayers());

  // Compute current timestep and final timestep.
  const Time relative_t0 = t0 - operating_point.t0;
  const size_t current_timestep =
      static_cast<size_t>(relative_t0 / time::getTimeStep());

  const Time relative_t = t - operating_point.t0;
  const size_t final_timestep =
      static_cast<size_t>(relative_t / time::getTimeStep());

  // Handle case where 't0' is after 'operating_point.t0' by integrating from
  // 't0' to the next discrete timestep.
  VectorXf x(x0);
  if (t0 > operating_point.t0)
    x = IntegrateToNextTimeStep(t0, x0, operating_point, strategies);

  // Integrate forward step by step up to timestep including t.
  x = Integrate(current_timestep + 1, final_timestep, x, operating_point,
                strategies);

  // Integrate forward from this timestep to t.
  return IntegrateFromPriorTimeStep(t, x, operating_point, strategies);
}

VectorXf MultiPlayerIntegrableSystem::Integrate(
    size_t initial_timestep, size_t final_timestep, const VectorXf& x0,
    const OperatingPoint& operating_point,
    const std::vector<Strategy>& strategies) const {
  VectorXf x(x0);
  std::vector<VectorXf> us(NumPlayers());
  for (size_t kk = initial_timestep; kk < final_timestep; kk++) {
    const Time t = operating_point.t0 + kk * time::getTimeStep();

    // Populate controls for all players.
    for (PlayerIndex ii = 0; ii < NumPlayers(); ii++)
      us[ii] = strategies[ii](kk, x - operating_point.xs[kk],
                              operating_point.us[kk][ii]);

    x = Integrate(t, time::getTimeStep(), x, us);
  }

  return x;
}

VectorXf MultiPlayerIntegrableSystem::IntegrateToNextTimeStep(
    Time t0, const VectorXf& x0, const OperatingPoint& operating_point,
    const std::vector<Strategy>& strategies) const {
  CHECK_GE(t0, operating_point.t0);

  // Compute remaining time this timestep.
  const Time relative_t0 = t0 - operating_point.t0;
  const size_t current_timestep = static_cast<size_t>(
      (relative_t0 +
       constants::kSmallNumber)  // Add to avoid inadvertently subtracting 1.
      / time::getTimeStep());
  const Time remaining_time_this_step =
      time::getTimeStep() * (current_timestep + 1) - relative_t0;
  CHECK_LT(remaining_time_this_step,
           time::getTimeStep() + constants::kSmallNumber);
  CHECK_LT(current_timestep, operating_point.xs.size());

  // Interpolate x0_ref.
  const float frac = remaining_time_this_step / time::getTimeStep();
  const VectorXf x0_ref =
      (current_timestep + 1 < operating_point.xs.size())
          ? frac * operating_point.xs[current_timestep] +
                (1.0 - frac) * operating_point.xs[current_timestep + 1]
          : operating_point.xs.back();

  // Populate controls for each player.
  std::vector<VectorXf> us(NumPlayers());
  for (PlayerIndex ii = 0; ii < NumPlayers(); ii++)
    us[ii] = strategies[ii](current_timestep, x0 - x0_ref,
                            operating_point.us[current_timestep][ii]);

  return Integrate(t0, remaining_time_this_step, x0, us);
}

VectorXf MultiPlayerIntegrableSystem::IntegrateFromPriorTimeStep(
    Time t, const VectorXf& x0, const OperatingPoint& operating_point,
    const std::vector<Strategy>& strategies) const {
  // Compute time until next timestep.
  const Time relative_t = t - operating_point.t0;
  const size_t current_timestep =
      static_cast<size_t>(relative_t / time::getTimeStep());
  const Time remaining_time_until_t =
      relative_t - time::getTimeStep() * current_timestep;
  CHECK_LT(current_timestep, operating_point.xs.size()) << t;
  CHECK_LT(remaining_time_until_t, time::getTimeStep());

  // Populate controls for each player.
  std::vector<VectorXf> us(NumPlayers());
  for (PlayerIndex ii = 0; ii < NumPlayers(); ii++) {
    us[ii] = strategies[ii](current_timestep,
                            x0 - operating_point.xs[current_timestep],
                            operating_point.us[current_timestep][ii]);
  }

  return Integrate(operating_point.t0 + time::getTimeStep() * current_timestep,
                   remaining_time_until_t, x0, us);
}

VectorXf MultiPlayerIntegrableSystem::Integrate(
    Time t0, Time time_interval, const Eigen::Ref<VectorXf>& x0,
    const std::vector<Eigen::Ref<VectorXf>>& us) const {
  std::vector<VectorXf> eval_us(us.size());
  std::transform(us.begin(), us.end(), eval_us.begin(),
                 [](const Eigen::Ref<VectorXf>& u) { return u.eval(); });

  return Integrate(t0, time_interval, x0.eval(), eval_us);
};

}  // namespace e2e_noa::planning
