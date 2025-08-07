//////////////////////////////////////////////////////////////////////////////
//
// Container to store solver logs.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef ILQGAMES_UTILS_LOG_H
#define ILQGAMES_UTILS_LOG_H

#include <math.h>

#include <vector>

#include "ilq_games/include/utils/ilq_types.h"
#include "ilq_games/include/utils/operating_point.h"
#include "ilq_games/include/utils/strategy.h"
namespace e2e_noa::planning {

struct VehicleStaticData;
struct VehicleDynamicData;
// Default experiment name to use.
std::string DefaultExperimentName();

class SolverLog {
 public:
  ~SolverLog() {}
  SolverLog() {}

  // Add a new solver iterate.
  void AddSolverIterate(const OperatingPoint &operating_point,
                        const std::vector<Strategy> &strategies) {
    operating_points_.push_back(operating_point);
    strategies_.push_back(strategies);
  }

  // Add a whole other log.
  void AddLog(const SolverLog &log) {
    for (size_t ii = 0; ii < log.NumIterates(); ii++) {
      AddSolverIterate(log.operating_points_[ii], log.strategies_[ii]);
    }
  }

  // Clear all but first entry. Used by the solver to return initial conditions
  // upon failure.
  void ClearAllButFirstIterate() {
    constexpr size_t kOneIterate = 1;

    CHECK_GE(operating_points_.size(), kOneIterate);
    operating_points_.resize(kOneIterate, operating_points_.front());
    strategies_.resize(kOneIterate);
  }

  Time InitialTime() const {
    return (NumIterates() > 0) ? operating_points_[0].t0 : 0.0;
  }
  Time FinalTime() const {
    return (NumIterates() > 0) ? IndexToTime(operating_points_[0].xs.size() - 1)
                               : 0.0;
  }
  PlayerIndex NumPlayers() const { return strategies_[0].size(); }
  size_t NumIterates() const { return operating_points_.size(); }

  const std::vector<Strategy> &InitialStrategies() const {
    return strategies_.front();
  }
  const OperatingPoint &InitialOperatingPoint() const {
    return operating_points_.front();
  }
  const std::vector<Strategy> &FinalStrategies() const {
    return strategies_.back();
  }
  const OperatingPoint &FinalOperatingPoint() const {
    return operating_points_.back();
  }

  VectorXf InterpolateState(size_t iterate, Time t) const;
  float InterpolateState(size_t iterate, Time t, Dimension dim) const;
  VectorXf InterpolateControl(size_t iterate, Time t, PlayerIndex player) const;
  float InterpolateControl(size_t iterate, Time t, PlayerIndex player,
                           Dimension dim) const;

  std::vector<MatrixXf> Ps(size_t iterate, size_t time_index) const;
  std::vector<VectorXf> alphas(size_t iterate, size_t time_index) const;
  MatrixXf P(size_t iterate, size_t time_index, PlayerIndex player) const;
  VectorXf alpha(size_t iterate, size_t time_index, PlayerIndex player) const;

  VectorXf State(size_t iterate, size_t time_index) const {
    return operating_points_[iterate].xs[time_index];
  }
  float State(size_t iterate, size_t time_index, Dimension dim) const {
    return operating_points_[iterate].xs[time_index](dim);
  }
  VectorXf Control(size_t iterate, size_t time_index,
                   PlayerIndex player) const {
    return operating_points_[iterate].us[time_index][player];
  }
  float Control(size_t iterate, size_t time_index, PlayerIndex player,
                Dimension dim) const {
    return operating_points_[iterate].us[time_index][player](dim);
  }

  std::vector<MatrixXf> Ps(size_t iterate, Time t) const {
    return Ps(iterate, TimeToIndex(t));
  }
  std::vector<VectorXf> alphas(size_t iterate, Time t) const {
    return alphas(iterate, TimeToIndex(t));
  }
  MatrixXf P(size_t iterate, Time t, PlayerIndex player) const {
    return P(iterate, TimeToIndex(t), player);
  }
  VectorXf alpha(size_t iterate, Time t, PlayerIndex player) const {
    return alpha(iterate, TimeToIndex(t), player);
  }

  // Get index corresponding to the time step immediately before the given time.
  size_t TimeToIndex(Time t) const {
    return static_cast<size_t>(
        std::max<Time>(constants::kSmallNumber, t - InitialTime()) /
        time::getTimeStep());
  }

  // Get time stamp corresponding to a particular index.
  Time IndexToTime(size_t idx) const {
    return InitialTime() + time::getTimeStep() * static_cast<Time>(idx);
  }

  // Save to disk.
  bool Save(const std::vector<VehicleStaticData> &vehicles_static_dataset,
            const std::vector<VehicleDynamicData> &vehicles_dynamic_dataset,
            const std::string &experiment_name = DefaultExperimentName(),
            const uint64_t seq_num = 0, const bool enable_online_log = false,
            const bool enable_offline_log = false) const;

 private:
  // Operating points, strategies, total costs, and cumulative runtime indexed
  // by solver iterate.
  std::vector<OperatingPoint> operating_points_;
  std::vector<std::vector<Strategy>> strategies_;
};  // class SolverLog

}  // namespace e2e_noa::planning

#endif
