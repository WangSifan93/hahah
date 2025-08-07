#ifndef PLANNER_SPEED_MCTS_VEHICLE_SIMULATION_MODEL_H_
#define PLANNER_SPEED_MCTS_VEHICLE_SIMULATION_MODEL_H_
#include <cmath>

namespace e2e_noa {
namespace planning {
class VehicleSimulationModel {
 public:
  VehicleSimulationModel() = default;

  VehicleSimulationModel(const double min_v, const double max_v,
                         const double t_1, const double jerk);

  ~VehicleSimulationModel() = default;

  void SetInitialState(const double s_0, const double v_0, const double a_0);

  bool IsInited() const { return is_inited_; }

  double ComputeDistance(const double t) const;

  double ComputeVelocity(const double t) const;

  double ComputeAcceleration(const double t) const;

  double ComputeArriveTime(const double s) const;

  std::pair<double, double> ComputeVelocityAndDistance(const double t) const;

  bool IsStopped() const { return is_stopped_; }

  double GetStopTime() const { return stop_t_; }

  double GetStopDistance() const { return stop_s_; }

 private:
  void Init();

 private:
  double s_0_ = 0.0;
  double v_0_ = 0.0;
  double a_0_ = 0.0;

  double jerk_ = 0.0;

  double min_v_ = 0.0;
  double max_v_ = std::numeric_limits<double>::max();

  double t_1_ = 0.0;
  double s_1_ = 0.0;
  double v_1_ = 0.0;

  double t_2_ = 0.0;
  double s_2_ = 0.0;
  double v_2_ = 0.0;

  bool is_stopped_ = false;
  double stop_t_ = 0.0;
  double stop_s_ = 0.0;

  bool is_valid_ = false;
  bool is_inited_ = false;
};
}  // namespace planning
}  // namespace e2e_noa
#endif
