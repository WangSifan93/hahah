#ifndef ONBOARD_PLANNER_UTIL_MOTION_UTIL_H_
#define ONBOARD_PLANNER_UTIL_MOTION_UTIL_H_

#include <optional>

namespace e2e_noa::planning {

class ConstJerkBrakingSpacetime {
 public:
  static constexpr double kSixInv = 1.0 / 6.0;

  ConstJerkBrakingSpacetime(double v0, double a0, double min_acc, double jerk);

  double GetS(double t) const;

  double GetV(double t) const;

  double GetA(double t) const;

  double t_stop() const { return t_stop_; }

 private:
  double v0_ = 0.0;
  double a0_ = 0.0;
  double min_acc_ = 0.0;
  double jerk_ = 0.0;

  double t_stop_ = 0.0;

  bool v_reach_zero_first_ = false;

  double min_acc_time_ = 0.0;
  double s_min_acc_ = 0.0;
  double v_min_acc_ = 0.0;
};

class ConstJerkAcceleratingSpacetime {
 public:
  static constexpr double kSixInv = 1.0 / 6.0;
  static constexpr double kJerkEpsilon = 1e-5;

  ConstJerkAcceleratingSpacetime(double v0, double a0, double max_acc,
                                 double jerk);

  double GetS(double t) const;

  double GetV(double t) const;

  double GetA(double t) const;

 private:
  double v0_ = 0.0;
  double a0_ = 0.0;
  double max_acc_ = 0.0;
  double jerk_ = 0.0;

  double t_stop_ = 0.0;
  double v_reach_zero_first_ = false;

  double max_acc_time_ = 0.0;

  double s_max_acc_ = 0.0;
  double v_max_acc_ = 0.0;

  double s_stop_ = 0.0;
  double t_restart_ = 0.0;
};

class ConstJerkSpacetime {
 public:
  ConstJerkSpacetime(double v0, double a0, double min_acc, double max_acc,
                     double jerk) {
    if (jerk >= 0.0) {
      accelerating_spacetime_.emplace(v0, a0, max_acc, jerk);
    } else {
      braking_spacetime_.emplace(v0, a0, min_acc, jerk);
    }
  }

  double GetS(double t) const {
    return accelerating_spacetime_ ? (accelerating_spacetime_->GetS(t))
                                   : (braking_spacetime_->GetS(t));
  }

  double GetV(double t) const {
    return accelerating_spacetime_ ? (accelerating_spacetime_->GetV(t))
                                   : (braking_spacetime_->GetV(t));
  }

  double GetA(double t) const {
    return accelerating_spacetime_ ? (accelerating_spacetime_->GetA(t))
                                   : (braking_spacetime_->GetA(t));
  }

 private:
  std::optional<ConstJerkBrakingSpacetime> braking_spacetime_;
  std::optional<ConstJerkAcceleratingSpacetime> accelerating_spacetime_;
};

}  // namespace e2e_noa::planning
#endif
