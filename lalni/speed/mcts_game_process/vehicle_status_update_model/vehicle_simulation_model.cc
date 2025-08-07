#include "vehicle_simulation_model.h"

#include "math/double.h"

namespace e2e_noa {
namespace planning {
constexpr double kEpsilon = 1e-3;
constexpr double kMaxTime = 60.0;
constexpr double kTimeThreshold = 0.1;
constexpr int kMaxDepth = 1000;

VehicleSimulationModel::VehicleSimulationModel(const double min_v,
                                               const double max_v,
                                               const double t_1,
                                               const double jerk) {
  t_1_ = std::max(t_1, 0.0);
  jerk_ = jerk;
  min_v_ = std::max(min_v, 0.0);
  max_v_ = std::max(max_v, 0.0);
  if (min_v_ > max_v_) {
    min_v_ = std::max(max_v_, 0.0);
    max_v_ = std::max(min_v_, 0.0);
  }
  if (ad_e2e::planning::math::Double::Compare(jerk_, 0.0, kEpsilon) ==
          ad_e2e::planning::math::Double::CompareType::EQUAL ||
      ad_e2e::planning::math::Double::Compare(t_1_, 0.0, kEpsilon) ==
          ad_e2e::planning::math::Double::CompareType::EQUAL ||
      ad_e2e::planning::math::Double::Compare(max_v_, min_v_, kEpsilon) ==
          ad_e2e::planning::math::Double::CompareType::EQUAL) {
    is_valid_ = false;
  } else {
    is_valid_ = true;
  }
}

void VehicleSimulationModel::SetInitialState(const double s_0, const double v_0,
                                             const double a_0) {
  if (!is_valid_) {
    return;
  }
  s_0_ = s_0;
  v_0_ = std::max(kEpsilon, v_0);
  a_0_ = a_0;
  jerk_ = std::copysign(jerk_, -a_0_);
  Init();
  is_inited_ = true;
}

void VehicleSimulationModel::Init() {
  is_stopped_ = false;
  stop_t_ = std::numeric_limits<double>::max();
  stop_s_ = std::numeric_limits<double>::max();

  if (v_0_ > max_v_) {
    max_v_ = v_0_ + kEpsilon;
  }
  if (v_0_ < min_v_) {
    if (v_0_ < kEpsilon) {
      v_0_ = min_v_ + kEpsilon;
    } else {
      min_v_ = std::max(0.0, v_0_ - kEpsilon);
    }
  }

  if (ad_e2e::planning::math::Double::Compare(a_0_, 0.0, kEpsilon) ==
      ad_e2e::planning::math::Double::CompareType::EQUAL) {
    a_0_ = 0.0;
    v_1_ = v_0_;
    s_1_ = s_0_ + v_0_ * t_1_;
    t_2_ = t_1_;
    v_2_ = v_1_;
    s_2_ = s_1_;
  } else if (a_0_ > 0.0) {
    const double v_t_1 = v_0_ + a_0_ * t_1_;
    if (v_t_1 > max_v_) {
      t_1_ = (max_v_ - v_0_) / a_0_;
      v_1_ = max_v_;
      s_1_ = s_0_ + (max_v_ * max_v_ - v_0_ * v_0_) / (2.0 * a_0_);

      t_2_ = t_1_;
      v_2_ = v_1_;
      s_2_ = s_1_;
    } else {
      v_1_ = v_t_1;
      s_1_ = s_0_ + v_0_ * t_1_ + 0.5 * a_0_ * t_1_ * t_1_;
      const double dt = std::abs(a_0_ / jerk_);
      const double v_t_2 = v_t_1 + 0.5 * a_0_ * dt;
      if (v_t_2 > max_v_) {
        const double delta_v = max_v_ - v_1_;
        const double final_a = std::sqrt(a_0_ * a_0_ + 2.0 * jerk_ * delta_v);
        const double delta_t = (final_a - a_0_) / jerk_;
        t_2_ = t_1_ + delta_t;
        v_2_ = max_v_;
        s_2_ = s_1_ + v_1_ * delta_t + 0.5 * a_0_ * std::pow(delta_t, 2) +
               1.0 / 6.0 * jerk_ * std::pow(delta_t, 3);
      } else {
        t_2_ = t_1_ + dt;
        v_2_ = v_t_2;
        s_2_ = s_1_ + v_t_1 * dt + 0.5 * a_0_ * std::pow(dt, 2) +
               1.0 / 6.0 * jerk_ * std::pow(dt, 3);
      }
    }
  } else {
    double v_t_1 = v_0_ + a_0_ * t_1_;
    if (v_t_1 < min_v_) {
      t_1_ = (min_v_ - v_0_) / a_0_;
      v_1_ = min_v_;
      s_1_ = s_0_ + (min_v_ * min_v_ - v_0_ * v_0_) / (2.0 * a_0_);

      if (v_1_ < kEpsilon) {
        is_stopped_ = true;
        stop_t_ = t_1_;
        stop_s_ = s_1_;
      }

      t_2_ = t_1_;
      v_2_ = v_1_;
      s_2_ = s_1_;
    } else {
      v_1_ = v_t_1;
      s_1_ = s_0_ + v_0_ * t_1_ + 0.5 * a_0_ * t_1_ * t_1_;
      const double dt = std::abs(a_0_) / jerk_;
      const double v_t_2 = v_t_1 + 0.5 * a_0_ * dt;
      if (v_t_2 < min_v_) {
        const double delta_v = min_v_ - v_1_;
        const double final_a = -std::sqrt(a_0_ * a_0_ + 2.0 * jerk_ * delta_v);
        const double delta_t = (final_a - a_0_) / jerk_;
        t_2_ = t_1_ + delta_t;
        v_2_ = min_v_;
        s_2_ = s_1_ + v_1_ * delta_t + 0.5 * a_0_ * std::pow(delta_t, 2) +
               1.0 / 6.0 * jerk_ * std::pow(delta_t, 3);
        if (v_2_ < kEpsilon) {
          is_stopped_ = true;
          stop_t_ = t_2_;
          stop_s_ = s_2_;
        }
      } else {
        t_2_ = t_1_ + dt;
        v_2_ = v_t_2;
        s_2_ = s_1_ + v_t_1 * dt + 0.5 * a_0_ * std::pow(dt, 2) +
               1.0 / 6.0 * jerk_ * std::pow(dt, 3);
        if (v_2_ < kEpsilon) {
          is_stopped_ = true;
          stop_t_ = t_2_;
          stop_s_ = s_2_;
        }
      }
    }
  }
}

double VehicleSimulationModel::ComputeDistance(const double t) const {
  const double t_tmp = std::max(0.0, t);
  if (t_tmp < t_1_) {
    return s_0_ + v_0_ * t_tmp + 0.5 * a_0_ * std::pow(t_tmp, 2);
  } else if (t_tmp < t_2_) {
    return s_1_ + v_1_ * (t_tmp - t_1_) +
           0.5 * a_0_ * std::pow(t_tmp - t_1_, 2) +
           1.0 / 6.0 * jerk_ * std::pow(t_tmp - t_1_, 3);
  } else {
    return s_2_ + v_2_ * (t_tmp - t_2_);
  }
  return 0.0;
}

double VehicleSimulationModel::ComputeVelocity(const double t) const {
  const double t_tmp = std::max(0.0, t);
  if (t_tmp < t_1_) {
    return v_0_ + a_0_ * t_tmp;
  } else if (t_tmp < t_2_) {
    return v_1_ + a_0_ * (t_tmp - t_1_) +
           0.5 * jerk_ * std::pow(t_tmp - t_1_, 2);
  } else {
    return v_2_;
  }
  return 0.0;
}

double VehicleSimulationModel::ComputeAcceleration(const double t) const {
  const double t_tmp = std::max(0.0, t);
  if (t_tmp < t_1_) {
    return a_0_;
  } else if (t_tmp < t_2_) {
    return a_0_ + jerk_ * (t_tmp - t_1_);
  } else {
    return 0.0;
  }
  return 0.0;
}

double VehicleSimulationModel::ComputeArriveTime(const double s) const {
  if (is_stopped_ && s > stop_s_) {
    return kMaxTime;
  }

  if (s < s_0_) {
    return 0.0;
  }

  if (ad_e2e::planning::math::Double::Compare(a_0_, 0.0, kEpsilon) ==
      ad_e2e::planning::math::Double::CompareType::EQUAL) {
    return (s - s_0_) / (v_0_);
  }

  if (s < s_1_) {
    const double final_v = std::sqrt(v_0_ * v_0_ + 2.0 * a_0_ * (s - s_0_));
    return (final_v - v_0_) / a_0_;
  } else if (s < s_2_) {
    double t_min = t_1_;
    double t_max = t_2_;
    double t = (t_1_ + t_2_) * 0.5;
    int depth = 0;
    while (t_min < t_max && t_max - t_min > kTimeThreshold) {
      if (depth > kMaxDepth) {
        break;
      }

      t = (t_min + t_max) * 0.5;
      const double s_tmp = ComputeDistance(t);
      if (ad_e2e::planning::math::Double::Compare(s_tmp, s) ==
          ad_e2e::planning::math::Double::CompareType::EQUAL) {
        break;
      } else if (s_tmp < s) {
        t_min = t;
      } else if (s_tmp > s) {
        t_max = t;
      }
      depth++;
    }
    return t;
  } else {
    return t_2_ + (s - s_2_) / (v_2_ + kEpsilon);
  }

  return 0.0;
}

std::pair<double, double> VehicleSimulationModel::ComputeVelocityAndDistance(
    const double t) const {
  const double v = ComputeVelocity(t);
  const double s = ComputeDistance(t);
  return std::make_pair(v, s);
}

}  // namespace planning
}  // namespace e2e_noa
