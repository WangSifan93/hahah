#ifndef PLANNER_SPEED_OPTIMIZER_CONSTRAINT_MANAGER_H_
#define PLANNER_SPEED_OPTIMIZER_CONSTRAINT_MANAGER_H_

#include <map>
#include <optional>
#include <utility>
#include <vector>

#include "absl/types/span.h"
#include "speed_planning.pb.h"
#include "speed_planning_params.pb.h"

namespace e2e_noa::planning {

class SpeedOptimizerConstraintManager {
 public:
  using Sfp = SpeedPlanningParamsProto;

  enum class BoundDirType {
    UPPER = 0,
    LOWER = 1,
  };

  enum class BoundStrType {
    WEAK = 0,
    MEDIUM = 1,
    STRONG = 2,
  };

  struct SoftConstraintInput {
    int knot_idx = 0;
    double weight = 0.0;
    double bound = 0.0;
  };

  struct SoftConstraint {
    int knot_idx = 0;
    int slack_idx = 0;
    double bound = 0.0;
  };

  struct HardConstraint {
    int knot_idx = 0;
    double lower_bound = 0.0;
    double upper_bound = 0.0;
  };

  struct SlackWeight {
    int slack_idx = 0;
    double weight = 0.0;
  };

  struct SSoftConstraintData {
    std::map<std::pair<int, BoundStrType>, int> slack_idx_map;

    std::vector<std::vector<std::optional<SoftConstraint>>> soft_constraints;

    std::map<int, std::map<int, double>> slack_weight_map;
  };

  struct SpeedSoftConstraintData {
    std::map<SpeedLimitLevelProto::Level, int> slack_idx_map;

    std::vector<std::vector<std::optional<SoftConstraint>>> soft_constraints;

    std::map<int, std::map<int, double>> slack_weight_map;
  };

  struct AccelSoftConstraintData {
    std::vector<SoftConstraint> soft_constraints;
    std::map<int, double> slack_weight_map;
  };

  struct JerkSoftConstraintData {
    std::vector<SoftConstraint> soft_constraints;
    std::map<int, double> slack_weight_map;
  };

  SpeedOptimizerConstraintManager(std::vector<double> piecewise_time_range,
                                  double delta_t, int total_knot_num);

  void FilterObjectSConstraint(int knot_idx);

  void FilterObjectSpeedConstraint(int knot_idx);

  void AddSoftSLowerConstraint(BoundStrType strength_type,
                               const SoftConstraintInput& constraint_input);

  void AddSoftSUpperConstraint(BoundStrType strength_type,
                               const SoftConstraintInput& constraint_input);

  void AddHardSConstraint(int knot_idx, double lower_bound, double upper_bound);

  void AddSoftSpeedLowerConstraints(
      SpeedLimitLevelProto::Level speed_limit_level,
      const SoftConstraintInput& constraint_input);

  void AddSoftSpeedUpperConstraints(
      SpeedLimitLevelProto::Level speed_limit_level,
      const SoftConstraintInput& constraint_input);

  void AddHardSpeedConstraint(int knot_idx, double lower_bound,
                              double upper_bound);

  void AddAccelConstraint(int knot_idx, double lower_bound, double upper_bound);

  void AddJerkConstraint(int knot_idx, double lower_bound, double upper_bound);

  void AddAccelSoftUpperConstraint(const SoftConstraintInput& accel_constraint);

  void AddAccelSoftLowerConstraint(const SoftConstraintInput& accel_constraint);

  void AddJerkSoftUpperConstraint(const SoftConstraintInput& jerk_constraint);

  void AddJerkSoftLowerConstraint(const SoftConstraintInput& jerk_constraint);

  std::vector<SoftConstraint> GetLowerConstraintOfSoftS() const;

  std::vector<SoftConstraint> GetUpperConstraintOfSoftS() const;

  std::vector<SoftConstraint> GetLowerConstraintOfSpeed() const;

  std::vector<SoftConstraint> GetUpperConstraintOfSpeed() const;

  absl::Span<const SoftConstraint> GetLowerConstraintOfSoftAccel() const {
    return accel_lower_data_.soft_constraints;
  }

  absl::Span<const SoftConstraint> GetUpperConstraintOfSoftAccel() const {
    return accel_upper_data_.soft_constraints;
  }

  absl::Span<const SoftConstraint> GetLowerConstraintOfSoftJerk() const {
    return jerk_lower_data_.soft_constraints;
  }

  absl::Span<const SoftConstraint> GetUpperConstraintOfSoftJerk() const {
    return jerk_upper_data_.soft_constraints;
  }

  absl::Span<const HardConstraint> hard_s_constraint() const {
    return hard_s_constraint_;
  }

  std::vector<SlackWeight> GetLowerSlackWeightOfSoftS() const;

  std::vector<SlackWeight> GetUpperSlackWeightOfSoftS() const;

  std::vector<SlackWeight> GetLowerSlackWeightOfSpeed() const;

  std::vector<SlackWeight> GetUpperSlackWeightOfSpeed() const;

  absl::Span<const HardConstraint> hard_v_constraint() const {
    return hard_v_constraint_;
  }

  const std::map<int, double>& GetLowerSlackWeightOfAccel() const {
    return accel_lower_data_.slack_weight_map;
  }

  const std::map<int, double>& GetUpperSlackWeightOfAccel() const {
    return accel_upper_data_.slack_weight_map;
  }

  absl::Span<const HardConstraint> hard_a_constraint() const {
    return hard_a_constraint_;
  }

  const std::map<int, double>& GetLowerSlackWeightOfJerk() const {
    return jerk_lower_data_.slack_weight_map;
  }

  const std::map<int, double>& GetUpperSlackWeightOfJerk() const {
    return jerk_upper_data_.slack_weight_map;
  }

  absl::Span<const HardConstraint> hard_j_constraint() const {
    return hard_j_constraint_;
  }

  int GetSlackNumOfLowerS() const {
    return s_lower_data_.slack_weight_map.size();
  }

  int GetSlackNumOfUpperS() const {
    return s_upper_data_.slack_weight_map.size();
  }

  int GetSlackNumOfLowerSpeed() const {
    return speed_lower_data_.slack_weight_map.size();
  }

  int GetSlackNumOfUpperSpeed() const {
    return speed_upper_data_.slack_weight_map.size();
  }

  int GetSlackNumOfLowerAccel() const {
    return accel_lower_data_.slack_weight_map.size();
  }

  int GetSlackNumOfUpperAccel() const {
    return accel_upper_data_.slack_weight_map.size();
  }

  int GetSlackNumOfLowerJerk() const {
    return jerk_lower_data_.slack_weight_map.size();
  }

  int GetSlackNumOfUpperJerk() const {
    return jerk_upper_data_.slack_weight_map.size();
  }

 private:
  int GetTimeRangeIndex(int knot_idx) const;

  int GetKnotNum(int time_range_idx) const;

  std::vector<double> piecewise_time_range_;
  double delta_t_ = 0.0;
  int total_knot_num_ = 0;

  int s_lower_slack_idx_ = 0;
  int s_upper_slack_idx_ = 0;
  int v_lower_slack_idx_ = 0;
  int v_upper_slack_idx_ = 0;

  SSoftConstraintData s_upper_data_;
  SSoftConstraintData s_lower_data_;
  std::vector<HardConstraint> hard_s_constraint_;

  SpeedSoftConstraintData speed_upper_data_;
  SpeedSoftConstraintData speed_lower_data_;
  std::vector<HardConstraint> hard_v_constraint_;

  AccelSoftConstraintData accel_upper_data_;
  AccelSoftConstraintData accel_lower_data_;

  JerkSoftConstraintData jerk_upper_data_;
  JerkSoftConstraintData jerk_lower_data_;

  std::vector<HardConstraint> hard_a_constraint_;
  std::vector<HardConstraint> hard_j_constraint_;
};

}  // namespace e2e_noa::planning

#endif
