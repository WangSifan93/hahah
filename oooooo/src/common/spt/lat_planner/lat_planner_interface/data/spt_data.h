#pragma once

#include <stddef.h>

#include <array>
#include <vector>

#include "math/geometry/segment2d.h"

namespace e2e_noa {
namespace spt {

enum StateIndex {
  X = 0,
  Y = 1,
  THETA = 2,
  KAPPA = 3,
  DKAPPA = 4,
  V = 5,
  ACC = 6,
  JERK = 7,
  STATE_SIZE
};
enum ControlIndex { DDKAPPA = 0, DJERK = 1, INPUT_SIZE };

enum SptConstant {
  REF_PATH_COST_HUBER_THRESHOLD = 0,
  REF_S_COST_HUBER_THRESHOLD,
  REF_PATH_START_INDEX,
  REF_PATH_END_INDEX,
  CONSTANT_DATA_SIZE,
};

enum SptCircleData {
  REF_LAT_OFFSET = 0,
  REF_S_OFFSET,
  LAT_OFFSET,
  S_OFFSET,

  MATCHED_REF_X,
  MATCHED_REF_Y,
  MATCHED_REF_ThETA,
  MATCHED_REF_SIN_THETA,
  MATCHED_REF_COS_THETA,
  MATCHED_REF_KAPPA,
  MATCHED_REF_S,
  MATCHED_REF_V,
  MATCHED_REF_A,

  SPT_CIRCLE_DATA_SIZE,
};

enum SptStepData {
  SAMPLE_S = 0,
  DT,
  T,
  // weight
  REF_PATH_COST_WEIGHT,
  REF_V_COST_WEIGHT,
  DDKAPPA_WEIGHT,
  DJERK_WEIGHT,
  HEADING_WEIGHT,
  REF_A_WEIGHT,
  REF_S_COST_WEIGHT,
  V_LIMIT_UB_WEIGHT,
  V_LIMIT_LB_WEIGHT,
  JERK_LONG_WEIGHT,
  JERK_LONG_UB_WEIGHT,
  JERK_LONG_LB_WEIGHT,
  // contraint
  DKAPPA_BOUNDARY_UPPER,
  DKAPPA_BOUNDARY_LOWER,
  JERK_LONG_BOUNDARY_UPPER,
  JERK_LONG_BOUNDARY_LOWER,

  V_SQUARE,
  SIN_THETA,
  COS_THETA,
  LAT_ACC,
  LAT_JERK,
  LON_JERK,

  LAST_REAR_CIRCLE_MATCH_INDEX,

  TERMINAL_FLAG,

  STEP_DATA_SIZE
};

enum SptRefPath {
  REF_X = 0,
  REF_Y,
  REF_THETA,
  REF_COS_THETA,
  REF_SIN_THETA,
  REF_KAPPA,
  REF_S,
  REF_V,
  REF_A,
  SPT_REF_PATH_SIZE
};

struct CircleModelInfo {
 public:
  CircleModelInfo() = default;
  inline void SetLength(const double length) { length_ = length; }
  inline double Length() const { return length_; }
  inline void SetWidth(const double width) { width_ = width; }
  inline double Width() const { return width_; }
  inline void SetRadius(const double r) { radius_ = r; }
  inline double Radius() const { return radius_; }
  inline void SetX(const double x) { x_ = x; }
  inline double X() const { return x_; }
  inline void SetY(const double y) { y_ = y; }
  inline double Y() const { return y_; }
  inline const double &GetCircleData(const SptCircleData index) const {
    return circle_data[index];
  };
  inline void SetCircleData(const SptCircleData index, const double val) {
    circle_data[index] = val;
  }

  void Initialize() {
    length_ = 0.0;
    width_ = 0.0;
    radius_ = 0.0;
    x_ = 0.0;
    y_ = 0.0;
    std::fill(circle_data.begin(), circle_data.end(), 0.0);
  }

 private:
  double length_ = 0.0;
  double width_ = 0.0;
  double radius_ = 0.0;
  double x_ = 0.0;
  double y_ = 0.0;
  std::array<double, SptCircleData::SPT_CIRCLE_DATA_SIZE + 1> circle_data;
};

struct LatSptData {
  std::array<double, CONSTANT_DATA_SIZE + 1> spt_constant_data;
  std::vector<std::array<double, STEP_DATA_SIZE + 1>> spt_step_data;
  std::vector<std::vector<CircleModelInfo>> circle_model_info;
  std::vector<std::array<double, SPT_REF_PATH_SIZE + 1>> ref_path;
  std::vector<Segment2d> ref_path_segment;
};

class LatSptDataCache {
 public:
  LatSptDataCache() = default;

  inline const double &GetSptConstant(const SptConstant index) const {
    return spt_data_.spt_constant_data[index];
  }

  inline const double &GetSptStepData(const size_t step,
                                      const SptStepData index) const {
    return spt_data_.spt_step_data[step][index];
  }

  inline void SetSptRefPath(const size_t step, const SptRefPath index,
                            const double val) {
    if (step >= spt_data_.ref_path.size()) {
      return;
    }
    spt_data_.ref_path[step][index] = val;
  }

  inline const double &GetRefPath(const size_t step,
                                  const SptRefPath index) const {
    return spt_data_.ref_path[step][index];
  }

  inline void SetSptRefPathSegment(const size_t step, const Segment2d &seg) {
    if (step >= spt_data_.ref_path_segment.size()) {
      return;
    }
    spt_data_.ref_path_segment[step] = seg;
  }

  inline const Segment2d &GetRefPathSegment(const size_t step) const {
    return spt_data_.ref_path_segment[step];
  }

  inline const std::vector<Segment2d> &GetRefPathSegment() const {
    return spt_data_.ref_path_segment;
  }

  inline void SetSptConstant(const SptConstant index, const double val) {
    spt_data_.spt_constant_data[index] = val;
  }

  inline const void SetCircleModelInfo(const size_t step,
                                       const std::vector<CircleModelInfo> val) {
    if (step > spt_data_.circle_model_info.size()) {
      return;
    }
    spt_data_.circle_model_info[step] = val;
  }

  inline const std::vector<CircleModelInfo> &GetCircleModelInfo(
      const size_t step) const {
    return spt_data_.circle_model_info[step];
  }
  inline std::vector<CircleModelInfo> *GetMutableCircleModelInfo(
      const size_t step) {
    return &spt_data_.circle_model_info[step];
  }
  inline const CircleModelInfo &GetRearCircleModelInfo(
      const size_t step) const {
    return spt_data_.circle_model_info[step].at(0);
  }
  inline const CircleModelInfo &GetFrontCircleModelInfo(
      const size_t step) const {
    return spt_data_.circle_model_info[step].at(2);
  }

  inline void SetSptStepData(const size_t step, const SptStepData index,
                             const double val) {
    if (step >= spt_data_.spt_step_data.size()) {
      return;
    }
    spt_data_.spt_step_data[step][index] = val;
  }

  inline int GetStepHorizon() const {
    return static_cast<int>(spt_data_.spt_step_data.size());
  }

  inline int GetRefPathHorizon() const {
    return static_cast<int>(spt_data_.ref_path.size());
  }
  inline void ResizeStepData(const size_t horizon) {
    spt_data_.spt_step_data.resize(horizon);
    spt_data_.circle_model_info.resize(horizon);
  }

  inline void ResizeRefpath(const size_t horizon) {
    spt_data_.ref_path.resize(horizon);
    spt_data_.ref_path_segment.resize(horizon - 1);
  }

  void Reset() {
    std::fill(spt_data_.spt_constant_data.begin(),
              spt_data_.spt_constant_data.end(), 0.0);
    for (auto &temp : spt_data_.spt_step_data) {
      std::fill(temp.begin(), temp.end(), 0.0);
    }
    for (auto &temp : spt_data_.spt_step_data) {
      std::fill(temp.begin(), temp.end(), 0.0);
    }
  }

 private:
  LatSptData spt_data_;
};

}  // namespace spt
}  // namespace e2e_noa
