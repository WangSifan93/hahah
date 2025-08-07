/////////////////////////////////////////////////////////////////////////////
//
// Custom types.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef ILQGAMES_UTILS_TYPES_H
#define ILQGAMES_UTILS_TYPES_H

// ------------------------------- INCLUDES -------------------------------- //

#include <math.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <algorithm>
#include <chrono>
#include <functional>
#include <iostream>
#include <limits>
#include <memory>
#include <random>
#include <string>
#include <unordered_map>
#include <vector>

namespace e2e_noa::planning {

// ------------------------ THIRD PARTY TYPEDEFS ---------------------------- //

using Eigen::MatrixXf;
using Eigen::VectorXf;

// --------------------------------- TYPES ---------------------------------- //

using PlayerIndex = unsigned short;
using Dimension = int;
using Point2 = Eigen::Vector2f;

// Rename the system clock for easier usage.
using Clock = std::chrono::system_clock;

#ifdef __APPLE__
using PointList2 = std::vector<Point2, Eigen::aligned_allocator<Point2>>;
using Time = float;
#else
using PointList2 = std::vector<Point2>;
using Time = double;
#endif

template <typename T>
using PlayerPtrMap = std::unordered_map<PlayerIndex, std::shared_ptr<T>>;

template <typename T>
using PlayerPtrMultiMap =
    std::unordered_multimap<PlayerIndex, std::shared_ptr<T>>;

template <typename T>
using PlayerMap = std::unordered_map<PlayerIndex, T>;

template <typename T>
using PlayerMultiMap = std::unordered_multimap<PlayerIndex, T>;

using PlayerDualMap = std::unordered_map<PlayerIndex, float*>;

template <typename T>
using PtrVector = std::vector<std::shared_ptr<T>>;

using RefVector = std::vector<Eigen::Ref<VectorXf>>;

// Empty struct for setting unused/unimplemented template args.
struct Empty {};

// ------------------------------- CONSTANTS -------------------------------- //

namespace constants {

// Acceleration due to gravity (m/s/s).
static constexpr float kGravity = 9.81;

// Small number for use in approximate equality checking.
static constexpr float kSmallNumber = 1e-4;

// Float precision infinity.
static constexpr float kInfinity = std::numeric_limits<float>::infinity();

// Constant for invalid values.
static constexpr float kInvalidValue = std::numeric_limits<float>::quiet_NaN();

// Default multiplier values.
static constexpr float kDefaultLambda = 0.0;
static constexpr float kDefaultMu = 10.0;

}  // namespace constants

namespace time {

inline Time& getTimeStep() {
  static Time kTimeStep = 0.1;
  return kTimeStep;
}

inline Time& getTimeHorizon() {
  static Time kTimeHorizon = 10.0;
  return kTimeHorizon;
}

inline size_t getNumTimeSteps() {
  static size_t kNumTimeSteps = static_cast<size_t>(
      (getTimeHorizon() + constants::kSmallNumber) / getTimeStep());
  return kNumTimeSteps;
}

inline void setTimeParameter(const Time& time_step, const Time& time_horizon) {
  getTimeStep() = time_step;
  getTimeHorizon() = time_horizon;
  // NumSteps会自动更新
}
}  // namespace time

// ---------------------------- SIMPLE FUNCTIONS ---------------------------- //

template <typename T, typename... Args>
std::unique_ptr<T> make_unique(Args&&... args) {
  return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
}

template <typename T>
inline constexpr T sgn(T x, std::false_type is_signed) {
  return T(0) < x;
}

template <typename T>
inline constexpr T sgn(T x, std::true_type is_signed) {
  return (T(0) < x) - (x < T(0));
}

template <typename T>
inline constexpr T sgn(T x) {
  return sgn(x, std::is_signed<T>());
}

template <typename T>
inline constexpr T signed_sqrt(T x) {
  return sgn(x) * std::sqrt(std::abs(x));
}

}  // namespace e2e_noa::planning

#endif
