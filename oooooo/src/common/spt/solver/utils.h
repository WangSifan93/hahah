#pragma once

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <vector>

#ifndef UNUSED
#define UNUSED(x) (void)(x)
#endif

static constexpr int kMaxBackwardPassCount = 10;
static constexpr double kLambdaMax = 1000000000.0;
static constexpr double kLambdaFactor = 10.0;
static constexpr double kLambdaMin = 1e-5;
static constexpr double kZMin = 1e-3;
static std::vector<double> alpha_vec = {1.0000, 0.6180, 0.3819, 0.2360, 0.1458,
                                        0.0901, 0.0557, 0.0344, 0.01};

namespace e2e_noa {

namespace spt {
struct SolverConfig {
  bool is_debug_mode;

  int horizon;
  int state_size;
  int input_size;
  double cost_tol;
  double cost_percent_tol;
  double max_iter;
  bool variable_cost_percent_tol;

  int max_outer_iterations;
  double penalty_factor;
  double init_rho;
};
#define CONSTRAIN_TYPES(M, N, C)               \
  typedef Eigen::Matrix<double, M, 1> State;   \
  typedef Eigen::Matrix<double, N, 1> Control; \
  typedef Eigen::Matrix<double, C, 1> ConFunc; \
  typedef Eigen::Matrix<double, C, M + N> ConJac;

#define SOLVER_TYPES(M, N)                                                    \
  typedef Eigen::Matrix<double, M, 1> State;                                  \
  typedef Eigen::Matrix<double, N, 1> Control;                                \
  typedef Eigen::Matrix<double, M, 1> Lx;                                     \
  typedef Eigen::Matrix<double, N, 1> Lu;                                     \
  typedef Eigen::Matrix<double, N, 1> CoeffD;                                 \
  typedef Eigen::Matrix<double, M, M> Lxx;                                    \
  typedef Eigen::Matrix<double, N, N> Luu;                                    \
  typedef Eigen::Matrix<double, M, N> Lxu;                                    \
  typedef Eigen::Matrix<double, N, M> Lux;                                    \
  typedef Eigen::Matrix<double, M, M> Fx;                                     \
  typedef Eigen::Matrix<double, M, N> Fu;                                     \
  typedef Eigen::Matrix<double, N, M> FuT;                                    \
  typedef Eigen::Matrix<double, N, M> CoeffK;                                 \
  typedef Eigen::Matrix<double, M, N> CoeffKT;                                \
  typedef std::vector<State, Eigen::aligned_allocator<State>> StateVec;       \
  typedef std::vector<Control, Eigen::aligned_allocator<Control>> ControlVec; \
  typedef std::vector<Lx, Eigen::aligned_allocator<Lx>> LxVec;                \
  typedef std::vector<Lu, Eigen::aligned_allocator<Lu>> LuVec;                \
  typedef std::vector<CoeffD, Eigen::aligned_allocator<CoeffD>> CoeffDVec;    \
  typedef std::vector<Lxx, Eigen::aligned_allocator<Lxx>> LxxVec;             \
  typedef std::vector<Luu, Eigen::aligned_allocator<Luu>> LuuVec;             \
  typedef std::vector<Lxu, Eigen::aligned_allocator<Lxu>> LxuVec;             \
  typedef std::vector<Lux, Eigen::aligned_allocator<Lux>> LuxVec;             \
  typedef std::vector<Fx, Eigen::aligned_allocator<Fx>> FxVec;                \
  typedef std::vector<Fu, Eigen::aligned_allocator<Fu>> FuVec;                \
  typedef std::vector<CoeffK, Eigen::aligned_allocator<CoeffK>> CoeffKVec;

template <class T>
static inline void ResizeAndResetEigen(T &x, size_t n) {
  x.resize(n);
  x.setZero();
}

template <class T>
static inline void ResizeAndResetEigen(T &x, size_t m, size_t n) {
  x.resize(m, n);
  x.setZero();
}

template <class T>
static inline void ResizeEigenVec(std::vector<T> &x_vec, size_t n) {
  for (size_t i = 0; i < x_vec.size(); ++i) {
    x_vec[i].resize(n);
  }
}

template <class T>
static inline void ResetEigenVec(std::vector<T> &x_vec) {
  for (size_t i = 0; i < x_vec.size(); ++i) {
    x_vec[i].setZero();
  }
}

template <class T>
static inline void ResizeVecAndResetEigen(std::vector<T> &x_vec, size_t N) {
  x_vec.resize(N);
  ResetEigenVec(x_vec);
}

template <class T>
static inline void ResizeAndResetEigenVec(std::vector<T> &x_vec, size_t N,
                                          size_t n) {
  x_vec.resize(N);
  ResizeEigenVec(x_vec, n);
  ResetEigenVec(x_vec);
}

template <class T>
void ResizeEigenVec(std::vector<T, Eigen::aligned_allocator<T>> &x_vec,
                    size_t n) {
  for (auto &vec : x_vec) {
    vec.resize(n);
  }
}
template <class T>
void ResetEigenVec(std::vector<T, Eigen::aligned_allocator<T>> &x_vec) {
  for (auto &vec : x_vec) {
    vec.setZero();
  }
}

template <class T>
static inline void ResizeAndResetEigenVec(
    std::vector<T, Eigen::aligned_allocator<T>> &x_vec, size_t N, size_t n) {
  x_vec.resize(N);
  ResizeEigenVec(x_vec, n);
  ResetEigenVec(x_vec);
}

template <class T>
static inline void ResizeEigenMat(std::vector<T> &x_vec, size_t m, size_t n) {
  for (size_t i = 0; i < x_vec.size(); ++i) {
    x_vec[i].resize(m, n);
  }
}

template <class T>
static inline void ResizeAndResetEigenMat(std::vector<T> &x_vec, size_t N,
                                          size_t m, size_t n) {
  x_vec.resize(N);
  ResizeEigenMat(x_vec, m, n);
  ResetEigenVec(x_vec);
}

template <class T>
void ResizeEigenMat(std::vector<T, Eigen::aligned_allocator<T>> &x_vec,
                    size_t m, size_t n) {
  for (auto &mat : x_vec) {
    mat.resize(m, n);
  }
}

template <class T>
static inline void ResizeAndResetEigenMat(
    std::vector<T, Eigen::aligned_allocator<T>> &x_vec, size_t N, size_t m,
    size_t n) {
  x_vec.resize(N);
  ResizeEigenMat(x_vec, m, n);
  ResetEigenVec(x_vec);
}

template <class T>
static inline void ResizeVecAndResetEigen(
    std::vector<T, Eigen::aligned_allocator<T>> &x_vec, size_t N) {
  x_vec.resize(N);
  ResetEigenVec(x_vec);
}

/*
@brief: Implement the logistic function y = y0 + l / (1 + exp(-k * (x - x0)))
@param: x: input value
@param: l: curve's y-range as multiplier of 1.0
@param: k: curve's steepness
@param: x0: x value of the sigmoid's midpoint
@param: y0: y value of the sigmoid's initial point
@return: the value of the logistic function
*/
static inline double Logistic(const double x, const double l = 1.0,
                              const double k = 1.0, const double x0 = 0.0,
                              const double y0 = 0.0) {
  constexpr double kOne{1.0};
  return y0 + kOne * l / (kOne + exp(-k * (x - x0)));
}

}  // namespace spt
}  // namespace e2e_noa
