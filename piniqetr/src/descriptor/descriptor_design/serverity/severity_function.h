
#pragma once

#include <algorithm>
#include <vector>

namespace e2e_noa {

namespace planning {

/**
 * Base class for all severity functions
 */
class SeverityFunction {
 public:
  enum class SeverityFunctionType {
    kConstant = 0,
    kPiecewiseLinear = 1,
  };
  explicit SeverityFunction(int dim) : dim_(dim) {}
  virtual ~SeverityFunction() = default;

  virtual SeverityFunctionType Type() const = 0;
  int dim() const { return dim_; }

  double Compute(const std::vector<double>& measurements) const {
    if (dim_ != static_cast<int>(measurements.size())) {
      return 0.0;  // Invalid input
    }
    return ComputeImpl(measurements);
  }

 protected:
  virtual double ComputeImpl(const std::vector<double>& measurements) const = 0;
  int dim_;
};

/**
 * Constant severity function - always returns the same value
 */
class ConstantSeverityFunction : public SeverityFunction {
 public:
  explicit ConstantSeverityFunction(double value)
      : SeverityFunction(0), value_(value) {}

  SeverityFunctionType Type() const override {
    return SeverityFunctionType::kConstant;
  }

 protected:
  double ComputeImpl(const std::vector<double>&) const override {
    return value_;
  }

 private:
  double value_;
};

/**
 * Piecewise linear severity function
 */
class PiecewiseLinearSeverityFunction : public SeverityFunction {
 public:
  PiecewiseLinearSeverityFunction(const std::vector<double>& breakpoints,
                                  const std::vector<double>& values)
      : SeverityFunction(1), breakpoints_(breakpoints), values_(values) {
    // Ensure breakpoints are sorted
    // In production code, add validation here
  }

  SeverityFunctionType Type() const override {
    return SeverityFunctionType::kPiecewiseLinear;
  }

 protected:
  double ComputeImpl(const std::vector<double>& measurements) const override {
    double x = measurements[0];

    // Handle edge cases
    if (x <= breakpoints_.front()) return values_.front();
    if (x >= breakpoints_.back()) return values_.back();

    // Find the interval and interpolate
    for (size_t i = 0; i < breakpoints_.size() - 1; ++i) {
      if (x >= breakpoints_[i] && x <= breakpoints_[i + 1]) {
        double alpha =
            (x - breakpoints_[i]) / (breakpoints_[i + 1] - breakpoints_[i]);
        return values_[i] * (1 - alpha) + values_[i + 1] * alpha;
      }
    }

    return 0.0;  // Should not reach here
  }

 private:
  std::vector<double> breakpoints_;
  std::vector<double> values_;
};

/**
 * Step severity function for binary constraints
 */
class StepSeverityFunction : public SeverityFunction {
 public:
  StepSeverityFunction(double threshold, double low_value, double high_value)
      : SeverityFunction(1),
        threshold_(threshold),
        low_value_(low_value),
        high_value_(high_value) {}

  SeverityFunctionType Type() const override {
    return SeverityFunctionType::kPiecewiseLinear;
  }

 protected:
  double ComputeImpl(const std::vector<double>& measurements) const override {
    return measurements[0] > threshold_ ? high_value_ : low_value_;
  }

 private:
  double threshold_;
  double low_value_;
  double high_value_;
};

}  // namespace planning
}  // namespace e2e_noa