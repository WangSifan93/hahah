#ifndef SPATIO_FORM_H_
#define SPATIO_FORM_H_

#include <vector>

#include "absl/types/span.h"
#include "math/vec.h"
#include "spacetime_search/spatio_graph/spatio_state.h"

namespace e2e_noa {
namespace planning {

class SpatioForm {
 public:
  virtual double length() const = 0;
  virtual SpatioState State(double s) const = 0;
  virtual const SpatioState& FastState(double s) const = 0;
  virtual std::vector<SpatioState> Sample(double delta_s) const = 0;
  virtual absl::Span<const SpatioState> states() const = 0;
  virtual ~SpatioForm() {}
};

class StraightLineGeometry : public SpatioForm {
 public:
  StraightLineGeometry(const Vec2d& start, const Vec2d& end);

  double length() const override;

  std::vector<SpatioState> Sample(double delta_s) const override;
  SpatioState State(double s) const override;
  const SpatioState& FastState(double s) const override;
  absl::Span<const SpatioState> states() const override {
    return absl::MakeSpan(densely_discretized_states_);
  }

 private:
  Vec2d start_;
  Vec2d end_;
  Vec2d unit_;
  std::vector<SpatioState> densely_discretized_states_;
};

class PiecewiseLinearGeometry : public SpatioForm {
 public:
  explicit PiecewiseLinearGeometry(absl::Span<const SpatioState> states);
  double length() const override { return length_; }
  std::vector<SpatioState> Sample(double delta_s) const override;
  SpatioState State(double s) const override;
  const SpatioState& FastState(double s) const override;
  absl::Span<const SpatioState> states() const override {
    return absl::MakeSpan(densely_discretized_states_);
  }

 private:
  double length_;
  std::vector<SpatioState> states_;
  std::vector<SpatioState> densely_discretized_states_;
  int dense_state_size_;
  std::vector<double> vec_s_;
};

class StationarySpatio : public SpatioForm {
 public:
  explicit StationarySpatio(const SpatioState& state) : state_({state}) {}

  double length() const override { return 0.0; }

  SpatioState State(double) const override { return state_[0]; }
  const SpatioState& FastState(double) const override { return state_[0]; }
  std::vector<SpatioState> Sample(double) const override { return state_; }
  absl::Span<const SpatioState> states() const override {
    return absl::MakeSpan(state_);
  }

 private:
  std::vector<SpatioState> state_;
};

}  
}  
#endif
