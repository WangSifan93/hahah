#ifndef SPACETIME_FORM_H_
#define SPACETIME_FORM_H_

#include <memory>
#include <utility>
#include <vector>

#include "spacetime_search/spacetime_state.h"
#include "spacetime_search/spatio_graph/spatio_form.h"
#include "spacetime_search/spatio_graph/spatio_state.h"

namespace e2e_noa::planning {

enum SpacetimeFormType {
  CONST_ACCEL_MOTION = 1,
  STATIONARY_MOTION = 2,
  COMPLETE_MOTION = 3,
};
struct SampledSpacetimeFormStates {
  std::vector<SpacetimeState> const_interval_states;
  std::vector<SpacetimeState> equal_interval_states;
};
class SpacetimeForm {
 public:
  virtual double duration() const = 0;
  virtual SpacetimeState GetStartSpacetimeState() const = 0;
  virtual SpacetimeState GetEndSpacetimeState() const = 0;
  virtual SpacetimeState State(double t) const = 0;
  virtual const SpatioForm* spatio() const = 0;
  virtual SpacetimeFormType type() const = 0;

  virtual SampledSpacetimeFormStates SampleStates() const = 0;

  virtual std::vector<SpacetimeState> SampleEqualIntervalStates() const = 0;
  virtual ~SpacetimeForm() {}

  static constexpr int kConstTimeIntervalSampleStep = 2;

  static constexpr int kMinEqualTimeIntervalSampleStep = 4;
  static constexpr int kMaxEqualTimeIntervalSampleStep = 11;
  static constexpr double kDesireEqualTimeInterval = 0.5;
};

class ConstAccelSpacetime final : public SpacetimeForm {
 public:
  explicit ConstAccelSpacetime(double init_v, double init_a,
                               const SpatioForm* spatio);

  explicit ConstAccelSpacetime(std::pair<double, double> v_pair,
                               const SpatioForm* spatio);

  double duration() const override { return duration_; }
  SpacetimeState GetStartSpacetimeState() const override;
  SpacetimeState GetEndSpacetimeState() const override;

  SpacetimeState State(double t) const override;

  const SpatioForm* spatio() const override { return spatio_; }
  SpacetimeFormType type() const override {
    return SpacetimeFormType::CONST_ACCEL_MOTION;
  }

  SampledSpacetimeFormStates SampleStates() const override;
  std::vector<SpacetimeState> SampleEqualIntervalStates() const override;

 private:
  double init_v_ = 0.0;
  double a_ = 0.0;
  double duration_ = 0.0;

  double stop_time_ = 0.0;
  double stop_distance_ = 0.0;

  const SpatioForm* spatio_ = nullptr;
};

class StationarySpacetime final : public SpacetimeForm {
 public:
  explicit StationarySpacetime(double duration, const StationarySpatio* spatio)
      : duration_(duration), spatio_(spatio) {}

  explicit StationarySpacetime(double duration, SpatioState state)
      : duration_(duration),
        spatio_form_(std::make_unique<StationarySpatio>(std::move(state))),
        spatio_(spatio_form_.get()) {}
  double duration() const override { return duration_; }
  SpacetimeState GetStartSpacetimeState() const override;
  SpacetimeState GetEndSpacetimeState() const override;
  SpacetimeState State(double t) const override;

  const SpatioForm* spatio() const override { return spatio_; }
  SpacetimeFormType type() const override {
    return SpacetimeFormType::STATIONARY_MOTION;
  }
  SampledSpacetimeFormStates SampleStates() const override;
  std::vector<SpacetimeState> SampleEqualIntervalStates() const override;

 private:
  std::vector<SpacetimeState> Sample(double d_t) const;
  double duration_ = 0.0;

  std::unique_ptr<StationarySpatio> spatio_form_;

  const SpatioForm* spatio_ = nullptr;
};

}  
#endif
