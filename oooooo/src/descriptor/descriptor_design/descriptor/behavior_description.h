/*
   In the ML Planner architecture, Planner Abstraction is a component that
   abstracts the  environment and “desired” driving behavior into
   representations that the rest of the planner  can easily consume (data
   structures and mathematical representations). The purpose of this  component
   is to: Facilitate downstream by providing a good abstraction and a unified
   interface. This can be  used, e.g. to create costs in
   search/optimization/selection. Make the system explainable, can be used to
   validate/augment ML generated plans (and  even training data filtering and
   validation), and provides safety guarantee.

   Incorporate road rules and laws so our ADC’s behavior is compliant.

   Provide an interface (or part of an interface) that allows direct injection
   of “hand-crafted”  features/plans. (The main purpose here is so we can
   quickly provide targeted fixes for  urgent issues. The nature of ML models
   make it hard to create targeted fixes and quick  deployments.)

   Modules
   Unlike PlanNet and SPT Optimizer that implement a “single” algorithm, Planner
   Abstraction is  a collection of smaller modules that deals with different
   aspects of the planning problem.  Each sub-module within Planner Abstraction
   is relatively independent and can have its own  design. Here is a
   (incomplete) list of sub modules grouped by their high-level objectives and
   some high level ideas how they may be represented.

   Traffic Rules
   These modules reason about traffic rules and laws. They generate constraints
   that help keep  our plan compliant. The rules and laws typically have clear
   logic and are relatively straight  forward to formulate. Stop Sign The
   desired behavior at the stop line is to come to a complete stop within some
   desired  distance to the stop sign, then proceed again (after yielding
   appropriately to other traffic).

   This is a simple module that abstracts this rule. Please refer to the later
   section “Behavior  Descriptor” to see how this may be implemented. There are
   some complications for special stop signs such as the ones on school buses
   where  the criteria for proceeding is to wait for the stop sign to be lifted.
   Traffic Light
   Traffic light logic is a bit more complex compared to the stop sign because
   of the changing  state, but still relatively clear. In general:

   Green light: OK to proceed - essentially adding no constraints to downstream.
   One complication is for unprotected turns, where the desired behavior is to
   yield to  high-priority traffic before proceeding. But the logic should
   probably be handled by the  “precedence” module. Yellow light: on a high
   level the guideline is to stop if this can be done safely (and  comfortably),
   otherwise proceed through. Abstracting this for downstream is a bit tricky.
   This module should estimate the remaining time of the yellow light, and
   quantitatively  define the criteria for “stopping safely and comfortably”.

   Red light: do not proceed. This can be represented as some form of stop line
   constraint for  downstream.  Some complications could arise for right turns
   that are allowed on a red light (right on  red). In this case the red light
   can be treated the same as a stop sign.

   Special lights Flashing yellow: this means proceed with caution and prepare
   to yield. The effect  should be the same as a yield sign. Flashing red: this
   is equivalent to a stop sign. Broken light: also equivalent to a stop sign.

   Note that many details are omitted in the discussion above, as the purpose
   here is to provide  a high level description of the module.

   Lane Maneuvers
   This module group reasons about various lane maneuvers such as lane change,
   merge, nudge  (side pass), etc and generate constraints to guide these
   behaviors.

   Boundary Reasoning TODO: think about how this relates to decisions - we may
   need to generate different  boundary constraints based on the decision.
   Perhaps the most important module in this group is boundary reasoning.

   Road User Interactions
    TODO: this is also affected by the decision module.
   Precedence
   This module reasons about road user precedence based on traffic laws and
   guidance. For  example, in unprotected turns, agents going straight have
   higher precedence over agents  turning. Precedence may affect pass-yield
   decisions and comfort/risk cost computation in  downstream.

   Safety and Comfort Buffers
   In general maintaining proper distance from other agents are desired for both
   safety and  comfort. This module provides guideline/constrains for downstream
   to maintain proper  buffers from agents. Safety buffer should take into
   consideration the perception/prediction and control  uncertainty, so that ADC
   is guaranteed to be collision free if it can maintain the safety buffer.
   Comfort buffer accounts for courtesy and social rules. These buffers may be
   different for  different types of objects. For example, a larger buffer is
   usually desired around moving  cyclists. The buffer may be defined in
   longitudinal and lateral directions separately. For example,  maintaining a
   proper following distance is a type of longitudinal buffer. It may also be
   necessary to define time-based buffers, which can be useful in agent-crossing
   scenarios.

   Behavior Descriptor Behavior descriptor is a mechanism to quantitatively
   define the “goodness” or “badness” of a  given maneuver or plan than can be
   used to score trajectories in optimization (and search/selection). A
   descriptor should contain the following elements: Context: specific contexts
   that this descriptor becomes relevant Signal: abstracted data structures that
   represent real world information Measurement: quantitative values that can be
   computed from data and Severity function: how measurements translate to
   severity/score Next we provide detailed explanations and examples for each
   element. Context Context refers to situations in which an abstraction becomes
   relevant. It is also possible that a  descriptor specifies different desired
   outcomes under different contexts. Here are two  examples. Traffic light
   descriptor: the context for this abstraction is the traffic light state. If
   the traffic  light is green, the desired behavior would be to proceed
   through; If the traffic light is red, the  desired behavior is to stop. Stop
   sign descriptor: the context for this abstraction is whether ADC has already
   stopped. If  so, the desired behavior is to proceed; Otherwise the desired
   behavior is to stop (close  enough) at the stop sign. Signal Signal provides
   necessary information to be used to quantify the desired behavior. Its
   representation should be easy to process by downstream components. Here are
   some  examples. Traffic light signal: location of the traffic light and its
   lane association. Speed region signal: polygon (or lane/road segments) that
   covers the speed region, and the  speed limit of the speed region. If we
   choose representations appropriately, a set of “basic” signals should be able
   to provide  enough information for a vast set of desired behaviors.

   Measurement Measurements are quantitative values that a severity function can
   use to compute a “cost”.  For example: Distance to the traffic light. ADC’s
   speed. Whether ADC is inside a polygon (e.g. speed region), or penetration
   distance into the  polygon. Measurements are typically computed against
   signals. Similar to signals, appropriate  representations should allow us to
   use a set of “basic” measurements for many types of  descriptors. Note that,
   although behavior descriptor defines what measurements to use, the actual
   values  of these measurements have to be computed within downstream
   components. More  discussions will be provided in the “API - Downstream
   Integration” section. Severity Function Severity function is the core to
   behavior descriptor - it defines the “goodness/badness” of a  particular
   aspect of a given behavior (part of a plan/trajectory). Severity usually
   depends on  contexts and measurements. Here are some examples.

   Traffic Light
   When context (light color) is green, severity is always 0 When context is
   red, severity is 0 when distance to traffic light is > 0, and 1 when the
   distance is < 0. When context is yellow, the severity goes up as the time
   passing through the traffic light  approaches the estimated time for the
   light to turn red.

   Speed Region:  Severity goes up as ADC’s speed goes over the speed limit and
   ADC is within  the speed region polygon. The idea of severity is to properly
   trade-off between “constraints” when necessary. For  example, running a red
   light should have higher severity than going over the speed limit (slightly).
   Therefore severity tends to have discrete “levels”. However, for smoothness
   and  stability of the planning pipeline, we should avoid having big abrupt
   changes in severity  function values. Here are some design ideas to help
   improve “smoothness”:

   Fix the range of severity to [0, 1] Set multiple levels instead of just a
   binary switch Consider linearly increasing severity between two levels
   (overall a piece-wise linear  function)

   Downstream Integration
    There are two main steps for downstream integration of behavior descriptors
   Implementing measurements
   Converting severity to actual costs
   Both steps depend on the particular component that needs integrations. For
   example, for the  SPT optimizer, measurement is essentially a method that
   maps the state or control to a real  value, and costs typically should be
   differentiable (and ideally convex). The above two code snippets present a
   rough design for the measurement and cost based on  behavior descriptors for
   the SPT optimizer. Each OptimizerMeasurementUnit should be  created from a
   corresponding Signal . A cost may contain several measurements and the number
   must match the dimension of the severity function. Note that code snippets in
   this section should not be used as the exact design of these  classes. They
   are just to illustrate the idea of the API.

   Phase1:
   Objective: In the initial phase the objective is to implement planner
   abstraction for a few  selected modules (e.g. traffic rules) and integrate
   with only the selection component. The  main reason is that selection should
   be the easiest component to integrate with. SPT  optimizer has the
   complication that it needs to compute Jacobians and Hessians of the cost.

   Phase2:
   Objectives:
   Create more descriptors and reasoning modules, e.g. Pullover module Road user
   interactions.

 */

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "descriptor/descriptor_design/measurement/measurement_units_base.h"
#include "descriptor/descriptor_design/serverity/severity_function.h"
#include "descriptor/descriptor_design/signal/signal_base.h"

namespace e2e_noa {
namespace planning {

// Forward declarations
class Signal;
class SeverityFunction;

/**
 * Base class for all behavior descriptors
 * Contains context, signal, and severity function
 *
 */
class BehaviorDescriptor {
 public:
  explicit BehaviorDescriptor(
      const std::string& name,
      absl::flat_hash_map<SignalType, std::vector<std::unique_ptr<Signal>>>
          signals,
      std::unique_ptr<SeverityFunction> severity_func)
      : name_(name),
        signals_(std::move(signals)),
        severity_func_(std::move(severity_func)) {}

  virtual ~BehaviorDescriptor() = default;

  const std::string& Name() const { return name_; }

  // 获取指定类型的所有信号
  const std::vector<std::unique_ptr<Signal>>* GetSignals(
      const SignalType& type) const {
    auto it = signals_.find(type);
    return (it != signals_.end()) ? &(it->second) : nullptr;
  }

  // 获取指定类型的第一个信号（向后兼容）
  const Signal* GetSignal(const SignalType& type) const {
    auto signals = GetSignals(type);
    return (signals && !signals->empty()) ? signals->front().get() : nullptr;
  }

  // 获取指定类型指定索引的信号
  const Signal* GetSignal(const SignalType& type, size_t index) const {
    auto signals = GetSignals(type);
    return (signals && index < signals->size()) ? (*signals)[index].get()
                                                : nullptr;
  }

  // 添加信号
  void AddSignal(SignalType type, std::unique_ptr<Signal> signal) {
    signals_[type].push_back(std::move(signal));
  }

  // 获取指定类型信号的数量
  size_t GetSignalCount(const SignalType& type) const {
    auto signals = GetSignals(type);
    return signals ? signals->size() : 0;
  }

  const SeverityFunction& severity_func() const { return *severity_func_; }

  // Virtual methods for context and computation
  virtual bool IsRelevant() const = 0;
  virtual std::vector<double> ComputeMeasurements() const = 0;
  virtual double ComputeSeverity() const;

 protected:
  virtual void UpdateContext() {}
  virtual void CreateSignal() {}
  virtual void CreateSeverityFunc() {}

 private:
  std::string name_;
  absl::flat_hash_map<SignalType, std::vector<std::unique_ptr<Signal>>>
      signals_;
  std::unique_ptr<SeverityFunction> severity_func_;
};

/**
 * Base class for all measurement units used in optimization
 */
class OptimizerMeasurementUnit {
 public:
  virtual ~OptimizerMeasurementUnit() = default;
  virtual MeasurementUnitType Type() const = 0;
  virtual double operator()(const spt_opt::state& state,
                            const spt_opt::control& control) const = 0;
};

/**
 * Cost function for behavior descriptors in optimization
 */
class OptimizerBehaviorDescriptorCost {
 public:
  OptimizerBehaviorDescriptorCost(
      std::vector<std::unique_ptr<OptimizerMeasurementUnit>> measurement_units,
      std::unique_ptr<SeverityFunction> severity_func);

  double ComputeCost(const spt_opt::state& state,
                     const spt_opt::control& control) const;

 private:
  std::vector<std::unique_ptr<OptimizerMeasurementUnit>> measurement_units_;
  std::unique_ptr<SeverityFunction> severity_func_;
};

}  // namespace planning
}  // namespace e2e_noa