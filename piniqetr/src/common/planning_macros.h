#ifndef AD_E2E_PLANNING_COMMON_PLANNING_MACROS_H
#define AD_E2E_PLANNING_COMMON_PLANNING_MACROS_H

#include <absl/cleanup/cleanup.h>
#include <glog/logging.h>

#include <cstddef>
#include <memory>
#include <sstream>

#include "common/log.h"
#include "common/log_data.h"
#include "common/timer.h"

#define DECLARE_CLASS_SINGLETON(classname) \
 public:                                   \
  static classname* instance() {           \
    static classname instance;             \
    return &instance;                      \
  }                                        \
                                           \
 private:                                  \
  classname()

#define QUIT_IF(CONDITION, RET, LEVEL, MSG, ...) \
  do {                                           \
    if (CONDITION) {                             \
      L##LEVEL(MSG, ##__VA_ARGS__);              \
      return RET;                                \
    }                                            \
  } while (0)

#define QUIT_IF_VOID_QUIET(CONDITION) \
  do {                                \
    if (CONDITION) {                  \
      return;                         \
    }                                 \
  } while (0)

#define QUIT_IF_VOID(CONDITION, LEVEL, MSG, ...) \
  do {                                           \
    if (CONDITION) {                             \
      L##LEVEL(MSG, ##__VA_ARGS__);              \
      return;                                    \
    }                                            \
  } while (0)

#define PRED_CALL_MACRO_(_, N, MACRO) MACRO(N)
#define PRED_DIRECT_RUN_(_, N, CODE) CODE

#define PLAN_REPEAT(N, MACRO) BOOST_PP_REPEAT(N, PRED_CALL_MACRO_, MACRO)

#define PLAN_SIMPLE_REPEAT(N, CODE) BOOST_PP_REPEAT(N, PRED_DIRECT_RUN_, CODE)

#define PLAN_UNUSED(variable) (void)(variable)

#define DECLARE_PTR(CLASS)                   \
  using Ptr = std::shared_ptr<CLASS>;        \
  using UPtr = std::unique_ptr<CLASS>;       \
  using CPtr = std::shared_ptr<const CLASS>; \
  using CUPtr = std::unique_ptr<const CLASS>

#define VAR_CONCAT_INNER(VAR1, VAR2) VAR1##VAR2

#define VAR_CONCAT(VAR1, VAR2) VAR_CONCAT_INNER(VAR1, VAR2)

#define RETURN_IF_NOT_OK(lhs, status_or_exp) \
  RETURN_IF_NOT_OK_3_(lhs, VAR_CONCAT(statusor, __LINE__), status_or_exp)

#define RETURN_IF_NOT_OK_3_(lhs, statusor_var, statusor_exp) \
  auto statusor_var = (statusor_exp);                        \
  if (!statusor_var.ok()) {                                  \
    return statusor_var.status();                            \
  }                                                          \
  lhs = std::move(statusor_var).value();

#define SCOPED_TRACE(name)                                             \
  const auto& _function_name = name;                                   \
  DLOG(INFO) << _function_name;                                        \
  e2e_noa::Timer _scope_timer(_function_name);                         \
  const absl::Cleanup timer_logdata = [&_scope_timer]() {              \
    e2e_noa::planning::Log2FG::LogDataV0(_function_name,               \
                                         _scope_timer.TimeUs() / 1e3); \
  }

#endif
