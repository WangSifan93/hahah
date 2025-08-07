
#ifndef ST_PLANNING_UTIL_STATUS_MACROS
#define ST_PLANNING_UTIL_STATUS_MACROS

#include <utility>

#include "absl/base/optimization.h"
#include "absl/status/status.h"
#include "util/status_builder.h"

namespace e2e_noa::status_macro_internal {
class StatusAdaptorForMacros;
}

#define RETURN_IF_ERROR(expr)                                             \
  STATUS_MACROS_IMPL_ELSE_BLOCKER_                                        \
  if (e2e_noa::status_macro_internal::StatusAdaptorForMacros              \
          status_macro_internal_adaptor = {(expr), __FILE__, __LINE__}) { \
  } else                                                                  \
    return status_macro_internal_adaptor.Consume()

#define ASSIGN_OR_RETURN(...)                                                \
  STATUS_MACROS_IMPL_GET_VARIADIC_((__VA_ARGS__,                             \
                                    STATUS_MACROS_IMPL_ASSIGN_OR_RETURN_3_,  \
                                    STATUS_MACROS_IMPL_ASSIGN_OR_RETURN_2_)) \
  (__VA_ARGS__)

#define STATUS_MACROS_IMPL_GET_VARIADIC_HELPER_(_1, _2, _3, NAME, ...) NAME
#define STATUS_MACROS_IMPL_GET_VARIADIC_(args) \
  STATUS_MACROS_IMPL_GET_VARIADIC_HELPER_ args

#define STATUS_MACROS_IMPL_ASSIGN_OR_RETURN_2_(lhs, rexpr) \
  STATUS_MACROS_IMPL_ASSIGN_OR_RETURN_3_(lhs, rexpr, _)
#define STATUS_MACROS_IMPL_ASSIGN_OR_RETURN_3_(lhs, rexpr, error_expression) \
  STATUS_MACROS_IMPL_ASSIGN_OR_RETURN_(                                      \
      STATUS_MACROS_IMPL_CONCAT_(_status_or_value, __LINE__), lhs, rexpr,    \
      error_expression)
#define STATUS_MACROS_IMPL_ASSIGN_OR_RETURN_(statusor, lhs, rexpr, \
                                             error_expression)     \
  auto statusor = (rexpr);                                         \
  if (ABSL_PREDICT_FALSE(!statusor.ok())) {                        \
    StatusBuilder _(std::move(statusor).status());                 \
    (void)_;                                                       \
    return (error_expression);                                     \
  }                                                                \
  lhs = std::move(statusor).value()

#define ASSIGN_OR_VOID_RETURN(...) \
  STATUS_MACROS_IMPL_ASSIGN_OR_VOID_RETURN_2_(__VA_ARGS__)

#define STATUS_MACROS_IMPL_ASSIGN_OR_VOID_RETURN_2_(lhs, rexpr) \
  STATUS_MACROS_IMPL_ASSIGN_OR_VOID_RETURN_(                    \
      STATUS_MACROS_IMPL_CONCAT_(_status_or_value, __LINE__), lhs, rexpr)
#define STATUS_MACROS_IMPL_ASSIGN_OR_VOID_RETURN_(statusor, lhs, rexpr) \
  auto statusor = (rexpr);                                              \
  if (ABSL_PREDICT_FALSE(!statusor.ok())) {                             \
    return;                                                             \
  }                                                                     \
  lhs = std::move(statusor).value()

#define ASSIGN_OR_DIE(...)                                                \
  STATUS_MACROS_IMPL_GET_VARIADIC_((__VA_ARGS__,                          \
                                    STATUS_MACROS_IMPL_ASSIGN_OR_DIE_3_,  \
                                    STATUS_MACROS_IMPL_ASSIGN_OR_DIE_2_)) \
  (__VA_ARGS__)

#define STATUS_MACROS_IMPL_ASSIGN_OR_DIE_2_(lhs, rexpr) \
  STATUS_MACROS_IMPL_ASSIGN_OR_DIE_3_(lhs, rexpr, std::move(_))
#define STATUS_MACROS_IMPL_ASSIGN_OR_DIE_3_(lhs, rexpr, error_expression) \
  STATUS_MACROS_IMPL_ASSIGN_OR_DIE_(                                      \
      STATUS_MACROS_IMPL_CONCAT_(_status_or_value, __LINE__), lhs, rexpr, \
      error_expression)
#define STATUS_MACROS_IMPL_ASSIGN_OR_DIE_(statusor, lhs, rexpr,    \
                                          error_expression)        \
                                                                   \
  auto statusor = (rexpr);                                         \
  if (ABSL_PREDICT_FALSE(!statusor.ok())) {                        \
    StatusBuilder _(statusor.status());                            \
    (void)_;                                                       \
    CHECK(false) << (_ << error_expression).JoinMessageToStatus(); \
  }                                                                \
  lhs = std::move(statusor).value();

#define ASSIGN_OR_CONTINUE(...) \
  STATUS_MACROS_IMPL_ASSIGN_OR_CONTINUE_2_(__VA_ARGS__)
#define STATUS_MACROS_IMPL_ASSIGN_OR_CONTINUE_2_(lhs, rexpr) \
  STATUS_MACROS_IMPL_ASSIGN_OR_CONTINUE_(                    \
      STATUS_MACROS_IMPL_CONCAT_(_status_or_value, __LINE__), lhs, rexpr)
#define STATUS_MACROS_IMPL_ASSIGN_OR_CONTINUE_(statusor, lhs, rexpr) \
  auto statusor = (rexpr);                                           \
  if (ABSL_PREDICT_FALSE(!statusor.ok())) {                          \
    continue;                                                        \
  }                                                                  \
  lhs = std::move(statusor).value()

#define ASSIGN_OR_BREAK(...) STATUS_MACROS_IMPL_ASSIGN_OR_BREAK_2_(__VA_ARGS__)
#define STATUS_MACROS_IMPL_ASSIGN_OR_BREAK_2_(lhs, rexpr) \
  STATUS_MACROS_IMPL_ASSIGN_OR_BREAK_(                    \
      STATUS_MACROS_IMPL_CONCAT_(_status_or_value, __LINE__), lhs, rexpr)
#define STATUS_MACROS_IMPL_ASSIGN_OR_BREAK_(statusor, lhs, rexpr) \
  auto statusor = (rexpr);                                        \
  if (ABSL_PREDICT_FALSE(!statusor.ok())) {                       \
    break;                                                        \
  }                                                               \
  lhs = std::move(statusor).value()

#define STATUS_MACROS_IMPL_CONCAT_INNER_(x, y) x##y
#define STATUS_MACROS_IMPL_CONCAT_(x, y) STATUS_MACROS_IMPL_CONCAT_INNER_(x, y)

#define STATUS_MACROS_IMPL_ELSE_BLOCKER_ \
  switch (0)                             \
  case 0:                                \
  default:

namespace e2e_noa {
namespace status_macro_internal {

class StatusAdaptorForMacros {
 public:
  StatusAdaptorForMacros(const absl::Status& status, const char*, int)
      : builder_(status) {}

  StatusAdaptorForMacros(absl::Status&& status, const char*, int)
      : builder_(status) {}

  StatusAdaptorForMacros(const StatusBuilder& builder, const char*, int)
      : builder_(builder) {}

  StatusAdaptorForMacros(StatusBuilder&& builder, const char*, int)
      : builder_(builder) {}

  StatusAdaptorForMacros(const StatusAdaptorForMacros&) = delete;
  StatusAdaptorForMacros& operator=(const StatusAdaptorForMacros&) = delete;

  explicit operator bool() const { return ABSL_PREDICT_TRUE(builder_.ok()); }

  StatusBuilder&& Consume() { return std::move(builder_); }

 private:
  StatusBuilder builder_;
};

}  // namespace status_macro_internal
}  // namespace e2e_noa

#endif
