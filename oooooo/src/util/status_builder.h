
#ifndef ONBOARD_UTILS_STATUS_BUILDER_H_
#define ONBOARD_UTILS_STATUS_BUILDER_H_

#include <memory>
#include <sstream>
#include <utility>

#include "absl/base/attributes.h"
#include "absl/status/status.h"
#include "absl/strings/string_view.h"

namespace e2e_noa {

class ABSL_MUST_USE_RESULT StatusBuilder {
 public:
  StatusBuilder(const StatusBuilder& sb);
  StatusBuilder& operator=(const StatusBuilder& sb);

  StatusBuilder(const absl::Status& original_status)
      : status_(original_status), stream_(new std::ostringstream) {}

  StatusBuilder(absl::Status&& original_status)
      : status_(std::move(original_status)), stream_(new std::ostringstream) {}

  StatusBuilder(absl::StatusCode code)
      : status_(code, ""), stream_(new std::ostringstream) {}

  bool ok() const { return status_.ok(); }

  template <typename T>
  T YieldsResult(T&& val) const {
    return std::forward<T>(val);
  }

  void Void() const {}

  StatusBuilder& SetAppend();

  StatusBuilder& SetPrepend();

  StatusBuilder& SetNoLogging();

  StatusBuilder& operator<<(const StatusBuilder& other) { return *this; }

  template <typename T>
  StatusBuilder& operator<<(const T& msg) {
    if (status_.ok()) return *this;
    *stream_ << msg;
    return *this;
  }

  operator absl::Status() const&;
  operator absl::Status() &&;

  absl::Status JoinMessageToStatus() const;

 private:
  enum class MessageJoinStyle {
    kAnnotate,
    kAppend,
    kPrepend,
  };

  absl::Status status_;
  bool no_logging_ = false;

  std::unique_ptr<std::ostringstream> stream_;

  MessageJoinStyle join_style_ = MessageJoinStyle::kAnnotate;
};

inline StatusBuilder AlreadyExistsErrorBuilder() {
  return StatusBuilder(absl::StatusCode::kAlreadyExists);
}

inline StatusBuilder FailedPreconditionErrorConstructor() {
  return StatusBuilder(absl::StatusCode::kFailedPrecondition);
}

inline StatusBuilder InternalErrorBuilder() {
  return StatusBuilder(absl::StatusCode::kInternal);
}

inline StatusBuilder InvalidArgumentErrorBuilder() {
  return StatusBuilder(absl::StatusCode::kInvalidArgument);
}

inline StatusBuilder NotFoundErrorBuilder() {
  return StatusBuilder(absl::StatusCode::kNotFound);
}

inline StatusBuilder UnavailableErrorConstructor() {
  return StatusBuilder(absl::StatusCode::kUnavailable);
}

inline StatusBuilder UnimplementedErrorBuilder() {
  return StatusBuilder(absl::StatusCode::kUnimplemented);
}

inline StatusBuilder UnknownErrorBuilder() {
  return StatusBuilder(absl::StatusCode::kUnknown);
}

}  

#endif
