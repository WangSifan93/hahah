
#include "util/status_builder.h"

#include <string>

#include "absl/memory/memory.h"
#include "absl/strings/str_cat.h"

namespace e2e_noa {

StatusBuilder::StatusBuilder(const StatusBuilder& sb) {
  status_ = sb.status_;
  no_logging_ = sb.no_logging_;
  stream_ = absl::make_unique<std::ostringstream>(sb.stream_->str());
  join_style_ = sb.join_style_;
}

StatusBuilder& StatusBuilder::operator=(const StatusBuilder& sb) {
  status_ = sb.status_;
  no_logging_ = sb.no_logging_;
  stream_ = absl::make_unique<std::ostringstream>(sb.stream_->str());
  join_style_ = sb.join_style_;
  return *this;
}

StatusBuilder& StatusBuilder::SetAppend() {
  if (status_.ok()) return *this;
  join_style_ = MessageJoinStyle::kAppend;
  return *this;
}

StatusBuilder& StatusBuilder::SetPrepend() {
  if (status_.ok()) return *this;
  join_style_ = MessageJoinStyle::kPrepend;
  return *this;
}

StatusBuilder& StatusBuilder::SetNoLogging() {
  no_logging_ = true;
  return *this;
}

StatusBuilder::operator absl::Status() const& {
  if (stream_->str().empty() || no_logging_) {
    return status_;
  }
  return StatusBuilder(*this).JoinMessageToStatus();
}

StatusBuilder::operator absl::Status() && {
  if (stream_->str().empty() || no_logging_) {
    return status_;
  }
  return JoinMessageToStatus();
}

absl::Status StatusBuilder::JoinMessageToStatus() const {
  std::string message;
  if (join_style_ == MessageJoinStyle::kAnnotate) {
    if (!status_.ok()) {
      message = absl::StrCat(status_.message(), "; ", stream_->str());
    }
  } else {
    message = join_style_ == MessageJoinStyle::kPrepend
                  ? absl::StrCat(stream_->str(), status_.message())
                  : absl::StrCat(status_.message(), stream_->str());
  }
  return absl::Status(status_.code(), message);
}

}  
