#ifndef ST_PLANNING_UTIL_TIME_UTIL
#define ST_PLANNING_UTIL_TIME_UTIL

#ifndef ONBOARD_UTILS_TIME_UTIL_H_
#define ONBOARD_UTILS_TIME_UTIL_H_

#include <cstdint>
#include <string>

#include "absl/time/clock.h"
#include "absl/time/time.h"
#include "google/protobuf/timestamp.pb.h"
#include "google/protobuf/util/time_util.h"

namespace e2e_noa {

constexpr absl::Time kMaxTime = absl::FromUnixSeconds(
    google::protobuf::util::TimeUtil::kTimestampMaxSeconds);

inline double ToUnixDoubleSeconds(absl::Time t) {
  return absl::ToDoubleSeconds(absl::time_internal::ToUnixDuration(t));
}

inline absl::Time FromUnixDoubleSeconds(double s) {
  return absl::time_internal::FromUnixDuration(
      absl::time_internal::MakePosDoubleDuration(s));
}

inline double UnixNow() { return ToUnixDoubleSeconds(absl::Now()); }

inline std::string FetchCurrentTimeString() {
  return absl::FormatTime("%Y-%m-%d %H:%M:%S", absl::Now(),
                          absl::UTCTimeZone());
}

inline std::string FetchTimeString(const absl::Time& time) {
  return absl::FormatTime("%Y-%m-%d %H:%M:%S", time, absl::UTCTimeZone());
}

inline std::string FetchTimeString(const absl::Time& time,
                                   absl::TimeZone zone) {
  return absl::FormatTime("%Y-%m-%d %H:%M:%S", time, zone);
}

constexpr double MicroSecondsToSeconds(int64_t microseconds) {
  return microseconds * 1E-6;
}

constexpr int64_t SecondsToMicroSeconds(double s) {
  return static_cast<int64_t>(s * 1E6);
}

absl::Time FromProto(const google::protobuf::Timestamp& proto);

void ToProto(absl::Time time, google::protobuf::Timestamp* proto);

}  // namespace e2e_noa

#endif

#endif
