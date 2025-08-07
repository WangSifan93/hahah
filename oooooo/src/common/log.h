#ifndef AD_E2E_PLANNING_COMMON_LOG
#define AD_E2E_PLANNING_COMMON_LOG

#include <glog/logging.h>

#include <sstream>
#include <string>

#include "common/gflags.h"

namespace ad_e2e {
namespace planning {
enum LogLevel { DEBUG = 0, INFO = 1, WARN = 2, ERROR = 3, FATAL = 4 };
}
}  // namespace ad_e2e

[[maybe_unused]] static std::string const_prefix(const char *file_name,
                                                 int line_no) {
  return std::string(std::string(file_name) + ":" + std::to_string(line_no));
}

[[maybe_unused]] static std::string const_prefix(char const *module_name,
                                                 char const *file_name,
                                                 int line_no) {
  return std::string(std::string(module_name) + ":" + std::string(file_name) +
                     ":" + std::to_string(line_no));
}

#define __FILENAME__ \
  (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)

#define LLOG_LEVEL(log_level)                                          \
  do {                                                                 \
    FLAGS_ad_e2e_planning_log_level = static_cast<int32_t>(log_level); \
  } while (0)

#define LFATAL(args...)                                               \
  do {                                                                \
    if (FLAGS_ad_e2e_planning_log_level <= ad_e2e::planning::FATAL) { \
      LOG(FATAL) << args;                                             \
    }                                                                 \
  } while (0)
#define LERROR(args...)                                               \
  do {                                                                \
    if (FLAGS_ad_e2e_planning_log_level <= ad_e2e::planning::ERROR) { \
      LOG(ERROR) << args;                                             \
    }                                                                 \
  } while (0)
#define LWARN(args...)                                               \
  do {                                                               \
    if (FLAGS_ad_e2e_planning_log_level <= ad_e2e::planning::WARN) { \
      LOG(WARNING) << args;                                          \
    }                                                                \
  } while (0)
#define LINFO(args...)                                               \
  do {                                                               \
    if (FLAGS_ad_e2e_planning_log_level <= ad_e2e::planning::INFO) { \
      LOG(INFO) << args;                                             \
    }                                                                \
  } while (0)
#define LDEBUG(args...)                                               \
  do {                                                                \
    if (FLAGS_ad_e2e_planning_log_level <= ad_e2e::planning::DEBUG) { \
      VLOG(1) << args;                                                \
    }                                                                 \
  } while (0)

#define LOG_FATAL(...)         \
  do {                         \
    std::ostringstream oss;    \
    oss << __VA_ARGS__;        \
    LFATAL(oss.str().c_str()); \
  } while (0)

#define LOG_ERROR(...)         \
  do {                         \
    std::ostringstream oss;    \
    oss << __VA_ARGS__;        \
    LERROR(oss.str().c_str()); \
  } while (0)

#define LOG_WARN(...)         \
  do {                        \
    std::ostringstream oss;   \
    oss << __VA_ARGS__;       \
    LWARN(oss.str().c_str()); \
  } while (0)

#define LOG_INFO(...)         \
  do {                        \
    std::ostringstream oss;   \
    oss << __VA_ARGS__;       \
    LINFO(oss.str().c_str()); \
  } while (0)

#define LOG_DEBUG(...)         \
  do {                         \
    std::ostringstream oss;    \
    oss << __VA_ARGS__;        \
    LDEBUG(oss.str().c_str()); \
  } while (0)

#endif
