#ifndef ONBOARD_LITE_UTILS_THREAD_UTIL_H_
#define ONBOARD_LITE_UTILS_THREAD_UTIL_H_

#include <mutex>
#include <string>
#include <unordered_map>

#include "base/singleton.h"

namespace e2e_noa {

#define CONTEXT_THREAD_NAME_SET(thread_name) \
  e2e_noa::ThreadUtil::Instance()->SetThreadName(thread_name);

#define CONTEXT_THREAD_NAME_GET(pid) \
  e2e_noa::ThreadUtil::Instance()->GetThreadName(pid);

class ThreadUtil {
 public:
  void SetThreadName(const std::string& name);
  const std::string GetThreadName(int pid);

 private:
  std::mutex mutex_;
  std::unordered_map<int, std::string> pid_to_name;

  DECLARE_SINGLETON(ThreadUtil);
};

}  // namespace e2e_noa

#endif
