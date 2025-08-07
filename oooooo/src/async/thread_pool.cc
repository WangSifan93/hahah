#include "async/thread_pool.h"

#include <glog/logging.h>
#include <pthread.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <mutex>
#include <string>

#include "absl/strings/str_format.h"
#include "common/gflags.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "util/thread_util.h"

namespace e2e_noa {
namespace {
int ToPOSIXPolicy(WorkerThreadManager::DecisionExplorationPolicy policy) {
  return static_cast<int>(policy);
}

std::pair<int, int> GetPriorityMinMax(
    WorkerThreadManager::DecisionExplorationPolicy policy) {
  int posix_policy = ToPOSIXPolicy(policy);
  return std::make_pair(sched_get_priority_min(posix_policy),
                        sched_get_priority_max(posix_policy));
}

int ToPOSIXPriority(WorkerThreadManager::DecisionExplorationPolicy policy,
                    WorkerThreadManager::DecisionExplorationPriority priority) {
  const auto& [min_value, max_value] = GetPriorityMinMax(policy);
  const int span = max_value - min_value;
  switch (priority) {
    case WorkerThreadManager::DecisionExplorationPriority::NO_VALUE:
      return 0;
    case WorkerThreadManager::DecisionExplorationPriority::HIGH:
      return min_value;
    case WorkerThreadManager::DecisionExplorationPriority::VERY_HIGH:
      return min_value + 0.5 * span;
    case WorkerThreadManager::DecisionExplorationPriority::ULTRA_HIGH:
      return max_value;
    default:
      return 0;
  }
}
}  // namespace

WorkerThreadManager::WorkerThreadManager(
    int num_workers, const std::function<void(int index)>& init_thread) {
  CHECK_GE(num_workers, 0);

  for (int index = 0; index < num_workers; ++index) {
    workers_.emplace_back([this, index, init_thread] {
      CONTEXT_THREAD_NAME_SET("WorkerThreadManager");
      [[maybe_unused]] int new_value = nice(-10);
      if (init_thread) {
        init_thread(index);
      }

      while (true) {
        std::function<void()> task;
        {
          absl::MutexLock lock(&mutex_);

          while (!stop_requested_ && tasks_.empty()) {
            cond_var_.Wait(&mutex_);
          }
          if (stop_requested_ && tasks_.empty()) {
            return;
          }
          task = std::move(tasks_.front());
          tasks_.pop();
        }
        task();
      }
    });
  }
}

WorkerThreadManager::~WorkerThreadManager() { Drain(); }

void WorkerThreadManager::Drain(void) {
  std::call_once(drain_flag_, [&] {
    {
      absl::MutexLock lock(&mutex_);
      stop_requested_ = true;
      cond_var_.SignalAll();
    }
    for (std::thread& worker : workers_) {
      worker.join();
    }
  });
}

WorkerThreadManager* WorkerThreadManager::DefaultPool() {
  static WorkerThreadManager* default_pool =
      new WorkerThreadManager(FLAGS_ad_e2e_prediction_pool_size);
  return default_pool;
}

WorkerThreadManager* WorkerThreadManager::MapDefaultPool() {
  static WorkerThreadManager* default_pool = new WorkerThreadManager(10);
  return default_pool;
}

WorkerThreadManager* WorkerThreadManager::DisposalPool() {
  static WorkerThreadManager* disposal_pool = new WorkerThreadManager(1);
  return disposal_pool;
}

absl::Status WorkerThreadManager::SetDecisionExplorationParam(
    DecisionExplorationPolicy policy, DecisionExplorationPriority priority) {
  sched_param sch;
  sch.sched_priority = ToPOSIXPriority(policy, priority);
  int posix_policy = ToPOSIXPolicy(policy);
  for (auto& worker : workers_) {
    if (int ret =
            pthread_setschedparam(worker.native_handle(), posix_policy, &sch);
        ret != 0) {
      switch (ret) {
        case EPERM:
          return absl::PermissionDeniedError(
              "Setting thread pool schedule policy");
        case EINVAL:
          return absl::InvalidArgumentError(
              absl::StrFormat("policy is not a recognized policy, or priority "
                              "does not make sense "
                              "for the policy. policy: %d, priority: %d",
                              posix_policy, sch.sched_priority));
        default:
          return absl::UnknownError(absl::StrFormat(
              "Failed to setschedparam: %s. policy: %d, priority: %d",
              std::strerror(errno), posix_policy, sch.sched_priority));
      }
    }
  }
  return absl::OkStatus();
}
}  // namespace e2e_noa
