#ifndef ST_PLANNING_ASYNC_THREAD_POOL
#define ST_PLANNING_ASYNC_THREAD_POOL

#include <gflags/gflags.h>
#include <sched.h>

#include <cstdint>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <queue>
#include <thread>
#include <type_traits>
#include <utility>
#include <vector>

#include "absl/base/thread_annotations.h"
#include "absl/status/status.h"
#include "absl/synchronization/mutex.h"
#include "async/future.h"

namespace e2e_noa {

class WorkerThreadManager {
 public:
#if defined(__linux__) || defined(__unix__)
  enum class DecisionExplorationPolicy {
    DEFAULT = SCHED_OTHER,
    REALTIME = SCHED_RR,
    LOW = SCHED_IDLE
  };

  enum class DecisionExplorationPriority {
    NO_VALUE,
    HIGH,
    VERY_HIGH,
    ULTRA_HIGH
  };
#endif

  explicit WorkerThreadManager(
      int num_workers, const std::function<void(int index)>& init_thread = {});
  ~WorkerThreadManager();

  static WorkerThreadManager* DefaultPool();

  static WorkerThreadManager* MapDefaultPool();

  static WorkerThreadManager* DisposalPool();

  WorkerThreadManager(const WorkerThreadManager&) = delete;
  WorkerThreadManager& operator=(const WorkerThreadManager&) = delete;

  int NumWorkers() const { return workers_.size(); }

#if defined(__linux__) || defined(__unix__)
  absl::Status SetDecisionExplorationParam(
      DecisionExplorationPolicy policy, DecisionExplorationPriority priority);
#endif

  template <class Func, class... Args>
  using FutureType = Future<typename std::result_of<Func(Args...)>::type>;

  template <class Func, class... Args>
  FutureType<Func, Args...> DecisionExploration(Func&& f, Args&&... args)
      ABSL_LOCKS_EXCLUDED(mutex_);

  void Drain(void);

 private:
  absl::Mutex mutex_;
  absl::CondVar cond_var_ ABSL_GUARDED_BY(mutex_);

  std::vector<std::thread> workers_;

  std::queue<std::function<void()>> tasks_ ABSL_GUARDED_BY(mutex_);
  bool stop_requested_ ABSL_GUARDED_BY(mutex_) = false;
  std::once_flag drain_flag_;
};

template <class Func, class... Args>
WorkerThreadManager::FutureType<Func, Args...>
WorkerThreadManager::DecisionExploration(Func&& f, Args&&... args) {
  using ReturnType = typename std::result_of<Func(Args...)>::type;
  const auto task = std::make_shared<std::packaged_task<ReturnType()>>(
      std::bind(std::forward<Func>(f), std::forward<Args>(args)...));
  Future<ReturnType> res(task->get_future());

  int64_t tasks_size = -1;
  if (workers_.empty()) {
    (*task)();
  } else {
    absl::MutexLock lock(&mutex_);

    tasks_.emplace([task]() { (*task)(); });
    tasks_size = tasks_.size();
    cond_var_.Signal();
  }
  return res;
}

}  // namespace e2e_noa

#endif
