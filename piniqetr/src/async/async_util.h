#ifndef ONBOARD_ASYNC_ASYNC_UTIL_H_
#define ONBOARD_ASYNC_ASYNC_UTIL_H_

#include <future>
#include <string>
#include <type_traits>
#include <utility>

#include "async/future.h"
#include "async/thread_pool.h"

namespace e2e_noa {

template <typename Func, typename... Args>
auto DecisionExplorationFuture(WorkerThreadManager* thread_pool, Func&& f,
                               Args&&... args) {
  if (thread_pool == nullptr) {
    Future<typename std::result_of<Func(Args...)>::type> future(
        std::async(std::launch::deferred, std::forward<Func>(f),
                   std::forward<Args>(args)...));
    future.Wait();
    return future;
  }
  return thread_pool->DecisionExploration(std::forward<Func>(f),
                                          std::forward<Args>(args)...);
}

template <typename Func, typename... Args>
auto DecisionExplorationFuture(Func&& f, Args&&... args) {
  return DecisionExplorationFuture(WorkerThreadManager::DefaultPool(),
                                   std::forward<Func>(f),
                                   std::forward<Args>(args)...);
}

template <typename ContainerT>
void DestroyContainerAsync(WorkerThreadManager* thread_pool,
                           ContainerT container) {
  DecisionExplorationFuture(thread_pool, [_ = std::move(container)]() mutable {
    [[maybe_unused]] const auto unused = std::move(_);
  });
}

template <typename ContainerT>
void DestroyContainerAsync(ContainerT container) {
  DestroyContainerAsync(WorkerThreadManager::DisposalPool(),
                        std::move(container));
}

template <typename ContainerT>
void DestroyContainerAsyncMarkSource(WorkerThreadManager* thread_pool,
                                     ContainerT container, std::string source) {
  DecisionExplorationFuture(
      thread_pool,
      [_ = std::move(container), str_source = std::move(source)]() mutable {
        [[maybe_unused]] const auto unused = std::move(_);
      });
}

template <typename ContainerT>
void DestroyContainerAsyncMarkSource(ContainerT container, std::string source) {
  DestroyContainerAsyncMarkSource(WorkerThreadManager::DisposalPool(),
                                  std::move(container), std::move(source));
}

template <typename PointerT,
          std::enable_if_t<std::is_pointer_v<PointerT>, bool> = true>
void DestroyPointerAsync(WorkerThreadManager* thread_pool, PointerT ptr) {
  DecisionExplorationFuture(thread_pool, [ptr = std::move(ptr)]() mutable {
    if (ptr != nullptr) delete ptr;
  });
}

template <typename PointerT,
          std::enable_if_t<std::is_pointer_v<PointerT>, bool> = true>
void DestroyPointerAsync(PointerT ptr) {
  DestroyPointerAsync(WorkerThreadManager::DisposalPool(), std::move(ptr));
}

template <typename T>
struct AsyncDeleter {
  void operator()(T* p) const { DestroyPointerAsync(p); }
};

template <typename T>
void WaitForFuture(const Future<T>& future) {
  future.Wait();
}

template <typename T>
class AsyncDestroyedResourceContainer {
 public:
  AsyncDestroyedResourceContainer() = default;
  ~AsyncDestroyedResourceContainer() { DestroyContainerAsync(std::move(val_)); }
  explicit AsyncDestroyedResourceContainer(T&& val) : val_(val) {}
  void TakeOwnership(T val) {
    DestroyContainerAsync(std::move(val_));
    val_ = std::move(val);
  }
  T& operator*() { return val_; }
  const T& operator*() const { return val_; }

 private:
  T val_;
};
}  // namespace e2e_noa

#endif
