#ifndef ONBOARD_ASYNC_PARALLEL_FOR_H_
#define ONBOARD_ASYNC_PARALLEL_FOR_H_

#include <functional>
#include <iterator>

#include "async/thread_pool.h"

namespace e2e_noa {

namespace {
class BlockUntilFinished {
 public:
  explicit BlockUntilFinished(int num_iters)
      : num_iters_(num_iters), iters_done_(0) {}

  void Finished(int iters_done) {
    absl::MutexLock lock(&mutex_);
    iters_done_ += iters_done;
    cond_var_.Signal();
  }

  void Block() {
    absl::MutexLock lock(&mutex_);
    while (iters_done_ != num_iters_) {
      cond_var_.Wait(&mutex_);
    }
  }

 private:
  absl::Mutex mutex_;
  absl::CondVar cond_var_ ABSL_GUARDED_BY(mutex_);
  int num_iters_ ABSL_GUARDED_BY(mutex_);
  int iters_done_ ABSL_GUARDED_BY(mutex_);
};

struct SharedState {
  explicit SharedState(int num_iters) : block_until_finished(num_iters) {}

  std::atomic<int> next_index{0};

  BlockUntilFinished block_until_finished;
};

}  // namespace

namespace parallel_for {

struct Options {
  int block_size = 0;
};

class WorkerIndex {
 public:
  explicit WorkerIndex(int index) : index_(index) {}
  operator int() const { return index_; }

 private:
  const int index_;
};

}  // namespace parallel_for

void ParallelFor(int begin, int end, WorkerThreadManager* thread_pool,
                 const parallel_for::Options& options,
                 std::function<void(parallel_for::WorkerIndex, int)>&& func);
void ParallelFor(int begin, int end, WorkerThreadManager* thread_pool,
                 std::function<void(parallel_for::WorkerIndex, int)>&& func);
void ParallelFor(int begin, int end,
                 std::function<void(parallel_for::WorkerIndex, int)>&& func);

void ParallelFor(int begin, int end, WorkerThreadManager* thread_pool,
                 const parallel_for::Options& options,
                 std::function<void(int)>&& func);
void ParallelFor(int begin, int end, WorkerThreadManager* thread_pool,
                 std::function<void(int)>&& func);
void ParallelFor(int begin, int end, std::function<void(int)>&& func);

template <typename InputIter, typename F>
void ParallelFor(
    InputIter begin_it, InputIter end_it, F f,
    WorkerThreadManager* thread_pool = WorkerThreadManager::DefaultPool()) {
  if (begin_it == end_it) return;
  int end = std::distance(begin_it, end_it);

  if (end == 1) {
    f(*begin_it);
    return;
  }

  auto func = [f = std::move(f)](int worker_index, InputIter it) { f(*it); };
  const int begin = 0;
  const int num_iters = end - begin;
  int block_size =
      std::max<int>(1, num_iters / ((thread_pool->NumWorkers() + 1) << 2));

  auto shared_state = std::make_shared<SharedState>(num_iters);

  const auto grab_tasks = [=, &func](parallel_for::WorkerIndex worker_index) {
    int iters_done = 0;
    while (true) {
      const int index = shared_state->next_index.fetch_add(
                            block_size, std::memory_order_acq_rel) +
                        begin;
      if (index >= end) break;
      for (int i = index; i < std::min(end, index + block_size);
           ++i, ++iters_done) {
        func(worker_index, std::next(begin_it, i));
      }
    }
    shared_state->block_until_finished.Finished(iters_done);
  };

  const int min_num_workers = (num_iters + block_size - 1) / block_size;
  const int num_workers =
      std::min(min_num_workers - 1, thread_pool->NumWorkers());

  for (int i = 0; i < num_workers; ++i) {
    thread_pool->DecisionExploration(grab_tasks,
                                     parallel_for::WorkerIndex(i + 1));
  }

  grab_tasks(parallel_for::WorkerIndex(0));

  shared_state->block_until_finished.Block();
}

template <typename InputIter, typename F>
void MapParallelFor(
    InputIter begin_it, InputIter end_it, F f,
    WorkerThreadManager* thread_pool = WorkerThreadManager::MapDefaultPool()) {
  if (begin_it == end_it) return;
  int end = std::distance(begin_it, end_it);
  if (end == 1) {
    f(*begin_it);
    return;
  }

  auto func = [f = std::move(f)](int worker_index, InputIter it) { f(*it); };
  const int begin = 0;
  const int num_iters = end - begin;
  int block_size =
      std::max<int>(1, num_iters / ((thread_pool->NumWorkers() + 1) << 2));

  auto shared_state = std::make_shared<SharedState>(num_iters);

  const auto grab_tasks = [=, &func](parallel_for::WorkerIndex worker_index) {
    int iters_done = 0;
    while (true) {
      const int index = shared_state->next_index.fetch_add(
                            block_size, std::memory_order_acq_rel) +
                        begin;
      if (index >= end) break;
      for (int i = index; i < std::min(end, index + block_size);
           ++i, ++iters_done) {
        func(worker_index, std::next(begin_it, i));
      }
    }
    shared_state->block_until_finished.Finished(iters_done);
  };

  const int min_num_workers = (num_iters + block_size - 1) / block_size;
  const int num_workers =
      std::min(min_num_workers - 1, thread_pool->NumWorkers());

  for (int i = 0; i < num_workers; ++i) {
    thread_pool->DecisionExploration(grab_tasks,
                                     parallel_for::WorkerIndex(i + 1));
  }

  grab_tasks(parallel_for::WorkerIndex(0));

  shared_state->block_until_finished.Block();
}

}  // namespace e2e_noa

#endif
