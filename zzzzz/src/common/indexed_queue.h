/******************************************************************************
 * Copyright 2017 The zpilot Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/**
 * @file:
 **/

#pragma once

#include <memory>
#include <queue>
#include <unordered_map>
#include <utility>

namespace {
// Returns a pointer to the const value associated with the given key if it
// exists, or nullptr otherwise.
template <class Collection>
const typename Collection::value_type::second_type* FindOrNull(
    const Collection& collection,
    const typename Collection::value_type::first_type& key) {
  typename Collection::const_iterator it = collection.find(key);
  if (it == collection.end()) {
    return 0;
  }
  return &it->second;
}

// Same as above but returns a pointer to the non-const value.
template <class Collection>
typename Collection::value_type::second_type* FindOrNull(
    Collection& collection,  // NOLINT
    const typename Collection::value_type::first_type& key) {
  typename Collection::iterator it = collection.find(key);
  if (it == collection.end()) {
    return 0;
  }
  return &it->second;
}
}  // namespace

namespace zark {
namespace planning {

template <typename I, typename T>
class IndexedQueue {
 public:
  // Get infinite capacity with 0.
  explicit IndexedQueue(size_t capacity) : capacity_(capacity) {}

  const T* Find(const I id) const {  // to do;
    auto* result = FindOrNull(map_, id);
    return result ? result->get() : nullptr;
    // return nullptr;
  }

  const T* Latest() const {
    if (queue_.empty()) {
      return nullptr;
    }
    return Find(queue_.back().first);
  }

  bool Add(const I id, std::unique_ptr<T> ptr) {
    if (Find(id)) {
      return false;
    }
    if (capacity_ > 0 && queue_.size() == capacity_) {
      map_.erase(queue_.front().first);
      queue_.pop();
    }
    queue_.emplace(id, ptr.get());
    map_[id] = std::move(ptr);
    return true;
  }

  void Clear() {
    while (!queue_.empty()) {
      queue_.pop();
    }
    map_.clear();
  }

 public:
  size_t capacity_ = 0;
  std::queue<std::pair<I, const T*>> queue_;
  std::unordered_map<I, std::unique_ptr<T>> map_;
};

}  // namespace planning
}  // namespace zark
