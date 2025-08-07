/******************************************************************************
 * Copyright 2024 The zpilot Authors. All Rights Reserved.
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
 * @file indexed_ptr_list.h
 * @brief indexed list container for pointers.
 **/

#pragma once

#include <unordered_map>
#include <vector>

#include "apps/planning/src/common/log.h"
#include "map_util.h"

namespace zark {
namespace planning {

template <typename I, typename T>
class IndexedPtrList {
 public:
  /**
   * @brief copy pointer into the container. If the id is already exist,
   * overwrite the pointer in the container.
   * @param id the id of the pointer
   * @param pointer the pointer to be copied to the container.
   * @return The pointer.
   */
  T Add(const I id, const T pointer) {
    T* found_in_dict = ::util::FindOrNull(pointer_dict_, id);
    if (found_in_dict) {
      AWARN << "pointer " << id << " is already in container";
      auto found_in_list = std::find(pointer_list_.begin(), pointer_list_.end(),
                                     pointer_dict_[id]);
      *found_in_dict = pointer;
      *found_in_list = pointer;
    } else {
      pointer_dict_.insert({id, pointer});
      pointer_list_.push_back(pointer);
    }
    return pointer;
  }

  /**
   * @brief Find pointer by id in the container
   * @param id the id of the pointer
   * @return the raw pointer to the pointer if found.
   * @return nullptr if the pointer is not found.
   */
  T Find(const I id) const {
    auto* found = ::util::FindOrNull(pointer_dict_, id);
    if (!found) {
      return nullptr;
    }
    return *found;
  }

  /**
   * @brief List all the items in the container.
   * @return the list of const raw pointers of the objects in the container.
   */
  const std::vector<T>& Items() const { return pointer_list_; }

  /**
   * @brief List all the items in the container.
   * @return the unordered_map of ids and objects in the container.
   */
  const std::unordered_map<I, T>& Dict() const { return pointer_dict_; }

  /**
   * @brief Copy the container with objects.
   */
  IndexedPtrList& operator=(const IndexedPtrList& other) {
    this->pointer_list_.clear();
    this->pointer_dict_.clear();
    for (const auto& item : other.Dict()) {
      Add(item.first, item.second);
    }
    return *this;
  }

 private:
  std::vector<T> pointer_list_;
  std::unordered_map<I, T> pointer_dict_;
};

}  // namespace planning
}  // namespace zark
