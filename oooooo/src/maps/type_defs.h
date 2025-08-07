#ifndef MAPS_TYPE_DEFS_H_
#define MAPS_TYPE_DEFS_H_

#include <cstdint>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/container/flat_hash_set.h"

namespace e2e_noa::mapping {
template <typename Key, typename T>
using FlatHashMap = absl::flat_hash_map<Key, T>;

template <typename T>
using FlatHashSet = absl::flat_hash_set<T>;

using PatchId = uint64_t;
using PixelId = uint64_t;

using PatchIds = FlatHashSet<PatchId>;
}  // namespace e2e_noa::mapping

#endif
