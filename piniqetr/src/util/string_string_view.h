#pragma once

#include <string_view>

#include "absl/container/flat_hash_map.h"
#include "absl/container/flat_hash_set.h"
#include "absl/strings/string_view.h"

namespace e2e_noa {
namespace planning {

struct TransparentHash {
  using is_transparent = void;
  size_t operator()(std::string_view sv) const noexcept {
    return absl::Hash<std::string_view>{}(sv);
  }
  size_t operator()(const std::string& s) const noexcept {
    return absl::Hash<std::string_view>{}(s);
  }
};

struct TransparentEqual {
  using is_transparent = void;
  bool operator()(std::string_view a, std::string_view b) const noexcept {
    return a == b;
  }
  bool operator()(const std::string& a, std::string_view b) const noexcept {
    return a == b;
  }
  bool operator()(std::string_view a, const std::string& b) const noexcept {
    return a == b;
  }
  bool operator()(const std::string& a, const std::string& b) const noexcept {
    return a == b;
  }
};

}  // namespace planning
}  // namespace e2e_noa
