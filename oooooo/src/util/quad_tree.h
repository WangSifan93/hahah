#ifndef ONBOARD_PLANNER_UTIL_QUAD_TREE_H_
#define ONBOARD_PLANNER_UTIL_QUAD_TREE_H_

#include <limits.h>

#include <algorithm>
#include <array>
#include <cstddef>
#include <iterator>
#include <tuple>
#include <utility>
#include <vector>

#include "glog/logging.h"
#include "math/geometry/aabox2d.h"
#include "math/vec.h"

namespace e2e_noa::planning {
template <class T>
class QuadTree {
 private:
  struct Node {
    int total_num_elements_in_tree = 0;
    int father = 0;
    int id_as_a_child = 0;
    int element_info_id = -1;
    int children[4] = {0};
  };

  struct ElementInfo {
    int level = 0;
    int xid = 0;
    int yid = 0;
    int node_id = 0;
    T element;
  };

 public:
  static constexpr int kMaxDepth = 28;

  struct ConstElementReferenceArray {
    int num_elements = 0;
    std::array<const T*, kMaxDepth> elements;
  };

  struct ElementReferenceArray {
    int num_elements = 0;
    std::array<T*, kMaxDepth> elements;
  };

  struct ElementInfoIndexArray {
    int num_elements = 0;
    std::array<int, kMaxDepth> elements;
  };

  class iterator {
   public:
    using iterator_category = std::forward_iterator_tag;
    using difference_type = std::ptrdiff_t;
    using value_type = T;
    using pointer = T*;
    using reference = T&;

    explicit iterator(std::vector<ElementInfo>* elements, int id)
        : elements_(CHECK_NOTNULL(elements)), id_(id) {}

    reference operator*() {
      DCHECK(IsValid());
      return (*elements_)[id_].element;
    }
    pointer operator->() {
      DCHECK(IsValid());
      return &(*elements_)[id_].element;
    }

    std::tuple<int, int, int> GetIndices() const {
      DCHECK(IsValid());
      const ElementInfo& info = (*elements_)[id_];
      return std::make_tuple(info.level, info.xid, info.yid);
    }

    iterator& operator++() {
      DCHECK(IsValid());
      id_++;
      return *this;
    }
    iterator operator++(int) {
      iterator retval = *this;
      ++(*this);
      return retval;
    }

    bool operator==(const iterator& other) { return id_ == other.id_; }
    bool operator!=(const iterator& other) { return id_ != other.id_; }

   private:
    bool IsValid() const {
      return elements_ != nullptr && id_ >= 0 && id_ < elements_->size();
    }

    std::vector<ElementInfo>* elements_ = nullptr;
    int id_ = -1;
  };

  class const_iterator {
   public:
    using iterator_category = std::forward_iterator_tag;
    using difference_type = std::ptrdiff_t;
    using value_type = T;
    using pointer = const T*;
    using reference = const T&;

    explicit const_iterator(const std::vector<ElementInfo>* elements, int id)
        : elements_(CHECK_NOTNULL(elements)), id_(id) {}

    reference operator*() const {
      DCHECK(IsValid());
      return (*elements_)[id_].element;
    }
    pointer operator->() const {
      DCHECK(IsValid());
      return &(*elements_)[id_].element;
    }

    std::tuple<int, int, int> GetIndices() const {
      DCHECK(IsValid());
      const ElementInfo& info = (*elements_)[id_];
      return std::make_tuple(info.level, info.xid, info.yid);
    }

    const_iterator& operator++() {
      DCHECK(IsValid());
      id_++;
      return *this;
    }
    const_iterator operator++(int) {
      const_iterator retval = *this;
      ++(*this);
      return retval;
    }

    bool operator==(const const_iterator& other) { return id_ == other.id_; }
    bool operator!=(const const_iterator& other) { return id_ != other.id_; }

   private:
    bool IsValid() const {
      return elements_ != nullptr && id_ >= 0 && id_ < elements_->size();
    }

    const std::vector<ElementInfo>* elements_ = nullptr;
    int id_ = -1;
  };

  explicit QuadTree(int depth)
      : max_num_elements_(INT_MAX / depth - 5), depth_(depth) {
    CHECK_GT(depth_, 0);
    CHECK_LE(depth_, kMaxDepth);

    nodes_.emplace_back();
  }

  bool IsInside(int level, int xid, int yid) const {
    if (level < 0 || level >= depth_) {
      return false;
    }
    const int id_upper_bound = (1 << level);

    return xid >= 0 && xid < id_upper_bound && yid >= 0 && yid < id_upper_bound;
  }

  static bool IsPositionInside(double x, double y, double root_box_width) {
    return (x >= 0 && x <= root_box_width && y >= 0 && y <= root_box_width);
  }

  AABox2d BoxAt(int level, int xid, int yid, double unit_box_width) const {
    CHECK(IsInside(level, xid, yid));
    const double width = (1 << (depth_ - 1 - level)) * unit_box_width;
    return AABox2d{Vec2d{(xid + 0.5) * width, (yid + 0.5) * width}, width,
                   width};
  }

  double RootBoxWidthToUnitBoxWidth(double root_box_width) const {
    return root_box_width / (1 << (depth_ - 1));
  }

  double UnitBoxWidthToRootBoxWidth(double unit_box_width) const {
    return unit_box_width * (1 << (depth_ - 1));
  }

  bool AddElement(int level, int xid, int yid, T element) {
    CHECK(IsInside(level, xid, yid));

    if (size() + 1 > max_num_elements_) {
      return false;
    }

    int* pnode = &root_;
    int node = 0;

    int father = 0;
    int id_as_a_child = 0;

    int relative_xid = xid;
    int relative_yid = yid;

    for (int l = 0; l <= level; ++l) {
      if (*pnode == 0) {
        const int new_node = static_cast<int>(nodes_.size());
        *pnode = new_node;

        nodes_.emplace_back();

        node = new_node;

        nodes_[node].father = father;
        nodes_[node].id_as_a_child = id_as_a_child;
      } else {
        node = *pnode;
      }

      if (l == level) {
        DCHECK_LT(nodes_[node].element_info_id, 0);
        const int new_element = static_cast<int>(element_infos_.size());
        element_infos_.push_back({.level = level,
                                  .xid = xid,
                                  .yid = yid,
                                  .node_id = node,
                                  .element = std::move(element)});
        nodes_[node].element_info_id = new_element;
      } else {
        const int mid = 1 << (level - 1 - l);
        father = node;

        if (relative_xid >= mid) {
          relative_xid -= mid;
          if (relative_yid >= mid) {
            relative_yid -= mid;
            id_as_a_child = 3;
          } else {
            id_as_a_child = 2;
          }
        } else {
          if (relative_yid >= mid) {
            relative_yid -= mid;
            id_as_a_child = 1;
          } else {
            id_as_a_child = 0;
          }
        }
        pnode = &nodes_[node].children[id_as_a_child];
      }
    }

    while (node != 0) {
      nodes_[node].total_num_elements_in_tree += 1;
      node = nodes_[node].father;
    }

    return true;
  }

  bool RemoveElement(int level, int xid, int yid) {
    DCHECK(IsInside(level, xid, yid));
    int node = root_;
    bool removed = false;

    for (int l = 0; l <= level; ++l) {
      if (node == 0) {
        break;
      } else if (l == level) {
        if (nodes_[node].element_info_id >= 0) {
          int removed_element = nodes_[node].element_info_id;

          int last_id = static_cast<int>(element_infos_.size() - 1);
          if (last_id != removed_element) {
            nodes_[element_infos_.back().node_id].element_info_id =
                removed_element;

            element_infos_[removed_element] =
                std::move(element_infos_[last_id]);
          }

          element_infos_.pop_back();

          nodes_[node].element_info_id = -1;
          removed = true;
        } else {
          removed = false;
        }
      } else {
        const int mid = 1 << (level - 1 - l);

        int id_as_a_child = -1;
        if (xid >= mid) {
          xid -= mid;

          if (yid >= mid) {
            yid -= mid;
            id_as_a_child = 3;
          } else {
            id_as_a_child = 2;
          }
        } else {
          if (yid >= mid) {
            yid -= mid;
            id_as_a_child = 1;
          } else {
            id_as_a_child = 0;
          }
        }
        node = nodes_[node].children[id_as_a_child];
      }
    }

    if (removed) {
      int node_reverse = node;

      while (node_reverse != 0) {
        nodes_[node_reverse].total_num_elements_in_tree -= 1;
        node_reverse = nodes_[node_reverse].father;
      }

      node_reverse = node;
      while (node_reverse != 0) {
        if (nodes_[node_reverse].total_num_elements_in_tree < 1) {
          node_reverse = DeleteChildLessNode(node_reverse);
        } else {
          break;
        }
      }
    }

    return removed;
  }

  const T* GetElement(int level, int xid, int yid) const {
    const int element_id = ElementInfoIdFromIndices(level, xid, yid);
    return element_id >= 0 ? &(element_infos_[element_id].element) : nullptr;
  }

  T* GetMutableElement(int level, int xid, int yid) {
    const int element_id = ElementInfoIdFromIndices(level, xid, yid);
    return element_id >= 0 ? &(element_infos_[element_id].element) : nullptr;
  }

  const T* GetElementByPosition(double x, double y, double root_box_width = 1.,
                                int max_level = INT_MAX) const {
    const int element_id =
        ElementInfoIdFromPosition(x, y, max_level, root_box_width);
    return element_id >= 0 ? &(element_infos_[element_id].element) : nullptr;
  }

  T* GetMutableElementByPosition(double x, double y, double root_box_width = 1.,
                                 int max_level = INT_MAX) {
    const int element_id =
        ElementInfoIdFromPosition(x, y, max_level, root_box_width);
    return element_id >= 0 ? &(element_infos_[element_id].element) : nullptr;
  }

  ConstElementReferenceArray GetElementsByPosition(
      double x, double y, double root_box_width = 1.,
      int max_level = INT_MAX) const {
    ConstElementReferenceArray result;
    const ElementInfoIndexArray element_info_ids =
        ElementInfoIdsFromPosition(x, y, root_box_width, max_level);

    result.num_elements = element_info_ids.num_elements;
    for (int i = 0; i < element_info_ids.num_elements; ++i) {
      result.elements[i] =
          &element_infos_[element_info_ids.elements[i]].element;
    }
    return result;
  }

  ElementReferenceArray GetMutableElementsByPosition(double x, double y,
                                                     double root_box_width = 1.,
                                                     int max_level = INT_MAX) {
    ElementReferenceArray result;
    const ElementInfoIndexArray element_info_ids =
        ElementInfoIdsFromPosition(x, y, root_box_width, max_level);

    result.num_elements = element_info_ids.num_elements;
    for (int i = 0; i < element_info_ids.num_elements; ++i) {
      result.elements[i] =
          &element_infos_[element_info_ids.elements[i]].element;
    }
    return result;
  }

  double depth() const { return depth_; }

  constexpr size_t size() const { return element_infos_.size(); }

  iterator begin() { return iterator{&element_infos_, 0}; }

  iterator end() {
    return iterator{&element_infos_, static_cast<int>(element_infos_.size())};
  }

  const_iterator cbegin() const { return const_iterator{&element_infos_, 0}; }

  const_iterator cend() const {
    return const_iterator{&element_infos_,
                          static_cast<int>(element_infos_.size())};
  }

  void reserve(int n) {
    n = std::min(n, max_num_elements_);

    size_t node_vector_size = n * depth_;
    size_t element_vector_size = n;

    nodes_.reserve(node_vector_size);
    element_infos_.reserve(element_vector_size);
  }

  void clear() {
    nodes_.clear();
    element_infos_.clear();
    root_ = 0;

    nodes_.emplace_back();
  }

  int max_num_elements() const { return max_num_elements_; }

 private:
  int max_num_elements_ = INT_MAX - 2;
  int DeleteChildLessNode(int id) {
    const int last_id = static_cast<int>(nodes_.size()) - 1;
    int father = 0;
    if (id > 0 && id < nodes_.size()) {
      const Node& current = nodes_[id];
      father = current.father;
      if (current.father != 0) {
        nodes_[current.father].children[current.id_as_a_child] = 0;
      }
      if (root_ == id) {
        root_ = 0;
      }

      if (last_id != id) {
        const Node& last = nodes_[last_id];
        if (last.father != 0) {
          nodes_[last.father].children[last.id_as_a_child] = id;
        }
        for (int i = 0; i < 4; ++i) {
          if (last.children[i] != 0) {
            nodes_[last.children[i]].father = id;
          }
        }
        if (last.element_info_id >= 0) {
          element_infos_[last.element_info_id].node_id = id;
        }
        if (root_ == last_id) {
          root_ = id;
        }
        if (father == last_id) {
          father = id;
        }
        nodes_[id] = nodes_[last_id];
      }

      nodes_.pop_back();
    }
    return father;
  }

  int ElementInfoIdFromIndices(int level, int xid, int yid) const {
    DCHECK(IsInside(level, xid, yid));

    int node = root_;
    int mid = 0;
    int id_as_a_child = 0;

    while (level > 0 && node != 0) {
      mid = 1 << (level - 1);
      if (xid >= mid) {
        xid -= mid;
        if (yid >= mid) {
          id_as_a_child = 3;
          yid -= mid;
        } else {
          id_as_a_child = 2;
        }
      } else {
        if (yid >= mid) {
          id_as_a_child = 1;
          yid -= mid;
        } else {
          id_as_a_child = 0;
        }
      }
      node = nodes_[node].children[id_as_a_child];
      level--;
    }

    if (node == 0) {
      return -1;
    }

    return nodes_[node].element_info_id;
  }

  int ElementInfoIdFromPosition(double x, double y, int max_level,
                                double root_box_width = 1.) const {
    if (!IsPositionInside(x, y, root_box_width)) {
      return -1;
    }
    max_level = std::min(max_level, depth_ - 1);
    if (max_level < 0) {
      return -1;
    }

    int node = root_;
    int target_element_info = -1;
    int curr_level = 0;
    double half_width = root_box_width * 0.5;

    while (curr_level < max_level && node != 0) {
      if (nodes_[node].element_info_id >= 0) {
        target_element_info = nodes_[node].element_info_id;
      }

      int id_as_a_child = 0;
      if (x >= half_width) {
        x -= half_width;
        if (y >= half_width) {
          id_as_a_child = 3;
          y -= half_width;
        } else {
          id_as_a_child = 2;
        }
      } else {
        if (y >= half_width) {
          id_as_a_child = 1;
          y -= half_width;
        } else {
          id_as_a_child = 0;
        }
      }
      node = nodes_[node].children[id_as_a_child];
      curr_level++;
      half_width *= 0.5;
    }

    if (node != 0 && nodes_[node].element_info_id >= 0) {
      target_element_info = nodes_[node].element_info_id;
    }

    return target_element_info;
  }

  ElementInfoIndexArray ElementInfoIdsFromPosition(
      double x, double y, int max_level, double root_box_width = 1.) const {
    ElementInfoIndexArray result;

    if (!IsPositionInside(x, y, root_box_width)) {
      return result;
    }
    max_level = std::min(max_level, depth_ - 1);
    if (max_level < 0) {
      return result;
    }

    int node = root_;
    int curr_level = 0;
    double half_width = root_box_width * 0.5;

    while (curr_level < max_level && node != 0) {
      int element_id = nodes_[node].element_info_id;
      if (element_id >= 0) {
        result.elements[result.num_elements] = element_id;
        result.num_elements += 1;
      }

      int id_as_a_child = 0;

      if (x >= half_width) {
        x -= half_width;
        if (y >= half_width) {
          id_as_a_child = 3;
          y -= half_width;
        } else {
          id_as_a_child = 2;
        }
      } else {
        if (y >= half_width) {
          id_as_a_child = 1;
          y -= half_width;
        } else {
          id_as_a_child = 0;
        }
      }
      node = nodes_[node].children[id_as_a_child];
      curr_level++;
      half_width *= 0.5;
    }

    if (node != 0 && nodes_[node].element_info_id >= 0) {
      result.elements[result.num_elements] = nodes_[node].element_info_id;
      result.num_elements += 1;
    }
    return result;
  }

  int depth_ = 0;
  std::vector<Node> nodes_;
  std::vector<ElementInfo> element_infos_;
  int root_ = 0;
};

}  // namespace e2e_noa::planning

#endif
