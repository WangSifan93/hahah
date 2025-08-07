#ifndef GEOMETRY_AABOX_KDTREE2D
#define GEOMETRY_AABOX_KDTREE2D

#include <math.h>
#include <stdint.h>

#include <algorithm>
#include <array>
#include <functional>
#include <iterator>
#include <limits>
#include <memory>
#include <new>
#include <ostream>
#include <queue>
#include <type_traits>
#include <utility>
#include <vector>

#include "absl/types/span.h"
#include "glog/logging.h"
#include "math/geometry/aabox2d.h"
#include "math/util.h"
#include "math/vec.h"

namespace e2e_noa {

struct AABoxKDTreeParams {
  int max_depth = -1;

  int max_leaf_size = -1;

  double max_leaf_dimension = -1.0;
};

template <class ObjectType>
class AABoxKDTree2dNode {
 public:
  using ObjectPtr = const ObjectType*;
  using ObjectPtrWithDistanceSqr = std::pair<ObjectPtr, double>;
  struct Compare {
    constexpr bool operator()(const ObjectPtrWithDistanceSqr& lhs,
                              const ObjectPtrWithDistanceSqr& rhs) const {
      return lhs.second < rhs.second;
    }
  };
  using PriorityQueue =
      std::priority_queue<ObjectPtrWithDistanceSqr,
                          std::vector<ObjectPtrWithDistanceSqr>, Compare>;

  AABoxKDTree2dNode(std::vector<ObjectPtr> objects,
                    const AABoxKDTreeParams& params, int depth)
      : depth_(depth) {
    CHECK(!objects.empty());

    ComputeBoundary(objects);
    ComputePartition();

    if (SplitToSubNodes(objects, params)) {
      std::vector<ObjectPtr> left_subnode_objects;
      std::vector<ObjectPtr> right_subnode_objects;
      PartitionObjects(std::move(objects), &left_subnode_objects,
                       &right_subnode_objects);

      if (!left_subnode_objects.empty()) {
        subnodes_[0] = std::make_unique<AABoxKDTree2dNode<ObjectType>>(
            std::move(left_subnode_objects), params, depth + 1);
      }
      if (!right_subnode_objects.empty()) {
        subnodes_[1] = std::make_unique<AABoxKDTree2dNode<ObjectType>>(
            std::move(right_subnode_objects), params, depth + 1);
      }
    } else {
      InitObjects(objects);
    }
  }

  ObjectPtr GetNearestObject(const Vec2d& point) const {
    ObjectPtr nearest_object = nullptr;
    double min_distance_sqr = std::numeric_limits<double>::infinity();
    GetNearestObjectInternal(point, &min_distance_sqr, &nearest_object);
    return nearest_object;
  }

  std::vector<ObjectPtr> GetKNearestObjects(int want_k,
                                            const Vec2d& point) const {
    PriorityQueue priority_queue = GetKNearestObjectsInternal(want_k, point);
    std::vector<ObjectPtr> nearest_objects;
    nearest_objects.reserve(priority_queue.size());
    while (!priority_queue.empty()) {
      auto& [ptr, _] = priority_queue.top();
      nearest_objects.push_back(ptr);
      priority_queue.pop();
    }
    std::reverse(nearest_objects.begin(), nearest_objects.end());
    return nearest_objects;
  }

  std::vector<ObjectPtr> GetObjects(const Vec2d& point,
                                    const double distance) const {
    std::vector<ObjectPtr> result_objects;
    GetObjectsInternal(point, distance, Sqr(distance), &result_objects);
    return result_objects;
  }

  std::vector<ObjectPtr> GetObjectsWithinAABox(
      const AABox2d& aabox,
      const std::function<bool(const ObjectPtr, const AABox2d&)>&
          is_aabox_contain_object) const {
    std::vector<ObjectPtr> result_objects;
    GetObjectsInternalWithinAABox(aabox, is_aabox_contain_object,
                                  &result_objects);
    return result_objects;
  }

  AABox2d GetBoundingBox() const {
    return AABox2d({min_x_, min_y_}, {max_x_, max_y_});
  }

 private:
  class ObjectsInfo {
   public:
    static std::unique_ptr<ObjectsInfo> MakeObjectInfo(int num_objects) {
      const int size_in_bytes = sizeof(ObjectsInfo) +
                                sizeof(ObjectPtr) * num_objects * 2 +
                                sizeof(double) * num_objects * 2;
      return std::unique_ptr<ObjectsInfo>(new (new char[size_in_bytes])
                                              ObjectsInfo(num_objects));
    }

    ObjectPtr* objects_sorted_by_min() {
      return reinterpret_cast<ObjectPtr*>(this + 1);
    }
    ObjectPtr* objects_sorted_by_max() {
      return reinterpret_cast<ObjectPtr*>(this + 1) + num_objects_;
    }
    double* objects_sorted_by_min_bound() {
      return reinterpret_cast<double*>(reinterpret_cast<ObjectPtr*>(this + 1) +
                                       num_objects_ * 2);
    }
    double* objects_sorted_by_max_bound() {
      return reinterpret_cast<double*>(reinterpret_cast<ObjectPtr*>(this + 1) +
                                       num_objects_ * 2) +
             num_objects_;
    }

   private:
    explicit ObjectsInfo(int num_objects) : num_objects_(num_objects) {}

    const int64_t num_objects_;
  };

  void InitObjects(const std::vector<ObjectPtr>& objects) {
    if (objects.empty()) return;

    num_objects_ = static_cast<int>(objects.size());
    objects_info_ = ObjectsInfo::MakeObjectInfo(num_objects_);
    auto* objects_sorted_by_min = objects_info_->objects_sorted_by_min();
    auto* objects_sorted_by_max = objects_info_->objects_sorted_by_max();
    auto* objects_sorted_by_min_bound =
        objects_info_->objects_sorted_by_min_bound();
    auto* objects_sorted_by_max_bound =
        objects_info_->objects_sorted_by_max_bound();

    for (int i = 0; i < num_objects_; ++i) {
      objects_sorted_by_min[i] = objects[i];
      objects_sorted_by_max[i] = objects[i];
    }

    if (partition_ == PARTITION_X) {
      std::stable_sort(
          objects_sorted_by_min, objects_sorted_by_min + num_objects_,
          [](ObjectPtr obj1, ObjectPtr obj2) {
            return obj1->ComputeAABox().min_x() < obj2->ComputeAABox().min_x();
          });
      for (int i = 0; i < num_objects_; ++i) {
        objects_sorted_by_min_bound[i] =
            objects_sorted_by_min[i]->ComputeAABox().min_x();
      }

      std::stable_sort(
          objects_sorted_by_max, objects_sorted_by_max + num_objects_,
          [](ObjectPtr obj1, ObjectPtr obj2) {
            return obj1->ComputeAABox().max_x() > obj2->ComputeAABox().max_x();
          });
      for (int i = 0; i < num_objects_; ++i) {
        objects_sorted_by_max_bound[i] =
            objects_sorted_by_max[i]->ComputeAABox().max_x();
      }
    } else {
      std::stable_sort(
          objects_sorted_by_min, objects_sorted_by_min + num_objects_,
          [](ObjectPtr obj1, ObjectPtr obj2) {
            return obj1->ComputeAABox().min_y() < obj2->ComputeAABox().min_y();
          });
      for (int i = 0; i < num_objects_; ++i) {
        objects_sorted_by_min_bound[i] =
            objects_sorted_by_min[i]->ComputeAABox().min_y();
      }

      std::stable_sort(
          objects_sorted_by_max, objects_sorted_by_max + num_objects_,
          [](ObjectPtr obj1, ObjectPtr obj2) {
            return obj1->ComputeAABox().max_y() > obj2->ComputeAABox().max_y();
          });
      for (int i = 0; i < num_objects_; ++i) {
        objects_sorted_by_max_bound[i] =
            objects_sorted_by_max[i]->ComputeAABox().max_y();
      }
    }
  }

  bool SplitToSubNodes(const std::vector<ObjectPtr>& objects,
                       const AABoxKDTreeParams& params) {
    if (params.max_depth >= 0 && depth_ >= params.max_depth) {
      return false;
    }
    if (objects.size() <= std::max(1, params.max_leaf_size)) {
      return false;
    }
    if (params.max_leaf_dimension >= 0.0 &&
        std::max(max_x_ - min_x_, max_y_ - min_y_) <=
            params.max_leaf_dimension) {
      return false;
    }
    return true;
  }

  double LowerDistanceSquareToPoint(const Vec2d& point) const {
    double dx = 0.0;
    if (point.x() < min_x_) {
      dx = min_x_ - point.x();
    } else if (point.x() > max_x_) {
      dx = point.x() - max_x_;
    }
    double dy = 0.0;
    if (point.y() < min_y_) {
      dy = min_y_ - point.y();
    } else if (point.y() > max_y_) {
      dy = point.y() - max_y_;
    }
    return dx * dx + dy * dy;
  }

  double UpperDistanceSquareToPoint(const Vec2d& point) const {
    const double mid_x = (min_x_ + max_x_) * 0.5;
    const double mid_y = (min_y_ + max_y_) * 0.5;
    const double dx =
        (point.x() > mid_x) ? (point.x() - min_x_) : (point.x() - max_x_);
    const double dy =
        (point.y() > mid_y) ? (point.y() - min_y_) : (point.y() - max_y_);
    return dx * dx + dy * dy;
  }

  void GetAllObjects(std::vector<ObjectPtr>* const result_objects) const {
    if (num_objects_ > 0) {
      result_objects->insert(
          result_objects->end(), objects_info_->objects_sorted_by_min(),
          objects_info_->objects_sorted_by_min() + num_objects_);
    }
    for (const auto& node : subnodes_) {
      if (node != nullptr) {
        node->GetAllObjects(result_objects);
      }
    }
  }

  void GetObjectsInternal(const Vec2d& point, const double distance,
                          const double distance_sqr,
                          std::vector<ObjectPtr>* const result_objects) const {
    if (LowerDistanceSquareToPoint(point) > distance_sqr) {
      return;
    }
    if (UpperDistanceSquareToPoint(point) <= distance_sqr) {
      GetAllObjects(result_objects);
      return;
    }
    const double pvalue = point[partition_];
    const double partition_position = ComputePartitionPosition();
    if (pvalue < partition_position) {
      const double limit = pvalue + distance;
      for (int i = 0; i < num_objects_; ++i) {
        if (objects_info_->objects_sorted_by_min_bound()[i] > limit) {
          break;
        }
        ObjectPtr object = objects_info_->objects_sorted_by_min()[i];
        if (object->DistanceSquareTo(point) <= distance_sqr) {
          result_objects->push_back(object);
        }
      }
    } else {
      const double limit = pvalue - distance;
      for (int i = 0; i < num_objects_; ++i) {
        if (objects_info_->objects_sorted_by_max_bound()[i] < limit) {
          break;
        }
        ObjectPtr object = objects_info_->objects_sorted_by_max()[i];
        if (object->DistanceSquareTo(point) <= distance_sqr) {
          result_objects->push_back(object);
        }
      }
    }
    for (const auto& node : subnodes_) {
      if (node != nullptr) {
        node->GetObjectsInternal(point, distance, distance_sqr, result_objects);
      }
    }
  }

  void GetObjectsInternalWithinAABox(
      const AABox2d& aabox,
      const std::function<bool(const ObjectPtr, const AABox2d&)>&
          is_aabox_contain_object,
      std::vector<ObjectPtr>* const result_objects) const {
    CHECK_NOTNULL(result_objects);
    if (aabox.min_x() > max_x_ || aabox.max_x() < min_x_ ||
        aabox.max_y() < min_y_ || aabox.min_y() > max_y_) {
      return;
    }

    if (partition_ == PARTITION_X) {
      for (int i = 0; i < num_objects_; ++i) {
        if (objects_info_->objects_sorted_by_max_bound()[i] < aabox.min_x()) {
          break;
        }
        ObjectPtr object = objects_info_->objects_sorted_by_max()[i];
        if (is_aabox_contain_object(object, aabox)) {
          result_objects->push_back(object);
        }
      }
    } else if (partition_ == PARTITION_Y) {
      for (int i = 0; i < num_objects_; ++i) {
        if (objects_info_->objects_sorted_by_max_bound()[i] < aabox.min_y()) {
          break;
        }
        ObjectPtr object = objects_info_->objects_sorted_by_max()[i];
        if (is_aabox_contain_object(object, aabox)) {
          result_objects->push_back(object);
        }
      }
    }
    for (const auto& node : subnodes_) {
      if (node != nullptr) {
        node->GetObjectsInternalWithinAABox(aabox, is_aabox_contain_object,
                                            result_objects);
      }
    }
  }

  template <bool kSearchLeft>
  void GetNearestObjectFromSelfObjects(const Vec2d& point,
                                       double* const min_distance_sqr,
                                       ObjectPtr* const nearest_object) const {
    double& min_dist2 = *min_distance_sqr;
    const double pvalue = point[partition_];
    for (int i = 0; i < num_objects_; ++i) {
      const double bound =
          kSearchLeft ? objects_info_->objects_sorted_by_min_bound()[i]
                      : objects_info_->objects_sorted_by_max_bound()[i];
      if ((kSearchLeft ? bound > pvalue : bound < pvalue) &&
          Sqr(bound - pvalue) > min_dist2) {
        break;
      }
      ObjectPtr object = kSearchLeft
                             ? objects_info_->objects_sorted_by_min()[i]
                             : objects_info_->objects_sorted_by_max()[i];
      const double distance_sqr = object->DistanceSquareTo(point);
      if (distance_sqr < min_dist2) {
        min_dist2 = distance_sqr;
        *nearest_object = object;
      }
    }
  }

  double GetKNearestObjectsFromSelfObjects(
      const int want_k, const Vec2d& point, bool on_left,
      PriorityQueue* priority_queue) const {
    double min_dist2_upper_bound = priority_queue->empty()
                                       ? std::numeric_limits<double>::infinity()
                                       : priority_queue->top().second;

    const double pvalue = point[partition_];
    for (int i = 0; i < num_objects_; ++i) {
      const double bound =
          on_left ? objects_info_->objects_sorted_by_min_bound()[i]
                  : objects_info_->objects_sorted_by_max_bound()[i];
      if ((on_left ? bound > pvalue : bound < pvalue) &&
          Sqr(bound - pvalue) > min_dist2_upper_bound) {
        break;
      }
      ObjectPtr object = on_left ? objects_info_->objects_sorted_by_min()[i]
                                 : objects_info_->objects_sorted_by_max()[i];
      const double distance_sqr = object->DistanceSquareTo(point);
      if (static_cast<int>(priority_queue->size()) < want_k ||
          distance_sqr < min_dist2_upper_bound) {
        if (static_cast<int>(priority_queue->size()) >= want_k)
          priority_queue->pop();
        priority_queue->push(std::make_pair(object, distance_sqr));
        min_dist2_upper_bound = priority_queue->top().second;
      }
    }
    return min_dist2_upper_bound;
  }

  void GetNearestObjectInternal(const Vec2d& point,
                                double* const min_distance_sqr,
                                ObjectPtr* const nearest_object) const {
    double& min_dist2 = *min_distance_sqr;

    std::vector<std::pair<const AABoxKDTree2dNode*, bool>> node_stack;
    node_stack.reserve(64);
    node_stack.emplace_back(this, true);
    while (!node_stack.empty()) {
      const auto [node, search_subtree] = node_stack.back();
      node_stack.pop_back();

      if (node == nullptr ||
          (search_subtree &&
           node->LowerDistanceSquareToPoint(point) >= min_dist2)) {
        continue;
      }

      const bool subnode_index =
          point[node->partition_] < node->ComputePartitionPosition() ? 0 : 1;
      if (search_subtree) {
        node_stack.emplace_back(node->subnodes_[1 - subnode_index].get(), true);
        node_stack.emplace_back(node, false);
        node_stack.emplace_back(node->subnodes_[subnode_index].get(), true);
      } else {
        if (subnode_index == 0) {
          node->template GetNearestObjectFromSelfObjects<true>(
              point, &min_dist2, nearest_object);
        } else {
          node->template GetNearestObjectFromSelfObjects<false>(
              point, &min_dist2, nearest_object);
        }
        if (min_dist2 <= kMathEpsilon) {
          break;
        }
      }
    }
  }

  PriorityQueue GetKNearestObjectsInternal(const int want_k,
                                           const Vec2d& point) const {
    PriorityQueue priority_queue;

    std::vector<std::pair<const AABoxKDTree2dNode*, bool>> node_stack;

    constexpr int kLargeDepth = 64;
    node_stack.reserve(kLargeDepth);
    node_stack.emplace_back(this, true);
    double min_dist2_upper_bound = std::numeric_limits<double>::infinity();
    while (!node_stack.empty()) {
      const auto [node, search_subtree] = node_stack.back();
      node_stack.pop_back();

      if (node == nullptr ||
          (search_subtree &&
           node->LowerDistanceSquareToPoint(point) >= min_dist2_upper_bound)) {
        continue;
      }

      if (search_subtree) {
        for (const auto& node : subnodes_) {
          node_stack.emplace_back(node.get(), true);
        }
        node_stack.emplace_back(node, false);
      } else {
        const bool is_left =
            point[node->partition_] < node->ComputePartitionPosition() ? true
                                                                       : false;
        min_dist2_upper_bound = node->GetKNearestObjectsFromSelfObjects(
            want_k, point, is_left, &priority_queue);
        if (min_dist2_upper_bound <= kMathEpsilon) {
          break;
        }
      }
    }
    return priority_queue;
  }

  void ComputeBoundary(const std::vector<ObjectPtr>& objects) {
    auto aabox = objects[0]->ComputeAABox();
    double min_x = aabox.min_x();
    double min_y = aabox.min_y();
    double max_x = aabox.max_x();
    double max_y = aabox.max_y();
    for (int i = 1; i < objects.size(); ++i) {
      aabox = objects[i]->ComputeAABox();
      min_x = std::fmin(min_x, aabox.min_x());
      max_x = std::fmax(max_x, aabox.max_x());
      min_y = std::fmin(min_y, aabox.min_y());
      max_y = std::fmax(max_y, aabox.max_y());
    }
    min_x_ = min_x;
    min_y_ = min_y;
    max_x_ = max_x;
    max_y_ = max_y;
  }

  void ComputePartition() {
    if (max_x_ - min_x_ >= max_y_ - min_y_) {
      partition_ = PARTITION_X;
    } else {
      partition_ = PARTITION_Y;
    }
  }

  double ComputePartitionPosition() const {
    return partition_ == PARTITION_X ? (min_x_ + max_x_) * 0.5
                                     : (min_y_ + max_y_) * 0.5;
  }

  void PartitionObjects(std::vector<ObjectPtr> objects,
                        std::vector<ObjectPtr>* const left_subnode_objects,
                        std::vector<ObjectPtr>* const right_subnode_objects) {
    DCHECK(CHECK_NOTNULL(left_subnode_objects)->empty());
    DCHECK(CHECK_NOTNULL(right_subnode_objects)->empty());
    left_subnode_objects->reserve(objects.size());
    right_subnode_objects->reserve(objects.size());
    int num_other_objects = 0;
    const double partition_position = ComputePartitionPosition();
    if (partition_ == PARTITION_X) {
      for (ObjectPtr object : objects) {
        const auto aabox = object->ComputeAABox();
        if (aabox.max_x() <= partition_position) {
          left_subnode_objects->push_back(object);
        } else if (aabox.min_x() >= partition_position) {
          right_subnode_objects->push_back(object);
        } else {
          objects[num_other_objects++] = object;
        }
      }
    } else {
      for (ObjectPtr object : objects) {
        const auto aabox = object->ComputeAABox();
        if (aabox.max_y() <= partition_position) {
          left_subnode_objects->push_back(object);
        } else if (aabox.min_y() >= partition_position) {
          right_subnode_objects->push_back(object);
        } else {
          objects[num_other_objects++] = object;
        }
      }
    }
    objects.resize(num_other_objects);
    InitObjects(std::move(objects));
  }

 private:
  int num_objects_ = 0;
  uint16_t depth_ = 0;
  enum Partition : int8_t {
    PARTITION_X = 0,
    PARTITION_Y = 1,
  };
  Partition partition_ = PARTITION_X;

  std::unique_ptr<ObjectsInfo> objects_info_;

  double min_x_ = 0.0;
  double max_x_ = 0.0;
  double min_y_ = 0.0;
  double max_y_ = 0.0;

  std::array<std::unique_ptr<AABoxKDTree2dNode<ObjectType>>, 2> subnodes_;
};
static_assert(sizeof(AABoxKDTree2dNode<void>) == 64);

template <
    class ObjectType,
    typename std::enable_if<std::is_member_function_pointer<
                                decltype(&ObjectType::ComputeAABox)>{} &&
                                std::is_member_function_pointer<
                                    decltype(&ObjectType::DistanceSquareTo)>{},
                            bool>::type = true>
class AABoxKDTree2d {
 public:
  using ObjectPtr = const ObjectType*;

  AABoxKDTree2d(absl::Span<const ObjectType> objects,
                const AABoxKDTreeParams& params) {
    if (!objects.empty()) {
      std::vector<ObjectPtr> object_ptrs;
      object_ptrs.reserve(objects.size());
      for (const auto& object : objects) {
        object_ptrs.push_back(&object);
      }
      root_ = std::make_unique<AABoxKDTree2dNode<ObjectType>>(
          std::move(object_ptrs), params, 0);
    }
  }

  ObjectPtr GetNearestObject(const Vec2d& point) const {
    return root_ == nullptr ? nullptr : root_->GetNearestObject(point);
  }

  std::vector<ObjectPtr> GetKNearestObjects(int want_k,
                                            const Vec2d& point) const {
    return root_ == nullptr ? std::vector<ObjectPtr>{}
                            : root_->GetKNearestObjects(want_k, point);
  }

  std::vector<ObjectPtr> GetObjects(const Vec2d& point,
                                    const double distance) const {
    if (root_ == nullptr) {
      return {};
    }
    return root_->GetObjects(point, distance);
  }

  std::vector<ObjectPtr> GetObjectsWithinAABox(
      const AABox2d& aabox,
      const std::function<bool(const ObjectPtr, const AABox2d&)>&
          is_aabox_contain_object) const {
    if (root_ == nullptr) {
      return {};
    }
    return root_->GetObjectsWithinAABox(aabox, is_aabox_contain_object);
  }

  AABox2d GetBoundingBox() const {
    return root_ == nullptr ? AABox2d() : root_->GetBoundingBox();
  }

 private:
  std::unique_ptr<AABoxKDTree2dNode<ObjectType>> root_;
};

}  // namespace e2e_noa

#endif
