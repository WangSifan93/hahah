#ifndef MATH_GEOMETRY_KDTREE_H_
#define MATH_GEOMETRY_KDTREE_H_

#include <cstddef>
#include <memory>
#include <set>
#include <vector>

#include "Eigen/Core"
#include "math/point_types.h"
#include "math/vec.h"

namespace e2e_noa {

template <typename PointType>
class KDTree {
 public:
  using ScalarType = typename PointType::Scalar;

  KDTree() = default;

  explicit KDTree(const std::vector<PointType>& points);

  KDTree(const std::vector<PointType>& points,
         const std::vector<int>& valid_indexes);

  const PointType& FindNearest(const PointType& point,
                               ScalarType* best_dist_sqr = nullptr) const;

  std::set<int> FindKNearest(const PointType& point, int k) const;

  std::set<int> FindNearestInRadius(const PointType& point,
                                    const ScalarType max_dist_sqr) const;

  const PointType& PointAt(int index) const { return nodes_[index].point; }

  int GetOriginalIndex(int tree_index) const {
    return nodes_[tree_index].original_index;
  }

  std::set<int> GetOriginalIndices(const std::set<int>& tree_indices) const;

  size_t size() const { return nodes_.size(); }

 private:
  struct TreeNode {
    PointType point;
    int left;
    int right;
    int original_index;
  };

  using NodeIter = typename std::vector<TreeNode>::iterator;
  static constexpr int kDim = PointType::RowsAtCompileTime;

  int FindMedian(int begin, int end, int dim);

  int MakeTree(int begin, int end, int dim);

  template <bool kIgnoreIndicesInSet>
  void FindNearest(const PointType& point, int node_index, int dim,
                   const std::set<int>& to_ignore, int* best,
                   ScalarType* best_dist_sqr) const;

  int root_index_ = -1;

  std::vector<TreeNode> nodes_;
};

}  // namespace e2e_noa

#endif
