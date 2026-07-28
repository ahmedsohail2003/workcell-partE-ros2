// From-scratch static 3-D KD-tree — CellOps (WorkCell Part E).
//
// The Python reference (graspsight/pose.py) leans on scipy's C-backed cKDTree,
// so an honest C++-vs-Python benchmark needs a real KD-tree on this side too —
// written from scratch (median split on the widest axis, branch-and-bound
// nearest-neighbour) to keep the WorkCell series' from-scratch through-line
// and avoid dragging in PCL for one data structure.
//
// v2, after profiling the naive port (recursive search over Eigen row views,
// ~2.8x vs Python): points are copied into node order (one contiguous array,
// cache-line friendly) and the search is iterative with an explicit stack —
// no recursion, no Eigen temporaries in the hot path.
#pragma once

#include <Eigen/Core>
#include <algorithm>
#include <array>
#include <cstddef>
#include <limits>
#include <numeric>
#include <vector>

namespace cellops {

class KdTree3 {
public:
  explicit KdTree3(const Eigen::Matrix<double, Eigen::Dynamic, 3> &pts) {
    const int n = static_cast<int>(pts.rows());
    std::vector<int> order(static_cast<size_t>(n));
    std::iota(order.begin(), order.end(), 0);
    nodes_.reserve(static_cast<size_t>(n));
    root_ = build(pts, order, 0, n);
  }

  struct Hit {
    int index = -1;   // row in the original point matrix
    double dist2 = std::numeric_limits<double>::infinity();
  };

  Hit nearest(const double q[3]) const {
    Hit best;
    // Explicit stack of (node, squared hyperplane distance at its parent).
    // 64 levels is far beyond any balanced tree we build here.
    struct Entry { int node; double delta2; };
    std::array<Entry, 64> stack;
    int top = 0;
    stack[top++] = {root_, 0.0};

    while (top > 0) {
      const Entry e = stack[--top];
      if (e.node < 0 || e.delta2 >= best.dist2) continue;
      const Node &nd = nodes_[static_cast<size_t>(e.node)];

      const double dx = q[0] - nd.p[0];
      const double dy = q[1] - nd.p[1];
      const double dz = q[2] - nd.p[2];
      const double d2 = dx * dx + dy * dy + dz * dz;
      if (d2 < best.dist2) best = {nd.orig, d2};

      const double delta = q[nd.axis] - nd.p[nd.axis];
      const int near = delta < 0.0 ? nd.left : nd.right;
      const int far = delta < 0.0 ? nd.right : nd.left;
      if (far >= 0) stack[top++] = {far, delta * delta};   // pruned on pop
      if (near >= 0) stack[top++] = {near, 0.0};
    }
    return best;
  }

  Hit nearest(const Eigen::Vector3d &q) const {
    const double a[3] = {q.x(), q.y(), q.z()};
    return nearest(a);
  }

private:
  struct Node {
    double p[3];
    int orig;         // original row index
    int left = -1, right = -1;
    int axis = 0;
  };

  int build(const Eigen::Matrix<double, Eigen::Dynamic, 3> &pts,
            std::vector<int> &order, int lo, int hi) {
    if (lo >= hi) return -1;
    double mn[3] = {1e300, 1e300, 1e300}, mx[3] = {-1e300, -1e300, -1e300};
    for (int i = lo; i < hi; ++i) {
      for (int a = 0; a < 3; ++a) {
        const double v = pts(order[static_cast<size_t>(i)], a);
        mn[a] = std::min(mn[a], v);
        mx[a] = std::max(mx[a], v);
      }
    }
    int axis = 0;
    double spread = mx[0] - mn[0];
    for (int a = 1; a < 3; ++a)
      if (mx[a] - mn[a] > spread) { spread = mx[a] - mn[a]; axis = a; }

    const int mid = lo + (hi - lo) / 2;
    std::nth_element(order.begin() + lo, order.begin() + mid, order.begin() + hi,
                     [&](int a, int b) { return pts(a, axis) < pts(b, axis); });

    const int me = static_cast<int>(nodes_.size());
    Node nd;
    const int oi = order[static_cast<size_t>(mid)];
    nd.p[0] = pts(oi, 0); nd.p[1] = pts(oi, 1); nd.p[2] = pts(oi, 2);
    nd.orig = oi;
    nd.axis = axis;
    nodes_.push_back(nd);
    const int l = build(pts, order, lo, mid);
    const int r = build(pts, order, mid + 1, hi);
    nodes_[static_cast<size_t>(me)].left = l;
    nodes_[static_cast<size_t>(me)].right = r;
    return me;
  }

  std::vector<Node> nodes_;
  int root_ = -1;
};

}  // namespace cellops
