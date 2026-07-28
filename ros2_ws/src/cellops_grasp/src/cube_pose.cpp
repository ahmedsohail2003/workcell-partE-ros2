// C++ port of graspsight/pose.py (WorkCell Part C) — see cube_pose.hpp.
//
// Porting notes, kept faithful for numerical parity with the Python reference:
//  * cube_surface_model replicates numpy arange (start + i*step, < half+1e-9),
//    including its asymmetric endpoint behaviour.
//  * the init sweep cost is the mean of the smallest int(0.6*n) distances —
//    int() truncation, exactly like np.sort(d)[: int(0.6*len(d))].
//  * the ICP trim threshold replicates np.quantile(_, 0.9) linear interpolation.
//  * yaw wrapping replicates numpy's non-negative % (fmod can go negative).
#include "cellops_grasp/cube_pose.hpp"

#include <Eigen/Dense>
#include <Eigen/SVD>
#include <algorithm>
#include <cmath>
#include <vector>

#include "cellops_grasp/kdtree.hpp"

namespace cellops {

namespace {

// Best-fit 2-D rotation R and translation t with R*p + t ~= q (SVD Kabsch).
void kabsch_2d(const Eigen::MatrixX2d &P, const Eigen::MatrixX2d &Q,
               Eigen::Matrix2d &R, Eigen::Vector2d &t) {
  const Eigen::RowVector2d cp = P.colwise().mean();
  const Eigen::RowVector2d cq = Q.colwise().mean();
  const Eigen::Matrix2d H =
      (P.rowwise() - cp).transpose() * (Q.rowwise() - cq);
  Eigen::JacobiSVD<Eigen::Matrix2d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
  const Eigen::Matrix2d U = svd.matrixU();
  const Eigen::Matrix2d V = svd.matrixV();
  const double d = ((V * U.transpose()).determinant() < 0.0) ? -1.0 : 1.0;
  R = V * Eigen::Vector2d(1.0, d).asDiagonal() * U.transpose();
  t = cq.transpose() - R * cp.transpose();
}

// numpy's non-negative modulo.
inline double pymod(double x, double p) {
  double m = std::fmod(x, p);
  return m < 0.0 ? m + p : m;
}

// np.quantile(v, q) with the default linear interpolation.
double quantile(std::vector<double> v, double q) {
  std::sort(v.begin(), v.end());
  const double h = q * static_cast<double>(v.size() - 1);
  const size_t lo = static_cast<size_t>(std::floor(h));
  const size_t hi = std::min(lo + 1, v.size() - 1);
  return v[lo] + (h - static_cast<double>(lo)) * (v[hi] - v[lo]);
}

}  // namespace

Cloud cube_surface_model(double half, double grid) {
  // Replicate np.arange(-half, half + 1e-9, grid): value_i = -half + i*grid.
  std::vector<double> ax;
  for (int i = 0;; ++i) {
    const double v = -half + i * grid;
    if (v >= half + 1e-9) break;
    ax.push_back(v);
  }
  const int n = static_cast<int>(ax.size());
  Cloud model(5 * n * n, 3);
  int row = 0;
  // Face order matches pose.py: top(+z), +y, -y, +x, -x — with (u, v) from
  // meshgrid(indexing="ij") raveled the same way.
  for (int face = 0; face < 5; ++face) {
    for (int i = 0; i < n; ++i) {
      for (int j = 0; j < n; ++j) {
        const double u = ax[i], v = ax[j], h = half;
        switch (face) {
          case 0: model.row(row++) << u, v, h; break;    // top
          case 1: model.row(row++) << u, h, v; break;    // +y side
          case 2: model.row(row++) << u, -h, v; break;   // -y side
          case 3: model.row(row++) << h, u, v; break;    // +x side
          case 4: model.row(row++) << -h, u, v; break;   // -x side
        }
      }
    }
  }
  return model;
}

CubePose estimate_cube_pose(const Cloud &cluster_points, double table_z,
                            double half, int max_iters, double tol) {
  const Cloud model = cube_surface_model(half);
  const double z_center = table_z + half;
  const Eigen::Index n_obs = cluster_points.rows();
  const Eigen::Index n_model = model.rows();

  // ---- init: coarse global sweep over the symmetry-reduced yaw range.
  Eigen::Vector2d t_xy = cluster_points.leftCols<2>().colwise().mean();
  const KdTree3 obs_tree(cluster_points);

  double best_yaw = 0.0, best_cost = std::numeric_limits<double>::infinity();
  const int k_trim = static_cast<int>(0.6 * static_cast<double>(n_model));
  std::vector<double> d(static_cast<size_t>(n_model));
  for (double deg = 0.0; deg < 90.0; deg += 5.0) {
    const double cand = deg * M_PI / 180.0;
    const double cy = std::cos(cand), sy = std::sin(cand);
    for (Eigen::Index i = 0; i < n_model; ++i) {
      const double mx = model(i, 0), my = model(i, 1);
      const Eigen::Vector3d w(cy * mx - sy * my + t_xy.x(),
                              sy * mx + cy * my + t_xy.y(),
                              model(i, 2) + z_center);
      d[static_cast<size_t>(i)] = std::sqrt(obs_tree.nearest(w).dist2);
    }
    // model->observed distances, trimmed: unobserved back faces shouldn't
    // dominate the score
    std::sort(d.begin(), d.end());
    double cost = 0.0;
    for (int i = 0; i < k_trim; ++i) cost += d[static_cast<size_t>(i)];
    cost /= static_cast<double>(k_trim);
    if (cost < best_cost) {
      best_cost = cost;
      best_yaw = cand;
    }
  }
  double yaw = best_yaw;

  // ---- trimmed ICP with closed-form 2-D Kabsch increments.
  //
  // Optimization over the naive port (which transformed the model into world
  // frame and rebuilt its KD-tree EVERY iteration): rigid transforms preserve
  // distances, so "observed -> nearest transformed-model point" equals
  // "inverse-transformed observed -> nearest model point". Build ONE static
  // tree over the local-frame model and inverse-transform the observations
  // instead — same correspondences, zero per-iteration tree builds.
  const KdTree3 model_tree(model);
  double prev_rmse = std::numeric_limits<double>::infinity();
  double rmse = 0.0;
  int n_done = 0;
  std::vector<double> dist(static_cast<size_t>(n_obs));
  std::vector<int> nn_idx(static_cast<size_t>(n_obs));

  for (int it = 0; it < max_iters; ++it) {
    n_done = it + 1;
    const double cy = std::cos(yaw), sy = std::sin(yaw);

    // correspondences: each observed point -> nearest model point, computed in
    // the model's local frame (inverse rigid transform of the observation)
    double sq_sum = 0.0;
    for (Eigen::Index i = 0; i < n_obs; ++i) {
      const double px = cluster_points(i, 0) - t_xy.x();
      const double py = cluster_points(i, 1) - t_xy.y();
      const double q[3] = {cy * px + sy * py,       // R(-yaw) * (p - t)
                           -sy * px + cy * py,
                           cluster_points(i, 2) - z_center};
      const auto hit = model_tree.nearest(q);
      dist[static_cast<size_t>(i)] = std::sqrt(hit.dist2);
      nn_idx[static_cast<size_t>(i)] = hit.index;
      sq_sum += hit.dist2;
    }
    rmse = std::sqrt(sq_sum / static_cast<double>(n_obs));

    // trim the worst 10% (occlusion boundaries, depth-edge artifacts)
    const double thresh = quantile(dist, 0.9);
    Eigen::Index n_keep = 0;
    for (Eigen::Index i = 0; i < n_obs; ++i)
      if (dist[static_cast<size_t>(i)] <= thresh) ++n_keep;

    // matched model points, transformed to world xy only for the kept subset
    Eigen::MatrixX2d src(n_keep, 2), dst(n_keep, 2);
    Eigen::Index k = 0;
    for (Eigen::Index i = 0; i < n_obs; ++i) {
      if (dist[static_cast<size_t>(i)] > thresh) continue;
      const int mi = nn_idx[static_cast<size_t>(i)];
      const double mx = model(mi, 0), my = model(mi, 1);
      src(k, 0) = cy * mx - sy * my + t_xy.x();
      src(k, 1) = sy * mx + cy * my + t_xy.y();
      dst(k, 0) = cluster_points(i, 0);
      dst(k, 1) = cluster_points(i, 1);
      ++k;
    }

    Eigen::Matrix2d R_inc;
    Eigen::Vector2d t_inc;
    kabsch_2d(src, dst, R_inc, t_inc);
    yaw = pymod(yaw + std::atan2(R_inc(1, 0), R_inc(0, 0)), M_PI / 2.0);
    t_xy = R_inc * t_xy + t_inc;

    if (std::abs(prev_rmse - rmse) < tol) break;
    prev_rmse = rmse;
  }

  CubePose out;
  out.position = Eigen::Vector3d(t_xy.x(), t_xy.y(), z_center);
  out.yaw = yaw;
  out.rmse = rmse;
  out.n_iters = n_done;
  return out;
}

}  // namespace cellops
