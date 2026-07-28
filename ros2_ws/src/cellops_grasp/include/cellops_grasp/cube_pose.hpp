// Table-constrained cube pose estimation — C++ port of graspsight/pose.py.
// CellOps (WorkCell Part E), Artifact 3 of the WorkCell series.
#pragma once

#include <Eigen/Core>

namespace cellops {

struct CubePose {
  Eigen::Vector3d position;  // world xyz of the cube centre
  double yaw = 0.0;          // radians, identifiable mod pi/2
  double rmse = 0.0;         // final ICP residual [m]
  int n_iters = 0;
};

using Cloud = Eigen::Matrix<double, Eigen::Dynamic, 3>;

// Points sampled on the cube's camera-visible surface (top + 4 sides), local frame.
Cloud cube_surface_model(double half, double grid = 0.0025);

// PCA-free coarse yaw sweep + trimmed 3-DoF ICP with closed-form 2-D Kabsch
// increments. Semantics match graspsight/pose.py estimate_cube_pose exactly
// (same trim quantile, same convergence rule) so results are comparable
// point-for-point with the Python reference.
CubePose estimate_cube_pose(const Cloud &cluster_points, double table_z,
                            double half = 0.012, int max_iters = 40,
                            double tol = 1e-5);

}  // namespace cellops
