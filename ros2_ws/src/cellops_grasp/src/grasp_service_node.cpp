// GraspSight-as-a-service: rclcpp node exposing the cube pose estimator over
// ROS 2 — CellOps (WorkCell Part E), Artifact 3.
//
// Request:  segmented object cloud (PointCloud2, world frame) + table plane z.
// Response: top-down grasp pose at the cube centre (yaw about the vertical,
//           identifiable mod 90 deg — the parallel-jaw face-pair symmetry) +
//           fit diagnostics (RMSE, iterations, solve time).
#include <chrono>
#include <cmath>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "cellops_grasp/cube_pose.hpp"
#include "cellops_interfaces/srv/estimate_cube_pose.hpp"

using EstimateCubePose = cellops_interfaces::srv::EstimateCubePose;

namespace {

cellops::Cloud from_msg(const sensor_msgs::msg::PointCloud2 &msg) {
  const size_t n = static_cast<size_t>(msg.width) * msg.height;
  cellops::Cloud pts(static_cast<Eigen::Index>(n), 3);
  sensor_msgs::PointCloud2ConstIterator<float> ix(msg, "x"), iy(msg, "y"), iz(msg, "z");
  for (Eigen::Index i = 0; ix != ix.end(); ++ix, ++iy, ++iz, ++i) {
    pts(i, 0) = static_cast<double>(*ix);
    pts(i, 1) = static_cast<double>(*iy);
    pts(i, 2) = static_cast<double>(*iz);
  }
  return pts;
}

}  // namespace

class GraspServiceNode : public rclcpp::Node {
public:
  GraspServiceNode() : Node("grasp_pose_service") {
    service_ = create_service<EstimateCubePose>(
        "estimate_cube_pose",
        [this](const std::shared_ptr<EstimateCubePose::Request> req,
               std::shared_ptr<EstimateCubePose::Response> res) {
          handle(req, res);
        });
    RCLCPP_INFO(get_logger(),
                "grasp_pose_service ready (table-constrained ICP, from-scratch KD-tree)");
  }

private:
  void handle(const std::shared_ptr<EstimateCubePose::Request> &req,
              std::shared_ptr<EstimateCubePose::Response> &res) {
    const cellops::Cloud cloud = from_msg(req->cloud);
    if (cloud.rows() < 30) {
      RCLCPP_WARN(get_logger(), "rejecting request: only %ld points", cloud.rows());
      res->rmse = -1.0;
      return;
    }
    const double half = req->half_extent > 0.0 ? req->half_extent : 0.012;

    const auto t0 = std::chrono::steady_clock::now();
    const cellops::CubePose pose =
        cellops::estimate_cube_pose(cloud, req->table_z, half);
    const auto t1 = std::chrono::steady_clock::now();
    const double ms =
        std::chrono::duration<double, std::milli>(t1 - t0).count();

    res->grasp_pose.header.stamp = now();
    res->grasp_pose.header.frame_id =
        req->cloud.header.frame_id.empty() ? "world" : req->cloud.header.frame_id;
    res->grasp_pose.pose.position.x = pose.position.x();
    res->grasp_pose.pose.position.y = pose.position.y();
    res->grasp_pose.pose.position.z = pose.position.z();
    // Top-down grasp: orientation is yaw about the vertical axis.
    res->grasp_pose.pose.orientation.z = std::sin(pose.yaw / 2.0);
    res->grasp_pose.pose.orientation.w = std::cos(pose.yaw / 2.0);
    res->yaw = pose.yaw;
    res->rmse = pose.rmse;
    res->iters = pose.n_iters;
    res->solve_ms = ms;

    RCLCPP_INFO(get_logger(),
                "solved %ld pts: xy=(%.4f, %.4f) yaw=%.1f deg rmse=%.2f mm "
                "iters=%d in %.2f ms",
                cloud.rows(), pose.position.x(), pose.position.y(),
                pose.yaw * 180.0 / M_PI, pose.rmse * 1000.0, pose.n_iters, ms);
  }

  rclcpp::Service<EstimateCubePose>::SharedPtr service_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GraspServiceNode>());
  rclcpp::shutdown();
  return 0;
}
