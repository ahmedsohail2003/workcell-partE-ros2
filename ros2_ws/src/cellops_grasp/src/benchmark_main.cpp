// Standalone benchmark for the C++ cube-pose estimator — no ROS required.
// Reads cloud_*.csv (x,y,z per line) from a directory plus table_z.csv
// ("cloud_XX.csv,<table_z>" per line — written by make_testdata.py so both
// implementations see the identical per-cloud table plane), solves each cloud
// `reps` times, and emits one JSON line per cloud with the median solve time
// and the pose result. parity_report.py joins this against the Python
// reference and the ground truth.
//
// Usage: benchmark <clouds_dir> [reps=20]
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "cellops_grasp/cube_pose.hpp"

namespace fs = std::filesystem;

static cellops::Cloud read_csv(const fs::path &p) {
  std::ifstream f(p);
  std::vector<double> vals;
  std::string line;
  while (std::getline(f, line)) {
    if (line.empty()) continue;
    std::stringstream ss(line);
    std::string cell;
    while (std::getline(ss, cell, ',')) vals.push_back(std::stod(cell));
  }
  const Eigen::Index n = static_cast<Eigen::Index>(vals.size() / 3);
  cellops::Cloud pts(n, 3);
  for (Eigen::Index i = 0; i < n; ++i) {
    pts(i, 0) = vals[static_cast<size_t>(3 * i)];
    pts(i, 1) = vals[static_cast<size_t>(3 * i + 1)];
    pts(i, 2) = vals[static_cast<size_t>(3 * i + 2)];
  }
  return pts;
}

static std::map<std::string, double> read_table_z(const fs::path &p) {
  std::map<std::string, double> out;
  std::ifstream f(p);
  std::string line;
  while (std::getline(f, line)) {
    const auto comma = line.find(',');
    if (comma == std::string::npos) continue;
    out[line.substr(0, comma)] = std::stod(line.substr(comma + 1));
  }
  return out;
}

int main(int argc, char **argv) {
  if (argc < 2) {
    std::fprintf(stderr, "usage: %s <clouds_dir> [reps]\n", argv[0]);
    return 2;
  }
  const fs::path dir(argv[1]);
  const int reps = argc > 2 ? std::stoi(argv[2]) : 20;
  const auto table_zs = read_table_z(dir / "table_z.csv");
  if (table_zs.empty()) {
    std::fprintf(stderr, "missing/empty %s\n", (dir / "table_z.csv").string().c_str());
    return 1;
  }

  std::vector<fs::path> files;
  for (const auto &e : fs::directory_iterator(dir)) {
    const std::string name = e.path().filename().string();
    if (name.rfind("cloud_", 0) == 0 && e.path().extension() == ".csv")
      files.push_back(e.path());
  }
  std::sort(files.begin(), files.end());
  if (files.empty()) {
    std::fprintf(stderr, "no cloud_*.csv in %s\n", dir.string().c_str());
    return 1;
  }

  for (const auto &f : files) {
    const auto tz = table_zs.find(f.filename().string());
    if (tz == table_zs.end()) {
      std::fprintf(stderr, "no table_z for %s, skipping\n",
                   f.filename().string().c_str());
      continue;
    }
    const cellops::Cloud cloud = read_csv(f);
    std::vector<double> ms(static_cast<size_t>(reps));
    cellops::CubePose pose;
    for (int r = 0; r < reps; ++r) {
      const auto t0 = std::chrono::steady_clock::now();
      pose = cellops::estimate_cube_pose(cloud, tz->second);
      const auto t1 = std::chrono::steady_clock::now();
      ms[static_cast<size_t>(r)] =
          std::chrono::duration<double, std::milli>(t1 - t0).count();
    }
    std::sort(ms.begin(), ms.end());
    const double med = ms[ms.size() / 2];
    std::printf(
        "{\"cloud\": \"%s\", \"n_points\": %ld, \"x\": %.6f, \"y\": %.6f, "
        "\"yaw\": %.6f, \"rmse\": %.6f, \"iters\": %d, \"median_ms\": %.3f}\n",
        f.filename().string().c_str(), cloud.rows(), pose.position.x(),
        pose.position.y(), pose.yaw, pose.rmse, pose.n_iters, med);
  }
  return 0;
}
