#pragma once

#define _USE_MATH_DEFINES

#include <Eigen/Dense>
#include <Eigen/Core>
#include <vector>
#include <set>

namespace ShortestPath
{
    double euclidDistance(const Eigen::Vector3d& x, const Eigen::Vector3d& y);
    double arcDistance(const Eigen::Vector3d& x, const Eigen::Vector3d& y);
    std::vector<std::set<int>> init(const std::string& fileName);
    double calcOverlap(const std::set<int>& A, const std::set<int>& B);
    std::vector<Eigen::Vector3d> getView(const Eigen::Vector3d& camera_normal, int horizontal_count, int vertical_count);
    void execute(const std::string& fileName);
}
