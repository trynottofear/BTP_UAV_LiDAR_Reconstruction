// Shared helpers for the OctoMap tools (dynamic_voxel_octomap, benchmark_octomap).
#pragma once

#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include <octomap/OcTree.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace btp {

// Loads sensor positions from a FAST-LIO state log (Log/mat_pre.txt or Log/mat_out.txt:
// col 0 = time, cols 1-3 = Euler angles, cols 4-6 = IMU position in camera_init) or from a
// plain "x y z" file (exactly 3 columns per line).
inline pcl::PointCloud<pcl::PointXYZ>::Ptr loadTrajectory(const std::string& path) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr traj(new pcl::PointCloud<pcl::PointXYZ>);
    std::ifstream in(path);
    std::string line;
    while (std::getline(in, line)) {
        std::istringstream ss(line);
        std::vector<double> v;
        double x;
        while (ss >> x) v.push_back(x);
        if (v.size() == 3) {
            traj->points.emplace_back(v[0], v[1], v[2]);
        } else if (v.size() >= 7) {
            traj->points.emplace_back(v[4], v[5], v[6]);
        }
    }
    traj->width = traj->points.size();
    traj->height = 1;
    return traj;
}

// Canonical key of the voxel at `depth` that contains `key` (clears the lower bits).
inline octomap::OcTreeKey keyAtDepth(const octomap::OcTreeKey& key, unsigned depth, unsigned tree_depth) {
    const unsigned shift = tree_depth - depth;
    const octomap::key_type mask = static_cast<octomap::key_type>(~((1u << shift) - 1u));
    return octomap::OcTreeKey(key[0] & mask, key[1] & mask, key[2] & mask);
}

// Edge length of a voxel at `depth`.
inline double voxelSize(double resolution, unsigned depth, unsigned tree_depth) {
    return resolution * std::ldexp(1.0, static_cast<int>(tree_depth - depth));
}

// Descends from the root to the deepest existing node containing `key`.
// Returns the leaf node (occupied or free) if one exists, else nullptr (unknown space).
// `depth` is set to the depth of that leaf, or to the depth of the first missing child.
inline const octomap::OcTreeNode* lookupLeaf(const octomap::OcTree& tree, const octomap::OcTreeKey& key,
                                             unsigned& depth) {
    const unsigned td = tree.getTreeDepth();
    const octomap::OcTreeNode* node = tree.getRoot();
    depth = 0;
    if (!node) return nullptr;
    while (depth < td) {
        if (!tree.nodeHasChildren(node)) return node;
        const unsigned pos = octomap::computeChildIdx(key, static_cast<int>(td - 1 - depth));
        if (!tree.nodeChildExists(node, pos)) {
            ++depth;
            return nullptr;
        }
        node = tree.getNodeChild(node, pos);
        ++depth;
    }
    return node;
}

}  // namespace btp
