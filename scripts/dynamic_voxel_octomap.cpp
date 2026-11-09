#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <chrono>
#include <algorithm>
#include <map>
#include <omp.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/search/kdtree.h>

#include <octomap/octomap.h>
#include <octomap/OcTree.h>

/**
 * @brief DynamicOcTree extends octomap::OcTree to support targeted depth insertion.
 * Enables variable-resolution voxelization based on range and geometric curvature.
 */
class DynamicOcTree : public octomap::OcTree {
public:
    DynamicOcTree(double resolution) : octomap::OcTree(resolution) {}

    octomap::OcTreeNode* updateNodeAtDepth(const octomap::OcTreeKey& key, unsigned int target_depth,
                                           float log_odds_update, bool lazy_eval = false) {
        if (target_depth > this->tree_depth) target_depth = this->tree_depth;
        if (this->root == NULL) {
            this->root = new octomap::OcTreeNode();
            this->tree_size++;
        }
        return updateNodeRecursAtDepth(this->root, false, key, 0, target_depth, log_odds_update, lazy_eval);
    }

    octomap::OcTreeNode* updateNodeAtDepth(const octomap::point3d& pt, unsigned int target_depth,
                                           bool occupied, bool lazy_eval = false) {
        octomap::OcTreeKey key;
        if (!coordToKeyChecked(pt, key)) return NULL;
        float log_odds = occupied ? this->prob_hit_log : this->prob_miss_log;
        return updateNodeAtDepth(key, target_depth, log_odds, lazy_eval);
    }

    /**
     * @brief Recursively traverses the octree to count leaf nodes grouped by their depth.
     */
    void getDepthHistogram(std::map<unsigned int, size_t>& histogram) const {
        histogram.clear();
        if (this->root) {
            countDepthRecurs(this->root, 0, histogram);
        }
    }

protected:
    void countDepthRecurs(const octomap::OcTreeNode* node, unsigned int depth,
                          std::map<unsigned int, size_t>& histogram) const {
        if (!node) return;
        if (!this->nodeHasChildren(node)) {
            histogram[depth]++;
        } else {
            for (unsigned int i = 0; i < 8; ++i) {
                if (this->nodeChildExists(node, i)) {
                    countDepthRecurs(this->getNodeChild(node, i), depth + 1, histogram);
                }
            }
        }
    }

    octomap::OcTreeNode* updateNodeRecursAtDepth(octomap::OcTreeNode* node, bool node_just_created,
                                                 const octomap::OcTreeKey& key,
                                                 unsigned int depth, unsigned int target_depth,
                                                 const float& log_odds_update, bool lazy_eval) {
        bool created_node = false;
        assert(node);

        if (depth < target_depth) {
            unsigned int pos = computeChildIdx(key, this->tree_depth - 1 - depth);
            if (!this->nodeChildExists(node, pos)) {
                if (!this->nodeHasChildren(node) && !node_just_created) {
                    this->expandNode(node);
                } else {
                    this->createNodeChild(node, pos);
                    created_node = true;
                }
            }

            if (lazy_eval) {
                return updateNodeRecursAtDepth(this->getNodeChild(node, pos), created_node, key,
                                               depth + 1, target_depth, log_odds_update, lazy_eval);
            } else {
                octomap::OcTreeNode* retval = updateNodeRecursAtDepth(this->getNodeChild(node, pos), created_node, key,
                                                                      depth + 1, target_depth, log_odds_update, lazy_eval);
                node->updateOccupancyChildren();
                return retval;
            }
        }

        // Target depth reached
        if (!this->nodeHasChildren(node)) {
            updateNodeLogOdds(node, log_odds_update);
        } else {
            node->updateOccupancyChildren();
        }
        return node;
    }
};

int main(int argc, char** argv) {
    if (argc < 3) {
        std::cout << "Usage: " << argv[0] << " <input.pcd> <output.bt> [base_res=0.05] [d_min=13] [d_max=16]" << std::endl;
        return 1;
    }

    std::string pcd_path = argv[1];
    std::string out_bt_path = argv[2];
    double base_res = (argc >= 4) ? std::stod(argv[3]) : 0.05;
    unsigned int d_min = (argc >= 5) ? std::stoul(argv[4]) : 13;
    unsigned int d_max = (argc >= 6) ? std::stoul(argv[5]) : 16;

    std::cout << "==========================================================" << std::endl;
    std::cout << "  DYNAMIC / ADAPTIVE VOXEL OCTOMAP GENERATOR" << std::endl;
    std::cout << "  Input PCD:         " << pcd_path << std::endl;
    std::cout << "  Output BT:         " << out_bt_path << std::endl;
    std::cout << "  Base Resolution:   " << base_res << " m (Leaf Depth " << d_max << ")" << std::endl;
    std::cout << "  Depth Range:       [" << d_min << " (coarse) to " << d_max << " (fine)]" << std::endl;
    std::cout << "==========================================================" << std::endl;

    // 1. Load PCD
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    auto t0 = std::chrono::high_resolution_clock::now();
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_path, *cloud) == -1) {
        std::cerr << "[ERROR] Could not load PCD: " << pcd_path << std::endl;
        return 1;
    }
    auto t1 = std::chrono::high_resolution_clock::now();
    std::cout << "[INFO] Loaded " << cloud->size() << " points in "
              << std::chrono::duration<double>(t1 - t0).count() << "s." << std::endl;

    // 2. Compute Surface Curvature via OpenMP Normal Estimation
    std::cout << "[INFO] Computing geometric surface variation & curvature (k=25)..." << std::endl;
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(cloud);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_search(new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setSearchMethod(tree_search);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    ne.setKSearch(25);
    ne.setNumberOfThreads(omp_get_max_threads());

    auto t2 = std::chrono::high_resolution_clock::now();
    ne.compute(*cloud_normals);
    auto t3 = std::chrono::high_resolution_clock::now();
    std::cout << "[INFO] Curvatures estimated in "
              << std::chrono::duration<double>(t3 - t2).count() << "s using "
              << omp_get_max_threads() << " threads." << std::endl;

    // 3. Initialize Dynamic OcTree
    DynamicOcTree tree(base_res);
    unsigned int tree_max_depth = tree.getTreeDepth();
    if (d_max > tree_max_depth) d_max = tree_max_depth;

    std::cout << "[INFO] OcTree initialized. Tree max depth: " << tree_max_depth << std::endl;

    // 4. Compute Target Depths & Populate Tree
    std::map<unsigned int, size_t> assigned_depth_counts;

    auto t4 = std::chrono::high_resolution_clock::now();
    for (size_t i = 0; i < cloud->size(); ++i) {
        const auto& pt = cloud->points[i];
        if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) {
            continue;
        }

        double d = std::sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
        float sigma_n = cloud_normals->points[i].curvature;
        if (!std::isfinite(sigma_n) || sigma_n < 0.0f) sigma_n = 0.0f;

        // Structured Adaptive Depth Policy:
        unsigned int depth;
        if (sigma_n < 0.025f) {
            // Highly planar region (flat ground / runway / smooth wall)
            depth = (d > 15.0) ? d_min : (d_min + 1);
        } else if (sigma_n < 0.06f) {
            // Moderate slope / curvature boundary
            depth = (d > 25.0) ? (d_min + 1) : (d_max - 1);
        } else {
            // High curvature / sharp edge / thin obstacle / wire / pole
            depth = d_max;
        }

        depth = std::clamp(depth, d_min, d_max);
        assigned_depth_counts[depth]++;

        // Insert into tree with lazy_eval for maximum batch throughput
        tree.updateNodeAtDepth(octomap::point3d(pt.x, pt.y, pt.z), depth, true, /*lazy_eval=*/true);
    }
    auto t5 = std::chrono::high_resolution_clock::now();
    std::cout << "[INFO] Inserted points in "
              << std::chrono::duration<double>(t5 - t4).count() << "s." << std::endl;

    std::cout << "\n--- Assigned Depth Distribution Across Points ---" << std::endl;
    for (const auto& [depth, count] : assigned_depth_counts) {
        double voxel_size = base_res * std::pow(2.0, static_cast<int>(tree_max_depth - depth));
        double pct = (100.0 * count) / cloud->size();
        std::cout << "  Depth " << depth << " (Voxel ~" << voxel_size << " m): "
                  << count << " points (" << pct << "%)" << std::endl;
    }

    // 5. Update Inner Occupancy & Prune
    std::cout << "\n[INFO] Updating inner node occupancies..." << std::endl;
    auto t6 = std::chrono::high_resolution_clock::now();
    tree.updateInnerOccupancy();
    auto t7 = std::chrono::high_resolution_clock::now();
    std::cout << "[INFO] Inner occupancies updated in "
              << std::chrono::duration<double>(t7 - t6).count() << "s." << std::endl;

    std::cout << "[INFO] Pre-prune tree size: " << tree.size()
              << ", leaf nodes: " << tree.getNumLeafNodes() << std::endl;

    std::cout << "[INFO] Pruning tree..." << std::endl;
    tree.prune();
    std::cout << "[INFO] Post-prune tree size: " << tree.size()
              << ", leaf nodes: " << tree.getNumLeafNodes() << std::endl;

    // 6. Inspect Leaf Depth Histogram
    std::map<unsigned int, size_t> leaf_hist;
    tree.getDepthHistogram(leaf_hist);
    std::cout << "\n--- Final Active Leaf Nodes by Depth ---" << std::endl;
    for (const auto& [depth, count] : leaf_hist) {
        double voxel_size = base_res * std::pow(2.0, static_cast<int>(tree_max_depth - depth));
        std::cout << "  Depth " << depth << " (Voxel " << voxel_size << " m): "
                  << count << " leaf voxels" << std::endl;
    }

    // 7. Metric Bounds
    double min_x, min_y, min_z, max_x, max_y, max_z;
    tree.getMetricMin(min_x, min_y, min_z);
    tree.getMetricMax(max_x, max_y, max_z);
    std::cout << "\n[INFO] OctoMap Metric Bounding Box:" << std::endl;
    std::cout << "       X: [" << min_x << " to " << max_x << "] m (extent: " << (max_x - min_x) << " m)" << std::endl;
    std::cout << "       Y: [" << min_y << " to " << max_y << "] m (extent: " << (max_y - min_y) << " m)" << std::endl;
    std::cout << "       Z: [" << min_z << " to " << max_z << "] m (extent: " << (max_z - min_z) << " m)" << std::endl;

    // 8. Write Output Files
    std::cout << "\n[INFO] Saving compact binary OctoMap: " << out_bt_path << std::endl;
    tree.writeBinaryConst(out_bt_path);

    std::string out_ot_path = out_bt_path.substr(0, out_bt_path.find_last_of(".")) + ".ot";
    std::cout << "[INFO] Saving full OctoMap: " << out_ot_path << std::endl;
    tree.write(out_ot_path);

    std::cout << "\n==========================================================" << std::endl;
    std::cout << "  DYNAMIC VOXEL OCTOMAP GENERATION COMPLETE [100%]" << std::endl;
    std::cout << "==========================================================" << std::endl;

    return 0;
}
