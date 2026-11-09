// Dynamic / adaptive voxel OctoMap generator.
//
// Each point of the accumulated FAST-LIO2 map is assigned a target octree depth from its local
// surface variation (sigma_n, PCA over k nearest neighbours) and its range to the nearest
// sensor pose on the FAST-LIO2 trajectory. The octree is then built in two passes:
//
//   Pass 1 (refinement): a voxel at depth d is marked "refine" if any point inside it requests a
//          depth > d. The resolution field is therefore the finest resolution requested inside
//          each coarse cell, and it does not depend on point order.
//   Pass 2 (insertion):  every point is inserted at the first depth (>= d_min) whose voxel is not
//          marked "refine". All points in a voxel share that depth, so a coarse occupied leaf is
//          never expanded and no voxel that contains no LiDAR return is created.
//
// The result contains only occupied leaves that are supported by at least one point; all other
// space stays unknown (no children are created for it). Free space is not carved: scans.pcd stores
// accumulated world-frame points without the per-point sensor origin needed for ray insertion.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <map>
#include <string>
#include <unordered_set>
#include <vector>

#include <omp.h>

#include <pcl/features/normal_3d_omp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>

#include <octomap/OcTree.h>
#include <octomap/octomap.h>

#include "octomap_common.hpp"

using Clock = std::chrono::steady_clock;
static double secondsSince(Clock::time_point t) {
    return std::chrono::duration<double>(Clock::now() - t).count();
}

class DynamicOcTree : public octomap::OcTree {
public:
    explicit DynamicOcTree(double resolution) : octomap::OcTree(resolution) {}

    size_t conflicts = 0;  // must stay 0: a leaf on the insertion path or children at the target

    // Inserts an occupancy update for `key` at exactly `target_depth`. Only the child on the path
    // is created; existing leaves are never expanded (expandNode would fill all 8 children).
    void insertAtDepth(const octomap::OcTreeKey& key, unsigned target_depth, float log_odds_update) {
        if (!root) {
            root = new octomap::OcTreeNode();
            tree_size++;
        }
        octomap::OcTreeNode* node = root;
        bool node_is_new = true;  // the root acts as an inner node from the first insertion on
        for (unsigned depth = 0; depth < target_depth; ++depth) {
            const unsigned pos = octomap::computeChildIdx(key, static_cast<int>(tree_depth - 1 - depth));
            if (!nodeChildExists(node, pos)) {
                if (!node_is_new && !nodeHasChildren(node) && node != root) conflicts++;
                createNodeChild(node, pos);
                node_is_new = true;
            } else {
                node_is_new = false;
            }
            node = getNodeChild(node, pos);
        }
        if (nodeHasChildren(node)) {
            conflicts++;
            return;
        }
        updateNodeLogOdds(node, log_odds_update);
    }

    void leafDepthHistogram(std::map<unsigned, size_t>& occupied, size_t& free_leaves) const {
        occupied.clear();
        free_leaves = 0;
        for (auto it = begin_leafs(), end = end_leafs(); it != end; ++it) {
            if (isNodeOccupied(*it)) occupied[it.getDepth()]++;
            else free_leaves++;
        }
    }
};

static void usage(const char* prog) {
    std::cout << "Usage: " << prog << " <input.pcd> <output.bt> [options]\n"
              << "  --traj FILE    FAST-LIO2 state log (Log/mat_pre.txt) or 'x y z' file; range is measured to\n"
              << "                 the nearest trajectory pose. Without it, range is measured to the map origin.\n"
              << "  --res R        finest voxel size in metres (default 0.05, leaf depth 16)\n"
              << "  --dmin D       coarsest depth (default 13 -> 0.40 m)\n"
              << "  --dmax D       finest depth (default 16)\n"
              << "  --k K          neighbours for surface variation (default 25)\n"
              << "  --stats FILE   write generation statistics as JSON\n";
}

int main(int argc, char** argv) {
    if (argc < 3) {
        usage(argv[0]);
        return 1;
    }
    const std::string pcd_path = argv[1];
    const std::string out_bt_path = argv[2];
    std::string traj_path, stats_path;
    double base_res = 0.05;
    unsigned d_min = 13, d_max = 16;
    int k_neighbors = 25;
    for (int i = 3; i < argc; ++i) {
        const std::string a = argv[i];
        auto next = [&]() -> std::string {
            if (i + 1 >= argc) {
                usage(argv[0]);
                std::exit(1);
            }
            return argv[++i];
        };
        if (a == "--traj") traj_path = next();
        else if (a == "--res") base_res = std::stod(next());
        else if (a == "--dmin") d_min = std::stoul(next());
        else if (a == "--dmax") d_max = std::stoul(next());
        else if (a == "--k") k_neighbors = std::stoi(next());
        else if (a == "--stats") stats_path = next();
        else {
            usage(argv[0]);
            return 1;
        }
    }

    // Depth policy thresholds (surface variation sigma_n and range d, see report Ch. 4).
    const float SIGMA_PLANAR = 0.025f, SIGMA_EDGE = 0.060f;
    const double RANGE_PLANAR = 15.0, RANGE_MODERATE = 25.0;

    std::cout << "==========================================================\n"
              << "  DYNAMIC / ADAPTIVE VOXEL OCTOMAP GENERATOR (two-pass)\n"
              << "  Input PCD:       " << pcd_path << "\n"
              << "  Output BT:       " << out_bt_path << "\n"
              << "  Trajectory:      " << (traj_path.empty() ? "(none: range to origin)" : traj_path) << "\n"
              << "  Base resolution: " << base_res << " m, depths [" << d_min << ", " << d_max << "]\n"
              << "==========================================================" << std::endl;

    // 1. Load the point cloud and drop non-finite points.
    auto t_load = Clock::now();
    pcl::PointCloud<pcl::PointXYZ>::Ptr raw(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_path, *raw) == -1) {
        std::cerr << "[ERROR] Could not load PCD: " << pcd_path << std::endl;
        return 1;
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->points.reserve(raw->size());
    for (const auto& p : raw->points)
        if (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z)) cloud->points.push_back(p);
    cloud->width = cloud->points.size();
    cloud->height = 1;
    const double load_s = secondsSince(t_load);
    std::cout << "[INFO] Loaded " << raw->size() << " points (" << cloud->size() << " finite) in " << load_s
              << " s." << std::endl;

    // 2. Surface variation sigma_n = l3 / (l1 + l2 + l3) via PCL (OpenMP).
    auto t_curv = Clock::now();
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(cloud);
    ne.setSearchMethod(pcl::search::KdTree<pcl::PointXYZ>::Ptr(new pcl::search::KdTree<pcl::PointXYZ>));
    ne.setKSearch(k_neighbors);
    ne.setNumberOfThreads(omp_get_max_threads());
    pcl::PointCloud<pcl::Normal> normals;
    ne.compute(normals);
    const double curv_s = secondsSince(t_curv);
    std::cout << "[INFO] Surface variation (k=" << k_neighbors << ") in " << curv_s << " s using "
              << omp_get_max_threads() << " threads." << std::endl;

    // 3. Range to the nearest sensor pose.
    auto t_range = Clock::now();
    std::vector<float> range(cloud->size());
    pcl::PointCloud<pcl::PointXYZ>::Ptr traj;
    if (!traj_path.empty()) {
        traj = btp::loadTrajectory(traj_path);
        if (traj->empty()) {
            std::cerr << "[ERROR] No poses read from trajectory: " << traj_path << std::endl;
            return 1;
        }
        pcl::KdTreeFLANN<pcl::PointXYZ> kd;
        kd.setInputCloud(traj);
#pragma omp parallel for schedule(static)
        for (long i = 0; i < static_cast<long>(cloud->size()); ++i) {
            std::vector<int> idx(1);
            std::vector<float> d2(1);
            kd.nearestKSearch(cloud->points[i], 1, idx, d2);
            range[i] = std::sqrt(d2[0]);
        }
        std::cout << "[INFO] Range to nearest of " << traj->size() << " trajectory poses computed in "
                  << secondsSince(t_range) << " s." << std::endl;
    } else {
        for (size_t i = 0; i < cloud->size(); ++i) {
            const auto& p = cloud->points[i];
            range[i] = std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
        }
    }

    // 4. Target depth per point.
    DynamicOcTree tree(base_res);
    const unsigned td = tree.getTreeDepth();
    d_max = std::min(d_max, td);
    d_min = std::min(d_min, d_max);

    std::vector<octomap::OcTreeKey> keys(cloud->size());
    std::vector<uint8_t> target(cloud->size());
    std::map<unsigned, size_t> target_counts;
    std::map<std::string, size_t> class_counts;
    size_t out_of_bounds = 0;
    for (size_t i = 0; i < cloud->size(); ++i) {
        const auto& p = cloud->points[i];
        if (!tree.coordToKeyChecked(octomap::point3d(p.x, p.y, p.z), keys[i])) {
            target[i] = 0;
            out_of_bounds++;
            continue;
        }
        float s = normals.points[i].curvature;
        if (!std::isfinite(s) || s < 0.0f) s = 0.0f;
        const double d = range[i];
        unsigned depth;
        if (s < SIGMA_PLANAR) {
            depth = (d > RANGE_PLANAR) ? d_min : d_min + 1;
            class_counts[d > RANGE_PLANAR ? "planar_far" : "planar_near"]++;
        } else if (s < SIGMA_EDGE) {
            depth = (d > RANGE_MODERATE) ? d_min + 1 : d_max - 1;
            class_counts[d > RANGE_MODERATE ? "moderate_far" : "moderate_near"]++;
        } else {
            depth = d_max;
            class_counts["edge"]++;
        }
        depth = std::clamp(depth, d_min, d_max);
        target[i] = static_cast<uint8_t>(depth);
        target_counts[depth]++;
    }

    // 5. Pass 1: refinement sets. refine[d] holds voxels at depth d that must be subdivided.
    auto t_build = Clock::now();
    std::vector<octomap::KeySet> refine(td + 1);
    for (size_t i = 0; i < cloud->size(); ++i) {
        if (target[i] == 0) continue;
        for (unsigned d = d_min; d < target[i]; ++d) refine[d].insert(btp::keyAtDepth(keys[i], d, td));
    }

    // 6. Pass 2: insertion at the first non-refined depth.
    std::map<unsigned, size_t> insert_counts;
    for (size_t i = 0; i < cloud->size(); ++i) {
        if (target[i] == 0) continue;
        unsigned d = d_min;
        while (d < d_max && refine[d].count(btp::keyAtDepth(keys[i], d, td))) ++d;
        insert_counts[d]++;
        tree.insertAtDepth(keys[i], d, tree.getProbHitLog());
    }
    tree.updateInnerOccupancy();
    tree.prune();
    const double build_s = secondsSince(t_build);
    std::cout << "[INFO] Two-pass octree build in " << build_s << " s. Conflicts: " << tree.conflicts
              << " (must be 0), out-of-bounds points: " << out_of_bounds << std::endl;
    if (tree.conflicts != 0) {
        std::cerr << "[ERROR] Insertion conflicts detected; refinement invariant violated." << std::endl;
        return 2;
    }

    auto printHist = [&](const char* title, const std::map<unsigned, size_t>& h, size_t total) {
        std::cout << "\n--- " << title << " ---" << std::endl;
        for (const auto& [d, c] : h)
            std::cout << "  Depth " << d << " (" << btp::voxelSize(base_res, d, td) << " m): " << c << " ("
                      << 100.0 * c / std::max<size_t>(total, 1) << "%)" << std::endl;
    };
    printHist("Requested depth per point (policy output)", target_counts, cloud->size());
    printHist("Insertion depth per point (after neighbourhood refinement)", insert_counts, cloud->size());

    // 7. Write the full-probability tree, then the compact max-likelihood binary tree (same export
    //    path as the uniform maps: writeBinary() thresholds to max-likelihood and prunes).
    const std::string out_ot_path = out_bt_path.substr(0, out_bt_path.find_last_of('.')) + ".ot";
    tree.write(out_ot_path);
    tree.writeBinary(out_bt_path);

    std::map<unsigned, size_t> leaf_hist;
    size_t free_leaves = 0;
    tree.leafDepthHistogram(leaf_hist, free_leaves);
    size_t occ_leaves = 0;
    for (const auto& [d, c] : leaf_hist) occ_leaves += c;
    printHist("Occupied leaves in written .bt (after max-likelihood pruning)", leaf_hist, occ_leaves);

    double mnx, mny, mnz, mxx, mxy, mxz;
    tree.getMetricMin(mnx, mny, mnz);
    tree.getMetricMax(mxx, mxy, mxz);
    std::cout << "\n[INFO] Tree size: " << tree.size() << " nodes, " << occ_leaves << " occupied leaves, "
              << free_leaves << " free leaves." << std::endl;
    std::cout << "[INFO] Metric bounds: X [" << mnx << ", " << mxx << "]  Y [" << mny << ", " << mxy << "]  Z ["
              << mnz << ", " << mxz << "]" << std::endl;
    std::cout << "[INFO] Wrote " << out_bt_path << " and " << out_ot_path << std::endl;

    if (!stats_path.empty()) {
        std::ofstream js(stats_path);
        auto hist = [&](const std::map<unsigned, size_t>& h) {
            std::string s = "{";
            bool first = true;
            for (const auto& [d, c] : h) {
                s += (first ? "" : ", ") + std::string("\"") + std::to_string(d) + "\": " + std::to_string(c);
                first = false;
            }
            return s + "}";
        };
        js << "{\n"
           << "  \"input_points\": " << raw->size() << ",\n"
           << "  \"finite_points\": " << cloud->size() << ",\n"
           << "  \"trajectory\": \"" << traj_path << "\",\n"
           << "  \"trajectory_poses\": " << (traj ? traj->size() : 0) << ",\n"
           << "  \"base_resolution\": " << base_res << ",\n"
           << "  \"d_min\": " << d_min << ", \"d_max\": " << d_max << ", \"k\": " << k_neighbors << ",\n"
           << "  \"requested_depth_points\": " << hist(target_counts) << ",\n"
           << "  \"insertion_depth_points\": " << hist(insert_counts) << ",\n"
           << "  \"occupied_leaf_depths\": " << hist(leaf_hist) << ",\n"
           << "  \"class_points\": {";
        bool first = true;
        for (const auto& [k, c] : class_counts) {
            js << (first ? "" : ", ") << "\"" << k << "\": " << c;
            first = false;
        }
        js << "},\n"
           << "  \"tree_nodes\": " << tree.size() << ",\n"
           << "  \"occupied_leaves\": " << occ_leaves << ",\n"
           << "  \"conflicts\": " << tree.conflicts << ",\n"
           << "  \"timing_s\": {\"load\": " << load_s << ", \"curvature\": " << curv_s
           << ", \"build\": " << build_s << "}\n"
           << "}\n";
        std::cout << "[INFO] Stats written to " << stats_path << std::endl;
    }
    return 0;
}
