#include <iostream>
#include <iomanip>
#include <vector>
#include <string>
#include <chrono>
#include <random>
#include <fstream>
#include <cmath>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <octomap/octomap.h>
#include <octomap/OcTree.h>

struct BenchmarkResult {
    std::string name;
    std::string file_path;
    double file_size_kb;
    size_t total_nodes;
    size_t leaf_nodes;
    double load_time_ms;
    double raycast_latency_us; // microseconds per ray
    double chamfer_distance_m;
};

// Computes Chamfer distance between cloud and octree leaf centers
double computeChamferDistance(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                             const octomap::OcTree& tree,
                             size_t sample_size = 10000) {
    // 1. Collect leaf centers from tree
    pcl::PointCloud<pcl::PointXYZ>::Ptr tree_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (octomap::OcTree::leaf_iterator it = tree.begin_leafs(), end = tree.end_leafs(); it != end; ++it) {
        if (tree.isNodeOccupied(*it)) {
            tree_cloud->points.emplace_back(it.getX(), it.getY(), it.getZ());
        }
    }
    tree_cloud->width = tree_cloud->points.size();
    tree_cloud->height = 1;

    if (tree_cloud->empty()) return 999.0;

    // 2. Build KD-Trees
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_tree;
    kdtree_tree.setInputCloud(tree_cloud);

    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_cloud;
    kdtree_cloud.setInputCloud(cloud);

    // 3. Subsample cloud points for Chamfer distance evaluation
    size_t step = std::max(size_t(1), cloud->size() / sample_size);
    double dist_cloud_to_tree = 0.0;
    size_t count_c2t = 0;

    std::vector<int> indices(1);
    std::vector<float> sqr_distances(1);

    for (size_t i = 0; i < cloud->size(); i += step) {
        if (kdtree_tree.nearestKSearch(cloud->points[i], 1, indices, sqr_distances) > 0) {
            dist_cloud_to_tree += std::sqrt(sqr_distances[0]);
            count_c2t++;
        }
    }

    // 4. Sample from tree to cloud
    size_t tree_step = std::max(size_t(1), tree_cloud->size() / sample_size);
    double dist_tree_to_cloud = 0.0;
    size_t count_t2c = 0;

    for (size_t i = 0; i < tree_cloud->size(); i += tree_step) {
        if (kdtree_cloud.nearestKSearch(tree_cloud->points[i], 1, indices, sqr_distances) > 0) {
            dist_tree_to_cloud += std::sqrt(sqr_distances[0]);
            count_t2c++;
        }
    }

    double mean_c2t = (count_c2t > 0) ? (dist_cloud_to_tree / count_c2t) : 0.0;
    double mean_t2c = (count_t2c > 0) ? (dist_tree_to_cloud / count_t2c) : 0.0;

    return 0.5 * (mean_c2t + mean_t2c);
}

// Benchmarks 100,000 raycasts simulating obstacle avoidance collision checking
double benchmarkRaycasting(const octomap::OcTree& tree, size_t num_rays = 50000) {
    std::mt19937 rng(42);
    std::uniform_real_distribution<double> dist_phi(0, 2.0 * M_PI);
    std::uniform_real_distribution<double> dist_theta(-M_PI / 4.0, M_PI / 4.0);

    octomap::point3d origin(0.0, 0.0, 0.0);
    octomap::point3d end;

    auto t0 = std::chrono::high_resolution_clock::now();
    size_t hits = 0;

    for (size_t i = 0; i < num_rays; ++i) {
        double phi = dist_phi(rng);
        double theta = dist_theta(rng);
        octomap::point3d dir(std::cos(theta) * std::cos(phi),
                             std::cos(theta) * std::sin(phi),
                             std::sin(theta));
        if (tree.castRay(origin, dir, end, true, 45.0)) {
            hits++;
        }
    }
    auto t1 = std::chrono::high_resolution_clock::now();

    double total_us = std::chrono::duration<double, std::micro>(t1 - t0).count();
    return total_us / num_rays;
}

int main(int argc, char** argv) {
    std::string pcd_path = "src/FAST_LIO_ROS2/PCD/scans.pcd";
    if (argc > 1) pcd_path = argv[1];

    std::cout << "==========================================================" << std::endl;
    std::cout << "  OCTOMAP COMPREHENSIVE BENCHMARKING ENGINE" << std::endl;
    std::cout << "  Ground Truth PCD: " << pcd_path << std::endl;
    std::cout << "==========================================================" << std::endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_path, *cloud) == -1) {
        std::cerr << "Failed to load PCD: " << pcd_path << std::endl;
        return 1;
    }
    std::cout << "[INFO] Loaded ground truth cloud with " << cloud->size() << " points.\n" << std::endl;

    std::vector<std::pair<std::string, std::string>> maps_to_test = {
        {"Uniform 0.40m", "maps/uav_octomap_0.4m.bt"},
        {"Uniform 0.20m", "maps/uav_octomap_0.2m.bt"},
        {"Uniform 0.10m", "maps/uav_octomap_0.1m.bt"},
        {"Dynamic Adaptive", "maps/dynamic_octomap.bt"}
    };

    std::vector<BenchmarkResult> results;

    for (const auto& [name, path] : maps_to_test) {
        std::cout << "[BENCHMARK] Testing map: " << name << " (" << path << ")..." << std::endl;
        std::ifstream file(path, std::ios::binary | std::ios::ate);
        if (!file.is_open()) {
            std::cerr << "  [ERROR] Could not open file: " << path << std::endl;
            continue;
        }
        double file_size_kb = static_cast<double>(file.tellg()) / 1024.0;
        file.close();

        // Load tree
        octomap::OcTree tree(0.05);
        auto t0 = std::chrono::high_resolution_clock::now();
        if (!tree.readBinary(path)) {
            std::cerr << "  [ERROR] Failed to parse binary octree: " << path << std::endl;
            continue;
        }
        auto t1 = std::chrono::high_resolution_clock::now();
        double load_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

        // Benchmarks
        std::cout << "  -> Loaded in " << load_ms << " ms (" << tree.size() << " nodes)." << std::endl;
        std::cout << "  -> Benchmarking raycasting latency..." << std::endl;
        double ray_us = benchmarkRaycasting(tree, 50000);

        std::cout << "  -> Computing Chamfer distance..." << std::endl;
        double chamfer = computeChamferDistance(cloud, tree, 10000);

        BenchmarkResult res;
        res.name = name;
        res.file_path = path;
        res.file_size_kb = file_size_kb;
        res.total_nodes = tree.size();
        res.leaf_nodes = tree.getNumLeafNodes();
        res.load_time_ms = load_ms;
        res.raycast_latency_us = ray_us;
        res.chamfer_distance_m = chamfer;

        results.push_back(res);
        std::cout << "  [DONE] Raycast: " << ray_us << " us/ray | Chamfer: " << chamfer << " m\n" << std::endl;
    }

    // Print Markdown Summary Table
    std::cout << "\n====================================================================================================" << std::endl;
    std::cout << "                                  BENCHMARK EVALUATION SUMMARY TABLE                                  " << std::endl;
    std::cout << "====================================================================================================" << std::endl;
    std::cout << "| Map Representation | File Size (KB) | Total Nodes | Leaf Voxels | Load Time (ms) | Raycast Latency (us) | Chamfer Dist (m) |" << std::endl;
    std::cout << "| :--- | :---: | :---: | :---: | :---: | :---: | :---: |" << std::endl;
    for (const auto& r : results) {
        std::cout << "| **" << std::left << std::setw(18) << r.name << "** | "
                  << std::right << std::setw(14) << std::fixed << std::setprecision(1) << r.file_size_kb << " | "
                  << std::setw(11) << r.total_nodes << " | "
                  << std::setw(11) << r.leaf_nodes << " | "
                  << std::setw(14) << std::fixed << std::setprecision(2) << r.load_time_ms << " | "
                  << std::setw(20) << std::fixed << std::setprecision(2) << r.raycast_latency_us << " | "
                  << std::setw(16) << std::fixed << std::setprecision(4) << r.chamfer_distance_m << " |"
                  << std::endl;
    }
    std::cout << "====================================================================================================\n" << std::endl;

    // Export JSON/Markdown report for inclusion in BTP report
    std::ofstream report_out("Reports/benchmark_octomap_results.md");
    if (report_out.is_open()) {
        report_out << "### Multi-Resolution OctoMap Quantitative Benchmark Results\n\n";
        report_out << "| Map Representation | File Size (KB) | Total Nodes | Leaf Voxels | Load Time (ms) | Raycast Latency (us) | Chamfer Distance (m) |\n";
        report_out << "| :--- | :---: | :---: | :---: | :---: | :---: | :---: |\n";
        for (const auto& r : results) {
            report_out << "| **" << r.name << "** | "
                       << std::fixed << std::setprecision(1) << r.file_size_kb << " | "
                       << r.total_nodes << " | "
                       << r.leaf_nodes << " | "
                       << std::fixed << std::setprecision(2) << r.load_time_ms << " | "
                       << std::fixed << std::setprecision(2) << r.raycast_latency_us << " | "
                       << std::fixed << std::setprecision(4) << r.chamfer_distance_m << " |\n";
        }
        report_out.close();
        std::cout << "[INFO] Benchmark report written to: Reports/benchmark_octomap_results.md" << std::endl;
    }

    return 0;
}

