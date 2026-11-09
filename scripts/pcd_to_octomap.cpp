#include <iostream>
#include <string>
#include <cmath>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>

int main(int argc, char** argv) {
    if (argc < 3) {
        std::cout << "Usage: " << argv[0] << " <input.pcd> <output.bt> [resolution=0.2]" << std::endl;
        return 1;
    }

    std::string pcd_path = argv[1];
    std::string out_path = argv[2];
    double resolution = (argc >= 4) ? std::stod(argv[3]) : 0.2;

    std::cout << "========================================" << std::endl;
    std::cout << "  PCD to OctoMap Converter" << std::endl;
    std::cout << "  Input:      " << pcd_path << std::endl;
    std::cout << "  Output:     " << out_path << std::endl;
    std::cout << "  Resolution: " << resolution << " m" << std::endl;
    std::cout << "========================================" << std::endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_path, *cloud) == -1) {
        std::cerr << "[ERROR] Could not load PCD file: " << pcd_path << std::endl;
        return 1;
    }

    std::cout << "[INFO] Successfully loaded " << cloud->points.size() << " points from PCD." << std::endl;

    octomap::OcTree tree(resolution);

    size_t valid_points = 0;
    for (const auto& pt : cloud->points) {
        if (std::isfinite(pt.x) && std::isfinite(pt.y) && std::isfinite(pt.z)) {
            tree.updateNode(octomap::point3d(pt.x, pt.y, pt.z), true);
            valid_points++;
        }
    }

    std::cout << "[INFO] Integrated " << valid_points << " valid 3D points." << std::endl;

    std::cout << "[INFO] Updating inner occupancy..." << std::endl;
    tree.updateInnerOccupancy();

    std::cout << "[INFO] Tree depth: " << tree.getTreeDepth() << std::endl;
    std::cout << "[INFO] Total tree nodes: " << tree.size() << std::endl;
    std::cout << "[INFO] Total leaf nodes: " << tree.getNumLeafNodes() << std::endl;

    // Check bounds
    double min_x, min_y, min_z, max_x, max_y, max_z;
    tree.getMetricMin(min_x, min_y, min_z);
    tree.getMetricMax(max_x, max_y, max_z);
    std::cout << "[INFO] OctoMap Metric Bounding Box:" << std::endl;
    std::cout << "       X: [" << min_x << " to " << max_x << "] m" << std::endl;
    std::cout << "       Y: [" << min_y << " to " << max_y << "] m" << std::endl;
    std::cout << "       Z: [" << min_z << " to " << max_z << "] m" << std::endl;

    if (out_path.substr(out_path.find_last_of(".") + 1) == "ot") {
        std::cout << "[INFO] Writing full OcTree (.ot)..." << std::endl;
        tree.write(out_path);
    } else {
        std::cout << "[INFO] Writing compact binary OcTree (.bt)..." << std::endl;
        tree.writeBinary(out_path);
    }

    std::cout << "[SUCCESS] Saved OctoMap to: " << out_path << std::endl;
    return 0;
}

