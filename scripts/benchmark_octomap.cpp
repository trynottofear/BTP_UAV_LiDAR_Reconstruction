// Benchmark and verification of uniform and adaptive OctoMaps against the FAST-LIO2 point cloud.
//
// Per map it reports:
//   * storage: file size, tree nodes, occupied leaves, in-memory size, load time
//   * correctness: unsupported occupied leaves (voxels that contain no LiDAR point = false
//     occupancy), point coverage (points inside an occupied voxel), occupied volume, occupied bbox
//   * geometry: Chamfer distance between the cloud and occupied voxel centres, and the voxel size
//     that contains edge (high sigma_n) versus planar (low sigma_n) points
//   * collision queries: the same set of rays (origins = FAST-LIO2 trajectory poses, directions
//     uniform on the sphere, 30 m max range) cast with (a) OctoMap's castRay() and (b) a
//     hierarchical traversal that skips whole unknown / free octree cells. Hit agreement between
//     the two is reported as a check.
//
// Usage: benchmark_octomap --pcd scans.pcd --traj mat_pre.txt [--json out.json] [--md out.md]
//                          [--rays N] [--range R] [Name=path.bt ...]

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <random>
#include <sstream>
#include <string>
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

struct Ray {
    octomap::point3d origin, dir;
};

struct RayStats {
    double us_per_ray = 0.0;
    size_t hits = 0;
    double mean_hit_dist = 0.0;
    size_t origin_occupied = 0;
    double mean_cells_visited = 0.0;  // hierarchical only
};

struct RaySuite {
    std::string key;
    RayStats castray, hier;
    double hit_agreement = 0;      // fraction of rays where both methods agree on hit / miss
    double hit_dist_diff_med = 0;  // median |d_castRay - d_hier| over rays both hit
};

struct Result {
    std::string name, path;
    double file_kb = 0, load_ms = 0, resolution = 0;
    size_t nodes = 0, leaves = 0, occ_leaves = 0, memory_bytes = 0;
    std::map<double, size_t> occ_leaf_sizes;  // voxel edge [m] -> count
    double occ_volume_m3 = 0;
    size_t unsupported = 0;
    double unsupported_volume_m3 = 0;
    double max_occ_voxel_m = 0;
    double bbox_min[3] = {0, 0, 0}, bbox_max[3] = {0, 0, 0};
    double coverage = 0;
    double chamfer_c2m = 0, chamfer_m2c = 0, chamfer = 0;
    double chamfer_c2m_edge = 0, chamfer_c2m_planar = 0;
    double voxel_at_edge = 0, voxel_at_planar = 0;
    std::vector<RaySuite> suites;
};

// Hierarchical ray traversal: at each step find the octree cell containing the current point;
// stop on an occupied leaf, otherwise jump to the exit face of that cell (which may be a large
// unknown cell or a coarse free leaf).
static bool castRayHierarchical(const octomap::OcTree& tree, const octomap::point3d& origin,
                                const octomap::point3d& dir, double max_range, double& hit_dist,
                                size_t& cells) {
    const unsigned td = tree.getTreeDepth();
    const double res = tree.getResolution();
    const double o[3] = {origin.x(), origin.y(), origin.z()};
    const double v[3] = {dir.x(), dir.y(), dir.z()};
    const double eps = 1e-4;  // 0.1 mm step past each cell face
    double t = 0.0;
    cells = 0;
    while (t <= max_range && cells < 1000000) {
        const double p[3] = {o[0] + v[0] * t, o[1] + v[1] * t, o[2] + v[2] * t};
        octomap::OcTreeKey key;
        if (!tree.coordToKeyChecked(p[0], p[1], p[2], key)) return false;
        unsigned depth;
        const octomap::OcTreeNode* node = btp::lookupLeaf(tree, key, depth);
        ++cells;
        if (node && tree.isNodeOccupied(node)) {
            hit_dist = t;
            return true;
        }
        const double size = btp::voxelSize(res, depth, td);
        const octomap::OcTreeKey k = btp::keyAtDepth(key, depth, td);
        double t_exit = std::numeric_limits<double>::infinity();
        for (int a = 0; a < 3; ++a) {
            // Cell spans [lo, lo + size) where lo is the coordinate of the cell's lowest key.
            const double lo = (static_cast<double>(k[a]) - static_cast<double>(1u << (td - 1))) * res;
            if (v[a] > 1e-12) t_exit = std::min(t_exit, (lo + size - p[a]) / v[a]);
            else if (v[a] < -1e-12) t_exit = std::min(t_exit, (lo - p[a]) / v[a]);
        }
        t += std::max(t_exit, 0.0) + eps;
    }
    return false;
}

static double median(std::vector<double> v) {
    if (v.empty()) return 0.0;
    std::nth_element(v.begin(), v.begin() + v.size() / 2, v.end());
    return v[v.size() / 2];
}

static std::string jsonEscape(const std::string& s) {
    std::string o;
    for (char c : s) {
        if (c == '"' || c == '\\') o += '\\';
        o += c;
    }
    return o;
}

int main(int argc, char** argv) {
    std::string pcd_path = "src/FAST_LIO_ROS2/PCD/scans.pcd";
    std::string traj_path = "src/FAST_LIO_ROS2/Log/mat_pre.txt";
    std::string json_path = "Reports/benchmark/benchmark_results.json";
    std::string md_path = "Reports/benchmark_octomap_results.md";
    size_t num_rays = 50000;
    double max_range = 30.0;
    std::vector<std::pair<std::string, std::string>> maps;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        auto next = [&]() { return std::string(argv[++i]); };
        if (a == "--pcd") pcd_path = next();
        else if (a == "--traj") traj_path = next();
        else if (a == "--json") json_path = next();
        else if (a == "--md") md_path = next();
        else if (a == "--rays") num_rays = std::stoul(next());
        else if (a == "--range") max_range = std::stod(next());
        else if (a.find('=') != std::string::npos) maps.emplace_back(a.substr(0, a.find('=')), a.substr(a.find('=') + 1));
    }
    if (maps.empty()) {
        maps = {{"Uniform 0.40 m", "maps/uav_octomap_0.4m.bt"},
                {"Uniform 0.20 m", "maps/uav_octomap_0.2m.bt"},
                {"Uniform 0.10 m", "maps/uav_octomap_0.1m.bt"},
                {"Uniform 0.05 m", "maps/uav_octomap_0.05m.bt"},
                {"Dynamic Adaptive", "maps/dynamic_octomap.bt"}};
    }

    std::cout << "==========================================================\n"
              << "  OCTOMAP BENCHMARK & VERIFICATION\n"
              << "  Point cloud: " << pcd_path << "\n  Trajectory:  " << traj_path << "\n"
              << "  Rays: " << num_rays << ", max range " << max_range << " m\n"
              << "==========================================================" << std::endl;

    // ---- Reference data -------------------------------------------------------------------
    pcl::PointCloud<pcl::PointXYZ>::Ptr raw(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_path, *raw) == -1) {
        std::cerr << "Failed to load PCD: " << pcd_path << std::endl;
        return 1;
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto& p : raw->points)
        if (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z)) cloud->points.push_back(p);
    cloud->width = cloud->points.size();
    cloud->height = 1;
    auto traj = btp::loadTrajectory(traj_path);
    if (traj->empty()) {
        std::cerr << "No trajectory poses in " << traj_path << std::endl;
        return 1;
    }
    std::cout << "[INFO] " << cloud->size() << " points, " << traj->size() << " trajectory poses." << std::endl;

    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(cloud);
    ne.setSearchMethod(pcl::search::KdTree<pcl::PointXYZ>::Ptr(new pcl::search::KdTree<pcl::PointXYZ>));
    ne.setKSearch(25);
    ne.setNumberOfThreads(omp_get_max_threads());
    pcl::PointCloud<pcl::Normal> normals;
    ne.compute(normals);
    std::vector<uint8_t> cls(cloud->size());  // 0 planar (<0.025), 1 moderate, 2 edge (>=0.06)
    size_t n_edge = 0, n_planar = 0;
    for (size_t i = 0; i < cloud->size(); ++i) {
        float s = normals.points[i].curvature;
        if (!std::isfinite(s)) s = 0.0f;
        cls[i] = s < 0.025f ? 0 : (s < 0.06f ? 1 : 2);
        n_edge += cls[i] == 2;
        n_planar += cls[i] == 0;
    }

    pcl::KdTreeFLANN<pcl::PointXYZ> kd_cloud;
    kd_cloud.setInputCloud(cloud);

    // Deterministic ray sets shared by all maps. Origins are FAST-LIO2 trajectory poses.
    //   omni:    directions uniform on the unit sphere (omnidirectional collision probe; most rays
    //            leave the scanned volume, so misses dominate)
    //   surface: direction towards a random cloud point within max_range of the pose (line-of-sight
    //            queries towards observed structure; hits dominate)
    std::mt19937 rng(42);
    std::uniform_int_distribution<size_t> pick_pose(0, traj->size() - 1);
    std::uniform_int_distribution<size_t> pick_point(0, cloud->size() - 1);
    std::normal_distribution<double> gauss(0.0, 1.0);
    std::vector<std::pair<std::string, std::vector<Ray>>> ray_sets = {{"omni", {}}, {"surface", {}}};
    for (size_t i = 0; i < num_rays; ++i) {
        const auto& p = traj->points[pick_pose(rng)];
        double x = gauss(rng), y = gauss(rng), z = gauss(rng);
        const double n = std::sqrt(x * x + y * y + z * z);
        ray_sets[0].second.push_back({octomap::point3d(p.x, p.y, p.z), octomap::point3d(x / n, y / n, z / n)});
    }
    while (ray_sets[1].second.size() < num_rays) {
        const auto& p = traj->points[pick_pose(rng)];
        const auto& q = cloud->points[pick_point(rng)];
        const octomap::point3d o(p.x, p.y, p.z), d = octomap::point3d(q.x, q.y, q.z) - o;
        const double n = d.norm();
        if (n < 1.0 || n > max_range) continue;
        ray_sets[1].second.push_back({o, d * static_cast<float>(1.0 / n)});
    }

    // Deterministic sample of cloud points for Chamfer cloud->map.
    const size_t sample = 20000;
    const size_t step = std::max<size_t>(1, cloud->size() / sample);

    std::vector<Result> results;
    for (const auto& [name, path] : maps) {
        std::cout << "\n[BENCHMARK] " << name << " (" << path << ")" << std::endl;
        std::ifstream f(path, std::ios::binary | std::ios::ate);
        if (!f.is_open()) {
            std::cerr << "  [SKIP] cannot open " << path << std::endl;
            continue;
        }
        Result R;
        R.name = name;
        R.path = path;
        R.file_kb = static_cast<double>(f.tellg()) / 1024.0;
        f.close();

        // Load time: median of 5 loads.
        std::vector<double> load_times;
        octomap::OcTree tree(0.1);
        for (int rep = 0; rep < 5; ++rep) {
            octomap::OcTree t(0.1);
            auto t0 = Clock::now();
            if (!t.readBinary(path)) {
                std::cerr << "  [ERROR] cannot parse " << path << std::endl;
                return 1;
            }
            load_times.push_back(std::chrono::duration<double, std::milli>(Clock::now() - t0).count());
            if (rep == 0) tree.readBinary(path);
        }
        R.load_ms = median(load_times);
        const unsigned td = tree.getTreeDepth();
        R.resolution = tree.getResolution();
        R.nodes = tree.size();
        R.leaves = tree.getNumLeafNodes();
        R.memory_bytes = tree.memoryUsage();

        // --- Occupied leaves: sizes, volume, bbox, centres --------------------------------
        pcl::PointCloud<pcl::PointXYZ>::Ptr centres(new pcl::PointCloud<pcl::PointXYZ>);
        std::vector<std::pair<octomap::OcTreeKey, unsigned>> occ_keys;
        for (int a = 0; a < 3; ++a) {
            R.bbox_min[a] = std::numeric_limits<double>::infinity();
            R.bbox_max[a] = -std::numeric_limits<double>::infinity();
        }
        std::map<unsigned, bool> depths_present;
        for (auto it = tree.begin_leafs(), end = tree.end_leafs(); it != end; ++it) {
            if (!tree.isNodeOccupied(*it)) continue;
            const double s = it.getSize();
            R.occ_leaves++;
            R.occ_leaf_sizes[std::round(s * 1000.0) / 1000.0]++;
            R.occ_volume_m3 += s * s * s;
            R.max_occ_voxel_m = std::max(R.max_occ_voxel_m, s);
            const double c[3] = {it.getX(), it.getY(), it.getZ()};
            for (int a = 0; a < 3; ++a) {
                R.bbox_min[a] = std::min(R.bbox_min[a], c[a] - s / 2);
                R.bbox_max[a] = std::max(R.bbox_max[a], c[a] + s / 2);
            }
            centres->points.emplace_back(c[0], c[1], c[2]);
            occ_keys.emplace_back(btp::keyAtDepth(it.getKey(), it.getDepth(), td), it.getDepth());
            depths_present[it.getDepth()] = true;
        }
        centres->width = centres->points.size();
        centres->height = 1;

        // --- Support check: each occupied leaf must contain at least one cloud point --------
        std::vector<octomap::OcTreeKey> pkeys(cloud->size());
        std::vector<bool> pkey_ok(cloud->size());
        for (size_t i = 0; i < cloud->size(); ++i)
            pkey_ok[i] = tree.coordToKeyChecked(octomap::point3d(cloud->points[i].x, cloud->points[i].y,
                                                                 cloud->points[i].z), pkeys[i]);
        std::map<unsigned, octomap::KeySet> support;
        for (const auto& [d, _] : depths_present) {
            auto& ks = support[d];
            for (size_t i = 0; i < cloud->size(); ++i)
                if (pkey_ok[i]) ks.insert(btp::keyAtDepth(pkeys[i], d, td));
        }
        for (const auto& [k, d] : occ_keys) {
            if (!support[d].count(k)) {
                R.unsupported++;
                const double s = btp::voxelSize(R.resolution, d, td);
                R.unsupported_volume_m3 += s * s * s;
            }
        }

        // --- Coverage and voxel size at edge / planar points ----------------------------------
        size_t covered = 0;
        double vox_edge = 0, vox_planar = 0;
        size_t cnt_edge = 0, cnt_planar = 0;
        for (size_t i = 0; i < cloud->size(); ++i) {
            if (!pkey_ok[i]) continue;
            unsigned d;
            const auto* node = btp::lookupLeaf(tree, pkeys[i], d);
            if (node && tree.isNodeOccupied(node)) {
                covered++;
                const double s = btp::voxelSize(R.resolution, d, td);
                if (cls[i] == 2) { vox_edge += s; cnt_edge++; }
                if (cls[i] == 0) { vox_planar += s; cnt_planar++; }
            }
        }
        R.coverage = static_cast<double>(covered) / cloud->size();
        R.voxel_at_edge = cnt_edge ? vox_edge / cnt_edge : 0;
        R.voxel_at_planar = cnt_planar ? vox_planar / cnt_planar : 0;

        // --- Chamfer distance -------------------------------------------------------------
        if (!centres->empty()) {
            pcl::KdTreeFLANN<pcl::PointXYZ> kd_centres;
            kd_centres.setInputCloud(centres);
            std::vector<int> idx(1);
            std::vector<float> d2(1);
            double s_all = 0, s_edge = 0, s_planar = 0;
            size_t c_all = 0, c_edge = 0, c_planar = 0;
            for (size_t i = 0; i < cloud->size(); i += step) {
                if (kd_centres.nearestKSearch(cloud->points[i], 1, idx, d2) <= 0) continue;
                const double d = std::sqrt(d2[0]);
                s_all += d; c_all++;
                if (cls[i] == 2) { s_edge += d; c_edge++; }
                if (cls[i] == 0) { s_planar += d; c_planar++; }
            }
            R.chamfer_c2m = s_all / std::max<size_t>(c_all, 1);
            R.chamfer_c2m_edge = s_edge / std::max<size_t>(c_edge, 1);
            R.chamfer_c2m_planar = s_planar / std::max<size_t>(c_planar, 1);
            const size_t cstep = std::max<size_t>(1, centres->size() / sample);
            double s_m2c = 0;
            size_t c_m2c = 0;
            for (size_t i = 0; i < centres->size(); i += cstep) {
                if (kd_cloud.nearestKSearch(centres->points[i], 1, idx, d2) <= 0) continue;
                s_m2c += std::sqrt(d2[0]);
                c_m2c++;
            }
            R.chamfer_m2c = s_m2c / std::max<size_t>(c_m2c, 1);
            R.chamfer = 0.5 * (R.chamfer_c2m + R.chamfer_m2c);
        }

        // --- Ray casting ------------------------------------------------------------------
        for (const auto& [set_name, rays] : ray_sets) {
            RaySuite S;
            S.key = set_name;
            std::vector<char> hit_a(rays.size()), hit_b(rays.size());
            std::vector<double> dist_a(rays.size()), dist_b(rays.size());
            std::vector<size_t> cells_b(rays.size());
            auto timeRuns = [&](auto&& body) {
                std::vector<double> runs;
                for (int rep = 0; rep < 3; ++rep) {
                    auto t0 = Clock::now();
                    body();
                    runs.push_back(std::chrono::duration<double, std::micro>(Clock::now() - t0).count());
                }
                return median(runs) / rays.size();
            };
            S.castray.us_per_ray = timeRuns([&]() {
                octomap::point3d end;
                for (size_t i = 0; i < rays.size(); ++i) {
                    hit_a[i] = tree.castRay(rays[i].origin, rays[i].dir, end, true, max_range);
                    dist_a[i] = hit_a[i] ? (end - rays[i].origin).norm() : 0.0;
                }
            });
            S.hier.us_per_ray = timeRuns([&]() {
                for (size_t i = 0; i < rays.size(); ++i) {
                    double d = 0;
                    hit_b[i] = castRayHierarchical(tree, rays[i].origin, rays[i].dir, max_range, d, cells_b[i]);
                    dist_b[i] = d;
                }
            });
            size_t agree = 0;
            std::vector<double> diffs;
            double sum_cells = 0;
            for (size_t i = 0; i < rays.size(); ++i) {
                octomap::OcTreeKey k;
                unsigned d;
                if (tree.coordToKeyChecked(rays[i].origin, k)) {
                    const auto* n = btp::lookupLeaf(tree, k, d);
                    if (n && tree.isNodeOccupied(n)) { S.castray.origin_occupied++; S.hier.origin_occupied++; }
                }
                if (hit_a[i]) { S.castray.hits++; S.castray.mean_hit_dist += dist_a[i]; }
                if (hit_b[i]) { S.hier.hits++; S.hier.mean_hit_dist += dist_b[i]; }
                if (hit_a[i] == hit_b[i]) agree++;
                if (hit_a[i] && hit_b[i]) diffs.push_back(std::fabs(dist_a[i] - dist_b[i]));
                sum_cells += cells_b[i];
            }
            S.castray.mean_hit_dist /= std::max<size_t>(S.castray.hits, 1);
            S.hier.mean_hit_dist /= std::max<size_t>(S.hier.hits, 1);
            S.hier.mean_cells_visited = sum_cells / rays.size();
            S.hit_agreement = static_cast<double>(agree) / rays.size();
            S.hit_dist_diff_med = median(diffs);
            R.suites.push_back(S);
        }

        std::cout << std::fixed << std::setprecision(4)
                  << "  nodes " << R.nodes << ", occupied leaves " << R.occ_leaves << ", file " << R.file_kb << " KB\n"
                  << "  unsupported leaves " << R.unsupported << " (" << R.unsupported_volume_m3 << " m^3), coverage "
                  << 100.0 * R.coverage << " %, occupied volume " << R.occ_volume_m3 << " m^3, max voxel "
                  << R.max_occ_voxel_m << " m\n"
                  << "  chamfer " << R.chamfer << " m (c2m " << R.chamfer_c2m << ", m2c " << R.chamfer_m2c
                  << "; edge c2m " << R.chamfer_c2m_edge << ", planar c2m " << R.chamfer_c2m_planar << ")\n"
                  << "  voxel size at edge pts " << R.voxel_at_edge << " m, at planar pts " << R.voxel_at_planar << " m\n"
                  << "  load " << R.load_ms << " ms, memory " << R.memory_bytes / 1024.0 << " KB" << std::endl;
        for (const auto& S : R.suites)
            std::cout << "  [rays:" << S.key << "] castRay " << S.castray.us_per_ray << " us/ray, hits "
                      << S.castray.hits << ", mean hit dist " << S.castray.mean_hit_dist
                      << " m, origins in occupied voxels " << S.castray.origin_occupied << " | hierarchical "
                      << S.hier.us_per_ray << " us/ray, hits " << S.hier.hits << ", cells/ray "
                      << S.hier.mean_cells_visited << ", agreement " << 100.0 * S.hit_agreement << " %, median |dd| "
                      << S.hit_dist_diff_med << " m" << std::endl;
        results.push_back(R);
    }

    // ---- Markdown summary ---------------------------------------------------------------------
    std::ostringstream md;
    md << std::fixed;
    md << "### OctoMap Benchmark & Verification Results\n\n"
       << "Generated by `scripts/benchmark_octomap` on `" << pcd_path << "` (" << cloud->size()
       << " points). Rays: " << num_rays << " from " << traj->size()
       << " FAST-LIO2 trajectory poses per ray set (omni: uniform directions; surface: towards random cloud points), "
       << std::setprecision(1) << max_range << " m max range. Full metrics in `Reports/benchmark/benchmark_results.json`.\n\n"
       << "| Map | File (KB) | Nodes | Occ. leaves | Unsupported leaves | Coverage (%) | Occ. volume (m^3) | "
          "Chamfer (m) | Voxel @ edge pts (m) | Voxel @ planar pts (m) | castRay omni (us/ray) | Hierarchical omni (us/ray) | "
          "castRay surface (us/ray) | Hierarchical surface (us/ray) |\n"
       << "| :--- |" << [] { std::string s; for (int c = 0; c < 13; ++c) s += " :---: |"; return s; }() << "\n";
    for (const auto& r : results) {
        md << "| " << r.name << " | " << std::setprecision(1) << r.file_kb << " | " << r.nodes << " | "
           << r.occ_leaves << " | " << r.unsupported << " | " << std::setprecision(2) << 100.0 * r.coverage << " | "
           << std::setprecision(1) << r.occ_volume_m3 << " | " << std::setprecision(4) << r.chamfer << " | "
           << std::setprecision(3) << r.voxel_at_edge << " | " << r.voxel_at_planar << " | " << std::setprecision(2)
           << r.suites[0].castray.us_per_ray << " | " << r.suites[0].hier.us_per_ray << " | "
           << r.suites[1].castray.us_per_ray << " | " << r.suites[1].hier.us_per_ray << " |\n";
    }
    std::ofstream(md_path) << md.str();
    std::cout << "\n" << md.str() << "[INFO] Markdown written to " << md_path << std::endl;

    // ---- JSON ---------------------------------------------------------------------------------
    std::ofstream js(json_path);
    js << std::setprecision(10);
    js << "{\n  \"pcd\": \"" << jsonEscape(pcd_path) << "\",\n  \"points\": " << cloud->size()
       << ",\n  \"edge_points\": " << n_edge << ",\n  \"planar_points\": " << n_planar
       << ",\n  \"trajectory\": \"" << jsonEscape(traj_path) << "\",\n  \"trajectory_poses\": " << traj->size()
       << ",\n  \"rays\": " << num_rays << ",\n  \"max_range_m\": " << max_range << ",\n  \"maps\": [\n";
    for (size_t m = 0; m < results.size(); ++m) {
        const auto& r = results[m];
        js << "    {\"name\": \"" << jsonEscape(r.name) << "\", \"path\": \"" << jsonEscape(r.path) << "\""
           << ", \"resolution\": " << r.resolution << ", \"file_kb\": " << r.file_kb << ", \"load_ms\": " << r.load_ms
           << ", \"nodes\": " << r.nodes << ", \"leaves\": " << r.leaves << ", \"occupied_leaves\": " << r.occ_leaves
           << ", \"memory_bytes\": " << r.memory_bytes << ", \"occupied_volume_m3\": " << r.occ_volume_m3
           << ", \"unsupported_leaves\": " << r.unsupported << ", \"unsupported_volume_m3\": " << r.unsupported_volume_m3
           << ", \"max_occupied_voxel_m\": " << r.max_occ_voxel_m << ", \"coverage\": " << r.coverage
           << ", \"bbox_min\": [" << r.bbox_min[0] << ", " << r.bbox_min[1] << ", " << r.bbox_min[2] << "]"
           << ", \"bbox_max\": [" << r.bbox_max[0] << ", " << r.bbox_max[1] << ", " << r.bbox_max[2] << "]"
           << ", \"chamfer\": " << r.chamfer << ", \"chamfer_c2m\": " << r.chamfer_c2m
           << ", \"chamfer_m2c\": " << r.chamfer_m2c << ", \"chamfer_c2m_edge\": " << r.chamfer_c2m_edge
           << ", \"chamfer_c2m_planar\": " << r.chamfer_c2m_planar << ", \"voxel_at_edge_m\": " << r.voxel_at_edge
           << ", \"voxel_at_planar_m\": " << r.voxel_at_planar << ", \"occupied_leaf_sizes\": {";
        bool first = true;
        for (const auto& [s, c] : r.occ_leaf_sizes) {
            js << (first ? "" : ", ") << "\"" << s << "\": " << c;
            first = false;
        }
        js << "}, \"rays\": {";
        for (size_t q = 0; q < r.suites.size(); ++q) {
            const auto& S = r.suites[q];
            js << (q ? ", " : "") << "\"" << S.key << "\": {\"castray\": {\"us_per_ray\": " << S.castray.us_per_ray
               << ", \"hits\": " << S.castray.hits << ", \"mean_hit_dist\": " << S.castray.mean_hit_dist
               << ", \"origin_occupied\": " << S.castray.origin_occupied << "}, \"hierarchical\": {\"us_per_ray\": "
               << S.hier.us_per_ray << ", \"hits\": " << S.hier.hits << ", \"mean_hit_dist\": " << S.hier.mean_hit_dist
               << ", \"cells_per_ray\": " << S.hier.mean_cells_visited << "}, \"hit_agreement\": " << S.hit_agreement
               << ", \"hit_dist_diff_median\": " << S.hit_dist_diff_med << "}";
        }
        js << "}, \"load_ms_median\": " << r.load_ms << "}" << (m + 1 < results.size() ? "," : "") << "\n";
    }
    js << "  ]\n}\n";
    std::cout << "[INFO] JSON written to " << json_path << std::endl;
    return 0;
}
