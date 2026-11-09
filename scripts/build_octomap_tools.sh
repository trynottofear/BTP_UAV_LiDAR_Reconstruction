#!/usr/bin/env bash
# Builds the offline OctoMap tools (uniform converter, adaptive generator, benchmark).
set -euo pipefail
cd "$(dirname "$0")"
FLAGS="-O3 -std=c++17 -fopenmp -I/usr/include/pcl-1.14 -I/usr/include/eigen3"
LIBS="-lpcl_io -lpcl_common -lpcl_features -lpcl_search -lpcl_kdtree -llz4 -loctomap -loctomath"
g++ $FLAGS pcd_to_octomap.cpp        -o pcd_to_octomap        $LIBS
g++ $FLAGS dynamic_voxel_octomap.cpp -o dynamic_voxel_octomap $LIBS
g++ $FLAGS benchmark_octomap.cpp     -o benchmark_octomap     $LIBS
echo "Built: pcd_to_octomap dynamic_voxel_octomap benchmark_octomap"
