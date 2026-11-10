#!/usr/bin/env bash
# Offline stages for one run: FAST-LIO2 outputs -> uniform + adaptive OctoMaps -> benchmark -> figures.
#
# Usage: scripts/run_offline_pipeline.sh <run_name> [--keep-traj-poses]
# Expects:  pointclouds/<run>/scans.pcd and pointclouds/<run>/mat_pre.txt (FAST-LIO2 map and per-scan states)
# Produces: maps/<run>/uniform/*.bt, maps/<run>/dynamic/*.bt|.ot,
#           Reports/benchmark/<run>/{benchmark_results.json,dynamic_octomap_stats.json,benchmark_octomap_results.md,logs/},
#           Reports/figures/<run>/*.png
set -euo pipefail
RUN="${1:?usage: $0 <run_name>}"
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"

PCD="pointclouds/$RUN/scans.pcd"
TRAJ="pointclouds/$RUN/mat_pre.txt"
MAPS="maps/$RUN"
BENCH="Reports/benchmark/$RUN"
LOGS="$BENCH/logs"
[ -f "$PCD" ] && [ -f "$TRAJ" ] || { echo "missing $PCD or $TRAJ" >&2; exit 1; }
mkdir -p "$MAPS/uniform" "$MAPS/dynamic" "$LOGS"

t() { date +%s; }
stage() { echo; echo "=== [$(date +%T)] $1"; }

stage "Uniform OctoMaps"
for r in 0.4 0.2 0.1 0.05; do
  S=$(t)
  scripts/pcd_to_octomap "$PCD" "$MAPS/uniform/uav_octomap_${r}m.bt" "$r" 2>&1 | tee "$LOGS/pcd_to_octomap_${r}m.log" | tail -n 3
  echo "  -> ${r} m done in $(( $(t) - S )) s"
done

stage "Adaptive (dynamic voxel) OctoMap"
S=$(t)
scripts/dynamic_voxel_octomap "$PCD" "$MAPS/dynamic/dynamic_octomap.bt" --traj "$TRAJ" \
  --stats "$BENCH/dynamic_octomap_stats.json" 2>&1 | tee "$LOGS/dynamic_voxel_octomap.log" | tail -n 25
echo "  -> adaptive done in $(( $(t) - S )) s"

stage "Benchmark"
S=$(t)
scripts/benchmark_octomap --pcd "$PCD" --traj "$TRAJ" \
  --json "$BENCH/benchmark_results.json" --md "$BENCH/benchmark_octomap_results.md" \
  "Uniform 0.40 m=$MAPS/uniform/uav_octomap_0.4m.bt" "Uniform 0.20 m=$MAPS/uniform/uav_octomap_0.2m.bt" \
  "Uniform 0.10 m=$MAPS/uniform/uav_octomap_0.1m.bt" "Uniform 0.05 m=$MAPS/uniform/uav_octomap_0.05m.bt" \
  "Dynamic Adaptive=$MAPS/dynamic/dynamic_octomap.bt" 2>&1 | tee "$LOGS/benchmark_octomap.log" | tail -n 40
echo "  -> benchmark done in $(( $(t) - S )) s"

stage "Figures"
PY=venv/bin/python; [ -x "$PY" ] || PY=python3
"$PY" scripts/generate_plots.py --run "$RUN"
echo; echo "Done: $RUN"
