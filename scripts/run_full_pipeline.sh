#!/usr/bin/env bash
# End-to-end run: ROS 1 bag -> ROS 2 bag -> FAST-LIO2 -> uniform + adaptive OctoMaps -> benchmark -> figures.
#
# Usage: scripts/run_full_pipeline.sh <run_name> <source.bag> [--duration SEC] [--rate 0.5]
# All outputs go to per-run folders (see run_fastlio_bag.sh and run_offline_pipeline.sh). A stage whose output
# already exists is skipped, so a failed run can be resumed by re-running the same command.
set -euo pipefail
RUN="${1:?usage: $0 <run_name> <source.bag> [--duration SEC] [--rate R]}"; BAG="${2:?source bag}"; shift 2
DURATION=""; RATE="0.5"
while [ $# -gt 0 ]; do case "$1" in --duration) DURATION="$2"; shift 2;; --rate) RATE="$2"; shift 2;; *) echo "bad arg $1" >&2; exit 1;; esac; done
ROOT="$(cd "$(dirname "$0")/.." && pwd)"; cd "$ROOT"
PY=venv/bin/python
ts() { date '+%F %T'; }

echo "[$(ts)] == 1/3 convert ROS1 -> ROS2"
if [ -f "datasets/$RUN/metadata.yaml" ]; then echo "  datasets/$RUN exists, skipping"; else
  $PY scripts/convert_mars_bag.py "$BAG" "datasets/$RUN" ${DURATION:+--duration "$DURATION"}
fi

echo "[$(ts)] == 2/3 FAST-LIO2 at ${RATE}x replay"
if [ -f "pointclouds/$RUN/scans.pcd" ] && [ -f "pointclouds/$RUN/mat_pre.txt" ]; then echo "  pointclouds/$RUN exists, skipping"; else
  scripts/run_fastlio_bag.sh "$RUN" "$RATE"
fi

echo "[$(ts)] -- trajectory sanity gate"
$PY - "$RUN" <<'PY'
import sys, numpy as np
a = np.loadtxt(f"pointclouds/{sys.argv[1]}/mat_pre.txt"); t = a[:, 0]; p = a[:, 4:7]
dt = np.diff(t); v = np.linalg.norm(np.diff(p, axis=0), axis=1) / np.maximum(dt, 1e-6)
print(f"  poses {len(a)}, span {t[-1]-t[0]:.1f} s, path {np.linalg.norm(np.diff(p,axis=0),axis=1).sum():.0f} m, "
      f"speed median {np.median(v):.2f} p99 {np.percentile(v,99):.2f} max {v.max():.2f} m/s, max dt {dt.max():.2f} s")
if v.max() > 25 or dt.max() > 1.0 or np.abs(p).max() > 5000:
    print("  TRAJECTORY LOOKS DIVERGED - stopping before building maps"); sys.exit(3)
PY

echo "[$(ts)] == 3/3 maps, benchmark, figures"
scripts/run_offline_pipeline.sh "$RUN"
echo "[$(ts)] ALL DONE: $RUN"
