#!/usr/bin/env bash
# Runs FAST-LIO2 (Livox Avia config) on a converted ROS 2 bag and collects its outputs.
#
# Usage: scripts/run_fastlio_bag.sh <run_name> [play_rate=1.0]
# Reads:    datasets/<run>/            (output of scripts/convert_mars_bag.py)
# Produces: pointclouds/<run>/{scans.pcd,mat_pre.txt,fastlio.log}
# FAST-LIO2 writes the map PCD and state log inside its package dir on shutdown; this script
# shuts it down with SIGINT and then moves those files into pointclouds/<run>/.
set -uo pipefail
RUN="${1:?usage: $0 <run_name> [play_rate]}"
RATE="${2:-1.0}"
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"
set +u; source /opt/ros/jazzy/setup.bash; source install/setup.bash; set -u

BAG="datasets/$RUN"
OUT="pointclouds/$RUN"
FL="src/FAST_LIO_ROS2"
[ -d "$BAG" ] || { echo "missing $BAG" >&2; exit 1; }
mkdir -p "$OUT"
rm -f "$FL/PCD/scans.pcd" "$FL/Log/mat_pre.txt"

setsid ros2 launch fast_lio mapping.launch.py config_file:=avia.yaml rviz:=false > "$OUT/fastlio.log" 2>&1 &
LAUNCH=$!
sleep 8
echo "[$(date +%T)] FAST-LIO2 up (pid $LAUNCH); playing $BAG at rate $RATE"
ros2 bag play "$BAG" --rate "$RATE" --read-ahead-queue-size 20000 > "$OUT/bag_play.log" 2>&1
echo "[$(date +%T)] bag finished; letting FAST-LIO2 drain, then SIGINT"
sleep 5
kill -INT -- -"$(ps -o pgid= -p $LAUNCH | tr -d ' ')" 2>/dev/null
# wait for the node to write the PCD and exit
for _ in $(seq 1 120); do pgrep -f fastlio_mapping >/dev/null || break; sleep 2; done
pkill -INT -f fastlio_mapping 2>/dev/null; sleep 2

[ -f "$FL/PCD/scans.pcd" ] || { echo "FAST-LIO2 produced no scans.pcd; see $OUT/fastlio.log" >&2; exit 2; }
mv "$FL/PCD/scans.pcd" "$OUT/scans.pcd"
mv "$FL/Log/mat_pre.txt" "$OUT/mat_pre.txt"
echo "[$(date +%T)] collected: $(wc -l < "$OUT/mat_pre.txt") poses, $(du -h "$OUT/scans.pcd" | cut -f1) map"
