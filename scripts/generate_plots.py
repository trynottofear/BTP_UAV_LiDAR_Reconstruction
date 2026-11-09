#!/usr/bin/env python3
"""Generates the report figures from the benchmark outputs (no hard-coded numbers).

Inputs:  Reports/benchmark/benchmark_results.json   (scripts/benchmark_octomap)
         Reports/benchmark/dynamic_octomap_stats.json (scripts/dynamic_voxel_octomap --stats)
Outputs: Reports/figures/*.png
"""
import json
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BENCH = os.path.join(ROOT, "Reports", "benchmark", "benchmark_results.json")
STATS = os.path.join(ROOT, "Reports", "benchmark", "dynamic_octomap_stats.json")
OUT = os.path.join(ROOT, "Reports", "figures")
os.makedirs(OUT, exist_ok=True)

# Palette (validated reference categorical slots 1-2) and neutral inks.
C1, C2 = "#2a78d6", "#eb6834"
INK, INK2, GRID = "#0b0b0b", "#52514e", "#e4e3df"

plt.rcParams.update({
    "font.family": "serif", "font.size": 10, "axes.labelsize": 10, "axes.titlesize": 10.5,
    "xtick.labelsize": 9, "ytick.labelsize": 9, "legend.fontsize": 9, "figure.dpi": 300,
    "axes.edgecolor": INK2, "axes.labelcolor": INK, "xtick.color": INK2, "ytick.color": INK2,
    "axes.spines.top": False, "axes.spines.right": False, "axes.grid": True, "grid.color": GRID,
    "grid.linewidth": 0.6, "axes.axisbelow": True, "legend.frameon": False,
})

with open(BENCH) as f:
    bench = json.load(f)
with open(STATS) as f:
    stats = json.load(f)

maps = [m for m in bench["maps"] if not m["name"].startswith("Legacy")]
by_name = {m["name"]: m for m in maps}
uniform = sorted([m for m in maps if m["name"].startswith("Uniform")], key=lambda m: -m["resolution"])
dynamic = by_name["Dynamic Adaptive"]
labels = [m["name"].replace("Uniform ", "Uniform\n").replace("Dynamic Adaptive", "Adaptive\n0.05-0.40 m") for m in maps]


short_labels = [m["name"].replace("Uniform ", "").replace(" m", "").replace("Dynamic Adaptive", "Adaptive") for m in maps]


def save(fig, name):
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, name), dpi=300)
    plt.close(fig)
    print("Saved", name)


def bar_labels(ax, bars, fmt):
    for b in bars:
        ax.annotate(fmt(b.get_height()), (b.get_x() + b.get_width() / 2, b.get_height()),
                    xytext=(0, 2), textcoords="offset points", ha="center", va="bottom", fontsize=7.5, color=INK)


# ---------------------------------------------------------------------------------------------
# Figure 1: ray-cast latency, two ray sets (small multiples), castRay vs hierarchical traversal.
# ---------------------------------------------------------------------------------------------
fig, axes = plt.subplots(1, 2, figsize=(8.6, 3.6), sharey=True)
titles = {"omni": "Omnidirectional rays (mostly misses)", "surface": "Surface-directed rays (mostly hits)"}
x = range(len(maps))
w = 0.38
for ax, key in zip(axes, ["omni", "surface"]):
    a = [m["rays"][key]["castray"]["us_per_ray"] for m in maps]
    b = [m["rays"][key]["hierarchical"]["us_per_ray"] for m in maps]
    b1 = ax.bar([i - w / 2 - 0.01 for i in x], a, w, color=C1, label="OctoMap castRay()")
    b2 = ax.bar([i + w / 2 + 0.01 for i in x], b, w, color=C2, label="Hierarchical traversal")
    bar_labels(ax, b1, lambda v: f"{v:.1f}")
    bar_labels(ax, b2, lambda v: f"{v:.2f}")
    ax.set_yscale("log")
    ax.set_ylim(0.2, 40)
    ax.set_xticks(list(x))
    ax.set_xticklabels(labels)
    ax.set_title(titles[key], color=INK)
    ax.grid(axis="x", visible=False)
axes[0].set_ylabel(r"Mean latency ($\mu$s / ray, log scale)")
axes[0].legend(loc="upper left")
save(fig, "raycast_latency_comparison.png")

# ---------------------------------------------------------------------------------------------
# Figure 2: storage footprint (small multiples, one axis each).
# ---------------------------------------------------------------------------------------------
fig, axes = plt.subplots(1, 3, figsize=(9.0, 3.0))
panels = [("file_kb", "Binary file (.bt), KB", lambda v: f"{v:,.0f}"),
          ("memory_bytes", "In-memory tree, MB", lambda v: f"{v:,.1f}"),
          ("occupied_leaves", "Occupied leaves, thousands", lambda v: f"{v:,.0f}")]
scale = {"file_kb": 1.0, "memory_bytes": 1.0 / 2**20, "occupied_leaves": 1e-3}
for ax, (key, title, fmt) in zip(axes, panels):
    vals = [m[key] * scale[key] for m in maps]
    colors = [C2 if m is dynamic else C1 for m in maps]
    bars = ax.bar(list(x), vals, 0.62, color=colors)
    bar_labels(ax, bars, fmt)
    ax.set_title(title, color=INK)
    ax.set_xticks(list(x))
    ax.set_xticklabels(short_labels, fontsize=8.5)
    ax.set_ylim(0, max(vals) * 1.18)
    ax.set_xlabel("Uniform voxel size (m) / adaptive", fontsize=8, color=INK2)
    ax.grid(axis="x", visible=False)
save(fig, "storage_footprint.png")

# ---------------------------------------------------------------------------------------------
# Figure 3: accuracy vs storage, split by point class (edge / planar).
# ---------------------------------------------------------------------------------------------
fig, axes = plt.subplots(1, 2, figsize=(8.6, 3.5), sharey=True)
for ax, key, title in [(axes[0], "chamfer_c2m_edge", "Edge / thin-structure points ($\\sigma_n \\geq 0.06$)"),
                       (axes[1], "chamfer_c2m_planar", "Planar points ($\\sigma_n < 0.025$)")]:
    ux = [m["file_kb"] for m in uniform]
    uy = [m[key] for m in uniform]
    ax.plot(ux, uy, "-o", color=C1, lw=2, ms=6, label="Uniform OctoMap", zorder=3,
            markeredgecolor="white", markeredgewidth=1.5)
    for m in uniform:
        ax.annotate(m["name"].replace("Uniform ", ""), (m["file_kb"], m[key]), xytext=(5, 4),
                    textcoords="offset points", fontsize=8, color=INK2)
    ax.plot([dynamic["file_kb"]], [dynamic[key]], "D", color=C2, ms=8, label="Adaptive (this work)", zorder=4,
            markeredgecolor="white", markeredgewidth=1.5)
    ax.annotate(f"Adaptive\n{dynamic[key]:.3f} m", (dynamic["file_kb"], dynamic[key]), xytext=(-62, 10),
                textcoords="offset points", fontsize=8, color=INK)
    ax.set_xscale("log")
    ax.set_xlabel("Binary file size (KB, log scale)")
    ax.set_title(title, color=INK)
axes[0].set_ylabel("Cloud-to-map distance (m)")
axes[0].legend(loc="upper right")
save(fig, "accuracy_vs_storage.png")

# ---------------------------------------------------------------------------------------------
# Figure 4: effective voxel size at edge vs planar points.
# ---------------------------------------------------------------------------------------------
fig, ax = plt.subplots(figsize=(7.2, 3.3))
e = [m["voxel_at_edge_m"] for m in maps]
p = [m["voxel_at_planar_m"] for m in maps]
b1 = ax.bar([i - w / 2 - 0.01 for i in x], e, w, color=C1, label="Edge / thin-structure points")
b2 = ax.bar([i + w / 2 + 0.01 for i in x], p, w, color=C2, label="Planar points")
bar_labels(ax, b1, lambda v: f"{v:.3f}")
bar_labels(ax, b2, lambda v: f"{v:.3f}")
ax.set_ylabel("Mean edge length of the voxel\ncontaining the point (m)")
ax.set_xticks(list(x))
ax.set_xticklabels(labels)
ax.set_ylim(0, max(e + p) * 1.15)
ax.legend(loc="upper right")
ax.grid(axis="x", visible=False)
save(fig, "effective_voxel_size.png")

# ---------------------------------------------------------------------------------------------
# Figure 5: adaptive depth distribution (points requested / inserted, leaves written).
# ---------------------------------------------------------------------------------------------
res0, dmax = stats["base_resolution"], stats["d_max"]
depths = sorted({int(d) for d in stats["requested_depth_points"]} | {int(d) for d in stats["insertion_depth_points"]})
dlabels = [f"Depth {d}\n{res0 * 2 ** (dmax - d):.2f} m" for d in depths]
fig, axes = plt.subplots(1, 2, figsize=(8.6, 3.2))
req = [stats["requested_depth_points"].get(str(d), 0) for d in depths]
ins = [stats["insertion_depth_points"].get(str(d), 0) for d in depths]
tot = sum(req)
yy = range(len(depths))
h = 0.38
r1 = axes[0].barh([i + h / 2 + 0.01 for i in yy], [100 * v / tot for v in req], h, color=C1, label="Requested by policy")
r2 = axes[0].barh([i - h / 2 - 0.01 for i in yy], [100 * v / tot for v in ins], h, color=C2,
                  label="Inserted (after refinement)")
for bars in (r1, r2):
    for b in bars:
        axes[0].annotate(f"{b.get_width():.1f}%", (b.get_width(), b.get_y() + b.get_height() / 2), xytext=(3, 0),
                         textcoords="offset points", va="center", fontsize=7.5, color=INK)
axes[0].set_yticks(list(yy))
axes[0].set_yticklabels(dlabels)
axes[0].set_xlabel(f"Share of {tot:,} points (%)")
axes[0].set_xlim(0, 88)
axes[0].set_title("Points per octree depth", color=INK)
axes[0].legend(loc="center right")
axes[0].grid(axis="y", visible=False)

leaf = stats["occupied_leaf_depths"]
ld = sorted(int(d) for d in leaf)
lv = [leaf[str(d)] / 1e3 for d in ld]
bars = axes[1].barh(range(len(ld)), lv, 0.6, color=C2)
for b in bars:
    axes[1].annotate(f"{b.get_width():,.1f}k", (b.get_width(), b.get_y() + b.get_height() / 2), xytext=(3, 0),
                     textcoords="offset points", va="center", fontsize=7.5, color=INK)
axes[1].set_yticks(range(len(ld)))
axes[1].set_yticklabels([f"Depth {d}\n{res0 * 2 ** (dmax - d):.2f} m" for d in ld])
axes[1].set_xlabel("Occupied leaves (thousands)")
axes[1].set_xlim(0, max(lv) * 1.2)
axes[1].set_title("Occupied leaves per depth (after pruning)", color=INK)
axes[1].grid(axis="y", visible=False)
save(fig, "depth_distribution.png")
