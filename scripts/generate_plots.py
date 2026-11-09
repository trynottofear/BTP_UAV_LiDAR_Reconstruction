import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import os

os.makedirs('Reports/figures', exist_ok=True)

# Set global academic style
plt.rcParams.update({
    'font.family': 'serif',
    'font.size': 11,
    'axes.labelsize': 12,
    'axes.titlesize': 13,
    'xtick.labelsize': 11,
    'ytick.labelsize': 11,
    'legend.fontsize': 10.5,
    'figure.titlesize': 14,
    'figure.dpi': 300
})

models = ['Uniform 0.40m', 'Uniform 0.20m', 'Uniform 0.10m', 'Dynamic Adaptive\n(0.05m–0.40m)']
file_sizes_kb = [21.7, 84.3, 320.9, 334.7]
total_nodes = [41343, 157247, 531026, 1370457]
leaf_voxels = [30299, 114165, 366772, 1199150]
raycast_latency_us = [4.356, 7.976, 14.587, 0.119]
chamfer_dist_m = [0.1623, 0.0878, 0.0472, 0.1890]

# =========================================================================
# Figure 1: Raycasting Collision Query Latency (Log Scale & Linear)
# =========================================================================
fig, ax = plt.subplots(figsize=(7.5, 4.5))
colors = ['#4682B4', '#5F9EA0', '#6495ED', '#D9534F']
bars = ax.bar(models, raycast_latency_us, color=colors, width=0.55, edgecolor='black', linewidth=1.2)

ax.set_ylabel(r'Raycast Latency ($\mu$s / query) [Log Scale]')
ax.set_title('Obstacle Collision Query Latency Across Octree Representations')
ax.set_yscale('log')
ax.set_ylim(0.05, 30)
ax.grid(axis='y', linestyle='--', alpha=0.7)

# Annotate values
for bar, val in zip(bars, raycast_latency_us):
    yval = bar.get_height()
    if val < 1.0:
        ax.text(bar.get_x() + bar.get_width()/2.0, yval * 1.5, f'{val:.3f} $\mu$s\n(122x Faster)', 
                ha='center', va='bottom', fontweight='bold', color='#B22222')
    else:
        ax.text(bar.get_x() + bar.get_width()/2.0, yval * 1.15, f'{val:.2f} $\mu$s', 
                ha='center', va='bottom', fontweight='medium')

plt.tight_layout()
plt.savefig('Reports/figures/raycast_latency_comparison.png', dpi=300)
plt.close()
print("Saved raycast_latency_comparison.png")

# =========================================================================
# Figure 2: Total Nodes vs File Size Trade-Off
# =========================================================================
fig, ax1 = plt.subplots(figsize=(8, 4.8))

x = np.arange(len(models))
width = 0.35

color1 = '#2B5B84'
color2 = '#E67E22'

rects1 = ax1.bar(x - width/2, [n / 1000.0 for n in total_nodes], width, label='Total Nodes (x10³)', 
                 color=color1, edgecolor='black', linewidth=1)
ax1.set_ylabel('Total Nodes (Thousands)', color=color1, fontweight='bold')
ax1.tick_params(axis='y', labelcolor=color1)
ax1.set_xticks(x)
ax1.set_xticklabels(models)
ax1.set_ylim(0, 1600)

ax2 = ax1.twinx()
rects2 = ax2.bar(x + width/2, file_sizes_kb, width, label='File Size (KB)', 
                 color=color2, edgecolor='black', linewidth=1)
ax2.set_ylabel('File Size on Disk (KB)', color=color2, fontweight='bold')
ax2.tick_params(axis='y', labelcolor=color2)
ax2.set_ylim(0, 420)

plt.title('Octree Complexity vs Binary Storage Footprint')
ax1.grid(axis='y', linestyle=':', alpha=0.6)

# Annotate Dynamic representation efficiency
ax1.annotate('1,370,457 nodes\n(2.58x detail of 0.1m)', 
             xy=(3 - width/2, 1370.457), xytext=(2.2, 1420),
             arrowprops=dict(facecolor='black', shrink=0.08, width=1, headwidth=6),
             fontweight='bold', fontsize=9.5)

ax2.annotate('334.7 KB\n(+4.3% storage)', 
             xy=(3 + width/2, 334.7), xytext=(2.5, 360),
             arrowprops=dict(facecolor='black', shrink=0.08, width=1, headwidth=6),
             fontweight='bold', fontsize=9.5, color='#B22222')

plt.tight_layout()
plt.savefig('Reports/figures/node_count_storage_tradeoff.png', dpi=300)
plt.close()
print("Saved node_count_storage_tradeoff.png")

# =========================================================================
# Figure 3: Point Cloud Adaptive Depth Distribution
# =========================================================================
depth_labels = [
    'Depth 13 (0.40m)\nDistant Runway/Ground',
    'Depth 14 (0.20m)\nNear-Field Ground',
    'Depth 15 (0.10m)\nBuilding Facades',
    'Depth 16 (0.05m)\nThin Edges/Cables/Poles'
]
depth_points = [572302, 234735, 346764, 753260]
depth_pcts = [30.01, 12.31, 18.18, 39.50]
palette = ['#85C1E9', '#5DADE2', '#3498DB', '#1B4F72']

fig, ax = plt.subplots(figsize=(8, 4.2))
bars = ax.barh(depth_labels, depth_pcts, color=palette, edgecolor='black', linewidth=1.1, height=0.55)

ax.set_xlabel('Point Distribution Percentage (%)')
ax.set_title('Spatial Point Cloud Distribution Across Adaptive Octree Depths (1,907,061 Points)')
ax.set_xlim(0, 48)
ax.grid(axis='x', linestyle='--', alpha=0.7)

for bar, pt, pct in zip(bars, depth_points, depth_pcts):
    w = bar.get_width()
    ax.text(w + 1.0, bar.get_y() + bar.get_height()/2.0, 
            f'{pct:.1f}% ({pt:,} pts)', va='center', fontweight='bold', fontsize=10)

plt.tight_layout()
plt.savefig('Reports/figures/depth_distribution.png', dpi=300)
plt.close()
print("Saved depth_distribution.png")

# =========================================================================
# Figure 4: Chamfer Distance vs Collision Checking Latency Tradeoff
# =========================================================================
fig, ax = plt.subplots(figsize=(7.5, 4.8))

for i, (m, c, r, s) in enumerate(zip(models, chamfer_dist_m, raycast_latency_us, file_sizes_kb)):
    color = '#D9534F' if 'Dynamic' in m else '#2B5B84'
    ax.scatter(c, r, s=s*2.5, color=color, alpha=0.85, edgecolors='black', linewidth=1.5, zorder=5)
    offset_x = 0.005 if i != 3 else -0.045
    offset_y = 0.8 if i != 3 else 0.05
    label_txt = m.replace('\n', ' ') + f' ({s:.1f} KB)'
    ax.annotate(label_txt, (c, r), xytext=(c + offset_x, r + offset_y),
                fontweight='bold' if 'Dynamic' in m else 'normal', fontsize=9.5)

ax.set_xlabel('Chamfer Distance $d_{CD}$ to Ground Truth (m)')
ax.set_ylabel(r'Raycast Latency ($\mu$s / query)')
ax.set_title('Pareto Frontier: Surface Discrepancy vs Collision Checking Latency')
ax.grid(True, linestyle='--', alpha=0.6)
ax.set_yscale('log')
ax.set_ylim(0.08, 25)
ax.set_xlim(0.02, 0.22)

# Size legend annotation
ax.text(0.03, 0.15, 'Circle area $\propto$ File Size (KB)', style='italic', fontsize=10,
        bbox=dict(boxstyle='round,pad=0.5', facecolor='#F2F3F4', alpha=0.8))

plt.tight_layout()
plt.savefig('Reports/figures/chamfer_vs_latency_pareto.png', dpi=300)
plt.close()
print("Saved chamfer_vs_latency_pareto.png")

