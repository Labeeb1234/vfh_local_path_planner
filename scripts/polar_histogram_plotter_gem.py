import re
import os
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np

# Configuration
LOG_FILE_PATH = "/home/inlabust/labeeb/mr_plan_ws/obstacle_logs.txt"
ALPHA_RES = 60
UPDATE_INTERVAL_MS = 1000  # 1000ms = 1 second (Low update rate)

# Initialize the figure and subplots
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 7))

# --- Left Plot Setup ---
scatter1 = ax1.scatter([], [], color='mediumblue', s=45, edgecolors='k', zorder=3)
ax1.set_title("Obstacle Distance ($r$) vs Histogram Index", fontsize=12)
ax1.set_xlabel("Histogram Bin Index (`pol_ind`)")
ax1.set_ylabel("Distance $r$ (meters)")
ax1.set_xlim(-1, ALPHA_RES)
ax1.set_xticks(np.arange(0, ALPHA_RES + 1, 5))
ax1.grid(True, linestyle='--', alpha=0.6)

# --- Right Plot Setup (Polar Mapping) ---
ax2 = plt.subplot(1, 2, 2, projection='polar')
scatter2 = ax2.scatter([], [], color='crimson', s=45, edgecolors='k', zorder=3)
ax2.set_title("ENU Grid Layout\n(Bin 0 = West [-180°], Bin 30 = East [0°])", va='bottom', y=1.05)

# Set up spatial ENU frame directions
ax2.set_theta_zero_location("E")  # Geometric 0 degrees is East (Right side)
ax2.set_theta_direction(1)        # Counter-clockwise rotation (North is Up at 90 deg)

# Create 60 distinct tick intervals spanning the full 0 to 360 degree space
ticks_deg = np.linspace(0, 360, ALPHA_RES, endpoint=False)
labels = []
for d in ticks_deg:
    wrapped_angle = d if d <= 180 else d - 360
    if wrapped_angle % 30 == 0:
        labels.append(f"{int(wrapped_angle)}°")
    else:
        labels.append("")

ax2.set_thetagrids(ticks_deg, labels=labels)
ax2.grid(True, which='major', linestyle='-', color='gray', alpha=0.4)

# Regex compiler for performance
pattern = re.compile(r"Vector:\s*([-\d.]+),\s*(\d+)")

def update(frame):
    """Callback function executed every UPDATE_INTERVAL_MS"""
    if not os.path.exists(LOG_FILE_PATH):
        return scatter1, scatter2

    r_values, index_values = [], []
    
    # Read and parse file
    with open(LOG_FILE_PATH, 'r') as f:
        for line in f:
            match = pattern.search(line)
            if match:
                r_values.append(float(match.group(1)))
                index_values.append(int(float(match.group(2))))
                
    if not r_values:
        return scatter1, scatter2

    r_arr = np.array(r_values)
    idx_arr = np.array(index_values)

    # 1. Update Left Plot Data
    # Stack Nx2 data points array for scatter representation
    scatter1.set_offsets(np.column_stack((idx_arr, r_arr)))
    
    # 2. Update Right Plot Data
    angles_rad = ((idx_arr / ALPHA_RES) * 2 * np.pi) + np.pi
    scatter2.set_offsets(np.column_stack((angles_rad, r_arr)))
    
    # Dynamically adjust radial scale limit based on maximum incoming distance value
    max_r = r_arr.max()
    ax1.set_ylim(0, max_r * 1.2)
    ax2.set_ylim(0, max_r * 1.2)

    return scatter1, scatter2

# Run the live loop
# blit=False is used because we dynamically update axis limits (ylim)
ani = FuncAnimation(fig, update, interval=UPDATE_INTERVAL_MS, blit=False, cache_frame_data=False)

plt.tight_layout()
plt.show()