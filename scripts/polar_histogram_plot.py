import re
import matplotlib.pyplot as plt
import numpy as np

# Your raw log data
log_data = """
[local_costmap_viewer_node-2] [INFO] [1783406472.365192623] [local_costmap]: Hisogram Value of Sector(0): [411324.750000]
[local_costmap_viewer_node-2] [INFO] [1783406472.365214631] [local_costmap]: Hisogram Value of Sector(1): [410387.937500]
[local_costmap_viewer_node-2] [INFO] [1783406472.365223678] [local_costmap]: Hisogram Value of Sector(2): [407026.062500]
[local_costmap_viewer_node-2] [INFO] [1783406472.365230750] [local_costmap]: Hisogram Value of Sector(3): [403345.093750]
[local_costmap_viewer_node-2] [INFO] [1783406472.365237518] [local_costmap]: Hisogram Value of Sector(4): [406734.687500]
[local_costmap_viewer_node-2] [INFO] [1783406472.365244050] [local_costmap]: Hisogram Value of Sector(5): [406176.312500]
[local_costmap_viewer_node-2] [INFO] [1783406472.365250364] [local_costmap]: Hisogram Value of Sector(6): [399584.500000]
[local_costmap_viewer_node-2] [INFO] [1783406472.365256690] [local_costmap]: Hisogram Value of Sector(7): [399335.687500]
[local_costmap_viewer_node-2] [INFO] [1783406472.365263231] [local_costmap]: Hisogram Value of Sector(8): [399787.781250]
[local_costmap_viewer_node-2] [INFO] [1783406472.365269917] [local_costmap]: Hisogram Value of Sector(9): [393214.000000]
[local_costmap_viewer_node-2] [INFO] [1783406472.365276280] [local_costmap]: Hisogram Value of Sector(10): [375957.031250]
[local_costmap_viewer_node-2] [INFO] [1783406472.365282706] [local_costmap]: Hisogram Value of Sector(11): [0.000000]
[local_costmap_viewer_node-2] [INFO] [1783406472.365288891] [local_costmap]: Hisogram Value of Sector(12): [0.000000]
[local_costmap_viewer_node-2] [INFO] [1783406472.365295075] [local_costmap]: Hisogram Value of Sector(13): [0.000000]
[local_costmap_viewer_node-2] [INFO] [1783406472.365300841] [local_costmap]: Hisogram Value of Sector(14): [0.000000]
[local_costmap_viewer_node-2] [INFO] [1783406472.365320310] [local_costmap]: Hisogram Value of Sector(15): [0.000000]
[local_costmap_viewer_node-2] [INFO] [1783406472.365327843] [local_costmap]: Hisogram Value of Sector(16): [0.000000]
[local_costmap_viewer_node-2] [INFO] [1783406472.365334005] [local_costmap]: Hisogram Value of Sector(17): [0.000000]
[local_costmap_viewer_node-2] [INFO] [1783406472.365339871] [local_costmap]: Hisogram Value of Sector(18): [0.000000]
[local_costmap_viewer_node-2] [INFO] [1783406472.365345611] [local_costmap]: Hisogram Value of Sector(19): [0.000000]
[local_costmap_viewer_node-2] [INFO] [1783406472.365351807] [local_costmap]: Hisogram Value of Sector(20): [0.000000]
[local_costmap_viewer_node-2] [INFO] [1783406472.365357888] [local_costmap]: Hisogram Value of Sector(21): [0.000000]
[local_costmap_viewer_node-2] [INFO] [1783406472.365363829] [local_costmap]: Hisogram Value of Sector(22): [0.000000]
[local_costmap_viewer_node-2] [INFO] [1783406472.365369739] [local_costmap]: Hisogram Value of Sector(23): [0.000000]
[local_costmap_viewer_node-2] [INFO] [1783406472.365375527] [local_costmap]: Hisogram Value of Sector(24): [0.000000]
[local_costmap_viewer_node-2] [INFO] [1783406472.365381343] [local_costmap]: Hisogram Value of Sector(25): [0.000000]
[local_costmap_viewer_node-2] [INFO] [1783406472.365387201] [local_costmap]: Hisogram Value of Sector(26): [0.000000]
[local_costmap_viewer_node-2] [INFO] [1783406472.365392974] [local_costmap]: Hisogram Value of Sector(27): [0.000000]
[local_costmap_viewer_node-2] [INFO] [1783406472.365398755] [local_costmap]: Hisogram Value of Sector(28): [0.000000]
[local_costmap_viewer_node-2] [INFO] [1783406472.365404836] [local_costmap]: Hisogram Value of Sector(29): [317314.625000]
[local_costmap_viewer_node-2] [INFO] [1783406472.365411371] [local_costmap]: Hisogram Value of Sector(30): [391838.875000]
[local_costmap_viewer_node-2] [INFO] [1783406472.365417654] [local_costmap]: Hisogram Value of Sector(31): [407169.875000]
[local_costmap_viewer_node-2] [INFO] [1783406472.365423982] [local_costmap]: Hisogram Value of Sector(32): [412452.000000]
[local_costmap_viewer_node-2] [INFO] [1783406472.365430247] [local_costmap]: Hisogram Value of Sector(33): [413445.906250]
[local_costmap_viewer_node-2] [INFO] [1783406472.365436373] [local_costmap]: Hisogram Value of Sector(34): [403672.593750]
[local_costmap_viewer_node-2] [INFO] [1783406472.365442689] [local_costmap]: Hisogram Value of Sector(35): [384990.906250]
[local_costmap_viewer_node-2] [INFO] [1783406472.365451971] [local_costmap]: Hisogram Value of Sector(36): [419536.843750]
[local_costmap_viewer_node-2] [INFO] [1783406472.365458553] [local_costmap]: Hisogram Value of Sector(37): [0.000000]
[local_costmap_viewer_node-2] [INFO] [1783406472.365464406] [local_costmap]: Hisogram Value of Sector(38): [0.000000]
[local_costmap_viewer_node-2] [INFO] [1783406472.365470112] [local_costmap]: Hisogram Value of Sector(39): [0.000000]
[local_costmap_viewer_node-2] [INFO] [1783406472.365475839] [local_costmap]: Hisogram Value of Sector(40): [0.000000]
[local_costmap_viewer_node-2] [INFO] [1783406472.365481446] [local_costmap]: Hisogram Value of Sector(41): [0.000000]
[local_costmap_viewer_node-2] [INFO] [1783406472.365487006] [local_costmap]: Hisogram Value of Sector(42): [0.000000]
[local_costmap_viewer_node-2] [INFO] [1783406472.365492609] [local_costmap]: Hisogram Value of Sector(43): [393806.000000]
[local_costmap_viewer_node-2] [INFO] [1783406472.365499046] [local_costmap]: Hisogram Value of Sector(44): [406375.375000]
[local_costmap_viewer_node-2] [INFO] [1783406472.365505248] [local_costmap]: Hisogram Value of Sector(45): [406091.718750]
[local_costmap_viewer_node-2] [INFO] [1783406472.365512505] [local_costmap]: Hisogram Value of Sector(46): [409027.562500]
[local_costmap_viewer_node-2] [INFO] [1783406472.365518979] [local_costmap]: Hisogram Value of Sector(47): [410113.937500]
[local_costmap_viewer_node-2] [INFO] [1783406472.365525181] [local_costmap]: Hisogram Value of Sector(48): [411058.031250]
[local_costmap_viewer_node-2] [INFO] [1783406472.365531390] [local_costmap]: Hisogram Value of Sector(49): [410089.250000]
[local_costmap_viewer_node-2] [INFO] [1783406472.365537683] [local_costmap]: Hisogram Value of Sector(50): [409385.437500]
[local_costmap_viewer_node-2] [INFO] [1783406472.365543818] [local_costmap]: Hisogram Value of Sector(51): [401316.812500]
[local_costmap_viewer_node-2] [INFO] [1783406472.365549955] [local_costmap]: Hisogram Value of Sector(52): [366403.906250]
[local_costmap_viewer_node-2] [INFO] [1783406472.365556217] [local_costmap]: Hisogram Value of Sector(53): [0.000000]
[local_costmap_viewer_node-2] [INFO] [1783406472.365561861] [local_costmap]: Hisogram Value of Sector(54): [0.000000]
[local_costmap_viewer_node-2] [INFO] [1783406472.365567607] [local_costmap]: Hisogram Value of Sector(55): [0.000000]
[local_costmap_viewer_node-2] [INFO] [1783406472.365573442] [local_costmap]: Hisogram Value of Sector(56): [0.000000]
[local_costmap_viewer_node-2] [INFO] [1783406472.365579265] [local_costmap]: Hisogram Value of Sector(57): [366007.906250]
[local_costmap_viewer_node-2] [INFO] [1783406472.365585520] [local_costmap]: Hisogram Value of Sector(58): [391985.062500]
[local_costmap_viewer_node-2] [INFO] [1783406472.365592019] [local_costmap]: Hisogram Value of Sector(59): [414562.312500]
"""

# Regex pattern
pattern = r"Sector\((\d+)\):\s*\[([\d\.]+)\]"
matches = re.findall(pattern, log_data)

# Parse data
sectors = np.array([int(m[0]) for m in matches])
hist_vals = np.array([float(m[1]) for m in matches])

# --- Plotting Bar Chart ---
plt.figure(figsize=(12, 6), dpi=100)
plt.style.use('seaborn-v0_8-whitegrid')

# Using plt.bar for the bar chart
plt.bar(sectors, hist_vals, color='skyblue', edgecolor='black', alpha=0.8)

plt.title("Histogram Value per Sector Index", fontsize=14, pad=15)
plt.xlabel("Sector Index", fontsize=12)
plt.ylabel("Value", fontsize=12)
plt.xlim(-1, max(sectors) + 1)
plt.xticks(np.arange(0, max(sectors)+1, 5))

plt.tight_layout()
plt.show()