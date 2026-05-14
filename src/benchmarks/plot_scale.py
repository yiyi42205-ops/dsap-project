#!/usr/bin/env python3
"""
plot_scale.py ── BFS / DFS 遞迴 / DFS 迭代 拓撲排序大規模效能圖（隨機 DAG）
讀取 results/scale_test.csv，輸出：
  1. results/scale_test_loglog.png  ── log-log 效能圖（3 條線，含 error bar）
  2. results/scale_test_speedup.png ── 各規模相對 BFS 的加速比（grouped bar）

執行方式：python3 plot_scale.py（在 src/benchmarks/ 目錄下執行）
依賴：matplotlib、pandas、numpy
"""

import sys
import os

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import matplotlib.font_manager as fm
except ImportError:
    print("請安裝 matplotlib：pip3 install matplotlib")
    sys.exit(1)

try:
    import pandas as pd
    import numpy as np
except ImportError:
    print("請安裝 pandas / numpy：pip3 install pandas numpy")
    sys.exit(1)

# ── 中文字型 ─────────────────────────────────────────────────
_ZH_CANDIDATES = ["Heiti TC", "Songti SC", "PingFang SC", "STHeiti",
                  "WenQuanYi Micro Hei", "SimHei", "Noto Sans CJK TC"]
_available = {f.name for f in fm.fontManager.ttflist}
_chosen = next((f for f in _ZH_CANDIDATES if f in _available), None)
if _chosen:
    matplotlib.rcParams["font.family"] = [_chosen, "DejaVu Sans"]
    matplotlib.rcParams["axes.unicode_minus"] = False
else:
    print("  [警告] 找不到中文字型，標籤可能顯示為方塊")

RESULTS  = "results"
CSV_PATH = os.path.join(RESULTS, "scale_test.csv")

if not os.path.exists(CSV_PATH):
    print(f"找不到 {CSV_PATH}，請先執行 make scale_bench")
    sys.exit(1)

# ── 讀取並聚合 ────────────────────────────────────────────────
df = pd.read_csv(CSV_PATH)

agg = df.groupby(["num_gates", "algorithm"])["mean_time_us"].agg(
    mean="mean", std="std"
).reset_index()

ALGOS = ["BFS", "DFS_recursive", "DFS_iterative"]
LABELS = {
    "BFS":           "BFS（Kahn's Algorithm）",
    "DFS_recursive": "DFS 遞迴版（反向後序）",
    "DFS_iterative": "DFS 迭代版（顯式 stack）",
}
COLORS = {"BFS": "steelblue", "DFS_recursive": "tomato", "DFS_iterative": "seagreen"}
MARKERS = {"BFS": "o", "DFS_recursive": "s", "DFS_iterative": "^"}
LINESTYLES = {"BFS": "-", "DFS_recursive": "--", "DFS_iterative": ":"}

bfs = agg[agg["algorithm"] == "BFS"].sort_values("num_gates")
scales = bfs["num_gates"].values

os.makedirs(RESULTS, exist_ok=True)

# ════════════════════════════════════════════════════════════
# 圖一：Log-Log 效能圖（3 條線）
# ════════════════════════════════════════════════════════════
fig, ax = plt.subplots(figsize=(9, 6))

for algo in ALGOS:
    sub = agg[agg["algorithm"] == algo].sort_values("num_gates")
    if sub.empty:
        continue
    ax.errorbar(sub["num_gates"], sub["mean"], yerr=sub["std"].fillna(0),
                marker=MARKERS[algo], linewidth=2, capsize=4,
                linestyle=LINESTYLES[algo],
                color=COLORS[algo], label=LABELS[algo])

# O(N) 參考線
ref_x = np.array([scales[0], scales[-1]], dtype=float)
bfs_means = bfs["mean"].values
ref_y = bfs_means[0] * ref_x / scales[0]
ax.plot(ref_x, ref_y, "--", color="gray", alpha=0.5, linewidth=1.5,
        label="O(N) 參考線")

ax.set_xscale("log")
ax.set_yscale("log")
ax.set_xlabel("閘數量（log 軸）", fontsize=12)
ax.set_ylabel("每次排序耗時 μs（log 軸）", fontsize=12)
ax.set_title("BFS / DFS 遞迴 / DFS 迭代 拓撲排序效能：Log-Log 圖\n"
             "（隨機 DAG，error bar = 5 個不同 seed 的標準差）", fontsize=12)
ax.legend(fontsize=10)
ax.grid(True, which="both", alpha=0.3)

plt.tight_layout()
out1 = os.path.join(RESULTS, "scale_test_loglog.png")
plt.savefig(out1, dpi=150, bbox_inches="tight")
plt.close()
print(f"Saved: {out1}")

# ════════════════════════════════════════════════════════════
# 圖二：相對 BFS 的加速比（grouped bar：DFS-rec / DFS-iter）
# ════════════════════════════════════════════════════════════
rec  = agg[agg["algorithm"] == "DFS_recursive"].sort_values("num_gates")
itr  = agg[agg["algorithm"] == "DFS_iterative"].sort_values("num_gates")

rec_means  = rec["mean"].values
iter_means = itr["mean"].values
sp_rec  = rec_means  / bfs_means   # < 1 表示 DFS 較快
sp_iter = iter_means / bfs_means

x     = np.arange(len(scales))
width = 0.35

fig, ax = plt.subplots(figsize=(10, 5))

bars_rec  = ax.bar(x - width/2, sp_rec,  width, label="DFS 遞迴版 / BFS",
                   color="tomato",    alpha=0.85)
bars_iter = ax.bar(x + width/2, sp_iter, width, label="DFS 迭代版 / BFS",
                   color="seagreen",  alpha=0.85)

for bar, r in zip(bars_rec, sp_rec):
    ax.text(bar.get_x() + bar.get_width() / 2,
            bar.get_height() + 0.005,
            f"{r:.2f}×", ha="center", va="bottom", fontsize=8)
for bar, r in zip(bars_iter, sp_iter):
    ax.text(bar.get_x() + bar.get_width() / 2,
            bar.get_height() + 0.005,
            f"{r:.2f}×", ha="center", va="bottom", fontsize=8)

ax.axhline(1.0, color="gray", linestyle="--", linewidth=1.5, label="基準（= BFS）")
ax.set_xticks(x)
ax.set_xticklabels([str(s) for s in scales])
ax.set_xlabel("閘數量", fontsize=12)
ax.set_ylabel("相對 BFS 的耗時比（< 1 = 比 BFS 快）", fontsize=12)
ax.set_title("各規模下 DFS 遞迴版 / DFS 迭代版相對 BFS 的耗時比\n"
             "（隨機 DAG，越低越快）", fontsize=12)
ax.legend(fontsize=10)
ax.grid(True, axis="y", alpha=0.3)
ax.set_ylim(0, max(max(sp_rec), max(sp_iter)) * 1.25)

plt.tight_layout()
out2 = os.path.join(RESULTS, "scale_test_speedup.png")
plt.savefig(out2, dpi=150, bbox_inches="tight")
plt.close()
print(f"Saved: {out2}")

# ── 印出摘要 ─────────────────────────────────────────────────
print("\n=== 各規模三種演算法比較（相對 BFS）===")
print(f"{'閘數':>8}  {'BFS(μs)':>10}  {'DFS-rec(μs)':>12}  {'DFS-iter(μs)':>13}  {'rec/BFS':>8}  {'iter/BFS':>9}")
print("-" * 68)
for s, bm, rm, im, sr, si in zip(scales, bfs_means, rec_means, iter_means, sp_rec, sp_iter):
    print(f"{int(s):>8}  {bm:>10.3f}  {rm:>12.3f}  {im:>13.3f}  {sr:>7.3f}×  {si:>8.3f}×")
