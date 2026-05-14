#!/usr/bin/env python3
"""
plot_structural.py ── 結構性 DAG × 三種演算法 效能圖
讀取 results/structural_test.csv，輸出：
  1. results/structural_loglog.png    ── log-log 效能圖（9 條線）
  2. results/structural_speedup.png   ── 加速比：DFS-rec/BFS、DFS-iter/BFS，3 種結構各一子圖
  3. results/structural_bar1000.png   ── 1000 閘 grouped bar（3 結構 × 3 演算法）

執行方式：python3 plot_structural.py（在 src/benchmarks/ 目錄下執行）
"""

import sys, os

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import matplotlib.font_manager as fm
except ImportError:
    print("請安裝 matplotlib：pip3 install matplotlib"); sys.exit(1)

try:
    import pandas as pd
    import numpy as np
except ImportError:
    print("請安裝 pandas / numpy：pip3 install pandas numpy"); sys.exit(1)

# ── 中文字型 ─────────────────────────────────────────────────
_ZH = ["Heiti TC", "Songti SC", "PingFang SC", "STHeiti",
       "WenQuanYi Micro Hei", "SimHei", "Noto Sans CJK TC"]
_avail = {f.name for f in fm.fontManager.ttflist}
_chosen = next((f for f in _ZH if f in _avail), None)
if _chosen:
    matplotlib.rcParams["font.family"] = [_chosen, "DejaVu Sans"]
    matplotlib.rcParams["axes.unicode_minus"] = False

RESULTS  = "results"
CSV_PATH = os.path.join(RESULTS, "structural_test.csv")

if not os.path.exists(CSV_PATH):
    print(f"找不到 {CSV_PATH}，請先執行 make structural_bench"); sys.exit(1)

df = pd.read_csv(CSV_PATH)

agg = (df.groupby(["num_gates", "dag_type", "algorithm"])["mean_time_us"]
         .agg(mean="mean", std="std")
         .reset_index())

DAG_TYPES = ["random", "deep_chain", "wide_fanout"]
ALGOS     = ["BFS", "DFS_recursive", "DFS_iterative"]
ALGO_LABELS = {
    "BFS":           "BFS（Kahn's）",
    "DFS_recursive": "DFS 遞迴版",
    "DFS_iterative": "DFS 迭代版",
}
DAG_LABELS = {
    "random":      "Random DAG",
    "deep_chain":  "Deep Chain",
    "wide_fanout": "Wide Fanout",
}

# 顏色：dag_type 決定色調
DAG_COLORS = {
    "random":      {"BFS": "steelblue",   "DFS_recursive": "tomato",     "DFS_iterative": "seagreen"},
    "deep_chain":  {"BFS": "dodgerblue",  "DFS_recursive": "orangered",  "DFS_iterative": "mediumseagreen"},
    "wide_fanout": {"BFS": "royalblue",   "DFS_recursive": "firebrick",  "DFS_iterative": "darkgreen"},
}
LINESTYLES = {"BFS": "-", "DFS_recursive": "--", "DFS_iterative": ":"}
MARKERS    = {"random": "o", "deep_chain": "s", "wide_fanout": "^"}

os.makedirs(RESULTS, exist_ok=True)

# ══════════════════════════════════════════════════════════════
# 圖 1：Log-Log 效能圖（9 條線）
# ══════════════════════════════════════════════════════════════
fig, ax = plt.subplots(figsize=(11, 7))

for dag in DAG_TYPES:
    for algo in ALGOS:
        sub = agg[(agg["dag_type"] == dag) & (agg["algorithm"] == algo)].sort_values("num_gates")
        if sub.empty: continue
        ax.errorbar(sub["num_gates"], sub["mean"], yerr=sub["std"].fillna(0),
                    marker=MARKERS[dag], linewidth=1.8, capsize=3,
                    linestyle=LINESTYLES[algo],
                    color=DAG_COLORS[dag][algo],
                    label=f"{DAG_LABELS[dag]} · {ALGO_LABELS[algo]}")

ax.set_xscale("log"); ax.set_yscale("log")
ax.set_xlabel("閘數量（log 軸）", fontsize=12)
ax.set_ylabel("每次排序耗時 μs（log 軸）", fontsize=12)
ax.set_title("三種 DAG 結構 × 三種演算法 拓撲排序效能\n"
             "（顏色 = DAG 結構，線型 = 演算法）", fontsize=12)
ax.legend(fontsize=8, ncol=3, loc="upper left")
ax.grid(True, which="both", alpha=0.3)
plt.tight_layout()
out1 = os.path.join(RESULTS, "structural_loglog.png")
plt.savefig(out1, dpi=150, bbox_inches="tight"); plt.close()
print(f"Saved: {out1}")

# ══════════════════════════════════════════════════════════════
# 圖 2：加速比子圖（3 種結構各一子圖，DFS-rec/BFS 和 DFS-iter/BFS）
# ══════════════════════════════════════════════════════════════
fig, axes = plt.subplots(1, 3, figsize=(15, 5), sharey=False)

for ax, dag in zip(axes, DAG_TYPES):
    sub   = agg[agg["dag_type"] == dag]
    bfs   = sub[sub["algorithm"] == "BFS"].sort_values("num_gates")
    rec   = sub[sub["algorithm"] == "DFS_recursive"].sort_values("num_gates")
    itr   = sub[sub["algorithm"] == "DFS_iterative"].sort_values("num_gates")

    scales = bfs["num_gates"].values
    sp_rec  = rec["mean"].values  / bfs["mean"].values
    sp_iter = itr["mean"].values  / bfs["mean"].values

    ax.plot(scales, sp_rec,  marker="s", linewidth=2, color="tomato",
            label="DFS 遞迴 / BFS")
    ax.plot(scales, sp_iter, marker="^", linewidth=2, color="seagreen",
            label="DFS 迭代 / BFS")

    for s, r in zip(scales, sp_rec):
        ax.annotate(f"{r:.2f}", xy=(s, r), xytext=(0, 5),
                    textcoords="offset points", ha="center", fontsize=7, color="tomato")
    for s, r in zip(scales, sp_iter):
        ax.annotate(f"{r:.2f}", xy=(s, r), xytext=(0, -12),
                    textcoords="offset points", ha="center", fontsize=7, color="seagreen")

    ax.axhline(1.0, color="gray", linestyle="--", linewidth=1.2)
    ax.set_xscale("log")
    ax.set_title(DAG_LABELS[dag], fontsize=12)
    ax.set_xlabel("閘數量（log 軸）", fontsize=10)
    ax.set_ylabel("加速比（DFS / BFS）" if dag == "random" else "", fontsize=10)
    ax.legend(fontsize=9)
    ax.grid(True, which="both", alpha=0.3)

fig.suptitle("DAG 結構對演算法相對效能的影響\n（< 1 = 比 BFS 快，> 1 = 比 BFS 慢）", fontsize=13)
plt.tight_layout()
out2 = os.path.join(RESULTS, "structural_speedup.png")
plt.savefig(out2, dpi=150, bbox_inches="tight"); plt.close()
print(f"Saved: {out2}")

# ══════════════════════════════════════════════════════════════
# 圖 3：1000 閘 Grouped Bar（3 結構 × 3 演算法 = 9 條長條）
# ══════════════════════════════════════════════════════════════
fig, ax = plt.subplots(figsize=(10, 6))

target_scale = 1000
sub1000 = agg[agg["num_gates"] == target_scale]

x     = np.arange(len(DAG_TYPES))
width = 0.25
offsets = [-width, 0, width]
BAR_COLORS = {"BFS": "steelblue", "DFS_recursive": "tomato", "DFS_iterative": "seagreen"}

for i, algo in enumerate(ALGOS):
    vals = []
    for dag in DAG_TYPES:
        row = sub1000[(sub1000["dag_type"] == dag) & (sub1000["algorithm"] == algo)]["mean"]
        vals.append(row.values[0] if len(row) > 0 else 0)
    bars = ax.bar(x + offsets[i], vals, width, label=ALGO_LABELS[algo],
                  color=BAR_COLORS[algo], alpha=0.85)
    for bar, v in zip(bars, vals):
        ax.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.5,
                f"{v:.1f}", ha="center", va="bottom", fontsize=8)

ax.set_xticks(x)
ax.set_xticklabels([DAG_LABELS[d] for d in DAG_TYPES], fontsize=11)
ax.set_ylabel("每次排序耗時 μs", fontsize=12)
ax.set_title(f"1000 閘下三種 DAG 結構 × 三種演算法耗時比較", fontsize=12)
ax.legend(fontsize=10)
ax.grid(True, axis="y", alpha=0.3)
plt.tight_layout()
out3 = os.path.join(RESULTS, "structural_bar1000.png")
plt.savefig(out3, dpi=150, bbox_inches="tight"); plt.close()
print(f"Saved: {out3}")

# ── 印出摘要表 ────────────────────────────────────────────────
print("\n=== 各結構 × 演算法摘要（相對 BFS 的比值）===")
for dag in DAG_TYPES:
    print(f"\n[{DAG_LABELS[dag]}]")
    print(f"{'閘數':>8}  {'BFS(μs)':>10}  {'DFS-rec(μs)':>12}  {'DFS-iter(μs)':>13}  {'rec/BFS':>8}  {'iter/BFS':>9}")
    print("-" * 68)
    sub  = agg[agg["dag_type"] == dag]
    bfs  = sub[sub["algorithm"] == "BFS"].sort_values("num_gates")
    rec  = sub[sub["algorithm"] == "DFS_recursive"].sort_values("num_gates")
    itr  = sub[sub["algorithm"] == "DFS_iterative"].sort_values("num_gates")
    for (_, br), (_, rr), (_, ir) in zip(bfs.iterrows(), rec.iterrows(), itr.iterrows()):
        print(f"{int(br['num_gates']):>8}  {br['mean']:>10.3f}  {rr['mean']:>12.3f}  "
              f"{ir['mean']:>13.3f}  {rr['mean']/br['mean']:>7.3f}×  {ir['mean']/br['mean']:>8.3f}×")
