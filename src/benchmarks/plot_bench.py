#!/usr/bin/env python3
"""
plot_bench.py ── BFS / DFS-rec / DFS-iter 拓撲排序效能圖
讀取 results/scale_test.csv 與 results/structural_test.csv，輸出：

  scale_test.csv 存在時：
    results/scale_test_loglog.png   ── log-log 效能圖（3 條線，含 error bar）
    results/scale_test_speedup.png  ── 相對 BFS 加速比（grouped bar）

  structural_test.csv 存在時：
    results/structural_loglog.png   ── log-log 效能圖（9 條線）
    results/structural_speedup.png  ── 加速比子圖（3 種結構各一子圖）
    results/structural_bar1000.png  ── 1000 閘 grouped bar

執行方式：python3 plot_bench.py（在 src/benchmarks/ 目錄下執行）
依賴：matplotlib、pandas、numpy
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
else:
    print("  [警告] 找不到中文字型，標籤可能顯示為方塊")

RESULTS = "results"
os.makedirs(RESULTS, exist_ok=True)

ALGOS = ["BFS", "DFS_recursive", "DFS_iterative"]
ALGO_LABELS = {
    "BFS":           "BFS（Kahn's Algorithm）",
    "DFS_recursive": "DFS 遞迴版（反向後序）",
    "DFS_iterative": "DFS 迭代版（顯式 stack）",
}
COLORS    = {"BFS": "steelblue", "DFS_recursive": "tomato", "DFS_iterative": "seagreen"}
MARKERS   = {"BFS": "o",         "DFS_recursive": "s",      "DFS_iterative": "^"}
LINESTYLES = {"BFS": "-",        "DFS_recursive": "--",     "DFS_iterative": ":"}


# ════════════════════════════════════════════════════════════════
# 一、scale_test：隨機 DAG 規模 vs 時間
# ════════════════════════════════════════════════════════════════

def plot_scale():
    csv_path = os.path.join(RESULTS, "scale_test.csv")
    if not os.path.exists(csv_path):
        print(f"  [skip] {csv_path} 不存在，請先執行 make scale_bench")
        return

    df  = pd.read_csv(csv_path)
    agg = df.groupby(["num_gates", "algorithm"])["mean_time_us"].agg(
        mean="mean", std="std").reset_index()

    bfs    = agg[agg["algorithm"] == "BFS"].sort_values("num_gates")
    scales = bfs["num_gates"].values
    bfs_means = bfs["mean"].values

    # ── 圖一：Log-Log ──────────────────────────────────────────
    fig, ax = plt.subplots(figsize=(9, 6))

    for algo in ALGOS:
        sub = agg[agg["algorithm"] == algo].sort_values("num_gates")
        if sub.empty: continue
        ax.errorbar(sub["num_gates"], sub["mean"], yerr=sub["std"].fillna(0),
                    marker=MARKERS[algo], linewidth=2, capsize=4,
                    linestyle=LINESTYLES[algo],
                    color=COLORS[algo], label=ALGO_LABELS[algo])

    ref_x = np.array([scales[0], scales[-1]], dtype=float)
    ref_y = bfs_means[0] * ref_x / scales[0]
    ax.plot(ref_x, ref_y, "--", color="gray", alpha=0.5, linewidth=1.5, label="O(N) 參考線")

    ax.set_xscale("log"); ax.set_yscale("log")
    ax.set_xlabel("閘數量（log 軸）", fontsize=12)
    ax.set_ylabel("每次排序耗時 μs（log 軸）", fontsize=12)
    ax.set_title("BFS / DFS 遞迴 / DFS 迭代 拓撲排序效能：Log-Log 圖\n"
                 "（隨機 DAG，error bar = 5 個不同 seed 的標準差）", fontsize=12)
    ax.legend(fontsize=10)
    ax.grid(True, which="both", alpha=0.3)
    plt.tight_layout()
    out = os.path.join(RESULTS, "scale_test_loglog.png")
    plt.savefig(out, dpi=150, bbox_inches="tight"); plt.close()
    print(f"Saved: {out}")

    # ── 圖二：加速比 Grouped Bar ───────────────────────────────
    rec  = agg[agg["algorithm"] == "DFS_recursive"].sort_values("num_gates")
    itr  = agg[agg["algorithm"] == "DFS_iterative"].sort_values("num_gates")
    sp_rec  = rec["mean"].values  / bfs_means
    sp_iter = itr["mean"].values  / bfs_means

    x = np.arange(len(scales)); w = 0.35
    fig, ax = plt.subplots(figsize=(10, 5))

    bars_rec  = ax.bar(x - w/2, sp_rec,  w, label="DFS 遞迴版 / BFS", color="tomato",   alpha=0.85)
    bars_iter = ax.bar(x + w/2, sp_iter, w, label="DFS 迭代版 / BFS", color="seagreen", alpha=0.85)

    for bar, r in zip(bars_rec, sp_rec):
        ax.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.005,
                f"{r:.2f}×", ha="center", va="bottom", fontsize=8)
    for bar, r in zip(bars_iter, sp_iter):
        ax.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.005,
                f"{r:.2f}×", ha="center", va="bottom", fontsize=8)

    ax.axhline(1.0, color="gray", linestyle="--", linewidth=1.5, label="基準（= BFS）")
    ax.set_xticks(x); ax.set_xticklabels([str(s) for s in scales])
    ax.set_xlabel("閘數量", fontsize=12)
    ax.set_ylabel("相對 BFS 的耗時比（< 1 = 比 BFS 快）", fontsize=12)
    ax.set_title("各規模下 DFS 遞迴版 / DFS 迭代版相對 BFS 的耗時比\n（隨機 DAG，越低越快）", fontsize=12)
    ax.legend(fontsize=10)
    ax.grid(True, axis="y", alpha=0.3)
    ax.set_ylim(0, max(max(sp_rec), max(sp_iter)) * 1.25)
    plt.tight_layout()
    out = os.path.join(RESULTS, "scale_test_speedup.png")
    plt.savefig(out, dpi=150, bbox_inches="tight"); plt.close()
    print(f"Saved: {out}")

    print("\n=== scale_test：各規模三種演算法比較（相對 BFS）===")
    rec_means  = rec["mean"].values
    iter_means = itr["mean"].values
    print(f"{'閘數':>8}  {'BFS(μs)':>10}  {'DFS-rec(μs)':>12}  {'DFS-iter(μs)':>13}  {'rec/BFS':>8}  {'iter/BFS':>9}")
    print("-" * 68)
    for s, bm, rm, im, sr, si in zip(scales, bfs_means, rec_means, iter_means, sp_rec, sp_iter):
        print(f"{int(s):>8}  {bm:>10.3f}  {rm:>12.3f}  {im:>13.3f}  {sr:>7.3f}×  {si:>8.3f}×")


# ════════════════════════════════════════════════════════════════
# 二、structural_test：三種 DAG 結構 × 三種演算法
# ════════════════════════════════════════════════════════════════

DAG_TYPES  = ["random", "deep_chain", "wide_fanout"]
DAG_LABELS = {"random": "Random DAG", "deep_chain": "Deep Chain", "wide_fanout": "Wide Fanout"}
DAG_COLORS = {
    "random":      {"BFS": "steelblue",  "DFS_recursive": "tomato",    "DFS_iterative": "seagreen"},
    "deep_chain":  {"BFS": "dodgerblue", "DFS_recursive": "orangered", "DFS_iterative": "mediumseagreen"},
    "wide_fanout": {"BFS": "royalblue",  "DFS_recursive": "firebrick", "DFS_iterative": "darkgreen"},
}
DAG_MARKERS = {"random": "o", "deep_chain": "s", "wide_fanout": "^"}


def plot_structural():
    csv_path = os.path.join(RESULTS, "structural_test.csv")
    if not os.path.exists(csv_path):
        print(f"  [skip] {csv_path} 不存在，請先執行 make structural_bench")
        return

    df  = pd.read_csv(csv_path)
    agg = (df.groupby(["num_gates", "dag_type", "algorithm"])["mean_time_us"]
             .agg(mean="mean", std="std").reset_index())

    # ── 圖一：Log-Log（9 條線）────────────────────────────────
    fig, ax = plt.subplots(figsize=(11, 7))

    for dag in DAG_TYPES:
        for algo in ALGOS:
            sub = agg[(agg["dag_type"] == dag) & (agg["algorithm"] == algo)].sort_values("num_gates")
            if sub.empty: continue
            ax.errorbar(sub["num_gates"], sub["mean"], yerr=sub["std"].fillna(0),
                        marker=DAG_MARKERS[dag], linewidth=1.8, capsize=3,
                        linestyle=LINESTYLES[algo], color=DAG_COLORS[dag][algo],
                        label=f"{DAG_LABELS[dag]} · {ALGO_LABELS[algo]}")

    ax.set_xscale("log"); ax.set_yscale("log")
    ax.set_xlabel("閘數量（log 軸）", fontsize=12)
    ax.set_ylabel("每次排序耗時 μs（log 軸）", fontsize=12)
    ax.set_title("三種 DAG 結構 × 三種演算法 拓撲排序效能\n（顏色 = DAG 結構，線型 = 演算法）", fontsize=12)
    ax.legend(fontsize=8, ncol=3, loc="upper left")
    ax.grid(True, which="both", alpha=0.3)
    plt.tight_layout()
    out = os.path.join(RESULTS, "structural_loglog.png")
    plt.savefig(out, dpi=150, bbox_inches="tight"); plt.close()
    print(f"Saved: {out}")

    # ── 圖二：加速比子圖（3 種結構各一子圖）─────────────────
    fig, axes = plt.subplots(1, 3, figsize=(15, 5), sharey=False)

    for ax, dag in zip(axes, DAG_TYPES):
        sub  = agg[agg["dag_type"] == dag]
        bfs  = sub[sub["algorithm"] == "BFS"].sort_values("num_gates")
        rec  = sub[sub["algorithm"] == "DFS_recursive"].sort_values("num_gates")
        itr  = sub[sub["algorithm"] == "DFS_iterative"].sort_values("num_gates")

        scales  = bfs["num_gates"].values
        sp_rec  = rec["mean"].values / bfs["mean"].values
        sp_iter = itr["mean"].values / bfs["mean"].values

        ax.plot(scales, sp_rec,  marker="s", linewidth=2, color="tomato",   label="DFS 遞迴 / BFS")
        ax.plot(scales, sp_iter, marker="^", linewidth=2, color="seagreen", label="DFS 迭代 / BFS")

        for s, r in zip(scales, sp_rec):
            ax.annotate(f"{r:.2f}", xy=(s, r), xytext=(0,  5),
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
    out = os.path.join(RESULTS, "structural_speedup.png")
    plt.savefig(out, dpi=150, bbox_inches="tight"); plt.close()
    print(f"Saved: {out}")

    # ── 圖三：1000 閘 Grouped Bar ─────────────────────────────
    sub1000 = agg[agg["num_gates"] == 1000]
    x = np.arange(len(DAG_TYPES)); w = 0.25

    fig, ax = plt.subplots(figsize=(10, 6))
    for i, algo in enumerate(ALGOS):
        vals = []
        for dag in DAG_TYPES:
            row = sub1000[(sub1000["dag_type"] == dag) & (sub1000["algorithm"] == algo)]["mean"]
            vals.append(row.values[0] if len(row) > 0 else 0)
        bars = ax.bar(x + (i - 1) * w, vals, w, label=ALGO_LABELS[algo],
                      color=COLORS[algo], alpha=0.85)
        for bar, v in zip(bars, vals):
            ax.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.5,
                    f"{v:.1f}", ha="center", va="bottom", fontsize=8)

    ax.set_xticks(x); ax.set_xticklabels([DAG_LABELS[d] for d in DAG_TYPES], fontsize=11)
    ax.set_ylabel("每次排序耗時 μs", fontsize=12)
    ax.set_title("1000 閘下三種 DAG 結構 × 三種演算法耗時比較", fontsize=12)
    ax.legend(fontsize=10)
    ax.grid(True, axis="y", alpha=0.3)
    plt.tight_layout()
    out = os.path.join(RESULTS, "structural_bar1000.png")
    plt.savefig(out, dpi=150, bbox_inches="tight"); plt.close()
    print(f"Saved: {out}")

    print("\n=== structural_test：各結構 × 演算法摘要（相對 BFS）===")
    for dag in DAG_TYPES:
        print(f"\n[{DAG_LABELS[dag]}]")
        print(f"{'閘數':>8}  {'BFS(μs)':>10}  {'DFS-rec(μs)':>12}  {'DFS-iter(μs)':>13}  {'rec/BFS':>8}  {'iter/BFS':>9}")
        print("-" * 68)
        sub = agg[agg["dag_type"] == dag]
        bfs = sub[sub["algorithm"] == "BFS"].sort_values("num_gates")
        rec = sub[sub["algorithm"] == "DFS_recursive"].sort_values("num_gates")
        itr = sub[sub["algorithm"] == "DFS_iterative"].sort_values("num_gates")
        for (_, br), (_, rr), (_, ir) in zip(bfs.iterrows(), rec.iterrows(), itr.iterrows()):
            print(f"{int(br['num_gates']):>8}  {br['mean']:>10.3f}  {rr['mean']:>12.3f}  "
                  f"{ir['mean']:>13.3f}  {rr['mean']/br['mean']:>7.3f}×  {ir['mean']/br['mean']:>8.3f}×")


# ════════════════════════════════════════════════════════════════
if __name__ == "__main__":
    print("=== Benchmark Plot Generator ===\n")
    plot_scale()
    print()
    plot_structural()
    print("\nDone. 圖片存在 results/ 目錄下。")
