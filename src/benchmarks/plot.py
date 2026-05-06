#!/usr/bin/env python3
"""
讀取 results/ 下的 CSV，畫圖並輸出 PNG。
執行方式：python3 plot.py   （請在 src/benchmarks/ 目錄下執行）
依賴：matplotlib（pip3 install matplotlib）
"""

import csv
import os
import sys

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import matplotlib.font_manager as fm
except ImportError:
    print("請先安裝 matplotlib：pip3 install matplotlib")
    sys.exit(1)

# ── 中文字型設定 ─────────────────────────────────────────────
# 優先嘗試 macOS 內建字型，再 fallback 到 Linux 字型
_ZH_FONT_CANDIDATES = [
    "Heiti TC",       # macOS（黑體-繁）
    "Songti SC",      # macOS（宋體-簡）
    "PingFang SC",    # macOS（蘋方，需從字型快取搜尋）
    "STHeiti",        # macOS 舊版
    "WenQuanYi Micro Hei",  # Linux
    "SimHei",         # Windows
]
_available = {f.name for f in fm.fontManager.ttflist}
_chosen = next((f for f in _ZH_FONT_CANDIDATES if f in _available), None)

if _chosen:
    matplotlib.rcParams["font.family"] = [_chosen, "DejaVu Sans"]
    matplotlib.rcParams["axes.unicode_minus"] = False  # 負號正常顯示
else:
    print("  [字型] 找不到中文字型，圖表標籤可能顯示為方塊。"
          " 建議：brew install font-noto-sans-cjk-tc")

RESULTS = "results"


def load_csv(filename):
    path = os.path.join(RESULTS, filename)
    if not os.path.exists(path):
        print(f"  [skip] {path} 不存在，請先執行對應的 benchmark binary")
        return None
    rows = []
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append({k: v for k, v in row.items()})
    return rows


# ── 1. hash_vs_linear ────────────────────────────────────────────
def plot_hash_vs_linear():
    rows = load_csv("hash_vs_linear.csv")
    if rows is None:
        return

    ns       = [int(r["N"])           for r in rows]
    hash_t   = [float(r["HashMap_us"]) for r in rows]
    lin_t    = [float(r["Linear_us"])  for r in rows]
    speedups = [float(r["Speedup"])    for r in rows]

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))
    fig.suptitle("Benchmark 1: unordered_map O(1) vs vector<pair> O(N)", fontsize=13)

    ax1.plot(ns, hash_t, "o-", label="unordered_map  O(1)", color="steelblue", linewidth=2)
    ax1.plot(ns, lin_t,  "s-", label="vector<pair>   O(N)", color="tomato",    linewidth=2)
    ax1.set_xlabel("Gate 數量 N")
    ax1.set_ylabel("查詢耗時 (μs)，1000 次平均")
    ax1.set_title("查詢耗時隨規模變化")
    ax1.legend()
    ax1.grid(True, alpha=0.3)

    bar_colors = ["mediumseagreen" if s >= 1 else "salmon" for s in speedups]
    ax2.bar([str(n) for n in ns], speedups, color=bar_colors, alpha=0.85)
    ax2.axhline(1, color="gray", linestyle="--", alpha=0.5, label="speedup = 1")
    ax2.set_xlabel("Gate 數量 N")
    ax2.set_ylabel("加速比 (Linear / HashMap)")
    ax2.set_title("HashMap 相對 Linear Scan 的加速比\n（越高 = HashMap 越有優勢）")
    ax2.legend()
    ax2.grid(True, alpha=0.3, axis="y")

    plt.tight_layout()
    out = os.path.join(RESULTS, "hash_vs_linear.png")
    plt.savefig(out, dpi=150)
    plt.close()
    print(f"  Saved: {out}")


# ── 2. topo_vs_naive ─────────────────────────────────────────────
def plot_topo_vs_naive():
    rows = load_csv("topo_vs_naive.csv")
    if rows is None:
        return

    labels   = [r["bits"] + "-bit" for r in rows]
    bfs_t    = [float(r["BFS_us"])   for r in rows]
    naive_t  = [float(r["Naive_us"]) for r in rows]
    speedups = [float(r["Speedup"])  for r in rows]

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))
    fig.suptitle("Benchmark 2: BFS Kahn's O(V+E) vs Naive Scan O(V²)", fontsize=13)

    ax1.plot(labels, bfs_t,   "o-", label="BFS Kahn's  O(V+E)", color="steelblue", linewidth=2)
    ax1.plot(labels, naive_t, "s-", label="Naive Scan  O(V²)",  color="tomato",    linewidth=2)
    ax1.set_xlabel("Ripple Carry Adder 規模")
    ax1.set_ylabel("平均排序耗時 (μs/call)")
    ax1.set_title("拓撲排序時間隨規模變化\n（carry chain 放大差距）")
    ax1.legend()
    ax1.grid(True, alpha=0.3)

    ax2.bar(labels, speedups, color="mediumseagreen", alpha=0.85)
    ax2.axhline(1, color="gray", linestyle="--", alpha=0.5)
    ax2.set_xlabel("Ripple Carry Adder 規模")
    ax2.set_ylabel("加速比 (Naive / BFS)")
    ax2.set_title("BFS 相對 Naive 的加速比\n（規模越大，O(V²) 代價越高）")
    ax2.grid(True, alpha=0.3, axis="y")

    plt.tight_layout()
    out = os.path.join(RESULTS, "topo_vs_naive.png")
    plt.savefig(out, dpi=150)
    plt.close()
    print(f"  Saved: {out}")


# ── 3. ordered_vs_unordered ─────────────────────────────────────
def plot_ordered_vs_unordered():
    rows = load_csv("ordered_vs_unordered.csv")
    if rows is None:
        return

    ns        = [int(r["N"])                      for r in rows]
    map_ins   = [float(r["Map_insert_us"])         for r in rows]
    unord_ins = [float(r["Unordered_insert_us"])   for r in rows]
    map_look  = [float(r["Map_lookup_us"])         for r in rows]
    unord_look= [float(r["Unordered_lookup_us"])   for r in rows]
    map_iter  = [float(r["Map_iter_us"])           for r in rows]
    unord_iter= [float(r["Unordered_iter_us"])     for r in rows]

    xs = list(range(len(ns)))
    w  = 0.35
    labels = [str(n) for n in ns]

    fig, axes = plt.subplots(1, 3, figsize=(16, 5))
    fig.suptitle("Benchmark 3: std::map (紅黑樹 O(log N)) vs std::unordered_map (雜湊 O(1))",
                 fontsize=13)

    def grouped_bars(ax, a_vals, b_vals, title, ylabel):
        ax.bar([x - w/2 for x in xs], a_vals, w,
               label="std::map",          color="steelblue", alpha=0.85)
        ax.bar([x + w/2 for x in xs], b_vals, w,
               label="std::unordered_map", color="tomato",    alpha=0.85)
        ax.set_xticks(xs)
        ax.set_xticklabels(labels)
        ax.set_xlabel("訊號數量 N")
        ax.set_ylabel(ylabel)
        ax.set_title(title)
        ax.legend()
        ax.grid(True, alpha=0.3, axis="y")

    grouped_bars(axes[0], map_ins,  unord_ins,
                 "插入效能\n（N 次插入，500 輪平均）", "時間 (μs)")
    grouped_bars(axes[1], map_look, unord_look,
                 "查詢效能\n（N 次查詢，500 輪平均）", "時間 (μs)")
    grouped_bars(axes[2], map_iter, unord_iter,
                 "迭代效能\n（全表迭代，500 輪平均）", "時間 (μs)")

    plt.tight_layout()
    out = os.path.join(RESULTS, "ordered_vs_unordered.png")
    plt.savefig(out, dpi=150)
    plt.close()
    print(f"  Saved: {out}")


# ── 4. random_circuit_bench（log-log，含 error bar）────────────
def plot_random_circuit_loglog():
    rows = load_csv("random_circuit_bench.csv")
    if rows is None:
        return

    try:
        import numpy as np
    except ImportError:
        print("  [skip] log-log 圖需要 numpy：pip3 install numpy")
        return

    # 以 scale 分組，聚合 10 個電路的均值
    from collections import defaultdict
    bfs_by_scale = defaultdict(list)
    dfs_by_scale = defaultdict(list)
    for r in rows:
        s = int(r["scale"])
        bfs_by_scale[s].append(float(r["bfs_mean_ns"]))
        dfs_by_scale[s].append(float(r["dfs_mean_ns"]))

    scales = sorted(bfs_by_scale.keys())
    bfs_means = np.array([np.mean(bfs_by_scale[s]) for s in scales]) / 1000  # ns → μs
    bfs_stds  = np.array([np.std(bfs_by_scale[s])  for s in scales]) / 1000
    dfs_means = np.array([np.mean(dfs_by_scale[s]) for s in scales]) / 1000
    dfs_stds  = np.array([np.std(dfs_by_scale[s])  for s in scales]) / 1000
    scales_arr = np.array(scales, dtype=float)

    fig, ax = plt.subplots(figsize=(8, 6))

    ax.errorbar(scales_arr, bfs_means, yerr=bfs_stds,
                marker="o", linewidth=2, capsize=4,
                label="BFS (Kahn's Algorithm)", color="steelblue")
    ax.errorbar(scales_arr, dfs_means, yerr=dfs_stds,
                marker="s", linewidth=2, capsize=4,
                label="DFS (反向後序遍歷)", color="tomato")

    # O(N) 參考線（以最小規模的 BFS 均值為錨點）
    ref_x = np.array([scales_arr[0], scales_arr[-1]])
    ref_y = bfs_means[0] * ref_x / scales_arr[0]
    ax.plot(ref_x, ref_y, "--", color="gray", alpha=0.5, linewidth=1.5,
            label="O(N) 參考線")

    ax.set_xscale("log")
    ax.set_yscale("log")
    ax.set_xlabel("Gate 數量（log 軸）", fontsize=12)
    ax.set_ylabel("每次排序耗時 μs（log 軸）", fontsize=12)
    ax.set_title("BFS vs DFS 拓撲排序：Log-Log 效能圖\n"
                 "（error bar = 10 個隨機電路的標準差）", fontsize=12)
    ax.legend(fontsize=10)
    ax.grid(True, which="both", alpha=0.3)

    # 標注各規模的 BFS/DFS 比值
    for s, bm, dm in zip(scales_arr, bfs_means, dfs_means):
        ratio = bm / dm
        ax.annotate(f"{ratio:.2f}x",
                    xy=(s, max(bm, dm)),
                    xytext=(0, 8), textcoords="offset points",
                    ha="center", fontsize=8, color="purple")

    plt.tight_layout()
    out = os.path.join(RESULTS, "bfs_vs_dfs_loglog.png")
    plt.savefig(out, dpi=150, bbox_inches="tight")
    plt.close()
    print(f"  Saved: {out}")


if __name__ == "__main__":
    print("=== Benchmark Plot Generator ===\n")
    plot_hash_vs_linear()
    plot_topo_vs_naive()
    plot_ordered_vs_unordered()
    plot_random_circuit_loglog()
    print("\nDone. 圖片存在 results/ 目錄下。")
