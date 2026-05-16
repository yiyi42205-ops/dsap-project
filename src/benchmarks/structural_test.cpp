// structural_test.cpp
// 結構性 DAG 對照實驗：random / deep_chain / wide_fanout
//                    × BFS / DFS recursive / DFS iterative 拓撲排序
//
// 規模：10 / 50 / 100 / 500 / 1000 / 2000 / 5000 閘
// 每規模每結構：5 個 seed（random）或 5 次重複測量（chain/fanout 結構固定）
// 各組合：WARMUP=10 + ITERS=100 次測量，取均值與標準差
// 輸出：results/structural_test.csv
//
// 編譯：g++ -std=c++17 -O2 -Wall structural_test.cpp -o structural_test
// 執行：./structural_test（需先建立 results/ 目錄）

#include "bench_utils.h"
#include <fstream>
#include <iomanip>
#include <cmath>
#include <iostream>

// ── 演算法選擇 ───────────────────────────────────────────────
enum class Algo { BFS, DFS_REC, DFS_ITER };

// ── 統計工具 ─────────────────────────────────────────────────
struct Stats { double mean_us; double std_us; };

Stats measure(Circuit& c, Algo algo, int warmup, int iters) {
    auto run_once = [&]() {
        switch (algo) {
            case Algo::BFS:      c.topologicalSortBFS(); break;
            case Algo::DFS_REC:  c.topologicalSortDFS(); break;
            case Algo::DFS_ITER: c.topologicalSortDFSIterative(); break;
        }
    };
    for (int i = 0; i < warmup; i++) run_once();
    std::vector<double> times(iters);
    for (int i = 0; i < iters; i++) {
        auto t0 = std::chrono::high_resolution_clock::now();
        run_once();
        auto t1 = std::chrono::high_resolution_clock::now();
        times[i] = std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0).count() / 1000.0;
    }
    double m = 0;
    for (double v : times) m += v;
    m /= iters;
    double sq = 0;
    for (double v : times) sq += (v - m) * (v - m);
    return { m, std::sqrt(sq / iters) };
}

// ── 一個 scale × dag_type 的跑法 ────────────────────────────
struct RunResult {
    double bfs_mean, dfs_rec_mean, dfs_iter_mean;
    long long avg_edges;
};

static const std::string algo_name(Algo a) {
    switch (a) {
        case Algo::BFS:      return "BFS";
        case Algo::DFS_REC:  return "DFS_recursive";
        case Algo::DFS_ITER: return "DFS_iterative";
    }
    return "";
}

RunResult runScale(std::ofstream& csv,
                   const std::string& dag_type,
                   int scale,
                   int seeds_per_scale,
                   int warmup, int iters,
                   int avg_fanin)
{
    double bfs_sum = 0, rec_sum = 0, iter_sum = 0;
    long long total_edges = 0;
    int valid = 0;

    for (int si = 0; si < seeds_per_scale; si++) {
        unsigned seed = (unsigned)(scale * 31 + si * 13 + 7);

        Circuit c;
        if (dag_type == "random") {
            c = generateRandomCircuit(scale, avg_fanin, seed);
            if (!c.topologicalSortBFS()) continue;
        } else if (dag_type == "deep_chain") {
            c = generateDeepChain(scale);
        } else { // wide_fanout
            c = generateWideFanout(scale);
        }

        long long edges = 0;
        for (Gate* g : c.getAllGates()) edges += (long long)g->inputNames.size();
        total_edges += edges;

        Stats bfs  = measure(c, Algo::BFS,      warmup, iters);
        Stats rec  = measure(c, Algo::DFS_REC,  warmup, iters);
        Stats iter = measure(c, Algo::DFS_ITER, warmup, iters);

        for (auto& [algo, st] : std::vector<std::pair<Algo, Stats>>{
                {Algo::BFS, bfs}, {Algo::DFS_REC, rec}, {Algo::DFS_ITER, iter}}) {
            csv << scale << "," << edges << "," << dag_type << ","
                << algo_name(algo) << ","
                << std::setprecision(4) << st.mean_us << "," << st.std_us << "," << seed << "\n";
        }

        bfs_sum  += bfs.mean_us;
        rec_sum  += rec.mean_us;
        iter_sum += iter.mean_us;
        valid++;
    }

    if (valid == 0) return {0, 0, 0, 0};
    return { bfs_sum/valid, rec_sum/valid, iter_sum/valid, total_edges/valid };
}

// ── 主程式 ───────────────────────────────────────────────────
int main() {
    const std::vector<int> SCALES        = {10, 50, 100, 500, 1000, 2000, 5000};
    const std::vector<std::string> TYPES = {"random", "deep_chain", "wide_fanout"};
    const int SEEDS_PER_SCALE            = 5;
    const int WARMUP                     = 10;
    const int ITERS                      = 100;
    const int AVG_FANIN                  = 3;

    std::ofstream csv("results/structural_test.csv");
    if (!csv.is_open()) {
        std::cerr << "錯誤：無法建立 results/structural_test.csv，請先 mkdir -p results\n";
        return 1;
    }
    csv << "num_gates,num_edges,dag_type,algorithm,mean_time_us,std_time_us,seed\n";

    std::cout << std::fixed << std::setprecision(3);
    std::cout << "\n=== Structural DAG Test: BFS / DFS-rec / DFS-iter 拓撲排序 ===\n";
    std::cout << std::left
              << std::setw(13) << "DAG Type"
              << std::setw(7)  << "規模"
              << std::setw(12) << "BFS(μs)"
              << std::setw(14) << "DFS-rec(μs)"
              << std::setw(14) << "DFS-iter(μs)"
              << std::setw(12) << "rec/BFS"
              << std::setw(12) << "iter/BFS"
              << "\n";
    std::cout << std::string(84, '-') << "\n";

    for (const auto& dag_type : TYPES) {
        for (int scale : SCALES) {
            auto r = runScale(csv, dag_type, scale,
                              SEEDS_PER_SCALE, WARMUP, ITERS, AVG_FANIN);
            if (r.bfs_mean == 0) continue;
            std::cout << std::left
                      << std::setw(13) << dag_type
                      << std::setw(7)  << scale
                      << std::setw(12) << r.bfs_mean
                      << std::setw(14) << r.dfs_rec_mean
                      << std::setw(14) << r.dfs_iter_mean
                      << std::setw(12) << (r.dfs_rec_mean  / r.bfs_mean)
                      << std::setw(12) << (r.dfs_iter_mean / r.bfs_mean)
                      << "\n";
        }
        std::cout << "\n";
    }

    csv.close();
    std::cout << "CSV 已寫入：results/structural_test.csv\n";
    std::cout << "執行 'python3 plot_bench.py' 可生成圖表。\n\n";
    return 0;
}
