// scale_test.cpp
// 大規模 BFS / DFS recursive / DFS iterative 拓撲排序效能比較（隨機 DAG）
//
// 規模：10 / 50 / 100 / 500 / 1000 / 2000 / 5000 閘
// 每規模：5 個不同 seed 的隨機 DAG，各跑 warm-up 10 + 測量 100 次
// 輸出：results/scale_test.csv
//
// 編譯：g++ -std=c++17 -O2 -Wall scale_test.cpp -o scale_test
// 執行：./scale_test（需先建立 results/ 目錄）

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

// ── 主程式 ───────────────────────────────────────────────────
int main() {
    const std::vector<int> SCALES  = {10, 50, 100, 500, 1000, 2000, 5000};
    const int SEEDS_PER_SCALE      = 5;
    const int WARMUP               = 10;
    const int ITERS                = 100;
    const int AVG_FANIN            = 3;

    std::ofstream csv("results/scale_test.csv");
    if (!csv.is_open()) {
        std::cerr << "錯誤：無法建立 results/scale_test.csv，請先 mkdir -p results\n";
        return 1;
    }
    csv << "num_gates,num_edges,algorithm,mean_time_us,std_time_us,seed\n";

    std::cout << std::fixed << std::setprecision(3);
    std::cout << "\n=== Scale Test: BFS / DFS-rec / DFS-iter 拓撲排序（隨機 DAG）===\n";
    std::cout << std::left
              << std::setw(7)  << "規模"
              << std::setw(10) << "邊數"
              << std::setw(12) << "BFS(μs)"
              << std::setw(14) << "DFS-rec(μs)"
              << std::setw(14) << "DFS-iter(μs)"
              << std::setw(12) << "rec/BFS"
              << std::setw(12) << "iter/BFS"
              << "\n";
    std::cout << std::string(81, '-') << "\n";

    for (int scale : SCALES) {
        double bfs_sum = 0, rec_sum = 0, iter_sum = 0;
        int valid = 0;
        long long total_edges = 0;

        for (int si = 0; si < SEEDS_PER_SCALE; si++) {
            unsigned seed = (unsigned)(scale * 31 + si * 13 + 7);
            Circuit c = generateRandomCircuit(scale, AVG_FANIN, seed);
            if (!c.topologicalSortBFS()) continue;

            long long edges = 0;
            for (Gate* g : c.getAllGates()) edges += (long long)g->inputNames.size();
            total_edges += edges;

            Stats bfs  = measure(c, Algo::BFS,      WARMUP, ITERS);
            Stats rec  = measure(c, Algo::DFS_REC,  WARMUP, ITERS);
            Stats iter = measure(c, Algo::DFS_ITER, WARMUP, ITERS);

            csv << scale << "," << edges << ",BFS,"
                << std::setprecision(4) << bfs.mean_us  << "," << bfs.std_us  << "," << seed << "\n";
            csv << scale << "," << edges << ",DFS_recursive,"
                << rec.mean_us  << "," << rec.std_us  << "," << seed << "\n";
            csv << scale << "," << edges << ",DFS_iterative,"
                << iter.mean_us << "," << iter.std_us << "," << seed << "\n";

            bfs_sum  += bfs.mean_us;
            rec_sum  += rec.mean_us;
            iter_sum += iter.mean_us;
            valid++;
        }

        if (valid == 0) continue;

        double bfs_avg  = bfs_sum  / valid;
        double rec_avg  = rec_sum  / valid;
        double iter_avg = iter_sum / valid;
        long long avg_edges = total_edges / valid;

        std::cout << std::left
                  << std::setw(7)  << scale
                  << std::setw(10) << avg_edges
                  << std::setw(12) << bfs_avg
                  << std::setw(14) << rec_avg
                  << std::setw(14) << iter_avg
                  << std::setw(12) << (rec_avg  / bfs_avg)
                  << std::setw(12) << (iter_avg / bfs_avg)
                  << "\n";
    }

    csv.close();
    std::cout << "\nCSV 已寫入：results/scale_test.csv\n";
    std::cout << "執行 'python3 plot_bench.py' 可生成圖表。\n\n";
    return 0;
}
