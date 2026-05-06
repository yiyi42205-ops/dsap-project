// random_circuit_gen.cpp
// 隨機生成合法 DAG 電路，嚴謹測量 BFS vs DFS 拓撲排序效能
//
// 設計：
//   - 生成策略：層序法保證無環——每個 gate 只能以已存在的訊號為輸入
//   - 規模：100 / 500 / 1000 / 5000 / 10000 個 gate
//   - 每個規模生成 10 個不同隨機電路（種子固定，結果可重現）
//   - 測量流程：100 次 warm-up（丟棄）→ 1000 次正式測量 → 計算均值與標準差
//   - 輸出：stdout 彙總表 + results/random_circuit_bench.csv（供 plot.py 繪圖）
//
// 編譯：g++ -std=c++17 -O2 -o random_circuit_gen random_circuit_gen.cpp
// 執行：./random_circuit_gen（需先建立 results/ 目錄）

#include "../circuit.h"
#include <random>
#include <numeric>
#include <cmath>
#include <fstream>
#include <iomanip>

// ============================================================
// 1. 隨機合法 DAG 生成器
// ============================================================
// 演算法：
//   1. 建立 n_inputs 個主要輸入訊號（可用訊號集 = {IN_0, ..., IN_{n-1}}）
//   2. 依序生成 gate_count 個閘：
//      每個閘從已存在的訊號集中隨機選取 fan_in 個輸入
//      閘的輸出加入可用訊號集
//   3. 此順序天然是拓撲序，保證 DAG
//
Circuit generateRandomCircuit(int gate_count, int avg_fanin, unsigned seed) {
    std::mt19937 rng(seed);

    // 主要輸入數量：取 sqrt(N) 確保訊號池夠豐富但不過大
    int n_inputs = std::max(3, (int)std::sqrt((double)gate_count));

    Circuit c;
    std::vector<std::string> available; // 目前可作為輸入的訊號

    for (int i = 0; i < n_inputs; i++) {
        std::string s = "IN_" + std::to_string(i);
        c.addInput(s);
        available.push_back(s);
    }

    // 閘類型：不含 NOT（單輸入），以維持 avg_fanin >= 2 的預期
    static const std::vector<std::string> types = {"AND", "OR", "XOR", "NAND", "NOR"};
    std::uniform_int_distribution<int> typeDist(0, (int)types.size() - 1);

    for (int i = 0; i < gate_count; i++) {
        std::string gname = "G_" + std::to_string(i);
        std::string oname = "W_" + std::to_string(i);

        // 確定 fan_in：至少 2，最多 min(available.size(), avg_fanin + 2)
        int max_fi = std::min((int)available.size(), avg_fanin + 2);
        int fi = std::max(2, std::min(max_fi, avg_fanin));

        // 從可用訊號池隨機抽取 fi 個不重複輸入
        // 用 partial_shuffle 取前 fi 個，O(fi) 效率
        std::vector<int> idx(available.size());
        std::iota(idx.begin(), idx.end(), 0);
        for (int j = 0; j < fi; j++) {
            std::uniform_int_distribution<int> d(j, (int)idx.size() - 1);
            std::swap(idx[j], idx[d(rng)]);
        }

        std::vector<std::string> inputs;
        inputs.reserve(fi);
        for (int j = 0; j < fi; j++) inputs.push_back(available[idx[j]]);

        c.addGate(types[typeDist(rng)], gname, inputs, oname);
        available.push_back(oname);
    }

    // 最後幾個閘的輸出作為電路輸出
    int n_outputs = std::max(1, gate_count / 20);
    for (int i = 0; i < n_outputs; i++)
        c.addOutput("W_" + std::to_string(gate_count - 1 - i));

    return c;
}

// ============================================================
// 2. 計時工具
// ============================================================
using ns = long long;

struct Stats {
    double mean_ns;
    double std_ns;
    double min_ns;
    double max_ns;
};

Stats computeStats(const std::vector<ns>& v) {
    double m = 0;
    for (auto x : v) m += x;
    m /= v.size();

    double sq = 0;
    for (auto x : v) sq += ((double)x - m) * ((double)x - m);
    double s = std::sqrt(sq / v.size());

    ns lo = *std::min_element(v.begin(), v.end());
    ns hi = *std::max_element(v.begin(), v.end());

    return {m, s, (double)lo, (double)hi};
}

Stats benchmarkBFS(Circuit& c, int warmup, int iters) {
    // Warm-up：讓 CPU cache 和分支預測器穩定
    for (int i = 0; i < warmup; i++) c.topologicalSortBFS();

    std::vector<ns> times(iters);
    for (int i = 0; i < iters; i++) {
        auto t0 = std::chrono::high_resolution_clock::now();
        c.topologicalSortBFS();
        auto t1 = std::chrono::high_resolution_clock::now();
        times[i] = std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0).count();
    }
    return computeStats(times);
}

Stats benchmarkDFS(Circuit& c, int warmup, int iters) {
    for (int i = 0; i < warmup; i++) c.topologicalSortDFS();

    std::vector<ns> times(iters);
    for (int i = 0; i < iters; i++) {
        auto t0 = std::chrono::high_resolution_clock::now();
        c.topologicalSortDFS();
        auto t1 = std::chrono::high_resolution_clock::now();
        times[i] = std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0).count();
    }
    return computeStats(times);
}

// ============================================================
// 3. 主程式
// ============================================================
int main() {
    const std::vector<int> SCALES = {100, 500, 1000, 5000, 10000};
    const int CIRCUITS_PER_SCALE  = 10;
    const int WARMUP              = 100;
    const int ITERS               = 1000;
    const int AVG_FANIN           = 3;

    // 準備 CSV
    std::ofstream csv("results/random_circuit_bench.csv");
    if (!csv.is_open()) {
        std::cerr << "錯誤：無法建立 results/random_circuit_bench.csv\n"
                  << "請先執行 mkdir -p results\n";
        return 1;
    }
    csv << "scale,circuit_id,bfs_mean_ns,bfs_std_ns,bfs_min_ns,bfs_max_ns,"
           "dfs_mean_ns,dfs_std_ns,dfs_min_ns,dfs_max_ns\n";

    std::cout << std::fixed;
    std::cout << "\n╔═══════════════════════════════════════════════════════════════════╗\n";
    std::cout << "║  隨機 DAG 電路 BFS vs DFS 拓撲排序嚴謹效能測試               ║\n";
    std::cout << "╠═══════════════════════════════════════════════════════════════════╣\n";
    std::cout << "║  fan-in avg=" << AVG_FANIN
              << "  warm-up=" << std::setw(3) << WARMUP
              << "  測量=" << ITERS << " 次  每規模 " << CIRCUITS_PER_SCALE << " 個電路  ║\n";
    std::cout << "╚═══════════════════════════════════════════════════════════════════╝\n\n";

    std::cout << std::left
              << std::setw(8)  << "Scale"
              << std::setw(14) << "BFS mean(ns)"
              << std::setw(14) << "BFS std(ns)"
              << std::setw(14) << "DFS mean(ns)"
              << std::setw(14) << "DFS std(ns)"
              << std::setw(10) << "BFS/DFS"
              << "\n";
    std::cout << std::string(74, '-') << "\n";

    for (int scale : SCALES) {
        // 收集 10 個電路的統計數據
        std::vector<double> bfs_means, bfs_stds, dfs_means, dfs_stds;
        bool all_valid = true;

        for (int cid = 0; cid < CIRCUITS_PER_SCALE; cid++) {
            unsigned seed = (unsigned)(scale * 1000 + cid * 7 + 42);
            Circuit c = generateRandomCircuit(scale, AVG_FANIN, seed);

            // 驗證電路有效
            if (!c.topologicalSortBFS()) {
                std::cerr << "生成了無效電路 scale=" << scale << " cid=" << cid << "\n";
                all_valid = false;
                continue;
            }

            Stats bs = benchmarkBFS(c, WARMUP, ITERS);
            Stats ds = benchmarkDFS(c, WARMUP, ITERS);

            bfs_means.push_back(bs.mean_ns);
            bfs_stds.push_back(bs.std_ns);
            dfs_means.push_back(ds.mean_ns);
            dfs_stds.push_back(ds.std_ns);

            csv << scale << "," << cid << ","
                << std::setprecision(1) << bs.mean_ns << "," << bs.std_ns << ","
                << bs.min_ns << "," << bs.max_ns << ","
                << ds.mean_ns << "," << ds.std_ns << ","
                << ds.min_ns << "," << ds.max_ns << "\n";
        }

        if (!all_valid || bfs_means.empty()) continue;

        // 對 10 個電路取平均
        auto agg = [](const std::vector<double>& v) {
            return std::accumulate(v.begin(), v.end(), 0.0) / v.size();
        };
        double bm = agg(bfs_means), bs = agg(bfs_stds);
        double dm = agg(dfs_means), ds = agg(dfs_stds);

        std::cout << std::left << std::setprecision(1)
                  << std::setw(8)  << scale
                  << std::setw(14) << bm
                  << std::setw(14) << bs
                  << std::setw(14) << dm
                  << std::setw(14) << ds
                  << std::setprecision(3) << std::setw(10) << (bm / dm)
                  << "\n";
    }

    csv.close();
    std::cout << "\nCSV 已寫入：results/random_circuit_bench.csv\n";
    std::cout << "執行 'python3 plot.py' 可生成 log-log 圖表。\n\n";
    return 0;
}
