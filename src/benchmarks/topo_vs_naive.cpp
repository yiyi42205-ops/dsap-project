// Benchmark: 拓撲排序 BFS O(V+E) vs Naive 逐輪掃描 O(V²)
// 電路：Ripple Carry Adder（carry chain 放大差距）
// 規模：4-bit (20 gate) / 8-bit (40) / 16-bit (80) / 32-bit (160)
// Naive 策略：每次從頭掃描，找到第一個可計算的閘就計算，再重頭掃，保證 O(V²)
// 輸出：results/topo_vs_naive.csv
//
// 編譯：g++ -std=c++17 -O2 -o topo_vs_naive topo_vs_naive.cpp
// 執行：./topo_vs_naive   （請在 src/benchmarks/ 目錄下執行）

#include <iostream>
#include <fstream>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <queue>
#include <string>
#include <chrono>
#include <algorithm>
#include <random>
#include <iomanip>
#include <filesystem>

using Clock = std::chrono::high_resolution_clock;
using ns    = std::chrono::nanoseconds;

// ── 簡化版 Gate（只存 name / inputs / output）─────────────────────
struct Gate {
    std::string              name;
    std::vector<std::string> inputs;
    std::string              output;
};

// ── 建立 N-bit Ripple Carry Adder ─────────────────────────────────
// 每個 bit：XOR_ia, XOR_ib, AND_ia, AND_ib, OR_i → 5 個閘
// carry chain：OR_i 的輸出 C_i 是 bit i+1 的 XOR_ib / AND_ib 的輸入
std::vector<Gate> buildNBitAdder(int N, std::unordered_set<std::string>& primaryInputs) {
    primaryInputs.clear();
    for (int i = 0; i < N; i++) {
        primaryInputs.insert("A" + std::to_string(i));
        primaryInputs.insert("B" + std::to_string(i));
    }
    primaryInputs.insert("Cin");

    std::vector<Gate> gates;
    gates.reserve(5 * N);
    std::string prevCarry = "Cin";

    for (int i = 0; i < N; i++) {
        std::string a  = "A"  + std::to_string(i);
        std::string b  = "B"  + std::to_string(i);
        std::string t  = "T"  + std::to_string(i);   // A XOR B
        std::string s  = "S"  + std::to_string(i);   // sum bit
        std::string g  = "G"  + std::to_string(i);   // generate
        std::string p  = "P"  + std::to_string(i);   // propagate
        std::string c  = (i == N - 1) ? "Cout" : "C" + std::to_string(i);

        gates.push_back({"XOR_" + std::to_string(i) + "a", {a, b},          t});
        gates.push_back({"XOR_" + std::to_string(i) + "b", {t, prevCarry},  s});
        gates.push_back({"AND_" + std::to_string(i) + "a", {a, b},          g});
        gates.push_back({"AND_" + std::to_string(i) + "b", {t, prevCarry},  p});
        gates.push_back({"OR_"  + std::to_string(i),       {g, p},          c});
        prevCarry = c;
    }

    // 洗牌：讓 Naive scan 無法依靠插入順序獲得便宜
    std::shuffle(gates.begin(), gates.end(), std::mt19937(42));
    return gates;
}

// ── BFS 拓撲排序 (Kahn's Algorithm) O(V+E) ───────────────────────
std::vector<int> bfsTopoSort(const std::vector<Gate>& gates,
                              const std::unordered_set<std::string>& primaryInputs) {
    const int V = (int)gates.size();
    std::unordered_map<std::string, int> signalSource;
    signalSource.reserve(V);
    for (int i = 0; i < V; i++) signalSource[gates[i].output] = i;

    std::vector<int>              inDeg(V, 0);
    std::vector<std::vector<int>> adj(V);

    for (int i = 0; i < V; i++) {
        for (const auto& inp : gates[i].inputs) {
            auto it = signalSource.find(inp);
            if (it != signalSource.end()) {
                adj[it->second].push_back(i);
                inDeg[i]++;
            }
        }
    }

    std::queue<int> q;
    for (int i = 0; i < V; i++) if (inDeg[i] == 0) q.push(i);

    std::vector<int> order;
    order.reserve(V);
    while (!q.empty()) {
        int cur = q.front(); q.pop();
        order.push_back(cur);
        for (int nxt : adj[cur])
            if (--inDeg[nxt] == 0) q.push(nxt);
    }
    return order;
}

// ── Naive Scan O(V²) ──────────────────────────────────────────────
// 每次從頭掃，找到第一個「輸入都已就緒」的閘就計算並 break 重頭掃
// 保證最壞情況 O(V) 次外迴圈 × O(V) 次內掃 = O(V²)
std::vector<int> naiveScan(const std::vector<Gate>& gates,
                            const std::unordered_set<std::string>& primaryInputs) {
    const int V = (int)gates.size();
    std::unordered_set<std::string> ready(primaryInputs.begin(), primaryInputs.end());
    ready.reserve(V + primaryInputs.size());

    std::vector<bool> computed(V, false);
    std::vector<int>  order;
    order.reserve(V);

    int remaining = V;
    while (remaining > 0) {
        bool found = false;
        for (int i = 0; i < V; i++) {
            if (computed[i]) continue;
            bool allReady = true;
            for (const auto& inp : gates[i].inputs) {
                if (!ready.count(inp)) { allReady = false; break; }
            }
            if (allReady) {
                computed[i] = true;
                ready.insert(gates[i].output);
                order.push_back(i);
                remaining--;
                found = true;
                break;   // ← 重頭掃，這是讓 complexity 是 O(V²) 的關鍵
            }
        }
        if (!found) break; // cycle detected
    }
    return order;
}

int main() {
    std::filesystem::create_directories("results");

    const std::vector<int> BITS = {4, 8, 16, 32};
    const int REPS = 80000;

    std::ofstream csv("results/topo_vs_naive.csv");
    csv << "bits,gates,BFS_us,Naive_us,Speedup\n";

    std::cout << "Naive 策略：每次從頭掃，找到第一個可算的閘就 break 重掃 → O(V²)\n";
    std::cout << "BFS 策略  ：Kahn's Algorithm → O(V+E)\n";
    std::cout << "閘順序已洗牌，消除插入順序帶來的巧合\n\n";

    std::cout << std::setw(6)  << "bits"
              << std::setw(8)  << "gates"
              << std::setw(16) << "BFS (μs/call)"
              << std::setw(18) << "Naive (μs/call)"
              << std::setw(12) << "Speedup\n";
    std::cout << std::string(60, '-') << "\n";

    for (int N : BITS) {
        std::unordered_set<std::string> primaryInputs;
        auto gates = buildNBitAdder(N, primaryInputs);
        int V = (int)gates.size();

        // Warmup
        { auto o1 = bfsTopoSort(gates, primaryInputs); (void)o1; }
        { auto o2 = naiveScan(gates, primaryInputs);   (void)o2; }

        // ── BFS timing ────────────────────────────────────
        long long bfsTotal = 0;
        for (int r = 0; r < REPS; r++) {
            auto t1 = Clock::now();
            auto order = bfsTopoSort(gates, primaryInputs);
            auto t2 = Clock::now();
            bfsTotal += std::chrono::duration_cast<ns>(t2 - t1).count();
            (void)order;
        }

        // ── Naive timing ──────────────────────────────────
        long long naiveTotal = 0;
        for (int r = 0; r < REPS; r++) {
            auto t1 = Clock::now();
            auto order = naiveScan(gates, primaryInputs);
            auto t2 = Clock::now();
            naiveTotal += std::chrono::duration_cast<ns>(t2 - t1).count();
            (void)order;
        }

        // 平均每次（ns → μs）
        double bfsUs   = (double)bfsTotal   / REPS / 1000.0;
        double naiveUs = (double)naiveTotal / REPS / 1000.0;
        double speedup = (bfsUs > 0) ? naiveUs / bfsUs : 0.0;

        std::cout << std::setw(6)  << N
                  << std::setw(8)  << V
                  << std::setw(16) << std::fixed << std::setprecision(3) << bfsUs
                  << std::setw(18) << naiveUs
                  << std::setw(11) << std::setprecision(1) << speedup << "x\n";

        csv << N << "," << V << ","
            << std::fixed << std::setprecision(3)
            << bfsUs << "," << naiveUs << ","
            << std::setprecision(2) << speedup << "\n";
    }

    csv.close();
    std::cout << "\nResults saved to results/topo_vs_naive.csv\n";
    return 0;
}
