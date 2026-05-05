// Benchmark: std::unordered_map O(1) vs std::vector<pair> O(N) 元件查詢
// 測試規模：N = 10, 50, 100, 500, 1000, 5000 個 gate
// 每組查詢 1000 次，重複 20 輪取平均（穩定計時）
// 輸出：results/hash_vs_linear.csv
//
// 編譯：g++ -std=c++17 -O2 -o hash_vs_linear hash_vs_linear.cpp
// 執行：./hash_vs_linear   （請在 src/benchmarks/ 目錄下執行）

#include <iostream>
#include <fstream>
#include <vector>
#include <unordered_map>
#include <string>
#include <chrono>
#include <random>
#include <iomanip>
#include <filesystem>

using Clock = std::chrono::high_resolution_clock;
using us    = std::chrono::microseconds;

int main() {
    std::filesystem::create_directories("results");

    const std::vector<int> SIZES   = {10, 50, 100, 500, 1000, 5000};
    const int              QUERIES = 1000;
    const int              REPS    = 20;   // 每個 N 重複幾輪取平均

    std::mt19937 rng(42);

    std::ofstream csv("results/hash_vs_linear.csv");
    csv << "N,HashMap_us,Linear_us,Speedup\n";

    std::cout << std::setw(6)  << "N"
              << std::setw(16) << "HashMap (μs)"
              << std::setw(16) << "Linear (μs)"
              << std::setw(12) << "Speedup\n";
    std::cout << std::string(50, '-') << "\n";

    for (int N : SIZES) {
        // ── 建立測試資料 ──────────────────────────────────
        std::vector<std::string> names;
        names.reserve(N);
        for (int i = 0; i < N; i++)
            names.push_back("GATE_" + std::to_string(i));

        std::unordered_map<std::string, int> hashMap;
        hashMap.reserve(N);
        for (int i = 0; i < N; i++) hashMap[names[i]] = i;

        std::vector<std::pair<std::string, int>> linearVec;
        linearVec.reserve(N);
        for (int i = 0; i < N; i++) linearVec.push_back({names[i], i});

        // 隨機查詢序列
        std::uniform_int_distribution<int> dist(0, N - 1);
        std::vector<int> queries(QUERIES);
        for (int& q : queries) q = dist(rng);

        // Warmup
        volatile int sink = 0;
        for (int q : queries) {
            auto it = hashMap.find(names[q]);
            if (it != hashMap.end()) sink += it->second;
        }

        // ── Benchmark: unordered_map ──────────────────────
        long long hashTotal = 0;
        for (int r = 0; r < REPS; r++) {
            auto t1 = Clock::now();
            for (int q : queries) {
                auto it = hashMap.find(names[q]);
                if (it != hashMap.end()) sink += it->second;
            }
            auto t2 = Clock::now();
            hashTotal += std::chrono::duration_cast<us>(t2 - t1).count();
        }
        double hashAvg = (double)hashTotal / REPS;

        // ── Benchmark: vector<pair> linear scan ──────────
        long long linearTotal = 0;
        for (int r = 0; r < REPS; r++) {
            auto t1 = Clock::now();
            for (int q : queries) {
                for (const auto& p : linearVec) {
                    if (p.first == names[q]) { sink += p.second; break; }
                }
            }
            auto t2 = Clock::now();
            linearTotal += std::chrono::duration_cast<us>(t2 - t1).count();
        }
        double linearAvg = (double)linearTotal / REPS;

        double speedup = (hashAvg > 0) ? linearAvg / hashAvg : 0.0;

        std::cout << std::setw(6)  << N
                  << std::setw(16) << std::fixed << std::setprecision(1) << hashAvg
                  << std::setw(16) << linearAvg
                  << std::setw(11) << std::setprecision(1) << speedup << "x\n";

        csv << N << ","
            << std::fixed << std::setprecision(2)
            << hashAvg << "," << linearAvg << ","
            << std::setprecision(2) << speedup << "\n";

        (void)sink; // suppress unused warning
    }

    csv.close();
    std::cout << "\nResults saved to results/hash_vs_linear.csv\n";
    return 0;
}
