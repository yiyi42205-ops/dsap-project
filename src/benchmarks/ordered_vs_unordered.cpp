// Benchmark: std::map (紅黑樹) vs std::unordered_map 用於真值表訊號儲存
// 重點：
//   1. 效能比較：插入 / 查詢 / 迭代
//   2. 正確性展示：std::map 保證字典序迭代 → 真值表欄位順序固定
//                  std::unordered_map 迭代順序不確定 → 真值表欄位順序不定
// 輸出：results/ordered_vs_unordered.csv
//
// 編譯：g++ -std=c++17 -O2 -o ordered_vs_unordered ordered_vs_unordered.cpp
// 執行：./ordered_vs_unordered   （請在 src/benchmarks/ 目錄下執行）

#include <iostream>
#include <fstream>
#include <vector>
#include <map>
#include <unordered_map>
#include <string>
#include <chrono>
#include <iomanip>
#include <filesystem>
#include <algorithm>
#include <random>

using Clock = std::chrono::high_resolution_clock;
using us    = std::chrono::microseconds;

// 產生 N 個訊號名稱，故意用非字母序插入（模擬電路解析後的結果）
std::vector<std::string> genSignalNames(int N, bool shuffle = true) {
    std::vector<std::string> names;
    names.reserve(N);
    for (int i = 0; i < N; i++) {
        char buf[32];
        snprintf(buf, sizeof(buf), "sig_%03d", i);
        names.push_back(buf);
    }
    if (shuffle) {
        // 打亂插入順序，模擬電路解析的非排序結果
        std::shuffle(names.begin(), names.end(), std::mt19937(42));
    }
    return names;
}

int main() {
    std::filesystem::create_directories("results");

    const std::vector<int> SIZES = {10, 50, 100, 500, 1000};
    const int REPS = 500;

    std::ofstream csv("results/ordered_vs_unordered.csv");
    csv << "N,Map_insert_us,Unordered_insert_us,Map_lookup_us,Unordered_lookup_us,"
           "Map_iter_us,Unordered_iter_us\n";

    std::cout << std::setw(6)  << "N"
              << std::setw(18) << "map insert(μs)"
              << std::setw(22) << "unord insert(μs)"
              << std::setw(17) << "map look(μs)"
              << std::setw(20) << "unord look(μs)"
              << std::setw(15) << "map iter(μs)"
              << std::setw(18) << "unord iter(μs)\n";
    std::cout << std::string(116, '-') << "\n";

    for (int N : SIZES) {
        auto names = genSignalNames(N, /*shuffle=*/true);

        // ── 插入效能 ─────────────────────────────────────
        long long mapInsTotal = 0, unordInsTotal = 0;
        for (int r = 0; r < REPS; r++) {
            {
                std::map<std::string, bool> m;
                auto t1 = Clock::now();
                for (int i = 0; i < N; i++) m[names[i]] = (i % 2 == 0);
                auto t2 = Clock::now();
                mapInsTotal += std::chrono::duration_cast<us>(t2 - t1).count();
            }
            {
                std::unordered_map<std::string, bool> um;
                um.reserve(N);
                auto t1 = Clock::now();
                for (int i = 0; i < N; i++) um[names[i]] = (i % 2 == 0);
                auto t2 = Clock::now();
                unordInsTotal += std::chrono::duration_cast<us>(t2 - t1).count();
            }
        }

        // ── 查詢效能 ─────────────────────────────────────
        std::map<std::string, bool>           m;
        std::unordered_map<std::string, bool> um;
        um.reserve(N);
        for (int i = 0; i < N; i++) { m[names[i]] = (i % 2 == 0); um[names[i]] = (i % 2 == 0); }

        volatile bool sink = false;
        long long mapLookTotal = 0, unordLookTotal = 0;
        for (int r = 0; r < REPS; r++) {
            {
                auto t1 = Clock::now();
                for (const auto& name : names) {
                    auto it = m.find(name);
                    if (it != m.end()) sink = it->second;
                }
                auto t2 = Clock::now();
                mapLookTotal += std::chrono::duration_cast<us>(t2 - t1).count();
            }
            {
                auto t1 = Clock::now();
                for (const auto& name : names) {
                    auto it = um.find(name);
                    if (it != um.end()) sink = it->second;
                }
                auto t2 = Clock::now();
                unordLookTotal += std::chrono::duration_cast<us>(t2 - t1).count();
            }
        }

        // ── 迭代效能（真值表輸出場景）────────────────────
        long long mapIterTotal = 0, unordIterTotal = 0;
        for (int r = 0; r < REPS; r++) {
            {
                auto t1 = Clock::now();
                std::string last;
                for (const auto& kv : m) last = kv.first; // 字典序保證
                auto t2 = Clock::now();
                mapIterTotal += std::chrono::duration_cast<us>(t2 - t1).count();
                (void)last;
            }
            {
                auto t1 = Clock::now();
                std::string last;
                for (const auto& kv : um) last = kv.first; // 順序不定
                auto t2 = Clock::now();
                unordIterTotal += std::chrono::duration_cast<us>(t2 - t1).count();
                (void)last;
            }
        }

        double mIns   = (double)mapInsTotal   / REPS;
        double uIns   = (double)unordInsTotal  / REPS;
        double mLook  = (double)mapLookTotal   / REPS;
        double uLook  = (double)unordLookTotal / REPS;
        double mIter  = (double)mapIterTotal   / REPS;
        double uIter  = (double)unordIterTotal / REPS;

        std::cout << std::setw(6)  << N
                  << std::setw(18) << std::fixed << std::setprecision(2) << mIns
                  << std::setw(22) << uIns
                  << std::setw(17) << mLook
                  << std::setw(20) << uLook
                  << std::setw(15) << mIter
                  << std::setw(18) << uIter << "\n";

        csv << N << ","
            << std::fixed << std::setprecision(2)
            << mIns  << "," << uIns  << ","
            << mLook << "," << uLook << ","
            << mIter << "," << uIter << "\n";

        (void)sink;
    }

    // ── 正確性示範：欄位順序 ─────────────────────────────
    std::cout << "\n=== 真值表欄位順序示範 ===\n";
    std::cout << "電路訊號（非排序插入）：Cout A B S Cin\n\n";

    std::vector<std::string> demo = {"Cout", "A", "B", "S", "Cin"};
    std::map<std::string, bool>           ordMap;
    std::unordered_map<std::string, bool> unordMap;
    for (const auto& n : demo) { ordMap[n] = true; unordMap[n] = true; }

    std::cout << "std::map 迭代順序（字典序保證，真值表欄位固定）：";
    for (const auto& kv : ordMap) std::cout << kv.first << " ";
    std::cout << "\n";

    std::cout << "std::unordered_map 迭代順序（不定，每次執行可能不同）：";
    for (const auto& kv : unordMap) std::cout << kv.first << " ";
    std::cout << "\n";

    std::cout << "\n→ 結論：真值表的輸入訊號欄位需要使用 std::map 或手動排序，\n";
    std::cout << "         才能確保輸出結果在不同執行間保持一致。\n";

    csv.close();
    std::cout << "\nResults saved to results/ordered_vs_unordered.csv\n";
    return 0;
}
