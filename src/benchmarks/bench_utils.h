#pragma once
// random_dag.h ── 隨機合法 DAG 電路生成器
//
// 演算法：層序法保證無環
//   1. 建立 n_inputs 個主要輸入訊號（可用訊號集初始化）
//   2. 依序生成 gate_count 個閘：
//      每個閘從已存在的訊號集中隨機抽 fan_in 個輸入
//      閘的輸出加入可用訊號集
//   3. 此順序天然是拓撲序，保證 DAG
//
// 參數：
//   gate_count : 邏輯閘總數
//   avg_fanin  : 平均扇入數（實際 fan-in 在 [2, avg_fanin+2] 間）
//   seed       : 亂數種子（固定 seed 可重現）

#include "../circuit.h"
#include <random>
#include <numeric>
#include <cmath>

inline Circuit generateRandomCircuit(int gate_count, int avg_fanin, unsigned seed) {
    std::mt19937 rng(seed);

    // 主要輸入數量：sqrt(N)，確保訊號池夠豐富
    int n_inputs = std::max(3, (int)std::sqrt((double)gate_count));

    Circuit c;
    std::vector<std::string> available;

    for (int i = 0; i < n_inputs; i++) {
        std::string s = "IN_" + std::to_string(i);
        c.addInput(s);
        available.push_back(s);
    }

    // 不含 NOT（單輸入），維持 avg_fanin >= 2 的預期
    static const std::vector<std::string> types = {"AND", "OR", "XOR", "NAND", "NOR"};
    std::uniform_int_distribution<int> typeDist(0, (int)types.size() - 1);

    for (int i = 0; i < gate_count; i++) {
        std::string gname = "G_" + std::to_string(i);
        std::string oname = "W_" + std::to_string(i);

        int max_fi = std::min((int)available.size(), avg_fanin + 2);
        int fi = std::max(2, std::min(max_fi, avg_fanin));

        // partial_shuffle 抽 fi 個不重複輸入，O(fi)
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

    int n_outputs = std::max(1, gate_count / 20);
    for (int i = 0; i < n_outputs; i++)
        c.addOutput("W_" + std::to_string(gate_count - 1 - i));

    return c;
}

// ── generateDeepChain ────────────────────────────────────────
// 線性鏈狀 DAG：IN_0 → G_0(NOT) → G_1(NOT) → ... → G_{n-1}(NOT)
// 深度 = num_gates，扇入扇出皆為 1。預期 DFS 因堆疊順序優勢明顯快於 BFS。
inline Circuit generateDeepChain(int num_gates) {
    Circuit c;
    c.addInput("IN_0");

    std::string prev = "IN_0";
    for (int i = 0; i < num_gates; i++) {
        std::string gname = "G_" + std::to_string(i);
        std::string oname = "W_" + std::to_string(i);
        c.addGate("NOT", gname, {prev}, oname);
        prev = oname;
    }
    c.addOutput("W_" + std::to_string(num_gates - 1));
    return c;
}

// ── generateWideFanout ───────────────────────────────────────
// 寬扁狀 DAG（兩層）：
//   層 1：IN_0, IN_1 → N/2 個 AND 閘（每閘輸入皆為 IN_0, IN_1）
//   層 2：所有 N/2 個 AND 閘的輸出 → 1 個聚合 AND 閘
// 深度 = 2，最大扇出 = N/2。預期兩演算法接近（BFS 可能略勝）。
inline Circuit generateWideFanout(int num_gates) {
    Circuit c;
    c.addInput("IN_0");
    c.addInput("IN_1");

    int n_mid = std::max(2, num_gates / 2);
    std::vector<std::string> mid_outputs;

    for (int i = 0; i < n_mid; i++) {
        std::string gname = "G_" + std::to_string(i);
        std::string oname = "W_" + std::to_string(i);
        c.addGate("AND", gname, {"IN_0", "IN_1"}, oname);
        mid_outputs.push_back(oname);
    }

    // 聚合 AND 閘
    c.addGate("AND", "G_agg", mid_outputs, "W_agg");
    c.addOutput("W_agg");
    return c;
}
