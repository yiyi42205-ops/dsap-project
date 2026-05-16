
#pragma once
// critical_path.h ── Top-K 最長路徑（Critical Path）分析模組
//
// 演算法：
//   Step 1. 以 BFS 拓撲排序建立計算順序（內部獨立執行，不影響 Circuit 的 executionOrder）
//   Step 2. 正向 DAG DP：dist[g] = g->delay + max(dist[pred] for all predecessor gates)
//   Step 3. Max-Heap 反向搜尋 Top-K 路徑
//
// 優先權設計（保證正確性）：
//   heap entry priority = incoming[frontier] + suffix_delay
//                       = (dist[frontier] - frontier->delay) + suffix_delay
//   其中 suffix_delay = 路徑中 frontier 到輸出端所有閘的 delay 總和
//   - 初始（frontier = output gate G）：priority = dist[G]
//   - 延伸到前驅 P：new_priority = dist[P] + cur.suffix_delay
//   - 完整路徑（抵達 primary input）：priority = suffix_delay = 實際總延遲
//
// 此設計確保：heap 優先彈出「實際延遲最大」的完整路徑，前 K 個完整路徑即 Top-K。

#include "circuit.h"
#include <vector>
#include <queue>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <algorithm>

// ── 資料結構 ───────────────────────────────────────────────────
struct CriticalPath {
    int totalDelay;
    std::vector<std::string> nodes;  // 從 primary input 到 output signal 的節點序列
};

// ── 內部 heap entry ────────────────────────────────────────────
struct CPHeapEntry {
    int priority;      // 上界（完整路徑時 == 實際 totalDelay）
    int suffixDelay;   // frontier 到輸出端的 gate delay 累計
    std::vector<std::string> nodes;  // 右到左：output signal 在 front
    Gate* frontier;    // 目前最左邊的 gate；nullptr 表示完整路徑

    bool operator<(const CPHeapEntry& o) const { return priority < o.priority; }
};

// ── 主要分析函式 ───────────────────────────────────────────────
// allGates      : circuit.getAllGates()
// inputSignals  : circuit.getInputs()
// outputSignals : circuit.getOutputs()
// K             : 要找幾條最長路徑
inline std::vector<CriticalPath> computeTopKCriticalPaths(
    const std::vector<Gate*>&        allGates,
    const std::vector<std::string>&  inputSignals,
    const std::vector<std::string>&  outputSignals,
    int  K)
{
    if (K <= 0 || allGates.empty()) return {};

    // ── 建立輔助映射 ──────────────────────────────────────────
    // signalSource[signal_name] = 產生該訊號的 Gate*
    std::unordered_map<std::string, Gate*> signalSource;
    for (Gate* g : allGates) signalSource[g->outputName] = g;

    // primary inputs 集合
    std::unordered_set<std::string> primaryInputs(
        inputSignals.begin(), inputSignals.end());

    // ── Step 1：內部 BFS 拓撲排序（僅供 DP 使用）────────────
    std::unordered_map<Gate*, int> inDegree;
    std::unordered_map<Gate*, std::vector<Gate*>> adj;
    for (Gate* g : allGates) inDegree[g] = 0;
    for (Gate* g : allGates)
        for (const auto& inp : g->inputNames)
            if (signalSource.count(inp)) inDegree[g]++;
    for (Gate* g : allGates)
        for (const auto& inp : g->inputNames)
            if (signalSource.count(inp))
                adj[signalSource.at(inp)].push_back(g);

    std::deque<Gate*> bfsQ;
    for (Gate* g : allGates)
        if (inDegree[g] == 0) bfsQ.push_back(g);
    std::vector<Gate*> topoOrder;
    topoOrder.reserve(allGates.size());
    while (!bfsQ.empty()) {
        Gate* cur = bfsQ.front(); bfsQ.pop_front();
        topoOrder.push_back(cur);
        for (Gate* nxt : adj[cur])
            if (--inDegree[nxt] == 0) bfsQ.push_back(nxt);
    }

    // ── Step 2：正向 DP ────────────────────────────────────
    // dist[g] = g->delay + max(dist[pred] for all pred gates)
    std::unordered_map<Gate*, int> dist;
    for (Gate* g : topoOrder) {
        int maxPred = 0;
        for (const auto& inp : g->inputNames) {
            auto it = signalSource.find(inp);
            if (it != signalSource.end())
                maxPred = std::max(maxPred, dist.at(it->second));
        }
        dist[g] = g->delay + maxPred;
    }

    // ── 建立反向邊（predecessor gates）────────────────────
    std::unordered_map<Gate*, std::vector<Gate*>> preds;
    for (Gate* g : allGates) preds[g]; // 確保每個閘都有對應的空 vector
    for (Gate* g : allGates) {
        std::unordered_set<Gate*> seen;
        for (const auto& inp : g->inputNames) {
            auto it = signalSource.find(inp);
            if (it != signalSource.end() && seen.insert(it->second).second)
                preds[g].push_back(it->second);
        }
    }

    // ── Step 3：Max-Heap 反向搜尋 Top-K 路徑 ─────────────
    std::priority_queue<CPHeapEntry> heap;

    // 初始化：每個 output signal 的產生閘各推一個 entry
    for (const std::string& out : outputSignals) {
        auto it = signalSource.find(out);
        if (it == signalSource.end()) continue;
        Gate* G = it->second;
        heap.push({dist.at(G), G->delay, {out, G->name}, G});
    }

    std::vector<CriticalPath> results;

    while (!heap.empty() && (int)results.size() < K) {
        CPHeapEntry cur = heap.top(); heap.pop();

        // ── 完整路徑：直接加入結果 ────────────────────────
        if (cur.frontier == nullptr) {
            CriticalPath cp;
            cp.totalDelay = cur.suffixDelay;
            cp.nodes = cur.nodes;
            std::reverse(cp.nodes.begin(), cp.nodes.end());
            results.push_back(cp);
            continue;
        }

        Gate* G = cur.frontier;

        // ── 找出直接以 primary input 餵入 G 的訊號 ────────
        // 每個不同的 primary input 代表一條完整路徑
        std::unordered_set<std::string> seenPrimary;
        for (const auto& inp : G->inputNames) {
            if (primaryInputs.count(inp) && seenPrimary.insert(inp).second) {
                std::vector<std::string> completeNodes = cur.nodes;
                completeNodes.push_back(inp);
                // 完整路徑的 priority == suffixDelay（無 incoming 餘量）
                heap.push({cur.suffixDelay, cur.suffixDelay,
                           std::move(completeNodes), nullptr});
            }
        }

        // ── 延伸至前驅 gate ────────────────────────────────
        // new_priority = dist[P] + cur.suffixDelay
        //              = incoming[P] + (suffixDelay + P->delay)
        for (Gate* P : preds.at(G)) {
            int newSuffix   = cur.suffixDelay + P->delay;
            int newPriority = dist.at(P) + cur.suffixDelay;
            std::vector<std::string> newNodes = cur.nodes;
            newNodes.push_back(P->name);
            heap.push({newPriority, newSuffix, std::move(newNodes), P});
        }
    }

    // 依延遲由大到小排序（heap 彈出順序已保證，但以防萬一）
    std::sort(results.begin(), results.end(),
        [](const CriticalPath& a, const CriticalPath& b) {
            return a.totalDelay > b.totalDelay;
        });

    return results;
}

// ── 印出 Critical Path 分析結果 ──────────────────────────────
inline void printCriticalPaths(const std::vector<CriticalPath>& paths) {
    std::cout << "\n=== Critical Path Analysis ===\n";
    if (paths.empty()) {
        std::cout << "（無法找到任何路徑）\n";
        return;
    }
    for (int i = 0; i < (int)paths.size(); i++) {
        const auto& cp = paths[i];
        int freq = (cp.totalDelay > 0) ? (1000 / cp.totalDelay) : 0;
        std::cout << "#" << (i + 1)
                  << " 延遲: " << cp.totalDelay << " ns"
                  << " (最高頻率: " << freq << " MHz)\n";
        std::cout << "   路徑: ";
        for (int j = 0; j < (int)cp.nodes.size(); j++) {
            if (j > 0) std::cout << " → ";
            std::cout << cp.nodes[j];
        }
        std::cout << "\n";
    }
}
