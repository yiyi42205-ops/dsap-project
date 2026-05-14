
#pragma once
// circuit.h ── Gate / Circuit 類別定義（header-only）
// 由 main.cpp 和 tests/ 共同 include，確保單一事實來源。

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <unordered_map>
#include <map>
#include <queue>
#include <stack>
#include <algorithm>
#include <cmath>
#include <sstream>
#include <memory>
#include <chrono>
#include <functional>
#include <iomanip>
#include <random>

// ============================================================
// 1. 邏輯閘基底類別 (Gate Base Class)
// ============================================================
class Gate {
public:
    std::string name;
    std::string type;
    std::vector<std::string> inputNames;
    std::string outputName;
    int delay;

    Gate(std::string name, std::string type, int delay = 1)
        : name(name), type(type), delay(delay) {}

    virtual ~Gate() = default;
    virtual bool compute(const std::vector<bool>& inputs) = 0;
};

// ============================================================
// 2. 各種邏輯閘子類別
// ============================================================
class AndGate : public Gate {
public:
    AndGate(std::string name) : Gate(name, "AND", 2) {}
    bool compute(const std::vector<bool>& inputs) override {
        for (bool val : inputs) { if (!val) return false; }
        return true;
    }
};

class OrGate : public Gate {
public:
    OrGate(std::string name) : Gate(name, "OR", 2) {}
    bool compute(const std::vector<bool>& inputs) override {
        for (bool val : inputs) { if (val) return true; }
        return false;
    }
};

class NotGate : public Gate {
public:
    NotGate(std::string name) : Gate(name, "NOT", 1) {}
    bool compute(const std::vector<bool>& inputs) override {
        return !inputs[0];
    }
};

class XorGate : public Gate {
public:
    XorGate(std::string name) : Gate(name, "XOR", 3) {}
    bool compute(const std::vector<bool>& inputs) override {
        bool result = false;
        for (bool val : inputs) { result = result ^ val; }
        return result;
    }
};

class NandGate : public Gate {
public:
    NandGate(std::string name) : Gate(name, "NAND", 2) {}
    bool compute(const std::vector<bool>& inputs) override {
        for (bool val : inputs) { if (!val) return true; }
        return false;
    }
};

class NorGate : public Gate {
public:
    NorGate(std::string name) : Gate(name, "NOR", 2) {}
    bool compute(const std::vector<bool>& inputs) override {
        for (bool val : inputs) { if (val) return false; }
        return true;
    }
};

// ============================================================
// 3. 電路類別 (Circuit Class)
// ============================================================
class Circuit {
private:
    std::vector<std::unique_ptr<Gate>> gates;
    std::unordered_map<std::string, bool> signals;
    std::vector<std::string> inputSignals;
    std::vector<std::string> outputSignals;
    std::vector<Gate*> executionOrder;

    std::vector<Gate*> bfsOrder;
    std::vector<Gate*> dfsOrder;
    std::vector<Gate*> dfsIterOrder;

public:
    void addInput(const std::string& name) {
        inputSignals.push_back(name);
        signals[name] = false;
    }

    void addOutput(const std::string& name) {
        outputSignals.push_back(name);
    }

    int getInputCount() const { return (int)inputSignals.size(); }
    int getGateCount()  const { return (int)gates.size(); }
    const std::vector<std::string>& getInputs()  const { return inputSignals; }
    const std::vector<std::string>& getOutputs() const { return outputSignals; }

    Gate* addGate(const std::string& type, const std::string& name,
                  const std::vector<std::string>& inputs, const std::string& output) {
        std::unique_ptr<Gate> gate;
        if      (type == "AND")  gate = std::make_unique<AndGate>(name);
        else if (type == "OR")   gate = std::make_unique<OrGate>(name);
        else if (type == "NOT")  gate = std::make_unique<NotGate>(name);
        else if (type == "XOR")  gate = std::make_unique<XorGate>(name);
        else if (type == "NAND") gate = std::make_unique<NandGate>(name);
        else if (type == "NOR")  gate = std::make_unique<NorGate>(name);
        else {
            std::cerr << "錯誤：不支援的閘類型 " << type << std::endl;
            return nullptr;
        }
        gate->inputNames = inputs;
        gate->outputName = output;
        signals[output] = false;
        Gate* ptr = gate.get();
        gates.push_back(std::move(gate));
        return ptr;
    }

    // ── BFS 拓撲排序 (Kahn's Algorithm) O(V+E) ──────────────
    bool topologicalSortBFS(bool trace = false, int max_steps = -1) {
        bfsOrder.clear();

        std::unordered_map<std::string, Gate*> signalSource;
        for (auto& g : gates) signalSource[g->outputName] = g.get();

        std::unordered_map<Gate*, int> inDegree;
        for (auto& g : gates) inDegree[g.get()] = 0;
        for (auto& g : gates)
            for (const auto& inp : g->inputNames)
                if (signalSource.count(inp)) inDegree[g.get()]++;

        std::unordered_map<Gate*, std::vector<Gate*>> adj;
        for (auto& g : gates)
            for (const auto& inp : g->inputNames)
                if (signalSource.count(inp))
                    adj[signalSource[inp]].push_back(g.get());

        // 使用 deque 取代 queue，支援 trace 模式下的隨機存取迭代
        // std::queue 底層預設即為 deque，時間複雜度相同
        std::deque<Gate*> q;
        for (auto& g : gates)
            if (inDegree[g.get()] == 0) q.push_back(g.get());

        if (trace) {
            std::cout << "[BFS Trace] 初始佇列（indegree=0 的閘）: [";
            for (int i = 0; i < (int)q.size(); i++) {
                if (i > 0) std::cout << ", ";
                std::cout << q[i]->name;
            }
            std::cout << "]\n\n";
        }

        int step = 0;
        bool bfs_truncated = false;
        while (!q.empty()) {
            step++;
            Gate* cur = q.front(); q.pop_front();
            bfsOrder.push_back(cur);

            // max_steps 限制：超過後繼續排序但停止印出
            if (trace && max_steps >= 0 && step == max_steps + 1 && !bfs_truncated) {
                std::cout << "... （已達 --max-steps " << max_steps
                          << "，後續步驟省略，排序仍繼續完成）\n\n";
                bfs_truncated = true;
            }

            if (trace && !bfs_truncated) {
                std::cout << "==========\n";
                std::cout << "Step " << step << ": Pop " << cur->name
                          << " (" << cur->type << " gate)\n";

                // 印出佇列目前狀態（pop 之後）
                std::cout << "  Current queue: [";
                for (int i = 0; i < (int)q.size(); i++) {
                    if (i > 0) std::cout << ", ";
                    std::cout << q[i]->name;
                }
                std::cout << "]\n";

                // 印出尚未完成的 indegree（只顯示 > 0 的）
                std::cout << "  Current indegree: {";
                bool firstDeg = true;
                for (auto& [gp, deg] : inDegree) {
                    if (deg > 0) {
                        if (!firstDeg) std::cout << ", ";
                        std::cout << gp->name << ":" << deg;
                        firstDeg = false;
                    }
                }
                std::cout << "}\n";

                std::cout << "  " << cur->name << " outputs to: [";
                const auto& nbrs = adj[cur];
                for (int i = 0; i < (int)nbrs.size(); i++) {
                    if (i > 0) std::cout << ", ";
                    std::cout << nbrs[i]->name;
                }
                std::cout << "]\n";
            }

            for (Gate* nxt : adj[cur]) {
                int old_deg = inDegree[nxt];
                --inDegree[nxt];
                if (inDegree[nxt] == 0) {
                    q.push_back(nxt);
                    if (trace && !bfs_truncated) {
                        std::cout << "  → " << nxt->name << " indegree "
                                  << old_deg << "→0, push to queue\n";
                    }
                } else {
                    if (trace && !bfs_truncated) {
                        std::cout << "  → " << nxt->name << " indegree "
                                  << old_deg << "→" << inDegree[nxt] << "\n";
                    }
                }
            }

            if (trace && !bfs_truncated) std::cout << "==========\n\n";
        }

        if (bfsOrder.size() != gates.size()) {
            std::cerr << "錯誤：電路中存在迴圈（BFS 偵測）！" << std::endl;
            return false;
        }
        if (trace) {
            std::cout << "[BFS Trace] 排序完成。執行順序：";
            for (auto* g : bfsOrder) std::cout << g->name << " ";
            std::cout << "\n";
        }
        return true;
    }

    // ── DFS 拓撲排序（反向後序遍歷）O(V+E) ──────────────────
    bool topologicalSortDFS(bool trace = false, int max_steps = -1) {
        dfsOrder.clear();

        std::unordered_map<std::string, Gate*> signalSource;
        for (auto& g : gates) signalSource[g->outputName] = g.get();

        std::unordered_map<Gate*, std::vector<Gate*>> adj;
        for (auto& g : gates)
            for (const auto& inp : g->inputNames)
                if (signalSource.count(inp))
                    adj[signalSource[inp]].push_back(g.get());

        // 三態：0=WHITE（未訪問）/ 1=GRAY（訪問中）/ 2=BLACK（已完成）
        std::unordered_map<Gate*, int> state;
        for (auto& g : gates) state[g.get()] = 0;

        std::stack<Gate*> resultStack;
        bool hasCycle = false;
        int depth = 0;
        int dfs_visit_count = 0;   // 追蹤已印出的節點數，供 max_steps 使用
        bool dfs_truncated = false;

        if (trace) {
            std::cout << "[DFS Trace] 從所有 WHITE 節點開始遞迴 DFS...\n"
                      << "  顏色說明：WHITE=未訪問 / GRAY=訪問中（在當前遞迴堆疊上）/ BLACK=已完成\n"
                      << "  → 表示進入（WHITE→GRAY），← 表示完成（GRAY→BLACK）\n\n";
        }

        std::function<void(Gate*)> dfs = [&](Gate* node) {
            if (hasCycle) return;
            state[node] = 1; // GRAY
            dfs_visit_count++;

            // max_steps 限制：進入第 max_steps+1 個節點時印出截斷訊息
            if (trace && max_steps >= 0 && dfs_visit_count == max_steps + 1 && !dfs_truncated) {
                std::string indent(depth * 2, ' ');
                std::cout << indent << "... （已達 --max-steps " << max_steps
                          << "，後續節點省略，排序仍繼續完成）\n\n";
                dfs_truncated = true;
            }

            if (trace && !dfs_truncated) {
                std::string indent(depth * 2, ' ');
                std::cout << indent << "→ Entering " << node->name
                          << " (" << node->type << ") [WHITE→GRAY, depth=" << depth << "]\n";
            }
            depth++;

            for (Gate* nxt : adj[node]) {
                if (state[nxt] == 1) {
                    // Back edge：發現迴圈
                    hasCycle = true;
                    if (trace && !dfs_truncated) {
                        std::string indent(depth * 2, ' ');
                        std::cout << indent << "!! Back edge: " << node->name
                                  << " → " << nxt->name
                                  << " （" << nxt->name << " 目前為 GRAY）[CYCLE DETECTED]\n";
                    }
                    return;
                }
                if (state[nxt] == 0) dfs(nxt);
                if (hasCycle) return;
            }

            depth--;
            state[node] = 2; // BLACK
            resultStack.push(node);

            if (trace && !dfs_truncated) {
                std::string indent(depth * 2, ' ');
                std::cout << indent << "← Finishing " << node->name
                          << " (" << node->type << ") [GRAY→BLACK, depth=" << depth << "]\n";
            }
        };

        for (auto& g : gates)
            if (state[g.get()] == 0) dfs(g.get());

        if (hasCycle) {
            std::cerr << "錯誤：電路中存在迴圈（DFS 偵測）！" << std::endl;
            return false;
        }

        while (!resultStack.empty()) {
            dfsOrder.push_back(resultStack.top());
            resultStack.pop();
        }

        if (trace) {
            std::cout << "\n[DFS Trace] 排序完成。執行順序：";
            for (auto* g : dfsOrder) std::cout << g->name << " ";
            std::cout << "\n";
        }
        return true;
    }

    // ── DFS 拓撲排序（迭代版，顯式 stack）O(V+E) ─────────────
    // 用 stack<pair<Gate*, int>> 模擬遞迴：int 為下一個待訪 child 的 index
    // 三色標記：0=WHITE / 1=GRAY / 2=BLACK，可偵測迴圈
    bool topologicalSortDFSIterative() {
        dfsIterOrder.clear();

        std::unordered_map<std::string, Gate*> signalSource;
        for (auto& g : gates) signalSource[g->outputName] = g.get();

        // 建立鄰接表（同 DFS 遞迴版）
        std::unordered_map<Gate*, std::vector<Gate*>> adj;
        for (auto& g : gates)
            for (const auto& inp : g->inputNames)
                if (signalSource.count(inp))
                    adj[signalSource[inp]].push_back(g.get());

        std::unordered_map<Gate*, int> state;
        for (auto& g : gates) state[g.get()] = 0; // WHITE

        std::stack<Gate*> resultStack;
        bool hasCycle = false;

        // worklist: pair<Gate*, child_index>
        std::stack<std::pair<Gate*, int>> worklist;

        for (auto& g : gates) {
            if (state[g.get()] != 0) continue; // 已訪問過

            state[g.get()] = 1; // GRAY
            worklist.push({g.get(), 0});

            while (!worklist.empty()) {
                auto& [node, idx] = worklist.top();
                auto& children    = adj[node];

                if (idx < (int)children.size()) {
                    Gate* child = children[idx++];
                    if (state[child] == 1) { hasCycle = true; break; }
                    if (state[child] == 0) {
                        state[child] = 1; // GRAY
                        worklist.push({child, 0});
                    }
                } else {
                    // 所有 child 都處理完：BLACK，寫入結果
                    state[node] = 2; // BLACK
                    resultStack.push(node);
                    worklist.pop();
                }
            }
            if (hasCycle) break;
        }

        if (hasCycle) {
            std::cerr << "錯誤：電路中存在迴圈（DFS iterative 偵測）！" << std::endl;
            return false;
        }

        while (!resultStack.empty()) {
            dfsIterOrder.push_back(resultStack.top());
            resultStack.pop();
        }
        return true;
    }

    const std::vector<Gate*>& getDFSIterOrder() const { return dfsIterOrder; }

    // ── 預設使用 BFS ─────────────────────────────────────────
    bool topologicalSort() {
        bool result = topologicalSortBFS();
        executionOrder = bfsOrder;
        return result;
    }

    // ── 模擬 ─────────────────────────────────────────────────
    void simulate(const std::vector<bool>& inputValues, const std::vector<Gate*>& order) {
        for (size_t i = 0; i < inputSignals.size(); i++)
            signals[inputSignals[i]] = inputValues[i];
        for (Gate* gate : order) {
            std::vector<bool> ins;
            for (const auto& n : gate->inputNames) ins.push_back(signals[n]);
            signals[gate->outputName] = gate->compute(ins);
        }
    }

    void simulate(const std::vector<bool>& inputValues) {
        simulate(inputValues, executionOrder);
    }

    bool getSignal(const std::string& name) const {
        auto it = signals.find(name);
        return it != signals.end() ? it->second : false;
    }

    const std::vector<Gate*>& getBfsOrder() const { return bfsOrder; }
    const std::vector<Gate*>& getDfsOrder() const { return dfsOrder; }

    // ── 真值表 ───────────────────────────────────────────────
    void generateTruthTable() {
        int n = (int)inputSignals.size();
        int total = 1 << n;

        for (const auto& name : inputSignals)  std::cout << name << "\t";
        std::cout << "| ";
        for (const auto& name : outputSignals) std::cout << name << "\t";
        std::cout << "\n";
        for (int i = 0; i < (int)(inputSignals.size() + outputSignals.size()); i++)
            std::cout << "--------";
        std::cout << "\n";

        for (int combo = 0; combo < total; combo++) {
            std::vector<bool> inputValues(n);
            for (int i = 0; i < n; i++) inputValues[n - 1 - i] = (combo >> i) & 1;
            simulate(inputValues);
            for (bool val : inputValues) std::cout << val << "\t";
            std::cout << "| ";
            for (const auto& name : outputSignals) std::cout << signals[name] << "\t";
            std::cout << "\n";
        }
    }

    // ── 效能比較（BFS vs DFS）────────────────────────────────
    void performanceComparison(int iterations = 10000) {
        std::cout << "\n╔══════════════════════════════════════════════════════╗\n";
        std::cout << "║  效能分析：BFS vs DFS 拓撲排序                     ║\n";
        std::cout << "╚══════════════════════════════════════════════════════╝\n";
        std::cout << "\n電路規模：" << gates.size() << " 個邏輯閘、"
                  << inputSignals.size() << " 個輸入\n";
        std::cout << "測試次數：" << iterations << " 次\n\n";

        auto bfsStart = std::chrono::high_resolution_clock::now();
        for (int i = 0; i < iterations; i++) topologicalSortBFS();
        auto bfsEnd   = std::chrono::high_resolution_clock::now();
        long long bfsDuration = std::chrono::duration_cast<std::chrono::microseconds>(bfsEnd - bfsStart).count();

        auto dfsStart = std::chrono::high_resolution_clock::now();
        for (int i = 0; i < iterations; i++) topologicalSortDFS();
        auto dfsEnd   = std::chrono::high_resolution_clock::now();
        long long dfsDuration = std::chrono::duration_cast<std::chrono::microseconds>(dfsEnd - dfsStart).count();

        int n = (int)inputSignals.size();
        int totalCombos = 1 << n;

        auto bfsSimStart = std::chrono::high_resolution_clock::now();
        for (int iter = 0; iter < iterations / 10; iter++) {
            topologicalSortBFS();
            for (int combo = 0; combo < totalCombos; combo++) {
                std::vector<bool> vals(n);
                for (int i = 0; i < n; i++) vals[n - 1 - i] = (combo >> i) & 1;
                simulate(vals, bfsOrder);
            }
        }
        auto bfsSimEnd = std::chrono::high_resolution_clock::now();
        long long bfsSimDuration = std::chrono::duration_cast<std::chrono::microseconds>(bfsSimEnd - bfsSimStart).count();

        auto dfsSimStart = std::chrono::high_resolution_clock::now();
        for (int iter = 0; iter < iterations / 10; iter++) {
            topologicalSortDFS();
            for (int combo = 0; combo < totalCombos; combo++) {
                std::vector<bool> vals(n);
                for (int i = 0; i < n; i++) vals[n - 1 - i] = (combo >> i) & 1;
                simulate(vals, dfsOrder);
            }
        }
        auto dfsSimEnd = std::chrono::high_resolution_clock::now();
        long long dfsSimDuration = std::chrono::duration_cast<std::chrono::microseconds>(dfsSimEnd - dfsSimStart).count();

        bool resultsMatch = true;
        topologicalSortBFS(); topologicalSortDFS();
        for (int combo = 0; combo < totalCombos; combo++) {
            std::vector<bool> vals(n);
            for (int i = 0; i < n; i++) vals[n - 1 - i] = (combo >> i) & 1;
            simulate(vals, bfsOrder);
            std::map<std::string, bool> bfsResults;
            for (const auto& out : outputSignals) bfsResults[out] = signals[out];
            simulate(vals, dfsOrder);
            for (const auto& out : outputSignals) {
                if (signals[out] != bfsResults[out]) { resultsMatch = false; break; }
            }
        }

        std::cout << std::fixed << std::setprecision(2);
        std::cout << "┌────────────────────────┬──────────────┬──────────────┐\n";
        std::cout << "│ 測試項目               │ BFS (Kahn's) │ DFS (反後序) │\n";
        std::cout << "├────────────────────────┼──────────────┼──────────────┤\n";
        std::cout << "│ 排序時間 (" << iterations << " 次)  │ "
                  << std::setw(8) << bfsDuration << " μs │ "
                  << std::setw(8) << dfsDuration << " μs │\n";
        std::cout << "├────────────────────────┼──────────────┼──────────────┤\n";
        std::cout << "│ 平均每次排序           │ "
                  << std::setw(8) << std::setprecision(3) << (double)bfsDuration / iterations << " μs │ "
                  << std::setw(8) << (double)dfsDuration / iterations << " μs │\n";
        std::cout << "├────────────────────────┼──────────────┼──────────────┤\n";
        std::cout << "│ 完整模擬 (" << iterations/10 << " 輪) │ "
                  << std::setw(8) << bfsSimDuration << " μs │ "
                  << std::setw(8) << dfsSimDuration << " μs │\n";
        std::cout << "├────────────────────────┼──────────────┼──────────────┤\n";
        std::cout << "│ 結果一致性             │ "
                  << (resultsMatch ? "✅ 一致     " : "❌ 不一致   ") << " │ "
                  << (resultsMatch ? "✅ 一致     " : "❌ 不一致   ") << " │\n";
        std::cout << "└────────────────────────┴──────────────┴──────────────┘\n";

        std::cout << "\n拓撲排序順序比較：\n  BFS: ";
        for (auto* g : bfsOrder) std::cout << g->name << " ";
        std::cout << "\n  DFS: ";
        for (auto* g : dfsOrder) std::cout << g->name << " ";
        std::cout << "\n";
    }

    // ── 從檔案載入 ───────────────────────────────────────────
    bool loadFromFile(const std::string& filename) {
        std::ifstream file(filename);
        if (!file.is_open()) {
            std::cerr << "錯誤：無法開啟檔案 " << filename << std::endl;
            return false;
        }
        std::string line;
        while (std::getline(file, line)) {
            if (line.empty() || line[0] == '#') continue;
            std::istringstream iss(line);
            std::string keyword;
            iss >> keyword;
            if (keyword == "INPUT") {
                std::string name;
                while (iss >> name) addInput(name);
            } else if (keyword == "OUTPUT") {
                std::string name;
                while (iss >> name) addOutput(name);
            } else if (keyword == "GATE") {
                std::string type, gateName;
                iss >> type >> gateName;
                std::vector<std::string> inputs;
                std::string token, output;
                while (iss >> token) {
                    if (token == "->") { iss >> output; break; }
                    inputs.push_back(token);
                }
                addGate(type, gateName, inputs, output);
            }
        }
        return true;
    }

    // ── 取得所有閘的裸指標（供 critical_path.h 使用，不轉移所有權）──
    std::vector<Gate*> getAllGates() const {
        std::vector<Gate*> result;
        result.reserve(gates.size());
        for (const auto& g : gates) result.push_back(g.get());
        return result;
    }

    // ── 印出電路資訊 ─────────────────────────────────────────
    void printInfo() {
        std::cout << "=== 電路資訊 ===\n";
        std::cout << "輸入訊號：";
        for (const auto& s : inputSignals) std::cout << s << " ";
        std::cout << "\n輸出訊號：";
        for (const auto& s : outputSignals) std::cout << s << " ";
        std::cout << "\n邏輯閘數量：" << gates.size() << "\n\n拓撲排序後的執行順序：\n";
        for (Gate* g : executionOrder) {
            std::cout << "  " << g->name << " (" << g->type << "): ";
            for (const auto& inp : g->inputNames) std::cout << inp << " ";
            std::cout << "-> " << g->outputName << "\n";
        }
        std::cout << "\n";
    }
};
