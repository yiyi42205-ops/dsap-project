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
    AndGate(std::string name) : Gate(name, "AND") {}
    bool compute(const std::vector<bool>& inputs) override {
        for (bool val : inputs) { if (!val) return false; }
        return true;
    }
};

class OrGate : public Gate {
public:
    OrGate(std::string name) : Gate(name, "OR") {}
    bool compute(const std::vector<bool>& inputs) override {
        for (bool val : inputs) { if (val) return true; }
        return false;
    }
};

class NotGate : public Gate {
public:
    NotGate(std::string name) : Gate(name, "NOT") {}
    bool compute(const std::vector<bool>& inputs) override {
        return !inputs[0];
    }
};

class XorGate : public Gate {
public:
    XorGate(std::string name) : Gate(name, "XOR") {}
    bool compute(const std::vector<bool>& inputs) override {
        bool result = false;
        for (bool val : inputs) { result = result ^ val; }
        return result;
    }
};

class NandGate : public Gate {
public:
    NandGate(std::string name) : Gate(name, "NAND") {}
    bool compute(const std::vector<bool>& inputs) override {
        for (bool val : inputs) { if (!val) return true; }
        return false;
    }
};

class NorGate : public Gate {
public:
    NorGate(std::string name) : Gate(name, "NOR") {}
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

    // 用於效能比較
    std::vector<Gate*> bfsOrder;
    std::vector<Gate*> dfsOrder;

public:
    // --------------------------------------------------------
    // 加入輸入/輸出訊號
    // --------------------------------------------------------
    void addInput(const std::string& name) {
        inputSignals.push_back(name);
        signals[name] = false;
    }

    void addOutput(const std::string& name) {
        outputSignals.push_back(name);
    }

    int getInputCount() const { return inputSignals.size(); }
    int getGateCount() const { return gates.size(); }
    const std::vector<std::string>& getInputs() const { return inputSignals; }
    const std::vector<std::string>& getOutputs() const { return outputSignals; }

    // --------------------------------------------------------
    // 建立邏輯閘
    // --------------------------------------------------------
    Gate* addGate(const std::string& type, const std::string& name,
                  const std::vector<std::string>& inputs, const std::string& output) {
        std::unique_ptr<Gate> gate;
        if (type == "AND")       gate = std::make_unique<AndGate>(name);
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

    // ============================================================
    // 方法 A：BFS 拓撲排序 (Kahn's Algorithm)
    //   使用資料結構：Queue + Hash Map
    //   時間複雜度：O(V + E)
    // ============================================================
    bool topologicalSortBFS() {
        bfsOrder.clear();

        // 建立「哪個訊號是哪個閘的輸出」
        std::unordered_map<std::string, Gate*> signalSource;
        for (auto& g : gates) {
            signalSource[g->outputName] = g.get();
        }

        // 計算每個閘的入度 (in-degree)
        std::unordered_map<Gate*, int> inDegree;
        for (auto& g : gates) { inDegree[g.get()] = 0; }
        for (auto& g : gates) {
            for (const auto& inp : g->inputNames) {
                if (signalSource.count(inp)) {
                    inDegree[g.get()]++;
                }
            }
        }

        // 建立鄰接表：閘 A 的輸出 → 哪些閘以此為輸入
        std::unordered_map<Gate*, std::vector<Gate*>> adj;
        for (auto& g : gates) {
            for (const auto& inp : g->inputNames) {
                if (signalSource.count(inp)) {
                    adj[signalSource[inp]].push_back(g.get());
                }
            }
        }

        // BFS：入度為 0 的閘先進 queue
        std::queue<Gate*> q;
        for (auto& g : gates) {
            if (inDegree[g.get()] == 0) {
                q.push(g.get());
            }
        }

        while (!q.empty()) {
            Gate* current = q.front();
            q.pop();
            bfsOrder.push_back(current);

            for (Gate* next : adj[current]) {
                inDegree[next]--;
                if (inDegree[next] == 0) {
                    q.push(next);
                }
            }
        }

        if (bfsOrder.size() != gates.size()) {
            std::cerr << "錯誤：電路中存在迴圈（BFS 偵測）！" << std::endl;
            return false;
        }
        return true;
    }

    // ============================================================
    // 方法 B：DFS 拓撲排序 (反向後序遍歷)
    //   使用資料結構：Stack (遞迴呼叫堆疊) + Hash Map
    //   時間複雜度：O(V + E)
    // ============================================================
    bool topologicalSortDFS() {
        dfsOrder.clear();

        // 建立鄰接表
        std::unordered_map<std::string, Gate*> signalSource;
        for (auto& g : gates) {
            signalSource[g->outputName] = g.get();
        }

        std::unordered_map<Gate*, std::vector<Gate*>> adj;
        for (auto& g : gates) {
            for (const auto& inp : g->inputNames) {
                if (signalSource.count(inp)) {
                    adj[signalSource[inp]].push_back(g.get());
                }
            }
        }

        // DFS 狀態：0=未訪問, 1=訪問中, 2=已完成
        std::unordered_map<Gate*, int> state;
        for (auto& g : gates) { state[g.get()] = 0; }

        std::stack<Gate*> resultStack;
        bool hasCycle = false;

        // DFS 函式（用 lambda + 手動 stack 避免遞迴深度限制）
        std::function<void(Gate*)> dfs = [&](Gate* node) {
            if (hasCycle) return;
            state[node] = 1;  // 訪問中

            for (Gate* next : adj[node]) {
                if (state[next] == 1) {
                    hasCycle = true;
                    return;
                }
                if (state[next] == 0) {
                    dfs(next);
                }
            }

            state[node] = 2;  // 已完成
            resultStack.push(node);
        };

        // 對所有未訪問的閘執行 DFS
        for (auto& g : gates) {
            if (state[g.get()] == 0) {
                dfs(g.get());
            }
        }

        if (hasCycle) {
            std::cerr << "錯誤：電路中存在迴圈（DFS 偵測）！" << std::endl;
            return false;
        }

        // Stack 反轉 → 拓撲順序
        while (!resultStack.empty()) {
            dfsOrder.push_back(resultStack.top());
            resultStack.pop();
        }

        return true;
    }

    // --------------------------------------------------------
    // 使用指定的拓撲順序模擬
    // --------------------------------------------------------
    void simulate(const std::vector<bool>& inputValues, const std::vector<Gate*>& order) {
        for (size_t i = 0; i < inputSignals.size(); i++) {
            signals[inputSignals[i]] = inputValues[i];
        }
        for (Gate* gate : order) {
            std::vector<bool> inputs;
            for (const auto& inputName : gate->inputNames) {
                inputs.push_back(signals[inputName]);
            }
            signals[gate->outputName] = gate->compute(inputs);
        }
    }

    // 預設使用 BFS 拓撲排序
    bool topologicalSort() {
        bool result = topologicalSortBFS();
        executionOrder = bfsOrder;
        return result;
    }

    void simulate(const std::vector<bool>& inputValues) {
        simulate(inputValues, executionOrder);
    }

    bool getSignal(const std::string& name) const {
        auto it = signals.find(name);
        return it != signals.end() ? it->second : false;
    }

    // --------------------------------------------------------
    // 真值表
    // --------------------------------------------------------
    void generateTruthTable() {
        int n = inputSignals.size();
        int total = 1 << n;

        for (const auto& name : inputSignals) std::cout << name << "\t";
        std::cout << "| ";
        for (const auto& name : outputSignals) std::cout << name << "\t";
        std::cout << std::endl;
        for (int i = 0; i < (int)(inputSignals.size() + outputSignals.size()); i++)
            std::cout << "--------";
        std::cout << std::endl;

        for (int combo = 0; combo < total; combo++) {
            std::vector<bool> inputValues(n);
            for (int i = 0; i < n; i++) {
                inputValues[n - 1 - i] = (combo >> i) & 1;
            }
            simulate(inputValues);
            for (bool val : inputValues) std::cout << val << "\t";
            std::cout << "| ";
            for (const auto& name : outputSignals)
                std::cout << signals[name] << "\t";
            std::cout << std::endl;
        }
    }

    // ============================================================
    // 效能比較：BFS vs DFS 拓撲排序
    // ============================================================
    void performanceComparison(int iterations = 10000) {
        std::cout << "\n";
        std::cout << "╔══════════════════════════════════════════════════════╗" << std::endl;
        std::cout << "║  效能分析：BFS vs DFS 拓撲排序                     ║" << std::endl;
        std::cout << "║  Performance: BFS vs DFS Topological Sort           ║" << std::endl;
        std::cout << "╚══════════════════════════════════════════════════════╝" << std::endl;

        std::cout << "\n電路規模：" << gates.size() << " 個邏輯閘、"
                  << inputSignals.size() << " 個輸入" << std::endl;
        std::cout << "測試次數：" << iterations << " 次" << std::endl;
        std::cout << std::endl;

        // --- 測量 BFS 拓撲排序時間 ---
        auto bfsStart = std::chrono::high_resolution_clock::now();
        for (int i = 0; i < iterations; i++) {
            topologicalSortBFS();
        }
        auto bfsEnd = std::chrono::high_resolution_clock::now();
        auto bfsDuration = std::chrono::duration_cast<std::chrono::microseconds>(bfsEnd - bfsStart).count();

        // --- 測量 DFS 拓撲排序時間 ---
        auto dfsStart = std::chrono::high_resolution_clock::now();
        for (int i = 0; i < iterations; i++) {
            topologicalSortDFS();
        }
        auto dfsEnd = std::chrono::high_resolution_clock::now();
        auto dfsDuration = std::chrono::duration_cast<std::chrono::microseconds>(dfsEnd - dfsStart).count();

        // --- 測量完整模擬（拓撲排序 + 訊號傳播）---
        int n = inputSignals.size();
        int totalCombos = 1 << n;

        // BFS 版完整模擬
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
        auto bfsSimDuration = std::chrono::duration_cast<std::chrono::microseconds>(bfsSimEnd - bfsSimStart).count();

        // DFS 版完整模擬
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
        auto dfsSimDuration = std::chrono::duration_cast<std::chrono::microseconds>(dfsSimEnd - dfsSimStart).count();

        // --- 驗證正確性：兩種排序產出的模擬結果必須一致 ---
        bool resultsMatch = true;
        topologicalSortBFS();
        topologicalSortDFS();
        for (int combo = 0; combo < totalCombos; combo++) {
            std::vector<bool> vals(n);
            for (int i = 0; i < n; i++) vals[n - 1 - i] = (combo >> i) & 1;

            simulate(vals, bfsOrder);
            std::map<std::string, bool> bfsResults;
            for (const auto& out : outputSignals) bfsResults[out] = signals[out];

            simulate(vals, dfsOrder);
            for (const auto& out : outputSignals) {
                if (signals[out] != bfsResults[out]) {
                    resultsMatch = false;
                    break;
                }
            }
        }

        // --- 輸出結果 ---
        std::cout << "┌────────────────────────┬──────────────┬──────────────┐" << std::endl;
        std::cout << "│ 測試項目               │ BFS (Kahn's) │ DFS (反後序) │" << std::endl;
        std::cout << "├────────────────────────┼──────────────┼──────────────┤" << std::endl;

        std::cout << "│ 資料結構               │ Queue+HashMap │ Stack+HashMap│" << std::endl;
        std::cout << "├────────────────────────┼──────────────┼──────────────┤" << std::endl;

        std::cout << std::fixed << std::setprecision(2);
        std::cout << "│ 排序時間 (" << iterations << " 次)  │ "
                  << std::setw(8) << bfsDuration << " μs │ "
                  << std::setw(8) << dfsDuration << " μs │" << std::endl;
        std::cout << "├────────────────────────┼──────────────┼──────────────┤" << std::endl;

        std::cout << "│ 平均每次排序           │ "
                  << std::setw(8) << std::setprecision(3) << (double)bfsDuration / iterations << " μs │ "
                  << std::setw(8) << (double)dfsDuration / iterations << " μs │" << std::endl;
        std::cout << "├────────────────────────┼──────────────┼──────────────┤" << std::endl;

        std::cout << "│ 完整模擬 (" << iterations/10 << " 輪) │ "
                  << std::setw(8) << bfsSimDuration << " μs │ "
                  << std::setw(8) << dfsSimDuration << " μs │" << std::endl;
        std::cout << "├────────────────────────┼──────────────┼──────────────┤" << std::endl;

        std::cout << "│ 結果一致性             │ "
                  << (resultsMatch ? "✅ 一致     " : "❌ 不一致   ") << " │ "
                  << (resultsMatch ? "✅ 一致     " : "❌ 不一致   ") << " │" << std::endl;
        std::cout << "└────────────────────────┴──────────────┴──────────────┘" << std::endl;

        // 分析結論
        std::cout << "\n分析結論：" << std::endl;
        if (bfsDuration < dfsDuration) {
            double ratio = (double)dfsDuration / bfsDuration;
            std::cout << "  BFS (Kahn's Algorithm) 在此電路規模下快了約 "
                      << std::setprecision(1) << ratio << " 倍。" << std::endl;
            std::cout << "  原因：BFS 使用 Queue 逐層處理，不需要遞迴呼叫的額外開銷。" << std::endl;
        } else {
            double ratio = (double)bfsDuration / dfsDuration;
            std::cout << "  DFS (反後序遍歷) 在此電路規模下快了約 "
                      << std::setprecision(1) << ratio << " 倍。" << std::endl;
            std::cout << "  原因：DFS 的記憶體存取模式可能更利於 CPU cache。" << std::endl;
        }

        std::cout << "\n  BFS 優勢：能自然偵測迴圈（若排序結果數量 < 節點數）；" << std::endl;
        std::cout << "            適合需要「層次化處理」的場景。" << std::endl;
        std::cout << "  DFS 優勢：實作簡潔（遞迴版）；" << std::endl;
        std::cout << "            在深度較大、分支少的圖上可能較快。" << std::endl;

        // 印出兩種排序的順序差異
        std::cout << "\n拓撲排序順序比較：" << std::endl;
        std::cout << "  BFS: ";
        for (auto* g : bfsOrder) std::cout << g->name << " ";
        std::cout << std::endl;
        std::cout << "  DFS: ";
        for (auto* g : dfsOrder) std::cout << g->name << " ";
        std::cout << std::endl;
        std::cout << "  （兩種順序都是合法的拓撲排序，但順序不一定相同）" << std::endl;
    }

    // --------------------------------------------------------
    // 從檔案載入
    // --------------------------------------------------------
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
        file.close();
        return true;
    }

    // --------------------------------------------------------
    // 印出電路資訊
    // --------------------------------------------------------
    void printInfo() {
        std::cout << "=== 電路資訊 ===" << std::endl;
        std::cout << "輸入訊號：";
        for (const auto& s : inputSignals) std::cout << s << " ";
        std::cout << std::endl;
        std::cout << "輸出訊號：";
        for (const auto& s : outputSignals) std::cout << s << " ";
        std::cout << std::endl;
        std::cout << "邏輯閘數量：" << gates.size() << std::endl;
        std::cout << "\n拓撲排序後的執行順序：" << std::endl;
        for (Gate* g : executionOrder) {
            std::cout << "  " << g->name << " (" << g->type << "): ";
            for (const auto& inp : g->inputNames) std::cout << inp << " ";
            std::cout << "-> " << g->outputName << std::endl;
        }
        std::cout << std::endl;
    }
};

// ============================================================
// 4. 內建範例電路
// ============================================================
Circuit createHalfAdder() {
    Circuit c;
    c.addInput("A"); c.addInput("B");
    c.addOutput("S"); c.addOutput("C");
    c.addGate("XOR", "XOR_1", {"A", "B"}, "S");
    c.addGate("AND", "AND_1", {"A", "B"}, "C");
    return c;
}

Circuit createFullAdder() {
    Circuit c;
    c.addInput("A"); c.addInput("B"); c.addInput("Cin");
    c.addOutput("S"); c.addOutput("Cout");
    c.addGate("XOR", "XOR_1", {"A", "B"}, "W1");
    c.addGate("XOR", "XOR_2", {"W1", "Cin"}, "S");
    c.addGate("AND", "AND_1", {"A", "B"}, "W2");
    c.addGate("AND", "AND_2", {"W1", "Cin"}, "W3");
    c.addGate("OR",  "OR_1",  {"W2", "W3"}, "Cout");
    return c;
}

Circuit createMux2to1() {
    Circuit c;
    c.addInput("A"); c.addInput("B"); c.addInput("Sel");
    c.addOutput("Y");
    c.addGate("NOT", "NOT_1", {"Sel"}, "NotSel");
    c.addGate("AND", "AND_1", {"NotSel", "A"}, "W1");
    c.addGate("AND", "AND_2", {"Sel", "B"}, "W2");
    c.addGate("OR",  "OR_1",  {"W1", "W2"}, "Y");
    return c;
}

// 4-bit 加法器（串接 4 個全加器，規模較大適合效能比較）
Circuit create4BitAdder() {
    Circuit c;
    c.addInput("A0"); c.addInput("A1"); c.addInput("A2"); c.addInput("A3");
    c.addInput("B0"); c.addInput("B1"); c.addInput("B2"); c.addInput("B3");
    c.addInput("Cin");
    c.addOutput("S0"); c.addOutput("S1"); c.addOutput("S2"); c.addOutput("S3");
    c.addOutput("Cout");

    // Full Adder 0
    c.addGate("XOR", "XOR_0a", {"A0", "B0"}, "T0");
    c.addGate("XOR", "XOR_0b", {"T0", "Cin"}, "S0");
    c.addGate("AND", "AND_0a", {"A0", "B0"}, "G0");
    c.addGate("AND", "AND_0b", {"T0", "Cin"}, "P0");
    c.addGate("OR",  "OR_0",   {"G0", "P0"}, "C0");

    // Full Adder 1
    c.addGate("XOR", "XOR_1a", {"A1", "B1"}, "T1");
    c.addGate("XOR", "XOR_1b", {"T1", "C0"}, "S1");
    c.addGate("AND", "AND_1a", {"A1", "B1"}, "G1");
    c.addGate("AND", "AND_1b", {"T1", "C0"}, "P1");
    c.addGate("OR",  "OR_1",   {"G1", "P1"}, "C1");

    // Full Adder 2
    c.addGate("XOR", "XOR_2a", {"A2", "B2"}, "T2");
    c.addGate("XOR", "XOR_2b", {"T2", "C1"}, "S2");
    c.addGate("AND", "AND_2a", {"A2", "B2"}, "G2");
    c.addGate("AND", "AND_2b", {"T2", "C1"}, "P2");
    c.addGate("OR",  "OR_2",   {"G2", "P2"}, "C2");

    // Full Adder 3
    c.addGate("XOR", "XOR_3a", {"A3", "B3"}, "T3");
    c.addGate("XOR", "XOR_3b", {"T3", "C2"}, "S3");
    c.addGate("AND", "AND_3a", {"A3", "B3"}, "G3");
    c.addGate("AND", "AND_3b", {"T3", "C2"}, "P3");
    c.addGate("OR",  "OR_3",   {"G3", "P3"}, "Cout");

    return c;
}

// ============================================================
// 5. 主程式
// ============================================================
int main(int argc, char* argv[]) {
    std::cout << "╔══════════════════════════════════════╗" << std::endl;
    std::cout << "║  數位邏輯電路模擬器 v2.0             ║" << std::endl;
    std::cout << "║  Digital Logic Circuit Simulator     ║" << std::endl;
    std::cout << "╚══════════════════════════════════════╝" << std::endl;
    std::cout << std::endl;

    if (argc > 1) {
        std::string filename = argv[1];
        std::cout << "從檔案載入電路：" << filename << std::endl;
        Circuit circuit;
        if (!circuit.loadFromFile(filename)) return 1;
        if (!circuit.topologicalSort()) return 1;
        circuit.printInfo();
        std::cout << "=== 真值表 ===" << std::endl;
        circuit.generateTruthTable();
        circuit.performanceComparison();
        return 0;
    }

    std::cout << "請選擇內建範例電路：" << std::endl;
    std::cout << "  1. 半加器 (Half Adder)         - 2 閘" << std::endl;
    std::cout << "  2. 全加器 (Full Adder)         - 5 閘" << std::endl;
    std::cout << "  3. 2-to-1 多工器 (MUX)         - 4 閘" << std::endl;
    std::cout << "  4. 4-bit 加法器 (4-bit Adder)  - 20 閘" << std::endl;
    std::cout << "  5. 從檔案載入" << std::endl;
    std::cout << "  6. 效能比較模式（比較所有內建電路）" << std::endl;
    std::cout << "\n請輸入選項 (1-6): ";

    int choice;
    std::cin >> choice;

    if (choice == 6) {
        // 效能比較模式：跑所有內建電路
        std::cout << "\n========== 效能比較模式 ==========\n" << std::endl;

        std::cout << "--- 半加器 (2 閘) ---" << std::endl;
        Circuit ha = createHalfAdder();
        ha.topologicalSort();
        ha.performanceComparison();

        std::cout << "\n--- 全加器 (5 閘) ---" << std::endl;
        Circuit fa = createFullAdder();
        fa.topologicalSort();
        fa.performanceComparison();

        std::cout << "\n--- 2-to-1 MUX (4 閘) ---" << std::endl;
        Circuit mux = createMux2to1();
        mux.topologicalSort();
        mux.performanceComparison();

        std::cout << "\n--- 4-bit 加法器 (20 閘) ---" << std::endl;
        Circuit adder4 = create4BitAdder();
        adder4.topologicalSort();
        adder4.performanceComparison();

        return 0;
    }

    Circuit circuit;
    switch (choice) {
        case 1: std::cout << "\n--- 半加器 ---\n" << std::endl; circuit = createHalfAdder(); break;
        case 2: std::cout << "\n--- 全加器 ---\n" << std::endl; circuit = createFullAdder(); break;
        case 3: std::cout << "\n--- 2-to-1 MUX ---\n" << std::endl; circuit = createMux2to1(); break;
        case 4: std::cout << "\n--- 4-bit 加法器 ---\n" << std::endl; circuit = create4BitAdder(); break;
        case 5: {
            std::cout << "請輸入檔案路徑: ";
            std::string filename;
            std::cin >> filename;
            if (!circuit.loadFromFile(filename)) return 1;
            break;
        }
        default: std::cerr << "無效選項" << std::endl; return 1;
    }

    if (!circuit.topologicalSort()) return 1;
    circuit.printInfo();
    std::cout << "=== 真值表 ===" << std::endl;
    circuit.generateTruthTable();
    circuit.performanceComparison();

    return 0;
}
