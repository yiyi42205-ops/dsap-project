// test_topo_consistency.cpp
// 驗證同一電路用 BFS 和 DFS 拓撲排序後，
// 所有輸入組合的模擬結果完全一致（順序可不同，答案必須相同）。
//
// 測試電路：半加器、全加器、2-to-1 MUX、4-bit 加法器
//
// 編譯：g++ -std=c++17 -O2 -o test_topo_consistency test_topo_consistency.cpp
// 執行：./test_topo_consistency

#include "../src/circuit.h"
#include <iostream>
#include <map>

// ── 簡易測試框架 ─────────────────────────────────────────────
static int g_passed = 0;
static int g_failed = 0;

#define EXPECT(cond, msg)                                              \
    do {                                                               \
        if (!(cond)) {                                                 \
            std::cerr << "    FAIL at " << __FILE__ << ":" << __LINE__ \
                      << "  →  " << (msg) << "\n";                    \
            return false;                                              \
        }                                                              \
    } while (0)

static void run(const char* name, bool (*fn)()) {
    std::cout << "  " << name << " ... ";
    std::cout.flush();
    if (fn()) { std::cout << "PASS\n"; g_passed++; }
    else       { std::cout << "FAIL\n"; g_failed++; }
}

// ── 核心驗證函式 ─────────────────────────────────────────────
// 對電路所有 2^n 種輸入組合，比較 BFS 和 DFS 排序後的模擬結果是否相同
bool checkConsistency(Circuit& c, const std::string& label) {
    EXPECT(c.topologicalSortBFS(), (label + ": BFS topo sort failed").c_str());
    EXPECT(c.topologicalSortDFS(), (label + ": DFS topo sort failed").c_str());

    // 兩種排序的結果不必相同，但都必須是合法的拓撲排序
    EXPECT(c.getBfsOrder().size() == (size_t)c.getGateCount(),
           (label + ": BFS order size mismatch").c_str());
    EXPECT(c.getDfsOrder().size() == (size_t)c.getGateCount(),
           (label + ": DFS order size mismatch").c_str());

    int n = c.getInputCount();
    int total = 1 << n;
    const auto& outputs = c.getOutputs();

    for (int combo = 0; combo < total; combo++) {
        // 產生輸入向量
        std::vector<bool> vals(n);
        for (int i = 0; i < n; i++) vals[n - 1 - i] = (combo >> i) & 1;

        // BFS 模擬
        c.simulate(vals, c.getBfsOrder());
        std::map<std::string, bool> bfsResult;
        for (const auto& sig : outputs) bfsResult[sig] = c.getSignal(sig);

        // DFS 模擬
        c.simulate(vals, c.getDfsOrder());
        for (const auto& sig : outputs) {
            bool dfsVal = c.getSignal(sig);
            EXPECT(dfsVal == bfsResult[sig],
                   (label + ": output " + sig + " differs between BFS and DFS").c_str());
        }
    }
    return true;
}

// ── 各電路的測試函式 ─────────────────────────────────────────
bool test_consistency_half_adder() {
    Circuit c;
    c.addInput("A"); c.addInput("B");
    c.addOutput("S"); c.addOutput("C");
    c.addGate("XOR", "XOR_1", {"A", "B"}, "S");
    c.addGate("AND", "AND_1", {"A", "B"}, "C");
    return checkConsistency(c, "HalfAdder");
}

bool test_consistency_full_adder() {
    Circuit c;
    c.addInput("A"); c.addInput("B"); c.addInput("Cin");
    c.addOutput("S"); c.addOutput("Cout");
    c.addGate("XOR", "XOR_1", {"A", "B"},    "W1");
    c.addGate("XOR", "XOR_2", {"W1", "Cin"}, "S");
    c.addGate("AND", "AND_1", {"A", "B"},    "W2");
    c.addGate("AND", "AND_2", {"W1", "Cin"}, "W3");
    c.addGate("OR",  "OR_1",  {"W2", "W3"},  "Cout");
    return checkConsistency(c, "FullAdder");
}

bool test_consistency_mux() {
    Circuit c;
    c.addInput("A"); c.addInput("B"); c.addInput("Sel");
    c.addOutput("Y");
    c.addGate("NOT", "NOT_1", {"Sel"},      "NotSel");
    c.addGate("AND", "AND_1", {"NotSel","A"}, "W1");
    c.addGate("AND", "AND_2", {"Sel","B"},    "W2");
    c.addGate("OR",  "OR_1",  {"W1","W2"},    "Y");
    return checkConsistency(c, "MUX2to1");
}

bool test_consistency_4bit_adder() {
    Circuit c;
    c.addInput("A0"); c.addInput("A1"); c.addInput("A2"); c.addInput("A3");
    c.addInput("B0"); c.addInput("B1"); c.addInput("B2"); c.addInput("B3");
    c.addInput("Cin");
    c.addOutput("S0"); c.addOutput("S1"); c.addOutput("S2"); c.addOutput("S3");
    c.addOutput("Cout");

    c.addGate("XOR","XOR_0a",{"A0","B0"},"T0"); c.addGate("XOR","XOR_0b",{"T0","Cin"},"S0");
    c.addGate("AND","AND_0a",{"A0","B0"},"G0"); c.addGate("AND","AND_0b",{"T0","Cin"},"P0");
    c.addGate("OR", "OR_0",  {"G0","P0"},"C0");

    c.addGate("XOR","XOR_1a",{"A1","B1"},"T1"); c.addGate("XOR","XOR_1b",{"T1","C0"},"S1");
    c.addGate("AND","AND_1a",{"A1","B1"},"G1"); c.addGate("AND","AND_1b",{"T1","C0"},"P1");
    c.addGate("OR", "OR_1",  {"G1","P1"},"C1");

    c.addGate("XOR","XOR_2a",{"A2","B2"},"T2"); c.addGate("XOR","XOR_2b",{"T2","C1"},"S2");
    c.addGate("AND","AND_2a",{"A2","B2"},"G2"); c.addGate("AND","AND_2b",{"T2","C1"},"P2");
    c.addGate("OR", "OR_2",  {"G2","P2"},"C2");

    c.addGate("XOR","XOR_3a",{"A3","B3"},"T3"); c.addGate("XOR","XOR_3b",{"T3","C2"},"S3");
    c.addGate("AND","AND_3a",{"A3","B3"},"G3"); c.addGate("AND","AND_3b",{"T3","C2"},"P3");
    c.addGate("OR", "OR_3",  {"G3","P3"},"Cout");

    // 4-bit adder 有 9 個輸入 → 512 種組合，全部驗證
    return checkConsistency(c, "4BitAdder(512 combos)");
}

// ── BFS 和 DFS 順序本身不必相同（驗證這一點）────────────────
bool test_order_may_differ() {
    // 全加器的 BFS 和 DFS 順序幾乎必定不同
    Circuit c;
    c.addInput("A"); c.addInput("B"); c.addInput("Cin");
    c.addOutput("S"); c.addOutput("Cout");
    c.addGate("XOR", "XOR_1", {"A","B"},    "W1");
    c.addGate("XOR", "XOR_2", {"W1","Cin"}, "S");
    c.addGate("AND", "AND_1", {"A","B"},    "W2");
    c.addGate("AND", "AND_2", {"W1","Cin"}, "W3");
    c.addGate("OR",  "OR_1",  {"W2","W3"},  "Cout");
    c.topologicalSortBFS();
    c.topologicalSortDFS();

    // 只要兩者都是合法拓撲排序就算通過（順序不必相同）
    const auto& bfs = c.getBfsOrder();
    const auto& dfs = c.getDfsOrder();

    EXPECT(bfs.size() == 5, "BFS order should have 5 gates");
    EXPECT(dfs.size() == 5, "DFS order should have 5 gates");

    // 驗證兩者都包含相同的閘集合（只是順序可能不同）
    std::map<std::string, int> bfsCnt, dfsCnt;
    for (auto* g : bfs) bfsCnt[g->name]++;
    for (auto* g : dfs) dfsCnt[g->name]++;
    EXPECT(bfsCnt == dfsCnt, "BFS and DFS should contain the same set of gates");

    return true;
}

// ── 主程式 ───────────────────────────────────────────────────
int main() {
    std::cout << "=== test_topo_consistency ===\n";
    run("half_adder_4_combos",       test_consistency_half_adder);
    run("full_adder_8_combos",       test_consistency_full_adder);
    run("mux_8_combos",              test_consistency_mux);
    run("4bit_adder_512_combos",     test_consistency_4bit_adder);
    run("order_may_differ_but_same_gates", test_order_may_differ);
    std::cout << "\n" << g_passed << " passed, " << g_failed << " failed.\n";
    return g_failed > 0 ? 1 : 0;
}
