// test_edge_cases.cpp
// 邊界條件測試：
//   1. 空電路
//   2. 單一 NOT 閘
//   3. 故意造迴圈（BFS 和 DFS 都必須偵測到）
//   4. 超深電路（100 個 NOT 串聯）—— BFS 應能正常完成
//
// 編譯：g++ -std=c++17 -O2 -o test_edge_cases test_edge_cases.cpp
// 執行：./test_edge_cases

#include "../src/circuit.h"
#include <iostream>
#include <sstream>

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

// ── 抑制 stderr 的小工具（迴圈測試時避免錯誤訊息干擾輸出）────
struct SuppressStderr {
    std::streambuf* orig;
    std::stringstream   buf;
    SuppressStderr()  { orig = std::cerr.rdbuf(buf.rdbuf()); }
    ~SuppressStderr() { std::cerr.rdbuf(orig); }
};

// ============================================================
// 測試 1：空電路
// ── 0 個 input、0 個 gate ──────────────────────────────────
// ============================================================
bool test_empty_circuit() {
    Circuit c;
    // 拓撲排序：gates 為空 → bfsOrder.size()==0 == gates.size()==0 → true
    EXPECT(c.topologicalSort() == true, "empty circuit topologicalSort should return true");
    EXPECT(c.getGateCount()  == 0,      "empty circuit gate count should be 0");
    EXPECT(c.getInputCount() == 0,      "empty circuit input count should be 0");

    // simulate 空輸入不應崩潰
    c.simulate({});
    return true;
}

// ============================================================
// 測試 2：單一 NOT 閘
// ── NOT(0)=1, NOT(1)=0 ────────────────────────────────────
// ============================================================
bool test_single_not_gate() {
    Circuit c;
    c.addInput("X");
    c.addOutput("Y");
    c.addGate("NOT", "NOT_1", {"X"}, "Y");

    EXPECT(c.topologicalSort() == true, "single NOT topologicalSort failed");
    EXPECT(c.getGateCount() == 1,       "single NOT gate count should be 1");

    c.simulate({false});
    EXPECT(c.getSignal("Y") == true,  "NOT(0) should be 1");

    c.simulate({true});
    EXPECT(c.getSignal("Y") == false, "NOT(1) should be 0");

    return true;
}

// ============================================================
// 測試 3：迴圈偵測
// ── wire_A → NOT_B → wire_B → NOT_A → wire_A（cycle！）──
// ============================================================
bool test_cycle_detection_bfs() {
    Circuit c;
    // 不宣告任何 primary input；兩個閘互相依賴
    c.addGate("NOT", "NOT_A", {"wire_B"}, "wire_A");
    c.addGate("NOT", "NOT_B", {"wire_A"}, "wire_B");

    SuppressStderr _;   // 吃掉「電路中存在迴圈」的錯誤訊息
    bool result = c.topologicalSortBFS();
    EXPECT(result == false, "BFS should detect cycle and return false");
    return true;
}

bool test_cycle_detection_dfs() {
    Circuit c;
    c.addGate("NOT", "NOT_A", {"wire_B"}, "wire_A");
    c.addGate("NOT", "NOT_B", {"wire_A"}, "wire_B");

    SuppressStderr _;
    bool result = c.topologicalSortDFS();
    EXPECT(result == false, "DFS should detect cycle and return false");
    return true;
}

// 三閘環：A→B→C→A
bool test_cycle_3gate_ring() {
    Circuit c;
    c.addGate("AND", "G_AB", {"wire_C", "wire_C"}, "wire_A");  // C → A
    c.addGate("AND", "G_BC", {"wire_A", "wire_A"}, "wire_B");  // A → B
    c.addGate("AND", "G_CA", {"wire_B", "wire_B"}, "wire_C");  // B → C

    SuppressStderr _;
    EXPECT(c.topologicalSortBFS() == false, "BFS should detect 3-gate ring");
    EXPECT(c.topologicalSortDFS() == false, "DFS should detect 3-gate ring");
    return true;
}

// ============================================================
// 測試 4：超深電路 —— 100 個 NOT 串聯
// ── BFS O(V+E)：安全，遞迴深度為 0（用 Queue）
// ── DFS 遞迴深度 = 電路深度 = 100，不會 stack overflow；
//    但若深度達 ~10000+ 且平台 stack 較小則有理論風險。
// ============================================================
bool test_deep_chain_bfs() {
    const int DEPTH = 100;
    Circuit c;
    c.addInput("X0");
    c.addOutput("X" + std::to_string(DEPTH));

    for (int i = 0; i < DEPTH; i++) {
        std::string in  = "X" + std::to_string(i);
        std::string out = "X" + std::to_string(i + 1);
        c.addGate("NOT", "NOT_" + std::to_string(i), {in}, out);
    }

    EXPECT(c.topologicalSortBFS() == true, "BFS on 100-NOT chain should succeed");

    // 偶數個 NOT 串聯：NOT^100(x) == x（NOT 抵消）
    // 明確傳入 bfsOrder，因為直接呼叫 topologicalSortBFS() 不會更新 executionOrder
    c.simulate({false}, c.getBfsOrder());
    EXPECT(c.getSignal("X" + std::to_string(DEPTH)) == false,
           "100 NOTs in chain: even depth → output == input");

    c.simulate({true}, c.getBfsOrder());
    EXPECT(c.getSignal("X" + std::to_string(DEPTH)) == true,
           "100 NOTs in chain: even depth → output == input");

    return true;
}

bool test_deep_chain_dfs() {
    const int DEPTH = 100;
    Circuit c;
    c.addInput("X0");
    c.addOutput("X" + std::to_string(DEPTH));

    for (int i = 0; i < DEPTH; i++) {
        std::string in  = "X" + std::to_string(i);
        std::string out = "X" + std::to_string(i + 1);
        c.addGate("NOT", "NOT_" + std::to_string(i), {in}, out);
    }

    // depth=100 遠低於典型 stack frame 上限（~8000 層）；此測試驗證 DFS 正確性。
    // 注意：若 DEPTH 增至 ~10000+ 可能在某些平台引發 stack overflow。
    EXPECT(c.topologicalSortDFS() == true, "DFS on 100-NOT chain should succeed");

    c.simulate({false}, c.getDfsOrder());
    EXPECT(c.getSignal("X" + std::to_string(DEPTH)) == false,
           "DFS 100 NOTs: even depth → output == input");

    return true;
}

// ── 主程式 ───────────────────────────────────────────────────
int main() {
    std::cout << "=== test_edge_cases ===\n";
    run("empty_circuit",         test_empty_circuit);
    run("single_not_gate",       test_single_not_gate);
    run("cycle_detection_bfs",   test_cycle_detection_bfs);
    run("cycle_detection_dfs",   test_cycle_detection_dfs);
    run("cycle_3gate_ring",      test_cycle_3gate_ring);
    run("deep_chain_100_bfs",    test_deep_chain_bfs);
    run("deep_chain_100_dfs",    test_deep_chain_dfs);
    std::cout << "\n" << g_passed << " passed, " << g_failed << " failed.\n";
    return g_failed > 0 ? 1 : 0;
}
