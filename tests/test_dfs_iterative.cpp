// test_dfs_iterative.cpp
// 驗證 topologicalSortDFSIterative() 的正確性
//
// 測試 1：半加器 — 迭代版 DFS 排序後模擬結果與預期一致（4 組輸入）
// 測試 2：全加器 — 迭代版 DFS 排序後模擬結果與遞迴版完全一致（8 組輸入）
// 測試 3：線性鏈（5 閘）— 排序結果滿足拓撲序約束
// 測試 4：迴圈偵測 — 有環電路應回傳 false
//
// 編譯：g++ -std=c++17 -O2 -o test_dfs_iterative test_dfs_iterative.cpp
// 執行：./test_dfs_iterative

#include "../src/circuit.h"
#include <cassert>
#include <iostream>

// ── 簡易測試框架（與其他 test 相同格式）────────────────────
static int g_passed = 0;
static int g_failed = 0;

#define EXPECT(cond, msg)                                               \
    do {                                                                \
        if (!(cond)) {                                                  \
            std::cerr << "    FAIL at " << __FILE__ << ":" << __LINE__ \
                      << "  →  " << (msg) << "\n";                     \
            return false;                                               \
        }                                                               \
    } while (0)

static void run(const char* name, bool (*fn)()) {
    std::cout << "  " << name << " ... ";
    std::cout.flush();
    if (fn()) { std::cout << "PASS\n"; g_passed++; }
    else       { std::cout << "FAIL\n"; g_failed++; }
}

// ── 輔助：驗證一個排序結果是否滿足拓撲序約束 ─────────────
// 對排序中每個 gate，其所有輸入 gate 都應出現在它之前
static bool isValidTopoOrder(const std::vector<Gate*>& order,
                              const std::unordered_map<std::string, int>& posMap) {
    for (int i = 0; i < (int)order.size(); i++) {
        Gate* g = order[i];
        for (const auto& inp : g->inputNames) {
            // 若輸入是另一個 gate 的輸出，該 gate 必須在 i 之前
            if (posMap.count(inp)) {
                EXPECT(posMap.at(inp) < i,
                       std::string("拓撲序違反：") + inp + " 應在 " + g->name + " 之前");
            }
        }
    }
    return true;
}

// ══════════════════════════════════════════════════════════════
// 測試 1：半加器 — 迭代版排序後模擬結果正確
// ══════════════════════════════════════════════════════════════
static bool test_half_adder_iter() {
    Circuit c;
    c.addInput("A"); c.addInput("B");
    c.addOutput("S"); c.addOutput("C");
    c.addGate("XOR", "XOR_1", {"A", "B"}, "S");
    c.addGate("AND", "AND_1", {"A", "B"}, "C");

    EXPECT(c.topologicalSortDFSIterative(), "迭代版 DFS 應成功（無環）");

    const auto& order = c.getDFSIterOrder();
    EXPECT(!order.empty(), "排序結果不應為空");

    // 驗證拓撲序
    std::unordered_map<std::string, int> posMap; // outputName → position
    for (int i = 0; i < (int)order.size(); i++)
        posMap[order[i]->outputName] = i;
    if (!isValidTopoOrder(order, posMap)) return false;

    // 驗證模擬結果：(A,B) → (S,C)
    // (0,0)→(0,0), (0,1)→(1,0), (1,0)→(1,0), (1,1)→(0,1)
    std::vector<std::pair<std::pair<bool,bool>, std::pair<bool,bool>>> cases = {
        {{false,false},{false,false}},
        {{false,true}, {true, false}},
        {{true, false},{true, false}},
        {{true, true}, {false,true}},
    };
    for (auto& [inp, exp] : cases) {
        c.simulate({inp.first, inp.second}, order);
        bool S = c.getSignal("S");
        bool C = c.getSignal("C");
        EXPECT(S == exp.first,  "半加器 S 計算錯誤");
        EXPECT(C == exp.second, "半加器 C 計算錯誤");
    }
    return true;
}

// ══════════════════════════════════════════════════════════════
// 測試 2：全加器 — 迭代版與遞迴版模擬結果完全一致
// ══════════════════════════════════════════════════════════════
static bool test_full_adder_consistency() {
    auto makeFA = []() {
        Circuit c;
        c.addInput("A"); c.addInput("B"); c.addInput("Cin");
        c.addOutput("S"); c.addOutput("Cout");
        c.addGate("XOR", "XOR_1", {"A", "B"},    "W1");
        c.addGate("XOR", "XOR_2", {"W1", "Cin"}, "S");
        c.addGate("AND", "AND_1", {"A", "B"},    "W2");
        c.addGate("AND", "AND_2", {"W1", "Cin"}, "W3");
        c.addGate("OR",  "OR_1",  {"W2", "W3"},  "Cout");
        return c;
    };

    Circuit c_iter = makeFA();
    Circuit c_rec  = makeFA();

    EXPECT(c_iter.topologicalSortDFSIterative(), "迭代版 DFS 全加器應成功");
    EXPECT(c_rec.topologicalSortDFS(),           "遞迴版 DFS 全加器應成功");

    const auto& ord_iter = c_iter.getDFSIterOrder();
    const auto& ord_rec  = c_rec.getDfsOrder();
    EXPECT(!ord_iter.empty(), "迭代版排序結果不應為空");
    EXPECT(!ord_rec.empty(),  "遞迴版排序結果不應為空");

    // 驗證迭代版拓撲序
    std::unordered_map<std::string, int> posMap;
    for (int i = 0; i < (int)ord_iter.size(); i++)
        posMap[ord_iter[i]->outputName] = i;
    if (!isValidTopoOrder(ord_iter, posMap)) return false;

    // 8 種輸入組合：模擬結果需完全一致
    for (int combo = 0; combo < 8; combo++) {
        bool A   = (combo >> 2) & 1;
        bool B   = (combo >> 1) & 1;
        bool Cin = (combo >> 0) & 1;

        c_iter.simulate({A, B, Cin}, ord_iter);
        bool S_iter = c_iter.getSignal("S");
        bool C_iter = c_iter.getSignal("Cout");

        c_rec.simulate({A, B, Cin}, ord_rec);
        bool S_rec = c_rec.getSignal("S");
        bool C_rec = c_rec.getSignal("Cout");

        EXPECT(S_iter == S_rec,  "全加器 S：迭代版與遞迴版結果不一致");
        EXPECT(C_iter == C_rec,  "全加器 Cout：迭代版與遞迴版結果不一致");
    }
    return true;
}

// ══════════════════════════════════════════════════════════════
// 測試 3：線性鏈（5 NOT 閘）— 拓撲序約束
// ══════════════════════════════════════════════════════════════
static bool test_chain_topo_order() {
    Circuit c;
    c.addInput("IN_0");
    // IN_0 → G0 → G1 → G2 → G3 → G4
    c.addGate("NOT", "G0", {"IN_0"}, "W0");
    c.addGate("NOT", "G1", {"W0"},   "W1");
    c.addGate("NOT", "G2", {"W1"},   "W2");
    c.addGate("NOT", "G3", {"W2"},   "W3");
    c.addGate("NOT", "G4", {"W3"},   "W4");
    c.addOutput("W4");

    EXPECT(c.topologicalSortDFSIterative(), "迭代版 DFS 鏈狀電路應成功");

    const auto& order = c.getDFSIterOrder();
    EXPECT(order.size() == 5, "應有 5 個閘");

    // G0 必須是第一個（唯一的入度 0 閘）
    EXPECT(order[0]->name == "G0", "G0 應排第一");
    // G4 必須是最後一個
    EXPECT(order[4]->name == "G4", "G4 應排最後");

    // 驗證完整拓撲序
    std::unordered_map<std::string, int> posMap;
    for (int i = 0; i < (int)order.size(); i++)
        posMap[order[i]->outputName] = i;
    return isValidTopoOrder(order, posMap);
}

// ══════════════════════════════════════════════════════════════
// 測試 4：迴圈偵測 — 有環電路應回傳 false
// ══════════════════════════════════════════════════════════════
static bool test_cycle_detection() {
    Circuit c;
    c.addInput("IN_0");
    // 建立環：G0 → G1 → G0（W1 為 G0 的輸入，W0 為 G1 的輸入）
    c.addGate("NOT", "G0", {"IN_0", "W1"}, "W0");
    c.addGate("NOT", "G1", {"W0"},         "W1");
    c.addOutput("W1");

    // 迭代版應偵測到迴圈並回傳 false
    // 把 cerr 暫時抑制，避免污染測試輸出
    std::streambuf* old = std::cerr.rdbuf(nullptr);
    bool result = c.topologicalSortDFSIterative();
    std::cerr.rdbuf(old);

    EXPECT(result == false, "有環電路應回傳 false");
    return true;
}

// ── main ─────────────────────────────────────────────────────
int main() {
    std::cout << "\n╔══════════════════════════════════════════════╗\n";
    std::cout << "║   DFS Iterative Topological Sort Tests      ║\n";
    std::cout << "╚══════════════════════════════════════════════╝\n\n";

    run("半加器：迭代版排序 + 模擬正確性",   test_half_adder_iter);
    run("全加器：迭代版 vs 遞迴版結果一致",  test_full_adder_consistency);
    run("線性鏈：拓撲序約束驗證",            test_chain_topo_order);
    run("迴圈偵測：有環電路回傳 false",       test_cycle_detection);

    std::cout << "\n";
    if (g_failed == 0)
        std::cout << "All " << g_passed << " tests passed.\n\n";
    else
        std::cout << g_passed << " passed, " << g_failed << " FAILED.\n\n";

    return g_failed > 0 ? 1 : 0;
}
