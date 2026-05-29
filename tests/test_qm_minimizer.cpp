// test_qm_minimizer.cpp
// 驗證 Quine-McCluskey 最小化器的正確性
//
// 涵蓋：基本化簡、有無 don't care、從 Circuit 提取、邊界條件、錯誤處理
//
// 編譯：g++ -std=c++17 -O2 -o tests/test_qm_minimizer tests/test_qm_minimizer.cpp
// 執行：./tests/test_qm_minimizer

#include "../src/qm_minimizer.h"
#include <iostream>
#include <sstream>
#include <unordered_set>

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

// ── 抑制 stderr（錯誤處理測試用）────────────────────────────
struct SuppressStderr {
    std::streambuf* orig;
    std::stringstream buf;
    SuppressStderr()  { orig = std::cerr.rdbuf(buf.rdbuf()); }
    ~SuppressStderr() { std::cerr.rdbuf(orig); }
};

// ── 驗證輔助 ─────────────────────────────────────────────────

// 計算 SOP 在輸入組合 combo 下的值
// ProductTerm bit i → varNames[i] → combo bit (numVars-1-i)
static bool evalSOP(const QMResult& r, int combo) {
    for (const auto& pt : r.terms) {
        bool termTrue = true;
        for (int i = 0; i < r.numVars && termTrue; i++) {
            if (!((pt.care >> i) & 1)) continue;
            int mintermBit = r.numVars - 1 - i;
            int bitVal  = (combo >> mintermBit) & 1;
            bool positive = (pt.polarity >> i) & 1;
            if (positive != (bool)bitVal) termTrue = false;
        }
        if (termTrue) return true;
    }
    return false;
}

// 驗證：SOP 在所有 on-set minterms 輸出 1，在 off-set 輸出 0，don't cares 不管
static bool verifySOP(const QMResult& r,
                      const std::vector<int>& minterms,
                      const std::vector<int>& dontCares = {}) {
    if (r.numVars <= 0) return false;
    int total = 1 << r.numVars;
    std::unordered_set<int> mintermSet(minterms.begin(), minterms.end());
    std::unordered_set<int> dcSet    (dontCares.begin(), dontCares.end());
    for (int combo = 0; combo < total; combo++) {
        if (dcSet.count(combo)) continue;
        if (evalSOP(r, combo) != (mintermSet.count(combo) > 0))
            return false;
    }
    return true;
}

// ── 內建電路 helper ──────────────────────────────────────────
static Circuit makeHalfAdder() {
    Circuit c;
    c.addInput("A"); c.addInput("B");
    c.addOutput("S"); c.addOutput("C");
    c.addGate("XOR", "XOR_1", {"A", "B"}, "S");
    c.addGate("AND", "AND_1", {"A", "B"}, "C");
    return c;
}

static Circuit makeFullAdder() {
    Circuit c;
    c.addInput("A"); c.addInput("B"); c.addInput("Cin");
    c.addOutput("S"); c.addOutput("Cout");
    c.addGate("XOR", "XOR_1", {"A", "B"},    "W1");
    c.addGate("XOR", "XOR_2", {"W1", "Cin"}, "S");
    c.addGate("AND", "AND_1", {"A", "B"},    "W2");
    c.addGate("AND", "AND_2", {"W1", "Cin"}, "W3");
    c.addGate("OR",  "OR_1",  {"W2", "W3"},  "Cout");
    return c;
}

// ============================================================
// 基本化簡
// ============================================================

// F(A,B) = Σm(0,1,3)  →  A' + B（2 項）
// PI: A'={0,1}, B={1,3}；essential: A'(for 0), B(for 3)
bool test_two_var_basic() {
    auto r = minimize(2, {0, 1, 3}, {}, {"A", "B"});
    EXPECT(r.numVars == 2,           "numVars 應為 2");
    EXPECT(r.varNames[0] == "A",     "varNames[0] 應為 A");
    EXPECT(r.varNames[1] == "B",     "varNames[1] 應為 B");
    EXPECT((int)r.terms.size() == 2, "應有 2 個乘積項");
    EXPECT(verifySOP(r, {0, 1, 3}),  "SOP 未正確覆蓋 minterms");
    return true;
}

// F(A,B,C) = Σm(0,1,2,3,4,6)  →  A' + C'（2 項）
// A' 覆蓋 {0,1,2,3}，C' 覆蓋 {0,2,4,6}；兩者均為 essential
bool test_three_var_absorption() {
    auto r = minimize(3, {0, 1, 2, 3, 4, 6}, {}, {"A", "B", "C"});
    EXPECT(r.numVars == 3,                     "numVars 應為 3");
    EXPECT((int)r.terms.size() == 2,           "應有 2 個乘積項");
    EXPECT(verifySOP(r, {0, 1, 2, 3, 4, 6}),  "SOP 未正確覆蓋 minterms");
    return true;
}

// F(A,B,C,D) = Σm(0,1,2,5,6,7,8,9,10,14)
// PIs: B'C'、B'D'、CD'、A'C'D、A'BD、A'BC
// Essential: B'C'(for 9), CD'(for 14)；greedy: A'BD(covers {5,7})
// 最小 SOP = 3 項
bool test_four_var_classic() {
    std::vector<int> ms = {0, 1, 2, 5, 6, 7, 8, 9, 10, 14};
    auto r = minimize(4, ms, {}, {"A", "B", "C", "D"});
    EXPECT(r.numVars == 4,           "numVars 應為 4");
    EXPECT((int)r.terms.size() == 3, "最小 SOP 應有 3 個乘積項");
    EXPECT(verifySOP(r, ms),         "SOP 未正確覆蓋 minterms");
    return true;
}

// F(A,B,C) = Σm(5)  →  AB'C（1 項，不可再化簡）
// 5 = 101：A=1, B=0, C=1
bool test_single_minterm() {
    auto r = minimize(3, {5}, {}, {"A", "B", "C"});
    EXPECT((int)r.terms.size() == 1, "單一 minterm 應得 1 項");
    // 三個變數全部出現（care 的低 3 個 bit 全為 1）
    EXPECT(r.terms[0].care == 0b111u, "所有變數都應出現");
    EXPECT(verifySOP(r, {5}),         "SOP 應只覆蓋 minterm 5");
    return true;
}

// F(A,B) = Σm(1,2)  →  A'B + AB'（2 項，XOR 本身已最簡）
// 1=01 和 2=10 差兩個 bit，無法合併
bool test_already_minimal() {
    auto r = minimize(2, {1, 2}, {}, {"A", "B"});
    EXPECT((int)r.terms.size() == 2, "XOR 應有 2 個乘積項");
    EXPECT(verifySOP(r, {1, 2}),     "SOP 應正確覆蓋 {1,2}");
    return true;
}

// ============================================================
// 邊界條件
// ============================================================

// 恆 0 函數：minterms 為空
bool test_constant_zero() {
    auto r = minimize(2, {}, {}, {"A", "B"});
    EXPECT(r.numVars == 2,       "numVars 應為 2");
    EXPECT(r.terms.empty(),      "恆 0 應無乘積項");
    EXPECT(qmResultToString(r) == "0", "toString 應為 \"0\"");
    return true;
}

// 恆 1 函數：所有 2^n 個 minterm 都在 on-set
bool test_constant_one() {
    auto r = minimize(2, {0, 1, 2, 3}, {}, {"A", "B"});
    EXPECT(r.numVars == 2,              "numVars 應為 2");
    EXPECT((int)r.terms.size() == 1,   "恆 1 應有一個乘積項");
    EXPECT(r.terms[0].care == 0u,      "care=0 表示恆真乘積項");
    EXPECT(qmResultToString(r) == "1", "toString 應為 \"1\"");
    return true;
}

// 自動 varNames：未提供時應用 "x0","x1",...
bool test_default_varnames() {
    auto r = minimize(3, {0, 7});
    EXPECT(r.numVars == 3,          "numVars 應為 3");
    EXPECT(r.varNames[0] == "x0",   "varNames[0] 應為 x0");
    EXPECT(r.varNames[2] == "x2",   "varNames[2] 應為 x2");
    EXPECT(verifySOP(r, {0, 7}),    "SOP 應正確覆蓋 {0,7}");
    return true;
}

// ============================================================
// Don't Care
// ============================================================

// F(A,B,C) = Σm(1,3,7) + d(0,5)  →  C（1 項）
// DC 讓所有合併路徑匯聚為 C = --1
bool test_dc_enables_merge() {
    auto r = minimize(3, {1, 3, 7}, {0, 5}, {"A", "B", "C"});
    EXPECT((int)r.terms.size() == 1,    "DC 合併後應只剩 1 項");
    EXPECT(verifySOP(r, {1, 3, 7}, {0, 5}), "SOP 應正確覆蓋 on-set");
    // 唯一的乘積項應只含 C（最後一個變數，ProductTerm bit 2）
    EXPECT(r.terms[0].care == 0b100u,   "應只有 C 出現（bit 2）");
    EXPECT(r.terms[0].polarity == 0b100u, "C 應為正形式");
    return true;
}

// F(A,B) = Σm(1,2) + d(3)  →  A + B（2 項）
// d(3) 讓 {1,3}→B 和 {2,3}→A 成立；essential: B(for 1), A(for 2)
bool test_dc_in_coverage() {
    auto r = minimize(2, {1, 2}, {3}, {"A", "B"});
    EXPECT((int)r.terms.size() == 2,      "應有 2 個乘積項");
    EXPECT(verifySOP(r, {1, 2}, {3}),     "SOP 應正確覆蓋 {1,2}");
    return true;
}

// ============================================================
// 從 Circuit 提取
// ============================================================

// full_adder 的 Cout = majority(A,B,Cin) = Σm(3,5,6,7)
// 最小 SOP = AB + ACin + BCin（3 項）
bool test_circuit_full_adder_cout() {
    Circuit fa = makeFullAdder();
    auto r = minimizeCircuitOutput(fa, "Cout");
    EXPECT(r.numVars == 3,               "Cout 有 3 個輸入變數");
    EXPECT(r.varNames[0] == "A",         "varNames[0] 應為 A");
    EXPECT(r.varNames[1] == "B",         "varNames[1] 應為 B");
    EXPECT(r.varNames[2] == "Cin",       "varNames[2] 應為 Cin");
    EXPECT((int)r.terms.size() == 3,     "majority 函數最小 SOP 應有 3 項");
    EXPECT(verifySOP(r, {3, 5, 6, 7}),  "SOP 應正確覆蓋 Cout 的 minterm");
    return true;
}

// half_adder 的 S = XOR(A,B) = Σm(1,2)，無法化簡，應得 2 項
bool test_circuit_half_adder_s() {
    Circuit ha = makeHalfAdder();
    auto r = minimizeCircuitOutput(ha, "S");
    EXPECT(r.numVars == 2,           "S 有 2 個輸入變數");
    EXPECT(r.varNames[0] == "A",     "varNames[0] 應為 A");
    EXPECT(r.varNames[1] == "B",     "varNames[1] 應為 B");
    EXPECT((int)r.terms.size() == 2, "XOR 最小 SOP 應有 2 項");
    EXPECT(verifySOP(r, {1, 2}),     "SOP 應正確覆蓋 S 的 minterm");
    return true;
}

// half_adder 的 C = AND(A,B) = Σm(3)，單一 minterm，應得 1 項 AB
bool test_circuit_half_adder_c() {
    Circuit ha = makeHalfAdder();
    auto r = minimizeCircuitOutput(ha, "C");
    EXPECT(r.numVars == 2,           "C 有 2 個輸入變數");
    EXPECT((int)r.terms.size() == 1, "AND 最小 SOP 應有 1 項");
    // 3 = 11：A=1, B=1 → 兩變數都正形式
    EXPECT(r.terms[0].care    == 0b11u, "A 和 B 都應出現");
    EXPECT(r.terms[0].polarity == 0b11u, "A 和 B 都應為正形式");
    EXPECT(verifySOP(r, {3}),         "SOP 應正確覆蓋 C 的 minterm");
    return true;
}

// ============================================================
// 錯誤處理
// ============================================================

// minterm 超出範圍
bool test_error_out_of_range() {
    SuppressStderr _;
    auto r = minimize(2, {5}); // 5 >= 2^2=4
    EXPECT(r.numVars == 0, "超出範圍的 minterm 應回傳空 QMResult");
    return true;
}

// don't care 超出範圍
bool test_error_dc_out_of_range() {
    SuppressStderr _;
    auto r = minimize(2, {1}, {4}); // 4 >= 4
    EXPECT(r.numVars == 0, "超出範圍的 DC 應回傳空 QMResult");
    return true;
}

// minterms 和 dontCares 有重疊
bool test_error_overlap() {
    SuppressStderr _;
    auto r = minimize(2, {1, 2}, {2, 3}); // 2 重疊
    EXPECT(r.numVars == 0, "重疊的 minterm/DC 應回傳空 QMResult");
    return true;
}

// varNames 長度不符
bool test_error_varnames_length() {
    SuppressStderr _;
    auto r = minimize(3, {0}, {}, {"A", "B"}); // 長度 2 ≠ numVars 3
    EXPECT(r.numVars == 0, "varNames 長度不符應回傳空 QMResult");
    return true;
}

// Circuit output 名稱不存在
bool test_error_invalid_output() {
    SuppressStderr _;
    Circuit ha = makeHalfAdder();
    auto r = minimizeCircuitOutput(ha, "Nonexistent");
    EXPECT(r.numVars == 0, "不存在的 output 應回傳空 QMResult");
    return true;
}

// ============================================================
// Petrick's method vs 貪心法 對照測試
// ============================================================
//
// 每題同時跑 minimize()（Petrick's）與 minimizeGreedy()（貪心），
// 驗證：
//   1. 兩者都正確覆蓋所有 minterms（verifySOP 通過）
//   2. Petrick's 項數 ≤ 貪心項數（保證最簡性）
//
// 同時印出對照表供報告使用。

struct CompareCase {
    std::string               label;
    int                       numVars;
    std::vector<int>          minterms;
    std::vector<int>          dontCares;
    std::vector<std::string>  varNames;
};

// 對單一題目執行對照，回傳 true 代表測試通過
static bool runCompare(const CompareCase& tc) {
    auto rP = minimize     (tc.numVars, tc.minterms, tc.dontCares, tc.varNames);
    auto rG = minimizeGreedy(tc.numVars, tc.minterms, tc.dontCares, tc.varNames);
    int np = (int)rP.terms.size();
    int ng = (int)rG.terms.size();
    bool okP = (rP.numVars > 0) && verifySOP(rP, tc.minterms, tc.dontCares);
    bool okG = (rG.numVars > 0) && verifySOP(rG, tc.minterms, tc.dontCares);
    // Petrick's 項數必須 ≤ 貪心項數
    bool optimal = (np <= ng);
    // 對照列（印到 stdout，不影響 PASS/FAIL 計數）
    std::string lbl = tc.label;
    while ((int)lbl.size() < 34) lbl += ' ';
    std::string status = (okP && okG && optimal) ? "OK  " : "FAIL";
    std::cout << "    | " << lbl
              << " | Petrick's=" << np
              << "  Greedy=" << ng
              << "  " << status << "\n";
    return okP && okG && optimal;
}

// 對照：F(A,B) = Σm(0,1,3) → 2 項（兩者相同）
bool test_compare_2var() {
    CompareCase tc{"2var Σm(0,1,3)", 2, {0,1,3}, {}, {"A","B"}};
    EXPECT(runCompare(tc), "2var 對照失敗");
    return true;
}

// 對照：F(A,B,C) = Σm(0,1,2,3,4,6) → 2 項（兩者相同）
bool test_compare_3var() {
    CompareCase tc{"3var Σm(0,1,2,3,4,6)", 3, {0,1,2,3,4,6}, {}, {"A","B","C"}};
    EXPECT(runCompare(tc), "3var 對照失敗");
    return true;
}

// 對照：F(A,B,C,D) = Σm(0,1,2,5,6,7,8,9,10,14) → 3 項
bool test_compare_4var_classic() {
    CompareCase tc{"4var Σm(0,1,2,5,6,7,8,9,10,14)", 4, {0,1,2,5,6,7,8,9,10,14}, {}, {"A","B","C","D"}};
    EXPECT(runCompare(tc), "4var classic 對照失敗");
    return true;
}

// 對照：循環覆蓋表（cyclic chart）— 8 個 PI，無 essential PI
// F(A,B,C,D) = Σm(2,3,5,7,8,10,12,13)
// Petrick's 保證選出 4 項的最小覆蓋；貪心可能亦為 4 項或更多
bool test_compare_cyclic() {
    CompareCase tc{"4var cyclic Σm(2,3,5,7,8,10,12,13)", 4, {2,3,5,7,8,10,12,13}, {}, {"A","B","C","D"}};
    EXPECT(runCompare(tc), "cyclic 對照失敗");
    // 額外驗證：Petrick's 選出 4 項（本題已知最小解）
    auto rP = minimize(4, {2,3,5,7,8,10,12,13}, {}, {"A","B","C","D"});
    EXPECT((int)rP.terms.size() == 4, "cyclic 最小解應為 4 項");
    return true;
}

// 對照：含 don't care
// F(A,B,C) = Σm(1,3,7) + d(0,5) → 1 項
bool test_compare_with_dc() {
    CompareCase tc{"3var+DC Σm(1,3,7)+d(0,5)", 3, {1,3,7}, {0,5}, {"A","B","C"}};
    EXPECT(runCompare(tc), "don't care 對照失敗");
    return true;
}

// 對照：5 變數
// F(A,B,C,D,E) = Σm(0,1,4,5,16,17,20,21) → 應化簡為 A'C' 或 2 項以內
bool test_compare_5var() {
    std::vector<int> ms = {0,1,4,5,16,17,20,21};
    CompareCase tc{"5var Σm(0,1,4,5,16,17,20,21)", 5, ms, {}, {"A","B","C","D","E"}};
    EXPECT(runCompare(tc), "5var 對照失敗");
    // 額外驗證覆蓋正確性
    auto rP = minimize(5, ms, {}, {"A","B","C","D","E"});
    EXPECT(verifySOP(rP, ms), "5var SOP 未正確覆蓋 minterms");
    return true;
}

// 對照：4 變數全部 essential PI（無循環）— majority 類
// full adder Cout: Σm(3,5,6,7)
bool test_compare_majority() {
    CompareCase tc{"3var majority Σm(3,5,6,7)", 3, {3,5,6,7}, {}, {"A","B","Cin"}};
    EXPECT(runCompare(tc), "majority 對照失敗");
    return true;
}

// 對照：單一乘積項
bool test_compare_single_term() {
    CompareCase tc{"3var single Σm(5)", 3, {5}, {}, {"A","B","C"}};
    EXPECT(runCompare(tc), "single term 對照失敗");
    return true;
}

// ── 主程式 ───────────────────────────────────────────────────
int main() {
    std::cout << "\n╔══════════════════════════════════════════════╗\n";
    std::cout << "║   Quine-McCluskey Minimizer Unit Tests      ║\n";
    std::cout << "╚══════════════════════════════════════════════╝\n\n";

    std::cout << "=== 基本化簡 ===\n";
    run("two_var_basic (A'+B)",         test_two_var_basic);
    run("three_var_absorption (A'+C')", test_three_var_absorption);
    run("four_var_classic (3 項)",      test_four_var_classic);
    run("single_minterm (AB'C)",        test_single_minterm);
    run("already_minimal (XOR, 2 項)",  test_already_minimal);

    std::cout << "\n=== 邊界條件 ===\n";
    run("constant_zero",                test_constant_zero);
    run("constant_one",                 test_constant_one);
    run("default_varnames (x0,x1,...)", test_default_varnames);

    std::cout << "\n=== Don't Care ===\n";
    run("dc_enables_merge (→C)",        test_dc_enables_merge);
    run("dc_in_coverage (→A+B)",        test_dc_in_coverage);

    std::cout << "\n=== 從 Circuit 提取 ===\n";
    run("circuit_full_adder_cout",      test_circuit_full_adder_cout);
    run("circuit_half_adder_s (XOR)",   test_circuit_half_adder_s);
    run("circuit_half_adder_c (AND)",   test_circuit_half_adder_c);

    std::cout << "\n=== 錯誤處理 ===\n";
    run("error_out_of_range",           test_error_out_of_range);
    run("error_dc_out_of_range",        test_error_dc_out_of_range);
    run("error_overlap",                test_error_overlap);
    run("error_varnames_length",        test_error_varnames_length);
    run("error_invalid_output",         test_error_invalid_output);

    std::cout << "\n=== Petrick's vs 貪心 對照表 ===\n";
    std::cout << "    | 題目                             | Petrick's  | Greedy  | 狀態\n";
    std::cout << "    |----------------------------------|------------|---------|------\n";
    run("compare_2var",                 test_compare_2var);
    run("compare_3var",                 test_compare_3var);
    run("compare_4var_classic",         test_compare_4var_classic);
    run("compare_cyclic (4var, 8PI)",   test_compare_cyclic);
    run("compare_with_dc",              test_compare_with_dc);
    run("compare_5var",                 test_compare_5var);
    run("compare_majority",             test_compare_majority);
    run("compare_single_term",          test_compare_single_term);

    std::cout << "\n";
    if (g_failed == 0)
        std::cout << "All " << g_passed << " tests passed.\n\n";
    else
        std::cout << g_passed << " passed, " << g_failed << " FAILED.\n\n";

    return g_failed > 0 ? 1 : 0;
}
