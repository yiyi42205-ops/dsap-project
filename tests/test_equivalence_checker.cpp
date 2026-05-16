// test_equivalence_checker.cpp
// 驗證電路等價性檢查器的正確性
//
// 涵蓋：等價電路、不等價電路（含反例驗證）、錯誤處理
//
// 編譯：g++ -std=c++17 -O2 -o tests/test_equivalence_checker tests/test_equivalence_checker.cpp
// 執行：./tests/test_equivalence_checker

#include "../src/equivalence_checker.h"
#include <iostream>
#include <sstream>

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

// ── 內建電路 helper ──────────────────────────────────────────
static Circuit makeHalfAdder() {
    Circuit c;
    c.addInput("A"); c.addInput("B");
    c.addOutput("S"); c.addOutput("C");
    c.addGate("XOR", "XOR_1", {"A", "B"}, "S");
    c.addGate("AND", "AND_1", {"A", "B"}, "C");
    return c;
}

// 半加器的 NAND 版本：S = NAND(NAND(A,NAND(A,B)), NAND(B,NAND(A,B)))
//                    C = NAND(NAND(A,B), NAND(A,B))
// 功能與標準半加器完全相同，但全用 NAND 閘實作
static Circuit makeHalfAdderNand() {
    Circuit c;
    c.addInput("A"); c.addInput("B");
    c.addOutput("S"); c.addOutput("C");
    // W1 = NAND(A,B)
    c.addGate("NAND", "NAND_1", {"A", "B"},   "W1");
    // W2 = NAND(A, W1)
    c.addGate("NAND", "NAND_2", {"A", "W1"},  "W2");
    // W3 = NAND(B, W1)
    c.addGate("NAND", "NAND_3", {"B", "W1"},  "W3");
    // S  = NAND(W2, W3) = XOR(A,B)
    c.addGate("NAND", "NAND_4", {"W2", "W3"}, "S");
    // C  = NAND(W1, W1) = NOT(NAND(A,B)) = AND(A,B)
    c.addGate("NAND", "NAND_5", {"W1", "W1"}, "C");
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

static Circuit makeMux2to1() {
    Circuit c;
    c.addInput("A"); c.addInput("B"); c.addInput("Sel");
    c.addOutput("Y");
    c.addGate("NOT", "NOT_1", {"Sel"},        "NotSel");
    c.addGate("AND", "AND_1", {"NotSel", "A"}, "W1");
    c.addGate("AND", "AND_2", {"Sel", "B"},    "W2");
    c.addGate("OR",  "OR_1",  {"W1", "W2"},    "Y");
    return c;
}

// ============================================================
// 等價電路
// ============================================================

// 最基本：同一種電路建兩份，必定等價
bool test_equiv_same_circuit() {
    Circuit c1 = makeHalfAdder();
    Circuit c2 = makeHalfAdder();
    auto r = checkEquivalence(c1, c2);
    EXPECT(r.equivalent == true,         "同一電路應等價");
    EXPECT(r.hasCounterExample == false, "等價時不應有反例");
    return true;
}

// 標準 XOR+AND 半加器 vs 全 NAND 實作——邏輯相同，閘不同
bool test_equiv_different_impl() {
    Circuit c1 = makeHalfAdder();
    Circuit c2 = makeHalfAdderNand();
    auto r = checkEquivalence(c1, c2);
    EXPECT(r.equivalent == true,         "NAND 版半加器應與標準版等價");
    EXPECT(r.hasCounterExample == false, "等價時不應有反例");
    return true;
}

// 全加器兩份獨立建構，5 閘，8 種輸入全部一致
bool test_equiv_full_adder() {
    Circuit c1 = makeFullAdder();
    Circuit c2 = makeFullAdder();
    auto r = checkEquivalence(c1, c2);
    EXPECT(r.equivalent == true, "兩份全加器應等價");
    return true;
}

// MUX 兩份，3 輸入 8 種組合
bool test_equiv_mux() {
    Circuit c1 = makeMux2to1();
    Circuit c2 = makeMux2to1();
    auto r = checkEquivalence(c1, c2);
    EXPECT(r.equivalent == true, "兩份 MUX 應等價");
    return true;
}

// ============================================================
// 不等價電路（含反例驗證）
// ============================================================

// 半加器 vs 輸出對調的半加器（S↔C 對調）
// 位置對齊下 out[0]=S vs out[0]=C，
// 第一個 differ 的 combo：
//   combo=1 (A=0,B=1): c1.S=1,c1.C=0 vs c2.C=0,c2.S=1 → out[0]: 1 vs 0 ✗
bool test_inequiv_swap_outputs() {
    Circuit c1 = makeHalfAdder();   // output 順序：S, C
    Circuit c2;
    c2.addInput("A"); c2.addInput("B");
    c2.addOutput("C"); c2.addOutput("S");  // 順序對調
    c2.addGate("XOR", "XOR_1", {"A", "B"}, "S");
    c2.addGate("AND", "AND_1", {"A", "B"}, "C");

    auto r = checkEquivalence(c1, c2);
    EXPECT(r.equivalent == false,       "輸出對調應不等價");
    EXPECT(r.hasCounterExample == true, "應提供反例");
    // 驗證反例確實讓兩電路輸出不同
    EXPECT(r.counterExample.outputs1[0] != r.counterExample.outputs2[0] ||
           r.counterExample.outputs1[1] != r.counterExample.outputs2[1],
           "反例輸出應確實不同");
    return true;
}

// 半加器 vs 錯誤實作（XOR 換成 OR）
// S = A OR B（而非 XOR），C = A AND B
// 第一個 differ 的 combo：
//   combo=3 (A=1,B=1): c1.S=0 vs c2.S=1（OR 輸出 1，XOR 輸出 0）
bool test_inequiv_wrong_gate() {
    Circuit c1 = makeHalfAdder();

    Circuit c2;
    c2.addInput("A"); c2.addInput("B");
    c2.addOutput("S"); c2.addOutput("C");
    c2.addGate("OR",  "OR_1",  {"A", "B"}, "S");  // 錯用 OR
    c2.addGate("AND", "AND_1", {"A", "B"}, "C");

    auto r = checkEquivalence(c1, c2);
    EXPECT(r.equivalent == false,       "OR 取代 XOR 應不等價");
    EXPECT(r.hasCounterExample == true, "應提供反例");
    // A=1,B=1（combo=3）是第一個 differ 的輸入
    const auto& ce = r.counterExample;
    EXPECT(ce.inputValues[0] == true && ce.inputValues[1] == true,
           "第一個反例應為 A=1, B=1");
    EXPECT(ce.outputs1[0] != ce.outputs2[0], "S 的輸出應不同");
    return true;
}

// XOR vs XNOR（NOT(XOR)）
// 第一個 differ 的 combo：
//   combo=0 (A=0,B=0): c1.Y=XOR(0,0)=0 vs c2.Y=NOT(0)=1
bool test_inequiv_xor_vs_xnor() {
    Circuit c1;
    c1.addInput("A"); c1.addInput("B");
    c1.addOutput("Y");
    c1.addGate("XOR", "XOR_1", {"A", "B"}, "Y");

    Circuit c2;
    c2.addInput("A"); c2.addInput("B");
    c2.addOutput("Y");
    c2.addGate("XOR", "XOR_1", {"A", "B"}, "W");
    c2.addGate("NOT", "NOT_1", {"W"},        "Y");

    auto r = checkEquivalence(c1, c2);
    EXPECT(r.equivalent == false,       "XOR 與 XNOR 應不等價");
    EXPECT(r.hasCounterExample == true, "應提供反例");
    // combo=0 (A=0,B=0) 應是第一個反例
    const auto& ce = r.counterExample;
    EXPECT(ce.inputValues[0] == false && ce.inputValues[1] == false,
           "第一個反例應為 A=0, B=0");
    EXPECT(ce.outputs1[0] == false && ce.outputs2[0] == true,
           "XOR(0,0)=0，XNOR(0,0)=1");
    return true;
}

// NOT-NOT（恆等）vs 單一 NOT（取反）
// combo=0 (A=0): c1=NOT(NOT(0))=0 vs c2=NOT(0)=1
bool test_inequiv_identity_vs_inverter() {
    Circuit c1;
    c1.addInput("A");
    c1.addOutput("Y");
    c1.addGate("NOT", "NOT_1", {"A"},  "W");
    c1.addGate("NOT", "NOT_2", {"W"},  "Y");

    Circuit c2;
    c2.addInput("A");
    c2.addOutput("Y");
    c2.addGate("NOT", "NOT_1", {"A"}, "Y");

    auto r = checkEquivalence(c1, c2);
    EXPECT(r.equivalent == false,       "恆等與取反應不等價");
    EXPECT(r.hasCounterExample == true, "應提供反例");
    EXPECT(r.counterExample.outputs1[0] != r.counterExample.outputs2[0],
           "反例輸出應確實不同");
    return true;
}

// ── 反例品質驗證 ─────────────────────────────────────────────

// 對 inequiv_wrong_gate 的反例，手動代入兩電路確認輸出確實不同
bool test_counterexample_is_valid() {
    Circuit c1 = makeHalfAdder();

    Circuit c2;
    c2.addInput("A"); c2.addInput("B");
    c2.addOutput("S"); c2.addOutput("C");
    c2.addGate("OR",  "OR_1",  {"A", "B"}, "S");
    c2.addGate("AND", "AND_1", {"A", "B"}, "C");

    auto r = checkEquivalence(c1, c2);
    EXPECT(r.hasCounterExample, "應有反例");

    // 用反例的輸入值手動驗證兩電路
    c1.topologicalSort();
    c2.topologicalSort();
    c1.simulate(r.counterExample.inputValues);
    c2.simulate(r.counterExample.inputValues);

    bool any_diff = false;
    const auto& outs1 = c1.getOutputs();
    const auto& outs2 = c2.getOutputs();
    for (int i = 0; i < (int)outs1.size(); i++) {
        if (c1.getSignal(outs1[i]) != c2.getSignal(outs2[i])) {
            any_diff = true; break;
        }
    }
    EXPECT(any_diff, "反例代入電路後應確實觀察到輸出不同");
    return true;
}

// 窮舉從 combo=0 開始，反例應是最小的 combo 編號
// inequiv_wrong_gate 中第一個 differ 應是 combo=3（A=1,B=1）
bool test_counterexample_is_first() {
    Circuit c1 = makeHalfAdder();

    Circuit c2;
    c2.addInput("A"); c2.addInput("B");
    c2.addOutput("S"); c2.addOutput("C");
    c2.addGate("OR",  "OR_1",  {"A", "B"}, "S");
    c2.addGate("AND", "AND_1", {"A", "B"}, "C");

    auto r = checkEquivalence(c1, c2);
    EXPECT(r.hasCounterExample, "應有反例");

    // combo=3 = 11（A=1, B=1）是 OR 與 XOR 的第一個差異點
    // combo=1(01) S: XOR=1, OR=1 → 同；combo=2(10) S: XOR=1, OR=1 → 同
    // combo=3(11) S: XOR=0, OR=1 → 不同 ✓
    const auto& ce = r.counterExample;
    EXPECT(ce.inputValues[0] == true && ce.inputValues[1] == true,
           "應回傳最小 combo（A=1,B=1）的反例");
    return true;
}

// ============================================================
// 錯誤處理
// ============================================================

bool test_error_input_count_mismatch() {
    SuppressStderr _;
    Circuit c1 = makeHalfAdder();  // 2 inputs
    Circuit c2 = makeFullAdder();  // 3 inputs
    auto r = checkEquivalence(c1, c2);
    EXPECT(r.equivalent == false,        "input 數不符應回傳 false");
    EXPECT(r.hasCounterExample == false, "input 數不符不應有反例");
    return true;
}

bool test_error_output_count_mismatch() {
    SuppressStderr _;
    Circuit c1 = makeHalfAdder();  // 2 outputs

    Circuit c2;
    c2.addInput("A"); c2.addInput("B");
    c2.addOutput("S");  // 只有 1 output
    c2.addGate("XOR", "XOR_1", {"A", "B"}, "S");

    auto r = checkEquivalence(c1, c2);
    EXPECT(r.equivalent == false,        "output 數不符應回傳 false");
    EXPECT(r.hasCounterExample == false, "output 數不符不應有反例");
    return true;
}

bool test_error_cycle_in_c1() {
    SuppressStderr _;
    Circuit c1;
    c1.addGate("NOT", "NOT_A", {"W_B"}, "W_A");
    c1.addGate("NOT", "NOT_B", {"W_A"}, "W_B");

    Circuit c2 = makeHalfAdder();

    // 有迴圈的電路 input/output 數為 0，先會因數量不符觸發錯誤
    // 改成用相同 input/output 數的有環電路來測 cycle 偵測
    Circuit c1b;
    c1b.addInput("A"); c1b.addInput("B");
    c1b.addOutput("S");
    c1b.addGate("NOT", "NOT_1", {"W"},  "W");  // 自環
    c1b.addGate("AND", "AND_1", {"A", "B"}, "S");

    Circuit c2b;
    c2b.addInput("A"); c2b.addInput("B");
    c2b.addOutput("S");
    c2b.addGate("XOR", "XOR_1", {"A", "B"}, "S");

    auto r = checkEquivalence(c1b, c2b);
    EXPECT(r.hasCounterExample == false, "有迴圈的電路應回傳 hasCounterExample=false");
    return true;
}

// ── 主程式 ───────────────────────────────────────────────────
int main() {
    std::cout << "\n╔══════════════════════════════════════════════╗\n";
    std::cout << "║   Equivalence Checker Unit Tests            ║\n";
    std::cout << "╚══════════════════════════════════════════════╝\n\n";

    std::cout << "=== 等價電路 ===\n";
    run("equiv_same_circuit",        test_equiv_same_circuit);
    run("equiv_different_impl",      test_equiv_different_impl);
    run("equiv_full_adder",          test_equiv_full_adder);
    run("equiv_mux",                 test_equiv_mux);

    std::cout << "\n=== 不等價電路 ===\n";
    run("inequiv_swap_outputs",           test_inequiv_swap_outputs);
    run("inequiv_wrong_gate",             test_inequiv_wrong_gate);
    run("inequiv_xor_vs_xnor",           test_inequiv_xor_vs_xnor);
    run("inequiv_identity_vs_inverter",  test_inequiv_identity_vs_inverter);

    std::cout << "\n=== 反例品質 ===\n";
    run("counterexample_is_valid",   test_counterexample_is_valid);
    run("counterexample_is_first",   test_counterexample_is_first);

    std::cout << "\n=== 錯誤處理 ===\n";
    run("error_input_count_mismatch",  test_error_input_count_mismatch);
    run("error_output_count_mismatch", test_error_output_count_mismatch);
    run("error_cycle_in_c1",           test_error_cycle_in_c1);

    std::cout << "\n";
    if (g_failed == 0)
        std::cout << "All " << g_passed << " tests passed.\n\n";
    else
        std::cout << g_passed << " passed, " << g_failed << " FAILED.\n\n";

    return g_failed > 0 ? 1 : 0;
}
