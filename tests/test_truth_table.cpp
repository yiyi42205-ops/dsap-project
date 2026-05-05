// test_truth_table.cpp
// 驗證半加器（4 組）、全加器（8 組）、4-bit 加法器（隨機 20 組）的真值表輸出
//
// 編譯：g++ -std=c++17 -O2 -o test_truth_table test_truth_table.cpp
// 執行：./test_truth_table

#include "../src/circuit.h"
#include <cassert>
#include <random>
#include <iostream>

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

// ── 輔助：建立電路並完成拓撲排序 ─────────────────────────────
static Circuit makeHalfAdder() {
    Circuit c;
    c.addInput("A"); c.addInput("B");
    c.addOutput("S"); c.addOutput("C");
    c.addGate("XOR", "XOR_1", {"A", "B"}, "S");
    c.addGate("AND", "AND_1", {"A", "B"}, "C");
    c.topologicalSort();
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
    c.topologicalSort();
    return c;
}

static Circuit make4BitAdder() {
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

    c.topologicalSort();
    return c;
}

// ── 測試函式 ─────────────────────────────────────────────────

// 半加器：S = A XOR B, C = A AND B
bool test_half_adder_all_combos() {
    Circuit c = makeHalfAdder();
    // {A, B} → expected {S, C}
    const std::vector<std::tuple<bool,bool,bool,bool>> cases = {
        {false, false, false, false},
        {false, true,  true,  false},
        {true,  false, true,  false},
        {true,  true,  false, true },
    };
    for (auto& [a, b, expS, expC] : cases) {
        c.simulate({a, b});
        EXPECT(c.getSignal("S") == expS,
               std::string("HalfAdder S mismatch A=") + (a?"1":"0") + " B=" + (b?"1":"0"));
        EXPECT(c.getSignal("C") == expC,
               std::string("HalfAdder C mismatch A=") + (a?"1":"0") + " B=" + (b?"1":"0"));
    }
    return true;
}

// 全加器：S = A XOR B XOR Cin, Cout = majority(A,B,Cin)
bool test_full_adder_all_combos() {
    Circuit c = makeFullAdder();
    for (int i = 0; i < 8; i++) {
        bool a   = (i >> 2) & 1;
        bool b   = (i >> 1) & 1;
        bool cin = (i >> 0) & 1;
        int  sum = (int)a + (int)b + (int)cin;
        bool expS    = (sum & 1);
        bool expCout = (sum >> 1) & 1;
        c.simulate({a, b, cin});
        EXPECT(c.getSignal("S")    == expS,    "FullAdder S mismatch");
        EXPECT(c.getSignal("Cout") == expCout, "FullAdder Cout mismatch");
    }
    return true;
}

// 4-bit 加法器：隨機抽 20 組，驗證 S[3:0] 和 Cout
bool test_4bit_adder_random_20() {
    Circuit c = make4BitAdder();
    std::mt19937 rng(2024);
    std::uniform_int_distribution<int> distA(0, 15);   // 4-bit
    std::uniform_int_distribution<int> distB(0, 15);
    std::uniform_int_distribution<int> distCin(0, 1);

    for (int trial = 0; trial < 20; trial++) {
        int A   = distA(rng);
        int B   = distB(rng);
        int Cin = distCin(rng);
        int Sum = A + B + Cin;

        // 拆位元：inputSignals 順序 A0 A1 A2 A3 B0 B1 B2 B3 Cin
        c.simulate({
            (bool)((A >> 0) & 1), (bool)((A >> 1) & 1),
            (bool)((A >> 2) & 1), (bool)((A >> 3) & 1),
            (bool)((B >> 0) & 1), (bool)((B >> 1) & 1),
            (bool)((B >> 2) & 1), (bool)((B >> 3) & 1),
            (bool)Cin
        });

        bool expS0 = (Sum >> 0) & 1;
        bool expS1 = (Sum >> 1) & 1;
        bool expS2 = (Sum >> 2) & 1;
        bool expS3 = (Sum >> 3) & 1;
        bool expCo = (Sum >> 4) & 1;

        EXPECT(c.getSignal("S0")   == expS0, "4bitAdder S0 mismatch");
        EXPECT(c.getSignal("S1")   == expS1, "4bitAdder S1 mismatch");
        EXPECT(c.getSignal("S2")   == expS2, "4bitAdder S2 mismatch");
        EXPECT(c.getSignal("S3")   == expS3, "4bitAdder S3 mismatch");
        EXPECT(c.getSignal("Cout") == expCo, "4bitAdder Cout mismatch");
    }
    return true;
}

// ── 主程式 ───────────────────────────────────────────────────
int main() {
    std::cout << "=== test_truth_table ===\n";
    run("half_adder_all_4_combos",       test_half_adder_all_combos);
    run("full_adder_all_8_combos",       test_full_adder_all_combos);
    run("4bit_adder_random_20_combos",   test_4bit_adder_random_20);
    std::cout << "\n" << g_passed << " passed, " << g_failed << " failed.\n";
    return g_failed > 0 ? 1 : 0;
}
