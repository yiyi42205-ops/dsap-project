// test_sop_to_circuit.cpp
// 驗證 sopToCircuit、mintermToCircuit、parseTruthTableFile
//
// 編譯：g++ -std=c++17 -O2 -o tests/test_sop_to_circuit tests/test_sop_to_circuit.cpp
// 執行：./tests/test_sop_to_circuit

#include "../src/sop_to_circuit.h"
#include "../src/qm_minimizer.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <cstdio>   // std::tmpnam / std::remove

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

struct SuppressStderr {
    std::streambuf* orig;
    std::stringstream buf;
    SuppressStderr()  { orig = std::cerr.rdbuf(buf.rdbuf()); }
    ~SuppressStderr() { std::cerr.rdbuf(orig); }
};

// ── 輔助：對電路跑全真值表，回傳各 combo 下 outputName 的值 ──
static std::vector<bool> evalCircuit(Circuit& c, const std::string& outputName) {
    c.topologicalSort();
    int n     = c.getInputCount();
    int total = 1 << n;
    std::vector<bool> results(total);
    for (int combo = 0; combo < total; combo++) {
        std::vector<bool> vals(n);
        for (int i = 0; i < n; i++) vals[n - 1 - i] = (combo >> i) & 1;
        c.simulate(vals);
        results[combo] = c.getSignal(outputName);
    }
    return results;
}

// ── 輔助：建立臨時 .tt 檔 ────────────────────────────────────
// 回傳臨時檔路徑；呼叫端負責 std::remove
static std::string writeTempTT(const std::string& content) {
    // 用固定名稱（tests/ 目錄）避免 tmpnam 警告
    static int counter = 0;
    std::string path = "tests/tmp_tt_" + std::to_string(counter++) + ".tt";
    std::ofstream f(path);
    f << content;
    return path;
}

// ============================================================
// sopToCircuit 測試
// ============================================================

// 1. 單一乘積項不產生 OR 閘
static bool test_sop_single_term() {
    // F = AB  →  一個 AND 閘，無 OR 閘
    QMResult r;
    r.numVars = 2;
    r.varNames = {"A", "B"};
    r.terms.push_back({0b11u, 0b11u}); // A B（均正形式，care=11, polarity=11）

    auto opt = sopToCircuit(r, "F");
    EXPECT(opt.has_value(), "單一乘積項應回傳有效電路");

    Circuit& c = *opt;
    c.topologicalSort();
    auto gates = c.getAllGates();

    // 不應有 OR 閘
    for (Gate* g : gates)
        EXPECT(g->type != "OR", "單一乘積項不應有 OR 閘");

    // 真值表：F=1 只有 combo=3（AB=11）
    auto results = evalCircuit(c, "F");
    EXPECT(results[0] == false, "F(0,0) should be 0");
    EXPECT(results[1] == false, "F(0,1) should be 0");
    EXPECT(results[2] == false, "F(1,0) should be 0");
    EXPECT(results[3] == true,  "F(1,1) should be 1");
    return true;
}

// 2. 多乘積項有 OR 閘
static bool test_sop_multi_term() {
    // F = A'B + AB' （XOR）→ 兩個 AND 閘 + 一個 OR 閘
    QMResult r;
    r.numVars = 2;
    r.varNames = {"A", "B"};
    // A'B: care=11, polarity=01（A 負，B 正），bit0=A, bit1=B
    r.terms.push_back({0b11u, 0b10u}); // care=11, polarity=10 → A正 B負？
    // 重新確認 bit encoding：bit i 對應 varNames[i]
    // varNames[0]=A, varNames[1]=B
    // A'B: A 負(bit0 polarity=0), B 正(bit1 polarity=1) → care=0b11, polarity=0b10
    // AB': A 正(bit0 polarity=1), B 負(bit1 polarity=0) → care=0b11, polarity=0b01
    r.terms.clear();
    r.terms.push_back({0b11u, 0b10u}); // A'B
    r.terms.push_back({0b11u, 0b01u}); // AB'

    auto opt = sopToCircuit(r, "F");
    EXPECT(opt.has_value(), "多乘積項應回傳有效電路");

    Circuit& c = *opt;
    c.topologicalSort();
    auto gates = c.getAllGates();

    bool hasOR = false;
    for (Gate* g : gates) if (g->type == "OR") hasOR = true;
    EXPECT(hasOR, "多乘積項應有 OR 閘");

    // XOR 真值表：F=1 iff A!=B（combo 1 和 2）
    auto results = evalCircuit(c, "F");
    EXPECT(results[0] == false, "F(0,0) XOR should be 0");
    EXPECT(results[1] == true,  "F(0,1) XOR should be 1");
    EXPECT(results[2] == true,  "F(1,0) XOR should be 1");
    EXPECT(results[3] == false, "F(1,1) XOR should be 0");
    return true;
}

// 3. 反變數加 NOT 閘，同一變數不重複
static bool test_sop_complement() {
    // F = A'B' + AB → XNOR（使用 A' 和 B'）
    QMResult r;
    r.numVars = 2;
    r.varNames = {"A", "B"};
    r.terms.push_back({0b11u, 0b00u}); // A'B'
    r.terms.push_back({0b11u, 0b11u}); // AB

    auto opt = sopToCircuit(r, "F");
    EXPECT(opt.has_value(), "XNOR 應回傳有效電路");

    Circuit& c = *opt;
    c.topologicalSort();
    auto gates = c.getAllGates();

    // 計算 NOT 閘數：A' 和 B' 各需一個，共 2 個，不應重複
    int notCount = 0;
    for (Gate* g : gates) if (g->type == "NOT") notCount++;
    EXPECT(notCount == 2, "應有且只有 2 個 NOT 閘（A' 和 B'）");

    // XNOR 真值表：F=1 iff A==B
    auto results = evalCircuit(c, "F");
    EXPECT(results[0] == true,  "F(0,0) XNOR should be 1");
    EXPECT(results[1] == false, "F(0,1) XNOR should be 0");
    EXPECT(results[2] == false, "F(1,0) XNOR should be 0");
    EXPECT(results[3] == true,  "F(1,1) XNOR should be 1");
    return true;
}

// 4. 恆 0 → nullopt
static bool test_sop_const_zero() {
    QMResult r;
    r.numVars = 2;
    r.varNames = {"A", "B"};
    // terms 為空 = 恆 0
    auto opt = sopToCircuit(r, "F");
    EXPECT(!opt.has_value(), "恆 0 應回傳 nullopt");
    return true;
}

// 5. 恆 1 → nullopt
static bool test_sop_const_one() {
    QMResult r;
    r.numVars = 2;
    r.varNames = {"A", "B"};
    r.terms.push_back({0u, 0u}); // care=0 → 恆 1
    auto opt = sopToCircuit(r, "F");
    EXPECT(!opt.has_value(), "恆 1 應回傳 nullopt");
    return true;
}

// 6. sopToCircuit 結果與 QM 輸入等價（equivalence_checker）
static bool test_sop_equiv_qm() {
    // F(A,B,C) = 多數決：minimize({3,5,6,7}) = AB + AC + BC
    std::vector<int> minterms = {3, 5, 6, 7};
    std::vector<std::string> vars = {"A", "B", "C"};
    QMResult qr = minimize(3, minterms, {}, vars);
    EXPECT(qr.numVars == 3, "QM 應成功");

    auto opt = sopToCircuit(qr, "F");
    EXPECT(opt.has_value(), "多數決應有有效電路");

    // 用 mintermToCircuit 建參考電路（直接展開），再做等價比較
    auto refOpt = mintermToCircuit(3, minterms, vars, "F");
    EXPECT(refOpt.has_value(), "直接版應有有效電路");

    auto r1 = evalCircuit(*opt,    "F");
    auto r2 = evalCircuit(*refOpt, "F");
    EXPECT(r1 == r2, "sopToCircuit 應與直接展開版等價");
    return true;
}

// ============================================================
// mintermToCircuit 測試
// ============================================================

// 7. 正確覆蓋所有 on-set minterms
static bool test_minterm_basic() {
    // F(A,B) = minterms {1,2}  →  F=1 for (A=0,B=1)=combo1 and (A=1,B=0)=combo2
    std::vector<int> minterms = {1, 2};
    std::vector<std::string> vars = {"A", "B"};
    auto opt = mintermToCircuit(2, minterms, vars, "F");
    EXPECT(opt.has_value(), "minterms {1,2} 應有有效電路");

    auto results = evalCircuit(*opt, "F");
    EXPECT(results[0] == false, "F(0,0) should be 0");
    EXPECT(results[1] == true,  "F(0,1) should be 1");
    EXPECT(results[2] == true,  "F(1,0) should be 1");
    EXPECT(results[3] == false, "F(1,1) should be 0");
    return true;
}

// 8. 空 minterms → nullopt
static bool test_minterm_const_zero() {
    auto opt = mintermToCircuit(2, {}, {"A", "B"}, "F");
    EXPECT(!opt.has_value(), "空 minterms 應回傳 nullopt");
    return true;
}

// 9. 全 minterms → nullopt
static bool test_minterm_const_one() {
    auto opt = mintermToCircuit(2, {0, 1, 2, 3}, {"A", "B"}, "F");
    EXPECT(!opt.has_value(), "全 minterms 應回傳 nullopt");
    return true;
}

// 10. 直接版與 QM 版真值表相同
static bool test_minterm_vs_sop() {
    std::vector<int> minterms = {1, 3, 5, 7};
    std::vector<std::string> vars = {"A", "B", "C"};
    QMResult qr = minimize(3, minterms, {}, vars);
    EXPECT(qr.numVars == 3, "QM 應成功");

    auto directOpt = mintermToCircuit(3, minterms, vars, "F");
    auto sopOpt    = sopToCircuit(qr, "F");
    EXPECT(directOpt.has_value(), "直接版應有有效電路");
    EXPECT(sopOpt.has_value(),    "SOP 版應有有效電路");

    auto r1 = evalCircuit(*directOpt, "F");
    auto r2 = evalCircuit(*sopOpt,    "F");
    EXPECT(r1 == r2, "直接版與 QM 版真值表應完全相同");
    return true;
}

// ============================================================
// parseTruthTableFile 測試
// ============================================================

// 11. 正常 .tt 解析正確
static bool test_parse_basic() {
    std::string path = writeTempTT(
        "VARS A B C\n"
        "MINTERMS 1 3 5 7\n"
        "DONTCARES 0\n"
    );
    int numVars; std::vector<int> m, dc; std::vector<std::string> v;
    bool ok = parseTruthTableFile(path, numVars, m, dc, v);
    std::remove(path.c_str());
    EXPECT(ok,          "正常 .tt 應解析成功");
    EXPECT(numVars == 3, "numVars 應為 3");
    EXPECT(v.size() == 3 && v[0] == "A" && v[1] == "B" && v[2] == "C", "varNames 應正確");
    EXPECT(m.size() == 4, "minterms 應有 4 項");
    EXPECT(dc.size() == 1 && dc[0] == 0, "dontCares 應有 1 項");
    return true;
}

// 12. 省略 DONTCARES 行
static bool test_parse_no_dontcare() {
    std::string path = writeTempTT(
        "VARS A B\n"
        "MINTERMS 1 2\n"
    );
    int numVars; std::vector<int> m, dc; std::vector<std::string> v;
    bool ok = parseTruthTableFile(path, numVars, m, dc, v);
    std::remove(path.c_str());
    EXPECT(ok,           "省略 DONTCARES 應解析成功");
    EXPECT(dc.empty(),   "dontCares 應為空");
    EXPECT(m.size() == 2, "minterms 應有 2 項");
    return true;
}

// 13. # 註解與空行被忽略
static bool test_parse_comments() {
    std::string path = writeTempTT(
        "# 這是註解\n"
        "\n"
        "VARS X Y\n"
        "# 另一行註解\n"
        "MINTERMS 0 3\n"
    );
    int numVars; std::vector<int> m, dc; std::vector<std::string> v;
    bool ok = parseTruthTableFile(path, numVars, m, dc, v);
    std::remove(path.c_str());
    EXPECT(ok,          "含註解的 .tt 應解析成功");
    EXPECT(numVars == 2, "numVars 應為 2");
    EXPECT(v[0] == "X" && v[1] == "Y", "varNames 應正確");
    EXPECT(m.size() == 2, "minterms 應有 2 項");
    return true;
}

// 14. 超出範圍 minterm → false
static bool test_parse_bad_minterm() {
    std::string path = writeTempTT(
        "VARS A B\n"
        "MINTERMS 0 4\n"  // 4 >= 2^2=4，超出範圍
    );
    int numVars; std::vector<int> m, dc; std::vector<std::string> v;
    SuppressStderr ss;
    bool ok = parseTruthTableFile(path, numVars, m, dc, v);
    std::remove(path.c_str());
    EXPECT(!ok, "超出範圍的 minterm 應回傳 false");
    return true;
}

// 15. 缺 VARS 行 → false
static bool test_parse_missing_vars() {
    std::string path = writeTempTT(
        "MINTERMS 0 1\n"
    );
    int numVars; std::vector<int> m, dc; std::vector<std::string> v;
    SuppressStderr ss;
    bool ok = parseTruthTableFile(path, numVars, m, dc, v);
    std::remove(path.c_str());
    EXPECT(!ok, "缺 VARS 行應回傳 false");
    return true;
}

// 16. minterm/dontcare 重疊 → false
static bool test_parse_overlap() {
    std::string path = writeTempTT(
        "VARS A B\n"
        "MINTERMS 0 1 2\n"
        "DONTCARES 2\n"   // 2 同時在 MINTERMS
    );
    int numVars; std::vector<int> m, dc; std::vector<std::string> v;
    SuppressStderr ss;
    bool ok = parseTruthTableFile(path, numVars, m, dc, v);
    std::remove(path.c_str());
    EXPECT(!ok, "minterm/dontcare 重疊應回傳 false");
    return true;
}

// ============================================================
// main
// ============================================================
int main() {
    std::cout << "\n╔══════════════════════════════════════╗\n";
    std::cout << "║  test_sop_to_circuit                 ║\n";
    std::cout << "╚══════════════════════════════════════╝\n\n";

    std::cout << "[sopToCircuit]\n";
    run("test_sop_single_term",  test_sop_single_term);
    run("test_sop_multi_term",   test_sop_multi_term);
    run("test_sop_complement",   test_sop_complement);
    run("test_sop_const_zero",   test_sop_const_zero);
    run("test_sop_const_one",    test_sop_const_one);
    run("test_sop_equiv_qm",     test_sop_equiv_qm);

    std::cout << "\n[mintermToCircuit]\n";
    run("test_minterm_basic",      test_minterm_basic);
    run("test_minterm_const_zero", test_minterm_const_zero);
    run("test_minterm_const_one",  test_minterm_const_one);
    run("test_minterm_vs_sop",     test_minterm_vs_sop);

    std::cout << "\n[parseTruthTableFile]\n";
    run("test_parse_basic",        test_parse_basic);
    run("test_parse_no_dontcare",  test_parse_no_dontcare);
    run("test_parse_comments",     test_parse_comments);
    run("test_parse_bad_minterm",  test_parse_bad_minterm);
    run("test_parse_missing_vars", test_parse_missing_vars);
    run("test_parse_overlap",      test_parse_overlap);

    std::cout << "\n────────────────────────────────────────\n";
    std::cout << "Passed: " << g_passed
              << "  Failed: " << g_failed << "\n\n";
    return (g_failed == 0) ? 0 : 1;
}
