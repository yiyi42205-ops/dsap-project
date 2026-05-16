#include "circuit.h"
#include "critical_path.h"
#include "json_export.h"
#include "sop_to_circuit.h"
#include "qm_minimizer.h"

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

// 4-bit 加法器（串接 4 個全加器）
Circuit create4BitAdder() {
    Circuit c;
    c.addInput("A0"); c.addInput("A1"); c.addInput("A2"); c.addInput("A3");
    c.addInput("B0"); c.addInput("B1"); c.addInput("B2"); c.addInput("B3");
    c.addInput("Cin");
    c.addOutput("S0"); c.addOutput("S1"); c.addOutput("S2"); c.addOutput("S3");
    c.addOutput("Cout");

    c.addGate("XOR", "XOR_0a", {"A0", "B0"}, "T0");
    c.addGate("XOR", "XOR_0b", {"T0", "Cin"}, "S0");
    c.addGate("AND", "AND_0a", {"A0", "B0"}, "G0");
    c.addGate("AND", "AND_0b", {"T0", "Cin"}, "P0");
    c.addGate("OR",  "OR_0",   {"G0", "P0"}, "C0");

    c.addGate("XOR", "XOR_1a", {"A1", "B1"}, "T1");
    c.addGate("XOR", "XOR_1b", {"T1", "C0"}, "S1");
    c.addGate("AND", "AND_1a", {"A1", "B1"}, "G1");
    c.addGate("AND", "AND_1b", {"T1", "C0"}, "P1");
    c.addGate("OR",  "OR_1",   {"G1", "P1"}, "C1");

    c.addGate("XOR", "XOR_2a", {"A2", "B2"}, "T2");
    c.addGate("XOR", "XOR_2b", {"T2", "C1"}, "S2");
    c.addGate("AND", "AND_2a", {"A2", "B2"}, "G2");
    c.addGate("AND", "AND_2b", {"T2", "C1"}, "P2");
    c.addGate("OR",  "OR_2",   {"G2", "P2"}, "C2");

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
    // ── 解析命令列引數 ───────────────────────────────────────
    int  topK       = 3;   // Critical Path Top-K，預設 3
    std::string fileArg    = "";
    std::string exportJson = ""; // --export-json <path>，空字串表示不匯出
    std::string qmArg      = ""; // --qm <file.tt>
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "--critical-path" && i + 1 < argc) {
            try {
                topK = std::stoi(argv[++i]);
                if (topK <= 0) { std::cerr << "錯誤：--critical-path K 必須為正整數\n"; return 1; }
            } catch (...) {
                std::cerr << "錯誤：--critical-path 的引數不是有效整數：" << argv[i] << "\n";
                return 1;
            }
        } else if (arg == "--export-json" && i + 1 < argc) {
            exportJson = argv[++i];
        } else if (arg == "--qm" && i + 1 < argc) {
            qmArg = argv[++i];
        } else {
            fileArg = arg;
        }
    }

    // ── --qm 模式：解析 .tt、跑 QM、印結果或匯出 JSON ───────
    if (!qmArg.empty()) {
        int numVars = 0;
        std::vector<int> minterms, dontCares;
        std::vector<std::string> varNames;
        if (!parseTruthTableFile(qmArg, numVars, minterms, dontCares, varNames))
            return 1;

        QMResult qmResult = minimize(numVars, minterms, dontCares, varNames);
        if (qmResult.numVars == 0) return 1;

        // 恆 0 / 恆 1 提示
        bool isConst0 = qmResult.terms.empty();
        bool isConst1 = (qmResult.terms.size() == 1 && qmResult.terms[0].care == 0);
        if (isConst0) std::cout << "函數恆 0，無需電路\n";
        if (isConst1) std::cout << "函數恆 1，無需電路\n";

        printQMResult(qmResult);

        if (!exportJson.empty()) {
            if (!exportQMToJson(qmArg, exportJson, topK)) return 1;
            std::cout << "Exported to " << exportJson << "\n";
        }
        return 0;
    }

    // ── --export-json 模式：靜默匯出，不印其他輸出 ──────────
    if (!exportJson.empty()) {
        if (fileArg.empty()) {
            std::cerr << "錯誤：--export-json 需要指定電路檔案路徑\n";
            return 1;
        }
        Circuit circuit;
        if (!circuit.loadFromFile(fileArg)) return 1;
        std::string name = circuitNameFromPath(fileArg);
        if (!exportCircuitToJson(circuit, name, exportJson, topK)) return 1;
        std::cout << "Exported to " << exportJson << "\n";
        return 0;
    }

    // ── 一般模式：印 banner ──────────────────────────────────
    std::cout << "╔══════════════════════════════════════╗\n";
    std::cout << "║  數位邏輯電路模擬器 v2.0             ║\n";
    std::cout << "║  Digital Logic Circuit Simulator     ║\n";
    std::cout << "╚══════════════════════════════════════╝\n\n";

    if (!fileArg.empty()) {
        std::cout << "從檔案載入電路：" << fileArg << "\n";
        Circuit circuit;
        if (!circuit.loadFromFile(fileArg)) return 1;

        if (!circuit.topologicalSort()) return 1;
        circuit.printInfo();
        std::cout << "=== 真值表 ===\n";
        circuit.generateTruthTable();
        auto paths = computeTopKCriticalPaths(
            circuit.getAllGates(), circuit.getInputs(),
            circuit.getOutputs(), topK);
        printCriticalPaths(paths);
        circuit.performanceComparison();
        return 0;
    }

    std::cout << "請選擇內建範例電路：\n";
    std::cout << "  1. 半加器 (Half Adder)         - 2 閘\n";
    std::cout << "  2. 全加器 (Full Adder)         - 5 閘\n";
    std::cout << "  3. 2-to-1 多工器 (MUX)         - 4 閘\n";
    std::cout << "  4. 4-bit 加法器 (4-bit Adder)  - 20 閘\n";
    std::cout << "  5. 從檔案載入\n";
    std::cout << "  6. 效能比較模式（比較所有內建電路）\n";
    std::cout << "\n請輸入選項 (1-6): ";

    int choice;
    std::cin >> choice;

    if (choice == 6) {
        std::cout << "\n========== 效能比較模式 ==========\n\n";
        auto runBench = [](const std::string& label, Circuit c) {
            std::cout << "--- " << label << " ---\n";
            c.topologicalSort();
            c.performanceComparison();
            std::cout << "\n";
        };
        runBench("半加器 (2 閘)",       createHalfAdder());
        runBench("全加器 (5 閘)",       createFullAdder());
        runBench("2-to-1 MUX (4 閘)",  createMux2to1());
        runBench("4-bit 加法器 (20 閘)", create4BitAdder());
        return 0;
    }

    Circuit circuit;
    switch (choice) {
        case 1: std::cout << "\n--- 半加器 ---\n\n";    circuit = createHalfAdder();  break;
        case 2: std::cout << "\n--- 全加器 ---\n\n";    circuit = createFullAdder();  break;
        case 3: std::cout << "\n--- 2-to-1 MUX ---\n\n"; circuit = createMux2to1();  break;
        case 4: std::cout << "\n--- 4-bit 加法器 ---\n\n"; circuit = create4BitAdder(); break;
        case 5: {
            std::cout << "請輸入檔案路徑: ";
            std::string filename;
            std::cin >> filename;
            if (!circuit.loadFromFile(filename)) return 1;
            break;
        }
        default: std::cerr << "無效選項\n"; return 1;
    }

    if (!circuit.topologicalSort()) return 1;
    circuit.printInfo();
    std::cout << "=== 真值表 ===\n";
    circuit.generateTruthTable();
    auto paths = computeTopKCriticalPaths(
        circuit.getAllGates(), circuit.getInputs(),
        circuit.getOutputs(), topK);
    printCriticalPaths(paths);
    circuit.performanceComparison();
    return 0;
}
