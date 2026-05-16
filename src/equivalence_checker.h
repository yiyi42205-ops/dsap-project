
#pragma once
// equivalence_checker.h ── 電路等價性檢查器（窮舉版，header-only）
//
// 演算法：
//   Step 1. 驗證兩電路的 input / output 數量相符
//   Step 2. 對兩電路各執行一次拓撲排序（BFS），確認無迴圈
//   Step 3. 窮舉 2^n 種輸入組合（combo=0 遞增至 2^n-1）：
//             輸入向量的 bit 對應關係與 generateTruthTable 一致
//             即 combo bit i → inputValues[n-1-i]（MSB 對應第 0 個 input）
//           對每個 combo 分別 simulate c1、c2，
//           比對所有 output 的值（依位置對齊）
//           第一個不同的 combo → 立即回傳反例
//   Step 4. 全部相同 → 回傳等價
//
// 對齊規則：input / output 均依位置對齊，不依名稱。

#include "circuit.h"

// ============================================================
// 1. 資料結構
// ============================================================

// 讓兩電路輸出不同的輸入組合
struct CounterExample {
    std::vector<bool> inputValues;  // 輸入向量，依 c1.getInputs() 的順序
    std::vector<bool> outputs1;     // c1 各 output 的值，依 c1.getOutputs() 順序
    std::vector<bool> outputs2;     // c2 各 output 的值，依 c2.getOutputs() 順序
};

struct EquivalenceResult {
    bool equivalent;          // true = 兩電路對所有輸入輸出相同
    bool hasCounterExample;   // false 表示因前置條件錯誤提前回傳
    CounterExample counterExample;  // 只在 equivalent=false && hasCounterExample=true 時有意義
};

// ============================================================
// 2. checkEquivalence()
// ============================================================

inline EquivalenceResult checkEquivalence(Circuit& c1, Circuit& c2) {
    // ── 驗證 input / output 數量相符 ─────────────────────────
    if (c1.getInputCount() != c2.getInputCount()) {
        std::cerr << "錯誤：兩電路的輸入數量不同（"
                  << c1.getInputCount() << " vs " << c2.getInputCount() << "）\n";
        return {false, false, {}};
    }
    if (c1.getOutputs().size() != c2.getOutputs().size()) {
        std::cerr << "錯誤：兩電路的輸出數量不同（"
                  << c1.getOutputs().size() << " vs " << c2.getOutputs().size() << "）\n";
        return {false, false, {}};
    }

    // ── 拓撲排序（確認無迴圈）────────────────────────────────
    if (!c1.topologicalSort()) {
        std::cerr << "錯誤：c1 包含迴圈，無法進行等價性檢查\n";
        return {false, false, {}};
    }
    if (!c2.topologicalSort()) {
        std::cerr << "錯誤：c2 包含迴圈，無法進行等價性檢查\n";
        return {false, false, {}};
    }

    const int n     = c1.getInputCount();
    const int total = 1 << n;
    const auto& outs1 = c1.getOutputs();
    const auto& outs2 = c2.getOutputs();
    const int numOuts = (int)outs1.size();

    // ── 窮舉所有輸入組合 ─────────────────────────────────────
    for (int combo = 0; combo < total; combo++) {
        // combo bit i（LSB=0）→ inputValues[n-1-i]，與 generateTruthTable 一致
        std::vector<bool> inputValues(n);
        for (int i = 0; i < n; i++)
            inputValues[n - 1 - i] = (combo >> i) & 1;

        c1.simulate(inputValues);
        c2.simulate(inputValues);

        // 比對所有 output（依位置）
        for (int oi = 0; oi < numOuts; oi++) {
            if (c1.getSignal(outs1[oi]) != c2.getSignal(outs2[oi])) {
                // 找到反例，收集完整輸出後回傳
                CounterExample ce;
                ce.inputValues = inputValues;
                ce.outputs1.reserve(numOuts);
                ce.outputs2.reserve(numOuts);
                for (int k = 0; k < numOuts; k++) {
                    ce.outputs1.push_back(c1.getSignal(outs1[k]));
                    ce.outputs2.push_back(c2.getSignal(outs2[k]));
                }
                return {false, true, ce};
            }
        }
    }

    return {true, false, {}};
}

// ============================================================
// 3. printEquivalenceResult()
// ============================================================

inline void printEquivalenceResult(const EquivalenceResult& result,
                                    const Circuit& c1,
                                    const Circuit& c2) {
    std::cout << "\n=== 電路等價性檢查結果 ===\n";
    if (result.equivalent) {
        std::cout << "結果：✓ 等價（所有輸入組合下輸出完全相同）\n";
        return;
    }
    if (!result.hasCounterExample) {
        std::cout << "結果：✗ 無法判斷（前置條件不符，請查看錯誤訊息）\n";
        return;
    }

    std::cout << "結果：✗ 不等價\n";
    std::cout << "反例輸入：";
    const auto& ins  = c1.getInputs();
    const auto& ce   = result.counterExample;
    for (int i = 0; i < (int)ins.size(); i++) {
        if (i > 0) std::cout << ", ";
        std::cout << ins[i] << "=" << (ce.inputValues[i] ? 1 : 0);
    }
    std::cout << "\n";

    const auto& outs1 = c1.getOutputs();
    const auto& outs2 = c2.getOutputs();
    std::cout << "c1 輸出：";
    for (int i = 0; i < (int)outs1.size(); i++) {
        if (i > 0) std::cout << ", ";
        std::cout << outs1[i] << "=" << (ce.outputs1[i] ? 1 : 0);
    }
    std::cout << "\n";
    std::cout << "c2 輸出：";
    for (int i = 0; i < (int)outs2.size(); i++) {
        if (i > 0) std::cout << ", ";
        std::cout << outs2[i] << "=" << (ce.outputs2[i] ? 1 : 0);
    }
    std::cout << "\n";
}
