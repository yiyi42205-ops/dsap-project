
#pragma once
// qm_minimizer.h ── Quine-McCluskey 邏輯最小化器（header-only）
//
// 演算法：
//   Step 1. 初始化：每個 minterm ∪ don't care 各建一個 Implicant(value=m, mask=0)
//   Step 2. 反覆合併（Quine 階段）：
//             兩個 Implicant 的 mask 相同，且 value 恰好差一個 bit（2 的冪）
//             → 合併為新 Implicant（mask 多一個 bit），標記兩者 used=true
//             直到無新合併為止
//   Step 3. 收集所有 used=false 的 Implicant → Prime Implicants (PIs)
//   Step 4. 建立覆蓋表（只對 minterms，don't cares 不列入）
//   Step 5. 選 Essential PIs（某 minterm 只被一個 PI 覆蓋）；重複直到無新 essential
//   Step 6. 貪心覆蓋剩餘（每次選能多蓋最多未蓋 minterm 的 PI）
//
// 位元對應規則：
//   minterm 的 bit j（j=0=LSB）對應 varNames[numVars-1-j]（最後一個變數）
//   ProductTerm 的 bit i（i=0）對應 varNames[i]（第一個變數）
//   兩者相差一個「位元倒序」轉換，在 Step 7 中完成

#include "circuit.h"
#include <vector>
#include <string>
#include <unordered_set>
#include <unordered_map>
#include <algorithm>
#include <cstdint>

// ============================================================
// 1. 公開資料結構
// ============================================================

// 乘積項（Product Term）— 以兩組位元遮罩表示
// bit i 對應 varNames[i]（i=0 = 第一個變數 = minterm 的 MSB）
struct ProductTerm {
    uint32_t care;      // bit i=1 → 變數 i 出現在此乘積項中
    uint32_t polarity;  // bit i=1 → 正形式（A）；0 → 負形式（A'）
                        // 僅當 care bit i=1 時有意義
};

// 最小化結果
struct QMResult {
    int numVars = 0;
    std::vector<std::string> varNames;  // 長度 = numVars
    std::vector<ProductTerm> terms;     // SOP 各乘積項
    // terms 為空          → 函數恆 0
    // terms 含 care=0 項  → 函數恆 1（此時 terms.size()==1）
};

// ============================================================
// 2. minimize()：從 minterm 列表最小化
// ============================================================
//
// numVars   : 輸入變數個數，合法範圍 [1, 20]
// minterms  : 輸出為 1 的 minterm 編號，合法範圍 [0, 2^numVars)
// dontCares : 輸出為 X 的 minterm 編號（不得與 minterms 重疊）
// varNames  : 可選；長度若非 0 則必須 == numVars；空時自動用 "x0","x1",...
//
// 錯誤時印 std::cerr，回傳空 QMResult（numVars=0, terms 為空）
inline QMResult minimize(int numVars,
                          const std::vector<int>& minterms,
                          const std::vector<int>& dontCares = {},
                          const std::vector<std::string>& varNames = {})
{
    QMResult result;

    // ── 驗證 numVars ─────────────────────────────────────────
    if (numVars <= 0 || numVars > 20) {
        std::cerr << "錯誤：numVars=" << numVars << "，合法範圍 [1, 20]\n";
        return {};
    }
    result.numVars = numVars;

    // ── 設定 varNames ────────────────────────────────────────
    if (!varNames.empty()) {
        if ((int)varNames.size() != numVars) {
            std::cerr << "錯誤：varNames 長度 " << varNames.size()
                      << " 與 numVars=" << numVars << " 不符\n";
            return {};
        }
        result.varNames = varNames;
    } else {
        result.varNames.reserve(numVars);
        for (int i = 0; i < numVars; i++)
            result.varNames.push_back("x" + std::to_string(i));
    }

    const int total = 1 << numVars;

    // ── 驗證 minterms / dontCares ────────────────────────────
    std::unordered_set<int> mintermSet(minterms.begin(), minterms.end());
    std::unordered_set<int> dcSet    (dontCares.begin(), dontCares.end());

    for (int m : minterms) {
        if (m < 0 || m >= total) {
            std::cerr << "錯誤：minterm " << m
                      << " 超出範圍 [0, " << total << ")\n";
            return {};
        }
    }
    for (int dc : dontCares) {
        if (dc < 0 || dc >= total) {
            std::cerr << "錯誤：don't care " << dc
                      << " 超出範圍 [0, " << total << ")\n";
            return {};
        }
        if (mintermSet.count(dc)) {
            std::cerr << "錯誤：" << dc
                      << " 同時出現在 minterms 和 dontCares\n";
            return {};
        }
    }

    // ── 邊界：恆 0 ───────────────────────────────────────────
    if (minterms.empty()) return result; // terms 為空 = 恆 0

    // ── 邊界：恆 1 ───────────────────────────────────────────
    if ((int)mintermSet.size() == total) {
        result.terms.push_back({0u, 0u}); // care=0 = 無條件乘積項
        return result;
    }

    // ============================================================
    // 蘊含項（Implicant）— 以 (value, mask) 對表示
    // mask bit=1 表示此位已被合併為 don't care
    // ============================================================
    struct Implicant {
        uint32_t value, mask;
        bool used = false;
    };

    // 輔助 lambda
    auto makeKey = [](uint32_t v, uint32_t m) -> uint64_t {
        return (uint64_t(v) << 32) | uint64_t(m);
    };

    // 判斷 implicant 是否覆蓋 minterm m：
    //   固定位（mask=0 的 bit）必須與 value 相符
    //   don't care 位（mask=1 的 bit）不限
    //   高位（>= numVars）均為 0，不影響結果
    auto covers = [](const Implicant& imp, int m) -> bool {
        uint32_t fixed = ~imp.mask;
        return (uint32_t(m) & fixed) == (imp.value & fixed);
    };

    // ── Step 1：建立初始蘊含項（minterms ∪ don't cares）──────
    std::vector<Implicant> current;
    {
        std::unordered_set<uint64_t> seen;
        for (int m : minterms)
            if (seen.insert(makeKey(uint32_t(m), 0u)).second)
                current.push_back({uint32_t(m), 0u, false});
        for (int dc : dontCares)
            if (seen.insert(makeKey(uint32_t(dc), 0u)).second)
                current.push_back({uint32_t(dc), 0u, false});
    }

    std::vector<Implicant> primeImplicants;

    // ── Step 2：反覆合併 ─────────────────────────────────────
    // 每輪對 current 中所有對做檢查：
    //   mask 相同 且 value 差恰好一個 bit（diff 是 2 的冪）→ 可合併
    // 合併結果收入 next（去重）；本輪未被使用的 → prime implicant
    while (!current.empty()) {
        std::vector<Implicant> next;
        std::unordered_set<uint64_t> nextSeen;

        const int sz = (int)current.size();
        for (int i = 0; i < sz; i++) {
            for (int j = i + 1; j < sz; j++) {
                if (current[i].mask != current[j].mask) continue;
                uint32_t diff = current[i].value ^ current[j].value;
                // diff 必須是 2 的冪（恰好一個 bit 不同）
                if (diff == 0 || (diff & (diff - 1)) != 0) continue;

                current[i].used = current[j].used = true;
                uint32_t nv = current[i].value & ~diff; // 清除該 bit
                uint32_t nm = current[i].mask  |  diff; // 該 bit 加入 mask
                if (nextSeen.insert(makeKey(nv, nm)).second)
                    next.push_back({nv, nm, false});
            }
        }

        // 本輪未被合併的蘊含項 → 候選 prime implicant
        for (const auto& imp : current)
            if (!imp.used) primeImplicants.push_back(imp);

        current = std::move(next);
    }

    // ── Step 3：PI 去重 ──────────────────────────────────────
    // 不同合併路徑可能產生相同的 (value, mask) 對，統一去重
    {
        std::unordered_set<uint64_t> piSeen;
        std::vector<Implicant> dedup;
        dedup.reserve(primeImplicants.size());
        for (const auto& pi : primeImplicants)
            if (piSeen.insert(makeKey(pi.value, pi.mask)).second)
                dedup.push_back(pi);
        primeImplicants = std::move(dedup);
    }

    const int numPIs = (int)primeImplicants.size();

    // ── Step 4：建覆蓋表（只含 minterms，不含 don't cares）───
    // mintermToPIs[m] = 覆蓋 minterm m 的 PI 索引列表
    std::unordered_map<int, std::vector<int>> mintermToPIs;
    for (int m : minterms) mintermToPIs[m] = {};
    for (int i = 0; i < numPIs; i++)
        for (int m : minterms)
            if (covers(primeImplicants[i], m))
                mintermToPIs[m].push_back(i);

    // ── Step 5：選 Essential PIs ────────────────────────────
    std::unordered_set<int> uncovered(minterms.begin(), minterms.end());
    std::vector<bool> selected(numPIs, false);

    // 選中 PI idx：標記 selected，並從 uncovered 移除其覆蓋的 minterms
    auto selectPI = [&](int idx) {
        selected[idx] = true;
        for (int m : minterms)
            if (covers(primeImplicants[idx], m))
                uncovered.erase(m);
    };

    // 重複掃描：找到某 uncovered minterm 只被一個未選 PI 覆蓋 → essential
    bool found = true;
    while (found && !uncovered.empty()) {
        found = false;
        for (int m : minterms) {
            if (!uncovered.count(m)) continue;
            std::vector<int> avail;
            for (int pi : mintermToPIs[m])
                if (!selected[pi]) avail.push_back(pi);
            if (avail.size() == 1) {
                selectPI(avail[0]);
                found = true;
                break; // uncovered 已改變，重新從頭掃描
            }
        }
    }

    // ── Step 6：貪心覆蓋剩餘 ────────────────────────────────
    // 每次選能覆蓋最多 uncovered minterms 的未選 PI
    // 平手時選索引最小的（保證結果確定性）
    while (!uncovered.empty()) {
        int bestIdx = -1, bestCnt = 0;
        for (int i = 0; i < numPIs; i++) {
            if (selected[i]) continue;
            int cnt = 0;
            for (int m : uncovered)
                if (covers(primeImplicants[i], m)) cnt++;
            if (cnt > bestCnt) { bestCnt = cnt; bestIdx = i; }
        }
        if (bestIdx < 0) break; // 防護，理論上不應發生
        selectPI(bestIdx);
    }

    // ── Step 7：轉換為 ProductTerm ───────────────────────────
    // PI 的 bit j（j=0=LSB）對應 varNames[numVars-1-j]
    // ProductTerm 的 bit i（i=0）對應 varNames[i]
    // 轉換：PI bit j → varIdx = numVars-1-j → ProductTerm bit varIdx
    for (int i = 0; i < numPIs; i++) {
        if (!selected[i]) continue;
        const auto& pi = primeImplicants[i];
        ProductTerm pt{0u, 0u};
        for (int j = 0; j < numVars; j++) {
            if ((pi.mask >> j) & 1) continue; // don't care bit，略過
            int varIdx = numVars - 1 - j;
            pt.care |= (1u << varIdx);
            if ((pi.value >> j) & 1)
                pt.polarity |= (1u << varIdx); // 正形式（uncomplemented）
        }
        result.terms.push_back(pt);
    }

    return result;
}

// ============================================================
// 3. minimizeCircuitOutput()：從電路 output 自動提取並最小化
// ============================================================
//
// circuit 若尚未完成拓撲排序，內部自動呼叫一次（不影響呼叫端其他狀態）
// outputSignalName 必須在 circuit.getOutputs() 中
// varNames 自動取 circuit.getInputs()（順序即 MSB→LSB，與 generateTruthTable 一致）
inline QMResult minimizeCircuitOutput(Circuit& circuit,
                                       const std::string& outputSignalName)
{
    // 驗證 output 名稱
    const auto& outputs = circuit.getOutputs();
    if (std::find(outputs.begin(), outputs.end(), outputSignalName) == outputs.end()) {
        std::cerr << "錯誤：'" << outputSignalName
                  << "' 不在電路的 output 訊號列表中\n";
        return {};
    }

    // 確保拓撲排序完成
    if (!circuit.topologicalSort()) {
        std::cerr << "錯誤：電路包含迴圈，無法提取真值表\n";
        return {};
    }

    const int n     = circuit.getInputCount();
    const int total = 1 << n;
    std::vector<int> mintermList;
    mintermList.reserve(total / 2);

    // 枚舉所有 2^n 種輸入，收集輸出為 1 的 minterm
    // bit 順序與 generateTruthTable 一致：combo bit i → inputValues[n-1-i]
    for (int combo = 0; combo < total; combo++) {
        std::vector<bool> inputValues(n);
        for (int i = 0; i < n; i++)
            inputValues[n - 1 - i] = (combo >> i) & 1;
        circuit.simulate(inputValues);
        if (circuit.getSignal(outputSignalName))
            mintermList.push_back(combo);
    }

    return minimize(n, mintermList, {}, circuit.getInputs());
}

// ============================================================
// 4. 輸出輔助函式
// ============================================================

// 將 QMResult 轉為可讀字串，例如 "A'B + BC'" / "0" / "1"
inline std::string qmResultToString(const QMResult& result) {
    if (result.terms.empty()) return "0";
    std::string s;
    for (int t = 0; t < (int)result.terms.size(); t++) {
        if (t > 0) s += " + ";
        const auto& pt = result.terms[t];
        std::string termStr;
        for (int i = 0; i < result.numVars; i++) {
            if (!((pt.care >> i) & 1)) continue;       // 此變數不出現
            termStr += result.varNames[i];
            if (!((pt.polarity >> i) & 1)) termStr += "'"; // 負形式加 '
        }
        s += termStr.empty() ? "1" : termStr; // care=0 → 恆真乘積項
    }
    return s;
}

// 印出 QMResult 到 stdout（格式與 printCriticalPaths 風格一致）
inline void printQMResult(const QMResult& result) {
    std::cout << "\n=== Quine-McCluskey 最小化結果 ===\n";
    std::cout << "變數（" << result.numVars << " 個）：";
    for (int i = 0; i < result.numVars; i++) {
        if (i > 0) std::cout << ", ";
        std::cout << result.varNames[i];
    }
    std::cout << "\n";
    std::cout << "最小 SOP：" << qmResultToString(result) << "\n";
    std::cout << "乘積項數：" << result.terms.size() << "\n";
    if (!result.terms.empty() && !(result.terms.size() == 1 && result.terms[0].care == 0)) {
        std::cout << "各乘積項：\n";
        for (int t = 0; t < (int)result.terms.size(); t++) {
            const auto& pt = result.terms[t];
            std::cout << "  #" << (t + 1) << ": ";
            bool anyLit = false;
            for (int i = 0; i < result.numVars; i++) {
                if (!((pt.care >> i) & 1)) continue;
                std::cout << result.varNames[i];
                if (!((pt.polarity >> i) & 1)) std::cout << "'";
                anyLit = true;
            }
            if (!anyLit) std::cout << "1（恆真）";
            std::cout << "\n";
        }
    }
}
