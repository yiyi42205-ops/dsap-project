
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
//   Step 6. Petrick's method（預設）或貪心覆蓋剩餘
//             ── Petrick's：展開乘積式，枚舉所有可行覆蓋，依三層規則選最佳
//             ── 貪心：每次選能多蓋最多未蓋 minterm 的 PI（保留供對照用）
//   Step 7. 轉換為 ProductTerm 輸出
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

// Prime Implicant 記錄（QM 中間步驟，供外部詳解使用）
struct PIRecord {
    std::string      term;          // 可讀式，如 "AB'" 或 "1"（恆真項）
    std::vector<int> minterms;      // 此 PI 覆蓋的 care minterms（已排序）
    int              literalCount = 0; // 此 PI 的 literal 個數（= numVars - popcount(mask)）
    bool essential = false;         // Step 5 選出：某 minterm 唯一由此 PI 覆蓋
    bool selected  = false;         // 最終入選解（essential 或 Step 6 選出）
};

// 最小化結果
struct QMResult {
    int numVars = 0;
    std::vector<std::string> varNames;  // 長度 = numVars
    std::vector<ProductTerm> terms;     // SOP 各乘積項
    // terms 為空          → 函數恆 0
    // terms 含 care=0 項  → 函數恆 1（此時 terms.size()==1）

    // 中間結果（QM 演算法步驟資訊，供外部詳解使用）
    std::vector<int>                         inputMinterms; // 輸入的 minterms
    std::vector<PIRecord>                    piRecords;     // 所有 PI 及選取狀態
    std::unordered_map<int, std::vector<int>> mintermToPIs; // minterm → PI index 清單
};

// ============================================================
// 2. Petrick's method 覆蓋選取輔助函式（內部使用）
// ============================================================
//
// 從仍未覆蓋的 minterms 出發，展開乘積式枚舉所有可行覆蓋，
// 依以下三層規則選出唯一確定的最佳解：
//
//   規則 1：PI 個數最少（保證項數最少 = 保證最簡）
//   規則 2：平手時，選中 PIs 的 literal 總數最少
//            （每個 PI 的 literal 數 = numVars - popcount(implicant.mask)，
//              已存入 PIRecord::literalCount）
//   規則 3：仍平手時，所有選中 PIs 的覆蓋 minterms 合集（排序後）字典序最小
//
// 此三層規則保證：對同一輸入，每次回傳完全相同的一組 PI 索引。
static inline std::vector<int> qm_petricksCover(
    const std::vector<int>&                          uncoveredList,
    const std::unordered_map<int, std::vector<int>>& mintermToPIs,
    const std::vector<bool>&                         alreadySelected,
    const std::vector<PIRecord>&                     piRecords)
{
    if (uncoveredList.empty()) return {};

    // 排序確保展開順序確定（相同輸入 → 相同輸出）
    std::vector<int> ordered = uncoveredList;
    std::sort(ordered.begin(), ordered.end());

    // 每個 uncovered minterm → 可用的（尚未選中的）PI 清單
    std::vector<std::vector<int>> factors;
    factors.reserve(ordered.size());
    for (int m : ordered) {
        std::vector<int> factor;
        auto it = mintermToPIs.find(m);
        if (it != mintermToPIs.end())
            for (int pi : it->second)
                if (!alreadySelected[pi]) factor.push_back(pi);
        std::sort(factor.begin(), factor.end()); // 確定順序
        factors.push_back(std::move(factor));
    }

    // 展開乘積式（Product of Sums → Sum of Products）
    // 每個「乘積項」（Cover）= 一組 PI 索引（sorted vector，方便集合比較）
    using Cover = std::vector<int>;

    // 初始：以第一個 factor 的每個 PI 各自成一個 Cover
    std::vector<Cover> sop;
    sop.reserve(factors[0].size());
    for (int pi : factors[0]) sop.push_back({pi});

    // 逐一乘上後續 factor
    for (int fi = 1; fi < (int)factors.size(); fi++) {
        std::vector<Cover> newSop;
        newSop.reserve(sop.size() * factors[fi].size());
        for (const auto& cover : sop) {
            for (int pi : factors[fi]) {
                Cover merged = cover;
                // 維持 sorted：找插入位置
                auto pos = std::lower_bound(merged.begin(), merged.end(), pi);
                if (pos == merged.end() || *pos != pi)
                    merged.insert(pos, pi);
                newSop.push_back(std::move(merged));
            }
        }

        // 去重（sorted vector 可直接比較）
        std::sort(newSop.begin(), newSop.end());
        newSop.erase(std::unique(newSop.begin(), newSop.end()), newSop.end());

        // Boolean 吸收律：若 A ⊆ B 則刪 B（A + AB = A）
        // 先按大小排序（小的在前），使吸收掃描更高效
        std::sort(newSop.begin(), newSop.end(),
            [](const Cover& a, const Cover& b) {
                return a.size() != b.size() ? a.size() < b.size() : a < b;
            });
        std::vector<Cover> absorbed;
        absorbed.reserve(newSop.size());
        for (auto& cov : newSop) {
            bool dominated = false;
            for (const auto& small : absorbed) {
                if (small.size() > cov.size()) break; // 後面的只會更大
                if (std::includes(cov.begin(), cov.end(),
                                   small.begin(), small.end())) {
                    dominated = true; break;
                }
            }
            if (!dominated) absorbed.push_back(std::move(cov));
        }
        sop = std::move(absorbed);
    }

    if (sop.empty()) return {};

    // ── 三層規則選最佳 ───────────────────────────────────────────
    // 輔助：計算一組 Cover 的 minterms 合集（已排序）
    auto coverMinset = [&](const Cover& cov) -> std::vector<int> {
        std::vector<int> ms;
        for (int pi : cov)
            for (int m : piRecords[pi].minterms)
                ms.push_back(m);
        std::sort(ms.begin(), ms.end());
        ms.erase(std::unique(ms.begin(), ms.end()), ms.end());
        return ms;
    };

    int bestIdx  = 0;
    int bestSize = (int)sop[0].size();
    int bestLits = 0;
    for (int pi : sop[0]) bestLits += piRecords[pi].literalCount;
    std::vector<int> bestMinset = coverMinset(sop[0]);

    for (int ci = 1; ci < (int)sop.size(); ci++) {
        const Cover& cov = sop[ci];
        int sz = (int)cov.size();
        if (sz > bestSize) continue;      // Rule 1 劣

        int lits = 0;
        for (int pi : cov) lits += piRecords[pi].literalCount;

        if (sz == bestSize) {
            if (lits > bestLits) continue; // Rule 2 劣
            if (lits == bestLits) {
                // Rule 3：比較 minterm 合集字典序
                std::vector<int> minset = coverMinset(cov);
                if (minset >= bestMinset) continue;
                bestMinset = std::move(minset);
            } else {
                bestMinset = coverMinset(cov);
            }
        } else {
            // sz < bestSize：Rule 1 更佳
            bestMinset = coverMinset(cov);
        }
        bestIdx  = ci;
        bestSize = sz;
        bestLits = lits;
    }

    return sop[bestIdx];
}

// ============================================================
// 3. minimizeImpl()：核心實作（供 minimize / minimizeGreedy 共用）
// ============================================================
//
// usePetricks = true  → Step 6 用 Petrick's method（保證最簡，預設）
// usePetricks = false → Step 6 用貪心法（保留供對照用）
static inline QMResult minimizeImpl(
    int numVars,
    const std::vector<int>& minterms,
    const std::vector<int>& dontCares,
    const std::vector<std::string>& varNames,
    bool usePetricks)
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
    while (!current.empty()) {
        std::vector<Implicant> next;
        std::unordered_set<uint64_t> nextSeen;

        const int sz = (int)current.size();
        for (int i = 0; i < sz; i++) {
            for (int j = i + 1; j < sz; j++) {
                if (current[i].mask != current[j].mask) continue;
                uint32_t diff = current[i].value ^ current[j].value;
                if (diff == 0 || (diff & (diff - 1)) != 0) continue;

                current[i].used = current[j].used = true;
                uint32_t nv = current[i].value & ~diff;
                uint32_t nm = current[i].mask  |  diff;
                if (nextSeen.insert(makeKey(nv, nm)).second)
                    next.push_back({nv, nm, false});
            }
        }

        for (const auto& imp : current)
            if (!imp.used) primeImplicants.push_back(imp);

        current = std::move(next);
    }

    // ── Step 3：PI 去重 ──────────────────────────────────────
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
    std::unordered_map<int, std::vector<int>> mintermToPIs;
    for (int m : minterms) mintermToPIs[m] = {};
    for (int i = 0; i < numPIs; i++)
        for (int m : minterms)
            if (covers(primeImplicants[i], m))
                mintermToPIs[m].push_back(i);

    // ── 儲存中間結果：覆蓋表與 PI 記錄 ─────────────────────
    result.inputMinterms = minterms;
    result.mintermToPIs  = mintermToPIs;
    {
        // 把每個 PI 轉成可讀字串並計算 literal count
        auto piToStr = [&](const Implicant& pi) -> std::string {
            std::string s;
            for (int j = 0; j < numVars; j++) {
                if ((pi.mask >> j) & 1) continue; // don't care bit
                int varIdx = numVars - 1 - j;
                s += result.varNames[varIdx];
                if (!((pi.value >> j) & 1)) s += "'";
            }
            return s.empty() ? "1" : s;
        };
        result.piRecords.resize(numPIs);
        for (int i = 0; i < numPIs; i++) {
            result.piRecords[i].term = piToStr(primeImplicants[i]);
            // literal count = 固定位個數 = numVars - popcount(mask)
            int popMask = 0;
            uint32_t m = primeImplicants[i].mask;
            while (m) { popMask += (m & 1); m >>= 1; }
            result.piRecords[i].literalCount = numVars - popMask;
        }
        // 從覆蓋表反查每個 PI 覆蓋了哪些 minterms
        for (const auto& kv : mintermToPIs)
            for (int piIdx : kv.second)
                result.piRecords[piIdx].minterms.push_back(kv.first);
        for (auto& pir : result.piRecords)
            std::sort(pir.minterms.begin(), pir.minterms.end());
    }

    // ── Step 5：選 Essential PIs ────────────────────────────
    std::unordered_set<int> uncovered(minterms.begin(), minterms.end());
    std::vector<bool> selected(numPIs, false);

    // 選中 PI idx：標記 selected 及 piRecords，並從 uncovered 移除其覆蓋的 minterms
    auto selectPI = [&](int idx, bool ess = false) {
        selected[idx] = true;
        result.piRecords[idx].selected  = true;
        result.piRecords[idx].essential = ess;
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
                selectPI(avail[0], true); // essential PI
                found = true;
                break;
            }
        }
    }

    // ── Step 6：覆蓋剩餘 minterms ────────────────────────────
    if (!uncovered.empty()) {
        if (usePetricks) {
            // ── Petrick's method：保證最簡 ─────────────────
            // 詳細規則見 qm_petricksCover 函式說明
            std::vector<int> uncovList(uncovered.begin(), uncovered.end());
            std::vector<int> chosen = qm_petricksCover(
                uncovList, mintermToPIs, selected, result.piRecords);
            for (int idx : chosen) selectPI(idx);
        } else {
            // ── 貪心法（保留供對照）────────────────────────
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
                if (bestIdx < 0) break;
                selectPI(bestIdx);
            }
        }
    }

    // ── Step 7：轉換為 ProductTerm ───────────────────────────
    // PI 的 bit j（j=0=LSB）對應 varNames[numVars-1-j]
    // ProductTerm 的 bit i（i=0）對應 varNames[i]
    for (int i = 0; i < numPIs; i++) {
        if (!selected[i]) continue;
        const auto& pi = primeImplicants[i];
        ProductTerm pt{0u, 0u};
        for (int j = 0; j < numVars; j++) {
            if ((pi.mask >> j) & 1) continue;
            int varIdx = numVars - 1 - j;
            pt.care |= (1u << varIdx);
            if ((pi.value >> j) & 1)
                pt.polarity |= (1u << varIdx);
        }
        result.terms.push_back(pt);
    }

    return result;
}

// ============================================================
// 4. 公開 API：minimize() 與 minimizeGreedy()
// ============================================================

// minimize()：Petrick's method，保證選出項數最少的最簡解
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
    return minimizeImpl(numVars, minterms, dontCares, varNames, /*usePetricks=*/true);
}

// minimizeGreedy()：貪心覆蓋，保留供「貪心 vs Petrick's」對照使用
// 介面與 minimize() 完全相同，差異僅在 Step 6 的選取策略
inline QMResult minimizeGreedy(int numVars,
                                const std::vector<int>& minterms,
                                const std::vector<int>& dontCares = {},
                                const std::vector<std::string>& varNames = {})
{
    return minimizeImpl(numVars, minterms, dontCares, varNames, /*usePetricks=*/false);
}

// ============================================================
// 5. minimizeCircuitOutput()：從電路 output 自動提取並最小化
// ============================================================
inline QMResult minimizeCircuitOutput(Circuit& circuit,
                                       const std::string& outputSignalName)
{
    const auto& outputs = circuit.getOutputs();
    if (std::find(outputs.begin(), outputs.end(), outputSignalName) == outputs.end()) {
        std::cerr << "錯誤：'" << outputSignalName
                  << "' 不在電路的 output 訊號列表中\n";
        return {};
    }

    if (!circuit.topologicalSort()) {
        std::cerr << "錯誤：電路包含迴圈，無法提取真值表\n";
        return {};
    }

    const int n     = circuit.getInputCount();
    const int total = 1 << n;
    std::vector<int> mintermList;
    mintermList.reserve(total / 2);

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
// 6. 輸出輔助函式
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
            if (!((pt.care >> i) & 1)) continue;
            termStr += result.varNames[i];
            if (!((pt.polarity >> i) & 1)) termStr += "'";
        }
        s += termStr.empty() ? "1" : termStr;
    }
    return s;
}

// 印出 QMResult 到 stdout
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
