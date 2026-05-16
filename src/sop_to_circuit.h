
#pragma once
// sop_to_circuit.h ── SOP/minterm → Circuit 轉換器 + .tt 檔解析（header-only）
//
// 閘命名規則（sopToCircuit）：
//   NOT 閘：NOT_<varName>，輸出訊號：w_not_<varName>（同一反變數只建一個）
//   AND 閘：AND_t<i>（0-based），輸出訊號：w_and_t<i>
//   OR  閘：OR_out，輸出訊號：<outputName>
//   單一乘積項時不建 OR 閘，AND 輸出直接命名為 <outputName>
//
// 閘命名規則（mintermToCircuit）：
//   NOT 閘：NOT_<varName>，輸出訊號：w_not_<varName>（同一反變數只建一個）
//   AND 閘：AND_m<minterm>，輸出訊號：w_and_m<minterm>
//   OR  閘：OR_out，輸出訊號：<outputName>
//   單一 minterm 時不建 OR 閘
//
// 恆 0 / 恆 1 → 回傳 std::nullopt（呼叫端負責處理）
// Circuit 為 move-only；回傳與接收均走 move，不觸發複製

#include "qm_minimizer.h"
#include "circuit.h"
#include <optional>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <unordered_set>
#include <algorithm>

// ============================================================
// 1. .tt 檔解析
// ============================================================
//
// 格式：
//   # 註解（整行忽略）
//   VARS   A B C          # 左→右 = MSB→LSB
//   MINTERMS 1 3 5 7      # 空白或逗號分隔
//   DONTCARES 0           # 可省略
//
// 回傳 false 表示解析失敗（已印 cerr）
inline bool parseTruthTableFile(const std::string& path,
                                 int& numVars,
                                 std::vector<int>& minterms,
                                 std::vector<int>& dontCares,
                                 std::vector<std::string>& varNames)
{
    numVars = 0;
    minterms.clear();
    dontCares.clear();
    varNames.clear();

    std::ifstream file(path);
    if (!file.is_open()) {
        std::cerr << "錯誤：無法開啟檔案 " << path << "\n";
        return false;
    }

    bool hasVars      = false;
    bool hasMinterms  = false;

    std::string line;
    int lineNo = 0;
    while (std::getline(file, line)) {
        ++lineNo;
        // 去掉 \r（Windows 換行）
        if (!line.empty() && line.back() == '\r') line.pop_back();
        // 忽略空行與註解
        if (line.empty() || line[0] == '#') continue;

        std::istringstream iss(line);
        std::string keyword;
        iss >> keyword;
        // 轉大寫
        for (char& c : keyword) c = (char)std::toupper((unsigned char)c);

        if (keyword == "VARS") {
            std::string name;
            while (iss >> name) varNames.push_back(name);
            if (varNames.empty()) {
                std::cerr << "錯誤（行 " << lineNo << "）：VARS 後面沒有變數名稱\n";
                return false;
            }
            numVars = (int)varNames.size();
            if (numVars < 1 || numVars > 20) {
                std::cerr << "錯誤（行 " << lineNo << "）：變數數量 " << numVars
                          << " 超出合法範圍 [1, 20]\n";
                return false;
            }
            hasVars = true;

        } else if (keyword == "MINTERMS") {
            std::string token;
            while (iss >> token) {
                // 允許逗號分隔：移除尾部逗號
                if (!token.empty() && token.back() == ',') token.pop_back();
                if (token.empty()) continue;
                try {
                    minterms.push_back(std::stoi(token));
                } catch (...) {
                    std::cerr << "錯誤（行 " << lineNo << "）：無效的 minterm 值 '"
                              << token << "'\n";
                    return false;
                }
            }
            hasMinterms = true;

        } else if (keyword == "DONTCARES") {
            std::string token;
            while (iss >> token) {
                if (!token.empty() && token.back() == ',') token.pop_back();
                if (token.empty()) continue;
                try {
                    dontCares.push_back(std::stoi(token));
                } catch (...) {
                    std::cerr << "錯誤（行 " << lineNo << "）：無效的 don't care 值 '"
                              << token << "'\n";
                    return false;
                }
            }

        } else {
            std::cerr << "錯誤（行 " << lineNo << "）：未知的關鍵字 '" << keyword << "'\n";
            return false;
        }
    }

    if (!hasVars) {
        std::cerr << "錯誤：缺少 VARS 行\n";
        return false;
    }
    if (!hasMinterms) {
        std::cerr << "錯誤：缺少 MINTERMS 行\n";
        return false;
    }

    // 驗證範圍與重疊（委由 minimize() 再次驗證，這裡先做快速檢查）
    const int total = 1 << numVars;
    std::unordered_set<int> mintermSet(minterms.begin(), minterms.end());
    for (int m : minterms) {
        if (m < 0 || m >= total) {
            std::cerr << "錯誤：minterm " << m
                      << " 超出範圍 [0, " << total << ")\n";
            return false;
        }
    }
    for (int dc : dontCares) {
        if (dc < 0 || dc >= total) {
            std::cerr << "錯誤：don't care " << dc
                      << " 超出範圍 [0, " << total << ")\n";
            return false;
        }
        if (mintermSet.count(dc)) {
            std::cerr << "錯誤：" << dc
                      << " 同時出現在 MINTERMS 和 DONTCARES\n";
            return false;
        }
    }

    return true;
}

// ============================================================
// 2. sopToCircuit：QMResult（最小化 SOP）→ Circuit
// ============================================================
inline std::optional<Circuit> sopToCircuit(const QMResult& result,
                                            const std::string& outputName = "F")
{
    // 恆 0：terms 為空
    if (result.terms.empty()) return std::nullopt;
    // 恆 1：唯一乘積項的 care == 0
    if (result.terms.size() == 1 && result.terms[0].care == 0) return std::nullopt;

    Circuit c;
    for (const auto& v : result.varNames) c.addInput(v);
    c.addOutput(outputName);

    const int numVars  = result.numVars;
    const int numTerms = (int)result.terms.size();

    // ── 收集需要互補的變數，統一建 NOT 閘（共用）────────────
    // bit i（varNames[i]）若在任何乘積項中以負形式出現 → 需要 NOT 閘
    std::unordered_set<int> needNot;
    for (const auto& pt : result.terms) {
        for (int i = 0; i < numVars; i++) {
            if (((pt.care >> i) & 1) && !((pt.polarity >> i) & 1))
                needNot.insert(i);
        }
    }
    // 按變數索引排序，保證閘的建立順序確定
    std::vector<int> notVars(needNot.begin(), needNot.end());
    std::sort(notVars.begin(), notVars.end());

    for (int i : notVars) {
        const std::string& vname = result.varNames[i];
        c.addGate("NOT",
                  "NOT_" + vname,
                  {vname},
                  "w_not_" + vname);
    }

    // ── 為每個乘積項建 AND 閘 ────────────────────────────────
    // andOutputs[i] = 第 i 個乘積項的輸出訊號名稱
    std::vector<std::string> andOutputs;
    andOutputs.reserve(numTerms);

    for (int t = 0; t < numTerms; t++) {
        const auto& pt = result.terms[t];
        std::vector<std::string> inputs;

        for (int i = 0; i < numVars; i++) {
            if (!((pt.care >> i) & 1)) continue;
            const std::string& vname = result.varNames[i];
            if ((pt.polarity >> i) & 1)
                inputs.push_back(vname);               // 正形式
            else
                inputs.push_back("w_not_" + vname);   // 負形式（已建 NOT 閘）
        }

        // 決定此 AND 閘的輸出訊號名稱
        std::string andOut;
        if (numTerms == 1) {
            andOut = outputName;  // 單一乘積項：AND 輸出直接為 outputName
        } else {
            andOut = "w_and_t" + std::to_string(t);
        }
        andOutputs.push_back(andOut);

        if (inputs.size() == 1) {
            // 單變數乘積項：退化成 wire，用 NOT(NOT(x)) 模擬 BUF？
            // 不行——改用 OR 閘只接一個輸入（OR 單輸入等於 identity）
            // 實際上最簡單：直接用訊號名稱，不建閘，把 andOut 對應到 inputs[0]
            // 但 Circuit 不支援純 wire。退而求其次：建一個 OR 閘（單輸入）
            // OR.compute({x}) = x，語意正確。
            c.addGate("OR", "AND_t" + std::to_string(t), inputs, andOut);
        } else {
            c.addGate("AND", "AND_t" + std::to_string(t), inputs, andOut);
        }
    }

    // ── 多乘積項：建 OR 閘 ────────────────────────────────────
    if (numTerms > 1) {
        c.addGate("OR", "OR_out", andOutputs, outputName);
    }

    return std::optional<Circuit>(std::move(c));
}

// ============================================================
// 3. mintermToCircuit：minterm 直接展開 → Circuit（不經 QM）
// ============================================================
inline std::optional<Circuit> mintermToCircuit(int numVars,
                                                const std::vector<int>& minterms,
                                                const std::vector<std::string>& varNames,
                                                const std::string& outputName = "F")
{
    const int total = 1 << numVars;

    // 恆 0：空 minterms
    if (minterms.empty()) return std::nullopt;
    // 恆 1：全部 minterms
    if ((int)minterms.size() == total) return std::nullopt;

    Circuit c;
    for (const auto& v : varNames) c.addInput(v);
    c.addOutput(outputName);

    const int numTerms = (int)minterms.size();

    // ── 收集需要互補的變數 ────────────────────────────────────
    // minterm m 的 bit j（j=0=LSB）對應 varNames[numVars-1-j]
    // 若 bit j == 0 → 變數 varNames[numVars-1-j] 以負形式出現
    std::unordered_set<int> needNot; // varNames 索引
    for (int m : minterms) {
        for (int j = 0; j < numVars; j++) {
            if (!((m >> j) & 1)) {
                int varIdx = numVars - 1 - j;
                needNot.insert(varIdx);
            }
        }
    }
    std::vector<int> notVars(needNot.begin(), needNot.end());
    std::sort(notVars.begin(), notVars.end());

    for (int i : notVars) {
        const std::string& vname = varNames[i];
        c.addGate("NOT",
                  "NOT_" + vname,
                  {vname},
                  "w_not_" + vname);
    }

    // ── 為每個 minterm 建 AND 閘 ──────────────────────────────
    std::vector<std::string> andOutputs;
    andOutputs.reserve(numTerms);

    for (int idx = 0; idx < numTerms; idx++) {
        int m = minterms[idx];
        std::vector<std::string> inputs;

        for (int j = numVars - 1; j >= 0; j--) {
            // j=numVars-1 → varNames[0]（MSB）
            int varIdx = numVars - 1 - j;
            const std::string& vname = varNames[varIdx];
            if ((m >> j) & 1)
                inputs.push_back(vname);
            else
                inputs.push_back("w_not_" + vname);
        }

        std::string andOut;
        if (numTerms == 1) {
            andOut = outputName;
        } else {
            andOut = "w_and_m" + std::to_string(m);
        }
        andOutputs.push_back(andOut);

        if (inputs.size() == 1) {
            c.addGate("OR", "AND_m" + std::to_string(m), inputs, andOut);
        } else {
            c.addGate("AND", "AND_m" + std::to_string(m), inputs, andOut);
        }
    }

    // ── 多 minterm：建 OR 閘 ──────────────────────────────────
    if (numTerms > 1) {
        c.addGate("OR", "OR_out", andOutputs, outputName);
    }

    return std::optional<Circuit>(std::move(c));
}
