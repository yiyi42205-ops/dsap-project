#include "json_export.h"
#include "sop_to_circuit.h"
#include "qm_minimizer.h"
#include <fstream>
#include <sstream>
#include <unordered_map>
#include <unordered_set>
#include <filesystem>
#include <cctype>
#include <optional>

// ============================================================
// 內部 helpers
// ============================================================

// escape JSON 字串中的 " 和 \（UTF-8 內容不需額外處理）
static std::string jsonEscape(const std::string& s) {
    std::string r;
    r.reserve(s.size());
    for (char c : s) {
        if      (c == '"')  r += "\\\"";
        else if (c == '\\') r += "\\\\";
        else                r += c;
    }
    return r;
}

static std::string toUpper(const std::string& s) {
    std::string r = s;
    for (char& c : r) c = (char)std::toupper((unsigned char)c);
    return r;
}

// 將訊號名或閘名轉換成 JSON node id：
//   primary input  → "IN_"  + uppercase(name)
//   output signal  → "OUT_" + uppercase(name)
//   gate name      → name（原樣）
static std::string toNodeId(const std::string& name,
                             const std::unordered_set<std::string>& inputSet,
                             const std::unordered_set<std::string>& outputSet) {
    if (inputSet.count(name))  return "IN_"  + toUpper(name);
    if (outputSet.count(name)) return "OUT_" + toUpper(name);
    return name;
}

// ============================================================
// 公開函式實作
// ============================================================

std::string circuitNameFromPath(const std::string& path) {
    size_t sep = path.find_last_of("/\\");
    std::string base = (sep == std::string::npos) ? path : path.substr(sep + 1);
    size_t dot = base.rfind('.');
    if (dot != std::string::npos) base = base.substr(0, dot);
    return base;
}

bool exportCircuitToJson(Circuit& circuit,
                         const std::string& circuitName,
                         const std::string& outputPath,
                         int topK) {
    // ── 執行拓撲排序（BFS + DFS）────────────────────────────
    if (!circuit.topologicalSortBFS()) {
        std::cerr << "錯誤：電路包含迴圈，無法匯出 JSON\n";
        return false;
    }
    if (!circuit.topologicalSortDFS()) {
        std::cerr << "錯誤：電路包含迴圈，無法匯出 JSON\n";
        return false;
    }
    circuit.topologicalSort(); // 設定 executionOrder（BFS），供 simulate() 使用

    const auto& inputSignals  = circuit.getInputs();
    const auto& outputSignals = circuit.getOutputs();
    std::vector<Gate*> allGates = circuit.getAllGates();

    std::unordered_set<std::string> inputSet(inputSignals.begin(),  inputSignals.end());
    std::unordered_set<std::string> outputSet(outputSignals.begin(), outputSignals.end());

    // signalSource: output wire name → Gate* 產生該訊號的閘
    std::unordered_map<std::string, Gate*> signalSource;
    for (Gate* g : allGates) signalSource[g->outputName] = g;

    // ============================================================
    // 建構 JSON 字串
    // ============================================================
    std::ostringstream j;
    j << "{\n";

    // ── circuit_name ─────────────────────────────────────────
    j << "  \"circuit_name\": \"" << jsonEscape(circuitName) << "\",\n";

    // ── nodes ────────────────────────────────────────────────
    // 順序：INPUT nodes → gate nodes → OUTPUT nodes
    j << "  \"nodes\": [\n";
    bool first = true;

    auto sep = [&]() -> std::ostringstream& {
        if (!first) j << ",\n";
        first = false;
        return j;
    };

    for (const auto& sig : inputSignals) {
        sep() << "    {\"id\": \"" << jsonEscape("IN_" + toUpper(sig))
              << "\", \"type\": \"INPUT\", \"delay\": 0}";
    }
    for (Gate* g : allGates) {
        sep() << "    {\"id\": \"" << jsonEscape(g->name)
              << "\", \"type\": \"" << jsonEscape(g->type)
              << "\", \"delay\": " << g->delay << "}";
    }
    for (const auto& sig : outputSignals) {
        sep() << "    {\"id\": \"" << jsonEscape("OUT_" + toUpper(sig))
              << "\", \"type\": \"OUTPUT\", \"delay\": 0}";
    }
    j << "\n  ],\n";

    // ── edges ────────────────────────────────────────────────
    // gate 的每條輸入邊：(source node) → (gate)
    // 加上每個 output signal：(producing gate) → (OUT_X)
    struct Edge { std::string from, to; };
    std::vector<Edge> edges;
    std::unordered_set<std::string> edgeSeen;

    auto addEdge = [&](const std::string& from, const std::string& to) {
        std::string key = from + '\x01' + to; // 用不會出現在名稱中的分隔符
        if (edgeSeen.insert(key).second)
            edges.push_back({from, to});
    };

    for (Gate* g : allGates) {
        for (const auto& inp : g->inputNames) {
            if (inputSet.count(inp)) {
                addEdge("IN_" + toUpper(inp), g->name);
            } else {
                auto it = signalSource.find(inp);
                if (it != signalSource.end())
                    addEdge(it->second->name, g->name);
            }
        }
    }
    for (const auto& sig : outputSignals) {
        auto it = signalSource.find(sig);
        if (it != signalSource.end())
            addEdge(it->second->name, "OUT_" + toUpper(sig));
    }

    j << "  \"edges\": [\n";
    for (int i = 0; i < (int)edges.size(); i++) {
        if (i > 0) j << ",\n";
        j << "    {\"from\": \"" << jsonEscape(edges[i].from)
          << "\", \"to\": \""   << jsonEscape(edges[i].to) << "\"}";
    }
    j << "\n  ],\n";

    // ── topo_order_bfs / dfs ─────────────────────────────────
    // 輸入節點（概念上在最前面）+ 閘依排序順序
    auto writeTopoArray = [&](const std::vector<Gate*>& order) {
        j << "[";
        bool f = true;
        for (const auto& sig : inputSignals) {
            if (!f) j << ", ";
            j << "\"" << jsonEscape("IN_" + toUpper(sig)) << "\"";
            f = false;
        }
        for (Gate* g : order) {
            if (!f) j << ", ";
            j << "\"" << jsonEscape(g->name) << "\"";
            f = false;
        }
        j << "]";
    };

    j << "  \"topo_order_bfs\": ";
    writeTopoArray(circuit.getBfsOrder());
    j << ",\n";

    j << "  \"topo_order_dfs\": ";
    writeTopoArray(circuit.getDfsOrder());
    j << ",\n";

    // ── critical_paths ───────────────────────────────────────
    auto paths = computeTopKCriticalPaths(
        allGates, inputSignals, outputSignals, topK);

    j << "  \"critical_paths\": [\n";
    for (int i = 0; i < (int)paths.size(); i++) {
        const auto& cp = paths[i];
        int freq = (cp.totalDelay > 0) ? (1000 / cp.totalDelay) : 0;
        if (i > 0) j << ",\n";
        j << "    {\n";
        j << "      \"rank\": " << (i + 1) << ",\n";
        j << "      \"delay\": " << cp.totalDelay << ",\n";
        j << "      \"max_freq_mhz\": " << freq << ",\n";
        j << "      \"path\": [";
        for (int k = 0; k < (int)cp.nodes.size(); k++) {
            if (k > 0) j << ", ";
            j << "\"" << jsonEscape(toNodeId(cp.nodes[k], inputSet, outputSet)) << "\"";
        }
        j << "]\n";
        j << "    }";
    }
    j << "\n  ],\n";

    // ── truth_table ──────────────────────────────────────────
    int n     = (int)inputSignals.size();
    int total = 1 << n;

    j << "  \"truth_table\": [\n";
    for (int combo = 0; combo < total; combo++) {
        if (combo > 0) j << ",\n";

        std::vector<bool> inputValues(n);
        for (int i = 0; i < n; i++)
            inputValues[n - 1 - i] = (combo >> i) & 1;
        circuit.simulate(inputValues);

        j << "    {\"inputs\": {";
        for (int i = 0; i < n; i++) {
            if (i > 0) j << ", ";
            j << "\"" << jsonEscape(toUpper(inputSignals[i])) << "\": "
              << (inputValues[i] ? 1 : 0);
        }
        j << "}, \"outputs\": {";
        for (int i = 0; i < (int)outputSignals.size(); i++) {
            if (i > 0) j << ", ";
            j << "\"" << jsonEscape(toUpper(outputSignals[i])) << "\": "
              << (circuit.getSignal(outputSignals[i]) ? 1 : 0);
        }
        j << "}}";
    }
    j << "\n  ]\n";
    j << "}\n";

    // ── 寫入檔案（自動建立父目錄）───────────────────────────
    std::filesystem::path outPath(outputPath);
    if (outPath.has_parent_path()) {
        std::error_code ec;
        std::filesystem::create_directories(outPath.parent_path(), ec);
        if (ec) {
            std::cerr << "錯誤：無法建立目錄 "
                      << outPath.parent_path() << "：" << ec.message() << "\n";
            return false;
        }
    }

    std::ofstream out(outputPath);
    if (!out.is_open()) {
        std::cerr << "錯誤：無法寫入 " << outputPath << "\n";
        return false;
    }
    out << j.str();
    return true;
}

// ============================================================
// qmStepsToJson：序列化 QM 中間結果為 JSON 子物件字串
// ============================================================
static std::string qmStepsToJson(const QMResult& r, const std::string& indent = "  ") {
    std::ostringstream j;
    j << "{\n";

    // input_minterms
    j << indent << "  \"input_minterms\": [";
    for (int i = 0; i < (int)r.inputMinterms.size(); i++) {
        if (i > 0) j << ", ";
        j << r.inputMinterms[i];
    }
    j << "],\n";

    // prime_implicants
    j << indent << "  \"prime_implicants\": [\n";
    for (int i = 0; i < (int)r.piRecords.size(); i++) {
        if (i > 0) j << ",\n";
        const auto& pi = r.piRecords[i];
        j << indent << "    {\n";
        j << indent << "      \"index\": "     << i << ",\n";
        j << indent << "      \"term\": \""    << jsonEscape(pi.term) << "\",\n";
        j << indent << "      \"minterms\": [";
        for (int k = 0; k < (int)pi.minterms.size(); k++) {
            if (k > 0) j << ", ";
            j << pi.minterms[k];
        }
        j << "],\n";
        j << indent << "      \"essential\": " << (pi.essential ? "true" : "false") << ",\n";
        j << indent << "      \"selected\": "  << (pi.selected  ? "true" : "false") << "\n";
        j << indent << "    }";
    }
    j << "\n" << indent << "  ],\n";

    // coverage_table（以 minterm 排序，確保輸出確定性）
    std::vector<int> sortedMints;
    sortedMints.reserve(r.mintermToPIs.size());
    for (const auto& kv : r.mintermToPIs) sortedMints.push_back(kv.first);
    std::sort(sortedMints.begin(), sortedMints.end());

    j << indent << "  \"coverage_table\": [\n";
    for (int k = 0; k < (int)sortedMints.size(); k++) {
        if (k > 0) j << ",\n";
        int m = sortedMints[k];
        const auto& piList = r.mintermToPIs.at(m);
        j << indent << "    {\"minterm\": " << m << ", \"pi_indices\": [";
        for (int p = 0; p < (int)piList.size(); p++) {
            if (p > 0) j << ", ";
            j << piList[p];
        }
        j << "]}";
    }
    j << "\n" << indent << "  ]\n";
    j << indent << "}";
    return j.str();
}

// ============================================================
// exportQMToJson：.tt → QM 分析 → JSON
// ============================================================

// 內部 helper：把一個 Circuit 序列化成與 exportCircuitToJson 相同格式的 JSON 子物件字串
// 縮排基底為 indent（通常 2 個空格），每行前綴 indent
// 回傳的字串不含最外層的欄位鍵名與尾部逗號
static std::string circuitToJsonObject(Circuit& circuit, int topK,
                                        const std::string& indent = "  ")
{
    // 確保拓撲排序完成
    if (!circuit.topologicalSortBFS()) return "null";
    if (!circuit.topologicalSortDFS()) return "null";
    circuit.topologicalSort();

    const auto& inputSignals  = circuit.getInputs();
    const auto& outputSignals = circuit.getOutputs();
    std::vector<Gate*> allGates = circuit.getAllGates();

    std::unordered_set<std::string> inputSet(inputSignals.begin(),  inputSignals.end());
    std::unordered_set<std::string> outputSet(outputSignals.begin(), outputSignals.end());

    std::unordered_map<std::string, Gate*> signalSource;
    for (Gate* g : allGates) signalSource[g->outputName] = g;

    // 複用 json_export.cpp 的輔助函式（同檔案內 static）
    auto nodeId = [&](const std::string& name) -> std::string {
        if (inputSet.count(name))  return "IN_"  + toUpper(name);
        if (outputSet.count(name)) return "OUT_" + toUpper(name);
        return name;
    };

    std::ostringstream j;
    j << "{\n";

    // nodes
    j << indent << "  \"nodes\": [\n";
    bool first = true;
    auto sep = [&]() -> std::ostringstream& {
        if (!first) j << ",\n";
        first = false;
        return j;
    };
    for (const auto& sig : inputSignals)
        sep() << indent << "    {\"id\": \"" << jsonEscape("IN_" + toUpper(sig))
              << "\", \"type\": \"INPUT\", \"delay\": 0}";
    for (Gate* g : allGates)
        sep() << indent << "    {\"id\": \"" << jsonEscape(g->name)
              << "\", \"type\": \"" << jsonEscape(g->type)
              << "\", \"delay\": " << g->delay << "}";
    for (const auto& sig : outputSignals)
        sep() << indent << "    {\"id\": \"" << jsonEscape("OUT_" + toUpper(sig))
              << "\", \"type\": \"OUTPUT\", \"delay\": 0}";
    j << "\n" << indent << "  ],\n";

    // edges
    struct Edge { std::string from, to; };
    std::vector<Edge> edges;
    std::unordered_set<std::string> edgeSeen;
    auto addEdge = [&](const std::string& from, const std::string& to) {
        std::string key = from + '\x01' + to;
        if (edgeSeen.insert(key).second) edges.push_back({from, to});
    };
    for (Gate* g : allGates) {
        for (const auto& inp : g->inputNames) {
            if (inputSet.count(inp))
                addEdge("IN_" + toUpper(inp), g->name);
            else {
                auto it = signalSource.find(inp);
                if (it != signalSource.end()) addEdge(it->second->name, g->name);
            }
        }
    }
    for (const auto& sig : outputSignals) {
        auto it = signalSource.find(sig);
        if (it != signalSource.end())
            addEdge(it->second->name, "OUT_" + toUpper(sig));
    }
    j << indent << "  \"edges\": [\n";
    for (int i = 0; i < (int)edges.size(); i++) {
        if (i > 0) j << ",\n";
        j << indent << "    {\"from\": \"" << jsonEscape(edges[i].from)
          << "\", \"to\": \""   << jsonEscape(edges[i].to) << "\"}";
    }
    j << "\n" << indent << "  ],\n";

    // topo orders
    auto writeTopoArray = [&](const std::vector<Gate*>& order) {
        j << "[";
        bool f = true;
        for (const auto& sig : inputSignals) {
            if (!f) j << ", ";
            j << "\"" << jsonEscape("IN_" + toUpper(sig)) << "\"";
            f = false;
        }
        for (Gate* g : order) {
            if (!f) j << ", ";
            j << "\"" << jsonEscape(g->name) << "\"";
            f = false;
        }
        j << "]";
    };
    j << indent << "  \"topo_order_bfs\": ";
    writeTopoArray(circuit.getBfsOrder());
    j << ",\n";
    j << indent << "  \"topo_order_dfs\": ";
    writeTopoArray(circuit.getDfsOrder());
    j << ",\n";

    // critical paths
    auto paths = computeTopKCriticalPaths(allGates, inputSignals, outputSignals, topK);
    j << indent << "  \"critical_paths\": [\n";
    for (int i = 0; i < (int)paths.size(); i++) {
        const auto& cp = paths[i];
        int freq = (cp.totalDelay > 0) ? (1000 / cp.totalDelay) : 0;
        if (i > 0) j << ",\n";
        j << indent << "    {\n";
        j << indent << "      \"rank\": "        << (i + 1)        << ",\n";
        j << indent << "      \"delay\": "       << cp.totalDelay  << ",\n";
        j << indent << "      \"max_freq_mhz\": "<< freq           << ",\n";
        j << indent << "      \"path\": [";
        for (int k = 0; k < (int)cp.nodes.size(); k++) {
            if (k > 0) j << ", ";
            j << "\"" << jsonEscape(nodeId(cp.nodes[k])) << "\"";
        }
        j << "]\n" << indent << "    }";
    }
    j << "\n" << indent << "  ],\n";

    // truth table
    int n     = (int)inputSignals.size();
    int total = 1 << n;
    j << indent << "  \"truth_table\": [\n";
    for (int combo = 0; combo < total; combo++) {
        if (combo > 0) j << ",\n";
        std::vector<bool> inputValues(n);
        for (int i = 0; i < n; i++) inputValues[n - 1 - i] = (combo >> i) & 1;
        circuit.simulate(inputValues);
        j << indent << "    {\"inputs\": {";
        for (int i = 0; i < n; i++) {
            if (i > 0) j << ", ";
            j << "\"" << jsonEscape(toUpper(inputSignals[i])) << "\": "
              << (inputValues[i] ? 1 : 0);
        }
        j << "}, \"outputs\": {";
        for (int i = 0; i < (int)outputSignals.size(); i++) {
            if (i > 0) j << ", ";
            j << "\"" << jsonEscape(toUpper(outputSignals[i])) << "\": "
              << (circuit.getSignal(outputSignals[i]) ? 1 : 0);
        }
        j << "}}";
    }
    j << "\n" << indent << "  ]\n";
    j << indent << "}";

    return j.str();
}

bool exportQMToJson(const std::string& ttPath,
                    const std::string& outputPath,
                    int topK)
{
    // ── 解析 .tt ──────────────────────────────────────────────
    int numVars = 0;
    std::vector<int> minterms, dontCares;
    std::vector<std::string> varNames;
    if (!parseTruthTableFile(ttPath, numVars, minterms, dontCares, varNames))
        return false;

    // ── QM 最小化 ─────────────────────────────────────────────
    QMResult qmResult = minimize(numVars, minterms, dontCares, varNames);
    if (qmResult.numVars == 0) {
        std::cerr << "錯誤：QM 最小化失敗\n";
        return false;
    }

    // ── 建兩個電路（可能是 nullopt）──────────────────────────
    std::optional<Circuit> directOpt   = mintermToCircuit(numVars, minterms, qmResult.varNames, "F");
    std::optional<Circuit> minimizedOpt = sopToCircuit(qmResult, "F");

    // ── 建構 JSON ─────────────────────────────────────────────
    std::ostringstream j;
    j << "{\n";

    // source_file
    j << "  \"source_file\": \"" << jsonEscape(ttPath) << "\",\n";

    // qm_result
    j << "  \"qm_result\": {\n";
    j << "    \"num_vars\": " << qmResult.numVars << ",\n";
    j << "    \"var_names\": [";
    for (int i = 0; i < (int)qmResult.varNames.size(); i++) {
        if (i > 0) j << ", ";
        j << "\"" << jsonEscape(qmResult.varNames[i]) << "\"";
    }
    j << "],\n";
    j << "    \"sop\": \"" << jsonEscape(qmResultToString(qmResult)) << "\",\n";
    j << "    \"term_count\": " << qmResult.terms.size() << ",\n";
    j << "    \"terms\": [\n";
    for (int t = 0; t < (int)qmResult.terms.size(); t++) {
        if (t > 0) j << ",\n";
        const auto& pt = qmResult.terms[t];
        j << "      {\"literals\": [";
        bool firstLit = true;
        for (int i = 0; i < qmResult.numVars; i++) {
            if (!((pt.care >> i) & 1)) continue;
            if (!firstLit) j << ", ";
            firstLit = false;
            j << "\"" << jsonEscape(qmResult.varNames[i]);
            if (!((pt.polarity >> i) & 1)) j << "'";
            j << "\"";
        }
        j << "]}";
    }
    j << "\n    ]\n";
    j << "  },\n";

    // qm_steps（中間結果）
    j << "  \"qm_steps\": " << qmStepsToJson(qmResult) << ",\n";

    // direct_circuit
    j << "  \"direct_circuit\": ";
    if (directOpt.has_value()) {
        j << circuitToJsonObject(*directOpt, topK, "  ");
    } else {
        j << "null";
    }
    j << ",\n";

    // minimized_circuit
    j << "  \"minimized_circuit\": ";
    if (minimizedOpt.has_value()) {
        j << circuitToJsonObject(*minimizedOpt, topK, "  ");
    } else {
        j << "null";
    }
    j << "\n}\n";

    // ── 寫入檔案（自動建立父目錄）───────────────────────────
    std::filesystem::path outPath(outputPath);
    if (outPath.has_parent_path()) {
        std::error_code ec;
        std::filesystem::create_directories(outPath.parent_path(), ec);
        if (ec) {
            std::cerr << "錯誤：無法建立目錄 "
                      << outPath.parent_path() << "：" << ec.message() << "\n";
            return false;
        }
    }

    std::ofstream out(outputPath);
    if (!out.is_open()) {
        std::cerr << "錯誤：無法寫入 " << outputPath << "\n";
        return false;
    }
    out << j.str();
    return true;
}

// ============================================================
// exportQMToStream：直接從已解析資料輸出 JSON 到任意 ostream
// ============================================================
bool exportQMToStream(std::ostream& out,
                      int numVars,
                      const std::vector<int>& minterms,
                      const std::vector<int>& dontCares,
                      const std::vector<std::string>& varNames,
                      int topK)
{
    QMResult qmResult = minimize(numVars, minterms, dontCares, varNames);
    if (qmResult.numVars == 0) {
        std::cerr << "錯誤：QM 最小化失敗\n";
        return false;
    }

    // 使用 qmResult.varNames（已補好 x0,x1,... 預設名稱），避免原始 varNames 為空時越界
    std::optional<Circuit> directOpt    = mintermToCircuit(numVars, minterms, qmResult.varNames, "F");
    std::optional<Circuit> minimizedOpt = sopToCircuit(qmResult, "F");

    std::ostringstream j;
    j << "{\n";
    j << "  \"source_file\": \"inline\",\n";

    j << "  \"qm_result\": {\n";
    j << "    \"num_vars\": " << qmResult.numVars << ",\n";
    j << "    \"var_names\": [";
    for (int i = 0; i < (int)qmResult.varNames.size(); i++) {
        if (i > 0) j << ", ";
        j << "\"" << jsonEscape(qmResult.varNames[i]) << "\"";
    }
    j << "],\n";
    j << "    \"sop\": \"" << jsonEscape(qmResultToString(qmResult)) << "\",\n";
    j << "    \"term_count\": " << qmResult.terms.size() << ",\n";
    j << "    \"terms\": [\n";
    for (int t = 0; t < (int)qmResult.terms.size(); t++) {
        if (t > 0) j << ",\n";
        const auto& pt = qmResult.terms[t];
        j << "      {\"literals\": [";
        bool firstLit = true;
        for (int i = 0; i < qmResult.numVars; i++) {
            if (!((pt.care >> i) & 1)) continue;
            if (!firstLit) j << ", ";
            firstLit = false;
            j << "\"" << jsonEscape(qmResult.varNames[i]);
            if (!((pt.polarity >> i) & 1)) j << "'";
            j << "\"";
        }
        j << "]}";
    }
    j << "\n    ]\n";
    j << "  },\n";

    j << "  \"qm_steps\": " << qmStepsToJson(qmResult) << ",\n";

    j << "  \"direct_circuit\": ";
    if (directOpt.has_value()) j << circuitToJsonObject(*directOpt, topK, "  ");
    else j << "null";
    j << ",\n";

    j << "  \"minimized_circuit\": ";
    if (minimizedOpt.has_value()) j << circuitToJsonObject(*minimizedOpt, topK, "  ");
    else j << "null";
    j << "\n}\n";

    out << j.str();
    return true;
}
