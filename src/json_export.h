#pragma once
// json_export.h ── JSON 匯出模組宣告
// 不依賴第三方函式庫，自行實作 JSON 字串建構與 escape。

#include "circuit.h"
#include "critical_path.h"
#include <string>

// 從檔案路徑取出 circuit name（basename，去除副檔名）
// 例："src/circuits/full_adder.txt" → "full_adder"
std::string circuitNameFromPath(const std::string& path);

// 將電路資料匯出成 JSON 檔案
// circuit     : 已載入的電路（函式內部會執行 topo sort 與模擬，不影響呼叫端狀態）
// circuitName : JSON 中的 "circuit_name" 欄位
// outputPath  : 輸出檔案路徑（父目錄不存在時自動建立）
// topK        : critical_paths 要輸出幾條（預設 3）
// 回傳 true 表示成功
bool exportCircuitToJson(Circuit& circuit,
                         const std::string& circuitName,
                         const std::string& outputPath,
                         int topK = 3);

// 從 .tt 真值表描述檔解析、執行 QM 最小化，並將以下內容匯出為 JSON：
//   - QM 結果摘要（SOP 字串、乘積項數）
//   - 直接版電路（每個 minterm 一個 AND 閘）
//   - QM 化簡版電路（每個乘積項一個 AND 閘）
// 恆 0 / 恆 1 時對應電路欄位輸出 null
// 回傳 true 表示成功
bool exportQMToJson(const std::string& ttPath,
                    const std::string& outputPath,
                    int topK = 3);
