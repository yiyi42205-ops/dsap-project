# 數位邏輯電路模擬器 (Digital Logic Circuit Simulator)

## Proposal Report

### 動機與目標

修讀資料結構課程時，學到 DAG 與拓撲排序的應用後，我意識到一件之前修交換電路時沒被點明的事：**邏輯閘電路本身就是一個 DAG，訊號傳播就是拓撲排序**。交換電路那門課從電子工程的角度切入，沒有特別著墨在底層的資料結構抽象；而資料結構課在介紹 DAG、Heap、Hash Map 等抽象演算法時，也很少直接連到真實硬體。這個跨課程的觀察，是本專題的起點。

本專題以「用電路學資料結構」為核心定位，開發一個 C++ 命令列工具，讓使用者以文字描述數位邏輯電路，程式會自動分析電路結構、計算輸出結果、產生真值表，並對拓撲排序與關鍵路徑分析等核心演算法進行可視化追蹤與效能實驗。專案的價值不在於取代既有電路模擬器（Logisim / CircuitVerse），而在於**讓 DAG、拓撲排序、Heap 這些抽象資料結構，透過硬體模擬被「看見」**——這也是現有電路工具普遍未直接呈現的部分。

### 競品比較

| 項目 | Logisim | CircuitVerse | 本專題 |
|------|---------|--------------|--------|
| 類型 | Java 桌面應用程式 | 線上網頁模擬器 | C++ 命令列工具 + React 視覺化 |
| 使用方式 | 圖形拖拉介面 | 圖形拖拉介面 | 文字描述檔定義電路 |
| 開源 | ✅ | ✅ | ✅ |
| 演算法過程可視化 | 未直接呈現 | 未直接呈現 | 逐步輸出 BFS / DFS 拓撲排序的計算順序與 DAG 結構 |
| 關鍵路徑分析 | ❌ | ❌ | ✅ 計算最長延遲路徑 + Top-K |
| 演算法教學用途 | 低（重視操作） | 低（重視操作） | 高（展示資料結構如何驅動模擬） |
| 學習門檻 | 中（需安裝 Java） | 低 | 低（只需 g++ 編譯） |

**本專題的定位差異：** Logisim 和 CircuitVerse 都是「給使用者操作電路」的工具，重視圖形介面。本專題則反過來，重視「電路模擬背後的演算法」——讓使用者看到 DAG 如何建構、拓撲排序如何決定計算順序、Heap 如何找出關鍵路徑。換句話說，前兩者是「用工具學電路」，本專題是「用電路學資料結構」。

### 預期功能

1. **基礎邏輯閘模擬**：支援 AND、OR、NOT、XOR、NAND、NOR 六種基本邏輯閘，每個閘以物件（class）實作，可接受任意數量的輸入。
2. **電路描述檔解析**：使用者撰寫簡單的文字檔描述電路結構（元件名稱、類型、連線關係），程式解析後建構出對應的電路圖。
3. **自動訊號傳播**：將電路視為有向無環圖（DAG），以拓撲排序決定各邏輯閘的計算順序，確保輸入訊號在輸出之前已正確計算。
4. **真值表產生**：窮舉所有輸入組合，自動計算並印出完整的真值表。
5. **組合電路範例**：內建半加器（Half Adder）、全加器（Full Adder）、4-bit 加法器、多工器（MUX）等經典電路範例。

#### 進階功能（視時間而定）

6. **時序電路支援**：加入 D 正反器（D Flip-Flop）與時脈訊號，支援計數器等時序邏輯電路的模擬。
7. **關鍵路徑分析**：為每個邏輯閘設定傳播延遲，計算電路從輸入到輸出的最長延遲路徑（Critical Path）。
8. **互動式視覺化介面**：以網頁呈現電路圖，支援即時切換輸入、動態顯示訊號傳播、拓撲排序步進展示。

### 使用技術

| 技術 | 說明 |
|------|------|
| **DAG（有向無環圖）** | 將電路視為圖：邏輯閘為節點，連線為邊。整個電路形成一個 DAG，確保訊號從輸入端單向傳播到輸出端。 |
| **拓撲排序（Topological Sort）** | 核心演算法。對 DAG 進行拓撲排序，決定邏輯閘的計算順序，保證每個閘在計算時其所有輸入已經就緒。 |
| **雜湊表（Hash Map）** | 使用 `std::unordered_map` 儲存元件名稱與物件的對應關係，實現 O(1) 的元件查詢。 |
| **二元搜尋樹（BST）** | 以 `std::map`（底層為紅黑樹）管理電路中的訊號線名稱，維持有序的訊號列表以便輸出真值表。 |
| **堆積與優先權佇列（Heap / Priority Queue）** | 用於「Top-K 關鍵路徑」查詢：找出電路中延遲最長的前 K 條路徑。DAG 單一最長路徑用拓撲排序 + DP 即可求得；引入 Max-Heap 是為了支援 Top-K 查詢，能在不窮舉所有路徑的前提下依序取出延遲最大的 K 條路徑。 |
| **Struct / Class（OOP）** | 使用繼承與多型設計邏輯閘的類別階層：基底類別 `Gate`，衍生出 `AndGate`、`OrGate`、`NotGate` 等子類別。 |

### Prototype 預計可驗證內容

Prototype 階段預計完成以下可驗證的功能：

1. **電路描述檔解析器（Parser）能正確讀取 .txt 檔案並建構 DAG**
   - 驗證方式：載入內建的半加器、全加器、MUX 描述檔，印出解析結果，確認元件與連線正確。

2. **拓撲排序能正確決定邏輯閘的計算順序**
   - 驗證方式：印出拓撲排序結果，人工檢查順序是否滿足「每個閘在計算前，其所有輸入皆已就緒」的條件。

3. **真值表輸出結果正確**
   - 驗證方式：以半加器為例，預期真值表為 `{(0,0)→(0,0), (0,1)→(1,0), (1,0)→(1,0), (1,1)→(0,1)}`，與程式輸出比對。以全加器驗證 8 種輸入組合的 Sum 與 Carry 是否正確。

4. **能偵測電路中的迴圈（非 DAG）並報錯**
   - 驗證方式：故意建立一個有迴圈的電路描述檔，確認程式能偵測並印出錯誤訊息。

> **換題說明：** 在進一步研究後，我發現數位邏輯電路模擬器能更完整地運用本課程教授的多種資料結構（DAG、拓撲排序、BST、Heap、Hash Map），同時也與我未來想深入學習的計算機硬體領域密切相關，因此決定更換主題。

---

## Prototype Report

### 目前進度

目前已完成 Proposal 中列出的全部 4 項可驗證功能，以及新增的效能比較功能：

**✅ 已完成的功能：**

1. **六種邏輯閘模擬（AND、OR、NOT、XOR、NAND、NOR）**
   以物件導向設計實作：Gate 基底類別 + 各子類別，使用虛擬函式（polymorphism）實現不同閘的 `compute()` 邏輯。

2. **電路描述檔解析器（Parser）**
   支援自定義的 `.txt` 格式，可定義 INPUT、OUTPUT、GATE 及連線關係（`->`）。已通過半加器、全加器、MUX 三個測試檔的驗證。

3. **DAG 建構 + 拓撲排序**
   實作了兩種拓撲排序演算法：
   - **BFS 版（Kahn's Algorithm）**：使用 Queue + Hash Map，逐步移除入度為 0 的節點。
   - **DFS 版（反向後序遍歷）**：使用 Stack（遞迴呼叫堆疊）+ Hash Map，深度優先搜尋後反轉結果。
   兩者皆能正確偵測電路中的迴圈。

4. **真值表自動產生**
   窮舉所有 2^n 種輸入組合，逐一模擬並印出完整真值表。已驗證半加器（4 組）、全加器（8 組）、4-bit 加法器（512 組）的輸出結果皆正確。

5. **4 個內建範例電路**
   半加器（2 閘）、全加器（5 閘）、2-to-1 MUX（4 閘）、4-bit 加法器（20 閘）。

6. **⭐ BFS vs DFS 拓撲排序效能比較（對應期末 Demo 要求）**
   針對同一電路，分別以 BFS 和 DFS 兩種方式執行拓撲排序，測量並比較：
   - 純排序時間（10,000 次迭代）
   - 完整模擬時間（排序 + 訊號傳播，1,000 輪）
   - 結果一致性驗證

   初步實測結果摘要：

   | 電路 | 閘數 | BFS 平均排序 | DFS 平均排序 | 勝者 |
   |------|------|-------------|-------------|------|
   | 半加器 | 2 | 0.37 μs | 0.35 μs | 接近 |
   | 全加器 | 5 | 0.98 μs | 0.91 μs | DFS |
   | MUX | 4 | 0.72 μs | 0.72 μs | 接近 |
   | 4-bit 加法器 | 20 | 9.82 μs | 6.61 μs | DFS（快 1.5x） |

   觀察：在小規模電路中兩者差異不大；隨著閘數增加，DFS 版本因為較少的記憶體配置開銷而領先。兩種演算法產出的拓撲排序順序不同，但模擬結果完全一致，驗證了「合法的拓撲排序不唯一，但計算結果唯一」。

   > **註：** 此初步觀察（「規模越大 DFS 越領先」）在 Final Report 階段的大規模實驗（10–5000 閘 × 三種 DAG 結構）中被修正——真正的變因是 DAG 形狀而非規模。詳見 Final Report 的「主要研究發現」一節。

### 遇到的困難

1. **迴圈偵測的設計**：DFS 版需要區分「訪問中」和「已完成」兩種狀態才能正確偵測迴圈（back edge），一開始只用 visited 布林值導致誤判。後來改用三態（未訪問 / 訪問中 / 已完成）解決。

2. **效能測量的穩定性**：單次執行時間太短（微秒級），受系統排程影響大。改為重複執行 10,000 次取平均後，數據穩定。

3. **4-bit 加法器的 DAG 深度**：4 個全加器串接形成較長的依賴鏈（carry 必須逐級傳播），拓撲排序的結果對計算順序影響較大，這也是當時猜測「規模越大 BFS 和 DFS 的差異越明顯」的原因（後續實驗推翻此假設）。

### 下一步計畫

1. **關鍵路徑分析（Critical Path）**：為每個邏輯閘加入傳播延遲值，使用 DAG 最長路徑演算法 + Priority Queue 找出電路的關鍵路徑，計算最大時脈頻率。
2. **互動式視覺化介面**：以 React 實作網頁版電路視覺化（已有原型），支援即時切換輸入、動態訊號傳播動畫、拓撲排序步進展示。
3. **更大規模的效能測試**：自動生成 100 閘、1000 閘的隨機電路，觀察 BFS vs DFS 在大規模下的效能差異曲線。
4. **錄製 Demo 影片**：展示電路模擬功能 + 效能比較結果 + 視覺化介面。

---

## Final Report

### 專案說明

本專題實作了一個 C++ 命令列數位邏輯電路模擬器，搭配 React 視覺化前端，以「用電路學資料結構」為核心定位：電路本身是一個 **有向無環圖（DAG）**，模擬器透過**拓撲排序**決定邏輯閘的計算順序，**Heap** 找出延遲最長的關鍵路徑，讓使用者能清楚看到演算法如何驅動硬體模擬。

### 主要研究發現

本專題的實驗階段產生了三個值得記錄的發現：

**1. 演算法效能取決於 DAG 結構，而非規模**

Prototype 階段觀察「規模越大 DFS 越領先」（4-bit 加法器 20 閘下 DFS 快 1.5x），但 `scale_bench`（10–5000 閘隨機 DAG）顯示加速比穩定在 1.1–1.2x、與規模幾乎無關——假設被推翻。

進一步的 `structural_bench` 揭示真正變因是 DAG 形狀：Random / Wide Fanout 下 DFS 比 BFS 快 8–15%；Deep Chain 下三者幾乎相同（±5%）。此結論在實驗規模（≤5000 閘）與測試結構內成立，推廣到更大規模或其他 DAG 應用領域（build system 依賴解析、ML 框架計算圖排程）需進一步驗證。

**2. 理論等價不蘊含實作等價**

我預期 DFS 在深鏈上大勝(因為 cache 友善),但實測發現三種演算法幾乎相同。進一步分析發現,深鏈下每個節點的計算都依賴前一個節點完成,造成 sequential dependency 主導執行時間,演算法層的 cache 差異被淹沒。這顯示一個常被忽略的事實:演算法的『常數因子優勢』必須要有足夠的平行處理能力才能被看見。在 sequential bottleneck 主導的情境下,選用更複雜的資料結構可能是無效的優化。

**3. Critical Path 將抽象資料結構連到具體物理量**

Top-K 最長路徑搜尋是本專題資料結構協作最緊密的部分：拓撲排序確定計算順序 → DP 求每個節點的最長到達延遲 → Max-Heap 反向取出延遲最大的 K 條完整路徑。

這個演算法的輸出可直接換算成電路最高工作頻率（全加器 critical path 7 ns → 142 MHz）。現代 CPU 的時脈設計本質上是同一個「DAG 最長路徑」問題的延伸（工業界 Static Timing Analysis 還會處理 false path、setup/hold time、clock skew 等）。資料結構不是抽象練習，是決定硬體效能的數學基礎。

### 最終實作的功能

| 功能 | 說明 |
|------|------|
| 6 種邏輯閘 | AND / OR / NOT / XOR / NAND / NOR，OOP 繼承 + 虛擬函式 |
| 電路描述檔解析 | 自訂 `.txt` 格式，支援 INPUT / OUTPUT / GATE 關鍵字 |
| BFS 拓撲排序 | Kahn's Algorithm,O(V+E),deque + inDegree map |
| DFS 拓撲排序（遞迴） | 反向後序遍歷，O(V+E)，三色標記（WHITE/GRAY/BLACK）、含環偵測 |
| DFS 拓撲排序（迭代） | 顯式 `std::stack` + 三色標記，消除遞迴 overhead |
| 真值表生成 | 窮舉 2^n 種輸入組合，自動計算並印出 |
| **Top-K 最長路徑分析** | DAG 最長路徑 DP（拓撲排序後 relax）+ Max-Heap 反向搜尋前 K 條路徑；輸出路徑序列與延遲總和；可用 `--critical-path K` 指定 K 值 |
| `--trace` 模式 | 逐步印出 BFS / DFS 排序決策與 Critical Path DP 過程 |
| **效能實驗** | 大規模測試（10–5000 閘）+ 結構性對照實驗（random / deep_chain / wide_fanout）× 三種演算法，輸出 log-log 圖、加速比圖、grouped bar chart |
| **React 網頁視覺化** | React + Vite + React Flow，支援電路選擇、INPUT 切換 + 訊號傳播、BFS/DFS 排序動畫、Critical Path 高亮；C++ 透過 `--export-json` 匯出資料給前端 |
| **Quine-McCluskey 邏輯最小化** | 從 minterm 列表（或直接從 Circuit output 提取）產生最簡 SOP；支援 don't care；演算法：PI 生成 → Essential PI 選取 → 貪心覆蓋 |
| **等價性檢查器（窮舉版）** | 輸入兩個電路，窮舉所有 2^n 輸入組合比對輸出；不等價時提供第一個反例（輸入向量 + 兩電路各自的輸出值）|

---

### 使用方式

#### 編譯

```bash
make          # 編譯主程式（生成 ./simulator）
make test     # 執行所有 unit tests（50 個測試）
```

#### 互動式選單模式

```bash
./simulator
```

選單選項：
1. 半加器（2 閘）
2. 全加器（5 閘）
3. 2-to-1 MUX（4 閘）
4. 4-bit 加法器（20 閘）
5. 從檔案載入
6. 效能比較模式（比較所有內建電路）

#### 從檔案載入

```bash
./simulator src/circuits/full_adder.txt
```

輸出：電路資訊 → 拓撲排序順序 → 真值表 → Critical Path Analysis → BFS vs DFS 效能比較

#### Critical Path 指定 K 值

```bash
./simulator --critical-path 5 src/circuits/full_adder.txt   # 印出前 5 條最長路徑
```

Critical Path 輸出範例（全加器，預設 K=3）：
```
=== Critical Path Analysis ===
#1 延遲: 7 ns (最高頻率: 142 MHz)
   路徑: A → XOR_1 → AND_2 → OR_1 → Cout
#2 延遲: 7 ns (最高頻率: 142 MHz)
   路徑: B → XOR_1 → AND_2 → OR_1 → Cout
#3 延遲: 6 ns (最高頻率: 166 MHz)
   路徑: A → XOR_1 → XOR_2 → S
```

#### `--trace` 模式（逐步追蹤）

```bash
./simulator --trace src/circuits/full_adder.txt
```

逐步印出 BFS 排序、DFS 排序，以及 Critical Path DP 計算每個節點延遲的過程。`--trace` 模式下跳過真值表和效能比較，建議用於小型電路（< 20 閘）。

#### JSON 匯出

```bash
./simulator --export-json output/full_adder.json src/circuits/full_adder.txt
```

匯出內容：`nodes`、`edges`、`topo_order_bfs` / `topo_order_dfs`、`critical_paths`（Top-K）、`truth_table`。可搭配 `--critical-path K` 控制匯出幾條路徑，供 React 前端使用。

#### 執行 Benchmark

```bash
make scale_bench         # 大規模效能測試：7 個規模 × 3 種演算法（BFS / DFS-rec / DFS-iter）
make scale_plot          # 生成 scale_test_loglog.png / scale_test_speedup.png
make structural_bench    # 結構性對照實驗：3 種 DAG 結構 × 7 規模 × 3 演算法
make structural_plot     # 生成 structural_loglog.png / structural_speedup.png / structural_bar1000.png
```

圖表輸出說明：

| 圖檔 | 說明 |
|------|------|
| `results/scale_test_loglog.png` | 隨機 DAG 規模 vs 時間 log-log 圖（含 error bar）|
| `results/scale_test_speedup.png` | 各規模 DFS-rec / DFS-iter 相對 BFS 的耗時比 grouped bar |
| `results/structural_loglog.png` | 三種結構 × 三種演算法 log-log 效能圖（9 條線）|
| `results/structural_speedup.png` | DFS-rec/BFS、DFS-iter/BFS 加速比，三種結構各一子圖 |
| `results/structural_bar1000.png` | 1000 閘 grouped bar：三種結構 × 三種演算法 |

#### Quine-McCluskey 邏輯最小化

```cpp
// 從 minterm 列表最小化（標頭：src/qm_minimizer.h）
QMResult r = minimize(3, {0,1,2,3,4,6}, {}, {"A","B","C"});
printQMResult(r);          // 印出最簡 SOP
// → "A' + C'"

// 含 don't care
QMResult r2 = minimize(3, {1,3,7}, {0,5}, {"A","B","C"});
// → "C"

// 從電路 output 自動提取
Circuit fa = makeFullAdder();
QMResult r3 = minimizeCircuitOutput(fa, "Cout");
// → "AB + ACin + BCin"
```

```bash
make test_qm   # 單獨跑 QM 最小化器的 18 個測試
```

#### 等價性檢查器

```cpp
// 標頭：src/equivalence_checker.h
Circuit c1 = makeHalfAdder();
Circuit c2 = makeHalfAdderNand();   // 全 NAND 實作
EquivalenceResult r = checkEquivalence(c1, c2);
printEquivalenceResult(r, c1, c2);
// → 結果：✓ 等價

Circuit c3 = makeBuggyHalfAdder();  // XOR 誤換成 OR
EquivalenceResult r2 = checkEquivalence(c1, c3);
printEquivalenceResult(r2, c1, c3);
// → 結果：✗ 不等價
// → 反例輸入：A=1, B=1
// → c1 輸出：S=0, C=1
// → c3 輸出：S=1, C=1
```

```bash
make test_equiv   # 單獨跑等價性檢查器的 13 個測試
```

#### 網頁視覺化

```bash
cd web
npm install
npm run dev     # 開啟 http://localhost:5173
```

功能：
- 電路選擇器（半加器 / 全加器 / MUX / 4-bit 加法器）
- 點擊 INPUT 節點切換 0/1，即時計算所有閘的輸出
- BFS / DFS 拓撲排序步進動畫（依排序順序高亮節點）
- Critical Path 高亮（紅色邊 + 高亮節點）
- hover tooltip 顯示閘的延遲、輸入值、輸出值

---

### 與課程的關聯總結

| 課程概念 | 在本專題中的體現 |
|----------|----------------|
| **有向無環圖（DAG）** | 電路本身的資料結構：邏輯閘為節點，連線為有向邊 |
| **拓撲排序（BFS/DFS）** | 決定閘的計算順序，兩種演算法的完整實作與比較 |
| **雜湊表（Hash Map）** | `unordered_map` 儲存訊號值與元件名稱，O(1) 查詢 |
| **平衡二元搜尋樹（BST）** | `std::map`（紅黑樹）管理訊號線名稱，維持有序以利真值表輸出 |
| **堆積與優先權佇列（Heap）** | Critical Path Top-K 搜尋的核心：`std::priority_queue`（Max-Heap）按延遲上界排序，確保前 K 個彈出的完整路徑即為最長 K 條 |
| **佇列（Queue）** | BFS 的核心資料結構，`deque` 實作 |
| **堆疊（Stack）** | DFS 的遞迴呼叫堆疊；迭代版用 `std::stack` 顯式管理 |
| **OOP / 繼承 / 多型** | `Gate` 基底類別 → `AndGate`、`OrGate` 等子類別，虛擬函式 `compute()` |
| **演算法複雜度分析** | O(V+E) 線性的理論分析與實測驗證；log-log 圖呈現 |
| **迴圈偵測（三色標記）** | DFS 的 WHITE/GRAY/BLACK 標記，back edge 偵測 |
| **統計測量** | warm-up / 正式測量 / 均值 / 標準差，避免系統排程抖動影響 |
| **迭代 vs 遞迴的實作等價性** | `topologicalSortDFSIterative()` 用 `std::stack` 取代遞迴呼叫堆疊，在深度鏈上比遞迴版快 ~4.5%（5000 閘）；實證「stack 與 recursion 等價」的理論在實作層仍有可量化的差距 |
| **實驗設計與對照分析** | 三種 DAG 結構 × 三種演算法的系統性對照；Random / Wide Fanout 下 DFS 比 BFS 快 8–15%、Deep Chain 下三者幾乎相同（±5%），符合 BFS queue 操作常數因子較高的理論預期 |

總結來說，本專題不僅實作了課程教授的多種資料結構，更透過系統性的對照實驗，驗證了這些資料結構在實務情境下的選擇取捨。對我個人而言，這個專案讓資料結構從「課本上的演算法」變成「能解釋電腦為什麼跑這麼快」的具體工具——這比通過任何單元測試都更值得記住。
