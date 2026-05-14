# 數位邏輯電路模擬器 (Digital Logic Circuit Simulator)

> **換題說明：** 在進一步研究後，我發現數位邏輯電路模擬器能更完整地運用本課程教授的多種資料結構（DAG、拓撲排序、BST、Heap、Hash Map），同時也與我未來想深入學習的計算機硬體領域密切相關，因此決定更換主題。

## Proposal Report

### 動機與目標

電腦的一切運算都建立在最基礎的邏輯閘之上：AND、OR、NOT。從這些簡單的元件出發，可以組裝出加法器、多工器，甚至整個 CPU。然而，在學習資料結構的過程中，我逐漸好奇：「這些抽象的資料結構，能不能直接對應到硬體的運作方式？」

本專題的目標是開發一個 C++ 命令列工具，讓使用者以文字描述一個數位邏輯電路（指定邏輯閘與連線方式），程式會自動分析電路結構、計算輸出結果，並產生完整的真值表。透過這個專題，我希望將課程所學的資料結構應用於模擬真實硬體的運作邏輯。

### 競品比較

| 項目 | Logisim | CircuitVerse | 本專題 |
|------|---------|--------------|--------|
| 類型 | Java 桌面應用程式 | 線上網頁模擬器 | C++ 命令列工具 |
| 使用方式 | 圖形拖拉介面 | 圖形拖拉介面 | 文字描述檔定義電路 |
| 開源 | ✅ | ✅ | ✅ |
| 演算法過程可視化 | 黑箱處理，使用者看不到排序過程 | 黑箱處理，使用者看不到排序過程 | **差異化定位**：明確輸出 BFS / DFS 拓撲排序的逐步計算順序與 DAG 結構，讓使用者看見演算法如何驅動模擬 |
| 關鍵路徑分析 | ❌ | ❌ | ✅ 計算最長延遲路徑 |
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

### 遇到的困難

1. **迴圈偵測的設計**：DFS 版需要區分「訪問中」和「已完成」兩種狀態才能正確偵測迴圈（back edge），一開始只用 visited 布林值導致誤判。後來改用三態（未訪問 / 訪問中 / 已完成）解決。

2. **效能測量的穩定性**：單次執行時間太短（微秒級），受系統排程影響大。改為重複執行 10,000 次取平均後，數據穩定。

3. **4-bit 加法器的 DAG 深度**：4 個全加器串接形成較長的依賴鏈（carry 必須逐級傳播），拓撲排序的結果對計算順序影響較大，這也是為什麼規模越大 BFS 和 DFS 的差異越明顯。

### 下一步計畫

1. **關鍵路徑分析（Critical Path）**：為每個邏輯閘加入傳播延遲值，使用 DAG 最長路徑演算法 + Priority Queue 找出電路的關鍵路徑，計算最大時脈頻率。
2. **互動式視覺化介面**：以 React 實作網頁版電路視覺化（已有原型），支援即時切換輸入、動態訊號傳播動畫、拓撲排序步進展示。
3. **更大規模的效能測試**：自動生成 100 閘、1000 閘的隨機電路，觀察 BFS vs DFS 在大規模下的效能差異曲線。
4. **錄製 Demo 影片**：展示電路模擬功能 + 效能比較結果 + 視覺化介面。

---

## Final Report

### 專案說明

本專題實作了一個 C++ 命令列數位邏輯電路模擬器，以「用電路學資料結構」為核心定位：
電路本身是一個 **有向無環圖（DAG）**，模擬器透過**拓撲排序**決定邏輯閘的計算順序，
讓使用者能清楚看到演算法如何驅動硬體模擬。

**最終實作的功能：**

| 功能 | 說明 |
|------|------|
| 6 種邏輯閘 | AND / OR / NOT / XOR / NAND / NOR，OOP 繼承 + 虛擬函式 |
| 電路描述檔解析 | 自訂 `.txt` 格式，支援 INPUT / OUTPUT / GATE 關鍵字 |
| BFS 拓撲排序 | Kahn's Algorithm，O(V+E)，deque + inDegree map |
| DFS 拓撲排序 | 反向後序遍歷，O(V+E)，三色標記（WHITE/GRAY/BLACK） |
| 迴圈偵測 | BFS 和 DFS 均支援，5 種不同拓撲的迴圈測試 |
| 真值表生成 | 窮舉 2^n 種輸入組合，自動計算並印出 |
| **關鍵路徑分析（Critical Path）** | DAG DP 求最長延遲 + Max-Heap Top-K 反向路徑搜尋；各閘有實際傳播延遲（NOT=1、AND/OR/NAND/NOR=2、XOR=3 ns）；輸出路徑序列、總延遲、最高工作頻率 |
| **JSON 匯出** | `--export-json <path>` 將電路的 nodes / edges / 拓撲序 / Critical Paths / 真值表匯出為 JSON，供前端視覺化使用；自行實作 JSON writer，無第三方依賴 |
| `--trace` 模式 | 逐步印出 BFS / DFS 排序的每一步決策，以及 Critical Path DP 計算過程 |
| `--critical-path K` | 指定回傳前 K 條最長路徑（預設 K=3） |
| 效能比較 | BFS vs DFS 拓撲排序時間，含平均值和標準差 |
| Ablation Study | 3 組資料結構對照實驗（Hash Map / 拓撲排序 / Map 選擇）|
| 嚴謹 benchmark | 隨機 DAG，100 warm-up + 1000 測量，log-log 圖 |
| 病態測試 | deep_chain / wide_fanout / dense_circuit + 5 個迴圈案例 |

---

### 使用方式

#### 編譯

```bash
make          # 編譯主程式（生成 ./simulator）
make test     # 執行所有 unit tests（15 個測試）
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

`--trace` 模式下會逐步印出 BFS 排序、DFS 排序，以及 Critical Path DP 計算每個節點延遲的過程：

```
[CP Trace] DP 計算各閘最長到達延遲（拓撲順序）：
  XOR_1 (XOR, delay=3)  incoming_max=0  dist=3
  AND_1 (AND, delay=2)  incoming_max=0  dist=2
  XOR_2 (XOR, delay=3)  incoming_max=3  dist=6
  AND_2 (AND, delay=2)  incoming_max=3  dist=5
  OR_1 (OR, delay=2)  incoming_max=5  dist=7
```

> 注意：`--trace` 模式下跳過真值表和效能比較，建議用於小型電路（< 20 閘）。

#### JSON 匯出

```bash
./simulator --export-json output/full_adder.json src/circuits/full_adder.txt
# 輸出：Exported to output/full_adder.json
```

匯出 JSON 結構：
- `nodes`：INPUT / 邏輯閘 / OUTPUT 節點，含 id、type、delay
- `edges`：閘間有向邊（中間訊號線 W1/W2/... 折疊為直接邊）
- `topo_order_bfs` / `topo_order_dfs`：兩種排序結果的節點 id 序列
- `critical_paths`：Top-K 最長路徑，含 rank / delay / max_freq_mhz / path
- `truth_table`：所有 2^n 種輸入組合的模擬結果

可搭配 `--critical-path K` 控制匯出幾條路徑：

```bash
./simulator --export-json output/full_adder.json --critical-path 5 src/circuits/full_adder.txt
```

#### 執行 Benchmark（任務 D / B）

```bash
make bench_random    # 嚴謹 BFS vs DFS（100 warm-up + 1000 測量）
make benchmarks      # 執行 Ablation Study 三組實驗
make plot            # 生成所有圖表（含 log-log 圖）
```

#### 執行病態測試

```bash
# 1000 NOT 串聯（deep chain）
./simulator examples/stress_tests/deep_chain.txt

# 1999 閘高扇出
./simulator examples/stress_tests/wide_fanout.txt

# 50 閘高密度（edge/gate ≈ 8.4）
./simulator examples/stress_tests/dense_circuit.txt

# 迴圈偵測（應輸出 ERROR）
./simulator examples/stress_tests/cycle_detection_cases/simple_loop.txt
./simulator examples/stress_tests/cycle_detection_cases/self_loop.txt
./simulator examples/stress_tests/cycle_detection_cases/triple_ring.txt
./simulator examples/stress_tests/cycle_detection_cases/compound_cycle.txt
./simulator examples/stress_tests/cycle_detection_cases/nested_cycles.txt
```

---

### 與課程的關聯總結

| 課程概念 | 在本專題中的體現 |
|----------|----------------|
| **有向無環圖（DAG）** | 電路本身的資料結構：邏輯閘為節點，連線為有向邊 |
| **拓撲排序（BFS/DFS）** | 決定閘的計算順序，兩種演算法的完整實作與比較 |
| **雜湊表（Hash Map）** | `unordered_map` 儲存訊號值，O(1) 查詢；Ablation Study 實驗一 |
| **平衡二元搜尋樹（BST）** | `map` vs `unordered_map` 的正確性與效能取捨；Ablation Study 實驗三 |
| **堆積與優先權佇列（Heap）** | Critical Path Top-K 搜尋的核心：`std::priority_queue`（Max-Heap）按延遲上界排序，確保前 K 個彈出的完整路徑即為最長 K 條 |
| **佇列（Queue）** | BFS 的核心資料結構，`deque` 實作 |
| **堆疊（Stack）** | DFS 的遞迴呼叫堆疊；結果反轉用 `stack<Gate*>` |
| **OOP / 繼承 / 多型** | `Gate` 基底類別 → `AndGate`、`OrGate` 等子類別，虛擬函式 `compute()` |
| **演算法複雜度分析** | O(V+E) vs O(V²) 的理論分析與實測驗證；log-log 圖呈現 |
| **迴圈偵測（三色標記）** | DFS 的 WHITE/GRAY/BLACK 標記，back edge 偵測 |
| **統計測量** | warm-up / 正式測量 / 均值 / 標準差，避免系統排程抖動影響 |
