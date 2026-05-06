# BFS vs DFS 拓撲排序：嚴謹效能分析

本文件基於 `src/benchmarks/random_circuit_gen.cpp` 的實測結果，分析 BFS（Kahn's Algorithm）
與 DFS（反向後序遍歷）在不同規模電路上的效能差異，並從記憶體存取、分支預測等角度解釋原因。

---

## 一、測試方法論

### 1.1 隨機電路生成

採用**層序法**保證生成的電路一定是合法 DAG：

```
可用訊號集 S = {主要輸入}
對每個 gate i：
    從 S 中隨機抽取 fan_in 個訊號作為輸入
    gate i 的輸出加入 S
```

參數設定：
- `fan_in avg = 3`（每個閘平均 3 個輸入）
- `n_inputs = max(3, √N)`（確保訊號池夠豐富）
- 每規模生成 10 個不同種子的電路，統計 10 個電路的均值

### 1.2 測量流程

```
for each circuit:
    1. 100 次 warm-up（讓 CPU cache 和分支預測器穩定，結果丟棄）
    2. 1000 次正式測量，記錄每次 nanoseconds
    3. 計算均值 (mean) 和標準差 (std)
```

warm-up 的必要性：C++ STL 容器（`deque`、`unordered_map`）在首次使用時有額外的
記憶體分配開銷，warm-up 讓後續測量不受首次分配影響。

---

## 二、實測結果

### 2.1 隨機電路（random_circuit_gen）

| Gate 數 | BFS mean (ns) | BFS std (ns) | DFS mean (ns) | DFS std (ns) | BFS/DFS 比值 |
|---------|:---:|:---:|:---:|:---:|:---:|
| 100     | 21,923        | 2,209        | 18,124        | 1,051        | **1.21x**    |
| 500     | 127,522       | 3,749        | 107,325       | 4,216        | **1.19x**    |
| 1,000   | 303,492       | 12,872       | 255,580       | 9,812        | **1.19x**    |
| 5,000   | 2,393,446     | 187,048      | 2,080,382     | 196,707      | **1.15x**    |
| 10,000  | 5,329,334     | 317,487      | 4,550,452     | 201,603      | **1.17x**    |

> 資料來源：`make bench_random`，每規模 10 個電路 × 1000 次測量的平均

**關鍵觀察：BFS 一致比 DFS 慢 15–21%，且比值在所有規模下保持穩定。**
這說明差異來自**常數因子**，而非演算法複雜度（兩者同為 O(V+E)）。

### 2.2 病態電路（stress tests）

| 電路 | Gate 數 | BFS 平均排序 | DFS 平均排序 | 差異 |
|------|---------|:---:|:---:|:---:|
| deep_chain（線性鏈） | 1,000 | 169.7 μs | 170.7 μs | 幾乎相同（-0.6%） |
| wide_fanout（高扇出） | 1,999 | 381.9 μs | 360.9 μs | BFS 慢 5.8% |
| dense_circuit（高密度） | 50   | 23.7 μs  | 18.9 μs  | BFS 慢 25.2% |

**有趣的例外：deep_chain 電路兩者幾乎相同。**
原因：深度線性鏈讓 BFS 每次只有 1 個元素在 deque 中，消除了 `inDegree`
遍歷的差距；DFS 則有遞迴呼叫的固定開銷。兩個效應相互抵消。

---

## 三、為什麼 DFS 比較快？

### 3.1 記憶體分配開銷

BFS 相比 DFS 需要額外維護：
1. **`inDegree` 的更新**：每次從 queue pop 後，遍歷鄰接列表，每個鄰居都要 `--inDegree[nxt]`，再 push/不push
2. **`deque` 的記憶體**：deque 在 push 時可能觸發記憶體分配（雖然 amortized O(1)）

DFS 只需：
1. `state[node] = 1` / `state[node] = 2`（寫入 unordered_map）
2. 遞迴呼叫（push/pop call stack frame）

對於 fan-in = 3 的電路（每個 gate 平均有 3 條入邊），BFS 的每個 gate 平均
需要 3 次 `inDegree` 更新，而 DFS 只需 1 次 state 變更。

### 3.2 Cache Locality 分析

#### BFS 的 cache 行為（理論分析）

BFS 按「寬度優先」處理 gate，同一深度的 gate 可能在記憶體中**不相鄰**（因為閘的
插入順序和 BFS 處理順序通常不同）。在 10,000 gate 規模下，`inDegree` map 的大小
為 ~10,000 個 entry，遠超 L1 cache（通常 32-64 KB），頻繁的 `--inDegree[nxt]`
操作會產生 cache miss。

#### DFS 的 cache 行為（理論分析）

DFS 深度優先，傾向於連續訪問相互依賴的閘。對於 fan-in = 3 的電路，相互依賴
的閘在建構時通常在 `gates` vector 中相鄰，DFS 的訪問模式更接近連續存取。

#### Valgrind/Cachegrind 指引

若需要精確的 cache miss 數據，可執行：

```bash
# 安裝 valgrind（macOS 可用 Homebrew）
brew install valgrind

# 或使用 Instruments（macOS 原生）
instruments -t "Cache Misses" ./simulator examples/stress_tests/dense_circuit.txt

# Linux 可用 perf
perf stat -e cache-misses,cache-references,branch-misses \
    ./src/benchmarks/random_circuit_gen
```

在沒有 valgrind/perf 的環境下，本文件的分析基於：
- 理論 cache line 大小（64 bytes）和容器大小的推算
- 實際測量中 BFS/DFS 比值在高 fan-in（dense_circuit）時差距更大（1.25x vs 1.17x）

### 3.3 `std::function` 的開銷

DFS 使用 `std::function<void(Gate*)>` 包裝 lambda，這有：
- 虛函式呼叫開銷（間接函式指標呼叫）
- 每次呼叫需透過 function wrapper 的 type-erasure 機制

這個開銷**對 DFS 不利**，但實測 DFS 仍更快，說明 cache/allocation 優勢壓過了
function wrapper 的負擔。

### 3.4 `std::deque` vs Call Stack

BFS 使用 `std::deque<Gate*>` 儲存待處理 gate，queue 操作需要記憶體分配。
DFS 使用系統 call stack，push/pop frame 的速度由 CPU 的 stack pointer 指令完成，
速度遠快於 heap allocation。

---

## 四、Log-Log 分析

執行 `make bench_random && make plot` 後，`results/bfs_vs_dfs_loglog.png` 會顯示：

- X 軸（log）：gate 數量 100 → 10,000
- Y 軸（log）：每次排序耗時（μs）
- Error bar：10 個隨機電路的標準差

**預期斜率 ≈ 1.0**：在 log-log 圖上，若曲線斜率接近 1，代表時間與 N 成線性正比，
符合 O(N) 複雜度。若斜率 > 1，代表隨機電路的 fan-in 導致邊數 E 相對 V 增長，
使 O(V+E) 中的 E 項主導。

---

## 五、結論

| 觀察 | BFS (Kahn's) | DFS (反向後序) |
|------|:---:|:---:|
| 時間複雜度 | O(V+E) | O(V+E) |
| 實測速度 | 基準 | **快 15–21%** |
| Stack 風險 | 無（deque 在 heap） | 有（深度 > ~10000 可能 overflow） |
| 迴圈偵測機制 | bfsOrder.size() != gates.size() | 三色標記，back edge |
| 適用場景 | 較寬的電路（高扇出） | 一般電路（較快） |

**建議**：對一般規模電路使用 DFS（速度優勢），但對深度 > 5000 的線性鏈電路，
建議使用 BFS（避免理論上的 stack overflow 風險）。本模擬器預設使用 BFS，
以安全性優先。
