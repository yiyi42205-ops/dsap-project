# 病態測試案例報告（Stress Test Report）

本文件記錄 `examples/stress_tests/` 下各病態電路的設計動機、結構說明，
以及實測結果（含時間、成功/失敗、錯誤訊息）。

---

## 一、deep_chain.txt — 1000 NOT 閘串聯

### 電路結構

```
INPUT X0
X0 → NOT_0 → X1 → NOT_1 → X2 → ... → NOT_999 → X1000
OUTPUT X1000
```

- Gate 數：1,000
- 拓撲深度：1,000（每層只有 1 個 gate）
- 邊數：999（純線性鏈）

### 設計動機：打中哪個痛點？

**DFS 遞迴深度**。這個電路的依賴鏈讓 DFS 的遞迴棧深度 = gate 數量。
本模擬器的 DFS 使用 `std::function` lambda 遞迴，每個 stack frame 包含：
- `node` 指標（8 bytes）
- lambda capture（`adj`、`state`、`hasCycle`、`depth`、`resultStack` 的引用，~48 bytes）
- 編譯器保留空間

在 macOS（預設 8MB stack），實際能承受的遞迴深度約 10,000–40,000 層（視編譯器最佳化）。
**1,000 層在所有主流平台均安全**，但清楚展現了 BFS 的結構優勢：BFS 使用 `deque`
存於 heap，深度完全不影響 stack 用量。

### 實測結果

```
$ ./simulator examples/stress_tests/deep_chain.txt
邏輯閘數量：1000

BFS 平均排序：169.685 μs/次（10,000 次測量）
DFS 平均排序：170.735 μs/次（10,000 次測量）
結果一致性：✅ 一致

BFS 排序順序：NOT_0, NOT_1, NOT_2, ..., NOT_999（嚴格線性順序）
DFS 排序順序：NOT_0, NOT_1, NOT_2, ..., NOT_999（相同，因線性鏈只有唯一拓撲序）
```

**BFS 和 DFS 幾乎一樣快（差距 < 1%）**。這與 random_circuit 測試的 15–21% 差距
形成對比。原因分析：

- BFS：線性鏈讓 deque 每次只有 1 個元素，`inDegree` 更新也只有 1 個鄰居
- DFS：遞迴呼叫 1000 次，函式呼叫開銷累積

兩個效應相互抵消，導致效能接近。

### Stack Overflow 邊界估計

若將 NOT 閘數增加到 50,000，DFS 可能 stack overflow（取決於系統 stack 大小和編譯器優化）：

```bash
# 測試更深的鏈（請先備份，可能 crash）
python3 -c "
lines = ['INPUT X0', 'OUTPUT X50000', '']
for i in range(50000): lines.append(f'GATE NOT NOT_{i} X{i} -> X{i+1}')
print('\n'.join(lines))
" > /tmp/deep50k.txt
./simulator /tmp/deep50k.txt  # 可能 segfault（DFS）
```

---

## 二、wide_fanout.txt — 高扇出電路

### 電路結構

```
INPUT A, B
A, B → AND_0..AND_999（1000 個 AND 閘並聯）→ W_0..W_999
W_0, W_1 → OR_0 → M_0
M_0, W_2 → OR_1 → M_1
...
M_997, W_999 → OR_998 → FINAL
OUTPUT FINAL
```

- Gate 數：1,999（1,000 AND + 999 OR）
- 訊號 A 和 B 各有扇出 = 1,000
- OR chain 深度 = 999

### 設計動機：打中哪個痛點？

1. **BFS 初始佇列**：所有 AND 閘的 `inDegree = 0`（輸入均為 primary signal），
   BFS 開始時佇列一次塞入 1,000 個 gate。這測試 `deque` 的 batch push 效能。

2. **高扇出 adj 遍歷**：在 BFS 的 adj 表中，訊號 A 對應的 `adj` 節點有 1,000 個。
   但 BFS 對 gate 建圖（gate → gate），primary input 不計入 gate 的 inDegree 計算，
   所以實際上 adj 結構較分散。

3. **DFS 遞迴深度**：DFS 從 AND_0 出發後，沿著 adj 進入 OR_0 → OR_1 → ... → OR_998，
   遞迴深度 = 999。與 deep_chain 類似。

### 實測結果

```
$ ./simulator examples/stress_tests/wide_fanout.txt
邏輯閘數量：1999

BFS 平均排序：381.926 μs/次
DFS 平均排序：360.886 μs/次
BFS/DFS 比值：1.058（BFS 慢 5.8%）
結果一致性：✅ 一致
```

與 random_circuit（fan-in=3）相比，wide_fanout 的 BFS/DFS 差距較小（5.8% vs 17%）。
原因：AND 閘的 adj 列表為空（AND 閘的輸出只供 OR_0 用），BFS 的 `inDegree` 更新
工作量相對少。DFS 的 OR chain 帶來固定的遞迴開銷，兩者趨近。

---

## 三、dense_circuit.txt — 高密度電路（5 層，每層 10 閘）

### 電路結構

```
10 primary inputs（IN_0..IN_9）
Layer 1（10 gates）：fan-in = 2，每個閘取 2 個相鄰 primary input
Layer 2（10 gates）：fan-in = 10，每個閘取 Layer 1 的全部 10 個輸出
Layer 3（10 gates）：fan-in = 10，取 Layer 2 全部輸出
Layer 4（10 gates）：fan-in = 10，取 Layer 3 全部輸出
Layer 5（10 gates）：fan-in = 10，取 Layer 4 全部輸出
10 outputs（R5_0..R5_9）
```

- Gate 數：50
- 邊數：20（L0→L1）+ 100（L1→L2）+ 100（L2→L3）+ 100（L3→L4）+ 100（L4→L5）= **420 edges**
- Edge/Gate 比：420/50 ≈ **8.4**（vs 典型全加器的 ~1.5）

### 設計動機：打中哪個痛點？

**O(V+E) 中的 E 項**。在 BFS 中，每次 pop 後需遍歷 adj[cur] 並更新 inDegree。
對 Layer 2-5 的 10 個 gate，每個 pop 需更新 10 個 inDegree，
比 random_circuit（平均 3 個）多 3 倍。這讓 BFS 的 `inDegree` 更新成本更高。

### 實測結果

```
$ ./simulator examples/stress_tests/dense_circuit.txt
邏輯閘數量：50

BFS 平均排序：23.689 μs/次
DFS 平均排序：18.934 μs/次
BFS/DFS 比值：1.252（BFS 慢 25.2%）
結果一致性：✅ 一致
```

**BFS 在 dense_circuit 的劣勢最明顯（25%）**，印證了高 E/V 比時
`inDegree` 頻繁更新的開銷。這是三個測試案例中 BFS/DFS 差距最大的。

---

## 四、cycle_detection_cases/ — 5 個迴圈偵測案例

所有案例均以 `./simulator <file>` 執行，預期輸出：
`錯誤：電路中存在迴圈（BFS 偵測）！`

### 4.1 simple_loop.txt — 2-gate 互環

**結構**：NOT_A(wire_B → wire_A) ↔ NOT_B(wire_A → wire_B)

| 演算法 | 偵測機制 | 實測 |
|--------|----------|------|
| BFS | 兩閘 inDegree 永不為 0，bfsOrder.size()=0 ≠ gates.size()=2 | ✅ 偵測到 |
| DFS | 從 NOT_A 出發(GRAY)→ NOT_B(GRAY)→ adj[NOT_B] 含 NOT_A(GRAY) → back edge | ✅ 偵測到 |

### 4.2 self_loop.txt — 自環

**結構**：`GATE AND SELF_AND wire_out wire_out -> wire_out`

| 演算法 | 偵測機制 | 實測 |
|--------|----------|------|
| BFS | `signalSource["wire_out"] = SELF_AND`，所以 `inDegree[SELF_AND]++` → indegree=2，永不入佇列 | ✅ 偵測到 |
| DFS | `adj[SELF_AND]` 包含 SELF_AND 自身，訪問自身時已是 GRAY → back edge | ✅ 偵測到 |

> 自環是最小的迴圈，長度 = 1，需要特別確認 `signalSource` 的正確建構。

### 4.3 triple_ring.txt — 3-gate 環

**結構**：G_AB → G_BC → G_CA → G_AB（循環）

| 演算法 | 偵測機制 | 實測 |
|--------|----------|------|
| BFS | 三閘 inDegree 各為 2，均不入佇列 | ✅ 偵測到 |
| DFS | 三色標記：GRAY → GRAY → GRAY → back edge | ✅ 偵測到 |

### 4.4 compound_cycle.txt — 合法前綴 + 獨立迴圈

**結構**：3 個合法閘（G1, G2, G3）+ 2 個迴圈閘（CYC_A, CYC_B）

| 演算法 | 偵測機制 | 實測 |
|--------|----------|------|
| BFS | G1/G2/G3 正常排序（bfsOrder 有 3 個），但 CYC_A/CYC_B 不進佇列 → bfsOrder.size()=3 ≠ 5 | ✅ 偵測到 |
| DFS | 訪問完 G1/G2/G3 後，啟動對 CYC_A 的 DFS，發現 back edge | ✅ 偵測到 |

> 這個案例驗證：即使電路有一部分合法，整體仍必須偵測到迴圈。

### 4.5 nested_cycles.txt — 兩個獨立迴圈

**結構**：迴圈 1（C1_A ↔ C1_B）和 迴圈 2（C2_AB → C2_BC → C2_CA → 循環）

| 演算法 | 偵測機制 | 實測 |
|--------|----------|------|
| BFS | 5 個閘均 inDegree > 0，bfsOrder.size()=0 ≠ 5 | ✅ 偵測到 |
| DFS | 外層 for loop 從不同連通分量啟動 DFS，兩個子圖各自偵測到 back edge | ✅ 偵測到 |

> 關鍵點：DFS 的外層迴圈 `for (auto& g : gates) if (state[g] == 0) dfs(g)`
> 確保不連通子圖也會被完整搜尋。

### 實測匯總

```bash
$ for f in examples/stress_tests/cycle_detection_cases/*.txt; do
    echo -n "$f: "
    ./simulator "$f" 2>&1 | grep "迴圈" || echo "MISS (BUG!)"
  done

examples/.../compound_cycle.txt:   錯誤：電路中存在迴圈（BFS 偵測）！  ✅
examples/.../nested_cycles.txt:    錯誤：電路中存在迴圈（BFS 偵測）！  ✅
examples/.../self_loop.txt:        錯誤：電路中存在迴圈（BFS 偵測）！  ✅
examples/.../simple_loop.txt:      錯誤：電路中存在迴圈（BFS 偵測）！  ✅
examples/.../triple_ring.txt:      錯誤：電路中存在迴圈（BFS 偵測）！  ✅
```

**所有 5 個迴圈案例均正確偵測。**

---

## 五、壓力測試結論

| 測試案例 | Gate 數 | 打中的痛點 | BFS 成功 | DFS 成功 | BFS/DFS 時間比 |
|----------|---------|-----------|:---:|:---:|:---:|
| deep_chain | 1,000 | DFS 遞迴深度 | ✅ | ✅ | 0.99x（幾乎相同）|
| wide_fanout | 1,999 | 高扇出 + 大佇列 | ✅ | ✅ | 1.06x |
| dense_circuit | 50 | 高 E/V 比（edge density）| ✅ | ✅ | 1.25x |
| 5 個迴圈案例 | 2–5 | 各種迴圈拓撲 | ✅ 均偵測 | ✅ 均偵測 | N/A |
