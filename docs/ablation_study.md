# Ablation Study：資料結構選擇對效能的影響

本文件記錄三組對照實驗，驗證「選用正確的資料結構」對電路模擬器效能的實際影響。
每組實驗皆附有可重現的 C++ benchmark 程式與 Python 繪圖腳本。

---

## 如何執行

```bash
# 在專案根目錄
make benchmarks        # 編譯並執行全部三個 benchmark，CSV 存入 src/benchmarks/results/
make plot              # 讀 CSV 畫圖，PNG 也存入 src/benchmarks/results/

# 或個別執行
make bench_hash
make bench_topo
make bench_ordered
```

---

## 實驗一：Hash Map vs Linear Scan 元件查詢

**檔案：** `src/benchmarks/hash_vs_linear.cpp`
**圖表：** `src/benchmarks/results/hash_vs_linear.png`

### 動機

`Circuit::addGate()` 和 `simulate()` 都需要依名稱查詢閘或訊號。
若用 `vector<pair<string, Gate*>>` 做線性掃描，每次查詢是 O(N)；
用 `unordered_map<string, Gate*>` 則是 O(1) 攤銷。

### 方法

- 規模 N = 10 / 50 / 100 / 500 / 1000 / 5000 個 gate 名稱
- 每組隨機查詢 1000 次，重複 20 輪取平均（消除系統排程抖動）
- 兩種實作：`std::unordered_map::find()` vs `vector<pair>` 線性走訪

### 複雜度分析

| 操作 | unordered_map | vector<pair> |
|------|:---:|:---:|
| 單次查詢 | O(1) 攤銷 | O(N) |
| 1000 次查詢 | O(1000) | O(1000N) |

### 預期結果

N=10 時兩者差距不大（小 N 下 hash 常數成本反而顯著）；
N ≥ 500 後 linear scan 因 O(N) 開銷快速增長，HashMap 優勢明顯。
實測 speedup 約與 N 成線性正比。

### 實測結果

> 執行 `make bench_hash` 後結果寫入 `results/hash_vs_linear.csv`。

---

## 實驗二：拓撲排序 BFS O(V+E) vs Naive 逐輪掃描 O(V²)

**檔案：** `src/benchmarks/topo_vs_naive.cpp`
**圖表：** `src/benchmarks/results/topo_vs_naive.png`

### 動機

「計算順序」的決定方式有兩種：

1. **BFS（Kahn's Algorithm）**：先算 in-degree，用 Queue 逐步取出 in-degree=0 的閘，O(V+E)。
2. **Naive 逐輪掃描**：每次從頭掃，找到第一個「所有輸入都已就緒」的閘就計算，再重頭掃，O(V²)。

Ripple Carry Adder 的 carry chain 形成一條線性依賴鏈，是放大兩者差距的理想結構：
深度（depth）= N（位元數），使 Naive 需要 O(N) 輪、每輪掃 O(N) 個閘。

### 方法

- 規模：4-bit / 8-bit / 16-bit / 32-bit Ripple Carry Adder
- Gate 陣列插入前先洗牌（`std::shuffle`），消除插入順序帶來的巧合。
- Naive 在找到第一個可算的閘後 `break` 重頭掃，保證最壞情況 O(V²)。
- 重複 80000 次計時取平均，以 ns 為單位，輸出 μs/call。

### 複雜度分析

| 規模 | gate 數 V | BFS 操作數 | Naive 操作數（估計） | 理論加速比 |
|------|:---:|:---:|:---:|:---:|
| 4-bit  |  20 | ~60   | ~200   | ~3x  |
| 8-bit  |  40 | ~120  | ~800   | ~7x  |
| 16-bit |  80 | ~240  | ~3200  | ~13x |
| 32-bit | 160 | ~480  | ~12800 | ~27x |

> BFS 操作數 ≈ V + E（每個閘 2 條輸入邊 + OR 閘 2 條 = 約 2.5V）
> Naive 操作數 ≈ V × depth = 5N × N = 5N²

### 實測結果

> 執行 `make bench_topo` 後結果寫入 `results/topo_vs_naive.csv`。

---

## 實驗三：std::map vs std::unordered_map 真值表訊號儲存

**檔案：** `src/benchmarks/ordered_vs_unordered.cpp`
**圖表：** `src/benchmarks/results/ordered_vs_unordered.png`

### 動機

電路的訊號（signals）在 C++ 中以 `name → bool` 的 map 儲存。
選用哪種 map 影響兩件事：

1. **效能**：插入 O(log N) vs O(1)；查詢 O(log N) vs O(1)
2. **正確性**：真值表的欄位是否按固定順序輸出

### 方法

- 規模 N = 10 / 50 / 100 / 500 / 1000 個訊號名稱
- 測三個場景：插入、查詢、迭代（對應真值表輸出）
- 各重複 500 輪取平均
- 附正確性展示：以 `{"Cout", "A", "B", "S", "Cin"}` 插入兩種 map，比較迭代順序

### 正確性關鍵

```
std::map 迭代順序（字典序保證）：A B Cin Cout S
std::unordered_map 迭代順序（不定）：可能是任意排列，每次執行都不同
```

真值表的欄位若順序不定，會造成不同執行的輸出不一致，難以自動化測試。
因此本專題在儲存訊號值時使用 `unordered_map`（O(1) 查詢效能），
但在產生真值表的輸入欄位標頭時，明確依 `inputSignals` 的 insertion order 輸出，
確保順序固定。

### 複雜度分析

| 操作 | std::map | std::unordered_map |
|------|:---:|:---:|
| 插入 | O(log N) | O(1) 攤銷 |
| 查詢 | O(log N) | O(1) 攤銷 |
| 迭代 | O(N)，字典序 | O(N)，順序不定 |

### 實測結果

> 執行 `make bench_ordered` 後結果寫入 `results/ordered_vs_unordered.csv`。

---

## 總結

| 實驗 | 決策 | 理由 |
|------|------|------|
| 1 | 採用 `unordered_map` 做元件查詢 | O(1) vs O(N)，N ≥ 500 後差距明顯 |
| 2 | 採用 BFS 拓撲排序，不用 Naive scan | O(V+E) vs O(V²)，carry chain 放大差距 |
| 3 | 訊號儲存用 `unordered_map`，欄位輸出用固定插入順序 | 效能 + 正確性兼顧 |
