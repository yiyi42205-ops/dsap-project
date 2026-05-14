// layoutUtils.js ── 拓撲分層 layout 演算法
// 不依賴任何外部函式庫
//
// 策略：
//   1. INPUT → column 0，OUTPUT → column max，gate → 1 + max(col of preds)
//   2. 由左到右逐 column 排列：
//      每個節點的 y 順序依「所有前驅節點的平均 y」排序，
//      讓連線盡量平行、減少交叉

const COL_WIDTH  = 200;  // px，水平間距
const ROW_HEIGHT = 120;  // px，垂直間距
const OFFSET_X   = 40;
const CENTER_Y   = 400;  // 整體垂直置中基準

// buildLayout(circuitData) → { [nodeId]: {x, y} }
export function buildLayout(circuitData) {
  const { nodes, edges } = circuitData;

  // ── predecessor map: nodeId → [fromId, ...] ──────────────
  const preds = new Map(nodes.map(n => [n.id, []]));
  for (const e of edges) {
    if (preds.has(e.to)) preds.get(e.to).push(e.from);
  }

  // ── Step 1：計算每個節點的 column ──────────────────────
  const col = new Map();

  // INPUT → col 0
  for (const n of nodes) {
    if (n.type === 'INPUT') col.set(n.id, 0);
  }

  const outputIds = new Set(nodes.filter(n => n.type === 'OUTPUT').map(n => n.id));
  let maxGateCol = 0;

  // gate：沿 topo_order_bfs 計算（保證前驅已算好）
  for (const id of circuitData.topo_order_bfs) {
    if (col.has(id)) continue; // INPUT 已設
    const predCols = preds.get(id).map(pid => col.get(pid) ?? 0);
    const c = predCols.length > 0 ? Math.max(...predCols) + 1 : 1;
    col.set(id, c);
    if (c > maxGateCol) maxGateCol = c;
  }

  // OUTPUT → 最右 column
  const outputCol = maxGateCol + 1;
  for (const id of outputIds) col.set(id, outputCol);

  // ── Step 2：依 column 分組 ──────────────────────────────
  const colGroups = new Map();  // col → [nodeId, ...]
  for (const n of nodes) {
    const c = col.get(n.id) ?? 0;
    if (!colGroups.has(c)) colGroups.set(c, []);
    colGroups.get(c).push(n.id);
  }

  // ── Step 3：由左到右，每列依前驅平均 y 排序後分配 y ────
  const posY = new Map(); // nodeId → y

  // 先處理 col 0（INPUT，沒有前驅，按原始順序垂直置中）
  const col0 = colGroups.get(0) ?? [];
  assignY(col0, null, posY, CENTER_Y, ROW_HEIGHT);

  // 逐 column 排序
  const sortedCols = [...colGroups.keys()].sort((a, b) => a - b);
  for (const c of sortedCols) {
    if (c === 0) continue; // 已處理
    const ids = colGroups.get(c);

    // 計算每個節點的「前驅平均 y」作為排序鍵
    const sortKey = ids.map(id => {
      const ps = preds.get(id) ?? [];
      const knownYs = ps.map(pid => posY.get(pid)).filter(y => y !== undefined);
      const avgY = knownYs.length > 0
        ? knownYs.reduce((a, b) => a + b, 0) / knownYs.length
        : CENTER_Y;
      return { id, avgY };
    });

    sortKey.sort((a, b) => a.avgY - b.avgY);
    assignY(sortKey.map(s => s.id), null, posY, CENTER_Y, ROW_HEIGHT);
  }

  // ── Step 4：組合最終 positions ──────────────────────────
  const positions = {};
  for (const n of nodes) {
    const c = col.get(n.id) ?? 0;
    positions[n.id] = {
      x: OFFSET_X + c * COL_WIDTH,
      y: posY.get(n.id) ?? CENTER_Y,
    };
  }

  return positions;
}

// 將 ids 陣列垂直置中排列，結果寫入 posY
function assignY(ids, _unused, posY, centerY, rowHeight) {
  const total = ids.length;
  const startY = centerY - ((total - 1) * rowHeight) / 2;
  ids.forEach((id, i) => {
    posY.set(id, startY + i * rowHeight);
  });
}
