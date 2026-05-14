// layoutUtils.js ── 拓撲分層 layout 演算法
// 不依賴任何外部函式庫
//
// 策略：
//   INPUT  → column 0
//   OUTPUT → column max
//   gate   → 1 + max(column of all predecessor nodes)
// 同 column 內節點垂直等間距排列

const COL_WIDTH  = 180;  // px，水平間距
const ROW_HEIGHT = 90;   // px，垂直間距
const OFFSET_X   = 40;
const OFFSET_Y   = 40;

// buildLayout(circuitData) → { [nodeId]: {x, y} }
export function buildLayout(circuitData) {
  const { nodes, edges } = circuitData;
  const nodeById = new Map(nodes.map(n => [n.id, n]));

  // 建立 predecessor map: nodeId → [predecessorId, ...]
  const preds = new Map(nodes.map(n => [n.id, []]));
  for (const e of edges) {
    if (preds.has(e.to)) preds.get(e.to).push(e.from);
  }

  // 計算每個節點的 column
  const col = new Map();
  const maxCol = { val: 0 };

  // INPUT 固定 col 0
  for (const n of nodes) {
    if (n.type === 'INPUT') col.set(n.id, 0);
  }

  // OUTPUT 先標記，最後設為 max+1
  const outputIds = new Set(nodes.filter(n => n.type === 'OUTPUT').map(n => n.id));

  // 用 topo_order_bfs 計算 gate 的 column（保證前驅已算好）
  for (const id of circuitData.topo_order_bfs) {
    if (col.has(id)) continue; // INPUT 已設
    const predCols = preds.get(id).map(pid => col.get(pid) ?? 0);
    const c = predCols.length > 0 ? Math.max(...predCols) + 1 : 1;
    col.set(id, c);
    if (c > maxCol.val) maxCol.val = c;
  }

  // OUTPUT 節點放在最右 column（max gate col + 1）
  const outputCol = maxCol.val + 1;
  for (const id of outputIds) {
    col.set(id, outputCol);
  }

  // 依 column 分組，計算每 column 的節點列表
  const colGroups = new Map();
  for (const n of nodes) {
    const c = col.get(n.id) ?? 0;
    if (!colGroups.has(c)) colGroups.set(c, []);
    colGroups.get(c).push(n.id);
  }

  // 計算 (x, y) 座標
  const positions = {};
  for (const [c, ids] of colGroups) {
    const totalH = (ids.length - 1) * ROW_HEIGHT;
    ids.forEach((id, i) => {
      positions[id] = {
        x: OFFSET_X + c * COL_WIDTH,
        y: OFFSET_Y + i * ROW_HEIGHT - totalH / 2 + 300, // 垂直置中在 y=300
      };
    });
  }

  return positions;
}
