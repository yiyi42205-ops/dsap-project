// gateLogic.js ── 純函式，計算邏輯閘輸出值
// computeGate(type, inputs) → boolean
// type: 'AND' | 'OR' | 'NOT' | 'XOR' | 'NAND' | 'NOR' | 'INPUT' | 'OUTPUT'
// inputs: boolean[]

export function computeGate(type, inputs) {
  switch (type) {
    case 'AND':
      return inputs.every(Boolean);
    case 'OR':
      return inputs.some(Boolean);
    case 'NOT':
      return !inputs[0];
    case 'XOR':
      return inputs.reduce((acc, v) => acc ^ v, false);
    case 'NAND':
      return !inputs.every(Boolean);
    case 'NOR':
      return !inputs.some(Boolean);
    case 'INPUT':
    case 'OUTPUT':
      return inputs[0] ?? false;
    default:
      console.warn('Unknown gate type:', type);
      return false;
  }
}

// propagateSignals(circuitData, inputValues) → Map<nodeId, boolean>
// circuitData: 從 JSON 載入的電路資料
// inputValues: { [inputNodeId]: boolean }  e.g. { IN_A: true, IN_B: false }
// 回傳每個 node id 的訊號值
export function propagateSignals(circuitData, inputValues) {
  const signalMap = new Map();

  // 初始化 INPUT 節點
  for (const node of circuitData.nodes) {
    if (node.type === 'INPUT') {
      signalMap.set(node.id, inputValues[node.id] ?? false);
    }
  }

  // 建立 edge lookup: to → [from, ...]
  const incomingEdges = new Map();
  for (const edge of circuitData.edges) {
    if (!incomingEdges.has(edge.to)) incomingEdges.set(edge.to, []);
    incomingEdges.get(edge.to).push(edge.from);
  }

  // 建立 nodeId → node 查找
  const nodeById = new Map(circuitData.nodes.map(n => [n.id, n]));

  // 依 topo_order_bfs 順序計算（INPUT 已算好，跳過）
  // topo_order_bfs 不含 OUTPUT 節點，最後再補算
  const topoOrder = circuitData.topo_order_bfs;

  for (const id of topoOrder) {
    if (signalMap.has(id)) continue; // INPUT 已設定
    const node = nodeById.get(id);
    if (!node) continue;
    const preds = incomingEdges.get(id) ?? [];
    const inputs = preds.map(fromId => signalMap.get(fromId) ?? false);
    signalMap.set(id, computeGate(node.type, inputs));
  }

  // 補算 OUTPUT 節點（直接取其輸入邊的值）
  for (const node of circuitData.nodes) {
    if (node.type === 'OUTPUT') {
      const preds = incomingEdges.get(node.id) ?? [];
      const val = preds.length > 0 ? (signalMap.get(preds[0]) ?? false) : false;
      signalMap.set(node.id, val);
    }
  }

  return signalMap;
}

// ── 簡易驗證（開發用，不影響生產）────────────────────────────
if (import.meta.env?.DEV) {
  // AND
  console.assert(computeGate('AND', [true, true]) === true,  'AND(1,1)=1');
  console.assert(computeGate('AND', [true, false]) === false, 'AND(1,0)=0');
  // OR
  console.assert(computeGate('OR', [false, false]) === false, 'OR(0,0)=0');
  console.assert(computeGate('OR', [false, true]) === true,  'OR(0,1)=1');
  // NOT
  console.assert(computeGate('NOT', [false]) === true,  'NOT(0)=1');
  console.assert(computeGate('NOT', [true])  === false, 'NOT(1)=0');
  // XOR
  console.assert(computeGate('XOR', [true, true])  === false, 'XOR(1,1)=0');
  console.assert(computeGate('XOR', [true, false]) === true,  'XOR(1,0)=1');
  // NAND / NOR
  console.assert(computeGate('NAND', [true, true])   === false, 'NAND(1,1)=0');
  console.assert(computeGate('NOR',  [false, false]) === true,  'NOR(0,0)=1');
  console.log('[gateLogic] all assertions passed');
}
