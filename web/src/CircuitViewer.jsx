import { useEffect } from 'react';
import ReactFlow, {
  Background,
  Controls,
  useNodesState,
  useEdgesState,
  MarkerType,
} from 'reactflow';
import 'reactflow/dist/style.css';

import { buildLayout } from './layoutUtils.js';

// ── node 樣式 ────────────────────────────────────────────────
function nodeStyle(type, highlighted, isInput) {
  const base = {
    padding: '6px 10px',
    borderRadius: 6,
    border: '2px solid #aaa',
    background: '#f5f5f5',
    fontSize: 13,
    fontFamily: 'monospace',
    minWidth: 80,
    textAlign: 'center',
    cursor: isInput ? 'pointer' : 'default',
    userSelect: 'none',
  };
  if (highlighted) {
    return { ...base, background: '#3b82f6', color: '#fff', border: '2px solid #1d4ed8' };
  }
  if (type === 'INPUT')  return { ...base, background: '#e0f2fe', border: '2px solid #38bdf8' };
  if (type === 'OUTPUT') return { ...base, background: '#dcfce7', border: '2px solid #4ade80' };
  return base;
}

// ── hover tooltip 文字 ───────────────────────────────────────
function buildTooltip(n, signalMap, incomingEdges) {
  const signal = signalMap?.get(n.id) ?? false;
  const val = signal ? '1' : '0';
  if (n.type === 'INPUT' || n.type === 'OUTPUT') {
    return `${n.id} / 目前值: ${val}`;
  }
  const preds = incomingEdges.get(n.id) ?? [];
  const inputVals = preds.map(pid => (signalMap?.get(pid) ?? false) ? '1' : '0');
  return `${n.id} (${n.type}) / 延遲: ${n.delay}ns / 輸入: [${inputVals.join(', ')}] / 輸出: ${val}`;
}

// ── circuitData → React Flow nodes ──────────────────────────
function buildRFNodes(circuitData, signalMap, highlightedId, onInputToggle) {
  const positions = buildLayout(circuitData);

  // 建立 incomingEdges: nodeId → [fromId, ...]
  const incomingEdges = new Map(circuitData.nodes.map(n => [n.id, []]));
  for (const e of circuitData.edges) {
    if (incomingEdges.has(e.to)) incomingEdges.get(e.to).push(e.from);
  }

  return circuitData.nodes.map(n => {
    const signal = signalMap?.get(n.id) ?? false;
    const isInput = n.type === 'INPUT';
    const label = isInput
      ? `${n.id}\n[${signal ? '1' : '0'}]`
      : n.type === 'OUTPUT'
      ? `${n.id}\n= ${signal ? '1' : '0'}`
      : `${n.id}\n(${n.type})`;

    const tooltip = buildTooltip(n, signalMap, incomingEdges);

    return {
      id: n.id,
      position: positions[n.id] ?? { x: 0, y: 0 },
      data: {
        label: (
          <div
            title={tooltip}
            onClick={isInput ? () => onInputToggle(n.id) : undefined}
          >
            {label.split('\n').map((l, i) => <div key={i}>{l}</div>)}
          </div>
        ),
      },
      style: nodeStyle(n.type, n.id === highlightedId, isInput),
    };
  });
}

// ── circuitData → React Flow edges ──────────────────────────
function buildRFEdges(circuitData, signalMap, criticalEdgeSet) {
  return circuitData.edges.map(e => {
    const edgeId = `${e.from}__${e.to}`;
    const isCritical = criticalEdgeSet?.has(edgeId);
    const signal = signalMap?.get(e.from) ?? false;
    const color = isCritical ? '#ef4444' : signal ? '#ef4444' : '#9ca3af';
    const strokeWidth = isCritical ? 4 : 2;
    return {
      id: edgeId,
      source: e.from,
      target: e.to,
      animated: false,
      style: { stroke: color, strokeWidth },
      markerEnd: { type: MarkerType.ArrowClosed, color },
    };
  });
}

// ────────────────────────────────────────────────────────────
export default function CircuitViewer({
  circuitData,
  signalMap,
  highlightedNodeId,
  criticalEdgeSet,
  onInputToggle,
}) {
  const [nodes, setNodes, onNodesChange] = useNodesState([]);
  const [edges, setEdges, onEdgesChange] = useEdgesState([]);

  useEffect(() => {
    if (!circuitData) return;
    setNodes(buildRFNodes(circuitData, signalMap, highlightedNodeId, onInputToggle));
    setEdges(buildRFEdges(circuitData, signalMap, criticalEdgeSet));
  }, [circuitData, signalMap, highlightedNodeId, criticalEdgeSet, onInputToggle]);

  return (
    <div style={{ width: '100%', height: '100%' }}>
      <ReactFlow
        nodes={nodes}
        edges={edges}
        onNodesChange={onNodesChange}
        onEdgesChange={onEdgesChange}
        fitView
        fitViewOptions={{ padding: 0.2 }}
        nodesDraggable={false}
        nodesConnectable={false}
        elementsSelectable={false}
      >
        <Background color="#e5e7eb" gap={20} />
        <Controls showInteractive={false} />
      </ReactFlow>
    </div>
  );
}
