import { useCallback, useEffect } from 'react';
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
function nodeStyle(type, signal, highlighted, isInput) {
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

// circuitData → React Flow nodes array
function buildRFNodes(circuitData, signalMap, highlightedId, onInputToggle) {
  const positions = buildLayout(circuitData);
  return circuitData.nodes.map(n => {
    const signal = signalMap?.get(n.id) ?? false;
    const isInput = n.type === 'INPUT';
    const label = isInput
      ? `${n.id}\n[${signal ? '1' : '0'}]`
      : n.type === 'OUTPUT'
      ? `${n.id}\n= ${signal ? '1' : '0'}`
      : `${n.id}\n(${n.type})`;

    return {
      id: n.id,
      position: positions[n.id] ?? { x: 0, y: 0 },
      data: {
        label: (
          <div onClick={isInput ? () => onInputToggle(n.id) : undefined}>
            {label.split('\n').map((l, i) => <div key={i}>{l}</div>)}
          </div>
        ),
      },
      style: nodeStyle(n.type, signal, n.id === highlightedId, isInput),
    };
  });
}

// circuitData + signalMap → React Flow edges array
function buildRFEdges(circuitData, signalMap, criticalEdgeSet) {
  return circuitData.edges.map((e, i) => {
    const edgeId = `${e.from}__${e.to}`;
    const isCritical = criticalEdgeSet?.has(edgeId);
    // signal on this edge = value of the source node
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

// ─────────────────────────────────────────────────────────────
export default function CircuitViewer({
  circuitData,
  signalMap,
  highlightedNodeId,
  criticalEdgeSet,
  onInputToggle,
}) {
  const [nodes, setNodes, onNodesChange] = useNodesState([]);
  const [edges, setEdges, onEdgesChange] = useEdgesState([]);

  // 每次 circuitData / signalMap / highlight / critical 變動時重算
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
