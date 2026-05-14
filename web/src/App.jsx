import { useState, useEffect, useCallback, useRef } from 'react';
import CircuitViewer from './CircuitViewer.jsx';
import ControlPanel from './ControlPanel.jsx';
import { propagateSignals } from './gateLogic.js';
import './App.css';

const CIRCUITS = [
  { label: 'Half Adder',  file: 'half_adder.json' },
  { label: 'Full Adder',  file: 'full_adder.json' },
  { label: '2-to-1 MUX', file: 'mux_2to1.json' },
  { label: '4-bit Adder', file: '4bit_adder.json' },
];

export default function App() {
  const [circuitIdx, setCircuitIdx] = useState(0);
  const [circuitData, setCircuitData] = useState(null);

  // inputValues: { [nodeId]: boolean }
  const [inputValues, setInputValues] = useState({});
  // signalMap: Map<nodeId, boolean>
  const [signalMap, setSignalMap] = useState(null);

  // 動畫
  const [highlightedNodeId, setHighlightedNodeId] = useState(null);
  const [isAnimating, setIsAnimating] = useState(false);
  const animTimers = useRef([]);

  // Critical Path
  const [selectedCriticalPathIdx, setSelectedCriticalPathIdx] = useState(null);
  const [criticalEdgeSet, setCriticalEdgeSet] = useState(null);

  // ── 載入電路 ─────────────────────────────────────────────
  useEffect(() => {
    const file = CIRCUITS[circuitIdx].file;
    fetch(`/circuits/${file}`)
      .then(r => r.json())
      .then(data => {
        setCircuitData(data);
        const initInputs = {};
        for (const n of data.nodes) {
          if (n.type === 'INPUT') initInputs[n.id] = false;
        }
        setInputValues(initInputs);
        setSelectedCriticalPathIdx(null);
        setCriticalEdgeSet(null);
        stopAnimation();
      });
  }, [circuitIdx]);

  // ── 訊號傳播 ─────────────────────────────────────────────
  useEffect(() => {
    if (!circuitData) return;
    const map = propagateSignals(circuitData, inputValues);
    setSignalMap(map);
  }, [circuitData, inputValues]);

  // ── INPUT toggle ─────────────────────────────────────────
  const handleInputToggle = useCallback((nodeId) => {
    setInputValues(prev => ({ ...prev, [nodeId]: !prev[nodeId] }));
  }, []);

  // ── 動畫 ─────────────────────────────────────────────────
  function stopAnimation() {
    animTimers.current.forEach(clearTimeout);
    animTimers.current = [];
    setHighlightedNodeId(null);
    setIsAnimating(false);
  }

  function playAnimation(order) {
    if (isAnimating) return;
    stopAnimation();
    setIsAnimating(true);
    order.forEach((nodeId, i) => {
      const t = setTimeout(() => {
        setHighlightedNodeId(nodeId);
        if (i === order.length - 1) {
          const t2 = setTimeout(() => {
            setHighlightedNodeId(null);
            setIsAnimating(false);
          }, 600);
          animTimers.current.push(t2);
        }
      }, i * 600);
      animTimers.current.push(t);
    });
  }

  const handlePlayBFS = () => circuitData && playAnimation(circuitData.topo_order_bfs);
  const handlePlayDFS = () => circuitData && playAnimation(circuitData.topo_order_dfs);

  // ── Critical Path 選取 ───────────────────────────────────
  const handleSelectCriticalPath = useCallback((idx) => {
    if (!circuitData) return;
    if (idx === selectedCriticalPathIdx) {
      setSelectedCriticalPathIdx(null);
      setCriticalEdgeSet(null);
      return;
    }
    const cp = circuitData.critical_paths[idx];
    const edgeSet = new Set();
    for (let i = 0; i < cp.path.length - 1; i++) {
      edgeSet.add(`${cp.path[i]}__${cp.path[i + 1]}`);
    }
    setSelectedCriticalPathIdx(idx);
    setCriticalEdgeSet(edgeSet);
  }, [circuitData, selectedCriticalPathIdx]);

  // ── Render ───────────────────────────────────────────────
  return (
    <div className="app-root">
      <header className="app-header">
        <span className="app-title">數位邏輯電路模擬器</span>
        <select
          className="circuit-select"
          value={circuitIdx}
          onChange={e => setCircuitIdx(Number(e.target.value))}
        >
          {CIRCUITS.map((c, i) => (
            <option key={i} value={i}>{c.label}</option>
          ))}
        </select>
        {circuitData && (
          <span className="circuit-info">
            {circuitData.nodes.filter(n => n.type === 'INPUT').length} inputs ·{' '}
            {circuitData.nodes.filter(n => !['INPUT', 'OUTPUT'].includes(n.type)).length} gates ·{' '}
            {circuitData.edges.length} edges
          </span>
        )}
      </header>

      <div className="app-body">
        <div className="viewer-area">
          <CircuitViewer
            circuitData={circuitData}
            signalMap={signalMap}
            highlightedNodeId={highlightedNodeId}
            criticalEdgeSet={criticalEdgeSet}
            onInputToggle={handleInputToggle}
          />
        </div>
        <ControlPanel
          circuitData={circuitData}
          onPlayBFS={handlePlayBFS}
          onPlayDFS={handlePlayDFS}
          isAnimating={isAnimating}
          onSelectCriticalPath={handleSelectCriticalPath}
          selectedCriticalPathIdx={selectedCriticalPathIdx}
        />
      </div>
    </div>
  );
}
