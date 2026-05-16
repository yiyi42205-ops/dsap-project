import { useState, useEffect, useCallback, useRef } from 'react';
import CircuitViewer from './CircuitViewer.jsx';
import ControlPanel from './ControlPanel.jsx';
import QMViewer from './QMViewer.jsx';
import { propagateSignals } from './gateLogic.js';
import './App.css';

const CIRCUITS = [
  { label: 'Half Adder',  file: 'half_adder.json',  desc: '兩個 1-bit 輸入相加，輸出 sum 與 carry' },
  { label: 'Full Adder',  file: 'full_adder.json',  desc: '3 輸入 1-bit 加法器，含進位輸入' },
  { label: '2-to-1 MUX', file: 'mux_2to1.json',    desc: '由 SEL 訊號選擇輸出 A 或 B' },
  { label: '4-bit Adder', file: '4bit_adder.json',  desc: '4-bit 加法器，4 個 full adder 串接' },
];

export default function App() {
  const [mode, setMode] = useState('circuit'); // 'circuit' | 'qm'
  const [circuitIdx, setCircuitIdx] = useState(0);
  const [circuitData, setCircuitData] = useState(null);
  const [inputValues, setInputValues] = useState({});
  const [signalMap, setSignalMap] = useState(null);
  const [highlightedNodeId, setHighlightedNodeId] = useState(null);
  const [isAnimating, setIsAnimating] = useState(false);
  const animTimers = useRef([]);
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
    setSignalMap(propagateSignals(circuitData, inputValues));
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
      {/* 頂部 header */}
      <header className="app-header">
        <div className="app-title-group">
          <span className="app-title">數位邏輯電路模擬器</span>
          <span className="app-subtitle">將電路視為 DAG，以拓撲排序決定邏輯閘的計算順序</span>
        </div>

        {/* 模式 Tab */}
        <div className="mode-tabs">
          <button
            className={`mode-tab${mode === 'circuit' ? ' mode-tab--active' : ''}`}
            onClick={() => setMode('circuit')}
          >
            電路模擬
          </button>
          <button
            className={`mode-tab${mode === 'qm' ? ' mode-tab--active' : ''}`}
            onClick={() => setMode('qm')}
          >
            真值表化簡
          </button>
        </div>

        {mode === 'circuit' && (
          <div className="app-controls">
            <select
              className="circuit-select"
              value={circuitIdx}
              onChange={e => setCircuitIdx(Number(e.target.value))}
            >
              {CIRCUITS.map((c, i) => (
                <option key={i} value={i}>{c.label}</option>
              ))}
            </select>
            <span className="circuit-desc">{CIRCUITS[circuitIdx].desc}</span>
          </div>
        )}
        {mode === 'circuit' && circuitData && (
          <span className="circuit-info">
            {circuitData.nodes.filter(n => n.type === 'INPUT').length} inputs ·{' '}
            {circuitData.nodes.filter(n => !['INPUT', 'OUTPUT'].includes(n.type)).length} gates ·{' '}
            {circuitData.edges.length} edges
          </span>
        )}
      </header>

      {/* 主體 */}
      <div className="app-body">
        {mode === 'qm' ? (
          <QMViewer />
        ) : (
          <>
            <div className="viewer-area">
              {/* 互動提示框 */}
              <div className="hint-box">
                💡 點擊左側 <strong>INPUT</strong> 節點切換 0/1，觀察訊號傳播。紅色邊代表訊號為 1。
              </div>
              <div className="viewer-canvas">
                <CircuitViewer
                  circuitData={circuitData}
                  signalMap={signalMap}
                  highlightedNodeId={highlightedNodeId}
                  criticalEdgeSet={criticalEdgeSet}
                  onInputToggle={handleInputToggle}
                />
              </div>
            </div>
            <ControlPanel
              circuitData={circuitData}
              onPlayBFS={handlePlayBFS}
              onPlayDFS={handlePlayDFS}
              isAnimating={isAnimating}
              onSelectCriticalPath={handleSelectCriticalPath}
              selectedCriticalPathIdx={selectedCriticalPathIdx}
            />
          </>
        )}
      </div>
    </div>
  );
}
