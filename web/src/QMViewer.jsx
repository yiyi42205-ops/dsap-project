import { useState, useEffect, useCallback } from 'react';
import QMCircuitPanel from './QMCircuitPanel.jsx';
import { propagateSignals } from './gateLogic.js';

const QM_CIRCUITS = [
  {
    label: '多數決 F(A,B,C)',
    file:  'majority_qm.json',
    desc:  '4 個 minterm，化簡前 8 閘 → 化簡後 4 閘',
  },
  {
    label: 'XOR F(A,B)',
    file:  'xor_qm.json',
    desc:  '已為最簡 SOP，展示無化簡空間的情況',
  },
  {
    label: '4 變數 + Don\'t Care F(A,B,C,D)',
    file:  'four_var_qm.json',
    desc:  '含 don\'t care，化簡前 9 閘 → 化簡後 3 閘',
  },
];

// ── 判斷 minimized_circuit 是否為恆 0/1（null 的原因）──────
// qm_result.sop 為 "0" 或 "1" 時是常數
function constType(qmResult) {
  if (!qmResult) return null;
  if (qmResult.sop === '0') return 'zero';
  if (qmResult.sop === '1') return 'one';
  return null;
}

// ── 從 circuitData 初始化 inputValues ───────────────────────
function initInputs(circuitData) {
  if (!circuitData) return {};
  const vals = {};
  for (const n of circuitData.nodes) {
    if (n.type === 'INPUT') vals[n.id] = false;
  }
  return vals;
}

const styles = {
  root: {
    display: 'flex',
    flexDirection: 'column',
    flex: 1,
    overflow: 'hidden',
    height: '100%',
  },
  infoBar: {
    display: 'flex',
    alignItems: 'center',
    gap: 12,
    padding: '8px 16px',
    background: '#eff6ff',
    borderBottom: '1px solid #bfdbfe',
    flexShrink: 0,
    flexWrap: 'wrap',
  },
  sopLabel: {
    fontSize: 13,
    color: '#1e3a8a',
    fontWeight: 700,
    fontFamily: 'monospace',
    whiteSpace: 'nowrap',
  },
  termBadge: {
    fontSize: 12,
    padding: '2px 8px',
    borderRadius: 12,
    background: '#bfdbfe',
    color: '#1e40af',
    fontWeight: 600,
    whiteSpace: 'nowrap',
  },
  hintText: {
    fontSize: 12,
    color: '#6b7280',
    marginLeft: 'auto',
    whiteSpace: 'nowrap',
  },
  body: {
    display: 'flex',
    flex: 1,
    gap: 8,
    padding: 8,
    overflow: 'hidden',
  },
  divider: {
    width: 1,
    background: '#e5e7eb',
    flexShrink: 0,
    alignSelf: 'stretch',
  },
};

export default function QMViewer() {
  const [qmIdx, setQmIdx]           = useState(0);
  const [qmData, setQmData]         = useState(null);
  const [inputValues, setInputValues] = useState({});
  const [signalMapDirect, setSignalMapDirect]       = useState(null);
  const [signalMapMinimized, setSignalMapMinimized] = useState(null);

  // ── 載入 QM JSON ────────────────────────────────────────────
  useEffect(() => {
    const file = QM_CIRCUITS[qmIdx].file;
    fetch(`/circuits/${file}`)
      .then(r => r.json())
      .then(data => {
        setQmData(data);
        // 兩邊電路變數相同，以 direct_circuit 初始化 inputValues
        setInputValues(initInputs(data.direct_circuit));
      });
  }, [qmIdx]);

  // ── 訊號傳播（兩邊各自算）──────────────────────────────────
  useEffect(() => {
    if (!qmData?.direct_circuit) return;
    setSignalMapDirect(propagateSignals(qmData.direct_circuit, inputValues));
  }, [qmData, inputValues]);

  useEffect(() => {
    if (!qmData?.minimized_circuit) return;
    setSignalMapMinimized(propagateSignals(qmData.minimized_circuit, inputValues));
  }, [qmData, inputValues]);

  // ── INPUT toggle（兩邊共用同一組 inputValues）──────────────
  const handleInputToggle = useCallback((nodeId) => {
    setInputValues(prev => ({ ...prev, [nodeId]: !prev[nodeId] }));
  }, []);

  const qmResult      = qmData?.qm_result ?? null;
  const directData    = qmData?.direct_circuit ?? null;
  const minimizedData = qmData?.minimized_circuit ?? null;
  const isConst       = constType(qmResult);

  const directGateCount = directData
    ? directData.nodes.filter(n => n.type !== 'INPUT' && n.type !== 'OUTPUT').length
    : null;

  return (
    <div style={styles.root}>
      {/* SOP 資訊列 + 範例選擇 */}
      <div style={styles.infoBar}>
        <select
          className="circuit-select"
          value={qmIdx}
          onChange={e => setQmIdx(Number(e.target.value))}
        >
          {QM_CIRCUITS.map((c, i) => (
            <option key={i} value={i}>{c.label}</option>
          ))}
        </select>
        {qmResult && (
          <>
            <span style={styles.sopLabel}>最小 SOP：{qmResult.sop}</span>
            <span style={styles.termBadge}>{qmResult.term_count} 個乘積項</span>
          </>
        )}
        <span style={styles.hintText}>
          💡 點擊 INPUT 節點切換 0/1，左右電路同步更新
        </span>
      </div>

      {/* 並排兩個電路 */}
      <div style={styles.body}>
        <QMCircuitPanel
          title="直接展開版"
          circuitData={directData}
          directGateCount={null}
          signalMap={signalMapDirect}
          onInputToggle={handleInputToggle}
          isConst={null}
        />

        <div style={styles.divider} />

        <QMCircuitPanel
          title="QM 化簡版"
          circuitData={minimizedData}
          directGateCount={directGateCount}
          signalMap={signalMapMinimized}
          onInputToggle={handleInputToggle}
          isConst={minimizedData ? null : isConst}
        />
      </div>
    </div>
  );
}
