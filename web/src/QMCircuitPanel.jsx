import CircuitViewer from './CircuitViewer.jsx';

// ── 閘數 badge 樣式 ──────────────────────────────────────────
const styles = {
  panel: {
    display: 'flex',
    flexDirection: 'column',
    flex: 1,
    minWidth: 0,
    height: '100%',
    border: '1px solid #e5e7eb',
    borderRadius: 8,
    overflow: 'hidden',
    background: '#fff',
  },
  panelHeader: {
    display: 'flex',
    alignItems: 'center',
    gap: 8,
    padding: '8px 14px',
    borderBottom: '1px solid #e5e7eb',
    background: '#f9fafb',
    flexShrink: 0,
  },
  title: {
    fontSize: 13,
    fontWeight: 600,
    color: '#374151',
    whiteSpace: 'nowrap',
  },
  gateBadge: {
    fontSize: 12,
    padding: '2px 8px',
    borderRadius: 12,
    background: '#dbeafe',
    color: '#1d4ed8',
    fontWeight: 600,
    whiteSpace: 'nowrap',
  },
  reductionBadge: {
    fontSize: 12,
    padding: '2px 8px',
    borderRadius: 12,
    background: '#dcfce7',
    color: '#15803d',
    fontWeight: 600,
    whiteSpace: 'nowrap',
  },
  sameBadge: {
    fontSize: 12,
    padding: '2px 8px',
    borderRadius: 12,
    background: '#fef9c3',
    color: '#a16207',
    fontWeight: 600,
    whiteSpace: 'nowrap',
  },
  viewerWrap: {
    flex: 1,
    position: 'relative',
    overflow: 'hidden',
  },
  constPlaceholder: {
    display: 'flex',
    alignItems: 'center',
    justifyContent: 'center',
    height: '100%',
    fontSize: 14,
    color: '#9ca3af',
    fontStyle: 'italic',
    padding: 24,
    textAlign: 'center',
  },
};

// ── 計算閘數（排除 INPUT / OUTPUT 節點）─────────────────────
function countGates(circuitData) {
  if (!circuitData) return null;
  return circuitData.nodes.filter(
    n => n.type !== 'INPUT' && n.type !== 'OUTPUT'
  ).length;
}

// ─────────────────────────────────────────────────────────────
// Props:
//   title           string             "直接展開版" / "QM 化簡版"
//   circuitData     object | null      direct_circuit 或 minimized_circuit
//   directGateCount number | null      僅化簡版使用，用來算節省閘數
//   signalMap       Map | null
//   onInputToggle   (nodeId) => void
//   isConst         "zero"|"one"|null  null = 正常電路
// ─────────────────────────────────────────────────────────────
export default function QMCircuitPanel({
  title,
  circuitData,
  directGateCount,
  signalMap,
  onInputToggle,
  isConst,
  highlightedNodeId,
}) {
  const gateCount = countGates(circuitData);
  const saved = (directGateCount != null && gateCount != null)
    ? directGateCount - gateCount
    : null;

  return (
    <div style={styles.panel}>
      {/* 面板標題列 */}
      <div style={styles.panelHeader}>
        <span style={styles.title}>{title}</span>
        {gateCount != null && (
          <span style={styles.gateBadge}>{gateCount} 個閘</span>
        )}
        {saved != null && saved > 0 && (
          <span style={styles.reductionBadge}>省了 {saved} 個閘</span>
        )}
        {saved != null && saved === 0 && (
          <span style={styles.sameBadge}>已最簡，無法化簡</span>
        )}
      </div>

      {/* 電路圖 或 null 佔位 */}
      <div style={styles.viewerWrap}>
        {isConst ? (
          <div style={styles.constPlaceholder}>
            此函數恆為 {isConst === 'zero' ? '0' : '1'}，無需電路
          </div>
        ) : (
          <CircuitViewer
            circuitData={circuitData}
            signalMap={signalMap}
            highlightedNodeId={highlightedNodeId ?? null}
            criticalEdgeSet={null}
            onInputToggle={onInputToggle}
          />
        )}
      </div>
    </div>
  );
}
