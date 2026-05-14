// ControlPanel.jsx ── 右側控制面板

export default function ControlPanel({
  circuitData,
  onPlayBFS,
  onPlayDFS,
  isAnimating,
  onSelectCriticalPath,
  selectedCriticalPathIdx,
}) {
  if (!circuitData) return <div style={styles.panel}><p style={styles.subtext}>載入中...</p></div>;

  const paths = circuitData.critical_paths ?? [];

  return (
    <div style={styles.panel}>
      {/* ── 拓撲排序動畫 ─────────────────────────────── */}
      <section style={styles.section}>
        <h3 style={styles.sectionTitle}>拓撲排序動畫</h3>
        <p style={styles.subtext}>依排序順序高亮節點 = 模擬器實際的計算順序</p>
        <div style={{ display: 'flex', gap: 8, marginTop: 10 }}>
          <button
            style={{ ...styles.btn, opacity: isAnimating ? 0.5 : 1 }}
            onClick={onPlayBFS}
            disabled={isAnimating}
          >
            ▶ Play BFS
          </button>
          <button
            style={{ ...styles.btn, opacity: isAnimating ? 0.5 : 1 }}
            onClick={onPlayDFS}
            disabled={isAnimating}
          >
            ▶ Play DFS
          </button>
        </div>
        {isAnimating && <p style={{ ...styles.subtext, marginTop: 6 }}>動畫播放中...</p>}
      </section>

      {/* ── Critical Paths ────────────────────────────── */}
      <section style={styles.section}>
        <h3 style={styles.sectionTitle}>Critical Paths（Top-{paths.length}）</h3>
        <p style={styles.subtext}>延遲最長的路徑決定電路最高時脈頻率</p>
        <div style={{ marginTop: 10 }}>
          {paths.length === 0 && <p style={styles.subtext}>無資料</p>}
          {paths.map((cp, idx) => (
            <div
              key={idx}
              style={{
                ...styles.cpCard,
                border: selectedCriticalPathIdx === idx
                  ? '2px solid #ef4444'
                  : '2px solid #e5e7eb',
              }}
              onClick={() => onSelectCriticalPath(idx)}
            >
              <div style={styles.cpHeader}>
                <span style={styles.cpRank}>#{cp.rank}</span>
                <span style={styles.cpDelay}>{cp.delay} ns</span>
                <span style={styles.cpFreq}>{cp.max_freq_mhz} MHz</span>
              </div>
              <div style={styles.cpPath}>
                {cp.path.join(' → ')}
              </div>
            </div>
          ))}
        </div>
      </section>
    </div>
  );
}

const styles = {
  panel: {
    width: 260,
    minWidth: 260,
    height: '100%',
    overflowY: 'auto',
    background: '#fff',
    borderLeft: '1px solid #e5e7eb',
    padding: '16px 14px',
    boxSizing: 'border-box',
    display: 'flex',
    flexDirection: 'column',
  },
  section: {
    marginBottom: 28,
  },
  sectionTitle: {
    margin: '0 0 4px 0',
    fontSize: 14,
    fontWeight: 600,
    color: '#374151',
  },
  subtext: {
    margin: 0,
    fontSize: 12,
    color: '#9ca3af',
    lineHeight: 1.5,
  },
  btn: {
    padding: '6px 12px',
    borderRadius: 6,
    border: '1px solid #d1d5db',
    background: '#f9fafb',
    cursor: 'pointer',
    fontSize: 13,
    fontWeight: 500,
  },
  cpCard: {
    padding: '8px 10px',
    borderRadius: 6,
    marginBottom: 8,
    cursor: 'pointer',
    background: '#fafafa',
  },
  cpHeader: {
    display: 'flex',
    gap: 8,
    alignItems: 'center',
    marginBottom: 4,
  },
  cpRank: {
    fontWeight: 700,
    fontSize: 13,
    color: '#1f2937',
  },
  cpDelay: {
    fontSize: 13,
    color: '#ef4444',
    fontWeight: 600,
  },
  cpFreq: {
    fontSize: 12,
    color: '#6b7280',
  },
  cpPath: {
    fontSize: 11,
    color: '#6b7280',
    wordBreak: 'break-all',
    lineHeight: 1.5,
  },
};
