// ControlPanel.jsx ── 右側控制面板
// 功能：BFS/DFS 動畫、Critical Path 顯示

export default function ControlPanel({
  circuitData,
  onPlayBFS,
  onPlayDFS,
  isAnimating,
  onSelectCriticalPath,
  selectedCriticalPathIdx,
}) {
  if (!circuitData) return <div style={styles.panel}><p>載入中...</p></div>;

  const paths = circuitData.critical_paths ?? [];

  return (
    <div style={styles.panel}>
      {/* ── 拓撲排序動畫 ─────────────────────────────── */}
      <section style={styles.section}>
        <h3 style={styles.sectionTitle}>拓撲排序動畫</h3>
        <div style={{ display: 'flex', gap: 8 }}>
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
        {isAnimating && <p style={styles.hint}>動畫播放中...</p>}
      </section>

      {/* ── Critical Paths ────────────────────────────── */}
      <section style={styles.section}>
        <h3 style={styles.sectionTitle}>Critical Paths（Top-{paths.length}）</h3>
        {paths.length === 0 && <p style={styles.hint}>無資料</p>}
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
      </section>

      {/* ── 說明 ─────────────────────────────────────── */}
      <section style={styles.section}>
        <p style={styles.hint}>點擊 INPUT 節點切換 0/1</p>
        <p style={styles.hint}>紅色邊 = 訊號為 1</p>
        <p style={styles.hint}>點選路徑後邊變粗紅線</p>
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
    gap: 0,
  },
  section: {
    marginBottom: 24,
  },
  sectionTitle: {
    margin: '0 0 10px 0',
    fontSize: 14,
    fontWeight: 600,
    color: '#374151',
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
  hint: {
    margin: '6px 0 0 0',
    fontSize: 12,
    color: '#6b7280',
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
