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
  const [inputMode, setInputMode]   = useState('preset'); // 'preset' | 'custom'
  const [form, setForm]             = useState({ numVars: '3', varNames: '', minterms: '', dontCares: '' });
  const [solving, setSolving]       = useState(false);
  const [solveError, setSolveError] = useState('');
  const [apiKey, setApiKey]               = useState('');
  const [aiExplanation, setAiExplanation] = useState('');
  const [explaining, setExplaining]       = useState(false);
  const [explainError, setExplainError]   = useState('');

  // ── 載入 QM JSON（僅 preset 模式）──────────────────────────
  useEffect(() => {
    if (inputMode !== 'preset') return;
    const file = QM_CIRCUITS[qmIdx].file;
    fetch(`/circuits/${file}`)
      .then(r => r.json())
      .then(data => {
        setQmData(data);
        setInputValues(initInputs(data.direct_circuit));
        setAiExplanation('');
        setExplainError('');
      });
  }, [qmIdx, inputMode]);

  // ── 訊號傳播（兩邊各自算）──────────────────────────────────
  useEffect(() => {
    if (!qmData?.direct_circuit) return;
    setSignalMapDirect(propagateSignals(qmData.direct_circuit, inputValues));
  }, [qmData, inputValues]);

  useEffect(() => {
    if (!qmData?.minimized_circuit) return;
    setSignalMapMinimized(propagateSignals(qmData.minimized_circuit, inputValues));
  }, [qmData, inputValues]);

  // ── 自訂題目送出 ────────────────────────────────────────────
  async function handleSolve(e) {
    e.preventDefault();
    setSolving(true);
    setSolveError('');
    try {
      const numVars = parseInt(form.numVars, 10);
      if (isNaN(numVars) || numVars < 1 || numVars > 20)
        throw new Error('變數數必須是 1–20 的整數');

      const minterms = form.minterms.split(',').map(s => s.trim()).filter(Boolean).map(Number);
      if (minterms.some(isNaN)) throw new Error('Minterms 格式錯誤，請輸入逗號分隔整數');

      const dontcares = form.dontCares.split(',').map(s => s.trim()).filter(Boolean).map(Number);
      if (dontcares.some(isNaN)) throw new Error("Don't care 格式錯誤，請輸入逗號分隔整數");

      const varNames = form.varNames.split(',').map(s => s.trim()).filter(Boolean);

      const res  = await fetch('/api/qm-solve', {
        method:  'POST',
        headers: { 'Content-Type': 'application/json' },
        body:    JSON.stringify({ numVars, minterms, dontcares, varNames }),
      });
      const data = await res.json();
      if (!res.ok) throw new Error(data.error || '計算失敗');
      setQmData(data);
      setInputValues(initInputs(data.direct_circuit));
      setAiExplanation('');
      setExplainError('');
    } catch (err) {
      setSolveError(err.message);
    } finally {
      setSolving(false);
    }
  }

  // ── 組 prompt（演算法答案全部帶入，AI 只負責講解）──────────
  //
  // 設計原則：
  //   ・所有數值均由 C++ QM 演算法計算，AI 只解釋，不推導
  //   ・固定四步骨架：① 全部 PI → ② essential PI → ③ Petrick's → ④ 化簡效益
  //   ・每步一句結論 + 最多一句「為什麼」，全文不超過 12 句
  //   ・術語第一次出現加最短括注，之後直接用
  //   ・輸出語言：繁體中文（台灣用語）
  function buildPrompt(data) {
    const result = data.qm_result;
    const steps  = data.qm_steps;
    const sop    = result.sop;

    // 全部 PI：單行格式，方便 AI 直接引用
    const allPIsLine = steps.prime_implicants
      .map(pi => `${pi.term}{${pi.minterms.join(',')}}`)
      .join('；');

    // Essential PI 清單
    const essPIs    = steps.prime_implicants.filter(pi => pi.essential);
    const petPIs    = steps.prime_implicants.filter(pi => pi.selected && !pi.essential);

    const essLine = essPIs.length
      ? essPIs.map(pi => `${pi.term}（蓋 {${pi.minterms.join(',')}}）`).join('、')
      : '無';

    const petLine = petPIs.length
      ? petPIs.map(pi => `${pi.term}（蓋 {${pi.minterms.join(',')}}）`).join('、')
      : '無（essential PI 已完整覆蓋）';

    // Essential PI 選完後的剩餘 minterms（交給 Petrick's 的那些）
    const covByEss  = new Set(essPIs.flatMap(pi => pi.minterms));
    const remaining = steps.input_minterms.filter(m => !covByEss.has(m));
    const remStr    = remaining.length > 0 ? `{${remaining.join(',')}}` : '（無）';

    // 化簡前 literal 數：每個 minterm 展開需要 num_vars 個 literal
    const beforeLits = steps.input_minterms.length * result.num_vars;
    // 化簡後 literal 數：從 qm_result.terms[].literals 精確加總
    const afterLits  = result.terms.reduce((s, t) => s + t.literals.length, 0);
    const savedLits  = Math.max(0, beforeLits - afterLits);
    const savedTerms = Math.max(0, steps.input_minterms.length - result.term_count);

    return `你是數位邏輯課的教學助理。

【硬性規則——違反即失格，請先讀完】
所有式子和數字均由 Quine-McCluskey 演算法（含 Petrick's method）計算，是唯一正確答案。
你只能「解釋」這份答案，不可自行推導、驗算，也不可提出任何替代答案。
最簡 SOP 是「${sop}」，不可更動。若你算出不同結果，代表你算錯了，請仍以「${sop}」為準。
輸出語言：繁體中文（台灣用語）。

【演算法資料（ground truth，禁止修改）】
變數：${result.var_names.join(', ')}
輸入 minterms：${steps.input_minterms.join(', ')}
全部 PI（共 ${steps.prime_implicants.length} 個）：${allPIsLine}
Essential PI：${essLine}
Petrick's 補選：${petLine}
最簡 SOP：${sop}（${result.term_count} 個乘積項，${afterLits} 個 literal）

【輸出格式——嚴格照做，不得擅自增加內容】
依序輸出四個步驟，每步一個標題。
每步：第一句是一句白話重點結論，第二句（選填）補一句「為什麼」。每步最多三句。
全文不超過 12 句、350 字。
術語第一次出現加最短括注，例如「essential PI（某 minterm 只有它蓋得到，非選不可）」，之後直接用。

① Prime Implicants（質主項）是什麼
  用一句話說清楚：共幾個 PI，它們各覆蓋哪些 minterms。
  可補一句：舉最典型的一兩個例子，說明是哪幾個 minterm 合併、消去了哪個變數。

② Essential PI 為什麼非選不可
  用一句話說：哪些是 essential PI，為什麼（覆蓋表中某 minterm 只被它蓋到，非選不可）。
  可補一句：這些 essential PI 合起來蓋了哪些 minterm。
  若 essLine 為「無」，說「本題無 essential PI，全部由 Petrick's 決定」。

③ Petrick's method 怎麼選到項數最少
  用一句話說：essential PI 選完後，剩餘 minterms ${remStr} 由 Petrick's 從候選 PI 中選出「${petLine}」，這樣保證項數最少。
  可補一句：Petrick's 展開所有可能的覆蓋組合，挑項數最少的那組，所以結果保證最簡。
  若 remStr 為「（無）」，說「essential PI 已完整覆蓋，Petrick's 不需補選」。

④ 化簡效益
  用一句話說：化簡前要 ${steps.input_minterms.length} 個乘積項（共 ${beforeLits} 個 literal），化簡後「${sop}」只需 ${result.term_count} 個乘積項（${afterLits} 個 literal），節省了 ${savedLits} 個 literal、減少 ${savedTerms} 個乘積項。

最後一行只寫：最簡 SOP = ${sop}`;
  }

  // ── 呼叫 Gemini API 取得白話詳解 ────────────────────────────
  async function handleExplain() {
    if (!apiKey.trim())     { setExplainError('請先輸入 Gemini API key'); return; }
    if (!qmData?.qm_steps) { setExplainError('請先解一題'); return; }
    if (qmData.qm_steps.prime_implicants.length === 0) {
      setExplainError('恆 0 或恆 1 函數無需詳解');
      return;
    }

    setExplaining(true);
    setExplainError('');
    setAiExplanation('');

    try {
      const res = await fetch(
        `/api/gemini/v1beta/models/gemini-2.5-flash-lite:generateContent?key=${encodeURIComponent(apiKey.trim())}`,
        {
          method:  'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({
            contents: [{ parts: [{ text: buildPrompt(qmData) }] }],
          }),
        }
      );
      const data = await res.json();
      if (!res.ok) {
        const msg = data.error?.message ?? '';
        const status = res.status;
        if (status === 429 || /quota|rate.?limit/i.test(msg)) {
          throw new Error('AI 詳解暫時無法使用，解題與電路圖功能不受影響（原因：API 額度不足或請求過於頻繁）');
        }
        if (/model.*not.*found|not.*support|unavailable/i.test(msg) || status === 404) {
          throw new Error('AI 詳解暫時無法使用，解題與電路圖功能不受影響（原因：模型目前不可用）');
        }
        throw new Error(`AI 詳解暫時無法使用，解題與電路圖功能不受影響（API 錯誤 ${status}）`);
      }
      setAiExplanation(data.candidates[0].content.parts[0].text);
    } catch (err) {
      const msg = err.message ?? '';
      const friendly = msg.startsWith('AI 詳解暫時無法使用')
        ? msg
        : `AI 詳解暫時無法使用，解題與電路圖功能不受影響（${msg}）`;
      setExplainError(friendly);
    } finally {
      setExplaining(false);
    }
  }

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
      {/* 資訊列：模式切換 + 輸入區 + 結果 */}
      <div style={styles.infoBar}>

        {/* 模式 Tab */}
        <div style={{ display: 'flex', gap: 4, flexShrink: 0 }}>
          {['preset', 'custom'].map(m => (
            <button
              key={m}
              onClick={() => setInputMode(m)}
              style={{
                padding: '3px 10px', borderRadius: 6, cursor: 'pointer', fontSize: 12, fontWeight: 600,
                border: '1px solid #bfdbfe',
                background: inputMode === m ? '#2563eb' : '#eff6ff',
                color:      inputMode === m ? '#fff'    : '#1e40af',
              }}
            >{m === 'preset' ? '內建範例' : '自訂題目'}</button>
          ))}
        </div>

        {/* 內建範例下拉 */}
        {inputMode === 'preset' && (
          <select
            className="circuit-select"
            value={qmIdx}
            onChange={e => setQmIdx(Number(e.target.value))}
          >
            {QM_CIRCUITS.map((c, i) => (
              <option key={i} value={i}>{c.label}</option>
            ))}
          </select>
        )}

        {/* 自訂輸入表單 */}
        {inputMode === 'custom' && (
          <form onSubmit={handleSolve} style={{ display: 'flex', gap: 6, alignItems: 'center', flexWrap: 'wrap' }}>
            <input
              type="number" min="1" max="20" required
              placeholder="變數數"
              value={form.numVars}
              onChange={e => setForm(f => ({ ...f, numVars: e.target.value }))}
              style={{ width: 60, padding: '3px 6px', borderRadius: 4, border: '1px solid #bfdbfe', fontSize: 12 }}
            />
            <input
              placeholder="變數名 A,B,C（選填）"
              value={form.varNames}
              onChange={e => setForm(f => ({ ...f, varNames: e.target.value }))}
              style={{ width: 148, padding: '3px 6px', borderRadius: 4, border: '1px solid #bfdbfe', fontSize: 12 }}
            />
            <input
              placeholder="Minterms  1,3,5,7" required
              value={form.minterms}
              onChange={e => setForm(f => ({ ...f, minterms: e.target.value }))}
              style={{ width: 152, padding: '3px 6px', borderRadius: 4, border: '1px solid #bfdbfe', fontSize: 12 }}
            />
            <input
              placeholder="Don't care（選填）"
              value={form.dontCares}
              onChange={e => setForm(f => ({ ...f, dontCares: e.target.value }))}
              style={{ width: 136, padding: '3px 6px', borderRadius: 4, border: '1px solid #bfdbfe', fontSize: 12 }}
            />
            <button
              type="submit" disabled={solving}
              style={{
                padding: '3px 14px', borderRadius: 4, border: 'none',
                cursor: solving ? 'default' : 'pointer',
                background: '#2563eb', color: '#fff', fontSize: 12, fontWeight: 600,
                opacity: solving ? 0.6 : 1,
              }}
            >{solving ? '計算中…' : '化簡'}</button>
            {solveError && (
              <span style={{ color: '#dc2626', fontSize: 12 }}>{solveError}</span>
            )}
          </form>
        )}

        {/* 化簡結果 */}
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

      {/* QM 中間步驟（純文字，確認資料流通，Phase 3 再美化）*/}
      {qmData?.qm_steps && (
        <div style={{
          padding: '6px 16px', borderTop: '1px solid #e5e7eb',
          background: '#f8fafc', fontSize: 12, fontFamily: 'monospace',
          flexShrink: 0, maxHeight: 160, overflowY: 'auto',
        }}>
          <strong>QM 中間步驟</strong>
          {' — '}
          <span style={{ color: '#6b7280' }}>
            {qmData.qm_steps.prime_implicants.length} 個 PI，
            input minterms: [{qmData.qm_steps.input_minterms.join(', ')}]
          </span>
          <div style={{ display: 'flex', flexWrap: 'wrap', gap: '2px 20px', marginTop: 3 }}>
            {qmData.qm_steps.prime_implicants.map(pi => {
              const tag  = pi.essential ? '★ Ess' : pi.selected ? '○ Petrick\'s' : '× 未選';
              const color = pi.essential ? '#1d4ed8' : pi.selected ? '#15803d' : '#9ca3af';
              return (
                <span key={pi.index} style={{ color }}>
                  [{pi.index}]{pi.term} m({pi.minterms.join(',')}) {tag}
                </span>
              );
            })}
          </div>
          <div style={{ marginTop: 3, color: '#374151' }}>
            <strong>覆蓋表：</strong>
            {qmData.qm_steps.coverage_table.map(row => (
              <span key={row.minterm} style={{ marginRight: 12 }}>
                m{row.minterm}→[{row.pi_indices.join(',')}]
              </span>
            ))}
          </div>
        </div>
      )}

      {/* AI 詳解區（需要 API key；無 key 則正常使用，只是看不到詳解）*/}
      {qmData?.qm_steps && (
        <div style={{
          padding: '6px 16px', borderTop: '1px solid #e5e7eb',
          background: '#fffbeb', flexShrink: 0,
        }}>
          <div style={{ display: 'flex', gap: 8, alignItems: 'center', flexWrap: 'wrap' }}>
            <span style={{ fontSize: 12, fontWeight: 700, color: '#92400e', flexShrink: 0 }}>
              AI 詳解
            </span>
            <input
              type="password"
              placeholder="Gemini API key（僅存於記憶體，不傳至任何伺服器）"
              value={apiKey}
              onChange={e => setApiKey(e.target.value)}
              style={{
                flex: 1, minWidth: 200, maxWidth: 380,
                padding: '3px 8px', borderRadius: 4,
                border: '1px solid #fcd34d', fontSize: 12,
              }}
            />
            <button
              onClick={handleExplain}
              disabled={explaining || !apiKey.trim()}
              style={{
                padding: '3px 14px', borderRadius: 4, border: 'none', flexShrink: 0,
                background: '#d97706', color: '#fff', fontSize: 12, fontWeight: 600,
                cursor: explaining || !apiKey.trim() ? 'default' : 'pointer',
                opacity: explaining || !apiKey.trim() ? 0.5 : 1,
              }}
            >{explaining ? '詳解中…' : '取得詳解'}</button>
            {explainError && (
              <span style={{ color: '#dc2626', fontSize: 12 }}>{explainError}</span>
            )}
          </div>
          {aiExplanation && (
            <div style={{
              marginTop: 6, padding: '8px 12px',
              background: '#fffef0', borderRadius: 4, border: '1px solid #fcd34d',
              fontSize: 13, lineHeight: 1.75, whiteSpace: 'pre-wrap',
              maxHeight: 320, overflowY: 'auto',
            }}>
              {aiExplanation}
            </div>
          )}
        </div>
      )}

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
