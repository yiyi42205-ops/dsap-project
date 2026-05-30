import { useState, useEffect, useCallback, useRef } from 'react';
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
    overflowY: 'auto',
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
    minHeight: 320,
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

  const [apiKey, setApiKey]             = useState('');
  const [systemPrompt, setSystemPrompt] = useState('');
  const [chatMessages, setChatMessages] = useState([]); // { role: 'user'|'assistant', text: string }[]
  const [chatInput, setChatInput]       = useState('');
  const [chatLoading, setChatLoading]   = useState(false);
  const [chatError, setChatError]       = useState('');
  const [highlightedNodeId, setHighlightedNodeId] = useState(null);

  const chatBottomRef = useRef(null);

  // ── 載入 QM JSON（僅 preset 模式）──────────────────────────
  useEffect(() => {
    if (inputMode !== 'preset') return;
    const file = QM_CIRCUITS[qmIdx].file;
    fetch(`/circuits/${file}`)
      .then(r => r.json())
      .then(data => {
        setQmData(data);
        setInputValues(initInputs(data.direct_circuit));
        resetChat();
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

  // ── 捲到最新訊息 ─────────────────────────────────────────
  useEffect(() => {
    chatBottomRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [chatMessages, chatLoading]);

  function resetChat() {
    setChatMessages([]);
    setChatInput('');
    setChatError('');
    setSystemPrompt('');
    setHighlightedNodeId(null);
  }

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
      resetChat();
    } catch (err) {
      setSolveError(err.message);
    } finally {
      setSolving(false);
    }
  }

  // ── 組 prompt ───────────────────────────────────────────────
  function buildPrompt(data) {
    const result   = data.qm_result;
    const steps    = data.qm_steps;
    const sop      = result.sop;
    const numVars  = result.num_vars;
    const varNames = result.var_names;

    const toBin = n => n.toString(2).padStart(numVars, '0');

    // ── 重建合併過程 ─────────────────────────────────────────
    function buildMergeSteps(pi) {
      const minterms = pi.minterms;

      if (minterms.length === 1)
        return `  ${pi.term}：m(${minterms[0]}) = ${toBin(minterms[0])}（單一 minterm，無法再合併）`;

      // 若 minterms 數不是 2 的冪，表示有 don't care 參與合併
      const isPow2 = n => n > 0 && (n & (n - 1)) === 0;
      if (!isPow2(minterms.length)) {
        const stableBits = Array.from({ length: numVars }, (_, i) => {
          const vals = new Set(minterms.map(m => (m >> (numVars - 1 - i)) & 1));
          return vals.size === 1 ? String([...vals][0]) : '-';
        }).join('');
        return `  ${pi.term}：m(${minterms.join(', ')})＋don't care 參與合併，最終 pattern = ${stableBits}`;
      }

      const lines = [`  ${pi.term}：m(${minterms.join(', ')})`];
      let current = minterms.map(toBin);

      for (let round = 1; current.length > 1; round++) {
        const next = [];
        const used = new Set();
        for (let i = 0; i < current.length; i++) {
          if (used.has(i)) continue;
          for (let j = i + 1; j < current.length; j++) {
            if (used.has(j)) continue;
            const a = current[i], b = current[j];
            let diffPos = -1, ok = true;
            for (let k = 0; k < numVars; k++) {
              if ((a[k] === '-') !== (b[k] === '-')) { ok = false; break; }
              if (a[k] !== b[k]) { if (diffPos !== -1) { ok = false; break; } diffPos = k; }
            }
            if (ok && diffPos !== -1) {
              const merged = a.split(''); merged[diffPos] = '-';
              lines.push(`    第${round}輪：${a} ∧ ${b} → ${merged.join('')}（消去 ${varNames[diffPos]}）`);
              next.push(merged.join(''));
              used.add(i); used.add(j);
              break;
            }
          }
        }
        if (!next.length) break;
        current = next;
      }
      return lines.join('\n');
    }

    // ── PI 清單 ──────────────────────────────────────────────
    const mergeSteps = steps.prime_implicants.map(buildMergeSteps).join('\n');

    const piList = steps.prime_implicants.map(pi => {
      const tag = pi.essential ? '★ Essential' : pi.selected ? '○ Petrick 補選' : '× 未選';
      return `  [${pi.index}] ${pi.term}  蓋 {${pi.minterms.join(',')}}  ${tag}`;
    }).join('\n');

    // ── Essential PI：標出讓它「非選不可」的那個 minterm ────
    const essPIs = steps.prime_implicants.filter(pi => pi.essential);
    const petPIs = steps.prime_implicants.filter(pi => pi.selected && !pi.essential);

    const essentialList = essPIs.length
      ? essPIs.map(pi => {
          const uniqueMs = pi.minterms.filter(m => {
            const row = steps.coverage_table.find(r => r.minterm === m);
            return row && row.pi_indices.length === 1;
          });
          const why = uniqueMs.length ? `（m(${uniqueMs.join(',')}) 只有它蓋得到）` : '';
          return `${pi.term}${why}`;
        }).join('、')
      : '無';

    const petrickResult = petPIs.length
      ? petPIs.map(pi => `${pi.term}（蓋 {${pi.minterms.join(',')}}）`).join('、')
      : '無';

    // Petrick's 展開資料：各剩餘 minterm 的候選 PI
    const covByEss  = new Set(essPIs.flatMap(pi => pi.minterms));
    const remaining = steps.input_minterms.filter(m => !covByEss.has(m));
    const petrickContext = remaining.length
      ? remaining.map(m => {
          const row  = steps.coverage_table.find(r => r.minterm === m);
          const cands = row
            ? row.pi_indices.map(idx => {
                const p = steps.prime_implicants.find(p => p.index === idx);
                return p ? p.term : `P${idx}`;
              }).join(' + ')
            : '?';
          return `m(${m}) → (${cands})`;
        }).join('；')
      : '（essential 已蓋滿，無剩餘 minterm）';

    // ── 字數統計 ─────────────────────────────────────────────
    const afterLits  = result.terms.reduce((s, t) => s + t.literals.length, 0);
    const beforeLits = steps.input_minterms.length * numVars;

    // Don't care：出現在 PI minterms 但不在 input_minterms 的（保守偵測）
    const inputSet = new Set(steps.input_minterms);
    const dcSet    = new Set(steps.prime_implicants.flatMap(pi => pi.minterms).filter(m => !inputSet.has(m)));
    const dontCareStr = dcSet.size ? [...dcSet].sort((a,b)=>a-b).join(', ') : '無';

    // ── 電路節點對照 ─────────────────────────────────────────
    const minNodes = data.minimized_circuit?.nodes ?? [];
    const andNodes = minNodes.filter(n => n.type !== 'INPUT' && n.type !== 'OUTPUT' && n.type !== 'OR');
    const orNodes  = minNodes.filter(n => n.type === 'OR');
    const nodeMappingLines = [
      ...andNodes.map((n, i) => {
        const term = result.terms[i];
        return `  ${n.id}（AND）→ 乘積項 ${term ? term.literals.join('') : '?'}`;
      }),
      ...orNodes.map(n => `  ${n.id}（OR）→ 最終輸出`),
    ];
    const nodeMapping = nodeMappingLines.length ? nodeMappingLines.join('\n') : '  （無）';

    // 所有等效最簡解
    const allSOPs = result.all_minimal_sops ?? [sop];
    const allSOPsNote = allSOPs.length > 1
      ? `等效最簡解（共 ${allSOPs.length} 組，學生寫出任一組均正確）：\n${allSOPs.map((s, i) => `  解 ${i + 1}：${s}`).join('\n')}`
      : `最簡解唯一：${sop}`;

    return `你是數位邏輯課的教學助理。

【硬性規則】
以下「演算法資料」由 Quine-McCluskey 演算法計算，是唯一正確答案。
- 答案層（PI 清單、essential 判定、最簡 SOP、項數、literal 數、合併過程）：直接引用，不可更動。若心算結果不同，以資料為準。
- 過程層（為什麼是 PI、為什麼 essential、Petrick 怎麼展開）：必須具體解釋步驟，不可用「演算法規則」搪塞。
- 若學生答案出現在等效最簡解清單，承認正確並解釋為何並列成立。
輸出語言：繁體中文（台灣用語）。

${allSOPsNote}

【演算法資料】
變數：${varNames.join(', ')}
輸入 minterms：${steps.input_minterms.join(', ')}
Don't care：${dontCareStr}
合併過程：
${mergeSteps}
全部 PI（${steps.prime_implicants.length} 個）：
${piList}
Essential PI：${essentialList}
Petrick's 補選：${petrickResult}
Petrick's 展開資料：${petrickContext}
最簡 SOP：${sop}
項數：${result.term_count}，literal 數：${afterLits}
化簡前項數：${steps.input_minterms.length}，literal 數：${beforeLits}

【電路節點】
${nodeMapping}
提到具體閘時用 [node:節點ID] 標記。

【首次回答格式】
四步，每步一句白話結論＋（選填）一句「為什麼」，最多三句，全文 ≤ 350 字。
最後一行：最簡 SOP = ${sop}

① Prime Implicants 是什麼——挑 1~2 個 PI 用合併過程說明它從哪些 minterm 來
② Essential PI 為什麼非選不可——點出哪個 minterm 只有它能蓋
③ Petrick's method 怎麼選到項數最少
   - 若「${petrickResult}」為「無」：說明 essential 已蓋滿、不需要 Petrick，並用一句話示意 Petrick 的作用
   - 若有補選：用 Petrick's 展開資料列出各 minterm 的候選 PI、AND 起來、挑乘積項最少的那組，結果是「${petrickResult}」
④ 化簡效益：一句話說節省了幾項、幾個 literal

【追問】
追問時放掉四步格式，直接針對問題回答。
- 答案層數字以 ground truth 為準；過程層可舉例展開
- 學生說「沒看懂」：先問哪步卡住，或直接用本題數字展開那步，不要重講四步全部
- 不要主動貼最簡 SOP，只在學生要結論時貼
- 只回答與「本題化簡過程、數位邏輯概念、課本章節」有關的問題：
  - 與題目無關（生活、其他科目）→ 一句話說明你是化簡助理，請對方換題相關問題
  - 數位邏輯範圍但資料不足（K-map、POS、其他電路）→ 可解釋概念，但說明「本工具只到 SOP 化簡，其他形式請手算或另工具驗證」，不編造數字
  - 質疑答案（「老師說是 X」）→ 不動搖；項數相同引導對照等效解清單，項數不同建議重查 minterms

【Petrick's method 展開模板——觸發時照這個講】
1. 對每個未被 essential 覆蓋的 minterm，列出能蓋它的 PI：m_i → (P_a + P_b + ...)
2. 全部 AND 起來：(P_a + P_b)(P_c + P_d)...
3. 用分配律乘開、吸收律化簡
4. 挑乘積項數最少（平手時挑 literal 最少）的那一項
5. 該項裡的 PI 就是補選的 PI`;
  }

  // ── 呼叫 Gemini API（共用） ──────────────────────────────────
  async function callGemini(contents) {
    const res = await fetch(
      `/api/gemini/v1beta/models/gemini-2.5-flash-lite:generateContent?key=${encodeURIComponent(apiKey.trim())}`,
      {
        method:  'POST',
        headers: { 'Content-Type': 'application/json' },
        body:    JSON.stringify({ contents }),
      }
    );
    const data = await res.json();
    if (!res.ok) {
      const msg    = data.error?.message ?? '';
      const status = res.status;
      if (status === 429 || /quota|rate.?limit/i.test(msg))
        throw new Error('API 額度不足或請求過於頻繁');
      if (/model.*not.*found|not.*support|unavailable/i.test(msg) || status === 404)
        throw new Error('模型目前不可用');
      throw new Error(`API 錯誤 ${status}`);
    }
    return data.candidates[0].content.parts[0].text;
  }

  // ── 第一次取得詳解 ───────────────────────────────────────────
  async function handleExplain() {
    if (!apiKey.trim())     { setChatError('請先輸入 Gemini API key'); return; }
    if (!qmData?.qm_steps) { setChatError('請先解一題'); return; }
    if (qmData.qm_steps.prime_implicants.length === 0) {
      setChatError('恆 0 或恆 1 函數無需詳解');
      return;
    }

    setChatLoading(true);
    setChatError('');
    setChatMessages([]);

    const prompt = buildPrompt(qmData);
    setSystemPrompt(prompt);

    try {
      const text = await callGemini([{ role: 'user', parts: [{ text: prompt }] }]);
      setChatMessages([{ role: 'assistant', text }]);
    } catch (err) {
      setChatError(`AI 詳解暫時無法使用（${err.message}）`);
    } finally {
      setChatLoading(false);
    }
  }

  // ── 追問 ────────────────────────────────────────────────────
  async function handleFollowUp(e) {
    e.preventDefault();
    const q = chatInput.trim();
    if (!q || chatLoading) return;

    const newMessages = [...chatMessages, { role: 'user', text: q }];
    setChatMessages(newMessages);
    setChatInput('');
    setChatLoading(true);
    setChatError('');

    // 重建完整對話歷史送給 Gemini
    const contents = [
      { role: 'user',  parts: [{ text: systemPrompt }] },
      ...chatMessages.map(m => ({
        role:  m.role === 'assistant' ? 'model' : 'user',
        parts: [{ text: m.text }],
      })),
      { role: 'user', parts: [{ text: q }] },
    ];

    try {
      const text = await callGemini(contents);
      setChatMessages([...newMessages, { role: 'assistant', text }]);
    } catch (err) {
      setChatError(`無法取得回應（${err.message}）`);
      // 移除剛加的 user bubble，避免假象
      setChatMessages(chatMessages);
    } finally {
      setChatLoading(false);
    }
  }

  // ── 把 [node:ID] 解析成可點擊的 chip ─────────────────────────
  function renderMessageText(text) {
    const parts = [];
    const re    = /\[node:([^\]]+)\]/g;
    let last = 0, m;
    while ((m = re.exec(text)) !== null) {
      if (m.index > last) parts.push(<span key={`t${m.index}`}>{text.slice(last, m.index)}</span>);
      const nodeId   = m[1];
      const isActive = highlightedNodeId === nodeId;
      parts.push(
        <span
          key={`n${m.index}`}
          onClick={() => setHighlightedNodeId(prev => prev === nodeId ? null : nodeId)}
          title={`點擊在電路圖上高亮 ${nodeId}`}
          style={{
            display: 'inline-block',
            padding: '1px 6px',
            borderRadius: 4,
            background: isActive ? '#3b82f6' : '#fef3c7',
            color:      isActive ? '#fff'    : '#92400e',
            border:     `1px solid ${isActive ? '#1d4ed8' : '#fcd34d'}`,
            cursor:     'pointer',
            fontFamily: 'monospace',
            fontSize:   11,
            fontWeight: 700,
            verticalAlign: 'middle',
            margin: '0 2px',
          }}
        >
          {nodeId}
        </span>
      );
      last = re.lastIndex;
    }
    if (last < text.length) parts.push(<span key="tail">{text.slice(last)}</span>);
    return parts;
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

  const hasChat = chatMessages.length > 0;

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

      {/* QM 中間步驟 */}
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
          <div style={{ display: 'flex', flexDirection: 'column', gap: 2, marginTop: 3 }}>
            {qmData.qm_steps.prime_implicants.map(pi => {
              const tag  = pi.essential ? '★ Ess' : pi.selected ? '○ Petrick\'s' : '× 未選';
              const color = pi.essential ? '#1d4ed8' : pi.selected ? '#15803d' : '#9ca3af';
              const mergedFrom = pi.minterms.length > 1
                ? ` ← m(${pi.minterms.join(', ')}) 合併`
                : ` ← m(${pi.minterms[0]})`;
              return (
                <span key={pi.index} style={{ color }}>
                  [{pi.index}] {pi.term.padEnd(12)}{mergedFrom}　{tag}
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

      {/* AI 詳解區（多輪對話）*/}
      {qmData?.qm_steps && (
        <div style={{
          padding: '6px 16px 8px', borderTop: '1px solid #e5e7eb',
          background: '#fffbeb', flexShrink: 0,
        }}>
          {/* 標題列：API key + 取得詳解按鈕 */}
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
              disabled={chatLoading || !apiKey.trim()}
              style={{
                padding: '3px 14px', borderRadius: 4, border: 'none', flexShrink: 0,
                background: '#d97706', color: '#fff', fontSize: 12, fontWeight: 600,
                cursor: chatLoading || !apiKey.trim() ? 'default' : 'pointer',
                opacity: chatLoading || !apiKey.trim() ? 0.5 : 1,
              }}
            >{chatLoading && !hasChat ? '詳解中…' : hasChat ? '重新詳解' : '取得詳解'}</button>
            {chatError && (
              <span style={{ color: '#dc2626', fontSize: 12 }}>{chatError}</span>
            )}
          </div>

          {/* 對話氣泡 */}
          {hasChat && (
            <div style={{
              marginTop: 6,
              maxHeight: 260, overflowY: 'auto',
              display: 'flex', flexDirection: 'column', gap: 6,
              border: '1px solid #fcd34d', borderRadius: 4,
              padding: '8px 10px', background: '#fffef0',
            }}>
              {chatMessages.map((msg, i) => (
                <div
                  key={i}
                  style={{
                    alignSelf: msg.role === 'user' ? 'flex-end' : 'flex-start',
                    maxWidth: '88%',
                  }}
                >
                  <div style={{
                    padding: '6px 10px',
                    borderRadius: msg.role === 'user'
                      ? '12px 12px 2px 12px'
                      : '12px 12px 12px 2px',
                    background: msg.role === 'user' ? '#dbeafe' : '#fff',
                    border:     msg.role === 'user' ? '1px solid #93c5fd' : '1px solid #e5e7eb',
                    fontSize: 13, lineHeight: 1.65,
                    whiteSpace: 'pre-wrap', color: '#1f2937',
                  }}>
                    {msg.role === 'assistant'
                      ? renderMessageText(msg.text)
                      : msg.text}
                  </div>
                </div>
              ))}
              {chatLoading && (
                <div style={{ alignSelf: 'flex-start', color: '#9ca3af', fontSize: 12, padding: '4px 8px' }}>
                  思考中…
                </div>
              )}
              <div ref={chatBottomRef} />
            </div>
          )}

          {/* 追問輸入欄（有第一次詳解後才出現）*/}
          {hasChat && (
            <form onSubmit={handleFollowUp} style={{ display: 'flex', gap: 6, marginTop: 6 }}>
              <input
                placeholder="繼續追問，例如：Essential PI 是怎麼判斷的？"
                value={chatInput}
                onChange={e => setChatInput(e.target.value)}
                disabled={chatLoading}
                style={{
                  flex: 1, padding: '4px 10px', borderRadius: 4,
                  border: '1px solid #fcd34d', fontSize: 12,
                  background: chatLoading ? '#f9fafb' : '#fff',
                }}
              />
              <button
                type="submit"
                disabled={chatLoading || !chatInput.trim()}
                style={{
                  padding: '4px 14px', borderRadius: 4, border: 'none',
                  background: '#d97706', color: '#fff', fontSize: 12, fontWeight: 600,
                  cursor: chatLoading || !chatInput.trim() ? 'default' : 'pointer',
                  opacity: chatLoading || !chatInput.trim() ? 0.5 : 1,
                  flexShrink: 0,
                }}
              >送出</button>
            </form>
          )}

          {/* 高亮提示 */}
          {highlightedNodeId && (
            <div style={{ marginTop: 4, fontSize: 11, color: '#92400e' }}>
              電路高亮：
              <strong style={{ fontFamily: 'monospace' }}>{highlightedNodeId}</strong>

              <span
                onClick={() => setHighlightedNodeId(null)}
                style={{ cursor: 'pointer', textDecoration: 'underline', color: '#b45309' }}
              >
                取消
              </span>
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
          highlightedNodeId={highlightedNodeId}
        />

        <div style={styles.divider} />

        <QMCircuitPanel
          title="QM 化簡版"
          circuitData={minimizedData}
          directGateCount={directGateCount}
          signalMap={signalMapMinimized}
          onInputToggle={handleInputToggle}
          isConst={minimizedData ? null : isConst}
          highlightedNodeId={highlightedNodeId}
        />
      </div>
    </div>
  );
}
