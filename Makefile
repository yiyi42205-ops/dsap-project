CXX      = g++
CXXFLAGS = -std=c++17 -O2 -Wall

BENCH    = src/benchmarks
RESULTS  = src/benchmarks/results
TESTS    = tests

# ── 預設目標 ─────────────────────────────────────────────────────
.PHONY: all main scale_bench structural_bench bench_plot test clean \
        test_dfs_iter test_qm test_sop

all: main

# ── 主程式 ───────────────────────────────────────────────────────
main: src/main.cpp src/json_export.cpp src/circuit.h src/critical_path.h \
      src/json_export.h src/qm_minimizer.h src/sop_to_circuit.h
	$(CXX) $(CXXFLAGS) src/main.cpp src/json_export.cpp -o simulator

# ── 大規模 BFS vs DFS 效能測試（隨機 DAG）──────────────────────
scale_bench:
	$(CXX) $(CXXFLAGS) $(BENCH)/scale_test.cpp -o $(BENCH)/scale_test
	mkdir -p $(RESULTS)
	cd $(BENCH) && ./scale_test

# ── 結構性 DAG 對照實驗 ───────────────────────────────────────
structural_bench:
	$(CXX) $(CXXFLAGS) $(BENCH)/structural_test.cpp -o $(BENCH)/structural_test
	mkdir -p $(RESULTS)
	cd $(BENCH) && ./structural_test

# ── 效能圖表（scale + structural 合一）──────────────────────────
bench_plot:
	cd $(BENCH) && python3 plot_bench.py

# ── Unit tests ───────────────────────────────────────────────────
$(TESTS)/test_truth_table: $(TESTS)/test_truth_table.cpp src/circuit.h
	$(CXX) $(CXXFLAGS) $< -o $@

$(TESTS)/test_edge_cases: $(TESTS)/test_edge_cases.cpp src/circuit.h
	$(CXX) $(CXXFLAGS) $< -o $@

$(TESTS)/test_topo_consistency: $(TESTS)/test_topo_consistency.cpp src/circuit.h
	$(CXX) $(CXXFLAGS) $< -o $@

$(TESTS)/test_dfs_iterative: $(TESTS)/test_dfs_iterative.cpp src/circuit.h
	$(CXX) $(CXXFLAGS) $< -o $@

$(TESTS)/test_qm_minimizer: $(TESTS)/test_qm_minimizer.cpp src/qm_minimizer.h src/circuit.h
	$(CXX) $(CXXFLAGS) $< -o $@

$(TESTS)/test_sop_to_circuit: $(TESTS)/test_sop_to_circuit.cpp \
                               src/sop_to_circuit.h src/qm_minimizer.h \
                               src/circuit.h
	$(CXX) $(CXXFLAGS) $< -o $@

test: $(TESTS)/test_truth_table \
      $(TESTS)/test_edge_cases \
      $(TESTS)/test_topo_consistency \
      $(TESTS)/test_dfs_iterative \
      $(TESTS)/test_qm_minimizer \
      $(TESTS)/test_sop_to_circuit
	@echo ""
	@echo "╔══════════════════════════════════╗"
	@echo "║        Running Unit Tests        ║"
	@echo "╚══════════════════════════════════╝"
	@$(TESTS)/test_truth_table      || (echo ""; echo "test_truth_table FAILED"; exit 1)
	@echo ""
	@$(TESTS)/test_edge_cases       || (echo ""; echo "test_edge_cases FAILED"; exit 1)
	@echo ""
	@$(TESTS)/test_topo_consistency || (echo ""; echo "test_topo_consistency FAILED"; exit 1)
	@echo ""
	@$(TESTS)/test_dfs_iterative    || (echo ""; echo "test_dfs_iterative FAILED"; exit 1)
	@echo ""
	@$(TESTS)/test_qm_minimizer     || (echo ""; echo "test_qm_minimizer FAILED"; exit 1)
	@echo ""
	@$(TESTS)/test_sop_to_circuit   || (echo ""; echo "test_sop_to_circuit FAILED"; exit 1)
	@echo ""
	@echo "All tests passed."

# ── Quine-McCluskey 最小化器（單獨跑）───────────────────────
test_qm: $(TESTS)/test_qm_minimizer
	@$(TESTS)/test_qm_minimizer

# ── SOP→Circuit 轉換器（單獨跑）─────────────────────────────
test_sop: $(TESTS)/test_sop_to_circuit
	@$(TESTS)/test_sop_to_circuit

# ── 清理 ─────────────────────────────────────────────────────────
clean:
	rm -f simulator \
	      $(BENCH)/scale_test \
	      $(BENCH)/structural_test \
	      $(TESTS)/test_truth_table \
	      $(TESTS)/test_edge_cases \
	      $(TESTS)/test_topo_consistency \
	      $(TESTS)/test_dfs_iterative \
	      $(TESTS)/test_qm_minimizer \
	      $(TESTS)/test_sop_to_circuit \
	      $(TESTS)/tmp_tt_*.tt
	rm -rf $(RESULTS)
