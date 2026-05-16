CXX      = g++
CXXFLAGS = -std=c++17 -O2 -Wall

BENCH    = src/benchmarks
RESULTS  = src/benchmarks/results
TESTS    = tests

# ── 預設目標 ─────────────────────────────────────────────────────
.PHONY: all main benchmarks bench_hash bench_topo bench_ordered bench_random \
        plot scale_bench scale_plot structural_bench structural_plot test clean \
        test_dfs_iter test_qm test_equiv

all: main

# ── 主程式 ───────────────────────────────────────────────────────
main: src/main.cpp src/json_export.cpp src/circuit.h src/critical_path.h src/json_export.h
	$(CXX) $(CXXFLAGS) src/main.cpp src/json_export.cpp -o simulator

# ── 任務 B：Ablation Study benchmarks ───────────────────────────
benchmarks: bench_hash bench_topo bench_ordered

bench_hash:
	$(CXX) $(CXXFLAGS) $(BENCH)/hash_vs_linear.cpp       -o $(BENCH)/hash_vs_linear
	mkdir -p $(RESULTS)
	cd $(BENCH) && ./hash_vs_linear

bench_topo:
	$(CXX) $(CXXFLAGS) $(BENCH)/topo_vs_naive.cpp        -o $(BENCH)/topo_vs_naive
	mkdir -p $(RESULTS)
	cd $(BENCH) && ./topo_vs_naive

bench_ordered:
	$(CXX) $(CXXFLAGS) $(BENCH)/ordered_vs_unordered.cpp -o $(BENCH)/ordered_vs_unordered
	mkdir -p $(RESULTS)
	cd $(BENCH) && ./ordered_vs_unordered

# ── 任務 D：嚴謹 BFS vs DFS 效能測試（隨機電路）────────────────
bench_random:
	$(CXX) $(CXXFLAGS) $(BENCH)/random_circuit_gen.cpp   -o $(BENCH)/random_circuit_gen
	mkdir -p $(RESULTS)
	cd $(BENCH) && ./random_circuit_gen

plot:
	cd $(BENCH) && python3 plot.py

# ── 大規模 BFS vs DFS 效能測試 ───────────────────────────────
scale_bench:
	$(CXX) $(CXXFLAGS) $(BENCH)/scale_test.cpp -o $(BENCH)/scale_test
	mkdir -p $(RESULTS)
	cd $(BENCH) && ./scale_test

scale_plot:
	cd $(BENCH) && python3 plot_scale.py

# ── 結構性 DAG 對照實驗 ───────────────────────────────────────
structural_bench:
	$(CXX) $(CXXFLAGS) $(BENCH)/structural_test.cpp -o $(BENCH)/structural_test
	mkdir -p $(RESULTS)
	cd $(BENCH) && ./structural_test

structural_plot:
	cd $(BENCH) && python3 plot_structural.py

# ── 任務 C：Unit tests ───────────────────────────────────────────
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

$(TESTS)/test_equivalence_checker: $(TESTS)/test_equivalence_checker.cpp \
                                    src/equivalence_checker.h src/circuit.h
	$(CXX) $(CXXFLAGS) $< -o $@

test: $(TESTS)/test_truth_table \
      $(TESTS)/test_edge_cases \
      $(TESTS)/test_topo_consistency \
      $(TESTS)/test_dfs_iterative \
      $(TESTS)/test_qm_minimizer \
      $(TESTS)/test_equivalence_checker
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
	@$(TESTS)/test_equivalence_checker || (echo ""; echo "test_equivalence_checker FAILED"; exit 1)
	@echo ""
	@echo "All tests passed."

# ── Quine-McCluskey 最小化器（單獨跑）───────────────────────
test_qm: $(TESTS)/test_qm_minimizer
	@$(TESTS)/test_qm_minimizer

# ── 等價性檢查器（單獨跑）───────────────────────────────────
test_equiv: $(TESTS)/test_equivalence_checker
	@$(TESTS)/test_equivalence_checker

# ── 清理 ─────────────────────────────────────────────────────────
clean:
	rm -f simulator \
	      $(BENCH)/hash_vs_linear \
	      $(BENCH)/topo_vs_naive \
	      $(BENCH)/ordered_vs_unordered \
	      $(BENCH)/random_circuit_gen \
	      $(BENCH)/scale_test \
	      $(BENCH)/structural_test \
	      $(TESTS)/test_truth_table \
	      $(TESTS)/test_edge_cases \
	      $(TESTS)/test_topo_consistency \
	      $(TESTS)/test_dfs_iterative \
	      $(TESTS)/test_qm_minimizer \
	      $(TESTS)/test_equivalence_checker
	rm -rf $(RESULTS)
