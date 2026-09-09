# SPDX-License-Identifier: MIT
#
# Build the evaluation workload against the external scx_fresh checkout.
#
# NOTE: You need bpftool, clang, and libbpf development headers installed.

BPF_CLANG ?= clang
BPF_CFLAGS ?= -O2 -g -target bpf

CXX ?= g++
CC ?= gcc
PYTHON ?= python3

BUILD_DIR ?= build
SCX_FRESH_DIR ?= $(abspath ../scx_fresh)
export SCX_FRESH_DIR

# Mode:
#   FRESH_FULL_SWITCH=0 => safer: only SCHED_EXT tasks scheduled by SCX (partial switch)
#   FRESH_FULL_SWITCH=1 => convenient: everything scheduled by SCX (full switch)
FRESH_FULL_SWITCH ?= 0

VMLINUX_H := $(BUILD_DIR)/vmlinux.h
BPF_OBJ   := $(BUILD_DIR)/scx_slam_fresh.bpf.o
SKEL_H    := $(BUILD_DIR)/scx_slam_fresh.skel.h

SCX_SOURCES := $(wildcard $(SCX_FRESH_DIR)/bpf/* $(SCX_FRESH_DIR)/src/* $(SCX_FRESH_DIR)/include/*) $(SCX_FRESH_DIR)/Makefile

all: bpf userspace

userspace: $(BUILD_DIR)/scx_slam_fresh_user $(BUILD_DIR)/slam_pipeline_demo $(BUILD_DIR)/dependent_workload

$(BUILD_DIR)/dependent_workload: demo/dependent_workload.cpp demo/dependent_graph.h $(SCX_FRESH_DIR)/src/freshqos.c $(wildcard $(SCX_FRESH_DIR)/include/* $(SCX_FRESH_DIR)/src/*.h) | $(BUILD_DIR)
	$(CXX) -std=c++17 -O2 -Wall -Wextra -Werror -I"$(SCX_FRESH_DIR)/include" -I"$(SCX_FRESH_DIR)/src" $< "$(SCX_FRESH_DIR)/src/freshqos.c" -lbpf -lpthread -o $@

bpf: $(SKEL_H) $(BUILD_DIR)/scx_fresh.revision $(BUILD_DIR)/scx_fresh.diff

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

$(BUILD_DIR)/scx_slam_fresh_user $(BPF_OBJ) $(SKEL_H) $(VMLINUX_H) $(BUILD_DIR)/scx_fresh.revision $(BUILD_DIR)/scx_fresh.diff &: $(SCX_SOURCES) Makefile | $(BUILD_DIR)
	$(MAKE) -C "$(SCX_FRESH_DIR)" BUILD_DIR=build FRESH_FULL_SWITCH=$(FRESH_FULL_SWITCH) BPF_CLANG="$(BPF_CLANG)" BPF_CFLAGS="$(BPF_CFLAGS)"
	cp --remove-destination "$(SCX_FRESH_DIR)/build/scx_fresh.bpf.o" "$(BPF_OBJ)"
	cp --remove-destination "$(SCX_FRESH_DIR)/build/scx_fresh.skel.h" "$(SKEL_H)"
	cp --remove-destination "$(SCX_FRESH_DIR)/build/vmlinux.h" "$(VMLINUX_H)"
	cp --remove-destination "$(SCX_FRESH_DIR)/build/scx_fresh" "$(BUILD_DIR)/scx_slam_fresh_user"
	git -C "$(SCX_FRESH_DIR)" rev-parse HEAD > "$(BUILD_DIR)/scx_fresh.revision"
	git -C "$(SCX_FRESH_DIR)" diff HEAD -- > "$(BUILD_DIR)/scx_fresh.diff"

$(BUILD_DIR)/slam_pipeline_demo: demo/slam_pipeline_demo.cpp demo/window_metrics.h ros2/scx_slam_executor/include/scx_slam_executor/application_stages.hpp $(SCX_FRESH_DIR)/src/freshqos.c $(wildcard $(SCX_FRESH_DIR)/include/* $(SCX_FRESH_DIR)/src/*.h) Makefile | $(BUILD_DIR)
	$(CXX) -O2 -g -I"$(SCX_FRESH_DIR)/include" -I"$(SCX_FRESH_DIR)/src" demo/slam_pipeline_demo.cpp "$(SCX_FRESH_DIR)/src/freshqos.c" \
		-lbpf -lelf -lz -lpthread -o $@

clean:
	rm -rf $(BUILD_DIR)

$(BUILD_DIR)/test_dependent_graph: tests/test_dependent_graph.cpp demo/dependent_graph.h | $(BUILD_DIR)
	$(CXX) -std=c++17 -O2 -Wall -Wextra -Werror $< -o $@

.PHONY: test-graph
test-graph: $(BUILD_DIR)/test_dependent_graph $(BUILD_DIR)/dependent_workload
	$(BUILD_DIR)/test_dependent_graph
	$(PYTHON) tests/test_dependent_workload.py

test-demo: $(BUILD_DIR)/slam_pipeline_demo
	DEMO_BIN="$(abspath $(BUILD_DIR)/slam_pipeline_demo)" $(PYTHON) tests/test_demo_cli.py

test-e4:
	$(PYTHON) tests/test_e4_eval.py
	$(PYTHON) tests/test_e4_execution.py
	$(PYTHON) tests/test_e4_perf.py

test-scheduler-mode: $(BUILD_DIR)/scx_slam_fresh_user
	LOADER_BIN="$(abspath $(BUILD_DIR)/scx_slam_fresh_user)" EXPECTED_FULL_SWITCH="$(FRESH_FULL_SWITCH)" $(PYTHON) "$(SCX_FRESH_DIR)/tests/test_scheduler_mode.py"

test-slice: $(VMLINUX_H)
	VMLINUX_H="$(abspath $(VMLINUX_H))" CC="$(CC)" $(PYTHON) "$(SCX_FRESH_DIR)/tests/test_enqueue_slice.py"

$(BUILD_DIR)/test_window_metrics: tests/test_window_metrics.cpp demo/window_metrics.h | $(BUILD_DIR)
	$(CXX) -O2 -Wall -Wextra tests/test_window_metrics.cpp -o $@

test-window: $(BUILD_DIR)/test_window_metrics $(BUILD_DIR)/slam_pipeline_demo
	$(BUILD_DIR)/test_window_metrics
	DEMO_BIN="$(abspath $(BUILD_DIR)/slam_pipeline_demo)" $(PYTHON) tests/test_window_stats.py

ros2:
	scripts/run_ros2.sh build

test-ros2:
	scripts/run_ros2.sh test

.PHONY: all bpf userspace clean test-demo test-e4 test-scheduler-mode test-window test-slice ros2 test-ros2
