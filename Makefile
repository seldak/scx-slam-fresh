# SPDX-License-Identifier: MIT
#
# Build scx_slam_fresh (BPF + userspace).
#
# NOTE: You need bpftool, clang, and libbpf development headers installed.

BPF_CLANG ?= clang
BPF_CFLAGS ?= -O2 -g -target bpf

CXX ?= g++
CC ?= gcc
PYTHON ?= python3

BUILD_DIR ?= build
PIN_DIR_DEFAULT ?= /sys/fs/bpf/scx_slam_fresh

# Mode:
#   SLAM_FULL_SWITCH=0 => safer: only SCHED_EXT tasks scheduled by SCX (partial switch)
#   SLAM_FULL_SWITCH=1 => convenient: everything scheduled by SCX (full switch)
SLAM_FULL_SWITCH ?= 0

VMLINUX_H := $(BUILD_DIR)/vmlinux.h
BPF_OBJ   := $(BUILD_DIR)/scx_slam_fresh.bpf.o
SKEL_H    := $(BUILD_DIR)/scx_slam_fresh.skel.h

VMLINUX_GEN := scripts/gen_vmlinux_h.sh

all: bpf userspace

userspace: $(BUILD_DIR)/scx_slam_fresh_user $(BUILD_DIR)/slam_pipeline_demo

bpf: $(SKEL_H)

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

$(VMLINUX_H): | $(BUILD_DIR)
	$(VMLINUX_GEN) $(VMLINUX_H)

$(BPF_OBJ): $(VMLINUX_H) bpf/scx_slam_fresh.bpf.c bpf/execution_trace.bpf.h include/scx_slam_fresh_shared.h include/scx_execution_trace.h | $(BUILD_DIR)
	$(BPF_CLANG) $(BPF_CFLAGS) -DSLAM_FULL_SWITCH=$(SLAM_FULL_SWITCH) \
		-I$(BUILD_DIR) -Iinclude -Ibpf \
		-c bpf/scx_slam_fresh.bpf.c -o $(BPF_OBJ)

$(SKEL_H): $(BPF_OBJ)
	bpftool gen skeleton $(BPF_OBJ) > $(SKEL_H)

$(BUILD_DIR)/scx_slam_fresh_user: src/scx_slam_fresh_user.c src/slamqos.c include/scx_slam_fresh_shared.h include/scx_execution_trace.h $(SKEL_H) | $(BUILD_DIR)
	$(CC) -O2 -g -I$(BUILD_DIR) -Iinclude -Isrc \
		src/scx_slam_fresh_user.c src/slamqos.c \
		-lbpf -lelf -lz -o $@

$(BUILD_DIR)/slam_pipeline_demo: demo/slam_pipeline_demo.cpp demo/window_metrics.h src/slamqos.c include/scx_slam_fresh_shared.h include/scx_execution_trace.h | $(BUILD_DIR)
	$(CXX) -O2 -g -Iinclude -Isrc demo/slam_pipeline_demo.cpp src/slamqos.c \
		-lbpf -lelf -lz -lpthread -o $@

userspace: $(BUILD_DIR)/scx_slam_fresh_user $(BUILD_DIR)/slam_pipeline_demo

clean:
	rm -rf $(BUILD_DIR)

test-demo: $(BUILD_DIR)/slam_pipeline_demo
	DEMO_BIN="$(abspath $(BUILD_DIR)/slam_pipeline_demo)" $(PYTHON) tests/test_demo_cli.py

test-e4:
	$(PYTHON) tests/test_e4_eval.py
	$(PYTHON) tests/test_e4_execution.py
	$(PYTHON) tests/test_e4_perf.py

test-scheduler-mode: $(BUILD_DIR)/scx_slam_fresh_user
	LOADER_BIN="$(abspath $(BUILD_DIR)/scx_slam_fresh_user)" EXPECTED_FULL_SWITCH="$(SLAM_FULL_SWITCH)" $(PYTHON) tests/test_scheduler_mode.py

test-slice: $(VMLINUX_H)
	VMLINUX_H="$(abspath $(VMLINUX_H))" CC="$(CC)" $(PYTHON) tests/test_enqueue_slice.py

$(BUILD_DIR)/test_window_metrics: tests/test_window_metrics.cpp demo/window_metrics.h | $(BUILD_DIR)
	$(CXX) -O2 -Wall -Wextra tests/test_window_metrics.cpp -o $@

test-window: $(BUILD_DIR)/test_window_metrics $(BUILD_DIR)/slam_pipeline_demo
	$(BUILD_DIR)/test_window_metrics
	DEMO_BIN="$(abspath $(BUILD_DIR)/slam_pipeline_demo)" $(PYTHON) tests/test_window_stats.py

ros2:
	scripts/run_ros2.sh build

test-ros2:
	scripts/run_ros2.sh test

.PHONY: all clean test-demo test-e4 test-scheduler-mode test-window test-slice ros2 test-ros2
