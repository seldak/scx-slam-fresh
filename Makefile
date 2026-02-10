# SPDX-License-Identifier: GPL-2.0
#
# Build scx_slam_fresh (BPF + userspace).
#
# NOTE: You need bpftool, clang, and libbpf development headers installed.

BPF_CLANG ?= clang
BPF_CFLAGS ?= -O2 -g -target bpf

CXX ?= g++
CC ?= gcc

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

$(VMLINUX_H): $(BUILD_DIR)
	$(VMLINUX_GEN) $(VMLINUX_H)

$(BPF_OBJ): $(VMLINUX_H) bpf/scx_slam_fresh.bpf.c include/scx_slam_fresh_shared.h | $(BUILD_DIR)
	$(BPF_CLANG) $(BPF_CFLAGS) -DSLAM_FULL_SWITCH=$(SLAM_FULL_SWITCH) \
		-I$(BUILD_DIR) -Iinclude -Ibpf \
		-c bpf/scx_slam_fresh.bpf.c -o $(BPF_OBJ)

$(SKEL_H): $(BPF_OBJ)
	bpftool gen skeleton $(BPF_OBJ) > $(SKEL_H)

$(BUILD_DIR)/scx_slam_fresh_user: src/scx_slam_fresh_user.c src/slamqos.c include/scx_slam_fresh_shared.h | $(BUILD_DIR)
	$(CC) -O2 -g -I$(BUILD_DIR) -Iinclude -Isrc \
		src/scx_slam_fresh_user.c src/slamqos.c \
		-lbpf -lelf -lz -o $@

$(BUILD_DIR)/slam_pipeline_demo: demo/slam_pipeline_demo.cpp src/slamqos.c include/scx_slam_fresh_shared.h | $(BUILD_DIR)
	$(CXX) -O2 -g -Iinclude -Isrc demo/slam_pipeline_demo.cpp src/slamqos.c \
		-lbpf -lelf -lz -lpthread -o $@

userspace: $(BUILD_DIR)/scx_slam_fresh_user $(BUILD_DIR)/slam_pipeline_demo

clean:
	rm -rf $(BUILD_DIR)

.PHONY: all clean
