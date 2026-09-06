// SPDX-License-Identifier: MIT

#include <scx_slam_executor/slamqos.h>

#include <gtest/gtest.h>

#include <sys/syscall.h>
#include <unistd.h>

TEST(SlamqosIdentity, PacksCurrentProcessAndThread)
{
  const uint64_t pid_tgid = slamqos_pid_tgid_self();
  EXPECT_EQ(static_cast<uint32_t>(pid_tgid >> 32), static_cast<uint32_t>(getpid()));
  EXPECT_EQ(
    static_cast<uint32_t>(pid_tgid),
    static_cast<uint32_t>(syscall(SYS_gettid)));
}
