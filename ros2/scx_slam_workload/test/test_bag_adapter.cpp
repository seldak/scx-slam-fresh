// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>
#include <scx_slam_workload/bag_adapter.hpp>

#include <stdexcept>
#include <unistd.h>

namespace
{
class IndexFile
{
public:
  IndexFile()
  {
    const int fd = mkstemp(path);
    if (fd < 0) {throw std::runtime_error("mkstemp");}
    close(fd);
  }
  ~IndexFile() {unlink(path);}
  void write(const char * content) {std::ofstream(path) << content;}
  char path[40] = "/tmp/scx-source-index-XXXXXX";
};
}

TEST(BagAdapter, BagOrdinalIsIndependentOfFirstDeliveredMessage)
{
  IndexFile file;
  file.write("1403636579758555500 1\n1403636579763555500 2\n1403636579768555500 3\n");
  const scx_slam_workload::SourceJobIndex index(file.path);
  EXPECT_EQ(index.job_id(1403636579768555500ULL, 1), 3U);
  EXPECT_EQ(index.job_id(1403636579768555500ULL, 3), 3U);
  EXPECT_THROW(index.job_id(1403636579768555501ULL, 4), std::runtime_error);
  EXPECT_EQ(scx_slam_workload::SourceJobIndex().job_id(42, 7), 7U);
}

TEST(BagAdapter, RejectsMalformedAndNonMonotonicSourceIndexes)
{
  IndexFile file;
  for (const char * contents : {"", "1", "1 1\n1 2\n", "2 1\n1 2\n", "1 2\n", "garbage"}) {
    file.write(contents);
    EXPECT_THROW(scx_slam_workload::SourceJobIndex{file.path}, std::runtime_error);
  }
}

TEST(BagAdapter, PreservesSourceTimeAndUsesMonotonicRelease)
{
  builtin_interfaces::msg::Time source;
  source.sec = 123;
  source.nanosec = 456;

  const auto job = scx_slam_workload::make_stamped_job(7, 9001, source);
  EXPECT_EQ(job.job_id, 7U);
  EXPECT_EQ(job.release_ts_ns, 9001U);
  EXPECT_EQ(job.source_ts_ns, 123000000456ULL);
}

TEST(BagAdapter, AcceptsAZeroSourceStamp)
{
  builtin_interfaces::msg::Time source;
  const auto job = scx_slam_workload::make_stamped_job(1, 2, source);
  EXPECT_EQ(job.source_ts_ns, 0U);
}

TEST(BagAdapter, RejectsInvalidIdentityAndTime)
{
  builtin_interfaces::msg::Time source;
  EXPECT_THROW(scx_slam_workload::make_stamped_job(0, 1, source), std::invalid_argument);
  EXPECT_THROW(scx_slam_workload::make_stamped_job(1, 0, source), std::invalid_argument);

  source.sec = -1;
  EXPECT_THROW(scx_slam_workload::make_stamped_job(1, 1, source), std::invalid_argument);
}
