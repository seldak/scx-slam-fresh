// SPDX-License-Identifier: GPL-2.0
#pragma once
#include <algorithm>
#include <cstdint>
#include <vector>

namespace scx_slam_workload
{
// Each timestamp records completion of one unchanged 1000us CPU-work loop.
// Count completion events in the exact wall window, including iterations that
// straddle its beginning and excluding ones that finish at/after its end.
inline uint64_t window_iterations(
  const std::vector<uint64_t> & completions, uint64_t start, uint64_t end)
{
  return std::lower_bound(completions.begin(), completions.end(), end) -
         std::lower_bound(completions.begin(), completions.end(), start);
}
}
