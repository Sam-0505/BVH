# Benchmark results

Every performance claim made anywhere in this project — docs, commit messages, the
interview knowledge base, conversation — must trace to a file in this directory. If a
number is not recorded here, it does not exist and must be written as `UNVERIFIED`.

## Format

One JSON file per run: `<scene>-<phase>-<YYYYMMDD-HHMMSS>.json`

```json
{
  "timestamp": "2026-09-20T12:30:00Z",
  "git_commit": "abc1234",
  "machine": {
    "cpu": "Apple M-series (exact model)",
    "gpu": "via MoltenVK / Metal",
    "os": "macOS 24.6",
    "compiler": "AppleClang 17.0.0",
    "build_type": "Release"
  },
  "scene": { "mesh": "bunny.obj", "triangles": 69451 },
  "config": {
    "leaf_size": 4,
    "split_strategy": "sah",
    "sah_bins": 16,
    "max_depth": 32
  },
  "build": {
    "build_time_ms": 0.0,
    "node_count": 0,
    "leaf_count": 0,
    "max_depth": 0,
    "avg_leaf_depth": 0.0,
    "memory_bytes": 0
  },
  "query": {
    "ray_count": 0,
    "runs": 0,
    "query_time_ms_median": 0.0,
    "query_time_ms_min": 0.0,
    "nodes_visited_avg": 0.0,
    "triangles_tested_avg": 0.0
  }
}
```

## Methodology rules

- **Release builds only.** A debug-build timing is not a benchmark.
- **Report median of N runs** (N >= 5), plus min. Never a single sample.
- **Discard warmup iterations** so you measure steady state, not first-touch page faults
  and cold caches.
- **Fixed ray sets.** Seed the RNG and record the seed; comparing two configurations
  against different rays measures nothing.
- **Change one variable at a time.** Leaf size, split strategy, and bin count are compared
  independently, on identical scenes and identical ray sets.
- **Record the commit.** A result that cannot be tied to a revision cannot be reproduced.
- **CPU and GPU timings are not directly comparable** unless the writeup states what is
  included: transfer, submission, and synchronization, or kernel time alone. Say which.
- GPU numbers here come from MoltenVK translating to Metal, not a native Vulkan driver.
  Label them accordingly.
