# lkh_tsp_solver (vendored)

Vendored copy of the LKH TSP solver wrapper used by FUEL
(`Fast-Exploration/src/fuel_planner/utils/lkh_tsp_solver`), trimmed for use as a
plain static library inside `semantic_exploration`.

## What is included

- `include/` — LKH 2.x core headers plus `lkh_tsp_solver/lkh_interface.h`
- `src/*.c` — all LKH core sources except `LKHmain.c` (no standalone entry point)
- `src/lkh_interface.cpp` — `solveTSPLKH(const char* parameter_file)` wrapper

The upstream `resource/` directory (example `single.par`/`single.tsp`/`single.txt`)
and the `lkh_tsp` test executable are **not** vendored. The global tour planner
writes its own problem/parameter/tour files under
`/tmp/semantic_exploration_lkh_<pid>/` at runtime.

## Interface

```cpp
extern "C" {
#include "LKH.h"
#include "Genetic.h"
}
int solveTSPLKH(const char* input_file);
```

`solveTSPLKH` reads the parameter file, solves the problem it references, and
writes the tour to `OUTPUT_TOUR_FILE`. LKH keeps global state, so calls must be
serialized (the planner uses a process-wide mutex).

## Build

Built by the package `CMakeLists.txt` as the `lkh_tsp_solver` static library:

- C sources: `-std=gnu99 -w`
- C++ wrapper: package C++17 flags
- links `m`
- include dirs: this `include/` and `include/lkh_tsp_solver/`

## Problem format used by this project

`single.tsp` is a TSPLIB ATSP with `EDGE_WEIGHT_FORMAT : FULL_MATRIX`, node 1 is
the robot, nodes 2..N are tasks, integer costs are scaled by `cost_scale`.
`single.par` sets `RUNS = 1`, `GAIN23 = NO`, `TRACE_LEVEL = 0` and an optional
`TIME_LIMIT`.
