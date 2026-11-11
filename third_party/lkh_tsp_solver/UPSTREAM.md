# Upstream provenance

- Upstream project: FUEL — `Fast-Exploration`
- Upstream path: `src/fuel_planner/utils/lkh_tsp_solver`
- Local upstream checkout: `/home/whd/Fast-Exploration/src/fuel_planner/utils/lkh_tsp_solver`
- Vendored into: `src/exploration/third_party/lkh_tsp_solver`
- Vendoring date: 2026-11-11

## Copied verbatim

- `include/*.h` (11 files, including `lkh_tsp_solver/lkh_interface.h`)
- `src/*.c` (105 files)
- `src/lkh_interface.cpp`

## Deliberately omitted

- `src/LKHmain.c` — standalone executable entry point with hard-coded paths to
  example resources
- `resource/` — example parameter/problem/tour files
- `CMakeLists.txt`, `package.xml` — replaced by the host package build
- `lkh_tsp` executable target

## Local modifications

- `src/lkh_interface.cpp` include path is `#include <lkh_tsp_solver/lkh_interface.h>`
  (unchanged from upstream, resolved through the added include directory)
- No source-level changes to the LKH core files.

## Licensing

The upstream bundle ships **no LICENSE file**. LKH is Copyright (c) Keld
Helsgaun; see `LICENSE` in this directory for the attribution notice and the
upstream terms pointer. `src/Delaunay.c` carries its own RMIT copyright notice,
which is preserved verbatim.
