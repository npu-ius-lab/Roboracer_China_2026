# Upstream provenance

- Upstream: <https://github.com/Liansheng-Wang/Super-LIO>
- Branch: `ros1`
- Imported component: `src/super_lio/include/OctVoxMap/` only
  (`OctVoxMap.hpp`, `HKNN_list60_gem.h`, `tsl/`).
- Upstream license: GPL-3.0 (see `LICENSE`). The `tsl/` robin-map headers are
  MIT licensed by their respective authors.
- Local modifications:
  - Added `numVoxels()` / `numRepresentatives()` public accessors to
    `OctVoxMap` for the Point-LIO map statistics diagnostics.
  - No changes to HKNN search order, early-stop thresholds, subvoxel
    compression constants, or KNN semantics.
