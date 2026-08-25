# Upstream provenance

- Upstream: <https://github.com/hku-mars/ikd-Tree>
- Commit: `e2e3f4e9d3b95a9e66b1ba83dc98d4a05ed8a3c4` (the version referenced by
  hku-mars/FAST_LIO as the `include/ikd-Tree` submodule, i.e. the FAST-LIO2 /
  original Point-LIO era implementation).
- Files: `ikd_Tree.h`, `ikd_Tree.cpp`; license GPL-2.0 (see `LICENSE`).
- Local modifications: none so far (build-system integration only).

This is used by the robust localizer for incremental scan-to-map target
maintenance, not by the Point-LIO frontend (whose local map is OctVox/HKNN).
