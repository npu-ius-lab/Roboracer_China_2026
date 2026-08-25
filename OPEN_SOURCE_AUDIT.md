# Open-source publication audit

The algorithm and simulation release is technically prepared, but publishing
the repository does not by itself relicense third-party or previously
proprietary files.

Items that remain subject to their component licenses after publication:

- `localization/src/point_lio_sam_lighterbev/package.xml` currently declares
  `Proprietary`. Replace that declaration and add a license file only if the
  copyright owner has approved an open-source license.
- `localization/src/pcd2pgm_package/pcd2pgm/package.xml` and
  `vehicle/src/tianracer_gazebo/package.xml` declare `TODO`; identify their
  upstream license and preserve the required notice.
- TianRacer packages declare GPLv3. Keep their GPL notices and account for the
  GPL's distribution requirements when deciding the repository-wide license.
- Point-LIO, Livox, Nano-GICP, Patchwork++, IKFoM, ikd-tree, Ackermann messages,
  RapidJSON and other vendored components retain their own licenses. Do not
  replace those notices with the repository's top-level license.
- The Point-LIO Gazebo track model is derived from
  `npu-ius-lab/Roboracer_China_2025` at commit
  `347460b7173408574076de7dbe3f86afd9b33365` and is GPLv3. Its source Blend
  file, preview, map files and `LICENSE.GPL-3.0` are included with the model.
- `simulation_ws/src/livox_laser_simulation` is a modified ROS Noetic/Gazebo
  11 port of Livox's MIT package at commit
  `1cce1073633a062b92e30243a4c2920e45551bb5`; retain its `LICENSE` and source
  attribution.
- `vehicle/src/tianracer_gazebo/scripts/upload.py` reads its SFTP password from
  `TIANRACER_SFTP_PASSWORD` or a masked run-time prompt. No password is stored
  in the publication tree. Rotate any credential previously used outside an
  isolated vehicle LAN.
- The private working snapshot preserves prebuilt executables and shared
  libraries for rollback. The clean public branches omit those binaries and
  build products; all maintained algorithm and simulation source is retained.

The clean publication repository is generated independently so credentials or
binaries that existed in the private snapshot's history cannot be recovered
from the public Git history.
