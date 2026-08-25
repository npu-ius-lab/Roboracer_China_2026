#!/usr/bin/env bash
set -euo pipefail

usage() {
  echo "usage: $0 <old|new>" >&2
}

VERSION="${1:-}"
case "$VERSION" in
  old)
    EXPECTED_MAP="da3d51daf379fdacc3410a792771c9e4df62d40557b01ff925712aae4f66e755"
    EXPECTED_POSES="f0ff05cee64482ca4a265fa324583667a95d01d84a82c79704e24553fca90f0b"
    EXPECTED_DESCRIPTOR="3483ecf3092bf9255df5aa88da256796bbc2c5745ab8cb95412c69307c5e7ff6"
    ;;
  new)
    EXPECTED_MAP="926bc628e5ac93638423b13f37f1f1e5a247807bb099e1ebdf1942f271cec7b9"
    EXPECTED_POSES="a0abf95eaca31dc73002e066da27f366d3b7630321ac99aed0eaec8a9ed705b1"
    EXPECTED_DESCRIPTOR="e87b3e379e007221494141219e316ff57f0da528f61b6a1702f1721960be24ad"
    ;;
  *) usage; exit 2 ;;
esac

MAP_DIR="$(rosparam get /pointlio_robust_localizer/map_dir 2>/dev/null || true)"
DESCRIPTOR="$(rosparam get /pointlio_robust_localizer/descriptor_cache_path 2>/dev/null || true)"
if [[ -z "$MAP_DIR" || ! -d "$MAP_DIR" ]]; then
  echo "[FAIL] localization map_dir is absent or unreadable: ${MAP_DIR:-unset}" >&2
  exit 1
fi
if [[ -z "$DESCRIPTOR" || ! -f "$DESCRIPTOR" ]]; then
  echo "[FAIL] localization descriptor cache is absent: ${DESCRIPTOR:-unset}" >&2
  exit 1
fi

verify() {
  local label="$1" path="$2" expected="$3" actual
  [[ -f "$path" ]] || { echo "[FAIL] missing $label: $path" >&2; exit 1; }
  actual="$(sha256sum "$path" | awk '{print $1}')"
  if [[ "$actual" != "$expected" ]]; then
    echo "[FAIL] localization $label belongs to the wrong map version" >&2
    echo "       path=$path" >&2
    echo "       expected=$expected" >&2
    echo "       actual=$actual" >&2
    exit 1
  fi
}

verify point_lio_map "$MAP_DIR/point_lio_map.pcd" "$EXPECTED_MAP"
verify poses "$MAP_DIR/poses_tum.txt" "$EXPECTED_POSES"
verify descriptor "$DESCRIPTOR" "$EXPECTED_DESCRIPTOR"
echo "[OK] localization identity=$VERSION map_dir=$MAP_DIR"
