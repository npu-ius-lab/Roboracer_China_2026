#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
V1="$ROOT/src/f1tenth_dynamic_mpcc/config/residual/residual_markov_v1.yaml"
V2="$ROOT/src/f1tenth_dynamic_mpcc/config/residual/residual_v2p1_physics.yaml"
V1_SHA256="16a18c94442f90415cf8bd0cea81b7c7887e0624adfa1f3e80a26e44246fe3fc"

source "$ROOT/scripts/ros_env.sh"
if ! rospack find f1tenth_residual_dynamics >/dev/null 2>&1; then
  echo "[FAIL] f1tenth_residual_dynamics is not available in the ROS overlay" >&2
  exit 1
fi

echo "[CHECK] V1"
V1_INFO="$(rosrun f1tenth_residual_dynamics residual_model_inspect "$V1")"
echo "$V1_INFO"
if [[ "$V1_INFO" != *"schema=1 feature_set=markov_v1"* ]]; then
  echo "[FAIL] V1 schema or feature set changed" >&2
  exit 1
fi
if [[ "$(sha256sum "$V1" | awk '{print $1}')" != "$V1_SHA256" ]]; then
  echo "[FAIL] V1 rollback model checksum changed" >&2
  exit 1
fi
if [[ -f "$V2" ]]; then
  echo "[CHECK] V2.1"
  V2_INFO="$(rosrun f1tenth_residual_dynamics residual_model_inspect "$V2")"
  echo "$V2_INFO"
  if [[ "$V2_INFO" != *"schema=2 feature_set=physics_v2"* ]]; then
    echo "[FAIL] V2.1 schema or feature set is not deployable" >&2
    exit 1
  fi
else
  echo "[INFO] V2.1 is not installed; V1 remains the only selectable profile."
fi

python3 - "$ROOT/src/f1tenth_dynamic_mpcc/config/controller.yaml" <<'PY'
import sys, yaml
config = yaml.safe_load(open(sys.argv[1], encoding="utf-8"))
residual = config["residual_dynamics"]
if residual.get("required_feature_set") != "markov_v1":
    raise SystemExit("[FAIL] controller.yaml default is not V1")
if not str(residual.get("model_path", "")).endswith("residual_markov_v1.yaml"):
    raise SystemExit("[FAIL] controller.yaml default model is not V1")
print("[OK] controller.yaml default profile is V1")
PY
