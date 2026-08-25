#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PACKAGE="$ROOT/src/f1tenth_dynamic_mpcc"
export PYTHONPATH="$PACKAGE/python${PYTHONPATH:+:$PYTHONPATH}"
python3 -m unittest discover -s "$PACKAGE/tests" -v
python3 -m unittest discover -s "$ROOT/lateral_identification/tests" -v
python3 -m py_compile "$PACKAGE"/python/f1tenth_dynamic_mpcc/*.py "$PACKAGE"/scripts/*.py
python3 -m py_compile "$ROOT"/lateral_identification/*.py
bash -n "$ROOT"/scripts/*.sh
