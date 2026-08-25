#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 3 ]]; then
  echo "usage: save_map_relative.sh OUTPUT_DIR OUTPUT_NAME MAP_TOPIC" >&2
  exit 2
fi

output_dir=$1
output_name=$2
map_topic=$3

mkdir -p -- "$output_dir"
cd -- "$output_dir"
exec rosrun map_server map_saver "map:=$map_topic" -f "$output_name"
