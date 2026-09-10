#!/usr/bin/env bash

set -euo pipefail

repo_root=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
build_dir=${1:-"$repo_root/build"}
runs=${2:-3}
output_dir=${3:-$(mktemp -d -t tinglecam-planning-bench.XXXXXX)}
binary="$build_dir/tinglecam"

if [[ ! -x "$binary" ]]; then
  echo "TingleCAM executable not found at $binary" >&2
  exit 1
fi
if [[ ! "$runs" =~ ^[1-9][0-9]*$ ]]; then
  echo "Run count must be a positive integer" >&2
  exit 1
fi

mkdir -p "$output_dir"
echo "Planning benchmark output: $output_dir"

run_case() {
  local name=$1
  local part=$2
  local stock=$3
  local expected_hash=
  shift 3
  for ((run = 1; run <= runs; ++run)); do
    local stem="$output_dir/${name}-${run}"
    /usr/bin/time \
      -f "planning_benchmark case=$name run=$run wall=%e user=%U sys=%S rss_kb=%M" \
      -o "$stem.time" \
      "$binary" \
      --part "$repo_root/$part" \
      --stock "$repo_root/$stock" \
      --machine makera-z1 \
      --material hdpe \
      --profile-planning \
      --gcode "$stem.nc" \
      "$@" \
      >"$stem.out" 2>"$stem.err"
    local actual_hash
    actual_hash=$(sha256sum "$stem.nc" | awk '{print $1}')
    if [[ -z "$expected_hash" ]]; then
      expected_hash=$actual_hash
    elif [[ "$actual_hash" != "$expected_hash" ]]; then
      echo "Non-deterministic G-code for $name: $actual_hash != $expected_hash" >&2
      exit 1
    fi
    echo "$actual_hash  $stem.nc"
    rg '^planning_profile' "$stem.err"
    cat "$stem.time"
  done
}

ctest --test-dir "$build_dir" --output-on-failure

# Representative roughing, simulation scaling, wave-count pressure, complex
# boundaries, and contour finishing. Each case prints its deterministic G-code
# hash alongside phase times and scale counters.
run_case L_normal tests/fixtures/meshes/final/L.obj tests/fixtures/meshes/stock/L.obj --path-pitch 0.2
run_case Rtwist_normal tests/fixtures/meshes/bench/Rtwist.obj tests/fixtures/meshes/bench/Rtwist_stock.obj --path-pitch 0.2
run_case L_fine_pitch tests/fixtures/meshes/final/L.obj tests/fixtures/meshes/stock/L.obj --path-pitch 0.1
run_case sSs_small_stepover tests/fixtures/meshes/final/sSs.obj tests/fixtures/meshes/stock/sSs.obj \
  --stepover 0.2 --path-pitch 0.2
run_case L_finish tests/fixtures/meshes/final/L.obj tests/fixtures/meshes/stock/L.obj \
  --mode finish --path-pitch 0.2
