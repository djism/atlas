#!/usr/bin/env bash
# Compiles the Atlas C++ core plus wasm/bindings.cpp to a single-file ES module
# for the web sandbox. Requires the Emscripten SDK (default: ~/emsdk); it is
# activated for this script only and not added to your shell profile.
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
EMSDK_DIR="${EMSDK:-$HOME/emsdk}"
if [[ ! -f "$EMSDK_DIR/emsdk_env.sh" ]]; then
    echo "Emscripten SDK not found at $EMSDK_DIR (set EMSDK=/path/to/emsdk)" >&2
    exit 1
fi
# shellcheck disable=SC1091
source "$EMSDK_DIR/emsdk_env.sh" >/dev/null 2>&1

OUT="$ROOT/web/src/engine/generated"
mkdir -p "$OUT"

em++ -std=c++17 -O2 -Wall -Wextra -Wpedantic -Werror \
    -fwasm-exceptions \
    -I"$ROOT/include" \
    "$ROOT/src/geometry.cpp" "$ROOT/src/simulation.cpp" "$ROOT/src/matching.cpp" \
    "$ROOT/src/icp.cpp" "$ROOT/src/tracker.cpp" "$ROOT/src/planner.cpp" "$ROOT/wasm/bindings.cpp" \
    -lembind \
    -sMODULARIZE=1 -sEXPORT_ES6=1 -sEXPORT_NAME=createAtlasEngine \
    -sENVIRONMENT=web,worker -sSINGLE_FILE=1 -sALLOW_MEMORY_GROWTH=1 \
    --emit-tsd "$OUT/atlas_engine.d.ts" \
    -o "$OUT/atlas_engine.js"

echo "Built $(du -h "$OUT/atlas_engine.js" | cut -f1) -> web/src/engine/generated/atlas_engine.js ($(em++ --version | head -1 | sed 's/.*) //'))"
