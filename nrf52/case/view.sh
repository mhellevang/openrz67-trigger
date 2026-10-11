#!/usr/bin/env bash
# Board + case in CAD Viewer: exports the board STEP from KiCad, builds GLBs into view/ and
# serves them. Prints the links; Ctrl-C stops the viewer.
#
# Usage:
#   ./view.sh
set -euo pipefail
cd "$(dirname "$0")"
for c in uv kicad-cli cadgen; do
  command -v "$c" >/dev/null 2>&1 || { echo "$c not found" >&2; exit 1; }
done

mkdir -p view
kicad-cli pcb export step --drill-origin --subst-models --no-dnp -o view/board.step ../kicad/openrz67-nrf.kicad_pcb >/dev/null
uv run view.py view/board.step view

echo
echo "Open the viewer URL below with ?file=nrf-closed.glb or ?file=nrf-floor-out.glb"
cd view && exec cadgen viewer
