#!/bin/sh
# Fetch STEP + WRL 3D models for every LCSC part on the board into openrz67.3dshapes/.
# WRL files are committed (small, used for rendering); STEP files are gitignored and
# only needed for `kicad-cli pcb export step --subst-models`. Requires network.
#   pip install easyeda2kicad   (or: uv tool install easyeda2kicad)
set -eu
cd "$(dirname "$0")/.."
ids=$(grep -o '"C[0-9]*"$' out/openrz67-bom.csv | tr -d '"' | sort -u)
for id in $ids; do
  easyeda2kicad --3d --lcsc_id "$id" --output ./openrz67 --overwrite
done
