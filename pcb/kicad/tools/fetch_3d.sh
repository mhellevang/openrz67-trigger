#!/bin/sh
# Fetch STEP + WRL 3D models for every LCSC part in the BOM into openrz67.3dshapes/.
# WRL files are committed (small, used for rendering); STEP files are gitignored and
# only needed for tools/export_step.sh. Requires network.
set -eu
cd "$(dirname "$0")/.."
if command -v easyeda2kicad >/dev/null 2>&1; then E=easyeda2kicad
elif command -v uvx >/dev/null 2>&1; then E="uvx easyeda2kicad"
elif command -v pipx >/dev/null 2>&1; then E="pipx run easyeda2kicad"
else
  [ -x .venv-tools/bin/easyeda2kicad ] || { python3 -m venv .venv-tools && .venv-tools/bin/pip install -q easyeda2kicad; }
  E=.venv-tools/bin/easyeda2kicad
fi
ids=$(grep -o '"C[0-9]*"$' out/openrz67-bom.csv | tr -d '"' | sort -u)
for id in $ids; do
  $E --3d --lcsc_id "$id" --output ./openrz67 --overwrite
done
