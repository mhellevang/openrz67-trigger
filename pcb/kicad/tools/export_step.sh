#!/bin/sh
# Export the assembled board as STEP (out/openrz67.step, gitignored ~20 MB).
# Needs the per-part .step models; fetches them if any are missing.
set -eu
cd "$(dirname "$0")/.."
K=/Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli
for w in openrz67.3dshapes/*.wrl; do
  [ -f "${w%.wrl}.step" ] || { tools/fetch_3d.sh; break; }
done
"$K" pcb export step --subst-models --no-dnp -f -o out/openrz67.step openrz67.kicad_pcb
