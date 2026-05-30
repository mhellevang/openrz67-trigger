#!/usr/bin/env bash
# Eksporterer base + lokk + lysleder til STL i én operasjon.
# Lyslederen (openrz67-lightpipe.stl) skal printes i KLART/transparent filament;
# base og lokk i vanlig opakt filament.
#
# Bruk:
#   ./export.sh                 # closure=screw (standard)
#   ./export.sh snap            # snap-fit-varianten
#   ./export.sh screw mine-stl  # egen output-mappe
#
set -euo pipefail
cd "$(dirname "$0")"

SCAD="openrz67-case.scad"
CLOSURE="${1:-screw}"
OUTDIR="${2:-stl}"

# Finn openscad (PATH, ellers macOS-appen)
if command -v openscad >/dev/null 2>&1; then
  OPENSCAD="openscad"
elif [ -x "/Applications/OpenSCAD.app/Contents/MacOS/OpenSCAD" ]; then
  OPENSCAD="/Applications/OpenSCAD.app/Contents/MacOS/OpenSCAD"
else
  echo "Fant ikke 'openscad'. Installer den eller legg den i PATH." >&2
  exit 1
fi

mkdir -p "$OUTDIR"

for part in base lid; do
  out="$OUTDIR/openrz67-${part}-${CLOSURE}.stl"
  echo "==> ${part} (${CLOSURE})  ->  ${out}"
  "$OPENSCAD" -o "$out" -D "closure=\"${CLOSURE}\"" -D "part=\"${part}\"" "$SCAD"
done

# Lysleder - uavhengig av closure, printes i klart filament
lp="$OUTDIR/openrz67-lightpipe.stl"
echo "==> lightpipe (klart filament)  ->  ${lp}"
"$OPENSCAD" -o "$lp" -D "part=\"lightpipe\"" "$SCAD"

# Lokk-tekst - kun teksten, for 2-farge-print (last som egen del/farge i sliceren)
lt="$OUTDIR/openrz67-lidtext.stl"
echo "==> lidtext (2. filamentfarge)  ->  ${lt}"
"$OPENSCAD" -o "$lt" -D "part=\"lidtext\"" "$SCAD"

echo "Ferdig. STL-filer ligger i ${OUTDIR}/ (lightpipe i klart filament, lidtext i 2. farge, resten opakt)"
