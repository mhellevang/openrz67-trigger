#!/usr/bin/env bash
# Export base + lid + light pipe to STL in one go.
# The light pipe (openrz67-lightpipe.stl) prints in CLEAR/transparent filament; base and
# lid in normal opaque filament. The lid text (off by default) is fused onto the lid and
# coloured by a filament change at the top layer - it is not a separate part.
#
# Usage:
#   ./export.sh                  # closure=screw (default)  -> stl/
#   ./export.sh snap             # the snap-fit variant
#   ./export.sh screw my-stl     # custom output dir
#
# Optional overrides — any .scad parameter can be set with OpenSCAD's -D; these are the
# handy ones, passed as environment variables (the .scad default applies when unset):
#   LID_TEXT_SHOW=true   ./export.sh    # turn the lid text ON (off by default; final print)
#   LID_TEXT="My text"   ./export.sh    # change the lid text string
#   LID_TEXT_SIZE=4.0    ./export.sh    # cap height (mm)
#   SCREW_ANCHOR=selftap ./export.sh    # heatset (default) | selftap
#   PCB_T=1.0            ./export.sh    # PCB thickness (sets the base/lid split height)
# Combine freely, e.g.:  LID_TEXT_SHOW=true LID_TEXT="v2" ./export.sh
set -euo pipefail
cd "$(dirname "$0")"

SCAD="openrz67-case.scad"
CLOSURE="${1:-screw}"
OUTDIR="${2:-stl}"

# Locate openscad (PATH, else the macOS app)
if command -v openscad >/dev/null 2>&1; then
  OPENSCAD="openscad"
elif [ -x "/Applications/OpenSCAD.app/Contents/MacOS/OpenSCAD" ]; then
  OPENSCAD="/Applications/OpenSCAD.app/Contents/MacOS/OpenSCAD"
else
  echo "Could not find 'openscad'. Install it or put it on PATH." >&2
  exit 1
fi

# Build optional -D overrides from environment variables. Only the ones that are set get
# passed, so the .scad defaults apply otherwise. Strings are quoted; numbers/bools are bare.
DEFS=()
[ -n "${LID_TEXT+x}" ]      && DEFS+=(-D "lid_text=\"${LID_TEXT}\"")
[ -n "${LID_TEXT_SHOW:-}" ] && DEFS+=(-D "lid_text_show=${LID_TEXT_SHOW}")
[ -n "${LID_TEXT_SIZE:-}" ] && DEFS+=(-D "lid_text_size=${LID_TEXT_SIZE}")
[ -n "${SCREW_ANCHOR:-}" ]  && DEFS+=(-D "screw_anchor=\"${SCREW_ANCHOR}\"")
[ -n "${PCB_T:-}" ]         && DEFS+=(-D "pcb_t=${PCB_T}")

# Run openscad with the optional overrides first (safe under `set -u` if DEFS is empty).
run() { "$OPENSCAD" ${DEFS[@]+"${DEFS[@]}"} "$@" "$SCAD"; }

mkdir -p "$OUTDIR"

for part in base lid; do
  out="$OUTDIR/openrz67-${part}-${CLOSURE}.stl"
  echo "==> ${part} (${CLOSURE})  ->  ${out}"
  run -o "$out" -D "closure=\"${CLOSURE}\"" -D "part=\"${part}\""
done

# Light pipe - independent of closure, prints in clear filament
lp="$OUTDIR/openrz67-lightpipe.stl"
echo "==> lightpipe (clear filament)  ->  ${lp}"
run -o "$lp" -D "part=\"lightpipe\""

echo "Done. STLs are in ${OUTDIR}/ (lightpipe in clear filament, the rest opaque)"

# Rebuild the Bambu/Orca project .3mf by swapping these STLs into the hand-made template
# (bambu-template.3mf), keeping the print profile + plate layout + filament assignment (the
# light pipe stays on its clear filament). Only runs for the screw closure (the template's
# part names are *-screw) and when the template + python are present. Disable with MAKE_3MF=false.
MAKE_3MF="${MAKE_3MF:-true}"
if [ "$MAKE_3MF" = "true" ] && [ "$CLOSURE" = "screw" ] \
   && [ -f bambu-template.3mf ] && command -v python3 >/dev/null 2>&1; then
  echo "==> project 3mf  ->  openrz67-case.3mf"
  python3 make_3mf.py --stl-dir "$OUTDIR" --out openrz67-case.3mf
fi
