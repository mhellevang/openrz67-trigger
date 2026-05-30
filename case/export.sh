#!/usr/bin/env bash
# Export base + lid + light pipe + lid text to STL in one go.
# The light pipe (openrz67-lightpipe.stl) prints in CLEAR/transparent filament; base and
# lid in normal opaque filament; the lid text (openrz67-lidtext.stl) in a 2nd colour.
#
# Usage:
#   ./export.sh                  # closure=screw (default)  -> stl/
#   ./export.sh snap             # the snap-fit variant
#   ./export.sh screw my-stl     # custom output dir
#
# Optional overrides — any .scad parameter can be set with OpenSCAD's -D; these are the
# handy ones, passed as environment variables (the .scad default applies when unset):
#   LID_TEXT="My text"   ./export.sh    # change the lid text string
#   LID_TEXT=""          ./export.sh    # empty string turns the text off ...
#   LID_TEXT_SHOW=false  ./export.sh    # ... or toggle it off explicitly
#   LID_TEXT_SIZE=4.0    ./export.sh    # cap height (mm)
#   SCREW_ANCHOR=selftap ./export.sh    # heatset (default) | selftap
#   PCB_T=1.0            ./export.sh    # PCB thickness (sets the base/lid split height)
# Combine freely, e.g.:  SCREW_ANCHOR=selftap LID_TEXT="v2" ./export.sh
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

# Lid text - text only, for 2-colour printing (load as a separate part/colour in the slicer).
# Skipped gracefully if the text is disabled (empty geometry) so turning it off via
# LID_TEXT_SHOW=false / LID_TEXT="" doesn't fail the whole export.
lt="$OUTDIR/openrz67-lidtext.stl"
if run -o "$lt" -D "part=\"lidtext\""; then
  echo "==> lidtext (2nd filament colour)  ->  ${lt}"
else
  rm -f "$lt"
  echo "==> lidtext: text disabled - no STL written"
fi

echo "Done. STLs are in ${OUTDIR}/ (lightpipe in clear filament, lidtext in a 2nd colour, the rest opaque)"
