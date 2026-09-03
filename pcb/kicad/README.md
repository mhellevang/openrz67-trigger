# openrz67 PCB — KiCad project

KiCad 10 project for the OpenRZ67 trigger board (48 × 25 mm, 2 layers, ESP32-C3,
two G6K relays, LGS5500 charger/boost). This is now the **source**; the EasyEDA Pro
project it was ported from is archived in `../archive/easyeda/` (v2 `.epro` and v3 `.epro2`);
the fabricated 2025-09-23 revision (Gerber, BOM, PnP, STEP) is in `../archive/2025-09-23-rev1/`.

| File | What |
|---|---|
| `openrz67.kicad_pro/.kicad_sch/.kicad_pcb` | project, schematic (1 sheet), board |
| `openrz67.kicad_sym`, `openrz67.pretty/` | symbols and footprints as imported from EasyEDA/LCSC (project-local libs) |
| `tools/post_import.py` | design rules, net classes, zone settings, layer names; idempotent, run with KiCad's bundled python |
| `tools/fetch_3d.sh` | downloads STEP+WRL models for every LCSC part into `openrz67.3dshapes/` (uses `easyeda2kicad`, `uvx`, `pipx` or a local venv, whichever exists) |
| `tools/export_step.sh` | board STEP to `out/openrz67.step` (gitignored, ~20 MB); fetches missing part STEPs first |
| `openrz67.3dshapes/` | 3D models; `.wrl` committed (render), `.step` gitignored (fetch when you need a board STEP) |
| `openrz67.kicad_dru` | custom rule: USB-C pads exempt from copper-to-edge clearance |
| `out/` | generated: Gerber+drill zip, position file, BOM (with LCSC numbers), schematic PDF, top/bottom renders, ERC/DRC reports |

## Regenerate outputs

```sh
K=/Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli
PY=/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3
$PY tools/post_import.py openrz67.kicad_pcb
$K pcb export gerbers --layers F.Cu,B.Cu,F.Mask,B.Mask,F.Paste,B.Paste,F.SilkS,B.SilkS,Edge.Cuts --subtract-soldermask -o out/gerber/ openrz67.kicad_pcb
$K pcb export drill --format excellon --excellon-units mm --excellon-separate-th --generate-map --map-format gerberx2 -o out/gerber/ openrz67.kicad_pcb
$K pcb export pos --format csv --units mm --side both --exclude-dnp --use-drill-file-origin -o out/openrz67-pos.csv openrz67.kicad_pcb
$K sch export bom --fields 'Reference,${QUANTITY},Value,Footprint,Manufacturer,Manufacturer Part,Supplier Part' \
   --labels 'Designator,Qty,Value,Footprint,Manufacturer,MPN,LCSC' --group-by 'Value,Footprint,Supplier Part' --exclude-dnp \
   -o out/openrz67-bom.csv openrz67.kicad_sch
$K sch export pdf -o out/openrz67-schematic.pdf openrz67.kicad_sch
$K pcb drc --schematic-parity --severity-all -o out/drc.rpt openrz67.kicad_pcb
```

## Design rules (from the EasyEDA project)

Clearance 0.127 mm (EasyEDA pour-to-track minimum; track-to-track was 0.152), min track
0.127, default track 0.16, via 0.45/0.20 (min 0.40/0.20), hole-to-track 0.175,
hole-to-hole 0.30, solder-mask expansion 0.051, thermal spoke 0.254 / gap 0.152.
Net classes: `gnd` (GND, 0.13 track), `3v` (VCC, 0.20), `5v` (+5V, 0.254 track, 0.5/0.3 via).
Copper-to-edge clearance is 0.30 mm (JLCPCB recommendation; the original pours ran to the
outline). `openrz67.kicad_dru` exempts the USB-C pads, which sit on the edge by design.

## Rev 2 layout changes (2026-09-03)

- **U4** camera connector: side-entry S4B-XH-A, opening out of the right board edge.
- **BAT1** battery connector: SMD side-entry **S2B-PH-SM4-TB** (LCSC C295747) on the **bottom**
  side at the left end (centre 9.7 mm from the left edge, 4.4 mm from the top edge), opening
  toward the board centre. The battery lies flat under the board and plugs in without bending
  the leads. Under-board height needed: 6.0 mm for the connector; keep a ~1.5 mm foam pad between
  the cell and the PCB (through-hole tails of USB1/U4 protrude ~1 mm). Pin 1 = BAT+ as before.
  BAT+ reaches the bottom through a 0.6/0.3 via next to C22; GND is the bottom pour.
  JLCPCB two-sided assembly costs extra: BAT1 can also be left unplaced and hand-soldered.
- **Board widened 22 → 25 mm** (new strip along the back edge, y 22–25 mm from the front edge).
- **J1 expansion header** in that strip: 1×3 × 2.54 mm through-hole pads, **not populated**
  (excluded from BOM/POS). Pin 1 = **GPIO4** (ADC1_CH4, I2C/UART capable), pin 2 = **3V3**,
  pin 3 = **GND**, each labelled in silkscreen on both sides so wires can be soldered straight in.
  Pin 1 is 33 mm from the left edge, 1.5 mm from the back edge. Only one GPIO
  is routed: the free ESP32-C3 pins (GPIO4–7, GPIO10) all sit on the QFN's right column and
  share a single escape between existing traces; a second signal would need the VCC/D1/CHIP_EN
  tracks around U1 moved. GPIO4 runs F.Cu stub → via → B.Cu around the west end of the VCC
  track → along the new strip. A handful of GND/AGND stitching vias in its way were removed.
- **S3** stays through-hole (B2B-PH-K-S): the side-entry SMD variant collides with C14/C28 and
  the vertical SMD one with D4/C28 unless parts are moved.

## Port notes (2026-09-03)

- Import path: EasyEDA Pro **v2** `.epro` → KiCad "Import Non-KiCad Project". The v3
  `.epro2` export loads as an empty board in KiCad 10.0.6.
- Removed the "JeefunPCB" A3 sheet-frame symbol that came with the EasyEDA template
  and replaced it with a KiCad title block. Annotated the 62 power flags (`#PWR001…`),
  deleted two floating GND flags, added a no-connect on U4 pin 1.
- EasyEDA local labels became global labels so net names match the board (`BAT+`, `D1`,
  `D6`, `D7`); EasyEDA auto-nets (`$1N…`) were renamed to KiCad's `Net-(…)` names.
  Connectivity was verified pad-for-pad against the schematic netlist before renaming.
- USB-C shell pads 13/14 are GND in the schematic; the EasyEDA board had them net-less.
  They are GND now (they sit in the GND pour).
- Schematic footprint fields for L2 (L0603→L0402), U1 (TL→BL QFN variant) and
  R18 (R0603→R0402) were corrected to what is actually on the board.
- DRC: 0 errors, 0 unconnected, schematic parity clean. Remaining warnings are silk
  overlaps/courtyards from the LCSC footprints. Two dangling vias (+5V_VIN, a duplicated
  VCC via) and a 0.1 mm track stub from the original layout were removed. Two small USB1
  polygons that the importer put on Edge.Cuts (not in the EasyEDA outline) moved to F.Fab.
- Gerber check against the fabricated 2025-09-23 set: copper and mask match apart from
  U4 (now S4B-XH-A side-entry instead of B4B-XH-A) and pour-fill details after KiCad's
  refill (thermal shapes); silkscreen differs in font rendering only.
- 3D models: the importer left dangling `EASYEDA_MODELS/…` references. Models were fetched
  per LCSC number with `easyeda2kicad` (`tools/fetch_3d.sh`) and the footprints repointed to
  `openrz67.3dshapes/<name>.wrl`, keeping the importer's offsets/rotations. Board STEP:
  `kicad-cli pcb export step --subst-models -o out/openrz67.step openrz67.kicad_pcb`
  (needs the `.step` files, run `tools/fetch_3d.sh` first).
- Bottom silkscreen label changed from "EPS32-C3 Camera Trigger V1.0 / 2025-08-23" to
  "OpenRZ67 Trigger v2 / 2026-09".
- Pin electrical types were set by hand for the ICs (ESP32-C3, LGS5500, ME6211, USB-C);
  passives/connectors/relays are `passive`. Supply nets without a driver carry `PWR_FLAG`
  (GND, AGND, BAT+, VBUS, +5V_VIN, VDDA). ERC runs at default severities: 0 errors, 8 warnings
  (short dangling wire ends inherited from the EasyEDA drawing; harmless, left as-is).
- The two mounting holes are real footprints (`MountingHole_2.0mm_Pad3.0`, plated 3.0 mm pad,
  2.0 mm drill) with `MountingHole` symbols H1/H2 in the schematic (excluded from BOM/POS).
- Courtyards were regenerated as pad/body bounding boxes (+0.05 mm) — the LCSC ones were
  oversized. Silkscreen outlines of 0402/0603 parts and of the two overhanging connectors
  (USB-C, U4) live on F.Fab now. DRC: 0 errors, 6 courtyard warnings where neighbours are
  closer than 0.1 mm on the fabricated layout (C28/S3, C21/USB1, L3 vs C20/R9/R12, H1/USB1).
- The library symbol file was pruned to the symbols in use; the schematic's embedded copies are
  regenerated from it, so "symbol differs from library" warnings are gone.
