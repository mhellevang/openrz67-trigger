# openrz67 PCB — KiCad project

KiCad 10 project for the OpenRZ67 trigger board (48 × 22 mm, 2 layers, ESP32-C3,
two G6K relays, LGS5500 charger/boost). This is now the **source**; the EasyEDA Pro
project it was ported from is archived as `../ProPrj_*.epro` (v2) and `.epro2` (v3).

| File | What |
|---|---|
| `openrz67.kicad_pro/.kicad_sch/.kicad_pcb` | project, schematic (1 sheet), board |
| `openrz67.kicad_sym`, `openrz67.pretty/` | symbols and footprints as imported from EasyEDA/LCSC (project-local libs) |
| `tools/post_import.py` | design rules, net classes, zone settings, layer names; idempotent, run with KiCad's bundled python |
| `out/` | generated: Gerber+drill zip, position file, BOM (with LCSC numbers), schematic PDF, top render |

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
Copper-to-edge clearance is 0 because the original pours run to the outline;
JLCPCB recommends 0.3 mm — tighten in a future revision and refill.

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
  overlaps/courtyards from the LCSC footprints, two dangling vias (+5V_VIN, VCC) and a
  0.1 mm track stub inherited from the original layout.
- Gerber check against the fabricated 2025-09-23 set: copper and mask match apart from
  U4 (now S4B-XH-A side-entry instead of B4B-XH-A) and pour-fill details after KiCad's
  refill (thermal shapes); silkscreen differs in font rendering only.
- 3D models were not carried over (EasyEDA fetches them online); STEP export is therefore
  bare. `../3D_*.step` is the last full model.
- ERC severities for `pin_not_driven`/`power_pin_not_driven` are set to warning: pin
  electrical types come from LCSC symbols and are not reliable.
