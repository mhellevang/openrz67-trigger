# openrz67 PCB — KiCad project

KiCad 10 project for the OpenRZ67 trigger board (48 × 22 mm, 2 layers, ESP32-C3,
two TLP172AM PhotoMOS relays, LGS5500 charger/boost). This is now the **source**; the EasyEDA Pro
project it was ported from is archived in `../archive/easyeda/` (v2 `.epro` and v3 `.epro2`);
the fabricated 2025-09-23 revision (Gerber, BOM, PnP, STEP) is in `../archive/2025-09-23-rev1/`.

| File | What |
|---|---|
| `openrz67.kicad_pro/.kicad_sch/.kicad_pcb` | project, schematic (1 sheet), board |
| `openrz67.kicad_sym`, `openrz67.pretty/` | symbols and footprints as imported from EasyEDA/LCSC (project-local libs) |
| `tools/regen.sh` | rebuilds everything in `out/` in one go; run it after any change to the board or schematic |
| `tools/post_import.py` | design rules, net classes, zone settings, layer names. **Authoritative**: it overwrites `openrz67.kicad_pro`, so rule changes made in Board Setup are reverted on the next run. Edit the values in the script. Rewrites the board in place, so close KiCad first. Idempotent. |
| `tools/fetch_3d.sh` | downloads STEP+WRL models for every LCSC part into `openrz67.3dshapes/` (uses `easyeda2kicad`, `uvx`, `pipx` or a local venv, whichever exists) |
| `tools/export_step.sh` | board STEP to `out/openrz67.step` (gitignored, ~20 MB); fetches missing part STEPs first |
| `openrz67.3dshapes/` | 3D models; `.wrl` committed (render), `.step` gitignored (fetch when you need a board STEP) |
| `out/` | generated: Gerber+drill zip, position file, BOM (with LCSC numbers), schematic PDF, top/bottom renders, ERC/DRC reports |

## Regenerate outputs

Everything in `out/` is generated. Rebuild all of it with one command, so the
committed outputs and the check reports always describe the committed sources:

```sh
tools/regen.sh
```

It runs `post_import.py`, exports Gerbers, drill files and their maps, zips them,
writes the position file, the BOM, the schematic PDF and the two board renders,
then runs DRC with schematic parity and ERC. It exits non-zero on errors;
warnings are printed and accepted (see below). Close KiCad first: `post_import.py`
rewrites the board and the project file in place.

Override the tool paths if KiCad is not in the macOS default location:

```sh
KICAD_CLI=/usr/bin/kicad-cli KICAD_PY=/usr/bin/python3 tools/regen.sh
```

## Design rules (from the EasyEDA project)

Clearance 0.127 mm (EasyEDA pour-to-track minimum; track-to-track was 0.152), min track
0.127, default track 0.16, via 0.45/0.20 (min 0.40/0.20), hole-to-track 0.175,
hole-to-hole 0.30, solder-mask expansion 0.051, thermal spoke 0.254 / gap 0.152.
Net classes: `gnd` (GND, 0.13 track), `3v` (VCC, 0.20), `5v` (+5V, 0.254 track, 0.5/0.3 via).
Copper-to-edge clearance is 0.30 mm (JLCPCB recommendation; the original pours ran to
the outline). Silkscreen minimums are JLCPCB's: 1.0 mm text height, 0.15 mm line width.

## Accepted warnings

ERC is clean: **0 errors, 0 warnings**. DRC has no errors; the warnings below are known,
understood and left in the reports rather than silenced, so a genuinely new one stands
out. No DRC or ERC exclusions are configured.

| Check | Count | What |
|---|---|---|
| `courtyards_overlap` | 5 | Neighbours closer than 0.1 mm on the fabricated rev-1 layout: C21/USB1, L3 against C20/R9/R12, H1/USB1. |
| `text_height`, `text_thickness` | 2 | The `BOOT EN` label on F.SilkS is 0.5 mm / 0.10 mm, under JLCPCB's 1.0 mm / 0.15 mm. Enlarged in place it runs over the R18 pads and the S3 body, where the fab clips silkscreen against solder mask. Splitting it into separate `BOOT` and `EN` labels does not help: the free band above the S1 pads is 0.85 mm and the one below the switches is 0.7 mm, both under the 1.0 mm the text needs. Fixing it means moving parts. Every other silkscreen text is at or above 1.067 mm. |

## Rev 2 layout changes (2026-09-03)

Outline unchanged from rev 1: **48 × 22 mm**, 2 mm corner radius, mounting holes at
(2, 20) and (46, 2). Edges are named here by their landmark, because the enclosure in
`../../case/` measures Y from the opposite side: the **USB-C end** is x = 0, the
**camera end** is x = 48, the **S3 edge** is y = 0, and the **switch edge** is y = 22.

- **U4** camera connector: side-entry S4B-XH-A (LCSC C157925) at (44.3, 11.0), opening
  out of the camera end.
- **BAT1** battery connector: SMD side-entry **S2B-PH-SM4-TB** (LCSC C295747) on the
  **bottom** side, centre at (9.6, 4.45), opening toward the board centre. The cell lies
  flat under the board and plugs in without bending the leads. Pin 1 = BAT+ as before.
  BAT+ reaches the bottom through a 0.6/0.3 via next to C22; GND is the bottom pour.
  JLCPCB two-sided assembly costs extra: BAT1 can also be left unplaced and hand-soldered.
  `BAT+` / `BAT-` are labelled on the bottom silkscreen at 1.067 mm.
- **Shutter outputs are PhotoMOS, not relays (2026-09-04).** K1/K2 (G6K-2F-Y), their DTC114E
  drivers Q1/Q2, flyback diodes D1/D2 and coil decoupling C27/C28 are gone. Each channel is one
  **TLP172AM** (Toshiba, LCSC C2152276, 4-pin SO6): the ESP32 GPIO drives the LED through a
  **220 Ω** 0402 (R21/R22, C25091), about 9 mA nominal. At 3.3 V, VOH = 0.8 × VDD,
  VF = 1.4 V and +1% resistance give 5.6 mA at 25 °C, within the datasheet's 5 to 25 mA
  recommendation. The MOSFET output closes S1/S2 to camera ground.
  TLP172AM replaced TLP172GM on 2026-09-05: maximum on-resistance is 2 Ω at 25 °C / IF = 5 mA,
  versus 50 Ω continuous for GM. AM's output rating is 60 V / 500 mA; package and pin functions
  are unchanged. The imported pad numbering 1/2/3/4 maps to Toshiba pins 1/3/4/6.
  Firmware activates S1, waits 10 ms, then activates S2. Camera input limits remain unverified;
  test the assembled prototype before ordering a production batch. See [research notes](notes/rz67-remote-inputs.md)
  and the [Toshiba datasheet](https://toshiba.semicon-storage.com/info/docget.jsp?did=36714&prodName=TLP172AM).
  U5 = S1 (net `S1_DRV`, **GPIO4**, U1 pin 9), U6 = S2 (net `S2_DRV`, GPIO3, U1 pin 8). Rev 1
  drove S1 from GPIO21, which is U0TXD: the ROM boot log leaves it high until `setup()` runs,
  so S1 was closed during every boot. Harmless alone, since the camera needs S1 and S2 together,
  but GPIO4 is a plain input at reset and GPIO21 is now free for serial logging. Isolation is kept: camera ground `AGND` still touches only U4 pin 2 and the two
  output pins, and the AGND pours (both layers) were pulled in from x = 32.1 to x = 34.6 so the
  LED side of each part sits in the GND domain. Eight parts became four, the tallest top-side part
  went from 5.2 mm (relay) to 2.2 mm, and the coil current (about 67 mA while an exposure is
  held) is gone. Footprint `SMD-4_L4.6-W3.7-P2.54-LS7.0-BR` and the 3D model came from
  `easyeda2kicad` for C261926; the unused relay/SOT-346/SOD-523 footprints, models and symbols
  were removed from the project libraries. R4/R5 (USB 22 Ω) are UNI-ROYAL 0402WGF220JTCE
  (C25092, JLCPCB Basic) since JLCPCB had 2 of the Yageo part in stock.
- **C5** moved to the analog supply: the 100 nF that sat on VCC next to the ferrite bead
  L2 now sits on the far side of it, on the new **VDDA** net feeding U1 pins 31/32 (radio and
  ADC supply). Before, nothing decoupled that net; the bead alone was just series impedance.
  VCC reaches L2 pin 1 through a new via beside the pad. Same part, same position, no BOM
  change. B.Cu ground under U1 stays one region (648.7 mm²).
- **Under-board clearance** the enclosure has to provide, from the datasheets:

  | What | Height below the board |
  |---|---|
  | BAT1 body, S2B-PH-SM4-TB (JST ePH p.4) | 5.5 mm, plus the PHR-2 plug |
  | S3 through-hole tails, B2B-PH-K-S | 3.4 mm unclipped |
  | U4 through-hole tails, S4B-XH-A | 3.4 mm unclipped |
  | USB1 shell posts | ~1 mm |

  The cell can only sit against the board if those tails are **clipped flush**. Budget
  3.4 mm otherwise. A foam pad between cell and board is still wanted, but 1.5 mm of it
  only fits after clipping.

**Removed again:** an earlier attempt widened the board to 25 mm for an unpopulated 1×3
expansion header (GPIO4 / 3V3 / GND) along the switch edge. It was reverted. Reaching the
new edge strip forced the GPIO4 return path across the bottom pour, which split the B.Cu
ground under U1 and LDO1 into two islands and displaced three stitching vias next to the
antenna trace. One GPIO, on a header that was not going to be populated, did not pay for
that. There is no room on this outline for a through-hole header: a hole needs both layers
clear at once, and only two isolated spots on the whole board qualify. Bottom-side solder
pads do fit, but the cell now occupies the bottom. Revisit it together with the enclosure.

## Ordering

`out/gerber/` is tracked unpacked, so a revision's copper is diffable in git.
`tools/regen.sh` also writes `out/openrz67-gerber.zip`, which is what you upload;
it is gitignored because it is one command away and would otherwise churn the
history as a binary blob on every regen. When a revision is actually ordered, copy
the zip, BOM and position file into `../archive/<date>-rev<n>/` as the record of
what was fabricated.

JLCPCB's uploader asks you to map columns; the exports do not use its header names:

| JLCPCB field | Column in `out/openrz67-bom.csv` |
|---|---|
| Comment | `Value` |
| Designator | `Designator` |
| Footprint | `Footprint` |
| LCSC Part # | `LCSC` |

The position file is rewritten by `regen.sh` into JLCPCB's CPL layout (`Designator, Mid X,
Mid Y, Layer, Rotation`, mm suffix); their parser rejects KiCad's native header with
"Failed processing the CPL file". Origin is the drill-file origin at the board's top-left
corner, the same convention JLCPCB accepted for rev 1, and rotations for the top-side
parts are unchanged since that order. **BAT1 is the one part on the bottom side**: check its orientation in
the fab's assembly preview before confirming, since bottom-side rotation conventions
differ between tools.

Two-sided assembly costs extra. BAT1 can be left unplaced and hand-soldered instead.

## Port notes (2026-09-03)

- Import path: EasyEDA Pro **v2** `.epro` → KiCad "Import Non-KiCad Project". The v3
  `.epro2` export loads as an empty board in KiCad 10.0.6.
- Removed the "JeefunPCB" A3 sheet-frame symbol that came with the EasyEDA template
  and replaced it with a KiCad title block. Annotated the 62 power flags (`#PWR001…`),
  deleted two floating GND flags, added a no-connect on U4 pin 1.
- EasyEDA local labels became global labels so net names match the board (`BAT+`, `D7`, and the
  Wemos-style `D1`/`D6`, renamed `S2_DRV`/`S1_DRV` on 2026-09-04); EasyEDA auto-nets (`$1N…`) were renamed to KiCad's `Net-(…)` names.
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
  per LCSC number with `easyeda2kicad` (`tools/fetch_3d.sh`) and both the board footprints and
  the library `.kicad_mod` files repointed to `openrz67.3dshapes/<name>.wrl`, keeping the
  importer's offsets/rotations, so "Update Footprints from Library" does not undo it. Board STEP:
  `kicad-cli pcb export step --subst-models -o out/openrz67.step openrz67.kicad_pcb`
  (needs the `.step` files, run `tools/fetch_3d.sh` first).
- Bottom silkscreen label changed from "EPS32-C3 Camera Trigger V1.0 / 2025-08-23" to
  "OpenRZ67 Trigger v2 / 2026-09".
- Pin electrical types were set by hand for the ICs (ESP32-C3, LGS5500, ME6211, USB-C);
  passives, connectors and the PhotoMOS pins are `passive`. Supply nets without a driver carry `PWR_FLAG`
  (GND, AGND, BAT+, VBUS, +5V_VIN, VDDA). ERC runs at default severities: 0 errors, 0
  warnings. The eight dangling wire ends inherited from the EasyEDA drawing were removed
  (an orphan S1/S2/AGND label cluster and three over-long wire tails); the netlist is
  unchanged, verified node-for-node before and after.
- The two mounting holes are real footprints (`MountingHole_2.0mm_Pad3.0`, plated 3.0 mm pad,
  2.0 mm drill) with `MountingHole` symbols H1/H2 in the schematic (excluded from BOM/POS).
- Courtyards were regenerated as pad/body bounding boxes (+0.05 mm) — the LCSC ones were
  oversized. Silkscreen outlines of 0402/0603 parts and of the two overhanging connectors
  (USB-C, U4) live on F.Fab now. DRC: 0 errors, 6 courtyard warnings where neighbours are
  closer than 0.1 mm on the fabricated layout (C28/S3, C21/USB1, L3 vs C20/R9/R12, H1/USB1).
- The library symbol file was pruned to the symbols in use; the schematic's embedded copies are
  regenerated from it, so "symbol differs from library" warnings are gone.
