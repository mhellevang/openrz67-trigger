# Parametric enclosure (build123d)

`openrz67_case.py` is a parametric two-part enclosure for the OpenRZ67 trigger PCB
**rev 2** (`../pcb/kicad/`), written in [build123d](https://build123d.readthedocs.io/)
(Python). It is built from the board file and its 3D models, not from eyeballed
measurements.

Rev 2 moved the battery connector `BAT1` to the **underside** of the board (SMD side-entry,
mouth toward the board centre) and made the camera connector `U4` side-entry. The case is
therefore a **stacked** box: the 31 × 20 × 6 mm cell lies flat on the floor **under the
PCB, across the board**, and plugs into BAT1 from below. The previous revision (battery in a
bay beside the board, for the fabricated 2025-09-23 rev-1 board) is in git history up to
commit `8fe7b2d`.

> **Not print-tested yet.** The geometry passes the built-in probe assertions, but this
> revision has not been printed. Print the `SNAP_TEST` corner first as before, then the
> base alone to check the cell, plug and lead against the real parts.

## Source data

| Measurement | Value | Source |
|-----|-------|-------|
| Board outline | 48.0 × 22.0 mm, R2.0 | `../pcb/kicad/openrz67.kicad_pcb` (Edge.Cuts) |
| Mounting holes | 2× Ø2.0 at (2,2) and (46,20) | `H1`, `H2` |
| PCB thickness | 1.6 mm | `pcb_t` (as ordered) |
| Component placement | see openrz67_case.py | footprint positions in the board file |
| `BAT1` S2B-PH-SM4-TB (bottom) | body board-X 5.4–14.0, Y 13.6–21.5, 5.5 mm below the board, mouth toward +X | footprint + LCSC STEP (`../pcb/kicad/openrz67.3dshapes/`) |
| `U4` S4B-XH-A (top, side-entry) | body 12.4 wide × 6.1 tall, protrudes 2.4 mm past the board's right edge, mouth facing out | LCSC STEP |
| THT tails below the board | `S3` (27.7/29.7, 19.96), `U4` (44.3, 7.25–14.75): 3.4 mm unclipped | JST datasheets |
| USB-C connector | body 8.95 × 3.2 mm on the PCB, left edge 4.8 mm in from the PCB's left edge; the shell protrudes ~2 mm past the board edge | measured physically |
| Total HW length | ~50 mm (PCB 48 + USB-C shell 2) | `pcb_overhang_left` |
| Slide switch (SS12F15) | actuator opening 10.65×6.3; flat bracket 19.45×5.75×0.4; screw holes Ø2.2 at 15.0 mm spacing; body ~19.5×20×12.9 | `SS12F15.stp` + measured |
| LEDs | D3 red (22.5, 20.2), D4 blue (24.2, 20.2) – 0603 SMD, top-emitting, 1.7 mm apart | board file |
| Battery cell | 31 × 20 × 6 mm LiPo, 250 mAh | measured |

Board coordinates in this file and in the script are the **enclosure's**: origin at the
board's front-left corner, Y toward the back (the switch wall). The KiCad board measures Y
from the other long edge, so `board_y_here = 22 - kicad_y`; the mounting holes make the
mapping easy to check.

Component **heights** are entered as editable constants (`h_relay`, `h_usbc`, `h_ph_plug`,
`h_xh`, `h_bat1`) from the datasheets and STEP models. Check them against your own parts.

The script carries **module-level assertions**: outer dimensions, plus probe checks that
every opening actually breaks through (USB, XH connector opening, LED window, locating-pin
recesses, switch screws) and that the fit-critical keepouts stay open: the switch body
envelope, the USB-C body, and under the board the **cell pocket, the BAT1 + plug envelope,
the six THT tails and the battery lead's route** into the back pocket. They run on every
export, so a parameter tweak that closes a hole or re-introduces a known collision fails
loudly instead of surfacing in the print.

## Construction
- **Base** (tub): floor, the **battery pocket** on the floor under the board, standoffs
  that raise the PCB `standoff_h` = cell 6.0 + `under_clr` 3.7 = **9.7 mm** (the cell plus
  air for the unclipped S3/U4 solder tails), **solid support pillars** in all four corners
  (`pcb_supports = [[2,20],[46,2]]` plus the two posts at the mounting holes), and two
  **Ø1.7 locating pins** up into the real Ø2 mounting holes.
  - **Cavity wider than the board.** The cell is 31 mm long and lies **across** the 22 mm
    board, so the cavity is `side_room` = 5 mm wider than the board on **both** long sides.
    The board sits centred; the two 5 mm **side pockets** hold the battery lead's slack and
    give fingers somewhere to go.
  - **Cell pocket:** the cell lies on the floor at board-X `batt_x0` 20 → 40, spanning the
    whole cavity in Y (the walls hold it, `batt_clr` 0.5 each side: pocket 32 × 21, the same
    as the old bay). Two low ribs (`rib_h` 2.5, `rib_t` 1.6) stop it sliding in X. The
    **left rib covers the front half only**, so the lead can loop from the BAT1 plug out
    into the back pocket. `batt_x0` keeps the cell clear of BAT1's mouth (board-X 14.0), the
    PHR-2 plug and the wire exit (`bat1_env`, to X 19).
  - **Under-board clearance** (`under_clr`): 3.7 mm by default, for the 3.4 mm solder tails
    of S3 and U4 that hang over the cell. If you **clip the tails flush**, set it to 1.5 and
    put a 1.5 mm foam pad between cell and board; the box gets 2.2 mm lower. BAT1's 5.5 mm
    body plus plug fits under the 9.7 mm standoff either way (asserted).
  - **Through-hole solder relief** (`th_keepouts`): pockets in a post top where a THT tail
    would land on it. Rev 2 has none; the list is kept for other boards.
  - **PCB guide fins** (`frame_*`): blocks from the inner wall face to `frame_clr` (0.2 mm)
    off the board's long edges, floor to `frame_proud` (0.6 mm) above the board top with a
    chamfered lead-in, at board-X `frame_ribs` = `[10, 45]` on **both** long edges (outside
    the cell's X span). The board drops straight down between them onto the pins. Because
    the fins start at the inner wall face they never meet the lid tongue, so the tongue
    needs no notches.
- **Lid**: telescopes down into the base via a perimeter tongue-and-groove edge (lap
  joint) for alignment. Carries the USB-C opening (left), the opening the XH camera connector
  sits in (right), the LED light-pipe
  window (top), and the slot + screw pillars for the SS12F15 slide switch.
  - **XH connector opening** (`cut_xh`): U4 is now a **side-entry** S4B-XH-A whose 12.4 ×
    6.1 mm body protrudes 2.4 mm past the board edge, i.e. 2.0 mm **into the right wall**.
    The lid wall gets a rectangular opening (`xh_w/h` + `xh_clr` 0.4) from the board top up
    that the connector body sits in, its mouth 0.4 mm inside the outer face. The XHP-4 plug
    goes in from **outside**, through the wall, so the cable is no longer inside the case and
    the lid lifts off untethered without a slit. Lid-only cut (all of it is above the split).
  - **LED light pipe** (`led_*`, separate part): D3 (red) and D4 (blue) are top-emitting
    SMD LEDs ~8 mm below the lid. A separate **clear light pipe** is inserted from above
    as a top hat: a wide head in a top counterbore (flush with the top face) + a rod that
    goes down to ~1.5 mm above the LEDs and channels the light into two sharp dots. The
    lid is printed opaque; **only the light pipe is printed in clear filament**. It rests
    in the counterbore (gravity + optionally a drop of glue to seal). Parameters:
    `led_win_l/w/r` (window size), `led_head_lip/t` (head/counterbore), `led_pipe_gap`
    (air gap to the LED), `led_pipe_clr` (fit in the hole). `led_pos` are the LED
    positions from pick&place.
  - **Component clearance** (`comp_keepouts`): rectangles `[board_x0, board_y0, board_x1,
    board_y1]` cut out of the lid's hold-down bosses (full height above the PCB) where a
    boss would collide with a component body — e.g. the Ø6 boss at hole (2,2) is cut back
    to a D-shape to clear the USB-C connector's near edge (~board-Y 4.8).
- **USB-C asymmetry**: the USB-C shell protrudes ~2 mm past the PCB's left short side.
  `pcb_overhang_left = 2.0` extends the cavity on the left; the PCB is therefore centred
  toward the RIGHT in the cavity (normal `clr = 0.4` against the right wall, `clr +
  overhang = 2.4 mm` against the left). The outer case width thus becomes ~**55.6 mm**.
- **The battery** (250 mAh LiPo, 31 × 20 × 6) lies **under the PCB, across the board**, see
  the base section above. BAT1 is on the board's underside at board ≈ (5.4–14, 13.6–21.5)
  with its mouth toward the board centre, so the plug goes in horizontally from the right
  while the board is in your hand; the lead then loops into the **back pocket** and along
  it to the cell's tab end. Assembly order: cell onto the floor between the ribs (tabs
  toward the back), plug the lead into BAT1, lower the board onto the pins with the lead
  draping into the back pocket, lid on. The pocket is exactly the cell's height plus
  `under_clr`; fix the cell with a foam pad or a dab of glue as before. Parameters:
  `batt_l/w/t` (cell), `batt_clr` (fit), `batt_x0` (position in X), `under_clr`, `rib_h/t`.

## Closure — snap-fit
**No hardware at all.** A perimeter **bead** (`snap_bead` 0.5 mm, `bead_h` 1.4, chamfered
0.4 top and bottom — click-in/pry-out ramps, and the first printed bead layer isn't a ledge
in mid-air) on the lid tongue clicks into a matching **groove** in the base lip (0.5 taller
/ 0.25 deeper for lead-in, ceiling chamfered so it prints without an overhang), holding the
whole rim down. The PCB is located by two **Ø1.7 pins** in its real Ø2
mounting holes and pressed onto the posts by the lid's hold-down bosses (blind pin recesses
inside — no holes through the top). Open with a coin or fingernail in the **pry slot**
(`pry_w/d/h`) on the back wall's lower lid edge, over the back pocket.

The case is rarely opened (charging is external via USB-C), so snap wear is not a concern
with PLA; print the lid in PETG if you want extra flex margin. **Tune before the full
print:** `SNAP_TEST=true ./export.sh` also exports a cropped front-left corner pair
(`openrz67-snaptest-*`) — print those and adjust `snap_bead` (click strength) and
`lap_gap` (sliding fit) until the corner snaps shut and pries open with reasonable force.

Finished size with default values: ~**55.6 × 37.6 × 24.3 mm** (X incl. the 2 mm USB-C
overhang, Y incl. the two 5 mm side pockets and the 0.8 mm orientation rib, Z = floor 2 +
cell 6 + `under_clr` 3.7 + PCB 1.6 + 9 mm component clearance + 2 mm top).

## Orientation mark — `orient_mark`
A small **raised rib** on the front wall (low Y) near the left corner, split across the
seam: the base carries the lower half, the lid the upper half. When the lid is on the
right way around the two halves line up into **one continuous vertical rib**; a lid put on
180° wrong moves its half to the opposite corner, so the mismatch is obvious at a glance.
It is raised (not a recessed groove) on purpose — a groove here would thin the 1.2 mm lap
wall. Parameters: `orient_mark_x` (board-X of the mark, near the USB side), `orient_mark_w`
(width), `orient_mark_d` (how far it sticks out), `orient_mark_h` (total height across the
seam). Delete the two `orient_mark_rib` lines to remove it.

## Lid text — second colour (`lid_texts`)
**Debossed badge layout** in front of the LED window (the LEDs sit on the board's back
edge, close to the back wall): **"OpenRZ67"** (9 mm caps) nearest the window, **"Trigger"**
(7 mm) toward the front, both centred on the light pipe's X, cut `lid_text_depth`
(0.6 mm) into the top face. Per the FDM rules: pockets, not raised letters, and well above
the 4.4 mm legibility minimum for a 0.4 nozzle. Placement is enforced by assertions (each
line must clear the LED recess, the walls and the face edges — an over-long string fails
the export instead of the print).

Colour it in the slicer with a **height range modifier on the lid**: select the lid,
add a height range **0.6–1.4 mm** and assign it the letter-colour filament. The lid prints
**upside down** (the pockets face the bed and come out crisp), so those four layers form
the pocket floors — the letters show in the contrast colour, recessed in the face colour,
with only a thin accent band at the lid's top edge. On a single-filament printer, do a
manual filament change at Z = 0.6 instead (recolours everything above 0.6, so slice the
lid alone).

**Do not save the height range into the project/template**: QIDI Studio (≤ v2.x)
**segfaults on opening** a .3mf that carries `Metadata/layer_config_ranges.xml`
(`ObjectList::add_settings_item` dereferences the not-yet-created model tab during
startup load). Add the range fresh after opening the project instead.

Edit the `lid_texts` constant to change strings/sizes/offsets; `LID_TEXT_SHOW=false`
disables the text (on by default).


## Usage
The parameters are plain Python constants at the top of `openrz67_case.py`. Requires
[uv](https://docs.astral.sh/uv/) — the script carries its own dependency header, so there
is no venv to manage.

**Easiest export** – run the script, which builds all three parts and the slicer project:

```bash
cd case
./export.sh            # base, lid, lightpipe -> stl/  + openrz67-case.3mf
./export.sh out        # custom output dir (skips the .3mf)
```

**Overrides** – `openrz67_case.py` reads these from the environment (the in-file default
applies when unset); anything else is a one-line edit of the constant:

```bash
LID_TEXT_SHOW=true   ./export.sh   # turn the lid text ON (off by default; for the final print)
LID_TEXT="My text"   ./export.sh   # change the lid text string
LID_TEXT_SIZE=4.0    ./export.sh   # cap height (mm)
SNAP_TEST=true       ./export.sh   # also export the corner test pair for snap tuning
PCB_T=1.0            ./export.sh   # PCB thickness (sets the base/lid split height)
LID_TEXT_SHOW=true LID_TEXT="v2" ./export.sh     # combine freely
```

Or run the script directly (same thing minus the .3mf step): `uv run openrz67_case.py`.
For a plain interpreter (IDE run button etc.) there is a local venv:
`.venv/bin/python openrz67_case.py` — recreate it anytime with
`uv venv .venv && uv pip install -p .venv "build123d>=0.11.1"` (it is git-ignored).
For an interactive 3D preview, open the file with an OCP viewer (e.g. `ocp_vscode`) or
just inspect the exported STLs in the slicer. STL output in `case/stl/` is git-ignored.

### Slicer project (.3mf)

After the STLs, `export.sh` rebuilds `openrz67-case.3mf` — a
**Bambu Studio / OrcaSlicer project** with the three parts arranged on the plate, the print
profile, and the **per-part filament assignment** (base + lid on filament 1, light pipe on
filament 4 = clear). It does this by swapping the fresh meshes into a hand-made template
(`bambu-template.3mf`) via `make_3mf.py`, so all slicer settings are preserved — only the
geometry changes. Skip it with `MAKE_3MF=false ./export.sh`, or run it standalone:

```bash
python3 make_3mf.py            # template + stl/ -> openrz67-case.3mf
```

To change the print settings, plate layout, or filament mapping, edit the project once in the
slicer and re-save it over `bambu-template.3mf`; subsequent exports inherit the new setup.
The plate thumbnails inside the .3mf are not regenerated (they show the template's geometry);
the slicer refreshes them when you open or slice the project — purely cosmetic.

## Print orientation
- **Base**: print as-is (floor down). No supports needed.
- **Lid**: print **upside down** (top plate against the bed). That way the bosses/lip
  point upward with no overhang and the LED window/counterbore gets a clean top surface.
- **Light pipe** (`openrz67-lightpipe.stl`): **clear filament**. Print upright (head down
  against the bed) for the fewest layer lines across the light path, or lying down for
  smoother walls – both work for an indicator. For the clearest light: print at a fine
  layer height and consider sanding/dipping the top face.

## Worth adjusting before printing
- `pcb_t` – PCB thickness (default **1.6 mm**, as ordered). Adjust only if your board
  differs; it sets the height of the base/lid split.
- `pcb_overhang_left` – the air gap on the left for the USB-C shell. Measure your own
  connector: distance from the PCB's left edge to the outermost point of the shell.
  Default 2.0.
- `pcb_supports` – extra support-pillar coordinates (without screw) on the base. Default
  is the two empty corners `[[2,20],[46,2]]`; keep new ones outside the cell's X span.
- `under_clr` – 3.7 with the S3/U4 tails as soldered, 1.5 if you clip them flush (then
  add a 1.5 mm foam pad). Sets the box height together with `batt_t`.
- `h_ph_plug` – the PHR-2 plug + wire standing in the top-entry S3 header is the tallest
  thing above the board and sets `comp_clr`. Lower it if your plug/wire is shorter.
- `xh_w/h/clr` – the opening the S4B-XH-A body sits in. Check that your XHP-4 plug passes
  through it and that the latch can be pressed from outside.
- `snap_bead` / `lap_gap` – snap click strength and sliding fit. Tune with the
  `SNAP_TEST=true` corner pieces before printing the whole box.
- `batt_l/w/t` / `batt_clr` – fit your LiPo; `batt_x0` moves it in X (asserted clear of
  BAT1's plug and of the right-hand posts).
- `sw_x` / `sw_z` / `sw_wall` – position of the slide switch. The **front wall**
  (`"front"`, low Y) is chosen deliberately: the back wall is blocked by the S3 connector
  (board-X 28.7) and the K2 relay (board-X 31–42), so there's no room there for a centred
  switch with 15 mm boss spacing. `sw_x = 22` is practically centred (case centre is 23;
  shifted 1 mm left so the right boss clears the K1 relay at board-X 33.16 by ~1.2 mm).
  The wire from S3 runs across the board to the switch. Verify against your own components.
- **Switch screw mount**: the switch's flat bracket mounts on the **outside** of the case;
  screws go from outside → through the bracket and wall → thread into two **rectangular
  pillars** that hang from the lid ceiling down to `sw_boss_gap` (1.5 mm) above the PCB. The
  pillars are anchored in the top plate and their front sits at the inner wall plane against
  the (now solid) inner wall, so tightening clamps **bracket → wall → boss** in compression —
  no free-standing cantilever. The `sw_boss_gap` air gap keeps the pillars clear of the
  densely packed front components under their footprint (Q1 SOT-23 ~1.1 mm, C15/C26 0603
  ~0.9 mm). `sw_screw_pitch` = 15.0 (centre spacing, ±7.5), `sw_screw_d` = 2.4
  (M2 clearance), `sw_boss_d/h/pilot` set the pillar cross-section and pilot hole.
  - **Body keepout** (`sw_body_l × sw_body_h × sw_body_w` = **11 × 7 × 8 mm**, + `sw_body_clr`
    0.4/side): the switch body that protrudes inside is wider than the gap between the
    bosses, so a keepout carves the body envelope out of the cavity, relieving the boss
    inner faces over the body's Y/Z extent only — the screw region (±7.5) and the pillar
    above the body stay full. Verify `sw_body_l/h/w` against your switch.
  - **Actuator opening** (`sw_slot_l × sw_slot_h`): **10.65 × 6.3 mm** through the wall,
    centred on `sw_x`/`sw_z`.
  - **Screw**: **M2 self-tapping**, ~**8 mm** long (M2×6 also fine). From outside through the
    bracket (0.4) + wall (1.6 behind the recess), threading `sw_boss_pilot` (1.5) into the
    pillar (up to `sw_boss_h` 5 mm engagement). The head sits on the external bracket, so the
    case has no head counterbore.
  - **Bracket recess** (`sw_plate_recess`, default on): the flat bracket (`sw_plate_w ×
    sw_plate_h × sw_plate_t` = **19.45 × 5.75 × 0.4 mm**) mounts on the **outside**, sitting
    in a pocket `sw_plate_t` deep cut into the **outer** wall so it is **flush** with the
    outside. The inner wall behind it stays solid, which is what the bosses bear against.
    `sw_plate_clr` (0.3) adds fit clearance around the bracket. Verify `sw_plate_w/h/t`
    against your switch; set `sw_plate_recess = false` to drop the pocket.

## USB-C opening
Two layers in the wall (outside in):
- `usb_open_w/h` = **11 × 7 mm**, R2 (`usb_open_r`) – pass-through through the whole wall.
  This outer edge IS the **lip** that stops the cable body/overmold (anything > 11 × 7)
  from coming in; the metal plug itself (~8.34 × 2.56) passes through easily.
- `usb_recess` (**on**) – an **inner recess** `usb_recess_w/h/d` = **13 × 9 × 1.5 mm**,
  R3 (`usb_recess_r`), on the cavity side of the wall. It eats away the last 1.5 mm of
  the wall so the USB-C receptacle's body/shell can poke slightly into the wall and bring
  the connector mouth close to the outer face.

The opening's centre sits ~1.65 mm **above** the base/lid split, so the lower ~1.85 mm of
the 7 mm-tall window falls below the seam. `cut_usb()` therefore runs in **both** `base()`
and `lid()` — otherwise the base side wall would block the lower edge of the hole.

Measure your own charging cable and adjust if needed. Test with a print of just the left
short end first.

## Known / to verify
- Check that your charging cable reaches the port (see "USB-C opening" above).
- **XH:** plug the XHP-4 into U4 through the wall opening with the lid on; check the plug's
  latch clears the top of the opening and can be released from outside.
- **Battery:** check the PHR-2 plug seats in BAT1 with the board on its pins, that the lead
  reaches the cell in the pocket via the back pocket, and that nothing presses on the cell.
  The `bat1_env` plug + wire-exit envelope (board-X to 19) is an estimate from the header
  drawing, not measured with a real plug.
- **THT tails:** with `under_clr` 3.7 the S3/U4 tails clear the cell by 0.3 mm. Clip them or
  keep the foam pad if you want more.
- **Light pipe**: verify the rod lands directly over D3/D4 (adjust `led_pos` if needed)
  and that the bottom clears the LEDs (`led_pipe_gap`). Print in clear filament; glue the
  head into the counterbore for a permanent/sealed fit.
- The USB-C, XH and switch openings are open out of necessity; the LED window is sealed by
  the light pipe.
- **Snap walls:** the 2.4 mm wall's lap halves held on the rev-1 print but felt thin for
  repeated opening. Not changed in this revision; revisit if the box is opened often.
