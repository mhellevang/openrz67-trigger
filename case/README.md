# Parametric enclosure (build123d)

`openrz67_case.py` is a parametric two-part enclosure for the ESP32CamTrigger PCB
(rev. 2025-09-23), written in [build123d](https://build123d.readthedocs.io/) (Python).
It is built from the actual production files in `../pcb/`, not from eyeballed
measurements. (It started life as OpenSCAD — `openrz67-case.scad` in git history — and
was ported 1:1; the port was verified against the OpenSCAD STLs by bounding box and
volume, matching within tessellation error.)

## Source data
| Measurement | Value | Source |
|-----|-------|-------|
| Board outline | 48.0 × 22.0 mm, R2.0 | `Gerber_BoardOutlineLayer.GKO` |
| Mounting holes | 2× Ø2.0 at (2,2) and (46,20) | `Drill_PTH_Through.DRL` (tool T06) |
| PCB thickness | 1.6 mm | `pcb_t` (as ordered) |
| Component placement | see openrz67_case.py | `PickAndPlace_…_2025-09-23.csv` |
| USB-C connector | body 8.95 × 3.2 mm on the PCB, left edge 4.8 mm in from the PCB's left edge; the shell protrudes ~2 mm past the board edge | measured physically |
| Total HW length | ~50 mm (PCB 48 + USB-C shell 2) | `pcb_overhang_left` |
| Slide switch (SS12F15) | actuator opening 10.65×6.3; flat bracket 19.45×5.75×0.4; screw holes Ø2.2 at 15.0 mm spacing; body ~19.5×20×12.9 | `SS12F15.stp` + measured |
| LEDs | D3 red (22.5, 20.2), D4 blue (24.2, 20.2) – 0603 SMD, top-emitting, 1.7 mm apart | `PickAndPlace_…_2025-09-23.csv` |

Component **heights** could not be read reliably from the STEP file (the 3D models live
in their own local coordinate systems), so they are entered as editable constants
(`h_relay`, `h_xh`, `h_usbc`, `h_ph`) based on datasheets. Check them against your own parts.

The script carries **module-level assertions**: outer dimensions, plus probe checks that
every opening actually breaks through (USB, XH port, LED window, locating-pin recesses,
switch screws) and that the fit-critical keepouts (switch body envelope, USB-C body) stay open.
They run on every export, so a parameter tweak that closes a hole or re-introduces a
known collision fails loudly instead of surfacing in the print.

## Construction
- **Base** (tub): floor, a **battery bay beside the PCB** (behind its back edge, separated
  by a divider wall), low standoffs that raise the PCB ~3.5 mm (only the
  through-hole solder tails need clearance underneath), **solid support pillars** in all
  four corners (`pcb_supports = [[2,20],[46,2]]` plus the two posts at the mounting
  holes), and two **Ø1.7 locating pins** up into the real Ø2 mounting holes.
  - **Through-hole solder relief** (`th_keepouts`): the support pillar at (2,20) sits
    next to BAT1 (a through-hole connector). A small pocket is cut into the top of the
    pillar where BAT1's pin tails/solder stick down, so they don't press against the
    pillar. The list is `[board_x, board_y, relief_diameter]`; `th_keepout_depth` sets
    how far down (default 3 mm). Add more entries if other through-hole solder joints
    collide with a standoff/pillar.
  - **PCB frame / guide fins** (`frame_*`): the lap joint removes the inner
    wall at board height, so the PCB rests only on the four corner points and has nothing
    to hold it sideways before the lid is on. There is only ~0.4 mm of clearance beside
    the board (too thin for a printable fin), but the lap joint has a ~1 mm **tongue
    channel** just outside the wall line. **Guide fins** along the front and back edges
    sit in that channel: a thick fin (anchored to the wall below the lap zone) beside the
    board edge with `frame_clr` (0.2 mm) clearance, plus a thin lead-in lip `frame_proud`
    (0.6 mm) above the board top. The lid tongue is cut away where the fins sit
    (`frame_tongue_notch`) so they clear when the lid closes. The board drops straight down and is captured in Y between the front
    fins and the battery-bay **divider wall** (whose front face sits `frame_clr` from the
    board's back edge, so no back fins are needed — `frame_ribs_back = []`).
    `frame_ribs_front` are the fin centres in board-X (default `[10, 38]`);
    `frame_notch_clr` is the clearance in the tongue notch.
- **Lid**: telescopes down into the base via a perimeter tongue-and-groove edge (lap
  joint) for alignment. Carries the USB-C opening (left), a small cable pass-through for
  the XH plug (right – the connector itself stays enclosed inside), the LED light-pipe
  window (top), and the slot + screw pillars for the SS12F15 slide switch.
  - **XH cable exit — drop-in** (`cut_xh`): the XH port sits high (near the plug's 8 mm
    top-entry wire exit, for a gentle bend) but is **open downward to the split** via a slit.
    The pre-wired bundle **lays into** the slit as the lid closes instead of being threaded
    through a blind hole, and the lid then **lifts straight off** without the wires tethering
    it. `cable_port_w/h` size the mouth; `cable_port_z` is its height above the PCB.
  - **LED light pipe** (`led_*`, separate part): D3 (red) and D4 (blue) are top-emitting
    SMD LEDs ~8 mm below the lid. A separate **clear light pipe** is inserted from above
    as a top hat: a wide head in a top counterbore (flush with the top face) + a rod that
    goes down to ~1.5 mm above the LEDs and channels the light into two sharp dots. The
    lid is printed opaque; **only the light pipe is printed in clear filament**. It rests
    in the counterbore (gravity + optionally a drop of glue to seal). Parameters:
    `led_win_l/w/r` (window size), `led_head_lip/t` (head/counterbore), `led_pipe_gap`
    (air gap to the LED), `led_pipe_clr` (fit in the hole). `led_pos` are the LED
    positions from pick&place.
  - **Component clearance** (`comp_keepouts`): the hold-down bosses in the lid can
    collide with component bodies above the PCB. The USB-C connector's near edge
    (~board-Y 4.8) sat against the Ø6 hold-down at hole (2,2), so the switch couldn't be
    lowered without pushing the PCB out of position. The clearance cuts the boss back
    (D-shape) on the connector side — the same approach as last year's design where the
    corner boss was small and tucked in toward the corner. The list is
    `[board_x0, board_y0, board_x1, board_y1]` (rectangle, full height above the PCB).
- **USB-C asymmetry**: the USB-C shell protrudes ~2 mm past the PCB's left short side.
  `pcb_overhang_left = 2.0` extends the cavity on the left; the PCB is therefore centred
  toward the RIGHT in the cavity (normal `clr = 0.4` against the right wall, `clr +
  overhang = 2.4 mm` against the left). The outer case width thus becomes ~**54.8 mm**.
- **The battery** (250 mAh LiPo) lies **beside the PCB** in its own bay behind the board's
  back edge (it used to sit *under* the PCB, which forced a tall blind stack and a stiff-wire
  riser channel up to BAT1 — in practice too cramped to assemble). The bay is bounded by the
  **divider wall** (front, top flush with the split — it doubles as the PCB's back guide),
  the case walls (back/sides), and two low ribs that stop the cell sliding in X. BAT1 sits at
  board ≈ (3.3, 17) on the PCB top, so the lead just **drapes over the divider** and plugs
  straight in — short and flat, no riser. Everything is assembled from above in one layer.
  The bay is taller than the cell; fix it with a foam pad or a dab of glue.
  Parameters: `batt_w/l/t` (cell), `batt_clr` (bay fit), `batt_dx` (X nudge), `divider_t`,
  `rib_h`.

## Closure — snap-fit
**No hardware at all.** A perimeter **bead** (`snap_bead` 0.5 mm, `bead_h` 1.4) on the lid
tongue clicks into a matching **groove** in the base lip (0.5 taller / 0.25 deeper for
lead-in), holding the whole rim down — including the battery-bay half, which a corner-screw
pattern would leave unclamped. The PCB is located by two **Ø1.7 pins** in its real Ø2
mounting holes and pressed onto the posts by the lid's hold-down bosses (blind pin recesses
inside — no holes through the top). Open with a coin or fingernail in the **pry slot**
(`pry_w/d/h`) on the back wall's lower lid edge, over the battery bay.

The case is rarely opened (charging is external via USB-C), so snap wear is not a concern
with PLA; print the lid in PETG if you want extra flex margin. **Tune before the full
print:** `SNAP_TEST=true ./export.sh` also exports a cropped front-left corner pair
(`openrz67-snaptest-*`) — print those and adjust `snap_bead` (click strength) and
`lap_gap` (sliding fit) until the corner snaps shut and pries open the way you like.

(The previous closure — M2 screws into heat-set inserts through the PCB holes — proved
fiddly: tight M2-through-Ø2 clearance and annoying insert installation. It lives in git
history.)

Finished size with default values: ~**54.8 × 49.2 × 18.1 mm** (X incl. the 2 mm USB-C
overhang, Y incl. the battery bay behind the board, Z depends on `pcb_t`). Flat-wide
instead of the old tall-tight ~54.8 × 28.8 × 21.6 stack.

## Orientation mark — `orient_mark`
A small **raised rib** on the front wall (low Y) near the left corner, split across the
seam: the base carries the lower half, the lid the upper half. When the lid is on the
right way around the two halves line up into **one continuous vertical rib**; a lid put on
180° wrong moves its half to the opposite corner, so the mismatch is obvious at a glance.
It is raised (not a recessed groove) on purpose — a groove here would thin the 1 mm lap
wall. Parameters: `orient_mark_x` (board-X of the mark, near the USB side), `orient_mark_w`
(width), `orient_mark_d` (how far it sticks out), `orient_mark_h` (total height across the
seam). Delete the two `orient_mark_rib` lines to remove it.

## Lid text — second colour (`lid_texts`)
**Debossed badge layout** around the LED window: **"OpenRZ67"** (9 mm caps) above it,
**"Trigger"** (7 mm) below, both centred on the light pipe's X, cut `lid_text_depth`
(0.6 mm) into the top face. Per the FDM rules: pockets, not raised letters, and well above
the 4.4 mm legibility minimum for a 0.4 nozzle. Placement is enforced by assertions (each
line must clear the LED recess, the walls and the face edges — an over-long string fails
the export instead of the print).

Colour it with a **filament change by height**: the lid prints **upside down** as always
(the pockets face the bed and come out crisp), so change filament at **Z = 0.6 mm** — the
first three layers are the face colour, everything after is the letter colour showing
through the pockets. No supports, single-extruder friendly.

Edit the `lid_texts` constant to change strings/sizes/offsets; `LID_TEXT_SHOW=false`
disables the text (on by default).


## Usage
The parameters are plain Python constants at the top of `openrz67_case.py`, grouped in
the same sections as before. Requires [uv](https://docs.astral.sh/uv/) — the script
carries its own dependency header, so there is no venv to manage.

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
  point upward with no overhang and the LED window/counterbore gets a nice top surface.
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
  is the two empty corners `[[2,20],[46,2]]`; add more if the board flexes.
- `h_xh` – the tallest component (the XH plug) sets the total height. Lower the
  `comp_clr` basis if your plug/cable is shorter.
- `cable_port_w/h/z` – the cable pass-through for the XH. Set `cable_port_w ==
  cable_port_h` for a round hole. Plug in the XH connector before assembling.
- `snap_bead` / `lap_gap` – snap click strength and sliding fit. Tune with the
  `SNAP_TEST=true` corner pieces before printing the whole box.
- `batt_w/l/t` / `batt_clr` – fit your LiPo in the bay; `batt_dx` nudges it in X if the
  leads want a different exit point.
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
  no free-standing cantilever. The `sw_boss_gap` air gap is needed because the pillar
  footprint (board-X 14.5 and 29.5) sits over densely packed front components (Q1 SOT-23
  ~1.1 mm, C15/C26 0603 ~0.9 mm); too small a gap pushed the pillars against them and held
  the lid ~0.5 mm open. `sw_screw_pitch` = 15.0 (centre spacing, ±7.5), `sw_screw_d` = 2.4
  (M2 clearance), `sw_boss_d/h/pilot` set the pillar cross-section and pilot hole.
  - **Body keepout** (`sw_body_l × sw_body_h × sw_body_w` = **11 × 7 × 8 mm**, + `sw_body_clr`
    0.4/side): the body that protrudes inside is ~11 mm wide, but the bosses (5 mm wide at
    ±7.5) leave only a 10 mm gap, so each boss bit 0.5 mm into the body and stopped it short.
    A keepout carves the body envelope out of the cavity, relieving the boss inner faces over
    the body's Y/Z extent only — the screw region (±7.5) and the pillar above the body stay
    full. Verify `sw_body_l/h/w` against your switch.
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
Matches last year's design (confirmed from `3D_ESP32CamTrigger_PCB v2 2025-09-01.step`).
Two layers in the wall (outside in):
- `usb_open_w/h` = **11 × 7 mm**, R2 (`usb_open_r`) – pass-through through the whole wall.
  This outer edge IS the **lip** that stops the cable body/overmold (anything > 11 × 7)
  from coming in; the metal plug itself (~8.34 × 2.56) passes through easily.
- `usb_recess` (**on**) – an **inner recess** `usb_recess_w/h/d` = **13 × 9 × 1.5 mm**,
  R3 (`usb_recess_r`), on the cavity side of the wall. It eats away the last 1.5 mm of
  the wall so the USB-C receptacle's body/shell can poke slightly into the wall and bring
  the connector mouth close to the outer face. **This is the trick from the previous
  case** – the same one used for reach there.

The opening's centre sits ~1.65 mm **above** the base/lid split, so the lower ~1.85 mm of
the 7 mm-tall window falls below the seam. `cut_usb()` therefore runs in **both** `base()`
and `lid()` — otherwise the base side wall would block the lower edge of the hole (only the
upper part, cut from the lid, would be open).

Measure your own charging cable and adjust if needed. Test with a print of just the left
short end first.

## Known / to verify
- Check that your charging cable reaches the port (see "USB-C opening" above).
- Check that the wire bundle from the XH cable fits the `cable_port_*` mouth (it now drops
  into the open-bottom slit rather than threading through — the lid lifts off without pulling
  the wires back through).
- **Light pipe**: verify the rod lands directly over D3/D4 (adjust `led_pos` if needed)
  and that the bottom clears the LEDs (`led_pipe_gap`). Print in clear filament; glue the
  head into the counterbore for a permanent/sealed fit.
- The USB-C and switch openings are open out of necessity; the LED window is sealed by
  the light pipe. The enclosure is dust-resistant-ish, not waterproof. Let me know if you
  want a USB plug/flap or a gasket.
