# Parametric enclosure for the nRF52 board (build123d)

`openrz67_nrf_case.py` is a two-part box, plus a light pipe, for the coin-cell board in
[`../kicad/`](../kicad/): a one-piece shell and a floor that slides out backwards. Outer size
with default values: **37.1 × 37.6 × 12.2 mm**. Printed in ABS, shell upside-down, floor
floor-down, no supports. The light pipe prints in clear filament, head-down, and is glued into
the shell.

Layout: the board sits in the shell component side up, against four bosses off the ceiling,
with the CR2032 and its clip hanging below it. The floor holds it up from below and carries
the lower back wall. Nothing flexes to open it.

**Changing the cell:** take the box off the camera and turn it upside down. Pull the floor out
backwards with a fingernail in the notch at the top of its back. The board stays in the shell.
Slide the cell out of the clip backwards, slide the new one in, + side up (toward you), then
slide the floor back in until it clicks.

Velcro goes on the floor's middle only, not across the shell's bottom edges, or it glues the
floor shut.

## Source data

Board coordinates are the KiCad coordinates of `openrz67-nrf.kicad_pcb` (origin top-left,
Y toward the back wall). KiCad's view turned 180° puts the antenna at the front, so board X
runs right to left in the case: the module sits front-right, the camera plug back-left. Values come from `../kicad/tools/design.py` and the part datasheets:

| What | Value |
|---|---|
| Board | 29.5 × 29 mm, R0.8, 1.6 thick, no mounting holes |
| Cell clip `BT1` (back) | 25.9 × 9.4, 3.75 high, cell Ø20 × 3.2 under it, centre (13.5, 18) |
| Button `SW1` | B3U-1000P at (6.0, 27.6), 1.6 high |
| LED `D1` | (18.2, 27.6) |
| Camera connector `J1` | SM04B-SRSS-TB at (23.6, 24.7), 6.0 wide, 2.9 high, mouth toward board y = 29 |
| SWD connector `J2` | same part at (12.3, 24.7), same mouth direction, no wall opening |
| Module `U1` | 15 × 19 × 2.0 incl. pads, antenna end at board y = 0 |

## Construction

- **Shell** (`lid` in the script and STL names): top and four walls, open at the bottom.
  Below the board the back wall is missing: that part belongs to the floor.
  - **Bosses** (`bosses`, board x/y/Ø) at four free spots press on bare board. The module owns
    the board's (0, 0) corner and `C1` + `J1` the (29.5, 29) one.
  - **Button**: a `tab_w × tab_l` (6 × 7.45) cantilever tab cut free by a `tab_slot` (0.7)
    U-slot with `tab_r` (1.5) corners at the free end, hinged at its front end, thinned to `tab_t` (0.8) from the inside, with a
    Ø`nub_d` (2.4) nub that stops `nub_gap` (0.3) above the switch cap. The free end is
    `tab_tip` (1.45) past the switch centre, so it ends over the cavity, not over the wall.
  - **LED light pipe**: a top hat over `D1`, as on the ESP32 case. Ø`led_stem_d` (2.2) stem,
    ending `led_pipe_gap` (0.8) above the board, in a slip-fit window (`led_pipe_clr` 0.2)
    that a collar off the ceiling lengthens down to the pipe's lower end. The head tapers out
    by `led_head_lip` (0.7) over `led_head_t` (1.4), 26.6° from vertical, into a matching
    funnel in the top, so it self-centres and nothing needs support. Glued.
  - **Text**: "OpenRZ67" (5.8) over a tracked "TRIGGER" (4.0), Futura Bold grown
    `lid_text_bold` (0.05) per side so every stroke is at least 0.6 mm, debossed
    `lid_text_depth` (0.8) into the top. Centred in X, and in Y between the front chamfer and
    the LED. A 0.9 mm rail runs from the left side into the LED seat, as on the ESP32 case.
    The pockets are filled by
    `openrz67-nrf-lid-text.stl` (text and rail), a second part of the lid on filament 2 in the .3mf.
    `LID_TEXT_SHOW=false` turns it off; `LID_TEXT` / `LID_TEXT_SIZE` change the name line.
    The text sets the case width: `wall` is 3.4 so "OpenRZ67" clears the edge chamfers.
  - **Plug opening**: a window in the back wall, connector width + 2 × `conn_clr` (0.5),
    from the board top up `conn_h + conn_clr`.
- **Floor** (`base`): a `floor_t` (1.6) plate with a dovetail edge that runs `slide_d` (1.0)
  into grooves in the side walls and the front wall. The lower `slide_d` of the edge is a
  45° chamfer, on both parts, so neither needs support. `slide_gap` (0.15) all round.
  - **Back band**: the back wall below the board, `wall` deep, full width, part of the floor.
    With the floor out, the back is open below the board, so the cell can slide out of the
    clip backwards (it needs 14.5 mm to clear the strap).
  - **Board support**: the plate top sits `clip_gap` (0.1) under the clip's strap, and a
    rail on the plate sits `clip_gap` under the board's edge at board x 29.5, the only edge
    the clip leaves free along the whole slide path. The board is held between these and the
    bosses.
  - **Latch**: the plate's right edge strip is an arm (`latch_l` 15 long, `latch_w` 2.2 wide)
    cut free by a `latch_slot` (1.4) slot. Its free end carries a catch, the dovetail edge
    pushed out `latch_p` (1.2), that clicks into a window through the bottom of the right side
    wall. The catch's front is a 45° ramp, its back face square. To open: push the catch in
    through the window with a fingernail and slide the floor out backwards. The arm prints flat
    and bends in the plane of the layers; an assertion keeps its root strain under
    `latch_strain` (2 %). Tune with `SLIDE_TEST=true`.
  - **Polarity mark**: a 10 mm "+" debossed 0.4 into the plate under the cell: the cell's +
    side faces the floor.

The script carries module-level assertions: each part is one body, the parts do not
overlap, outer dimensions, the board / cell / clip / module / connector / switch / LED
keepouts (the clip as strap plus solder legs, from its 3D model), the bosses and the
light-pipe collar clear of every top-side part, the light pipe clear of the shell, the button
tab tip over the cavity, the camera plug opening on the left (a board mapped without the
mirror fails here), every opening breaking through, the board's way into the shell from
below, the floor sliding out past the shell and under the board, cell and clip at every step,
the latch catch holding against a pull, bending the arm and showing in its window, the floor actually under the strap and the board edge, and the cell's
way out backwards with the floor out. They run on every export.

## Usage

```sh
cd nrf52/case
./export.sh                          # base + lid + light pipe -> stl/, then openrz67-nrf-case.3mf
SLIDE_TEST=true ./export.sh          # also the back end of both parts, for tuning the fit (not in the .3mf)
./view.sh                            # board + case in CAD Viewer (browser), closed and with the floor out
```

`view.sh` exports the board STEP from KiCad (`kicad-cli`), places it in the case with
`view.py`, writes GLBs to `view/` and serves them with `cadgen viewer`. Open the printed URL
with `?file=nrf-closed.glb` or `?file=nrf-floor-out.glb`.

Requires [uv](https://docs.astral.sh/uv/); the script carries its own dependency header.

`openrz67-nrf-case.3mf` is the QIDI Studio project: plate 1 floor floor-down and shell upside-down
(ABS profile, lid text on filament 2), plate 2 the light pipe head-down on filament 3 (clear,
0.28 mm layers, 100 % infill, 20 mm/s). The template has the ESP32 case's settings, except a
0.42 mm first-layer line width (the lid text is the first layer). `../../case/make_3mf.py` builds it by swapping the fresh STLs into
`qidi-template.3mf`, so the slicer settings survive every re-export. To change settings or the
plate layout: open the project in QIDI Studio, change it, save, and copy it over
`qidi-template.3mf`.
