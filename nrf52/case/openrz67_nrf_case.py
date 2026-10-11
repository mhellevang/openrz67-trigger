# /// script
# requires-python = ">=3.12"
# dependencies = ["build123d>=0.11.1"]
# ///
"""openrz67 nRF52 trigger enclosure (build123d).

Two-part box for the coin-cell board in ../kicad/ (29.5 x 29 mm, CR2032 clip on the back):
a one-piece shell (top and walls, open at the bottom) and a sliding floor. The board sits in
the shell component side up, against four bosses off the ceiling. The floor slides in from
the back in dovetail grooves along the shell's side walls and front wall, and holds the
board up from below: its plate presses on the cell clip's strap, a rail on it under the one
side edge the clip leaves free. The floor carries the lower back wall with it, so with the
floor out the cell slides out of the clip backwards. A latch arm in the floor plate clicks a
catch into a window in the right side wall; push the catch in to slide the floor out. The
arm prints flat and bends in the plane of the layers.

The shell carries a cantilever button tab over SW1, a light pipe over D1 and the opening for
the JST SH camera plug in its back wall.

Parts: lid (the shell), base (the sliding floor) and light pipe (clear filament, glued into
the lid). Print orientation: lid upside-down, base floor-down, light pipe head-down. No
supports.

Coordinate system: case coords, origin at the outer box's front-left-bottom corner.
Board coords are the KiCad coords of ../kicad/openrz67-nrf.kicad_pcb (origin top-left of
the board, Y down in KiCad = toward the back wall here). bx()/by() map them into case
coords. KiCad shows the board from above with the antenna at the top of the screen; with
the antenna at the front wall that view is turned 180 degrees, so board X runs right to
left in case coords (bx mirrors it) while board Y keeps its direction. Antenna end of the module at the front wall (board y = 0), camera connector mouth
at the back wall (board y = 29).

Run (exports STLs to stl/ and checks the assertions):
    uv run openrz67_nrf_case.py

Env overrides: PCB_T, OUTDIR (STL directory, default stl/), SLIDE_TEST=true (also export the
back end of both parts for tuning slide_gap / the latch), LID_TEXT_SHOW=false, LID_TEXT,
LID_TEXT_SIZE.
"""

import math
import os
from pathlib import Path

from build123d import *


def check(cond, msg):
    """Keepout / printability guard. Not `assert`: `python -O` would strip those."""
    if not cond:
        raise SystemExit(msg)


# --- Board (from ../kicad/tools/design.py) --------------------------------------------
board_w, board_h, board_r = 29.5, 29.0, 0.8
pcb_t = float(os.environ.get("PCB_T", "1.6"))
cell_c = (13.5, 18.0)                 # CR2032 centre (BT1), on the back
cell_d, cell_t = 20.0, 3.2
clip_l, clip_w, clip_h = 25.9, 9.4, 3.75   # MY-2032-16 clip incl. feet; highest point under the board
# From the clip's 3D model: within 2.5 mm of the board only the two solder legs (pad 2 height)
# are there; the strap and its contact dimples take the lowest clip_strap_h.
clip_leg_w, clip_strap_h = 5.0, 1.3
sw_c, sw_h = (6.0, 27.6), 1.6         # B3U-1000P centre and height
led_c = (18.2, 27.6)                  # D1
conn_c, conn_w, conn_h, conn_mouth = (23.6, 24.7), 6.0, 2.9, 2.47   # SM04B-SRSS-TB (J1): centre, body, height, body past the centre toward the mouth
swd_c = (12.3, 24.7)                  # J2, same connector, same mouth direction, no opening (flashing is a lid-off job)
module = (1.0, 0.0, 13.0, 17.2, 2.5)  # E73-2G4M08S1F: x0, y0, x1, y1, height (2.4 per the manual)
max_part_h = 2.9                      # tallest part above the board (the connector)

# --- Enclosure ---------------------------------------------------------------------------
clr = 0.4                 # board edge to inner wall
wall = 3.4                 # the lid text at 5.8 sets the width, not the parts
floor_t = 1.6             # sliding floor plate
lid_top_t = 1.6
edge_r = 1.0              # 45 deg chamfer on the top and bottom outer edges
clip_gap = 0.1            # floor plate to the clip strap, and rail to the board: the board's play
over_clr = 0.6            # air over the tallest part
# lid hold-down bosses: (board x, y, diameter). Not at the board corners: the module owns
# the front-left one and C1 + J1 the back-right one
bosses = [(1.3, 21.2, 2.4), (28.0, 1.5, 2.4), (1.5, 27.5, 2.4), (28.5, 23.0, 2.0)]
ant_wall_clr = 1.0        # air in front of the antenna end (board y 0)
# Sliding floor: dovetail edge, slide_d into the side and front walls, the lower slide_d of it
# a 45 deg chamfer (prints without support on both parts).
slide_d, slide_gap = 1.0, 0.15
# Latch: the floor's right edge strip is a cantilever arm cut free by a slot, flexing in the
# plane of the layers. A catch at its free end (the dovetail edge pushed out by latch_p) clicks
# into a window through the right side wall; its back face is square, so the floor cannot be
# pulled out until the catch is pushed in through the window. A bump detent (shell walls
# springing over it) held only by friction and let go to a pull at 0.3 and 0.4 proud.
latch_l, latch_w, latch_slot = 15.0, 2.2, 1.4   # arm root to free end, width at the plate top, slot
latch_p, latch_catch_l = 1.2, 3.0               # catch proud of the floor edge, catch length
latch_hold = 3.0          # side wall behind the window: takes the pull
latch_strain = 0.02       # max bending strain at the arm root (ABS yields around 3 %)
# Rail on the floor under the board edge at board x = board_w: the only edge the clip leaves free
# along the whole slide path (the clip spans board x 0.55 .. 26.45).
rail_x0 = cell_c[0] + clip_l / 2 + 0.45

# Button: cantilever tab in the lid top over SW1, nub underneath
tab_w, tab_l, tab_slot, tab_r = 6.0, 7.45, 0.7, 1.5   # tab_r: free-end corners
tab_tip = 1.45            # switch centre to the tab's free end: the tip stays over the cavity
tab_t = 0.8               # tab thinned from the inside to this: about 5 N to press (8 N at 1.0)
nub_d, nub_gap = 2.4, 0.3  # nub diameter, air over the switch cap at rest
# Light pipe: top hat as on the ESP32 case (../../case/openrz67_case.py has the history):
# tapered head in a funnel, stem in a slip-fit window, glued. The collar off the ceiling
# keeps it from tilting; only D1 and R3 sit under its ring, both under low_part_h.
led_stem_d = 2.2
led_head_lip, led_head_t, led_pipe_clr, led_pipe_gap = 0.7, 1.4, 0.2, 0.8   # gap from the board top
led_collar_w = 0.87       # two perimeter lines
low_part_h = 0.6          # D1 (0603 LED); R3 (0402) is lower
conn_clr = 0.5            # around the connector body in the wall opening
cell_mark_l, cell_mark_w, cell_mark_d = 10.0, 1.5, 0.4   # "+" debossed in the floor under the cell: its + side faces the floor
# Lid text: debossed in the top, filled by an inlay part on filament 2 (make_3mf.py), as on
# the ESP32 case (../../case/openrz67_case.py has the reasoning). Centred in X, and in Y
# between the front edge chamfer and the LED seat. A rail runs from the left wall into the
# light-pipe seat, as on the ESP32 case: the LED reads as an indicator on a line.
# (text, font size, letter tracking)
lid_text_show = os.environ.get("LID_TEXT_SHOW", "true") == "true"
lid_font = "Futura"
lid_texts = [(os.environ.get("LID_TEXT", "OpenRZ67"), float(os.environ.get("LID_TEXT_SIZE", 5.8)), 0.0),
             ("TRIGGER", 4.0, 0.6)]
lid_text_gap, lid_text_min_stroke, lid_text_depth = 1.2, 0.6, 0.8
# At the size this lid allows, Futura Bold's thinnest strokes ("e", "pn") are 0.52 mm; every
# glyph outline is grown by this much per side to clear lid_text_min_stroke. No more: it
# closes the counters.
lid_text_bold = 0.05
lid_rail_w, lid_rail_end, lid_rail_gap = 0.9, 4.0, 1.2   # width, inset from the left wall, air to the LED seat
eps = 0.01

# --- Derived --------------------------------------------------------------------------------
inner_w = board_w + 2 * clr
inner_h = board_h + 2 * clr + ant_wall_clr
inner_r = board_r + clr
outer_w, outer_h, outer_r = inner_w + 2 * wall, inner_h + 2 * wall, inner_r + wall
pcb_z = floor_t + clip_h + clip_gap
pcb_top_z = pcb_z + pcb_t
comp_clr = max_part_h + over_clr
total_h = pcb_top_z + comp_clr + lid_top_t
ceiling_z = total_h - lid_top_t
band_y = outer_h - wall   # the floor's back band: the back wall below the board
# Latch, in case coords: floor edge, arm inner side, catch from y0 to y1 (the free end)
latch_xe = outer_w - wall + slide_d
latch_xa = latch_xe - latch_w
latch_y1 = band_y - 2 * slide_gap - latch_hold
latch_y0 = latch_y1 - latch_catch_l
latch_defl = latch_p - slide_gap          # the arm's bend while the catch rides in the groove

check((wall - edge_r) / math.sqrt(2) >= 0.87, "groove lip between the two chamfers under two perimeter lines")
check(latch_slot >= latch_defl + 0.3, "latch slot too narrow for the arm to bend")
check(1.5 * latch_w * latch_defl / latch_l ** 2 <= latch_strain, "latch arm bends past latch_strain: lengthen it")
check(latch_p < wall - slide_d - slide_gap, "latch catch pokes out of the side wall")
check(floor_t - cell_mark_d >= 1.0, "floor under the + mark too thin")


def bx(x):
    return wall + clr + board_w - x


def by(y):
    return wall + ant_wall_clr + clr + y


# --- Helpers ---------------------------------------------------------------------------------
def rrect(w, h, r, x0=0.0, y0=0.0):
    return Pos(x0 + w / 2, y0 + h / 2) * RectangleRounded(w, h, r)


def prism(sk, z0, h):
    return extrude(Plane.XY.offset(z0) * sk, amount=h)


def box(x0, y0, z0, dx, dy, dz):
    return Pos(x0, y0, z0) * Box(dx, dy, dz, align=Align.MIN)


def cyl(x, y, z0, d, h):
    return Pos(x, y, z0) * Cylinder(d / 2, h, align=(Align.CENTER, Align.CENTER, Align.MIN))


def dovetail(sk, grow, top):
    """Sliding-floor body: sk grown by `grow`, its lower slide_d a 45 deg chamfer, up to z = top.
    Starts 1 mm below z = 0 so the shell's groove cut clears the bottom face."""
    low = extrude(Plane.XY.offset(-1) * offset(sk, grow - slide_d - 1), amount=slide_d + 1, taper=-45)
    return low + prism(offset(sk, grow), slide_d - eps, top - slide_d + eps)


outer_sk = rrect(outer_w, outer_h, outer_r)
inner_sk = rrect(inner_w, inner_h, inner_r, wall, wall)
board_sk = rrect(board_w, board_h, board_r, bx(board_w), by(0))
# The floor plate in plan, and its slide path (the same, run out past the back)
floor_sk = rrect(inner_w + 2 * slide_d, outer_h - wall + slide_d, inner_r + slide_d, wall - slide_d, wall - slide_d)
slide_sk = rrect(inner_w + 2 * slide_d, outer_h + 10, inner_r + slide_d, wall - slide_d, wall - slide_d)

envelope = prism(outer_sk, 0, total_h)
envelope = chamfer(envelope.edges().group_by(Axis.Z)[0] + envelope.edges().group_by(Axis.Z)[-1], edge_r)
back_band = box(-1, band_y, -1, outer_w + 2, wall + 2, pcb_z - slide_gap + 1)

# --- LID (the shell) -------------------------------------------------------------------------
lid = envelope - box(-1, band_y - slide_gap, -1, outer_w + 2, wall + 2, pcb_z + 1)   # the back band's place
lid -= prism(inner_sk, -1, ceiling_z + 1)                                         # cavity, open at the bottom
lid -= box(wall, wall + inner_r, -1, inner_w, outer_h, pcb_z + 1)                 # square back corners below the board: the rail's way out
lid -= dovetail(slide_sk, slide_gap, floor_t + slide_gap)                         # floor grooves
lid -= box(outer_w - wall, latch_y0 - slide_gap, -1, wall + 1, latch_catch_l + 2 * slide_gap, floor_t + slide_gap + 1)   # latch window
for cx, cy, d in bosses:
    lid += cyl(bx(cx), by(cy), pcb_top_z, d, ceiling_z - pcb_top_z + eps)
# Camera plug opening: body width + clearance, from the board top up
conn_x0 = bx(conn_c[0]) - conn_w / 2 - conn_clr
conn_open_w = conn_w + 2 * conn_clr
conn_open_h = conn_h + conn_clr
lid -= box(conn_x0, band_y - 1, pcb_top_z - eps, conn_open_w, wall + 2, conn_open_h + eps)
# Button: U-slot around a tab hinged at its front end, tab thinned from inside, nub under it
tx, ty = bx(sw_c[0]), by(sw_c[1])
tab_y0 = ty - (tab_l - tab_tip)
tab_x0 = tx - tab_w / 2
tab_sk = rrect(tab_w, tab_l, tab_r, tab_x0, tab_y0) + Pos(tx, tab_y0 + tab_l / 4) * Rectangle(tab_w, tab_l / 2)
slot_sk = (rrect(tab_w + 2 * tab_slot, tab_l + tab_slot, tab_r + tab_slot, tab_x0 - tab_slot, tab_y0)
           + Pos(tx, tab_y0 + tab_l / 4) * Rectangle(tab_w + 2 * tab_slot, tab_l / 2))
lid -= prism(slot_sk - tab_sk, ceiling_z - 1, lid_top_t + 2)
lid -= prism(tab_sk, ceiling_z - eps, lid_top_t - tab_t + eps)   # thin the tab
nub_z0 = pcb_top_z + sw_h + nub_gap
lid += cyl(tx, ty, nub_z0, nub_d, (ceiling_z + lid_top_t - tab_t) - nub_z0 + eps)
# LED light pipe: collar off the ceiling, stem window through plate + collar, funnel for the head
lx, ly = bx(led_c[0]), by(led_c[1])
led_taper = math.degrees(math.atan(led_head_lip / led_head_t))
check(led_taper <= 28, f"light-pipe head taper {led_taper:.1f} deg would get slicer support (30 deg threshold)")
check(led_head_t < lid_top_t, "light-pipe head seat goes through the top plate")
pipe_z0 = pcb_top_z + led_pipe_gap
collar_z = pipe_z0
collar_d = led_stem_d + led_pipe_clr + 2 * led_collar_w
check(collar_z - pcb_top_z >= low_part_h + 0.2 - 1e-9, "light-pipe collar sits on D1/R3")


def led_head(clr, extra=0.0):
    """Frustum: stem-window size at z = total_h - led_head_t, growing by the lip to the top."""
    return extrude(Plane.XY.offset(total_h - led_head_t) * Pos(lx, ly) * Circle((led_stem_d + clr) / 2),
                   amount=led_head_t + extra, taper=-led_taper)


lid += cyl(lx, ly, collar_z, collar_d, ceiling_z - collar_z + eps)
lid -= cyl(lx, ly, collar_z - eps, led_stem_d + led_pipe_clr, total_h - collar_z + 2 * eps)
lid -= led_head(led_pipe_clr, eps)
lightpipe = led_head(0.0) + cyl(lx, ly, pipe_z0, led_stem_d, total_h - led_head_t - pipe_z0 + eps)


def grow(f, d):
    """Offset a glyph face by d (negative shrinks). By hand: build123d 0.11's offset()
    crashes on glyphs with holes."""
    g = Face(f.outer_wire().offset_2d(d))
    for w in f.inner_wires():
        g = g.cut(Face(w.offset_2d(-d)))
    return g.faces() if hasattr(g, "faces") else [g]


def tracked_text(txt, size, tracking):
    """Text sketch, each glyph emboldened by lid_text_bold and shifted by `tracking` per
    glyph (glyphs sorted by X, one face each). The stroke check runs on the raw glyph: OCCT
    cannot reliably shrink an outline it has just grown."""
    sk = Text(txt, font=lid_font, font_size=size, font_style=FontStyle.BOLD)
    glyphs = []
    for i, f in enumerate(sorted(sk.faces(), key=lambda f: f.center().X)):
        # every stroke >= lid_text_min_stroke once grown: shrinking the raw glyph by half of
        # the rest must keep it one face
        try:
            n = len(grow(f, -(lid_text_min_stroke / 2 - lid_text_bold)))
        except Exception:   # the wire collapsed: stroke too thin
            n = 0
        check(n == 1, f"'{txt}' has strokes under {lid_text_min_stroke} mm (a glyph -> {n} faces)")
        glyphs += [Pos(i * tracking, 0) * x for x in grow(f, lid_text_bold)]
    return Sketch(glyphs)


lid_text = None            # the inlay: exactly the pocket volume, printed in filament 2
if lid_text_show:
    text_cut = None
    lines = [tracked_text(*t) for t in lid_texts]
    heights = [l.bounding_box().size.Y for l in lines]
    block_h = sum(heights) + lid_text_gap * (len(lines) - 1)
    band_y0, band_y1 = edge_r, tab_y0 - tab_slot
    led_seat_r = (led_stem_d + led_pipe_clr) / 2 + led_head_lip
    y = (band_y0 + ly - led_seat_r) / 2 + block_h / 2      # top of the block
    for (txt, *_), sk, h in zip(lid_texts, lines, heights):
        tb = sk.bounding_box()
        sk = Pos(outer_w / 2 - (tb.min.X + tb.max.X) / 2, y - h - tb.min.Y) * sk
        tb = sk.bounding_box()
        check(tb.min.X > edge_r + 1 and tb.max.X < outer_w - edge_r - 1, f"'{txt}' too wide ({tb.size.X:.1f}mm)")
        check(tb.max.Y < band_y1 - 1, f"'{txt}' hits the button tab")
        check(tb.min.Y > band_y0 + 0.5, f"'{txt}' runs into the top-edge chamfer")
        c = prism(sk, total_h - lid_text_depth, lid_text_depth + eps)
        text_cut = c if text_cut is None else text_cut + c
        y -= h + lid_text_gap
    rail_x1 = lx - led_seat_r - lid_rail_gap
    check(rail_x1 - lid_rail_end > 5, "rail too short")
    text_cut += box(lid_rail_end, ly - lid_rail_w / 2, total_h - lid_text_depth, rail_x1 - lid_rail_end, lid_rail_w, lid_text_depth + eps)
    lid_text = lid & text_cut
    lid -= text_cut

# --- BASE (the sliding floor) -----------------------------------------------------------------
base = (dovetail(floor_sk, 0, floor_t) + back_band) & envelope
# Rail under the free board edge, behind the front corner's rounding
rail = box(wall + slide_gap, wall + inner_r, floor_t - eps,
           bx(rail_x0) - wall - slide_gap, band_y - wall - inner_r + eps, pcb_z - clip_gap - floor_t + eps)
base += rail
# Latch arm: slot along its inner side and behind its free end
base -= box(latch_xa - latch_slot, latch_y1 - latch_l, -1, latch_slot, latch_l + latch_slot, floor_t + 2)
base -= box(latch_xa - latch_slot, latch_y1, -1, outer_w + 1 - latch_xa + latch_slot, latch_slot, floor_t + 2)
base_slide = base                      # without the catch: what must pass the shell freely
_far = latch_xe + latch_p + 1
latch_catch = (Pos(latch_p, 0, 0) * dovetail(floor_sk, 0, floor_t)) & prism(Polygon(
    (latch_xa, latch_y0), (latch_xe, latch_y0), (_far, latch_y0 + _far - latch_xe), (_far, latch_y1), (latch_xa, latch_y1),
    align=None), 0, floor_t)           # 45 deg lead-in ramp at the front, square back face
base += latch_catch
# "+" in the floor under the cell
for dx, dy in ((cell_mark_l, cell_mark_w), (cell_mark_w, cell_mark_l)):
    base -= box(bx(cell_c[0]) - dx / 2, by(cell_c[1]) - dy / 2, floor_t - cell_mark_d, dx, dy, cell_mark_d + eps)

# --- Assertions --------------------------------------------------------------------------------
for name, p in (("base", base), ("lid", lid), ("lightpipe", lightpipe)):
    check(p.is_valid, f"{name}: invalid solid")
    check(len(p.solids()) == 1, f"{name}: {len(p.solids())} separate bodies, something floats")
lb = lid.bounding_box()
check(abs(lb.size.X - outer_w) < 1e-3 and abs(lb.size.Y - outer_h) < 1e-3, "lid X/Y")
check(abs(lb.min.Z) < 1e-3 and abs(lb.max.Z - total_h) < 1e-3, "lid Z")
bb = base.bounding_box()
check(abs(bb.min.Z) < 1e-3 and abs(bb.max.Z - (pcb_z - clip_gap)) < 1e-3 and abs(bb.max.Y - outer_h) < 1e-3, "base")
if lid_text is not None:
    check(lid_text.is_valid and lid_text.volume > 1e-3, "lid text inlay: empty or invalid")
    tb = lid_text.bounding_box()
    check(abs(tb.max.Z - total_h) < 1e-3 and tb.min.Z > total_h - lid_text_depth - 1e-3, "inlay not in the pocket")
    check((lid & lid_text).volume < 1e-6, "inlay overlaps the lid")


def _open(solid, probe, what):
    v = (solid & probe).volume
    check(v < 1e-6, f"{what} blocked (probe volume {v:.3f})")


def _clear(solid, keep, what):
    v = (solid & keep).volume
    check(v < 1e-6, f"{what} collides (volume {v:.3f})")


# Keepouts: the board itself, the cell + clip below it, the parts above it
board_keep = prism(board_sk, pcb_z + eps, pcb_t - 2 * eps)
under_keep = (cyl(bx(cell_c[0]), by(cell_c[1]), pcb_z - cell_t - 0.1, cell_d + 1.0, cell_t + 0.1 - eps)
              + box(bx(cell_c[0]) - clip_l / 2 - 0.3, by(cell_c[1]) - clip_w / 2 - 0.3, pcb_z - clip_h + eps,
                    clip_l + 0.6, clip_w + 0.6, clip_strap_h)
              + box(bx(cell_c[0]) - clip_l / 2 - 0.3, by(cell_c[1]) - clip_leg_w / 2 - 0.3, pcb_z - clip_h + eps,
                    clip_l + 0.6, clip_leg_w + 0.6, clip_h - 2 * eps))
_clear(base + lid, board_keep, "board")
_clear(base + lid, under_keep, "cell + clip")
_clear(lid, box(bx(module[2]), by(module[1]), pcb_top_z, module[2] - module[0], module[3] - module[1], module[4] + 0.3), "module")
_clear(lid, box(bx(conn_c[0]) - conn_w / 2 - 0.3, by(conn_c[1]) - 2.5, pcb_top_z, conn_w + 0.6, 2.5 + conn_mouth + clr + 1, conn_h + 0.3), "connector")
_clear(lid, cyl(bx(led_c[0]), by(led_c[1]), pcb_top_z, 2.4, 0.9), "LED")
_clear(lid, cyl(bx(sw_c[0]), by(sw_c[1]), pcb_top_z, 3.6, sw_h + 0.1), "switch")
# the whole board top stays free up to the tallest part, except under the bosses and the nub
_free = prism(board_sk, pcb_top_z, max_part_h - eps) - cyl(tx, ty, pcb_top_z, nub_d + 0.5, max_part_h)
for cx, cy, d in bosses:
    _free -= cyl(bx(cx), by(cy), pcb_top_z - 1, d + 0.2, max_part_h + 2)
_free -= cyl(lx, ly, pcb_top_z - 1, collar_d + 0.2, max_part_h + 2)
_clear(lid, _free, "parts under the lid")
# Bosses press on bare board: clear of every top-side part (KiCad footprint bboxes, board coords)
top_parts = {"U1": (0.78, 0.17, 13.22, 17.32), "U2": (16.68, 5.67, 21.0, 14.32), "U3": (21.07, 5.67, 25.4, 14.32),
             "R1": (18.85, 3.43, 20.89, 4.57), "R2": (22.82, 3.43, 24.86, 4.57), "R3": (16.83, 24.38, 17.97, 26.42),
             "C1": (25.33, 14.37, 27.67, 19.64), "C2": (8.03, 19.73, 10.51, 20.87), "C3": (1.28, 17.21, 2.42, 19.69), "D1": (16.73, 26.78, 19.76, 28.43),
             "J1": (20.2, 21.98, 27.0, 27.54), "J2": (8.9, 21.98, 15.7, 27.54), "SW1": (3.38, 26.22, 8.62, 28.98),
             "Q1": (3.67, 19.34, 7.12, 23.25)}


def plan_gap(cx, cy, d, x0, y0, x1, y1):
    return ((max(x0 - cx, 0, cx - x1)) ** 2 + (max(y0 - cy, 0, cy - y1)) ** 2) ** 0.5 - d / 2


for cx, cy, d in bosses:
    for ref, bb in top_parts.items():
        gap = plan_gap(cx, cy, d, *bb)
        check(gap >= 0.2, f"boss at ({cx}, {cy}) within {gap:.2f} mm of {ref}")
# Light-pipe collar: clear of the tall parts in plan. J1/J2 by their bodies: the footprint bbox
# includes the flat side pads, which the collar passes over.
_tall = {r: bb for r, bb in top_parts.items() if r not in ("D1", "R3")}
for ref, c in (("J1", conn_c), ("J2", swd_c)):
    _tall[ref] = (c[0] - conn_w / 2, top_parts[ref][1], c[0] + conn_w / 2, top_parts[ref][3])
for ref, bb in _tall.items():
    gap = plan_gap(*led_c, collar_d, *bb)
    check(gap >= 0.2, f"light-pipe collar within {gap:.2f} mm of {ref}")
# J1 sits at board x 23.6, right of centre in KiCad: seen from above with the antenna at the
# front it is on the left. Catches a board mapped without the mirror.
check(conn_x0 + conn_open_w / 2 < outer_w / 2, "camera plug opening not on the left: board mirrored?")
# Button tab: the tip ends over the cavity (no wall under it to bottom out on), the nub sits on the tab
check(tab_y0 + tab_l <= by(board_h + clr) - 0.3, "button tab tip over the wall")
check(ty + nub_d / 2 <= tab_y0 + tab_l - 0.2, "nub overhangs the tab tip")
# Openings break through
_open(lid, box(conn_x0 + 0.2, band_y - 0.5, pcb_top_z + 0.1, conn_open_w - 0.4, wall + 1, conn_open_h - 0.3), "plug opening")
_open(lid, cyl(lx, ly, collar_z - 0.5, led_stem_d + led_pipe_clr - 0.1, total_h - collar_z + 1), "light-pipe window")
_clear(lid, lightpipe, "light pipe")
_open(lid, box(tab_x0 + tab_r, tab_y0 + tab_l + 0.1, ceiling_z - 0.5, tab_w - 2 * tab_r, tab_slot - 0.2, lid_top_t + 1), "button slot")
for x0 in (tab_x0 - tab_slot + 0.1, tab_x0 + tab_w + 0.1):
    _open(lid, box(x0, tab_y0 + 0.1, ceiling_z - 0.5, tab_slot - 0.2, tab_l - tab_r, lid_top_t + 1), "button slot side")
_clear(base, lid, "lid")
_open(base, box(bx(cell_c[0]) - cell_mark_l / 2 + 0.1, by(cell_c[1]) - cell_mark_w / 2 + 0.1, floor_t - cell_mark_d + 0.05,
                cell_mark_l - 0.2, cell_mark_w - 0.2, cell_mark_d), "+ mark in the floor")
# The board goes into the shell from below, up to the bosses
_open(lid, prism(board_sk, -1, pcb_top_z + 1 - eps), "board's way in from below")
# The floor slides out backwards: past the shell, and under the board, cell and clip
for d in (0.5, 1, 2, 4, 8, 14, 20, outer_h):
    moved = Pos(0, d, 0) * base_slide
    _clear(lid, moved, f"floor slid back {d} mm")
    _clear(board_keep + under_keep, moved, f"floor slid back {d} mm, under the board")
# Latch: the catch hits the window's back face on a pull, rides in the wall until the window
# (the arm bends), and its outer face shows in the window for a fingernail
check((lid & Pos(0, slide_gap + 0.1, 0) * latch_catch).volume > 1e-3, "latch catch does not hold against a pull")
check((lid & Pos(0, latch_catch_l + 2 * slide_gap + 0.5, 0) * latch_catch).volume > 1e-3, "latch catch does not bend the arm")
_clear(base, box(latch_xa - latch_slot + eps, latch_y1 - latch_l + 0.5, -1, latch_slot - 2 * eps, latch_l - 0.5, floor_t + 2), "latch slot")
_open(lid, box(latch_xe + latch_p + 0.1, latch_y0 + latch_p + 0.1, 0.2, outer_w, latch_y1 - latch_y0 - latch_p - 0.2, floor_t - 0.2), "latch window")
# The floor holds the board: the rail top and the plate top clip_gap under board and strap
_rail_top = box(wall + slide_gap + 0.1, by(5), pcb_z - clip_gap - 0.05, bx(rail_x0) - wall - slide_gap - 0.2, board_h - 10, 0.05 - eps)
check((base & _rail_top).volume > 0.9 * _rail_top.volume, "rail does not reach under the board")
check(abs(pcb_z - clip_h - clip_gap - floor_t) < 1e-6, "floor not under the clip strap")
check(bx(rail_x0) - (wall + clr) >= 1.0, "rail reaches less than 1 mm under the board")
# With the floor out the cell slides out of the clip backwards (it needs 14.5 mm to clear the strap)
_cx, _cy, _cz0 = bx(cell_c[0]), by(cell_c[1]), pcb_z - cell_t - 0.1
_open(lid, cyl(_cx, _cy, _cz0, cell_d, cell_t) + box(_cx - cell_d / 2, _cy, _cz0, cell_d, outer_h - _cy + 1, cell_t), "cell exit (floor out)")

# --- Export -------------------------------------------------------------------------------------
if __name__ == "__main__":
    out = Path(os.environ.get("OUTDIR", Path(__file__).resolve().parent / "stl"))
    out.mkdir(parents=True, exist_ok=True)
    parts = [("openrz67-nrf-base", base), ("openrz67-nrf-lid", lid), ("openrz67-nrf-lightpipe", lightpipe)]
    stale = out / "openrz67-nrf-lid-text.stl"
    if lid_text is not None:
        parts.append(("openrz67-nrf-lid-text", lid_text))
    elif stale.exists():
        stale.unlink()     # make_3mf.py would otherwise add an old inlay
    if os.environ.get("SLIDE_TEST", "false") == "true":
        # back end of both parts from the latch root: band, latch and grooves, the shell cut off above the plug opening
        _y = latch_y1 - latch_l - 1
        crop = box(-1, _y, -1, outer_w + 2, outer_h - _y + 1, pcb_top_z + conn_open_h + 2)
        parts += [("openrz67-nrf-slidetest-base", base & crop), ("openrz67-nrf-slidetest-lid", lid & crop)]
    for name, p in parts:
        export_stl(p, str(out / f"{name}.stl"), ascii_format=True)
        b = p.bounding_box()
        print(f"{name}: {b.size.X:.2f} x {b.size.Y:.2f} x {b.size.Z:.2f} mm, {p.volume / 1000:.2f} cm3")
    print(f"outer {outer_w:.1f} x {outer_h:.1f} x {total_h:.1f} mm; STLs in {out}")
