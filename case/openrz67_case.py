# /// script
# requires-python = ">=3.12"
# dependencies = ["build123d>=0.11.1"]
# ///
"""openrz67-trigger enclosure (build123d).

Two-part snap-fit box for the OpenRZ67 trigger PCB rev 2 (../pcb/kicad/), with
the battery cell lying flat UNDER the PCB, across the board. Dimensions come
from the KiCad board and its 3D models — see README.md for the design rationale.

Parts: base (tub), lid (telescoping ship-lap edge), lightpipe (clear filament).
Print orientation: base floor-down; lid upside-down (the debossed top text
faces the bed — filament change at Z = lid_text_depth colours the letters);
lightpipe head-down. No supports.

Coordinate system: case coords, origin at the outer box's front-left-bottom
corner, Y toward the back. bx()/by() map board coords into case coords. Board
coords here are the enclosure's: origin at the board's front-left corner, Y
toward the back — the KiCad board measures Y from the other long edge, so
board_y_here = 22 - kicad_y (the two mounting holes are (2,2) and (46,20)).

Run (exports STLs to stl/ and checks the assertions):
    uv run openrz67_case.py

Closure: snap-fit — a perimeter bead on the lid tongue clicks into a groove in
the base lip, holding the whole rim down (no screws, no heat-set inserts). The
PCB is located by two Ø1.7 pins in its real mounting holes and pressed down by
the lid bosses. Open the lid with a coin in the pry slot on the back wall.

Env overrides: PCB_T, LID_TEXT_SHOW (default true), and
SNAP_TEST=true to also export a cropped corner pair for tuning snap_bead /
lap_gap with a small test print before committing to the full box.
"""

import os
from pathlib import Path

from build123d import *

# --- Board (rev 2, from ../pcb/kicad/) ---------------------------------------
board_w, board_h, board_r = 48.0, 22.0, 2.0
pcb_t = float(os.environ.get("PCB_T", 1.6))
mount_holes = [(2, 2), (46, 20)]     # the real Ø2 holes (board coords)
hole_d = 2.0                         # their diameter (locating pins are hole_d - 0.3)
pcb_supports = [(2, 20), (46, 2)]    # solid posts in the other two corners
pcb_overhang_left = 2.0              # USB-C shell protrudes past the board edge
# TH solder tails below the PCB that would land on a post: [board_x, board_y, relief_dia].
# Rev 2 has none (the S3 and U4 tails hang over the cell gap, see under_clr).
th_keepouts = []
th_keepout_depth = 3.0
# Component bodies above the PCB the lid bosses must clear: (x0, y0, x1, y1)
comp_keepouts = [(0.0, 4.0, 8.0, 14.5)]   # USB-C connector body
# Under the PCB: BAT1 (S2B-PH-SM4-TB, bottom side, mouth toward the board centre)
# plus its PHR-2 plug and the wire exit, as one envelope (x0, y0, x1, y1), h_bat1 deep.
bat1_env, h_bat1 = (5.4, 13.6, 19.0, 21.5), 5.5
# THT tails under the PCB (S3 switch header, U4 camera connector), probed for clearance
tht_tails = [(27.71, 19.96), (29.71, 19.96)] + [(44.3, y) for y in (7.25, 9.75, 12.25, 14.75)]
tht_tail_len = 3.4

# --- PCB guide fins (both long edges; the cavity is wider than the board) ---
frame_clr, frame_proud, frame_cham = 0.2, 0.6, 0.4
frame_rib_l = 8.0
frame_ribs = [10, 45]                # fin centres in board-X (outside the cell's X span)

# --- Fit and walls ----------------------------------------------------------
clr, wall, floor_t, lid_top_t = 0.4, 2.4, 2.0, 2.0   # wall 2.4: the lap halves must slice as real perimeters

# --- Battery: the cell lies flat under the PCB, ACROSS the board ------------
batt_l, batt_w, batt_t = 31.0, 20.0, 6.0   # cell: long side (case Y), short side (case X), thickness
batt_clr = 0.5                             # fit clearance around the cell (pocket 32 x 21, as before)
batt_x0 = 20.0                             # cell's left edge in board-X: clear of BAT1 + plug + wire exit
under_clr = 3.7                            # air between cell top and PCB bottom: THT tails 3.4 + margin.
#                                            1.5 (a foam pad) if the S3/U4 tails are clipped flush.
rib_h, rib_t = 2.5, 1.6                    # low ribs on the floor that stop the cell sliding in X

# --- Vertical stack ---------------------------------------------------------
standoff_h = batt_t + under_clr      # PCB underside above the floor
h_relay, h_usbc, h_ph_plug, h_xh = 5.2, 3.3, 8.0, 6.1   # component heights above PCB
# h_ph_plug: PHR-2 plug + wire in the top-entry S3 header; h_xh: S4B-XH-A body

# --- Openings ---------------------------------------------------------------
usb_open_w, usb_open_h, usb_open_r = 11.0, 7.0, 2.0     # outer lip (stops the cable body)
usb_recess_w, usb_recess_h, usb_recess_r = 13.0, 9.0, 3.0
usb_recess_d = wall - 0.5            # keep a 0.5 lip so the plug still seats fully
led_pos = [(22.514, 20.186), (24.224, 20.186)]           # D3, D4
led_win_l, led_win_w, led_win_r = 6.0, 3.0, 1.2
led_head_lip, led_head_t, led_pipe_clr, led_pipe_gap = 1.0, 1.0, 0.2, 1.5
# U4 camera connector: side-entry S4B-XH-A on the board's right edge, mouth facing
# out; the 12.4 x 6.1 body protrudes 2.4 past the board edge INTO the wall, so the
# lid wall gets an opening the body sits in and the XHP plug passes through.
xh_w, xh_h, xh_clr = 12.4, 6.1, 0.4
xh_y = 22 - 11.0                     # connector centre, board-Y

# --- Slide switch SS12F15 (front wall, wired to S3) --------------------------
sw_x, sw_z = 22, 4.5
sw_slot_l, sw_slot_h = 10.65, 6.3
sw_body_l, sw_body_h, sw_body_w, sw_body_clr = 11.0, 7.0, 8.0, 0.4
sw_screw_pitch, sw_screw_d = 15.0, 2.4
sw_boss_d, sw_boss_h, sw_boss_pilot, sw_boss_gap = 5.0, 5.0, 1.5, 1.5
sw_plate_w, sw_plate_h, sw_plate_t, sw_plate_clr = 19.45, 5.75, 0.4, 0.3

# --- Snap closure --------------------------------------------------------------
snap_bead = 0.5    # how far the bead on the lid tongue sticks out
bead_h = 1.4       # bead height; the base groove is 0.5 taller and 0.25 deeper (lead-in)
holddown_d = 5.0   # lid bosses that press the PCB onto the posts
pin_proud = 0.8    # locating pins stand this far above the PCB top
pry_w, pry_d, pry_h = 12.0, 1.0, 1.2   # coin slot in the lid's lower edge, back wall

# --- Ship-lap edge / orientation mark / lid text -----------------------------
lap, lap_gap = 4.0, 0.15             # lap <= split_z - floor_t (lip must end above the floor)
orient_mark_x, orient_mark_w, orient_mark_d, orient_mark_h = 8.0, 2.5, 0.8, 9.0
# Debossed (pocket) text in the lid top, per the FDM rules: prints upside-down
# against the bed (crisp), and one filament change at Z = lid_text_depth colours
# the letters. The LED window sits near the back edge (the LEDs are on the board's
# back edge), so both lines go in front of it, centred on the light pipe's X.
# (text, cap size, Y offset from the LED row)
lid_text_show = os.environ.get("LID_TEXT_SHOW", "true") == "true"
lid_texts = [("OpenRZ67", 9.0, -10.5), ("Trigger", 7.0, -21.0)]
lid_text_depth = 0.6              # pocket depth = the colour-change height (3 x 0.2 layers)
eps = 0.01

# --- Derived ------------------------------------------------------------------
inner_w = board_w + 2 * clr + pcb_overhang_left
side_room = (batt_l + 2 * batt_clr - board_h) / 2   # cavity is this much wider than the board, each side
assert side_room >= clr, "cell narrower than the board: make side_room at least clr"
inner_h = board_h + 2 * side_room
inner_r = board_r + clr
outer_w, outer_h, outer_r = inner_w + 2 * wall, inner_h + 2 * wall, inner_r + wall
standoff_d = 4.0
comp_clr = max(h_relay, h_ph_plug, h_xh) + 1.0
assert standoff_h >= h_bat1 + 0.5, "BAT1 body + plug do not fit under the PCB"
assert under_clr >= tht_tail_len + 0.2 or under_clr >= 1.5, "under_clr too small"
assert batt_x0 >= bat1_env[2] + batt_clr, "cell overlaps the BAT1 plug / wire exit"
right_posts_x = min(x for x, _ in pcb_supports + mount_holes if x > board_w / 2)
assert batt_x0 + batt_w + batt_clr + rib_t <= right_posts_x - standoff_d / 2, \
    "cell / right rib run into the right-hand posts"
pcb_z = floor_t + standoff_h
pcb_top_z = pcb_z + pcb_t
split_z = pcb_top_z
lid_h = comp_clr + lid_top_t
total_h = split_z + lid_h
bead_z = split_z - lap + 2.2            # bead centre height on the tongue
assert lap <= split_z - floor_t, "lid lip would reach the floor"
assert split_z - lap < bead_z - (bead_h + 0.5) / 2 and bead_z + (bead_h + 0.5) / 2 < split_z, \
    "snap bead/groove outside the lap zone"
# Printability: the lap halves must slice as real perimeters (0.4 mm nozzle:
# one line 0.42 mm, a two-line freestanding wall 0.87 mm)
assert wall / 2 - (snap_bead + 0.25) >= 0.42, "base lip behind the snap groove under one line"
assert wall / 2 - lap_gap >= 0.87, "lid tongue under two perimeter lines"


def bx(x):
    return wall + clr + pcb_overhang_left + x


def by(y):
    return wall + side_room + y


led_cx = (bx(led_pos[0][0]) + bx(led_pos[1][0])) / 2
led_cy = (by(led_pos[0][1]) + by(led_pos[1][1])) / 2
sx = bx(sw_x)                        # switch centre, case-X
swz = split_z + sw_z                 # switch centre, case-Z

# --- Helpers -------------------------------------------------------------------
def rrect(w, h, r, x0=0.0, y0=0.0):
    """Rounded rectangle with min-corner at (x0, y0)."""
    return Pos(x0 + w / 2, y0 + h / 2) * RectangleRounded(w, h, r)


def prism(sk, z0, h):
    return extrude(Plane.XY.offset(z0) * sk, amount=h)


def xprism(sk, x0, length):
    """Extrude a YZ-plane sketch (local x -> case Y, local y -> case Z) along +X."""
    return extrude(Plane.YZ.offset(x0) * sk, amount=length)


def box(x0, y0, z0, dx, dy, dz):
    return Pos(x0, y0, z0) * Box(dx, dy, dz, align=Align.MIN)


def cyl(x, y, z0, d, h):
    return Pos(x, y, z0) * Cylinder(d / 2, h, align=(Align.CENTER, Align.CENTER, Align.MIN))


def ycyl(x, y0, z, d, length):
    """Cylinder along +Y (for the switch screws)."""
    return Pos(x, y0, z) * Rot(-90, 0, 0) * Cylinder(
        d / 2, length, align=(Align.CENTER, Align.CENTER, Align.MIN))


outer_sk = rrect(outer_w, outer_h, outer_r)
inner_sk = rrect(inner_w, inner_h, inner_r, wall, wall)
mid_sk = rrect(inner_w + wall, inner_h + wall, inner_r + wall / 2, wall / 2, wall / 2)

# --- Shared cuts -----------------------------------------------------------------
# USB-C (left wall): 11x7 R2 through-cut whose outer edge is the cable-body lip,
# plus a 13x9x1.5 inner recess so the receptacle shell pokes into the wall and the
# mouth reaches the outer face. Straddles the split -> cut from BOTH base and lid.
usb_cy, usb_zc = by(22 - 13.259), pcb_top_z + h_usbc / 2
cut_usb = xprism(Pos(usb_cy, usb_zc) * RectangleRounded(usb_open_w, usb_open_h, usb_open_r),
                 -1, wall + 2)
cut_usb += xprism(Pos(usb_cy, usb_zc) * RectangleRounded(usb_recess_w, usb_recess_h, usb_recess_r),
                  wall - usb_recess_d, usb_recess_d + 1)

# XH camera connector (right wall): opening for the S4B-XH-A body that protrudes
# into the wall, from the board top up; the XHP plug goes in from outside. Lid-only
# cut (everything is above the split).
xh_cy = by(xh_y)
cut_xh = box(outer_w - wall - 1, xh_cy - xh_w / 2 - xh_clr, split_z - eps,
             wall + 2, xh_w + 2 * xh_clr, xh_h + xh_clr + eps)


def orient_mark_rib(z0):
    """Half of the front-wall orientation rib (base: lower, lid: upper). The halves
    line up only when the lid is on the right way around. Overlaps 0.2 into the wall."""
    return box(bx(orient_mark_x) - orient_mark_w / 2, -orient_mark_d, z0,
               orient_mark_w, orient_mark_d + 0.2, orient_mark_h / 2)


def fin(cx, front):
    """PCB guide fin: a block from the inner wall face to frame_clr off the board's
    long edge, floor to frame_proud above the board top, with a chamfered lead-in on
    the inner top edge. Front (low Y) or back (high Y) edge. Starts at the inner wall
    face, so it never meets the lid tongue (no notches needed)."""
    if front:
        y_wall, y_cap, s = wall, by(0) - frame_clr, 1
    else:
        y_wall, y_cap, s = wall + inner_h, by(board_h) + frame_clr, -1
    top = split_z + frame_proud
    pts = [(y_wall, floor_t), (y_cap, floor_t), (y_cap, top - frame_cham),
           (y_cap - s * frame_cham, top), (y_wall, top)]
    # keep the polygon counter-clockwise, or the extrusion runs toward -X
    profile = Polygon(*(pts if s > 0 else pts[::-1]), align=None)
    return xprism(profile, bx(cx) - frame_rib_l / 2, frame_rib_l)


# --- BASE ---------------------------------------------------------------------------
base = prism(outer_sk, 0, split_z)
base -= prism(inner_sk, floor_t, total_h)                    # cavity
base -= prism(mid_sk, split_z - lap, lap + 1)                # rabbet: outer wall half only

# Posts at the mount holes with Ø1.7 locating pins into the real PCB holes, plus
# solid posts in the other corners
for hx, hy in mount_holes:
    base += cyl(bx(hx), by(hy), floor_t, standoff_d, standoff_h)
    base += cyl(bx(hx), by(hy), pcb_z - eps, hole_d - 0.3, pcb_t + pin_proud + eps)
for px, py in pcb_supports:
    base += cyl(bx(px), by(py), floor_t, standoff_d, standoff_h)

# Battery pocket: the cell lies on the floor across the whole cavity (the walls
# hold it in Y); two low ribs stop it sliding in X. The left rib covers the front
# half only, so the battery lead can loop from the BAT1 plug (back-left, under the
# board) out into the back pocket and along it to the cell's tab end.
cell_x0, cell_x1 = bx(batt_x0 - batt_clr), bx(batt_x0 + batt_w + batt_clr)
base += box(cell_x0 - rib_t, wall, floor_t, rib_t, by(10) - wall, rib_h)
base += box(cell_x1, wall, floor_t, rib_t, inner_h, rib_h)

# Guide fins on both long edges (the cavity is side_room wider than the board)
for cx in frame_ribs:
    base += fin(cx, front=True)
    base += fin(cx, front=False)

base += orient_mark_rib(split_z - orient_mark_h / 2)

# Snap groove in the base lip: slightly taller and deeper than the bead (lead-in).
# The cut's top edges are chamfered so the groove ceiling prints as a ~45° ramp
# instead of an unsupported 0.75 mm overhang (base prints floor-down).
groove = prism(offset(mid_sk, snap_bead + 0.25) - offset(mid_sk, -lap_gap - 0.25),
               bead_z - (bead_h + 0.5) / 2, bead_h + 0.5)
base -= chamfer(groove.edges().group_by(Axis.Z)[-1], 0.55)
# TH solder-tail pockets in the post tops (none on rev 2; kept for other boards)
for kx, ky, kd in th_keepouts:
    base -= cyl(bx(kx), by(ky), pcb_z - th_keepout_depth, kd, th_keepout_depth + eps)
base -= cut_usb

# --- LID ----------------------------------------------------------------------------
lid = prism(outer_sk, split_z, lid_h)                        # top + walls
lid += prism(offset(mid_sk, -lap_gap) - inner_sk, split_z - lap, lap)   # tongue
# Snap bead on the tongue, chamfered top and bottom: the lid prints upside-down,
# so an unchamfered bead starts as a 0.5 mm ledge in mid-air; the ramps also ease
# click-in and coin-open. (The inner edges' chamfers end up buried in the tongue.)
bead = prism(offset(mid_sk, -lap_gap + snap_bead) - offset(mid_sk, -lap_gap - 0.4),
             bead_z - bead_h / 2, bead_h)
bead_ends = bead.edges().group_by(Axis.Z)
lid += chamfer(bead_ends[0] + bead_ends[-1], 0.4)
lid -= prism(inner_sk, split_z - lap - eps, comp_clr + lap + eps)       # cavity

# Switch screw bosses: rectangular pillars from sw_boss_gap above the PCB up into
# the ceiling; front face at the inner wall plane so the screw clamps bracket ->
# wall -> boss in compression. (They now stand over the side pocket, not the board.)
for s in (-1, 1):
    lid += box(sx + s * sw_screw_pitch / 2 - sw_boss_d / 2, wall, split_z + sw_boss_gap,
               sw_boss_d, sw_boss_h, (total_h - lid_top_t + 1) - (split_z + sw_boss_gap))
# Hold-down bosses over the mount holes (press the PCB onto the posts)
for hx, hy in mount_holes:
    lid += cyl(bx(hx), by(hy), pcb_top_z, holddown_d, (total_h - lid_top_t) - pcb_top_z)
lid += orient_mark_rib(split_z)

lid -= cut_usb
lid -= cut_xh
# LED light-pipe hole: stem window through the top plate + head recess (flush top)
lid -= prism(Pos(led_cx, led_cy) * RectangleRounded(
    led_win_l + led_pipe_clr, led_win_w + led_pipe_clr, led_win_r),
    total_h - lid_top_t - eps, lid_top_t + 2 * eps)
lid -= prism(Pos(led_cx, led_cy) * RectangleRounded(
    led_win_l + 2 * led_head_lip + led_pipe_clr, led_win_w + 2 * led_head_lip + led_pipe_clr,
    led_win_r + led_head_lip),
    total_h - led_head_t, led_head_t + eps)
# Switch: actuator slot, flush bracket recess in the outer wall, screw holes,
# and the body-envelope keepout (the bosses give way over the body only)
lid -= box(sx - sw_slot_l / 2, -1, swz - sw_slot_h / 2, sw_slot_l, wall + 2, sw_slot_h)
lid -= box(sx - (sw_plate_w + sw_plate_clr) / 2, -0.5, swz - (sw_plate_h + sw_plate_clr) / 2,
           sw_plate_w + sw_plate_clr, sw_plate_t + 0.5, sw_plate_h + sw_plate_clr)
for s in (-1, 1):
    x = sx + s * sw_screw_pitch / 2
    lid -= ycyl(x, -1, swz, sw_screw_d, wall + 1)            # clearance through wall
    lid -= ycyl(x, wall - eps, swz, sw_boss_pilot, sw_boss_h + eps)  # pilot into boss
lid -= box(sx - sw_body_l / 2 - sw_body_clr, wall, swz - sw_body_h / 2 - sw_body_clr,
           sw_body_l + 2 * sw_body_clr, sw_body_w + eps, sw_body_h + 2 * sw_body_clr)
# Blind recesses in the hold-down bosses for the locating pins (not through the
# top plate)
for hx, hy in mount_holes:
    lid -= cyl(bx(hx), by(hy), pcb_top_z - eps, 2.6, pin_proud + 0.7 + eps)
# Coin/fingernail pry slot in the lid's lower edge, back wall centre. 1mm deep
# leaves 0.35 skin before the tongue.
lid -= box(outer_w / 2 - pry_w / 2, outer_h - pry_d, split_z - eps,
           pry_w, pry_d + eps, pry_h + eps)
# Component keepouts (USB-C body vs the (2,2) hold-down boss)
for x0, y0, x1, y1 in comp_keepouts:
    lid -= box(bx(x0), by(y0), pcb_top_z - eps,
               x1 - x0, y1 - y0, (total_h - lid_top_t) - pcb_top_z + 2 * eps)

if lid_text_show:
    # Each line must stay on its own side of the LED head recess (half-height 2.6 in Y)
    recess_half = (led_win_w + 2 * led_head_lip + led_pipe_clr) / 2
    for txt, size, dy in lid_texts:
        sk = Pos(led_cx, led_cy + dy) * Text(txt, font_size=size, font_style=FontStyle.BOLD)
        tb = sk.bounding_box()
        assert tb.min.X > 3 and tb.max.X < outer_w - 3, f"'{txt}' too wide ({tb.size.X:.1f}mm)"
        assert (tb.min.Y > led_cy + recess_half + 1 if dy > 0
                else tb.max.Y < led_cy - recess_half - 1), f"'{txt}' hits the LED recess"
        assert tb.min.Y > 2 and tb.max.Y < outer_h - 2, f"'{txt}' off the top face"
        lid -= prism(sk, total_h - lid_text_depth, lid_text_depth + eps)

# --- LIGHT PIPE (clear filament, inserted from above as a top hat) --------------------
lightpipe = prism(Pos(led_cx, led_cy) * RectangleRounded(
    led_win_l + 2 * led_head_lip, led_win_w + 2 * led_head_lip, led_win_r + led_head_lip),
    total_h - led_head_t, led_head_t)
lightpipe += prism(Pos(led_cx, led_cy) * RectangleRounded(led_win_l, led_win_w, led_win_r),
                   pcb_top_z + led_pipe_gap, (total_h - led_head_t) - (pcb_top_z + led_pipe_gap) + eps)

# --- Assertions ------------------------------------------------------------------------
for name, p in (("base", base), ("lid", lid), ("lightpipe", lightpipe)):
    assert p.is_valid, f"{name}: invalid solid"

bb = base.bounding_box()
assert abs(bb.size.X - outer_w) < 1e-3, f"base X {bb.size.X}"
assert abs(bb.max.Y - outer_h) < 1e-3 and abs(bb.min.Y + orient_mark_d) < 1e-3, "base Y"
assert abs(bb.max.Z - (pcb_z + pcb_t + pin_proud)) < 1e-3, f"base Z {bb.max.Z}"  # pins on top
lb = lid.bounding_box()
assert abs(lb.min.Z - (split_z - lap)) < 1e-3 and abs(lb.max.Z - total_h) < 1e-3, "lid Z"

# Holes must actually break through (a wrong axis gives the same volume):
def _open(solid, probe, what):
    v = (solid & probe).volume
    assert v < 1e-6, f"{what} blocked (probe volume {v:.3f})"

_open(base + lid, box(-0.5, usb_cy - 4, usb_zc - 2.5, wall + 1, 8, 5), "USB opening")
_open(base + lid, box(outer_w - wall - 0.5, xh_cy - xh_w / 2, split_z + eps, wall + 1, xh_w, xh_h),
      "XH connector opening through the right wall")
_open(lid, box(led_cx - 2, led_cy - 1, total_h - lid_top_t + eps, 4, 2, lid_top_t), "LED window")
for hx, hy in mount_holes:
    _open(lid, cyl(bx(hx), by(hy), pcb_top_z + eps, 2.0, pin_proud), "locating-pin recess")
for s in (-1, 1):
    _open(lid, ycyl(sx + s * sw_screw_pitch / 2, eps, swz, 1.0, wall - 2 * eps), "switch screw")
# Fit-critical keepouts stay open (enforced as geometry, not eyeballed in preview):
_open(lid, box(sx - sw_body_l / 2, wall + eps, swz - sw_body_h / 2,
               sw_body_l, sw_body_w - 2 * eps, sw_body_h), "switch body envelope")
_open(lid, box(bx(0.5), by(4.5), pcb_top_z + eps, 7, 9.5, 2), "USB-C body keepout")
# Under the PCB: the cell pocket, BAT1 + plug + wire exit, the THT tails, and the
# lead's route from the plug into the back pocket
_open(base, box(cell_x0 + eps, wall + eps, floor_t + eps, cell_x1 - cell_x0 - 2 * eps,
                inner_h - 2 * eps, batt_t), "battery cell pocket")
_open(base, box(bx(bat1_env[0]), by(bat1_env[1]), pcb_z - h_bat1, bat1_env[2] - bat1_env[0],
                bat1_env[3] - bat1_env[1], h_bat1 - eps), "BAT1 + plug envelope under the PCB")
for tx, ty in tht_tails:
    _open(base, cyl(bx(tx), by(ty), pcb_z - tht_tail_len, 1.6, tht_tail_len - eps),
          f"THT tail at board ({tx}, {ty})")
_open(base, box(bx(bat1_env[2]) - 3, by(bat1_env[1]), floor_t + rib_h + eps,
                bx(batt_x0 + batt_w) - bx(bat1_env[2]) + 3, wall + inner_h - by(bat1_env[1]) - eps,
                pcb_z - floor_t - rib_h - 2 * eps), "battery lead route to the back pocket")

# --- Export -------------------------------------------------------------------------------
if __name__ == "__main__":
    # Assembled preview in VS Code's OCP CAD Viewer, when it is open (port 3939).
    # Toggle part visibility in the viewer tree; show() does nothing/raises without it.
    try:
        from ocp_vscode import show
        show(base, lid, lightpipe, names=["base", "lid", "lightpipe"])
    except Exception as exc:
        print(f"OCP-viewer utilgjengelig ({type(exc).__name__})")
    out = Path(os.environ.get("OUTDIR", Path(__file__).resolve().parent / "stl"))
    out.mkdir(parents=True, exist_ok=True)
    parts = [("openrz67-base", base), ("openrz67-lid", lid), ("openrz67-lightpipe", lightpipe)]
    if os.environ.get("SNAP_TEST", "false") == "true":
        # Front-left corner crop of base + lid: one snap corner, a guide fin, a
        # locating pin and its boss. Print these first to tune snap_bead / lap_gap.
        crop = box(-2, -2, -1, 24, 24, total_h + 2)
        parts += [("openrz67-snaptest-base", base & crop), ("openrz67-snaptest-lid", lid & crop)]
    for name, p in parts:
        export_stl(p, str(out / f"{name}.stl"), ascii_format=True)  # make_3mf.py parses ASCII
        b = p.bounding_box()
        print(f"{name}: {b.size.X:.2f} x {b.size.Y:.2f} x {b.size.Z:.2f} mm, "
              f"{p.volume / 1000:.2f} cm3")
    print(f"STLs in {out}")
