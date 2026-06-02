/* =====================================================================
 * openrz67-trigger — parametric enclosure (OpenSCAD)
 * ---------------------------------------------------------------------
 * Dimensions taken from the PCB deliverables (2025-09-23):
 *   - Board outline:  Gerber_BoardOutlineLayer.GKO  -> 48.0 x 22.0 mm, R2.0
 *   - Mount holes:    Drill_PTH_Through.DRL (T06)   -> 2x Ø2.0 on a diagonal
 *   - Component placement: PickAndPlace_..._2025-09-23.csv
 *   - PCB thickness:  1.6mm (as actually ordered)  -> pcb_t = 1.6
 *
 * Setup (chosen with the user):
 *   - Snap-fit (clip) lid via ship-lap joint + perimeter bead
 *   - Battery (250mAh LiPo) UNDER the PCB; the board is raised on standoffs
 *   - Openings: USB-C (left), XH camera plug (right), LED window (top),
 *               slot + holder for the SS12F15 slide switch
 *
 * Coordinate system: board coordinates with origin BOTTOM-LEFT, Y up.
 * Pick&place uses origin top-left (Y down); converted as
 *   case_Y = 22 + midY.  bx()/by() move board coords into the case frame.
 * ===================================================================== */

/* [Render] */
// What to generate
part = "preview";   // [preview, base, lid, lightpipe, both]
// Show PCB + rough component blocks in preview (visual only)
show_pcb = true;

/* [Board] */
board_w = 48.0;     // X
board_h = 22.0;     // Y
board_r = 2.0;      // corner radius
pcb_t   = 1.6;      // PCB thickness (as ordered; adjust if your board differs)
hole_d  = 2.0;      // mount holes
// mount holes in board coords [x, y] (Y up):
mount_holes = [[2, 2], [46, 20]];
// The USB-C shell sticks out ~2mm past the PCB's left short edge (the connector
// "mouth" protrudes beyond the board edge). The case's left side is extended to
// match: the cavity becomes inner_w wide, and the PCB is centred toward the RIGHT
// short edge (normal clr), leaving the whole overhang_left + clr as clearance on
// the LEFT for the USB shell.
pcb_overhang_left = 2.0;
// Solid support posts on the base in the corners WHERE THERE IS NO SCREW HOLE.
// Hold the PCB evenly in all four corners without threading against a screw.
pcb_supports = [[2, 20], [46, 2]];
// Through-hole solder joints that stick DOWN below the PCB. Standoffs/support posts
// that come too close to these get a pocket cut in their top so they don't press
// against the solder. [board_x, board_y, clearance_diameter]. (BAT1 = TH PH connector
// right next to the support post on the USB side; the solder collided otherwise.)
th_keepouts = [
    [3.275, 16.189, 3.2],   // BAT1 pin 1
    [3.275, 18.189, 3.2],   // BAT1 pin 2
];
th_keepout_depth = 3.0;     // how far the solder/pin tail sticks down below the PCB
// Component bodies ABOVE the PCB that lid bosses (hold-downs/switch bosses) must avoid.
// [board_x0, board_y0, board_x1, board_y1] - rectangle in board coords, full height above
// the PCB. (USB-C connector: near edge ~board-Y 4.8; the Ø6 hold-down at hole (2,2) reached
// to Y 5 and collided. The clearance cuts the boss back to board-Y 4.0 on that side, like
// last year's design where the corner boss was small and tucked into the corner.)
comp_keepouts = [
    [0.0, 4.0, 8.0, 14.5],    // USB-C connector body (left short edge)
];

/* [PCB frame (guide fins)] */
// Guide fins along the FRONT and BACK edges that the PCB is set down between, so it isn't
// pushed off the standoffs before the screws are in. There is only ~0.4mm clearance beside
// the board, too thin for a printable fin. But the ship-lap has a ~1mm TONGUE CHANNEL just
// outside the wall line: the fin stands in it, beside the board edge, and a small NOTCH is
// cut in the lid tongue where the fins stand so they clear. Since the tongue is already gone
// there, the fin's anchored side is run all the way OUT to the outer wall face, so it merges
// with the outer wall across the full lap-zone height (a wall-backed rib, not a free blade).
// The fins are NEVER above the board (they hit no components). Open at the corners + the
// left/right short edges (USB-C / XH cable).
pcb_frame   = true;
frame_clr   = 0.2;     // clearance between the fin's catching face and the board edge (drop-in)
frame_proud = 0.6;     // thin lead-in lip above the board top (toward the ceiling + alignment)
frame_cham  = 0.4;     // chamfer on the lip
frame_rib_l = 8.0;     // fin length along the edge
frame_fin_reach = 2.0; // how far the anchored side reaches toward the outer wall; = wall fully
                       // embeds the fin into the wall for the full lap-zone height (stiffness).
                       // Clamped to the outer face, so larger values can't poke outside.
frame_notch_clr = 0.3; // clearance where the lid tongue is cut away around the fin
// Fin centre in board-X (bx() converts). Front = low Y, back = high Y.
frame_ribs_front = [10, 38];
frame_ribs_back  = [10, 38];

/* [Fit and walls] */
clr       = 0.4;    // clearance between PCB edge and inner wall
wall      = 2.0;    // side wall
floor_t   = 2.0;    // floor
lid_top_t = 2.0;    // top plate

/* [Vertical stack] */
batt_t   = 6.0;     // LiPo thickness (under the PCB)
batt_gap = 1.0;     // clearance above the battery
// Component heights above the PCB top (datasheet/estimate - adjust as needed):
h_relay  = 5.2;     // G6K-2F-Y relays (K1, K2)
h_xh     = 8.0;     // JST XH 4-pin camera plug (U4)  <- tallest
h_usbc   = 3.3;     // USB-C 16P 2MD (USB1)
h_ph     = 6.0;     // JST PH 2-pin (BAT1, S3)

/* [Battery bay] */
batt_w   = 31;      // footprint X (~20% larger than the previous 26)
batt_l   = 20;      // footprint Y (the narrow dimension; +1mm for a slightly roomier fit)
batt_pos = [8.5, 1.0]; // min-corner in board coords (centred; clear of USB1/U4/standoffs)
rib_h    = 2.5;     // height of the retaining ribs

/* [Battery cable riser] */
// The battery lives under the PCB; its cable must come up to BAT1 on the PCB top, in the
// back-left corner by the left wall (BAT1 at board ~(3.3, 17)). The PCB-to-wall gap there is
// only ~2.4mm and the ship-lap tongue closes down into it, so the cable can't be routed AND
// the lid closed. This carves a rounded vertical channel at that corner from the floor up
// into the lid: it scallops the inner wall by batt_cable_depth (leaving wall - depth ~= 1mm)
// and locally notches the lid tongue, giving the cable a protected path through the seam.
// Cut from BOTH base and lid. Sits between the USB-C recess (front) and the back corner.
batt_cable       = true;
batt_cable_y     = 17.6;  // board-Y where the channel centre meets the LEFT wall (by BAT1)
batt_cable_r     = 1.6;   // channel radius (cable conduit; ~3.2mm wide)
batt_cable_depth = 1.0;   // how far it scallops INTO the inner wall (wall - this stays >= ~1mm)

/* [Openings] */
// USB-C (matches last year's design from the STEP file):
//   - Outer face: 11x7 through-cut. This edge IS the lip that stops the
//     cable body/overmould (anything > 11x7) from entering.
//   - Inside (toward the cavity): a 13x9 recess that eats into the last
//     1.5mm of the wall. Lets the USB-C receptacle body/shell poke a little
//     into the wall and brings the connector mouth near the outer face.
//   - The metal plug itself (8.34x2.56) passes 11x7 with room to spare.
usb_open_w   = 11.0;        // along Y - lip width
usb_open_h   = 7.0;         // along Z - lip height
usb_open_r   = 2.0;         // corner radius of the outer lip (last year's STEP: R2)
usb_chamfer  = 0.0;         // 45-degree chamfer (lead-in) on the mouth, 0 = off (matches last year's clean face)
usb_recess   = true;        // inner recess on the cavity side of the wall
usb_recess_w = 13.0;        // width inside (matches last year's 13x9)
usb_recess_h = 9.0;         // height inside
usb_recess_d = 1.5;         // depth in from the inner wall (must be < wall)
usb_recess_r = 3.0;         // corner radius of the inner recess (last year's STEP: R3)
/* [LED light pipe] */
// D3 (red) and D4 (blue) are SMD LEDs on the PCB top that shine STRAIGHT UP, ~8mm below
// the lid. A separate CLEAR light-pipe insert is guided down to just above the LEDs and
// catches the light into two crisp dots on the top face. The lid itself prints opaque; only
// the light pipe prints in clear filament and is inserted from above (top hat in a recess).
led_show  = true;
// LED positions in board coords (from pick&place: D3, D4 - 1.71mm apart):
led_pos   = [[22.514, 20.186], [24.224, 20.186]];
led_win_l = 6.0;     // optical window length (X, along the LED row) - covers both
led_win_w = 3.0;     // window width (Y)
led_win_r = 1.2;     // corner radius
led_head_lip = 1.0;  // how much wider the head/flange is than the stem (sits in the recess)
led_head_t   = 1.0;  // head thickness = depth of the top recess
led_pipe_clr = 0.2;  // clearance between light pipe and hole/recess in the lid
led_pipe_gap = 1.5;  // air gap between light-pipe bottom and LED top

/* [Camera plug cable exit (XH)] */
// The XH connector is plugged in BEFORE assembly and stays inside the enclosure;
// only the thin wires are routed out through this small grommet/port.
cable_port_w = 5.0;         // width (along Y)  -- set == _h for a round hole
cable_port_h = 2.6;         // height (along Z)
cable_port_z = 3.0;         // centre height above the PCB top

/* [Slide switch SS12F15 (wired to S3)] */
sw_enable = true;
sw_wall   = "front";  // [front, back]  which long side. FRONT (low Y) chosen because the
                      // back wall is blocked by S3 (connector) + the K2 relay; only the
                      // front has room for the 15mm boss spacing centred. The wire from S3
                      // is routed across the board to the switch.
sw_x      = 22;       // board-coord X. Case centre is 23; shifted 1mm left so the right
                      // boss clears the K1 relay (board-X 33.16) by ~1.2mm. Practically centred.
sw_z      = 4.5;      // centre height above the PCB top
// Actuator opening (through the wall). Measured from the switch: 10.65 x 6.3 mm.
sw_slot_l = 10.65;    // along the wall (X, travel + actuator)
sw_slot_h = 6.3;      // height (Z)
sw_body_w = 8.0;      // how far the switch body sticks in from the wall (clearance)
// Screw mount: the ears sit against the inner wall, screws go in from outside -> through
// the wall -> thread into bosses on the inside (the ears are Ø2.2 clearance holes in the
// STEP file).
sw_screw        = true;
sw_screw_pitch  = 15.0;  // centre spacing between the ears (from STEP: X = +-7.5)
sw_screw_d      = 2.4;   // clearance hole through the wall (M2)
sw_screw_dz     = 0.0;   // height offset from the slot centre
sw_boss_d       = 5.0;   // screw boss on the inner wall
sw_boss_h       = 5.0;   // how far the boss sticks into the case
sw_boss_pilot   = 1.5;   // pilot hole (M2 self-tapping)
// Mounting-plate recess: the switch's flat bracket (sw_plate_w x sw_plate_h, sw_plate_t
// thick, carrying the Ø2.2 screw holes + the actuator opening) mounts on the OUTSIDE of the
// case. A pocket sw_plate_t deep is carved into the OUTER wall over the plate footprint so
// the plate is FLUSH with the outside. Screws pass from outside through the bracket and the
// wall and thread into the bosses inside; the inner wall behind the bracket stays solid, so
// the screw clamps bracket -> wall -> boss in compression (no free-standing cantilever).
sw_plate_recess = true;
sw_plate_w      = 19.45;  // bracket width  (X, along the wall) - from the switch
sw_plate_h      = 5.75;   // bracket height (Z)
sw_plate_t      = 0.4;    // bracket thickness = recess depth (sits flush)
sw_plate_clr    = 0.3;    // clearance added around the bracket in the pocket (fit)
// The switch bosses' footprint (Ø5, X14.5 & X29.5, board-Y 0.2-5.2) sits OVER densely
// packed components along the front edge: Q1 (SOT-23, ~1.1mm), C15/C26 (0603, ~0.9mm)
// + 0402s. The boss bottom must clear the tallest, otherwise the lid is lifted. (Was 0.5mm
// -> Q1/C15 hit -> ~0.5mm gap. 1.5mm gives ~0.4mm margin over the SOT-23.)
sw_boss_gap     = 1.5;   // air between boss bottom and PCB top (clears the components beneath)

/* [Closure] */
// How the lid is held shut. Both keep the tongue-in-groove edge for alignment.
closure = "screw";    // [screw, snap]

/* [Screws (closure=screw)] */
// 2 screws from above through the real mount holes. Screw: countersunk head in the
// lid -> clearance through the lid boss -> through the PCB Ø2 hole -> down into the base post.
screw_d      = 2.0;   // M2. Use 1.6 for roomier clearance through the PCB hole.
screw_engage = 5.0;   // (self-tap only) thread length down into the post
head_d       = 4.0;   // counterbore for the screw head (M2 socket ~3.8)
head_h       = 1.8;   // depth of the counterbore
boss_d_screw = 6.0;   // diameter of the lid boss (hold-down) in screw mode
screw_clear  = screw_d + 0.4;   // clearance hole in the lid boss + top

// How the screw is anchored in the BASE post:
//   "heatset" - brass insert melted into the top of the post (strongest, reusable)
//   "selftap" - self-tapping screw straight into plastic (simplest, but strips on reuse)
screw_anchor = "heatset";   // [heatset, selftap]
// Heat-set M2 (check AGAINST YOUR inserts - varies between brands!):
heatset_hole_d = 2.6;   // hole diameter for the insert (the brass displaces plastic into the knurl)
heatset_depth  = 4.0;   // insert length / how deep the hole is bored from the post top
heatset_boss_d = 6.0;   // post diameter at the heat-set (needs ~1.7mm wall around the insert)
// Self-tapping:
screw_pilot  = screw_d - 0.5;   // pilot hole for a self-tapping screw in plastic
// derived: effective post diameter + hole diameter/depth from the post top
anchor_boss_d = (screw_anchor == "heatset") ? heatset_boss_d : standoff_d;
anchor_hole_d = (screw_anchor == "heatset") ? heatset_hole_d : screw_pilot;
anchor_hole_h = (screw_anchor == "heatset") ? heatset_depth  : screw_engage;

/* [Snap-fit (closure=snap)] */
lap       = 6.0;      // overlap between base and lid
lap_gap   = 0.15;     // sliding clearance in the joint
snap_bead = 0.5;      // how much the bead sticks out
bead_h    = 1.4;      // bead height

/* [Orientation mark] */
// Vertical alignment mark on the FRONT wall (low Y) near the LEFT corner. Added to both
// the base and the lid and crossing the seam, so the two halves form ONE continuous rib
// when the lid is on the right way around. A lid assembled 180 degrees wrong moves the
// lid's half to the opposite corner -> the marks clearly miss each other.
orient_mark   = true;
orient_mark_x = 8.0;      // board-X of the mark centre (near the left/USB side)
orient_mark_w = 2.5;      // width along X
orient_mark_d = 0.8;      // depth out from the wall (< wall)
orient_mark_h = 9.0;      // total height (half on each side of the seam)

/* [Lid text] */
// Raised text on the lid top, for printing in a second filament colour. The text stands
// lid_text_h proud of the otherwise flat top face, so it is the only geometry above the top
// and a single filament change at that top layer paints exactly the letters (print the lid
// TEXT-UP). Centred on the top face at mid-Y, clear of the LED window (back) and the two
// screw counterbores (front-left / back-right corners).
// OFF by default: the text adds noticeable render time, so leave it off while prototyping
// and turn it on for the final print (set lid_text_show = true, or LID_TEXT_SHOW=true via
// export.sh). Set lid_text = "" to disable regardless.
lid_text_show  = false;
lid_text       = "Open RZ67 Trigger";
lid_text_size  = 3.5;     // cap height (mm); ~43mm wide at this size, fits the 54.8mm top
lid_text_h     = 0.6;     // how far the text stands proud of the top (the 2nd-colour layer)
lid_text_font  = "Liberation Sans:style=Bold";
lid_text_dx    = 0;       // nudge from the top-face centre, X
lid_text_dy    = 0;       // nudge from the top-face centre, Y
lid_text_angle = 0;       // rotation on the top face (deg)

/* [Misc] */
loc_pin = true;       // locating pins in the mount holes (snap mode only)
$fa = 2; $fs = 0.4;
eps = 0.01;

/* ============================ derived ============================ */
off        = wall + clr;                    // board(0,0) -> case-Y; X-offset is bx()
inner_w    = board_w + 2*clr + pcb_overhang_left;  // extra clearance on the left for the USB-C shell
inner_h    = board_h + 2*clr;
inner_r    = board_r + clr;
outer_w    = inner_w + 2*wall;
outer_h    = inner_h + 2*wall;
outer_r    = inner_r + wall;

standoff_h = max(7.0, batt_t + batt_gap);   // PCB underside above the inner floor
standoff_d = 4.0;
comp_clr   = max(h_relay, h_xh, h_ph) + 1.0; // clearance above the PCB for components

pcb_z      = floor_t + standoff_h;          // PCB underside
pcb_top_z  = pcb_z + pcb_t;
split_z    = pcb_top_z;                      // base/lid split
lid_h      = comp_clr + lid_top_t;
total_h    = split_z + lid_h;
bead_z     = split_z - lap + 2.2;

function bx(x) = wall + clr + pcb_overhang_left + x;  // board-X -> case-X (PCB shifted right for USB overhang)
function by(y) = off + y;                             // board-Y -> case-Y

// LED light-pipe centre (mean of D3/D4)
led_cx = (bx(led_pos[0][0]) + bx(led_pos[1][0])) / 2;
led_cy = (by(led_pos[0][1]) + by(led_pos[1][1])) / 2;

/* ====================== 2D profiles / helpers ==================== */
// rounded rectangle with min-corner at (0,0)
module rrect(w, h, r) offset(r) translate([r, r]) square([w - 2*r, h - 2*r]);
// rounded rectangle centred at the origin
module rrect_c(w, h, r) translate([-w/2, -h/2]) rrect(w, h, r);

module outer2D() rrect(outer_w, outer_h, outer_r);
module inner2D() translate([wall, wall]) rrect(inner_w, inner_h, inner_r);
// mid-line in the wall (the split for the ship-lap)
module mid2D()   translate([wall/2, wall/2]) rrect(inner_w + wall, inner_h + wall, inner_r + wall/2);

/* ============================= BASE ============================== */
module base() {
    difference() {
        union() {
            difference() {
                // outer shell up to the split
                linear_extrude(split_z) outer2D();
                // cavity (open upward)
                translate([0, 0, floor_t]) linear_extrude(total_h) inner2D();
                // rabbet: keep only the outer wall half in the lap zone
                translate([0, 0, split_z - lap]) linear_extrude(lap + eps) mid2D();
                // groove for the snap bead in the outer lip (snap mode only)
                if (closure == "snap") snap_groove();
            }
            standoffs();
            pcb_corner_supports();
            batt_retainer();
            pcb_frame_ribs();
            orient_mark_rib(+1);   // orientation mark, lower half
        }
        // clearance pockets for the through-hole solder in the tops of the posts
        th_keepout_pockets();
        // The USB-C opening straddles the split: its centre sits ~1.65mm ABOVE split_z,
        // but the lower ~1.85mm of the hole falls BELOW the split and would otherwise be
        // blocked by the base side wall. Cut the same opening from the base so the whole
        // hole is open.
        cut_usb();
        // channel for the battery cable to come up to BAT1 (also cut from the lid)
        batt_cable_channel();
    }
}

// Raised guide ribs along the front/back edges. They sit in the clearance gap (Y) outside
// the board edge, from the board underside up to frame_proud above the board top, with a
// lead-in chamfer on top. The board is set down BETWEEN the front and back ribs and caught
// in Y.
module pcb_frame_ribs() {
    if (pcb_frame) {
        for (cx = frame_ribs_front) frame_rib(bx(cx), +1);   // FRONT (low Y)
        for (cx = frame_ribs_back)  frame_rib(bx(cx), -1);   // BACK (high Y)
    }
}
// One guide fin, dir=+1 front (low Y) / dir=-1 back (high Y). Stands in the ship-lap's
// tongue channel beside the board edge:
//   - THICK fin from the floor to the board top (split_z), Y from the OUTER wall face (so it
//     merges with the outer wall across the whole lap zone, not just below it) to the catching
//     face frame_clr from the board edge. Catches the board edge laterally across the full
//     board thickness and is backed by the wall along its full height.
//   - THIN lead-in lip above the board top (cavity side only, Y >= wall inner face, so it
//     clears the lid wall), with a chamfer. The lid tongue is cut away where the fin stands
//     (frame_tongue_notch) so the fin clears when the lid closes.
module frame_rib(cx, dir) {
    x0     = cx - frame_rib_l/2;
    w_in   = (dir > 0) ? wall : wall + inner_h;          // cavity-wall inner face
    ycap   = w_in + dir * (clr - frame_clr);             // catching face (frame_clr from board edge)
    yraw   = w_in - dir * frame_fin_reach;
    yreach = (dir > 0) ? max(0, yraw) : min(outer_h, yraw);  // anchored side, clamped to the outer face
    z_cham = split_z + frame_proud - frame_cham;
    flo = min(yreach, ycap); fhi = max(yreach, ycap);    // thick-fin Y range
    clo = min(w_in, ycap);   chi = max(w_in, ycap);      // thin-lip Y range (cavity side)
    union() {
        // thick fin: floor -> board top (anchored in the wall below the lap zone)
        translate([x0, flo, floor_t]) cube([frame_rib_l, fhi - flo, split_z - floor_t]);
        // thin lead-in lip above the board (cavity side, clears the lid wall)
        translate([x0, clo, split_z - eps]) cube([frame_rib_l, chi - clo, z_cham - (split_z - eps)]);
        // chamfer on the lip, narrowing toward the wall inner face
        y_top = (dir > 0) ? w_in : w_in - eps;
        hull() {
            translate([x0, clo, z_cham])                        cube([frame_rib_l, chi - clo, eps]);
            translate([x0, y_top, split_z + frame_proud - eps]) cube([frame_rib_l, eps, eps]);
        }
    }
}
// Cut from the LID: removes the ship-lap tongue where the base fins stand, so the fins clear
// when the lid closes. The notches are frame_notch_clr larger than the fins on all edges.
module frame_tongue_notch() {
    if (pcb_frame) {
        for (cx = frame_ribs_front) tongue_notch(bx(cx), +1);
        for (cx = frame_ribs_back)  tongue_notch(bx(cx), -1);
    }
}
module tongue_notch(cx, dir) {
    x0     = cx - frame_rib_l/2 - frame_notch_clr;
    L      = frame_rib_l + 2*frame_notch_clr;
    w_in   = (dir > 0) ? wall : wall + inner_h;
    ycap   = w_in + dir * (clr - frame_clr);
    yraw   = w_in - dir * frame_fin_reach;
    yreach = (dir > 0) ? max(0, yraw) : min(outer_h, yraw);
    nlo = min(yreach, ycap) - frame_notch_clr;
    nhi = max(yreach, ycap) + frame_notch_clr;
    translate([x0, nlo, split_z - lap - eps]) cube([L, nhi - nlo, lap + 2*eps]);
}

// Rounded vertical cable channel at the back-left corner (left wall) for the battery cable.
// Cut from BOTH base and lid so it spans the split: it scallops the inner wall (leaving
// wall - batt_cable_depth of skin) and removes the lid tongue locally, so the cable runs from
// the under-PCB battery space up to BAT1 on the PCB top without being pinched by the closing
// lid. Centred at the left inner wall face (X = wall) so it reaches X = wall - depth into the
// wall and opens batt_cable_r into the cavity gap; runs floor -> lid ceiling.
module batt_cable_channel() {
    if (batt_cable) {
        cx = wall - batt_cable_depth + batt_cable_r;   // leftmost point reaches wall - depth
        cy = by(batt_cable_y);
        translate([cx, cy, floor_t - eps])
            cylinder(h = (total_h - lid_top_t) - floor_t + 2*eps, r = batt_cable_r);
    }
}

// Solid support posts in the corners WITHOUT a screw hole. Same height as the standoffs,
// without a pilot hole or pin - the PCB just rests on top of them.
module pcb_corner_supports() {
    for (p = pcb_supports)
        translate([bx(p[0]), by(p[1]), floor_t])
            cylinder(h = standoff_h, d = standoff_d);
}

// Cuts a cylindrical pocket down from the PCB underside where TH solder sticks
// down, so nearby posts don't press against the solder joint.
module th_keepout_pockets() {
    for (k = th_keepouts)
        translate([bx(k[0]), by(k[1]), pcb_z - th_keepout_depth])
            cylinder(h = th_keepout_depth + eps, d = k[2]);
}

module standoffs() {
    for (h = mount_holes) {
        difference() {
            // post: wider at heat-set (needs wall around the insert)
            translate([bx(h[0]), by(h[1]), floor_t])
                cylinder(h = standoff_h, d = (closure == "screw") ? anchor_boss_d : standoff_d);
            // hole from the post top: heat-set pocket or self-tapping pilot
            if (closure == "screw")
                translate([bx(h[0]), by(h[1]), pcb_z - anchor_hole_h])
                    cylinder(h = anchor_hole_h + eps, d = anchor_hole_d);
        }
        // locating pin in snap mode only (the screw locates otherwise)
        if (closure == "snap" && loc_pin)
            translate([bx(h[0]), by(h[1]), pcb_z - eps])
                cylinder(h = pcb_t + 0.8, d = hole_d - 0.3);
    }
}

module batt_retainer() {
    x0 = bx(batt_pos[0]); y0 = by(batt_pos[1]);
    translate([0, 0, floor_t])
        difference() {
            translate([x0 - 1.2, y0 - 1.2, 0])
                cube([batt_w + 2.4, batt_l + 2.4, rib_h]);
            translate([x0, y0, -eps])
                cube([batt_w, batt_l, rib_h + 2*eps]);
        }
}

module snap_groove() {
    translate([0, 0, bead_z - (bead_h + 0.5)/2])
        linear_extrude(bead_h + 0.5)
            difference() {
                offset(snap_bead + 0.25) mid2D();
                offset(-lap_gap - 0.25) mid2D();
            }
}

/* ============================== LID ============================== */
module lid() {
    union() {
    difference() {
        union() {
            difference() {
                union() {
                    // top + walls from the split upward
                    translate([0, 0, split_z]) linear_extrude(lid_h) outer2D();
                    // inner lip down into the base (ship-lap)
                    translate([0, 0, split_z - lap]) linear_extrude(lap)
                        difference() { offset(-lap_gap) mid2D(); inner2D(); }
                    // snap bead on the lip (snap mode only)
                    if (closure == "snap") snap_bead_band();
                }
                // cavity under the top plate
                translate([0, 0, split_z - lap - eps])
                    linear_extrude(comp_clr + lap + eps) inner2D();
            }
            // added AFTER the cavity (otherwise they'd be removed):
            if (sw_enable && sw_screw) switch_bosses();
            holddowns();
            orient_mark_rib(-1);   // orientation mark, upper half
        }
        // all openings cut last (through wall + holder)
        cut_usb();
        cut_xh();
        cut_led();
        if (sw_enable) switch_slot();
        if (sw_enable && sw_plate_recess) switch_plate_recess();
        if (sw_enable && sw_screw) switch_screws();
        if (closure == "screw") screw_holes(); else holddown_pinholes();
        // clearance for component bodies above the PCB (bosses give way to the USB-C connector etc.)
        comp_keepout_volumes();
        // notch in the lid tongue where the base posts melt into the wall (else they hit the tongue)
        if (closure == "screw") boss_tongue_relief();
        // notch in the lid tongue where the PCB frame fins stand (else the lid won't go down)
        frame_tongue_notch();
        // channel for the battery cable to come up to BAT1 (also cut from the base)
        batt_cable_channel();
    }
    // raised text, added on top of the finished lid so the openings can't cut it
    lid_text_relief();
    }
}

// Raised lid text, fused onto the lid top (lid_text_h proud of the top face). Colour it by
// a filament change at the top layer in the slicer; off by default (see lid_text_show).
module lid_text_relief() {
    if (lid_text_show && lid_text != "")
        translate([outer_w/2 + lid_text_dx, outer_h/2 + lid_text_dy, total_h - eps])
            rotate([0, 0, lid_text_angle])
                linear_extrude(lid_text_h + eps)
                    text(lid_text, size = lid_text_size, font = lid_text_font,
                         halign = "center", valign = "center", $fn = 32);
}

// Cuts out the component clearances (full height above the PCB) so lid bosses don't
// collide with component bodies like the USB-C connector.
module comp_keepout_volumes() {
    for (k = comp_keepouts)
        translate([bx(k[0]), by(k[1]), pcb_top_z - eps])
            cube([k[2] - k[0], k[3] - k[1], (total_h - lid_top_t) - pcb_top_z + 2*eps]);
}

// Relieves the lid tongue (ship-lap) where the base posts at the mount holes are so large
// (Ø6 heat-set boss) that they stick past the cavity wall and into the tongue channel. Without
// this the tongue collides with the post and the lid can't be fitted. The cut sits ONLY in
// the lap zone (below split_z) - above split_z the tongue is gone anyway. The screw holds the
// alignment in the corners, so a small notch in the tongue here does no harm.
module boss_tongue_relief() {
    relief_d = anchor_boss_d + 2*lap_gap + 0.3;   // post diameter + sliding clearance
    for (h = mount_holes)
        translate([bx(h[0]), by(h[1]), split_z - lap - eps])
            cylinder(h = lap + 2*eps, d = relief_d);
}

module snap_bead_band() {
    translate([0, 0, bead_z - bead_h/2]) linear_extrude(bead_h)
        difference() {
            offset(-lap_gap + snap_bead) mid2D();
            offset(-lap_gap - 0.4) mid2D();
        }
}

module holddowns() {
    bd = (closure == "screw") ? boss_d_screw : 5;
    for (h = mount_holes)
        translate([bx(h[0]), by(h[1]), pcb_top_z])
            cylinder(h = (total_h - lid_top_t) - pcb_top_z, d = bd);
}
// snap mode: clearance for the locating pin
module holddown_pinholes() {
    for (h = mount_holes)
        translate([bx(h[0]), by(h[1]), pcb_top_z - eps])
            cylinder(h = total_h, d = 2.6);
}
// screw mode: clearance hole through the boss + countersunk head in the top
module screw_holes() {
    for (h = mount_holes) {
        translate([bx(h[0]), by(h[1]), pcb_top_z - eps])
            cylinder(h = total_h, d = screw_clear);
        translate([bx(h[0]), by(h[1]), total_h - head_h])
            cylinder(h = head_h + eps, d = head_d);
    }
}

/* =========================== OPENINGS ============================ */
// thin slab in the YZ plane, centred at (cy, zc), for hull() along X
module usb_slab(cy, zc, w, h)
    translate([0, cy - w/2, zc - h/2]) cube([eps, w, h]);

// rounded-rectangle hull in the YZ plane, extruded along +X from x0, length L
module usb_rrect_cut(cy, zc, W, H, R, x0, L) {
    yhalf = W/2 - R;
    zhalf = H/2 - R;
    hull() {
        for (sy = [-1, 1], sz = [-1, 1])
            translate([x0, cy + sy*yhalf, zc + sz*zhalf])
                rotate([0, 90, 0])
                    cylinder(h = L, r = R);
    }
}

module cut_usb() {
    cy = by(22 - 13.259);            // USB1 centre
    zc = pcb_top_z + h_usbc/2;       // centre of the connector mouth
    // 1) Through-hole 11x7 R2 through the WHOLE wall. The outer face is the lip that
    //    stops the cable body; the metal plug (8.34x2.56) passes through fine.
    usb_rrect_cut(cy, zc, usb_open_w, usb_open_h, usb_open_r, -1, wall + 2);
    // 2) Inner recess 13x9 R3 on the cavity side of the wall: eats into the last
    //    usb_recess_d mm of the wall, so the USB-C shell/body can poke into the wall
    //    and the connector mouth comes near the outer face.
    if (usb_recess)
        usb_rrect_cut(cy, zc, usb_recess_w, usb_recess_h, usb_recess_r,
                      wall - usb_recess_d, usb_recess_d + 1);  // +1 out into the cavity
    // 3) Optional 45-degree chamfer on the outer face as a lead-in (off by default).
    if (usb_chamfer > 0) {
        hull() {
            translate([-eps, 0, 0]) usb_slab(cy, zc, usb_open_w + 2*usb_chamfer,
                                                       usb_open_h + 2*usb_chamfer);
            translate([usb_chamfer, 0, 0]) usb_slab(cy, zc, usb_open_w, usb_open_h);
        }
    }
}
module cut_xh() {
    // small rounded cable port in the right wall (not the whole connector)
    cy = by(22 - 11.0);                 // U4 centre
    zc = pcb_top_z + cable_port_z;
    r  = cable_port_h / 2;
    dy = max((cable_port_w - cable_port_h) / 2, 0);
    hull() for (s = [-1, 1])
        translate([outer_w - wall - 1, cy + s*dy, zc])
            rotate([0, 90, 0]) cylinder(h = wall + 2, r = r);
}
// Lid cut-out for the light pipe: through-hole (stem) + top recess (head).
module cut_led() {
    if (led_show) {
        // stem hole through the whole top plate (window)
        translate([led_cx, led_cy, total_h - lid_top_t - eps])
            linear_extrude(lid_top_t + 2*eps)
                rrect_c(led_win_l + led_pipe_clr, led_win_w + led_pipe_clr, led_win_r);
        // top recess for the head (so the head sits flush with the top face)
        translate([led_cx, led_cy, total_h - led_head_t])
            linear_extrude(led_head_t + eps)
                rrect_c(led_win_l + 2*led_head_lip + led_pipe_clr,
                        led_win_w + 2*led_head_lip + led_pipe_clr,
                        led_win_r + led_head_lip);
    }
}

// The light pipe itself (printed separately in CLEAR filament, inserted from above).
// Top hat: wide head in the recess (flush with the top) + stem down toward the LEDs.
module light_pipe() {
    z_bot = pcb_top_z + led_pipe_gap;          // just above the LED top
    z_cb  = total_h - led_head_t;              // recess bottom (head rests here)
    union() {
        // head (full size, the clearance comes from the recess)
        translate([led_cx, led_cy, z_cb])
            linear_extrude(led_head_t)
                rrect_c(led_win_l + 2*led_head_lip, led_win_w + 2*led_head_lip,
                        led_win_r + led_head_lip);
        // stem: from the recess bottom down to just above the LEDs
        translate([led_cx, led_cy, z_bot])
            linear_extrude(z_cb - z_bot + eps)
                rrect_c(led_win_l, led_win_w, led_win_r);
    }
}

// Raised vertical orientation/alignment mark on the front wall (low Y) at the left corner.
// ADDED to both the base (lower half) and the lid (upper half), centred on the seam, so the
// halves line up into ONE continuous rib when the lid is on the right way around. A lid
// assembled 180 degrees wrong moves the lid's half to the opposite corner -> the two halves
// clearly miss each other. The mark sticks OUT from the wall (adds material; a recessed groove
// here would thin out the 1mm-thin lap wall). Clear of the switch slot/bosses (centred around
// bx(sw_x), well to the right of bx(orient_mark_x)). dir=+1: lower half (base), dir=-1: upper
// half (lid).
module orient_mark_rib(dir) {
    if (orient_mark) {
        cx = bx(orient_mark_x);
        z0 = (dir > 0) ? split_z - orient_mark_h/2 : split_z;
        translate([cx - orient_mark_w/2, -orient_mark_d, z0])
            cube([orient_mark_w, orient_mark_d + eps, orient_mark_h/2]);
    }
}

/* ========================== SLIDE SWITCH ========================= */
module switch_slot() {
    sx = bx(sw_x);
    zc = split_z + sw_z;
    if (sw_wall == "front")
        translate([sx - sw_slot_l/2, -1, zc - sw_slot_h/2])
            cube([sw_slot_l, wall + 2, sw_slot_h]);
    else
        translate([sx - sw_slot_l/2, outer_h - wall - 1, zc - sw_slot_h/2])
            cube([sw_slot_l, wall + 2, sw_slot_h]);
}
// 2 screw bosses flanking the actuator. Vertical rectangular PILLARS that run from the
// screw centre ALL THE WAY UP into the lid ceiling -> the screw force is taken by the
// ceiling and (above the plate recess) by the wall. The pillar front sits at the inner wall
// plane, right behind the recessed mounting plate; tightening clamps boss -> plate -> recess
// floor (solid wall) in compression. (Added to the lid solid.)
module switch_bosses() {
    sx       = bx(sw_x);
    front    = (sw_wall == "front");
    y_wall   = front ? wall : outer_h - wall;   // inner wall face = pillar front
    pillar_w = sw_boss_d;                 // X width around the screw hole
    pillar_d = sw_boss_h;                 // Y depth inward from the wall
    // The PCB top is at split_z; the pillar starts sw_boss_gap above and runs up into the
    // ceiling. The gap clears the components under the boss (Q1 SOT-23 etc.); the rest of the
    // height gives lateral stiffness when the screw is tightened.
    z_bot    = split_z + sw_boss_gap;
    z_top    = total_h - lid_top_t + 1;   // 1mm overlap -> fuses with the top plate
    h_pillar = z_top - z_bot;
    for (s = [-1, 1]) {
        x = sx + s * sw_screw_pitch / 2;
        if (front)
            translate([x - pillar_w/2, y_wall,            z_bot])
                cube([pillar_w, pillar_d, h_pillar]);
        else
            translate([x - pillar_w/2, y_wall - pillar_d, z_bot])
                cube([pillar_w, pillar_d, h_pillar]);
    }
}

// Pocket in the OUTER wall for the switch's flat mounting plate, so it sits flush with the
// outside. Cut from the lid (the switch is on the lid portion of the wall). sw_plate_t deep,
// plate footprint + clearance, centred on the screw height. The inner wall behind it stays
// solid, so the screw bosses bear against full wall.
module switch_plate_recess() {
    if (sw_plate_recess) {
        sx = bx(sw_x);
        zc = split_z + sw_z + sw_screw_dz;
        w  = sw_plate_w + sw_plate_clr;
        h  = sw_plate_h + sw_plate_clr;
        if (sw_wall == "front")
            translate([sx - w/2, -eps, zc - h/2])                  // outer face at Y=0
                cube([w, sw_plate_t + eps, h]);
        else
            translate([sx - w/2, outer_h - sw_plate_t, zc - h/2])  // outer face at Y=outer_h
                cube([w, sw_plate_t + eps, h]);
    }
}

// clearance hole through the wall (the head sits on the external bracket) + pilot in the boss
module switch_screws() {
    sx    = bx(sw_x);
    zc    = split_z + sw_z + sw_screw_dz;
    front = (sw_wall == "front");
    for (s = [-1, 1]) {
        x = sx + s*sw_screw_pitch/2;
        if (front) {
            translate([x, -1, zc])            rotate([-90,0,0]) cylinder(h = wall + 1, d = sw_screw_d);     // clearance through bracket + wall
            translate([x, wall - eps, zc])    rotate([-90,0,0]) cylinder(h = sw_boss_h + eps, d = sw_boss_pilot);  // pilot into the boss
        } else {
            translate([x, outer_h - wall - 1, zc]) rotate([-90,0,0]) cylinder(h = wall + 1, d = sw_screw_d);
            translate([x, outer_h - wall + eps, zc]) rotate([90,0,0]) cylinder(h = sw_boss_h + eps, d = sw_boss_pilot);
        }
    }
}

// simple proxy of the switch for preview (visual space check only)
module switch_preview() {
    sx = bx(sw_x); zc = split_z + sw_z;
    color([0.12, 0.12, 0.12, 0.85]) {
        translate([sx - 5.5, wall, zc - 3.5]) cube([11, sw_body_w, 7]);     // body
        translate([sx - 1.5, -2, zc - 1.5]) cube([3, wall + 2.5, 3]);       // actuator
        for (s = [-1, 1])
            translate([sx + s*sw_screw_pitch/2 - 3, wall, zc - 1]) cube([6, 0.6, 2]); // ear
    }
}

/* ===================== PREVIEW: PCB + components ================= */
module pcb() {
    color([0.0, 0.45, 0.18, 0.55])
        translate([off, off, pcb_z]) linear_extrude(pcb_t)
            translate([0, 0]) rrect(board_w, board_h, board_r);
}
module comp(cx, cy, sx, sy, sz)
    translate([bx(cx) - sx/2, by(22 + cy) - sy/2, 0]) cube([sx, sy, sz]);
module components() {
    color([0.25, 0.25, 0.28, 0.7]) translate([0, 0, pcb_top_z]) {
        comp(36.661, -16.242, 7.0, 11.0, h_relay);  // K1 relay
        comp(36.760,  -5.686, 11.0, 7.0, h_relay);  // K2 relay
        comp(44.3,   -11.0,   6.0, 12.0, h_xh);      // U4 XH plug
        // USB-C: body 8.95x3.2 on the PCB (left edge 4.8mm in from the PCB left edge),
        // the shell sticks ~6.8mm past the body left, total 15.75mm X-extent from
        // board X=-2 to X=13.75 (centred 5.875). Y-width 3.2mm.
        comp(5.875,  -13.259, 15.75, 3.2, h_usbc);   // USB1 (body + shell)
        comp(3.275,   -4.811, 4.0, 6.0,  h_ph);      // BAT1
        comp(28.713,  -2.04,  6.0, 4.0,  h_ph);      // S3
        comp(21.257, -14.419, 5.0, 5.0,  1.0);       // ESP32 U1
    }
    // LEDs (D3 red, D4 blue) - 0603, shine up
    color([1.0, 0.1, 0.1]) translate([0,0,pcb_top_z]) comp(22.514, -1.814, 1.6, 0.8, 0.6);
    color([0.1, 0.3, 1.0]) translate([0,0,pcb_top_z]) comp(24.224, -1.814, 1.6, 0.8, 0.6);
}

/* ============================= RENDER ============================ */
if (part == "base") base();
else if (part == "lid") lid();
else if (part == "lightpipe") light_pipe();         // clear insert - print in clear filament
else if (part == "both") { base(); translate([0, outer_h + 8, 0]) lid(); }
else {                      // preview: assembled, lid translucent
    base();
    if (show_pcb) { pcb(); components(); }
    if (sw_enable && sw_screw) switch_preview();
    color([0.6, 0.6, 0.65, 0.25]) lid();
    color([0.9, 0.9, 0.2]) lid_text_relief();            // lid text (2nd colour)
    if (led_show) color([1, 1, 1, 0.55]) light_pipe();   // light pipe in place
}
