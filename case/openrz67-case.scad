/* =====================================================================
 * openrz67-trigger — parametrisk kabinett (OpenSCAD)
 * ---------------------------------------------------------------------
 * Dimensjoner hentet fra PCB-leveransen (2025-09-23):
 *   - Brettomriss:  Gerber_BoardOutlineLayer.GKO  -> 48.0 x 22.0 mm, R2.0
 *   - Monteringshull: Drill_PTH_Through.DRL (T06)  -> 2x Ø2.0 i diagonal
 *   - Komponentplassering: PickAndPlace_..._2025-09-23.csv
 *   - PCB-tykkelse: 1.6mm (faktisk bestilt)  -> pcb_t = 1.6
 *
 * Oppsett (valgt sammen med bruker):
 *   - Snap-fit (klips) lokk via skipslask + omkringgående perle
 *   - Batteri (250mAh LiPo) UNDER PCB; brettet heves paa standoffs
 *   - Aapninger: USB-C (venstre), XH-kameraplugg (hoyre), LED-vindu (topp),
 *                spor + holder for SS12F15 skyvebryter
 *
 * Koordinatsystem: brettkoordinater med origo NEDE-VENSTRE, Y opp.
 * Pick&place bruker origo oppe-venstre (Y nedover); konvertert som
 *   case_Y = 22 + midY.  bx()/by() flytter brettkoord inn i case-rammen.
 * ===================================================================== */

/* [Render] */
// Hva som skal genereres
part = "preview";   // [preview, base, lid, lightpipe, both]
// Vis PCB + grove komponentklosser i preview (kun visuelt)
show_pcb = true;

/* [Brett] */
board_w = 48.0;     // X
board_h = 22.0;     // Y
board_r = 2.0;      // hjorneradius
pcb_t   = 1.6;      // PCB-tykkelse (faktisk bestilt; juster om ditt brett avviker)
hole_d  = 2.0;      // monteringshull
// monteringshull i brettkoord [x, y] (Y opp):
mount_holes = [[2, 2], [46, 20]];
// USB-C-skallet stikker ~2mm forbi PCB-ens venstre kortside (kontaktens "munning"
// rager utenfor brettkanten). Casens venstre side forlenges tilsvarende: hulrommet
// blir inner_w bredt, og PCB-en sentreres mot HOYRE kortside (vanlig clr), med
// hele overhang_left + clr som luft paa VENSTRE for USB-skallet.
pcb_overhang_left = 2.0;
// Solide stoettepilarer paa basen i hjornene HVOR DET IKKE ER SKRUEHULL.
// Holder PCB-en jevnt opp i alle fire hjorner uten aa gjenge mot skrue.
pcb_supports = [[2, 20], [46, 2]];
// Gjennomhulls-loddepunkter som stikker NED under PCB. Standoffs/stoettepilarer
// som kommer for naer disse faar en lomme skaaret i toppen saa de ikke presser
// mot loddetinnet. [brett_x, brett_y, frigangsdiameter]. (BAT1 = TH PH-kontakt
// rett ved stoettepilaren paa USB-siden; loddetinnet kolliderte ellers.)
th_keepouts = [
    [3.275, 16.189, 3.2],   // BAT1 pin 1
    [3.275, 18.189, 3.2],   // BAT1 pin 2
];
th_keepout_depth = 3.0;     // hvor langt ned loddetinn/pin-tail stikker under PCB
// Komponentlegemer OVER PCB som lokk-bosser (nedholdere/bryterbosser) maa vike for.
// [brett_x0, brett_y0, brett_x1, brett_y1] - rektangel i brettkoord, full hoyde over
// PCB. (USB-C-kontakten: naer kant ~brett-Y 4.8; Ø6-nedholderen ved hull (2,2) naadde
// til Y 5 og tok borti. Frigangen kutter bossen tilbake til brett-Y 4.0 paa den siden,
// som fjoraarets design der hjorne-bossen var liten og gjemt inn mot hjornet.)
comp_keepouts = [
    [0.0, 4.0, 8.0, 14.5],    // USB-C-kontaktlegeme (venstre kortside)
];

/* [PCB-ramme (guide-finner)] */
// Guide-finner langs FRONT- og BAKKANT som PCB-en legges ned mellom, saa den ikke
// skyves av standoffene for skruene er satt. Det er bare ~0.4mm klaring ved siden av
// brettet, for tynt for en printbar finne. Men skipslasken har en ~1mm TUNGE-KANAL rett
// utenfor vegglinjen: finnen staar i den (tykk og godt forankret i veggen), ved siden av
// brettkanten, og et lite HAKK skjaeres i lokk-tunga der finnene staar saa de gaar klar.
// Finnene ligger ALDRI over brettet (treffer ingen komponenter). Apne i hjorner +
// venstre/hoyre kortside (USB-C / XH-kabel).
pcb_frame   = true;
frame_clr   = 0.2;     // klaring mellom finnens fangeflate og brettkant (drop-in)
frame_proud = 0.6;     // tynn lead-in lippe over brett-toppen (mot loft + innretting)
frame_cham  = 0.4;     // fas paa lippa
frame_rib_l = 8.0;     // finnelengde langs kanten
frame_fin_reach = 0.7; // hvor langt finnen naar UT i tunge-kanalen (stivhet/forankring)
frame_notch_clr = 0.3; // klaring naar lokk-tunga skjaeres bort rundt finnen
// Finne-senter i brett-X (bx() konverterer). Front = lav Y, bak = hoy Y.
frame_ribs_front = [10, 38];
frame_ribs_back  = [10, 38];

/* [Passform og vegger] */
clr       = 0.4;    // klaring mellom PCB-kant og innervegg
wall      = 2.0;    // sidevegg
floor_t   = 2.0;    // bunn
lid_top_t = 2.0;    // topplate

/* [Vertikal oppbygning] */
batt_t   = 6.0;     // LiPo-tykkelse (under PCB)
batt_gap = 1.0;     // klaring over batteriet
// Komponenthoyder over PCB-topp (datablad/estimat - juster ved behov):
h_relay  = 5.2;     // G6K-2F-Y reléer (K1, K2)
h_xh     = 8.0;     // JST XH 4-pin kameraplugg (U4)  <- hoyeste
h_usbc   = 3.3;     // USB-C 16P 2MD (USB1)
h_ph     = 6.0;     // JST PH 2-pin (BAT1, S3)

/* [Batterirom] */
batt_w   = 31;      // fotavtrykk X (~20% storre enn forrige 26)
batt_l   = 19;      // fotavtrykk Y (~20% storre enn forrige 16)
batt_pos = [8.5, 1.5]; // min-hjorne i brettkoord (sentrert; klar av USB1/U4/standoffs)
rib_h    = 2.5;     // hoyde paa holderibber

/* [Aapninger] */
// USB-C (matcher fjoraarets design fra STEP-fila):
//   - Ytterflate: 11x7 gjennomgang. Denne kanten ER leppa som stopper
//     kabelkroppen/overstopet (alt > 11x7) fra aa komme inn.
//   - Innvendig (mot hulrommet): 13x9 innsenkning som spiser inn de siste
//     1.5mm av veggen. Lar USB-C-mottakerens kropp/skall stikke litt inn i
//     veggen og bringer kontaktmunningen naer ytterflata.
//   - Selve metallpluggen (8.34x2.56) passerer rikelig gjennom 11x7.
usb_open_w   = 11.0;        // langs Y - leppe-bredde
usb_open_h   = 7.0;         // langs Z - leppe-hoyde
usb_open_r   = 2.0;         // hjorneradius paa ytre leppe (fjoraarets STEP: R2)
usb_chamfer  = 0.0;         // 45-graders fas (lead-in) paa munningen, 0 = av (matcher fjoraarets rene flate)
usb_recess   = true;        // innvendig innsenkning paa cavity-siden av veggen
usb_recess_w = 13.0;        // bredde innvendig (matcher fjoraarets 13x9)
usb_recess_h = 9.0;         // hoyde innvendig
usb_recess_d = 1.5;         // dybde inn fra innerveggen (maa vaere < wall)
usb_recess_r = 3.0;         // hjorneradius paa innvendig innsenkning (fjoraarets STEP: R3)
/* [LED-lysleder] */
// D3 (rod) og D4 (bla) er SMD-LED-er paa PCB-toppen som lyser RETT OPP, ~8mm under
// lokket. En separat KLAR lysleder-innsats (light pipe) ledes ned til like over LED-ene
// og fanger lyset til to skarpe prikker paa toppflaten. Lokket selv printes opakt; bare
// lyslederen printes i klart filament og settes inn ovenfra (topp-hatt i forsenkning).
led_show  = true;
// LED-posisjoner i brettkoord (fra pick&place: D3, D4 - 1.71mm fra hverandre):
led_pos   = [[22.514, 20.186], [24.224, 20.186]];
led_win_l = 6.0;     // optisk vindu lengde (X, langs LED-raden) - dekker begge
led_win_w = 3.0;     // vindu bredde (Y)
led_win_r = 1.2;     // hjorneradius
led_head_lip = 1.0;  // hvor mye hodet/flensen er bredere enn staven (sitter i forsenkning)
led_head_t   = 1.0;  // hode-tykkelse = dybde paa topp-forsenkningen
led_pipe_clr = 0.2;  // klaring mellom lysleder og hull/forsenkning i lokket
led_pipe_gap = 1.5;  // luftgap mellom lysleder-bunn og LED-topp

/* [Kabelutgang kameraplugg (XH)] */
// XH-kontakten plugges i FOER montering og blir vaerende inni kabinettet;
// kun de tynne ledningene fores ut gjennom denne lille gjennomfoeringen.
cable_port_w = 5.0;         // bredde (langs Y)  -- sett == _h for et rundt hull
cable_port_h = 2.6;         // hoyde  (langs Z)
cable_port_z = 3.0;         // senterhoyde over PCB-topp

/* [Skyvebryter SS12F15 (kobles til S3)] */
sw_enable = true;
sw_wall   = "front";  // [front, back]  hvilken langside. FRONT (lav Y) valgt fordi
                      // bakveggen er blokkert av S3 (kontakt) + K2-rele; bare fronten
                      // har plass til 15mm boss-avstand sentrert. Ledning fra S3 fores
                      // tvers over brettet til bryteren.
sw_x      = 22;       // brettkoord X. Case-senter er 23; skjovet 1mm venstre saa hoyre
                      // boss klarerer K1-reléet (brett-X 33.16) med ~1.2mm. Praktisk sentrert.
sw_z      = 4.5;      // senterhoyde over PCB-topp
// Aktuatorslisse (flatt mot vegg; tapp ~3mm + reise). Mal fra SS12F15.stp.
sw_slot_l = 9.0;      // langs veggen (reise + tapp)
sw_slot_h = 3.6;      // hoyde (tapp ~3mm + klaring)
sw_body_w = 8.0;      // hvor langt bryterkroppen stikker inn fra veggen (klaring)
// Skruefeste: orene ligger mot innerveggen, skruer utenfra -> gjennom vegg ->
// gjenger seg i bosser paa innsiden (orene er Ø2.2 klaringshull i STEP-fila).
sw_screw        = true;
sw_screw_pitch  = 15.0;  // senteravstand mellom orene (fra STEP: X = +-7.5)
sw_screw_d      = 2.4;   // klaringshull gjennom veggen (M2)
sw_screw_dz     = 0.0;   // hoyde-offset fra slissesenter
sw_screw_head_d = 4.2;   // counterbore for hode paa ytterflaten
sw_screw_head_h = 1.4;
sw_boss_d       = 5.0;   // skrueboss paa innervegg
sw_boss_h       = 5.0;   // hvor langt bossen stikker inn i casen
sw_boss_pilot   = 1.5;   // pilothull (M2 selvgjengende)
sw_ear_t        = 0.6;   // tykkelse paa bryterens ore - bossen trekkes tilbake saa mye
// Bryter-bossenes fotavtrykk (Ø5, X14.5 & X29.5, brett-Y 0.2-5.2) ligger OVER tett
// pakkede komponenter langs frontkanten: Q1 (SOT-23, ~1.1mm), C15/C26 (0603, ~0.9mm)
// + 0402-er. Boss-bunnen maa klarere den hoyeste, ellers loftes lokket. (Var 0.5mm
// -> Q1/C15 traff -> ~0.5mm glippe. 1.5mm gir ~0.4mm margin over SOT-23.)
sw_boss_gap     = 1.5;   // luft mellom boss-bunn og PCB-topp (klarerer komponenter under)

/* [Lukking] */
// Hvordan lokket holdes igjen. Begge beholder tunge-i-spor-kanten for innretting.
closure = "screw";    // [screw, snap]

/* [Skruer (closure=screw)] */
// 2 skruer ovenfra gjennom de ekte monteringshullene. Skrue: forsenket hode i
// lokket -> klaring gjennom lokk-boss -> gjennom PCB Ø2-hull -> ned i base-stolpen.
screw_d      = 2.0;   // M2. Bruk 1.6 for romsligere klaring gjennom PCB-hullet.
screw_engage = 5.0;   // (kun selftap) gjengelengde ned i stolpen
head_d       = 4.0;   // counterbore for skruehode (M2 sokkel ~3.8)
head_h       = 1.8;   // dybde paa counterbore
boss_d_screw = 6.0;   // diameter paa lokk-boss (nedholder) i skruemodus
screw_clear  = screw_d + 0.4;   // klaringshull i lokk-boss + topp

// Hvordan skruen forankres i BASE-stolpen:
//   "heatset" - messinginnsats smeltet inn i toppen av stolpen (sterkest, gjenbrukbar)
//   "selftap" - selvgjengende skrue rett i plast (enklest, men stripper ved gjenbruk)
screw_anchor = "heatset";   // [heatset, selftap]
// Heat-set M2 (sjekk MOT DINE innsatser - varierer mellom merker!):
heatset_hole_d = 2.6;   // hulldiameter for innsatsen (messingen fortrenger plast i knurl)
heatset_depth  = 4.0;   // innsatslengde / hvor dypt hullet bores fra stolpetoppen
heatset_boss_d = 6.0;   // stolpediameter ved heat-set (trenger ~1.7mm vegg rundt innsatsen)
// Selvgjengende:
screw_pilot  = screw_d - 0.5;   // pilothull for selvgjengende skrue i plast
// avledet: effektiv stolpediameter + hulldiameter/-dybde fra stolpetoppen
anchor_boss_d = (screw_anchor == "heatset") ? heatset_boss_d : standoff_d;
anchor_hole_d = (screw_anchor == "heatset") ? heatset_hole_d : screw_pilot;
anchor_hole_h = (screw_anchor == "heatset") ? heatset_depth  : screw_engage;

/* [Snap-fit (closure=snap)] */
lap       = 6.0;      // overlapp mellom base og lokk
lap_gap   = 0.15;     // glidklaring i lasken
snap_bead = 0.5;      // hvor mye perla stikker ut
bead_h    = 1.4;      // perlehoyde

/* [Diverse] */
loc_pin = true;       // lokaliseringspinner i monteringshull (kun snap-modus)
$fa = 2; $fs = 0.4;
eps = 0.01;

/* ============================ avledet ============================ */
off        = wall + clr;                    // brett(0,0) -> case-Y; X-offset er bx()
inner_w    = board_w + 2*clr + pcb_overhang_left;  // ekstra luft pa venstre for USB-C-skall
inner_h    = board_h + 2*clr;
inner_r    = board_r + clr;
outer_w    = inner_w + 2*wall;
outer_h    = inner_h + 2*wall;
outer_r    = inner_r + wall;

standoff_h = max(7.0, batt_t + batt_gap);   // PCB-underside over innvendig bunn
standoff_d = 4.0;
comp_clr   = max(h_relay, h_xh, h_ph) + 1.0; // klaring over PCB for komponenter

pcb_z      = floor_t + standoff_h;          // PCB-underside
pcb_top_z  = pcb_z + pcb_t;
split_z    = pcb_top_z;                      // base/lokk-skille
lid_h      = comp_clr + lid_top_t;
total_h    = split_z + lid_h;
bead_z     = split_z - lap + 2.2;

function bx(x) = wall + clr + pcb_overhang_left + x;  // brett-X -> case-X (PCB skjovet hoyre for USB-overhang)
function by(y) = off + y;                             // brett-Y -> case-Y

// LED-lysleder-senter (snitt av D3/D4)
led_cx = (bx(led_pos[0][0]) + bx(led_pos[1][0])) / 2;
led_cy = (by(led_pos[0][1]) + by(led_pos[1][1])) / 2;

/* ====================== 2D-profiler / hjelpere ==================== */
// avrundet rektangel med min-hjorne i (0,0)
module rrect(w, h, r) offset(r) translate([r, r]) square([w - 2*r, h - 2*r]);
// avrundet rektangel sentrert i origo
module rrect_c(w, h, r) translate([-w/2, -h/2]) rrect(w, h, r);

module outer2D() rrect(outer_w, outer_h, outer_r);
module inner2D() translate([wall, wall]) rrect(inner_w, inner_h, inner_r);
// midtlinje i veggen (skillet for skipslasken)
module mid2D()   translate([wall/2, wall/2]) rrect(inner_w + wall, inner_h + wall, inner_r + wall/2);

/* ============================= BASE ============================== */
module base() {
    difference() {
        union() {
            difference() {
                // ytre skall opp til skillet
                linear_extrude(split_z) outer2D();
                // hulrom (aapent oppover)
                translate([0, 0, floor_t]) linear_extrude(total_h) inner2D();
                // rabbet: behold kun ytre vegghalvdel i lasksonen
                translate([0, 0, split_z - lap]) linear_extrude(lap + eps) mid2D();
                // spor for snap-perla i ytre leppe (kun snap-modus)
                if (closure == "snap") snap_groove();
            }
            standoffs();
            pcb_corner_supports();
            batt_retainer();
            pcb_frame_ribs();
        }
        // frigangslommer for gjennomhulls-loddetinn i toppen av pilarene
        th_keepout_pockets();
    }
}

// Hevede guide-ribber langs front-/bakkant. Ligger i klaringsgapet (Y) utenfor
// brettkanten, fra brett-underside opp til frame_proud over brett-toppen, med lead-in
// fas paa toppen. Brettet legges ned MELLOM front- og bak-ribbene og fanges i Y.
module pcb_frame_ribs() {
    if (pcb_frame) {
        for (cx = frame_ribs_front) frame_rib(bx(cx), +1);   // FRONT (lav Y)
        for (cx = frame_ribs_back)  frame_rib(bx(cx), -1);   // BAK (hoy Y)
    }
}
// En guide-finne, dir=+1 front (lav Y) / dir=-1 bak (hoy Y). Staar i skipslaskens
// tunge-kanal ved siden av brettkanten:
//   - TYKK finne fra gulvet til brett-topp (split_z), Y fra ut-i-tunge-kanalen (anker
//     mot full vegg under lask-sonen) til fangeflaten frame_clr fra brettkanten. Fanger
//     brettkanten lateralt over hele brett-tykkelsen.
//   - TYNN lead-in lippe over brett-toppen (kun kavitet-siden, Y >= vegg-innerflate, saa
//     den klarerer lokk-veggen), med fas. Lokk-tunga skjaeres bort der finnen staar
//     (frame_tongue_notch) saa finnen gaar klar naar lokket lukkes.
module frame_rib(cx, dir) {
    x0     = cx - frame_rib_l/2;
    w_in   = (dir > 0) ? wall : wall + inner_h;          // cavity-vegg innerflate
    ycap   = w_in + dir * (clr - frame_clr);             // fangeflate (frame_clr fra brettkant)
    yreach = w_in - dir * frame_fin_reach;               // ut i tunge-kanalen (forankring)
    z_cham = split_z + frame_proud - frame_cham;
    flo = min(yreach, ycap); fhi = max(yreach, ycap);    // tykk finne Y-omraade
    clo = min(w_in, ycap);   chi = max(w_in, ycap);      // tynn lippe Y-omraade (cavity-side)
    union() {
        // tykk finne: gulv -> brett-topp (forankret i veggen under lask-sonen)
        translate([x0, flo, floor_t]) cube([frame_rib_l, fhi - flo, split_z - floor_t]);
        // tynn lead-in lippe over brettet (kavitet-side, klarerer lokk-veggen)
        translate([x0, clo, split_z - eps]) cube([frame_rib_l, chi - clo, z_cham - (split_z - eps)]);
        // fas paa lippa, smalner inn mot vegg-innerflaten
        y_top = (dir > 0) ? w_in : w_in - eps;
        hull() {
            translate([x0, clo, z_cham])                        cube([frame_rib_l, chi - clo, eps]);
            translate([x0, y_top, split_z + frame_proud - eps]) cube([frame_rib_l, eps, eps]);
        }
    }
}
// Skjaeres fra LOKKET: fjerner skipslask-tunga der base-finnene staar, saa finnene gaar
// klar naar lokket lukkes. Hakkene er frame_notch_clr storre enn finnene paa alle kanter.
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
    yreach = w_in - dir * frame_fin_reach;
    nlo = min(yreach, ycap) - frame_notch_clr;
    nhi = max(yreach, ycap) + frame_notch_clr;
    translate([x0, nlo, split_z - lap - eps]) cube([L, nhi - nlo, lap + 2*eps]);
}

// Solide stoettepilarer i hjornene UTEN skruehull. Samme hoyde som standoffs,
// uten pilothull eller pinne - PCB-en hviler bare paa toppen av dem.
module pcb_corner_supports() {
    for (p = pcb_supports)
        translate([bx(p[0]), by(p[1]), floor_t])
            cylinder(h = standoff_h, d = standoff_d);
}

// Skjaerer en sylindrisk lomme ned fra PCB-undersiden der TH-loddetinn stikker
// ned, slik at pilarer i naerheten ikke presser mot loddepunktet.
module th_keepout_pockets() {
    for (k = th_keepouts)
        translate([bx(k[0]), by(k[1]), pcb_z - th_keepout_depth])
            cylinder(h = th_keepout_depth + eps, d = k[2]);
}

module standoffs() {
    for (h = mount_holes) {
        difference() {
            // stolpe: bredere ved heat-set (trenger vegg rundt innsatsen)
            translate([bx(h[0]), by(h[1]), floor_t])
                cylinder(h = standoff_h, d = (closure == "screw") ? anchor_boss_d : standoff_d);
            // hull fra stolpetoppen: heat-set-lomme eller selvgjengende pilot
            if (closure == "screw")
                translate([bx(h[0]), by(h[1]), pcb_z - anchor_hole_h])
                    cylinder(h = anchor_hole_h + eps, d = anchor_hole_d);
        }
        // lokaliseringspinne kun i snap-modus (skruen lokaliserer ellers)
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

/* ============================== LOKK ============================= */
module lid() {
    difference() {
        union() {
            difference() {
                union() {
                    // topp + vegger fra skillet og opp
                    translate([0, 0, split_z]) linear_extrude(lid_h) outer2D();
                    // indre leppe ned i basen (skipslask)
                    translate([0, 0, split_z - lap]) linear_extrude(lap)
                        difference() { offset(-lap_gap) mid2D(); inner2D(); }
                    // snap-perle paa leppa (kun snap-modus)
                    if (closure == "snap") snap_bead_band();
                }
                // hulrom under topp-plata
                translate([0, 0, split_z - lap - eps])
                    linear_extrude(comp_clr + lap + eps) inner2D();
            }
            // legges til ETTER hulrom (ellers fjernes de):
            if (sw_enable && sw_screw) switch_bosses();
            holddowns();
        }
        // alle aapninger kuttes til slutt (gjennom vegg + holder)
        cut_usb();
        cut_xh();
        cut_led();
        if (sw_enable) switch_slot();
        if (sw_enable && sw_screw) switch_screws();
        if (closure == "screw") screw_holes(); else holddown_pinholes();
        // frigang for komponentlegemer over PCB (bosser viker for USB-C-kontakt o.l.)
        comp_keepout_volumes();
        // hakk i lokk-tunga der base-stolpene smelter inn i veggen (ellers treffer de tunga)
        if (closure == "screw") boss_tongue_relief();
        // hakk i lokk-tunga der PCB-ramme-finnene staar (ellers gaar ikke lokket ned)
        frame_tongue_notch();
    }
}

// Kutter ut komponent-frigangene (full hoyde over PCB) saa lokk-bosser ikke
// kolliderer med komponentlegemer som USB-C-kontakten.
module comp_keepout_volumes() {
    for (k = comp_keepouts)
        translate([bx(k[0]), by(k[1]), pcb_top_z - eps])
            cube([k[2] - k[0], k[3] - k[1], (total_h - lid_top_t) - pcb_top_z + 2*eps]);
}

// Frigjor lokk-tunga (skipslask) der base-stolpene ved monteringshullene er saa store
// (Ø6 heat-set-boss) at de stikker forbi kavitetsveggen og inn i tunge-kanalen. Uten
// dette kolliderer tunga med stolpen og lokket lar seg ikke tre paa. Kuttet ligger KUN
// i lask-sonen (under split_z) - over split_z er tunga borte uansett. Skruen holder
// innrettingen i hjornene, saa et lite hakk i tunga her gjor ingen skade.
module boss_tongue_relief() {
    relief_d = anchor_boss_d + 2*lap_gap + 0.3;   // stolpediameter + glidklaring
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
// snap-modus: klaring for lokaliseringspinnen
module holddown_pinholes() {
    for (h = mount_holes)
        translate([bx(h[0]), by(h[1]), pcb_top_z - eps])
            cylinder(h = total_h, d = 2.6);
}
// skruemodus: klaringshull gjennom boss + forsenket hode i toppen
module screw_holes() {
    for (h = mount_holes) {
        translate([bx(h[0]), by(h[1]), pcb_top_z - eps])
            cylinder(h = total_h, d = screw_clear);
        translate([bx(h[0]), by(h[1]), total_h - head_h])
            cylinder(h = head_h + eps, d = head_d);
    }
}

/* =========================== AAPNINGER =========================== */
// tynn skive i YZ-planet, sentrert i (cy, zc), for hull() langs X
module usb_slab(cy, zc, w, h)
    translate([0, cy - w/2, zc - h/2]) cube([eps, w, h]);

// rounded-rectangle hull i YZ-planet, ekstrudert langs +X fra x0, lengde L
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
    cy = by(22 - 13.259);            // USB1 senter
    zc = pcb_top_z + h_usbc/2;       // senter paa kontaktmunningen
    // 1) Gjennomgangshull 11x7 R2 gjennom HELE veggen. Ytterflata er leppa som
    //    stopper kabelkroppen; metallpluggen (8.34x2.56) gaar greit igjennom.
    usb_rrect_cut(cy, zc, usb_open_w, usb_open_h, usb_open_r, -1, wall + 2);
    // 2) Innvendig innsenkning 13x9 R3 paa cavity-siden av veggen: spiser inn
    //    de siste usb_recess_d mm av veggen, slik at USB-C-skallet/-kroppen kan
    //    stikke inn i veggen og kontaktmunningen kommer naer ytterflata.
    if (usb_recess)
        usb_rrect_cut(cy, zc, usb_recess_w, usb_recess_h, usb_recess_r,
                      wall - usb_recess_d, usb_recess_d + 1);  // +1 ut i hulrom
    // 3) Valgfri 45-grader fas paa ytterflata som lead-in (av som default).
    if (usb_chamfer > 0) {
        hull() {
            translate([-eps, 0, 0]) usb_slab(cy, zc, usb_open_w + 2*usb_chamfer,
                                                       usb_open_h + 2*usb_chamfer);
            translate([usb_chamfer, 0, 0]) usb_slab(cy, zc, usb_open_w, usb_open_h);
        }
    }
}
module cut_xh() {
    // liten avrundet kabelgjennomfoering i hoyre vegg (ikke hele kontakten)
    cy = by(22 - 11.0);                 // U4 senter
    zc = pcb_top_z + cable_port_z;
    r  = cable_port_h / 2;
    dy = max((cable_port_w - cable_port_h) / 2, 0);
    hull() for (s = [-1, 1])
        translate([outer_w - wall - 1, cy + s*dy, zc])
            rotate([0, 90, 0]) cylinder(h = wall + 2, r = r);
}
// Lokk-utskjaering for lyslederen: gjennomgangshull (stav) + topp-forsenkning (hode).
module cut_led() {
    if (led_show) {
        // staven-hull gjennom hele topp-plata (vindu)
        translate([led_cx, led_cy, total_h - lid_top_t - eps])
            linear_extrude(lid_top_t + 2*eps)
                rrect_c(led_win_l + led_pipe_clr, led_win_w + led_pipe_clr, led_win_r);
        // topp-forsenkning for hodet (sa hodet flukter med toppflaten)
        translate([led_cx, led_cy, total_h - led_head_t])
            linear_extrude(led_head_t + eps)
                rrect_c(led_win_l + 2*led_head_lip + led_pipe_clr,
                        led_win_w + 2*led_head_lip + led_pipe_clr,
                        led_win_r + led_head_lip);
    }
}

// Selve lyslederen (printes separat i KLART filament, settes inn ovenfra).
// Topp-hatt: bredt hode i forsenkningen (flukter med toppen) + stav ned mot LED-ene.
module light_pipe() {
    z_bot = pcb_top_z + led_pipe_gap;          // like over LED-toppen
    z_cb  = total_h - led_head_t;              // forsenkningsbunn (hode hviler her)
    union() {
        // hode (full storrelse, klaring kommer fra forsenkningen)
        translate([led_cx, led_cy, z_cb])
            linear_extrude(led_head_t)
                rrect_c(led_win_l + 2*led_head_lip, led_win_w + 2*led_head_lip,
                        led_win_r + led_head_lip);
        // stav: fra forsenkningsbunn ned til like over LED-ene
        translate([led_cx, led_cy, z_bot])
            linear_extrude(z_cb - z_bot + eps)
                rrect_c(led_win_l, led_win_w, led_win_r);
    }
}

/* ========================== SKYVEBRYTER ========================== */
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
// 2 skruebosser som flankerer aktuatoren. Vertikale rektangulaere PILARER som
// gaar fra skruesenter HELT OPP i lokk-taket -> skruekraften tas opp av taket,
// ikke av den tynne veggen. Pilarens ytterside er sw_ear_t inne fra veggen, slik
// at bryterens ore klemmes inn i mellomrommet. (Legges til lokk-solid.)
module switch_bosses() {
    sx       = bx(sw_x);
    zc       = split_z + sw_z + sw_screw_dz;
    front    = (sw_wall == "front");
    y_inner  = front ? wall + sw_ear_t : outer_h - wall - sw_ear_t;
    pillar_w = sw_boss_d;                 // X-bredde rundt skruehullet
    pillar_d = sw_boss_h;                 // Y-dybde innover fra (nesten) veggen
    // PCB-toppen er ved split_z; pilaren starter sw_boss_gap over og gaar opp i taket.
    // Gapet klarerer komponentene under bossen (Q1 SOT-23 m.fl.); resten av hoyden
    // gir stivhet i sideretningen naar skruen strammes.
    z_bot    = split_z + sw_boss_gap;
    z_top    = total_h - lid_top_t + 1;   // 1mm overlapp -> fusjonerer med topp-platen
    h_pillar = z_top - z_bot;
    for (s = [-1, 1]) {
        x = sx + s * sw_screw_pitch / 2;
        if (front)
            translate([x - pillar_w/2, y_inner,             z_bot])
                cube([pillar_w, pillar_d, h_pillar]);
        else
            translate([x - pillar_w/2, y_inner - pillar_d,  z_bot])
                cube([pillar_w, pillar_d, h_pillar]);
    }
}

// klaringshull gjennom vegg + forsenket hode utvendig + pilothull i boss (kuttes)
module switch_screws() {
    sx    = bx(sw_x);
    zc    = split_z + sw_z + sw_screw_dz;
    front = (sw_wall == "front");
    for (s = [-1, 1]) {
        x = sx + s*sw_screw_pitch/2;
        if (front) {
            translate([x, -1, zc])                  rotate([-90,0,0]) cylinder(h = wall + sw_ear_t + 1, d = sw_screw_d);
            translate([x, -eps, zc])                rotate([-90,0,0]) cylinder(h = sw_screw_head_h + eps, d = sw_screw_head_d);
            translate([x, wall + sw_ear_t - eps, zc]) rotate([-90,0,0]) cylinder(h = sw_boss_h + eps, d = sw_boss_pilot);
        } else {
            translate([x, outer_h - wall - sw_ear_t - 1, zc]) rotate([-90,0,0]) cylinder(h = wall + sw_ear_t + 1, d = sw_screw_d);
            translate([x, outer_h - sw_screw_head_h, zc])     rotate([-90,0,0]) cylinder(h = sw_screw_head_h + eps, d = sw_screw_head_d);
            translate([x, outer_h - wall - sw_ear_t + eps, zc]) rotate([90,0,0]) cylinder(h = sw_boss_h + eps, d = sw_boss_pilot);
        }
    }
}

// enkel proxy av bryteren for preview (kun visuell sjekk av plass)
module switch_preview() {
    sx = bx(sw_x); zc = split_z + sw_z;
    color([0.12, 0.12, 0.12, 0.85]) {
        translate([sx - 5.5, wall, zc - 3.5]) cube([11, sw_body_w, 7]);     // kropp
        translate([sx - 1.5, -2, zc - 1.5]) cube([3, wall + 2.5, 3]);       // aktuator
        for (s = [-1, 1])
            translate([sx + s*sw_screw_pitch/2 - 3, wall, zc - 1]) cube([6, 0.6, 2]); // ore
    }
}

/* ===================== PREVIEW: PCB + komponenter ================= */
module pcb() {
    color([0.0, 0.45, 0.18, 0.55])
        translate([off, off, pcb_z]) linear_extrude(pcb_t)
            translate([0, 0]) rrect(board_w, board_h, board_r);
}
module comp(cx, cy, sx, sy, sz)
    translate([bx(cx) - sx/2, by(22 + cy) - sy/2, 0]) cube([sx, sy, sz]);
module components() {
    color([0.25, 0.25, 0.28, 0.7]) translate([0, 0, pcb_top_z]) {
        comp(36.661, -16.242, 7.0, 11.0, h_relay);  // K1 relé
        comp(36.760,  -5.686, 11.0, 7.0, h_relay);  // K2 relé
        comp(44.3,   -11.0,   6.0, 12.0, h_xh);      // U4 XH-plugg
        // USB-C: kropp 8.95x3.2 paa PCB (venstre kant 4.8mm inn fra PCB-venstrekant),
        // skallet stikker ~6.8mm forbi kropp-venstre, totalt 15.75mm X-extent fra
        // board X=-2 til X=13.75 (sentrert 5.875). Y-bredde 3.2mm.
        comp(5.875,  -13.259, 15.75, 3.2, h_usbc);   // USB1 (kropp + skall)
        comp(3.275,   -4.811, 4.0, 6.0,  h_ph);      // BAT1
        comp(28.713,  -2.04,  6.0, 4.0,  h_ph);      // S3
        comp(21.257, -14.419, 5.0, 5.0,  1.0);       // ESP32 U1
    }
    // LED-er (D3 rod, D4 bla) - 0603, lyser opp
    color([1.0, 0.1, 0.1]) translate([0,0,pcb_top_z]) comp(22.514, -1.814, 1.6, 0.8, 0.6);
    color([0.1, 0.3, 1.0]) translate([0,0,pcb_top_z]) comp(24.224, -1.814, 1.6, 0.8, 0.6);
}

/* ============================= RENDER ============================ */
if (part == "base") base();
else if (part == "lid") lid();
else if (part == "lightpipe") light_pipe();         // klar innsats - print i klart filament
else if (part == "both") { base(); translate([0, outer_h + 8, 0]) lid(); }
else {                      // preview: montert, lokk gjennomskinnelig
    base();
    if (show_pcb) { pcb(); components(); }
    if (sw_enable && sw_screw) switch_preview();
    color([0.6, 0.6, 0.65, 0.25]) lid();
    if (led_show) color([1, 1, 1, 0.55]) light_pipe();   // lysleder p plass
}
