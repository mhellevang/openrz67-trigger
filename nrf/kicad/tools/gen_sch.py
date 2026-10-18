"""Write openrz67-nrf.kicad_sch from design.py.

A netlist-style schematic: symbols sit in functional groups, every connected pin gets a
short wire stub and a global label with the net name, every unused pin a no-connect. Pins come from the
project symbol library, so the schematic always matches the board the same model built.

  python3 tools/gen_sch.py
"""
import os, re, sys, uuid

sys.path.insert(0, os.path.dirname(__file__))
import design as D

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LIBFILE = os.path.join(HERE, f"{D.LIB}.kicad_sym")
SCH = os.path.join(HERE, "openrz67-nrf.kicad_sch")
PROJECT_UUID = "5d1c1d2e-0a3b-4c4d-9e5f-6a7b8c9d0e1f"

# ref -> library symbol name
SYMBOL = {
    "U1": "E73-2G4M08S1F", "BT1": "MY-2032-16", "J1": "SM04B-SRSS-TB",
    "U2": "TLP172AM", "U3": "TLP172AM", "R1": "R", "R2": "R", "R3": "R",
    "C1": "CL31A107MQHNNNE", "C2": "GRM155R71H104KE14D", "C3": "GRM155R71H104KE14D", "J2": "Conn_01x04",
    "SW1": "B3U-1000P", "D1": "KT-0603R", "Q1": "AO3401A",
}
# sheet positions (mm, snapped to the 2.54 grid), symbol rotation (CCW)
PLACE = {
    "BT1": (45, 32, 0), "Q1": (95, 32, 0), "C1": (140, 32, 0), "C2": (180, 32, 0), "C3": (220, 32, 0),
    "J2": (265, 32, 0),
    "U1": (65, 108, 0),
    "R1": (135, 90, 90), "U2": (190, 90, 0), "R2": (135, 122, 90), "U3": (190, 122, 0), "J1": (250, 106, 90),
    "SW1": (40, 165, 0), "R3": (95, 165, 90), "D1": (140, 165, 180),
}
FLAGS = (("VBAT", 140, 50), ("VDD", 170, 50), ("GND", 200, 50))
# (title, x0, y0, x1, y1)
GROUPS = [
    ("Power, reverse-cell block", 15, 16, 240, 68),
    ("Debug", 243, 16, 284, 68),
    ("MCU", 15, 71, 110, 144),
    ("Shutter: PhotoMOS relays, camera port", 113, 71, 284, 144),
    ("Button, status LED", 15, 147, 174, 182),
]
STUB = 5.08


def tokenize(s):
    return re.findall(r'"(?:[^"\\]|\\.)*"|[()]|[^\s()"]+', s)


def parse(tokens, i=0):
    out = []
    while i < len(tokens):
        t = tokens[i]
        if t == "(":
            sub, i = parse(tokens, i + 1); out.append(sub)
        elif t == ")":
            return out, i + 1
        else:
            out.append(t[1:-1] if t.startswith('"') else t); i += 1
    return out, i


def symbol_blocks(text):
    """Top-level symbol blocks of a .kicad_sym as (name, raw_text)."""
    for m in re.finditer(r'^\(symbol "([^"]+)"', text, re.M):
        depth = 0; j = m.start()
        while True:
            c = text[j]
            if c == '(': depth += 1
            elif c == ')':
                depth -= 1
                if depth == 0: break
            j += 1
        yield m.group(1), text[m.start():j + 1]


def pins(block):
    """(number, x, y, angle) per pin, all units, lib coordinates (y up)."""
    tree, _ = parse(tokenize(block))
    found = []
    def walk(node):
        if isinstance(node, list):
            if node and node[0] == "pin":
                at = next(n for n in node if isinstance(n, list) and n[0] == "at")
                num = next(n for n in node if isinstance(n, list) and n[0] == "number")[1]
                found.append((num, float(at[1]), float(at[2]), float(at[3]) if len(at) > 3 else 0.0))
            for n in node: walk(n)
    walk(tree)
    return found


def pin_names(block):
    """(number, name) per pin."""
    tree, _ = parse(tokenize(block))
    found = []
    def walk(node):
        if isinstance(node, list):
            if node and node[0] == "pin":
                name = next(n for n in node if isinstance(n, list) and n[0] == "name")[1]
                num = next(n for n in node if isinstance(n, list) and n[0] == "number")[1]
                found.append((num, name))
            for n in node: walk(n)
    walk(tree)
    return found


def graphics(block):
    """Body outline points (lib coordinates) of a symbol."""
    return [(float(a), float(b)) for a, b in
            re.findall(r'\((?:xy|start|end|mid|center) (-?[\d.]+) (-?[\d.]+)\)', block)]


def rot(x, y, deg):
    """Rotate a sheet vector (y down) CCW on screen."""
    return {0: (x, y), 90: (y, -x), 180: (-x, -y), 270: (-y, x)}[deg % 360]


def snap(v):
    return round(round(v / 2.54) * 2.54, 4)


def u():
    return str(uuid.uuid4())


def esc(s):
    return s.replace('\\', '\\\\').replace('"', '\\"')


def main():
    lib = open(LIBFILE, encoding="utf-8").read()
    blocks = dict(symbol_blocks(lib))
    for name in set(SYMBOL.values()) | {"PWR_FLAG"}:
        if name not in blocks:
            raise SystemExit(f"symbol {name} missing from {LIBFILE}")
    padnet = {(ref, pad): net for net, pl in D.NETS.items() for ref, pad in pl}

    out = []
    w = out.append
    w('(kicad_sch (version 20250114) (generator "gen_sch") (generator_version "10.0")')
    w(f'  (uuid "{PROJECT_UUID}")')
    w('  (paper "A4")')
    w('  (title_block (title "OpenRZ67 trigger, nRF52 / coin cell") (rev "A") (company "openrz67"))')
    w('  (lib_symbols')
    for name in sorted(set(SYMBOL.values()) | {"PWR_FLAG"}):
        blk = blocks[name].replace(f'(symbol "{name}"', f'(symbol "{D.LIB}:{name}"', 1)
        w("    " + blk.replace("\n", "\n    "))
    w('  )')

    labels, ncs, wires = [], [], []
    for ref, (fp, value, lcsc, mpn, mfr, layer, *_rest, descr) in D.PARTS.items():
        name = SYMBOL[ref]; X, Y, R = PLACE[ref]; X, Y = snap(X), snap(Y)
        body = [rot(gx, -gy, R) for gx, gy in graphics(blocks[name])]
        pinlist = []
        for num, px, py, pa in pins(blocks[name]):
            rx, ry = rot(px, -py, R)        # lib y up -> sheet y down
            body.append((rx, ry))
            la = int((pa + 180 + R) % 360)  # outward from the body: 0 right, 90 up, 180 left, 270 down
            pinlist.append((num, rx, ry, la))
        top, bot = min(y for _, y in body), max(y for _, y in body)
        sides = {la for *_, la in pinlist}
        if 90 not in sides and 270 not in sides:
            ref_at, val_at = (X, Y + top - 2.54), (X, Y + bot + 2.54)
        elif 90 not in sides:
            ref_at, val_at = (X, Y + top - 5.08), (X, Y + top - 2.54)
        else:
            ref_at, val_at = (X, Y + bot + 2.54), (X, Y + bot + 5.08)
        w(f'  (symbol (lib_id "{D.LIB}:{name}") (at {X} {Y} {R}) (unit 1)')
        w(f'    (in_bom {"yes" if lcsc else "no"}) (on_board yes) (dnp no)')
        w(f'    (uuid "{u()}")')
        props = [("Reference", ref, False, ref_at), ("Value", value, False, val_at),
                 ("Footprint", f"{D.LIB}:{fp}", True, (X, Y)), ("Datasheet", "", True, (X, Y)),
                 ("Description", descr, True, (X, Y)), ("LCSC", lcsc, True, (X, Y)),
                 ("MPN", mpn, True, (X, Y)), ("Manufacturer", mfr, True, (X, Y))]
        for k, v, hide, (tx, ty) in props:
            w(f'    (property "{k}" "{esc(v)}" (at {round(tx, 4)} {round(ty, 4)} {R if R in (90, 270) else 0})')
            w(f'      (effects (font (size 1.27 1.27)){" hide" if hide else ""})')
            w('    )')
        for num, rx, ry, la in pinlist:
            w(f'    (pin "{num}" (uuid "{u()}"))')
            lx, ly = round(X + rx, 4), round(Y + ry, 4)
            net = padnet.get((ref, num))
            if net:
                dx, dy = rot(STUB, 0, la)
                ex, ey = round(lx + dx, 4), round(ly + dy, 4)
                wires.append((lx, ly, ex, ey))
                labels.append((net, ex, ey, la))
            else:
                ncs.append((lx, ly))
        w('    (instances')
        w(f'      (project "openrz67-nrf" (path "/{PROJECT_UUID}" (reference "{ref}") (unit 1)))')
        w('    )')
        w('  )')

    # power flags so ERC sees the cell, VDD (through Q1) and GND driven
    for i, (net, X, Y) in enumerate(FLAGS):
        X, Y = snap(X), snap(Y)
        w(f'  (symbol (lib_id "{D.LIB}:PWR_FLAG") (at {X} {Y} 0) (unit 1)')
        w('    (in_bom no) (on_board no) (dnp no)')
        w(f'    (uuid "{u()}")')
        w(f'    (property "Reference" "#FLG{i+1}" (at {X} {Y - 4} 0) (effects (font (size 1.27 1.27)) hide))')
        w(f'    (property "Value" "PWR_FLAG" (at {X} {Y - 2} 0) (effects (font (size 1.27 1.27))))')
        w(f'    (property "Footprint" "" (at {X} {Y} 0) (effects (font (size 1.27 1.27)) hide))')
        w(f'    (property "Datasheet" "" (at {X} {Y} 0) (effects (font (size 1.27 1.27)) hide))')
        w(f'    (pin "1" (uuid "{u()}"))')
        w(f'    (instances (project "openrz67-nrf" (path "/{PROJECT_UUID}" (reference "#FLG{i+1}") (unit 1))))')
        w('  )')
        wires.append((X, Y, X, round(Y + STUB, 4)))
        labels.append((net, X, round(Y + STUB, 4), 270))

    for title, x0, y0, x1, y1 in GROUPS:
        w(f'  (rectangle (start {x0} {y0}) (end {x1} {y1}) (stroke (width 0) (type dash)) (fill (type none)) (uuid "{u()}"))')
        w(f'  (text "{esc(title)}" (exclude_from_sim no) (at {x0 + 2} {y0 + 4} 0)')
        w(f'    (effects (font (size 2 2) (bold yes)) (justify left bottom)) (uuid "{u()}"))')
    for x0, y0, x1, y1 in wires:
        w(f'  (wire (pts (xy {x0} {y0}) (xy {x1} {y1})) (stroke (width 0) (type default)) (uuid "{u()}"))')

    for net, x, y, a in labels:
        just = {0: "left", 180: "right", 90: "left", 270: "right"}[a]
        w(f'  (global_label "{net}" (shape passive) (at {x} {y} {a}) (fields_autoplaced yes)')
        w(f'    (effects (font (size 1.27 1.27)) (justify {just}))')
        w(f'    (uuid "{u()}")')
        w('  )')
    for x, y in ncs:
        w(f'  (no_connect (at {x} {y}) (uuid "{u()}"))')
    w('  (sheet_instances (path "/" (page "1")))')
    w('  (embedded_fonts no)')
    w(')')
    open(SCH, "w", encoding="utf-8").write("\n".join(out) + "\n")
    print("saved", SCH, f"{len(labels)} labels, {len(ncs)} no-connects")


if __name__ == "__main__":
    main()
