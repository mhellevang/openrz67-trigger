# /// script
# requires-python = ">=3.12"
# dependencies = ["build123d>=0.11.1"]
# ///
"""Board + case as GLB for CAD Viewer: closed, and with the floor slid out.

    uv run view.py <board.step> <outdir>
"""

import runpy
import sys
from pathlib import Path

from build123d import Color, Compound, Pos, Rot, export_gltf, import_step

here = Path(__file__).resolve().parent
g = runpy.run_path(str(here / "openrz67_nrf_case.py"), run_name="case")
step, out = sys.argv[1], Path(sys.argv[2])


def col(p, c, a=1.0):
    p.color = Color(*c, a)
    return p


board = Pos(g["wall"] + g["clr"] + g["board_w"], g["wall"] + g["ant_wall_clr"] + g["clr"], g["pcb_z"]) * Rot(0, 0, 180) * import_step(step)
for name, floor_dy in (("nrf-closed", 0), ("nrf-floor-out", 25)):
    # Pos(0, 0, 0) copies: a shape can only be in one Compound
    parts = [col(Pos(0, floor_dy, 0) * g["base"], (0.23, 0.23, 0.24)),
             col(Pos(0, 0, 0) * g["lid"], (0.6, 0.6, 0.62), 0.35),
             col(Pos(0, 0, 0) * g["lightpipe"], (0.9, 0.9, 1), 0.6),
             Pos(0, 0, 0) * board]
    export_gltf(Compound(children=parts), str(out / f"{name}.glb"), binary=True)
    print(out / f"{name}.glb")
