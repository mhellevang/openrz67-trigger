#!/usr/bin/env python3
"""
Build a Bambu Studio / OrcaSlicer project .3mf by swapping fresh STL geometry
into a hand-made template project, keeping ALL slicer settings intact:
print profile, plate layout, and the per-part filament assignment (the light
pipe stays on its clear filament). Only the three mesh blocks are replaced.

The template was made once by hand in the slicer (import the STLs, arrange,
assign the light pipe to the clear filament, save project). After that, run
this script whenever the geometry changes instead of redoing that by hand.

How placement is preserved: each object's mesh lives in its own local frame,
centred by the slicer on import (model_vertex = stl_vertex - bbox_centre). The
slicer records that centre as source_offset_{x,y,z} in model_settings.config.
We reproduce that import by recentring each fresh mesh on ITS OWN bounding-box
centre and writing that centre back into source_offset. Recomputing (rather than
reusing the template's stored offset) matters: if the SCAD geometry changes size
later - e.g. back_margin grew the case in Y - the stored offset goes stale and
the part would load shifted off its plate spot. A fresh centre keeps base, lid
and light pipe aligned with each other and on the plate, and inherits the same
filament/extruder mapping.

Usage:
    python3 make_3mf.py [--template bambu-template.3mf] [--stl-dir stl]
                        [--out openrz67-case.3mf]

Pure stdlib (no numpy). ASCII STL in, project .3mf out.
"""
import argparse
import datetime
import os
import re
import shutil
import sys
import tempfile
import zipfile


def parse_ascii_stl(path):
    """Return (vertices, triangles): unique vertex list + index triples.

    Vertices are deduplicated by rounded key so the 3mf gets a shared-vertex
    mesh (smaller, and what a hand mesh looks like). Winding is preserved from
    the STL facet order, so outward normals carry over.
    """
    verts = []
    index = {}
    tris = []
    cur = []
    with open(path, "r") as f:
        for line in f:
            s = line.strip()
            if s.startswith("vertex"):
                _, x, y, z = s.split()
                cur.append((float(x), float(y), float(z)))
                if len(cur) == 3:
                    tri = []
                    for v in cur:
                        key = (round(v[0], 6), round(v[1], 6), round(v[2], 6))
                        i = index.get(key)
                        if i is None:
                            i = len(verts)
                            index[key] = i
                            verts.append(v)
                        tri.append(i)
                    tris.append(tuple(tri))
                    cur = []
    if cur:
        raise ValueError(f"{path}: dangling vertices (not a multiple of 3)")
    return verts, tris


def bbox_center(verts):
    """Centre of the mesh's axis-aligned bounding box (what the slicer centres on)."""
    xs = [v[0] for v in verts]
    ys = [v[1] for v in verts]
    zs = [v[2] for v in verts]
    return ((min(xs) + max(xs)) / 2, (min(ys) + max(ys)) / 2, (min(zs) + max(zs)) / 2)


def mesh_xml(verts, tris, offset):
    """Build the <mesh> block, translating each vertex by -offset."""
    ox, oy, oz = offset
    out = ["   <mesh>", "    <vertices>"]
    for x, y, z in verts:
        out.append(
            f'     <vertex x="{x - ox:.8g}" y="{y - oy:.8g}" z="{z - oz:.8g}"/>'
        )
    out.append("    </vertices>")
    out.append("    <triangles>")
    for a, b, c in tris:
        out.append(f'     <triangle v1="{a}" v2="{b}" v3="{c}"/>')
    out.append("    </triangles>")
    out.append("   </mesh>")
    return "\n".join(out)


def build_mapping(model_settings, dmodel):
    """object id -> dict(name, offset, model_path), joined across the two files."""
    objs = {}
    for m in re.finditer(r'<object id="(\d+)">(.*?)</object>', model_settings, re.S):
        oid, body = m.group(1), m.group(2)
        name = re.search(r'key="name" value="([^"]+)"', body)
        ox = re.search(r'source_offset_x" value="([-0-9.eE]+)"', body)
        oy = re.search(r'source_offset_y" value="([-0-9.eE]+)"', body)
        oz = re.search(r'source_offset_z" value="([-0-9.eE]+)"', body)
        if not (name and ox and oy and oz):
            continue
        objs[oid] = {
            "name": name.group(1),
            "offset": (float(ox.group(1)), float(oy.group(1)), float(oz.group(1))),
        }
    for m in re.finditer(
        r'<object id="(\d+)"[^>]*>\s*<components>\s*<component p:path="([^"]+)"',
        dmodel, re.S,
    ):
        oid, path = m.group(1), m.group(2)
        if oid in objs:
            objs[oid]["model_path"] = path.lstrip("/")
    return objs


def main():
    ap = argparse.ArgumentParser(description="Swap STL geometry into a Bambu/Orca project 3mf.")
    here = os.path.dirname(os.path.abspath(__file__))
    ap.add_argument("--template", default=os.path.join(here, "bambu-template.3mf"))
    ap.add_argument("--stl-dir", default=os.path.join(here, "stl"))
    ap.add_argument("--out", default=os.path.join(here, "openrz67-case.3mf"))
    args = ap.parse_args()

    if not os.path.isfile(args.template):
        sys.exit(f"template not found: {args.template}")

    work = tempfile.mkdtemp(prefix="mk3mf_")
    try:
        with zipfile.ZipFile(args.template) as z:
            names = z.namelist()
            z.extractall(work)

        ms_path = os.path.join(work, "Metadata", "model_settings.config")
        dm_path = os.path.join(work, "3D", "3dmodel.model")
        model_settings = open(ms_path).read()
        dmodel = open(dm_path).read()
        mapping = build_mapping(model_settings, dmodel)

        for oid, info in mapping.items():
            stl = os.path.join(args.stl_dir, info["name"])
            if not os.path.isfile(stl):
                sys.exit(f"STL missing for object {oid}: {stl}")
            verts, tris = parse_ascii_stl(stl)
            # Recentre on THIS mesh's own bbox centre (a fresh slicer import), not the
            # template's stored offset, which goes stale when the geometry changes size.
            offset = bbox_center(verts)
            block = mesh_xml(verts, tris, offset)

            mfile = os.path.join(work, info["model_path"])
            txt = open(mfile).read()
            new_txt, n = re.subn(r"   <mesh>.*?</mesh>", block, txt, count=1, flags=re.S)
            if n != 1:
                sys.exit(f"could not locate <mesh> in {info['model_path']}")
            open(mfile, "w").write(new_txt)

            # keep the slicer's metadata honest: face count AND the recorded source offset
            # (the latter so it matches the centre we just recentred the mesh on)
            fc = len(tris)
            model_settings = re.sub(
                rf'(<object id="{oid}">.*?face_count=")\d+(")',
                rf"\g<1>{fc}\g<2>", model_settings, count=1, flags=re.S,
            )
            model_settings = re.sub(
                rf'(<object id="{oid}">.*?<mesh_stat face_count=")\d+(")',
                rf"\g<1>{fc}\g<2>", model_settings, count=1, flags=re.S,
            )
            for axis, val in zip("xyz", offset):
                model_settings = re.sub(
                    rf'(<object id="{oid}">.*?source_offset_{axis}" value=")[-0-9.eE]+(")',
                    rf"\g<1>{val:.8g}\g<2>", model_settings, count=1, flags=re.S,
                )
            cx, cy, cz = offset
            print(f"  {info['name']:30} {len(verts)} verts / {fc} tris "
                  f"@ ({cx:.3f},{cy:.3f},{cz:.3f})  -> {info['model_path']}")

        open(ms_path, "w").write(model_settings)

        # refresh the dates so the slicer doesn't show a stale project date
        today = datetime.date.today().isoformat()
        dmodel = re.sub(r'(<metadata name="ModificationDate">)[^<]*(</metadata>)',
                        rf"\g<1>{today}\g<2>", dmodel)
        open(dm_path, "w").write(dmodel)

        # rewrite the zip, preserving entry order (Content_Types first)
        tmp_out = args.out + ".tmp"
        with zipfile.ZipFile(tmp_out, "w", zipfile.ZIP_DEFLATED) as z:
            for name in names:
                z.write(os.path.join(work, name), name)
        os.replace(tmp_out, args.out)
        print(f"Wrote {args.out}")
    finally:
        shutil.rmtree(work, ignore_errors=True)


if __name__ == "__main__":
    main()
