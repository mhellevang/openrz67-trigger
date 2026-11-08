#!/usr/bin/env python3
"""Blender helpers and the shared look (camera, lights, backdrop) for render_nrf.py."""

import math
import os
import sys
from pathlib import Path

import bpy
from mathutils import Vector

# --- paths -----------------------------------------------------------------
HERE = Path(__file__).resolve().parent
DOCS = HERE.parent
REPO = DOCS.parent

# --- look ------------------------------------------------------------------
# The user prints the case white with black text. Pure white blows out against
# a light page background, so the case is a warm near-white that
# still reads as white, and the inlay is the same near-black as the page ink.
C_CASE = (0.88, 0.87, 0.85)
C_TEXT = (0.05, 0.05, 0.05)
C_BG = (0.957, 0.937, 0.902)  # --bg  #f4efe6
RES = int(os.environ.get("RENDER_RES", 1400))
SAMPLES = int(os.environ.get("RENDER_SAMPLES", 160))


def log(msg):
    print(f"[render] {msg}", file=sys.stderr, flush=True)


def clear_scene():
    bpy.ops.wm.read_factory_settings(use_empty=True)


def import_stl(path):
    before = set(bpy.data.objects)
    # Blender 4.x ships the fast C++ importer; 3.x only has the Python one.
    if hasattr(bpy.ops.wm, "stl_import"):
        bpy.ops.wm.stl_import(filepath=str(path))
    else:
        bpy.ops.import_mesh.stl(filepath=str(path))
    new = [o for o in bpy.data.objects if o not in before]
    if not new:
        raise RuntimeError(f"nothing imported from {path}")
    return new


def import_glb(path):
    before = set(bpy.data.objects)
    bpy.ops.import_scene.gltf(filepath=str(path))
    return [o for o in bpy.data.objects if o not in before]


def bbox(objs):
    lo = Vector((1e9,) * 3)
    hi = Vector((-1e9,) * 3)
    for o in objs:
        if o.type != "MESH":
            continue
        for corner in o.bound_box:
            w = o.matrix_world @ Vector(corner)
            for i in range(3):
                lo[i] = min(lo[i], w[i])
                hi[i] = max(hi[i], w[i])
    return lo, hi


def mat(name, rgb, rough=0.55, metallic=0.0):
    m = bpy.data.materials.new(name)
    m.use_nodes = True
    b = m.node_tree.nodes["Principled BSDF"]
    b.inputs["Base Color"].default_value = (*rgb, 1.0)
    b.inputs["Roughness"].default_value = rough
    b.inputs["Metallic"].default_value = metallic
    return m


def assign(objs, material):
    for o in objs:
        if o.type != "MESH":
            continue
        o.data.materials.clear()
        o.data.materials.append(material)


def shade_smooth(objs, angle_deg=30.0):
    """Smooth only across shallow angles, so printed edges stay crisp.

    Blender 4.1 dropped mesh.use_auto_smooth for a "Smooth by Angle" modifier
    driven by this operator; 5.x is operator-only.
    """
    meshes = [o for o in objs if o.type == "MESH"]
    if not meshes:
        return
    for o in bpy.context.selected_objects:
        o.select_set(False)
    for o in meshes:
        o.select_set(True)
    bpy.context.view_layer.objects.active = meshes[0]
    if hasattr(bpy.ops.object, "shade_smooth_by_angle"):
        bpy.ops.object.shade_smooth_by_angle(angle=math.radians(angle_deg))
    else:
        bpy.ops.object.shade_smooth()
    for o in meshes:
        o.select_set(False)


def move(objs, dz=0.0, delta=None):
    d = Vector(delta) if delta else Vector((0.0, 0.0, dz))
    for o in objs:
        if o.parent is None:
            o.location += d


def shoot(everything, out, blend):
    """Backdrop, camera, lights and Cycles around an already placed scene."""
    scene = bpy.context.scene
    lo, hi = bbox(everything)
    centre = (lo + hi) / 2.0
    radius = max((hi - lo).x, (hi - lo).y, (hi - lo).z) / 2.0
    log(f"scene bbox {tuple(round(v, 1) for v in lo)} .. {tuple(round(v, 1) for v in hi)}")

    # --- backdrop ----------------------------------------------------------
    bpy.ops.mesh.primitive_plane_add(size=1200, location=(centre.x, centre.y, lo.z))
    floor = bpy.context.object
    floor.name = "backdrop"
    assign([floor], mat("backdrop", C_BG, rough=0.95))
    floor.is_shadow_catcher = True

    world = bpy.data.worlds.new("w")
    scene.world = world
    world.use_nodes = True
    world.node_tree.nodes["Background"].inputs[0].default_value = (*C_BG, 1.0)
    world.node_tree.nodes["Background"].inputs[1].default_value = 0.35

    # --- camera ------------------------------------------------------------
    # Three-quarter view from the front-left and above, matching the 2026-09-15
    # framing: USB end to the back-left, camera connector to the front-right.
    az = math.radians(float(os.environ.get("CAM_AZ", -55.0)))
    el = math.radians(float(os.environ.get("CAM_EL", 26.0)))
    lens = float(os.environ.get("CAM_LENS", 85.0))
    # Fit the bounding sphere with a margin, rather than a magic distance.
    fit = radius / math.tan(0.5 * 2.0 * math.atan(18.0 / lens))
    dist = float(os.environ.get("CAM_DIST", fit * float(os.environ.get("CAM_MARGIN", 1.42))))
    target = Vector((centre.x, centre.y, centre.z))
    eye = target + Vector((
        dist * math.cos(el) * math.cos(az),
        dist * math.cos(el) * math.sin(az),
        dist * math.sin(el),
    ))
    cam_data = bpy.data.cameras.new("cam")
    cam_data.lens = lens
    cam = bpy.data.objects.new("cam", cam_data)
    scene.collection.objects.link(cam)
    cam.location = eye
    d = (target - eye).normalized()
    cam.rotation_euler = d.to_track_quat("-Z", "Y").to_euler()
    scene.camera = cam

    # --- lights ------------------------------------------------------------
    def area(name, loc, energy, size, rot=None):
        ld = bpy.data.lights.new(name, type="AREA")
        ld.energy = energy
        ld.size = size
        o = bpy.data.objects.new(name, ld)
        scene.collection.objects.link(o)
        o.location = Vector(loc) + target
        aim = (target - o.location).normalized()
        o.rotation_euler = aim.to_track_quat("-Z", "Y").to_euler()
        return o

    area("key", (-160, -170, 260), 420000, 260)
    area("fill", (220, -120, 90), 110000, 320)
    area("rim", (60, 230, 170), 150000, 200)

    # --- render ------------------------------------------------------------
    scene.render.engine = "CYCLES"
    try:
        scene.cycles.device = "GPU"
        prefs = bpy.context.preferences.addons["cycles"].preferences
        prefs.get_devices()
        for backend in ("METAL", "OPTIX", "CUDA", "HIP", "ONEAPI"):
            try:
                prefs.compute_device_type = backend
                break
            except TypeError:
                continue
        for dev in prefs.devices:
            dev.use = True
        log(f"cycles device: {prefs.compute_device_type}")
    except Exception as exc:  # CPU is fine, just slower
        scene.cycles.device = "CPU"
        log(f"GPU unavailable ({exc}); rendering on CPU")

    scene.cycles.samples = SAMPLES
    scene.cycles.use_denoising = True
    scene.render.film_transparent = True
    scene.render.resolution_x = RES
    scene.render.resolution_y = RES
    scene.render.resolution_percentage = 100
    scene.render.image_settings.file_format = "PNG"
    scene.render.image_settings.color_mode = "RGBA"
    scene.render.image_settings.compression = 90
    # Standard, not AgX/Filmic: those remap the backdrop away from #f4efe6.
    scene.view_settings.view_transform = "Standard"
    scene.view_settings.look = "None"
    scene.view_settings.exposure = float(os.environ.get("RENDER_EXPOSURE", -0.35))

    out.parent.mkdir(parents=True, exist_ok=True)
    scene.render.filepath = str(out)
    log(f"rendering {RES}x{RES} @ {SAMPLES} samples -> {out}")
    bpy.ops.render.render(write_still=True)
    log("done")

    if os.environ.get("SAVE_BLEND"):
        bpy.ops.wm.save_as_mainfile(filepath=str(blend))
        log(f"saved {blend}")
