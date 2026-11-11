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
RES = int(os.environ.get("RENDER_RES", 1800))
ASPECT = 0.8  # width / height; the exploded stack is tall
SAMPLES = int(os.environ.get("RENDER_SAMPLES", 256))


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


def mat(name, rgb, rough=0.55, metallic=0.0, bevel=0.0, grain=0.0, layers=0.0):
    """bevel: shader-rounded edge radius in mm, so sharp STL edges catch light.
    grain: strength of a fine bump, the matte tooth of printed plastic.
    layers: layer height in mm; walls get the ribbing of an FDM print."""
    m = bpy.data.materials.new(name)
    m.use_nodes = True
    nt = m.node_tree
    b = nt.nodes["Principled BSDF"]
    b.inputs["Base Color"].default_value = (*rgb, 1.0)
    b.inputs["Roughness"].default_value = rough
    b.inputs["Metallic"].default_value = metallic
    normal = None
    if bevel:
        bv = nt.nodes.new("ShaderNodeBevel")
        bv.samples = 8
        bv.inputs["Radius"].default_value = bevel
        normal = bv.outputs["Normal"]
    if grain:
        noise = nt.nodes.new("ShaderNodeTexNoise")
        noise.inputs["Scale"].default_value = 6.0
        noise.inputs["Detail"].default_value = 4.0
        bump = nt.nodes.new("ShaderNodeBump")
        bump.inputs["Strength"].default_value = grain
        bump.inputs["Distance"].default_value = 0.05
        nt.links.new(noise.outputs["Fac"], bump.inputs["Height"])
        if normal:
            nt.links.new(normal, bump.inputs["Normal"])
        normal = bump.outputs["Normal"]
    if layers:
        geo = nt.nodes.new("ShaderNodeNewGeometry")
        sep = nt.nodes.new("ShaderNodeSeparateXYZ")
        freq = nt.nodes.new("ShaderNodeMath")
        freq.operation = "MULTIPLY"
        freq.inputs[1].default_value = 2.0 * math.pi / layers
        sine = nt.nodes.new("ShaderNodeMath")
        sine.operation = "SINE"
        ribs = nt.nodes.new("ShaderNodeBump")
        ribs.inputs["Strength"].default_value = 0.6
        ribs.inputs["Distance"].default_value = 0.03
        nt.links.new(geo.outputs["Position"], sep.inputs[0])
        nt.links.new(sep.outputs["Z"], freq.inputs[0])
        nt.links.new(freq.outputs[0], sine.inputs[0])
        nt.links.new(sine.outputs[0], ribs.inputs["Height"])
        if normal:
            nt.links.new(normal, ribs.inputs["Normal"])
        normal = ribs.outputs["Normal"]
    if normal:
        nt.links.new(normal, b.inputs["Normal"])
    return m


def fix_step_materials(objs):
    """KiCad's STEP parts all arrive as rough metal. Sort them by colour into
    shiny metal and plastic, and give the soldermask its gloss."""
    seen = set()
    for o in objs:
        if o.type != "MESH":
            continue
        for m in o.data.materials:
            if m is None or m.name in seen or not m.use_nodes:
                continue
            seen.add(m.name)
            b = m.node_tree.nodes.get("Principled BSDF")
            if b is None:
                continue
            r, g, bl, _ = b.inputs["Base Color"].default_value
            rough, metal = b.inputs["Roughness"], b.inputs["Metallic"]
            if metal.default_value < 0.5:
                if g > r * 1.5 and g > bl:  # soldermask
                    # Thin lacquer: copper shows through, surface is not quite flat.
                    rough.default_value = 0.25
                    b.inputs["Alpha"].default_value = 0.55
                    nt = m.node_tree
                    noise = nt.nodes.new("ShaderNodeTexNoise")
                    noise.inputs["Scale"].default_value = 150.0
                    bump = nt.nodes.new("ShaderNodeBump")
                    bump.inputs["Strength"].default_value = 0.15
                    bump.inputs["Distance"].default_value = 0.02
                    nt.links.new(noise.outputs["Fac"], bump.inputs["Height"])
                    nt.links.new(bump.outputs["Normal"], b.inputs["Normal"])
                continue
            hi, lo = max(r, g, bl), min(r, g, bl)
            neutral = hi - lo < 0.08
            gold = r > 0.5 and bl < 0.1
            if rough.default_value < 0.9 or gold or (neutral and hi > 0.45):
                rough.default_value = 0.28
            else:
                metal.default_value = 0.0
                rough.default_value = 0.4 if hi < 0.2 else 0.5


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
    hdri = Path(bpy.utils.resource_path("LOCAL")) / "datafiles/studiolights/world/studio.exr"
    # Dark below, bright above: metal and glossy parts get something to reflect.
    wn = world.node_tree
    coord = wn.nodes.new("ShaderNodeTexCoord")
    sep = wn.nodes.new("ShaderNodeSeparateXYZ")
    ramp = wn.nodes.new("ShaderNodeValToRGB")
    ramp.color_ramp.elements[0].position = 0.35
    ramp.color_ramp.elements[0].color = (0.04, 0.04, 0.045, 1.0)
    ramp.color_ramp.elements[1].position = 0.9
    ramp.color_ramp.elements[1].color = (1.0, 0.98, 0.95, 1.0)
    remap = wn.nodes.new("ShaderNodeMapRange")
    remap.inputs["From Min"].default_value = -1.0
    wn.links.new(coord.outputs["Generated"], sep.inputs[0])
    wn.links.new(sep.outputs["Z"], remap.inputs["Value"])
    wn.links.new(remap.outputs["Result"], ramp.inputs["Fac"])
    wn.links.new(ramp.outputs["Color"], wn.nodes["Background"].inputs[0])
    wn.nodes["Background"].inputs[1].default_value = 0.5
    if hdri.exists():  # a real room in the reflections beats a gradient
        env = wn.nodes.new("ShaderNodeTexEnvironment")
        env.image = bpy.data.images.load(str(hdri))
        wn.links.new(env.outputs["Color"], wn.nodes["Background"].inputs[0])
        wn.nodes["Background"].inputs[1].default_value = 0.35

    # --- camera ------------------------------------------------------------
    # Three-quarter view from the front-left and above, matching the 2026-09-15
    # framing: USB end to the back-left, camera connector to the front-right.
    az = math.radians(float(os.environ.get("CAM_AZ", -55.0)))
    el = math.radians(float(os.environ.get("CAM_EL", 24.0)))
    lens = float(os.environ.get("CAM_LENS", 85.0))
    # Fit the bounding sphere with a margin, rather than a magic distance.
    fit = radius / math.tan(0.5 * 2.0 * math.atan(18.0 / lens))
    dist = float(os.environ.get("CAM_DIST", fit * float(os.environ.get("CAM_MARGIN", 1.5))))
    target = Vector((centre.x, centre.y, centre.z))
    eye = target + Vector((
        dist * math.cos(el) * math.cos(az),
        dist * math.cos(el) * math.sin(az),
        dist * math.sin(el),
    ))
    cam_data = bpy.data.cameras.new("cam")
    cam_data.lens = lens
    # Scene units are mm; say so, or depth of field is computed for metres.
    scene.unit_settings.scale_length = 0.001
    cam_data.dof.use_dof = True
    cam_data.dof.focus_distance = dist
    cam_data.dof.aperture_fstop = float(os.environ.get("CAM_FSTOP", 11.0))
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

    area("key", (-170, -150, 240), 380000, 220)
    area("fill", (230, -130, 60), 30000, 360)
    area("rim", (90, 240, 150), 200000, 160)
    # Sits where the camera's mirror image off a flat top lands: lights up the
    # shield can and the cell instead of leaving them to reflect a dark room.
    area("card", (-157, 224, 122), 90000, 300)

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
    scene.render.resolution_x = int(RES * ASPECT)
    scene.render.resolution_y = RES
    scene.render.resolution_percentage = 100
    scene.render.image_settings.file_format = "PNG"
    scene.render.image_settings.color_mode = "RGBA"
    scene.render.image_settings.compression = 90
    scene.view_settings.view_transform = "AgX"
    scene.view_settings.look = "AgX - Medium High Contrast"
    scene.view_settings.exposure = float(os.environ.get("RENDER_EXPOSURE", -0.15))

    out.parent.mkdir(parents=True, exist_ok=True)
    scene.render.filepath = str(out)
    log(f"rendering {int(RES * ASPECT)}x{RES} @ {SAMPLES} samples -> {out}")
    bpy.ops.render.render(write_still=True)
    log("done")

    if os.environ.get("SAVE_BLEND"):
        bpy.ops.wm.save_as_mainfile(filepath=str(blend))
        log(f"saved {blend}")
