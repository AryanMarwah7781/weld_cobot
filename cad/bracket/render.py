"""Render the bracket designs with Blender (4.x).

    "C:\\Program Files\\Blender Foundation\\Blender 4.4\\blender.exe" -b -P cad/bracket/render.py -- [--quick] [--only twin]

Reads out/render/*.stl (parts in the flange frame, mm) and writes out/img/<design>-<view>.png.
--quick renders with the Workbench engine for geometry checks; the default is Cycles.
"""
import math
import os
import sys

import json

import bpy
from mathutils import Matrix, Vector

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(HERE, "out", "render")
IMG = os.path.join(HERE, "out", "img")
os.makedirs(IMG, exist_ok=True)
args = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
QUICK = "--quick" in args
ONLY = args[args.index("--only") + 1] if "--only" in args else None
DESIGNS = ["twin", "inline", "angled"]


def material(name, color, rough=0.5, metal=0.0, transmission=0.0, emission=None):
    m = bpy.data.materials.get(name) or bpy.data.materials.new(name)
    m.use_nodes = True
    bsdf = m.node_tree.nodes.get("Principled BSDF")
    bsdf.inputs["Base Color"].default_value = (*color, 1)
    bsdf.inputs["Roughness"].default_value = rough
    bsdf.inputs["Metallic"].default_value = metal
    if transmission:
        bsdf.inputs["Transmission Weight"].default_value = transmission
    m.diffuse_color = (*color, 1)
    return m


MATS = {}


def mats():
    MATS["head"] = material("PETG orange", (0.85, 0.30, 0.06), 0.42)
    MATS["adapter"] = material("PETG graphite", (0.08, 0.085, 0.09), 0.5)
    MATS["ring"] = material("PETG graphite", (0.08, 0.085, 0.09), 0.5)
    MATS["d405"] = material("D405 anodised", (0.16, 0.165, 0.17), 0.35, 0.8)
    MATS["ard"] = material("PCB green", (0.04, 0.22, 0.12), 0.45)
    MATS["plug"] = material("Cable black", (0.015, 0.015, 0.015), 0.55)
    MATS["flange"] = material("Robot white", (0.82, 0.82, 0.80), 0.35)
    MATS["floor"] = material("Floor", (0.9, 0.9, 0.88), 0.9)
    MATS["steel"] = material("Mild steel", (0.42, 0.43, 0.45), 0.45, 0.85)
    MATS["bead"] = material("Weld bead", (0.30, 0.27, 0.24), 0.35, 0.9)
    ray = material("Sight line", (1.0, 0.55, 0.1), 0.5)
    ray.node_tree.nodes["Principled BSDF"].inputs["Emission Color"].default_value = (1.0, 0.5, 0.08, 1)
    ray.node_tree.nodes["Principled BSDF"].inputs["Emission Strength"].default_value = 6.0
    MATS["ray"] = ray
    fov = material("Field of view", (1.0, 0.6, 0.2), 0.5)
    fov.node_tree.nodes["Principled BSDF"].inputs["Alpha"].default_value = 0.16
    fov.node_tree.nodes["Principled BSDF"].inputs["Emission Color"].default_value = (1.0, 0.55, 0.15, 1)
    fov.node_tree.nodes["Principled BSDF"].inputs["Emission Strength"].default_value = 0.4
    fov.blend_method = "BLEND" if hasattr(fov, "blend_method") else None
    MATS["fov"] = fov


def clear():
    bpy.ops.wm.read_factory_settings(use_empty=True)


def stl(path, mat):
    bpy.ops.wm.stl_import(filepath=path)
    ob = bpy.context.selected_objects[0]
    ob.data.materials.clear()
    ob.data.materials.append(MATS[mat])
    for poly in ob.data.polygons:
        poly.use_smooth = False
    mod = ob.modifiers.new("autosmooth", "EDGE_SPLIT")
    mod.split_angle = math.radians(30)
    for poly in ob.data.polygons:
        poly.use_smooth = True
    return ob


def flange_stub():
    """The robot's Ø75 tool flange and a little of the J6 housing behind it, for scale."""
    bpy.ops.mesh.primitive_cylinder_add(vertices=96, radius=37.5, depth=10, location=(0, 0, -5))
    a = bpy.context.active_object
    bpy.ops.mesh.primitive_cylinder_add(vertices=96, radius=31, depth=40, location=(0, 0, -30))
    b = bpy.context.active_object
    for ob in (a, b):
        ob.data.materials.append(MATS["flange"])
        bpy.ops.object.select_all(action="DESELECT")
        ob.select_set(True)
        bpy.context.view_layer.objects.active = ob
        bpy.ops.object.shade_smooth()


def light(name, loc, power, size):
    data = bpy.data.lights.new(name, type="AREA")
    data.energy = power
    data.size = size
    ob = bpy.data.objects.new(name, data)
    ob.location = loc
    bpy.context.collection.objects.link(ob)
    direction = Vector((0, 0, 40)) - Vector(loc)
    ob.rotation_euler = direction.to_track_quat("-Z", "Y").to_euler()


def camera(loc, target, lens=55):
    data = bpy.data.cameras.new("cam")
    data.lens = lens
    data.clip_start = 1
    data.clip_end = 5000
    ob = bpy.data.objects.new("cam", data)
    ob.location = loc
    ob.rotation_euler = (Vector(target) - Vector(loc)).to_track_quat("-Z", "Y").to_euler()
    bpy.context.collection.objects.link(ob)
    bpy.context.scene.camera = ob


def scene_setup(width, height):
    sc = bpy.context.scene
    sc.render.resolution_x = width
    sc.render.resolution_y = height
    sc.render.film_transparent = True
    if QUICK:
        sc.render.engine = "BLENDER_WORKBENCH"
        sc.display.shading.light = "STUDIO"
        sc.display.shading.color_type = "MATERIAL"
        sc.display.shading.show_cavity = True
    else:
        sc.render.engine = "CYCLES"
        sc.cycles.samples = 96
        sc.cycles.use_denoising = True
        sc.cycles.device = "CPU"
        sc.view_settings.view_transform = "AgX"
        sc.view_settings.look = "AgX - Medium High Contrast"
    world = bpy.data.worlds.new("w")
    world.use_nodes = True
    world.node_tree.nodes["Background"].inputs[0].default_value = (0.75, 0.76, 0.78, 1)
    world.node_tree.nodes["Background"].inputs[1].default_value = 0.6
    sc.world = world


VIEWS = {
    # product shots per design: (camera location, target, lens); the cameras face +z (angled: +y +z)
    "twin": {"hero": ((300, -300, 420), (8, 0, 45), 62), "side": ((0, -560, 110), (8, 0, 55), 62)},
    "inline": {"hero": ((300, -300, 440), (-15, 0, 70), 62), "side": ((0, -560, 130), (-15, 0, 75), 62)},
    "angled": {"hero": ((280, 400, 360), (0, 25, 50), 62), "side": ((560, 0, 120), (0, 20, 55), 62)},
}
ANALYSIS = json.load(open(os.path.join(HERE, "out", "analysis.json")))
SPECS = json.load(open(os.path.join(HERE, "out", "specs.json")))


def matrix(R, t):
    return Matrix(((R[0][0], R[0][1], R[0][2], t[0]), (R[1][0], R[1][1], R[1][2], t[1]), (R[2][0], R[2][1], R[2][2], t[2]), (0, 0, 0, 1)))


def place(ob, M):
    ob.matrix_world = M @ ob.matrix_world


def cylinder_between(p0, p1, r, mat, verts=48):
    p0, p1 = Vector(p0), Vector(p1)
    d = p1 - p0
    bpy.ops.mesh.primitive_cylinder_add(vertices=verts, radius=r, depth=d.length, location=(p0 + p1) / 2)
    ob = bpy.context.active_object
    ob.rotation_euler = d.to_track_quat("Z", "Y").to_euler()
    ob.data.materials.append(mat)
    bpy.ops.object.shade_smooth()
    return ob


def box_world(mn, mx, mat):
    c = [(a + b) / 2 for a, b in zip(mn, mx)]
    bpy.ops.mesh.primitive_cube_add(size=1, location=c)
    ob = bpy.context.active_object
    ob.scale = [b - a for a, b in zip(mn, mx)]
    ob.data.materials.append(mat)
    return ob


def fov_pyramid(lens, R, dist, h_deg=82, v_deg=66):
    """The Arducam's field of view: apex at the lens, base at the working distance."""
    x = Vector((R[0][0], R[1][0], R[2][0]))
    y = Vector((R[0][1], R[1][1], R[2][1]))
    z = Vector((R[0][2], R[1][2], R[2][2]))
    a = Vector(lens)
    hx = dist * math.tan(math.radians(h_deg / 2))
    hy = dist * math.tan(math.radians(v_deg / 2))
    c = a + z * dist
    verts = [a, c + x * hx + y * hy, c - x * hx + y * hy, c - x * hx - y * hy, c + x * hx - y * hy]
    faces = [(0, 1, 2), (0, 2, 3), (0, 3, 4), (0, 4, 1)]
    me = bpy.data.meshes.new("fov")
    me.from_pydata([tuple(v) for v in verts], [], faces)
    ob = bpy.data.objects.new("fov", me)
    bpy.context.collection.objects.link(ob)
    ob.data.materials.append(MATS["fov"])
    return ob


def bracket_objects(design):
    obs = [stl(os.path.join(SRC, "adapter.stl"), "adapter"), stl(os.path.join(SRC, "centring-ring.stl"), "ring"), stl(os.path.join(SRC, f"head-{design}.stl"), "head"),
           stl(os.path.join(SRC, f"d405-{design}.stl"), "d405"), stl(os.path.join(SRC, f"ard-{design}.stl"), "ard"), stl(os.path.join(SRC, f"plug-{design}.stl"), "plug")]
    return obs


def lights():
    light("key", (300, -250, 450), 2.6e6, 300)
    light("fill", (-400, -100, 200), 0.9e6, 400)
    light("rim", (-100, 350, 300), 1.4e6, 250)


def shoot(name, loc, target, lens):
    for ob in [o for o in bpy.data.objects if o.type == "CAMERA"]:
        bpy.data.objects.remove(ob)
    camera(loc, target, lens)
    bpy.context.scene.render.filepath = os.path.join(IMG, f"{name}{'-quick' if QUICK else ''}.png")
    bpy.ops.render.render(write_still=True)
    print("rendered", name)


def framed(direction, lens, aspect, fill=0.82):
    """Camera location and target that fit every mesh in the frame, looking along -direction."""
    pts = [ob.matrix_world @ Vector(c) for ob in bpy.data.objects if ob.type == "MESH" for c in ob.bound_box]
    lo = Vector((min(p.x for p in pts), min(p.y for p in pts), min(p.z for p in pts)))
    hi = Vector((max(p.x for p in pts), max(p.y for p in pts), max(p.z for p in pts)))
    centre = (lo + hi) / 2
    radius = max((p - centre).length for p in pts)
    half = math.atan(18.0 / aspect / lens)
    dist = radius / math.sin(half) * (1.0 / fill) * 0.9
    d = Vector(direction).normalized()
    return tuple(centre + d * dist), tuple(centre)


def render(design):
    clear()
    mats()
    scene_setup(1400, 1000)
    bracket_objects(design)
    flange_stub()
    lights()
    for view, (loc, target, lens) in VIEWS[design].items():
        cam, aim = framed(Vector(loc) - Vector(target), lens, 1400 / 1000)
        shoot(f"{design}-{view}", cam, aim, lens)


def render_context(design, layout):
    """The bracket on the Lite 6 wrist over the T-joint, in the pose the analysis solved (pass 1, mid-seam)."""
    clear()
    mats()
    scene_setup(1600, 1000)
    r = ANALYSIS["designs"][design][layout]
    F = r["pose"]["frames"]
    Mf = matrix(F[5]["R"], F[5]["t"])
    for ob in bracket_objects(design):
        place(ob, Mf)
    # the robot: tool flange, J6 module, J5 housing and the end of the forearm
    flange_obs_before = set(bpy.data.objects)
    flange_stub()
    for ob in set(bpy.data.objects) - flange_obs_before:
        place(ob, Mf)
    w = Vector(F[3]["t"])
    z4 = Vector((F[3]["R"][0][2], F[3]["R"][1][2], F[3]["R"][2][2]))
    z5 = Vector((F[4]["R"][0][2], F[4]["R"][1][2], F[4]["R"][2][2]))
    cylinder_between(w - z5 * 30, w + z5 * 30, 34, MATS["flange"])
    cylinder_between(w - z4 * 150, w - z4 * 36, 30, MATS["flange"])
    # the work
    for b in ANALYSIS["boxes"][layout]:
        box_world(b["min"], b["max"], MATS["steel"])
    web = ANALYSIS["boxes"][layout][1]
    xw = web["max"][0] if layout == "back" else web["min"][0]
    sgn = 1 if layout == "back" else -1
    cylinder_between((xw + sgn * 2.2, -74, 8.2), (xw + sgn * 2.2, 74, 8.2), 4.6, MATS["bead"], 32)
    bpy.ops.mesh.primitive_plane_add(size=3000, location=(0, 0, 0))
    bpy.context.active_object.data.materials.append(MATS["floor"])
    # the Arducam's line of sight and field of view
    d = SPECS["designs"][design]
    lens = Mf @ Vector(d["lens_front"])
    tcp = Mf @ Vector(d["tcp"])
    cylinder_between(lens, tcp, 0.7, MATS["ray"], 16)
    A = Matrix(d["ard"] + [[0, 0, 0, 1]])
    Rw = (Mf @ A).to_3x3()
    fov_pyramid(lens, [[Rw[i][j] for j in range(3)] for i in range(3)], ANALYSIS["standoff"])
    light("key", (600, -500, 700), 6e6, 500)
    light("fill", (-300, -300, 300), 2e6, 500)
    light("rim", (100, 500, 500), 3e6, 400)
    flange = Vector(F[5]["t"])
    target = tcp * 0.45 + flange * 0.55
    loc = target + (Vector((260, -720, 330)) if layout == "back" else Vector((-260, -720, 330)))
    shoot(f"{design}-context-{layout}", tuple(loc), tuple(target), 50)


for d in DESIGNS:
    if ONLY and d != ONLY:
        continue
    render(d)
    if "--products" in args:
        continue
    render_context(d, "back")
    if d == "angled":
        render_context(d, "out")
