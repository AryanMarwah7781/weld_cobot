"""Camera brackets for the UFACTORY Lite 6 tool flange: Arducam B0495 + Intel RealSense D405.

Run with FreeCAD's command-line Python (FreeCAD 1.x):

    "C:\\Program Files\\FreeCAD 1.1\\bin\\freecadcmd.exe" -c "__file__=r'cad/bracket/bracket.py';exec(open(__file__,encoding='utf-8').read())"

Writes cad/bracket/out/:
    print/*.stl          parts in print orientation (flat on the bed, no supports)
    step/*.step          parts and full assemblies (with camera stand-ins), for CAD
    fcstd/*.FCStd        one FreeCAD document per design, every part a separate object
    render/*.stl         parts in the flange frame, for rendering
    specs.json           masses, centres of mass, TCPs and point clouds for the analysis and the page

Every dimension comes from docs/hardware/mounting-reference.md. Frame: the flange frame, origin at the
centre of the flange face, +Z out of the flange (tool z), +X toward the flange's Ø6 locating hole.

The system is one shared FLANGE ADAPTER (4 x M6 on PCD 50, Ø6 dowel, centring ring in the Ø31.5 recess)
and three interchangeable CAMERA HEADS that bolt to it with 4 x M3 into heat-set inserts:
    twin     both cameras side by side, looking straight out along the flange axis
    inline   the Arducam on the J6 axis on a short tower, the D405 beside it
    angled   both cameras on a 45° deck: the camera axis is pitched 45° off the flange axis
"""
import json
import math
import os

import FreeCAD as App
import MeshPart
import Part
from FreeCAD import Vector as V

HERE = os.path.dirname(os.path.abspath(__file__)) if "__file__" in globals() else os.path.join(os.getcwd(), "cad", "bracket")
OUT = os.path.join(HERE, "out")
for sub in ("print", "step", "fcstd", "render"):
    os.makedirs(os.path.join(OUT, sub), exist_ok=True)

PETG = 1.27e-3          # g/mm³
STANDOFF = 120.0         # working distance assumed in the project figure (open question in claude.md)

# ---- flange (docs/hardware/mounting-reference.md) ---------------------------------------------------
FLANGE_R = 37.5          # Ø75
M6_PCD_R = 25.0          # 4 x M6 on PCD 50 at 45°, 135°, 225°, 315° from the locating hole
M6_ANGLES = (45, 135, 225, 315)
DOWEL_AT = (25.0, 0.0)   # Ø6 locating hole on PCD 50 at 0°
RECESS_R = 15.75         # Ø31.5 centre recess, 6 deep

# ---- adapter and head interface -----------------------------------------------------------------
T_AD = 10.0              # adapter thickness
M3_R = 30.0              # head screws: 4 x M3 into heat-set inserts, clear of the M6 counterbores and the dowel
M3_ANGLES = (22.5, 112.5, 202.5, 292.5)
INSERT_D, INSERT_DEPTH = 4.0, 7.0       # M3 x 5.7 heat-set insert
M3_CLEAR, M3_CB_D, M3_CB_DEPTH = 3.4, 6.2, 3.2

# ---- cameras -------------------------------------------------------------------------------------
D405 = dict(w=42.0, h=42.0, d=23.0, baseline=18.0)            # back face on the head, looks +z
D405_TRIPOD = 8.35                                              # tripod hole centre from the back face
ARD = dict(board=34.0, holes=28.0, stack=20.2, lens=20.0)     # B0495: 34 x 34 boards, 4 holes on 28 x 28
ARD_FRONT = ARD["stack"] + ARD["lens"]                          # 40.2 from the back of the rear board
# The B0495's USB-C receptacle sits on the rear board toward its bottom edge, pointing backward. Its exact
# distance from the centre isn't known, so the head has a slot from near the board's centre out past that
# edge (USB_SLOT), and the plug goes straight through wherever it lands. The stand-in puts it USB_OFF out.
USB_SLOT = dict(back=4.0, out=22.0, width=15.0)
USB_OFF = 11.0


def cyl(r, h, x=0.0, y=0.0, z=0.0, d=(0, 0, 1)):
    return Part.makeCylinder(r, h, V(x, y, z), V(*d))


def box(x0, y0, z0, w, d, h):
    return Part.makeBox(w, d, h, V(x0, y0, z0))


def rbox(cx, cy, z0, w, d, h, r):
    """Box centred on (cx, cy) with its four vertical edges rounded."""
    b = box(cx - w / 2, cy - d / 2, z0, w, d, h)
    if r > 0:
        edges = [e for e in b.Edges if abs(e.Vertexes[0].Point.z - e.Vertexes[1].Point.z) > 1e-6]
        b = b.makeFillet(r, edges)
    return b


def slot(x0, y0, x1, y1, z0, h, r):
    """Stadium from (x0, y0) to (x1, y1), r wide each side, from z0 up h."""
    length = math.hypot(x1 - x0, y1 - y0)
    s = cyl(r, h, x0, y0, z0).fuse(cyl(r, h, x1, y1, z0))
    if length > 1e-6:
        b = box(0, -r, z0, length, 2 * r, h)
        b.rotate(V(0, 0, 0), V(0, 0, 1), math.degrees(math.atan2(y1 - y0, x1 - x0)))
        b.translate(V(x0, y0, 0))
        s = s.fuse(b)
    return s


def polar(r, deg):
    a = math.radians(deg)
    return r * math.cos(a), r * math.sin(a)


def prism_yz(points, x0, x1):
    """A prism of a polygon given in the y-z plane, extruded along x from x0 to x1."""
    wire = Part.makePolygon([V(x0, y, z) for y, z in points] + [V(x0, points[0][0], points[0][1])])
    return Part.Face(wire).extrude(V(x1 - x0, 0, 0))


def prism_xz(points, y0, y1):
    wire = Part.makePolygon([V(x, y0, z) for x, z in points] + [V(points[0][0], y0, points[0][1])])
    return Part.Face(wire).extrude(V(0, y1 - y0, 0))


def frame_matrix(o, ex, ey, ez):
    """Matrix taking local (x, y, z) to o + x·ex + y·ey + z·ez."""
    return App.Matrix(ex[0], ey[0], ez[0], o[0], ex[1], ey[1], ez[1], o[1], ex[2], ey[2], ez[2], o[2], 0, 0, 0, 1)


def placed(shape, m):
    s = shape.copy()
    s.transformShape(m)
    return s


def clean(s):
    try:
        return s.removeSplitter()
    except Exception:
        return s


def chamfer_outer(s, d, zs, radius=FLANGE_R):
    """Chamfer the outer circular edge(s) at heights zs (the edges a hand touches)."""
    edges = [e for e in s.Edges if isinstance(e.Curve, Part.Circle) and abs(e.Curve.Radius - radius) < 1e-3 and any(abs(e.Curve.Center.z - z) < 1e-4 for z in zs)]
    if not edges:
        return s
    try:
        return s.makeChamfer(d, edges)
    except Exception:
        return s


# =================================================================================================
# Shared parts
# =================================================================================================
def adapter():
    """Flange adapter: printed back face down (on the flange side), counterbores open upward."""
    a = cyl(FLANGE_R, T_AD)
    a = a.cut(cyl(11.0, T_AD + 2, z=-1))                                   # lightening bore Ø22
    a = a.cut(cyl(RECESS_R, 3.0, z=-0.01))                                  # pocket for the centring ring
    for ang in M6_ANGLES:                                                   # M6 x 12 SHCS, heads 3.8 below the face
        x, y = polar(M6_PCD_R, ang)
        a = a.cut(cyl(3.3, T_AD + 2, x, y, -1)).cut(cyl(5.6, 7.0, x, y, T_AD - 6.2))
    a = a.cut(cyl(3.0, T_AD + 2, DOWEL_AT[0], DOWEL_AT[1], -1))           # Ø6 x 16 steel dowel, pressed in
    for ang in M3_ANGLES:                                                   # heat-set inserts for the head
        x, y = polar(M3_R, ang)
        a = a.cut(cyl(INSERT_D / 2, INSERT_DEPTH + 0.01, x, y, T_AD - INSERT_DEPTH))
    return clean(chamfer_outer(a, 0.6, [T_AD]))


def ring():
    """Centring ring: sits 3 mm in the adapter, 4 mm in the flange's Ø31.5 recess (6 deep)."""
    r = cyl(15.65, 7.0).cut(cyl(11.6, 9.0, z=-1))
    try:
        r = r.makeChamfer(0.5, [e for e in r.Edges if abs(e.Vertexes[0].Point.z) < 1e-6 or abs(e.Vertexes[0].Point.z - 7) < 1e-6])
    except Exception:
        pass
    return r


def head_screws(part, z_foot, t_foot):
    """4 x M3 clearance holes with counterbores; a counterbore-wide shaft straight up gives the hex key a path."""
    for ang in M3_ANGLES:
        x, y = polar(M3_R, ang)
        part = part.cut(cyl(M3_CLEAR / 2, t_foot + 2, x, y, z_foot - 1))
        part = part.cut(cyl(M3_CB_D / 2, 400, x, y, z_foot + t_foot - M3_CB_DEPTH))
    return part


def ard_mount(top, cx, cy, standoff):
    """Arducam B0495 on four standoffs (M2 thread-forming screws), with a window for its USB-C plug."""
    posts = None
    h = ARD["holes"] / 2
    for dx in (-h, h):
        for dy in (-h, h):
            p = cyl(3.0, standoff, cx + dx, cy + dy, top)
            posts = p if posts is None else posts.fuse(p)
    return posts


def usb_slot(cx, cy, toward, z0, h, out=USB_SLOT["out"]):
    """The slot under the Arducam's USB-C edge: from USB_SLOT back of the centre to `out` past it, along `toward`."""
    length = USB_SLOT["back"] + out
    mid = (out - USB_SLOT["back"]) / 2
    tx, ty = toward
    w, d = (length, USB_SLOT["width"]) if abs(tx) > abs(ty) else (USB_SLOT["width"], length)
    return rbox(cx + tx * mid, cy + ty * mid, z0, w, d, h, 3.0)


def ard_holes(part, top, cx, cy, standoff, toward=(1, 0), z_from=None, out=USB_SLOT["out"]):
    h = ARD["holes"] / 2
    for dx in (-h, h):
        for dy in (-h, h):
            part = part.cut(cyl(0.875, 9.0, cx + dx, cy + dy, top + standoff - 9.0))   # Ø1.75 pilot for M2
    return part.cut(usb_slot(cx, cy, toward, (z_from if z_from is not None else top - 30), 60, out))


def d405_ledge(x_c, y_bottom, top, length=24.0, t=5.0):
    """Shelf under the D405's tripod face. Its 1/4-20 hole is 8.35 mm from the back face (D400 Series Datasheet,
    Figure 10-13); the back face sits on the head, so the clearance slot is centred 8.35 out, ±0.75 for print tolerance."""
    w = D405["w"] + 4
    ledge = box(x_c - w / 2, y_bottom - t, top, w, t, length)
    lo, hi = D405_TRIPOD - 0.75, D405_TRIPOD + 0.75
    s = box(-3.4, -1, lo, 6.8, t + 2, hi - lo).fuse(cyl(3.4, t + 2, 0, -1, lo, d=(0, 1, 0))).fuse(cyl(3.4, t + 2, 0, -1, hi, d=(0, 1, 0)))
    s.translate(V(x_c, y_bottom - t, top))
    return ledge, s


# =================================================================================================
# Heads (modelled in the flange frame, their back face on the adapter at z = T_AD)
# =================================================================================================
def head_twin():
    z0, t = T_AD, 6.0
    top = z0 + t
    ard = (52.0, 0.0)                    # outboard, so the USB-C cable runs straight back past the flange
    d405 = (-30.0, 0.0)
    plate = rbox(8.0, 0.0, z0, 140.0, 52.0, t, 8.0).fuse(cyl(FLANGE_R, t, z=z0))
    ribs = rbox(57.0, 24.5, top, 42.0, 3.0, 8.0, 1.2).fuse(rbox(57.0, -24.5, top, 42.0, 3.0, 8.0, 1.2))
    ledge, ledge_slot = d405_ledge(d405[0], d405[1] - D405["h"] / 2, top)
    head = plate.fuse(ribs).fuse(ledge).fuse(ard_mount(top, *ard, 4.0))
    head = head.cut(ledge_slot)
    head = ard_holes(head, top, *ard, 4.0, toward=(1, 0))         # USB-C edge outboard, away from the robot
    head = head_screws(head, z0, t)
    cams = {
        "ard": frame_matrix((ard[0], ard[1], top + 4.0), (1, 0, 0), (0, 1, 0), (0, 0, 1)),
        "d405": frame_matrix((d405[0], d405[1], top), (1, 0, 0), (0, 1, 0), (0, 0, 1)),
        "usb": (1, 0),
    }
    return clean(head), cams


def head_inline():
    z0, t = T_AD, 6.0
    z1 = z0 + t                            # walls start
    deck0, deck1 = z1 + 40.0, z1 + 45.0    # deck 5 thick, 40 mm above the foot: room for a straight USB-C plug
    foot = cyl(FLANGE_R, t, z=z0)
    walls = box(-22, 19.5, z1, 44, 3.5, 40).fuse(box(-22, -23, z1, 44, 3.5, 40))
    gusset = prism_xz([(-22, z1), (-22, deck0), (-62, deck0)], 19.5, 23).fuse(prism_xz([(-22, z1), (-22, deck0), (-62, deck0)], -23, -19.5))
    deck = rbox(-22.0, -1.5, deck0, 88.0, 49.0, 5.0, 4.0)        # x -66..22, y -26..23
    d405 = (-42.0, 0.0)
    ledge, ledge_slot = d405_ledge(d405[0], d405[1] - D405["h"] / 2, deck1)
    head = foot.fuse(walls).fuse(gusset).fuse(deck).fuse(ledge).fuse(ard_mount(deck1, 0.0, 0.0, 3.0))
    head = head.cut(ledge_slot)
    head = ard_holes(head, deck1, 0.0, 0.0, 3.0, toward=(1, 0), z_from=deck0 - 1, out=28.0)   # toward the open side, through the deck edge
    head = head_screws(head, z0, t)
    cams = {
        "ard": frame_matrix((0.0, 0.0, deck1 + 3.0), (1, 0, 0), (0, 1, 0), (0, 0, 1)),
        "d405": frame_matrix((d405[0], d405[1], deck1), (1, 0, 0), (0, 1, 0), (0, 0, 1)),
        "usb": (1, 0),
    }
    return clean(head), cams


PITCH = 45.0
def head_angled():
    """Hollow 45° wedge: side walls, a 5 mm deck and an open back for the cables."""
    z0, t = T_AD, 6.0
    c = math.cos(math.radians(PITCH))
    s = math.sin(math.radians(PITCH))
    y_lo, y_hi = 34.0, -20.0                      # deck runs from (y 34, z0) up to (y -20, z0 + 54)
    rise = y_lo - y_hi
    outer = prism_yz([(y_lo, z0), (y_hi, z0), (y_hi, z0 + rise)], -52.0, 52.0)
    # inside: below the deck (offset 5 along its normal), above the foot, between the walls, open at the back
    n = (0.0, s, c)                               # deck normal (the camera direction)
    off = 5.0 / c                                 # vertical offset of a 5 mm (normal) deck at 45°
    inner = prism_yz([(y_lo - off - t, z0 + t), (y_hi - 30, z0 + t), (y_hi - 30, z0 + rise + 30 - off)], -48.0, 48.0)
    inner = inner.common(prism_yz([(y_lo - off, z0), (y_hi - 60, z0), (y_hi - 60, z0 + rise + 60 - off)], -48.0, 48.0))
    wedge = outer.cut(inner)
    foot = cyl(FLANGE_R, t, z=z0).fuse(rbox(0.0, 7.0, z0, 104.0, 54.0, t, 6.0))
    head = foot.fuse(wedge)
    # deck frame: origin on the deck surface at the bottom edge, ex along x, es up the slope, n out of the deck
    es = (0.0, -c, s)
    P0 = (0.0, y_lo, z0)
    M = frame_matrix(P0, (1, 0, 0), es, n)
    w_cam = 44.0
    # D405 on the +x side: on the -x side the M3 screw at 112.5° sits under its ledge, and that screw's
    # access hole would cut straight through the ledge
    ard = (-25.0, w_cam)
    d405 = (27.0, w_cam)
    feats = ard_mount(0.0, *ard, 4.0)
    ledge, ledge_slot = d405_ledge(d405[0], d405[1] - D405["h"] / 2, 0.0)
    feats = feats.fuse(ledge)
    head = head.fuse(placed(feats, M))
    cutters = None
    h = ARD["holes"] / 2
    for dx in (-h, h):
        for dy in (-h, h):
            p = cyl(0.875, 9.0, ard[0] + dx, ard[1] + dy, 4.0 - 9.0)
            cutters = p if cutters is None else cutters.fuse(p)
    cutters = cutters.fuse(usb_slot(ard[0], ard[1], (0, 1), -20.0, 30.0)).fuse(ledge_slot)   # up the slope: the plug leaves by the open back
    head = head.cut(placed(cutters, M))
    head = head_screws(head, z0, t)
    cams = {
        "ard": M.multiply(frame_matrix((ard[0], ard[1], 4.0), (1, 0, 0), (0, 1, 0), (0, 0, 1))),
        "d405": M.multiply(frame_matrix((d405[0], d405[1], 0.0), (1, 0, 0), (0, 1, 0), (0, 0, 1))),
        "usb": (0, 1),
    }
    return clean(head), cams


# =================================================================================================
# Camera stand-ins (for the assemblies, renders and clearance checks; not printed)
# =================================================================================================
def d405_model():
    b = rbox(0, 0, 0, D405["w"], D405["h"], D405["d"], 4.0)
    for x in (-D405["baseline"] / 2, D405["baseline"] / 2):
        b = b.fuse(cyl(3.2, 0.4, x, 4.0, D405["d"]))
    return b


def ard_model(toward=(1, 0)):
    h = ARD["holes"] / 2
    m = rbox(0, 0, 0, 34, 34, 1.6, 2.5)
    for dx in (-h, h):
        for dy in (-h, h):
            m = m.fuse(cyl(1.6, ARD["stack"] - 3.2, dx, dy, 1.6))
    m = m.fuse(rbox(0, 0, ARD["stack"] - 1.6, 34, 34, 1.6, 2.5))
    m = m.fuse(rbox(0, 0, ARD["stack"], 20, 20, 8, 1.0))
    m = m.fuse(cyl(7.0, ARD["lens"] - 8 - 2.5, 0, 0, ARD["stack"] + 8))
    m = m.fuse(cyl(7.8, 2.5, 0, 0, ARD_FRONT - 2.5))
    tx, ty = toward                                                # USB-C receptacle toward the bottom edge
    w, d = (3.4, 9.0) if abs(tx) > abs(ty) else (9.0, 3.4)
    m = m.fuse(box(tx * USB_OFF - w / 2, ty * USB_OFF - d / 2, -3.2, w, d, 3.2))
    return m


def usb_plug(toward=(1, 0)):
    """A straight USB-C plug and its strain relief, entering the receptacle from behind."""
    tx, ty = toward
    w, d = (6.6, 12.4) if abs(tx) > abs(ty) else (12.4, 6.6)
    x, y = tx * USB_OFF, ty * USB_OFF
    p = rbox(x, y, -3.2 - 22, w, d, 22, 2.0)
    return p.fuse(cyl(2.2, 8, x, y, -3.2 - 22 - 8))


# =================================================================================================
# Build, export
# =================================================================================================
def mesh_of(shape, lin=0.02, ang=0.1):
    return MeshPart.meshFromShape(Shape=shape, LinearDeflection=lin, AngularDeflection=ang, Relative=False)


def points_of(shape, step):
    m = mesh_of(shape, 0.3, 0.5)
    pts = [(round(p.x, 1), round(p.y, 1), round(p.z, 1)) for p in m.Points]
    keep, seen = [], set()
    for p in pts:
        key = (round(p[0] / step), round(p[1] / step), round(p[2] / step))
        if key not in seen:
            seen.add(key)
            keep.append(p)
    return keep


def viewer_mesh(shape):
    m = mesh_of(shape, 0.06, 0.25)
    pts = [[round(p.x, 2), round(p.y, 2), round(p.z, 2)] for p in m.Points]
    faces = [list(f) for f in m.Topology[1]]
    return {"v": [c for p in pts for c in p], "f": [i for f in faces for i in f]}


def mat_list(m):
    return [[m.A11, m.A12, m.A13, m.A14], [m.A21, m.A22, m.A23, m.A24], [m.A31, m.A32, m.A33, m.A34]]


def write_stl(shape, path, lin=0.01, ang=0.08):
    mesh_of(shape, lin, ang).write(path)


ADAPTER = adapter()
RING = ring()
RING_PLACED = RING.copy()
RING_PLACED.translate(V(0, 0, -4.0))                     # 3 mm in the adapter, 4 mm proud of its back face
D405_SHAPE = d405_model()
ARD_SHAPE = ard_model()
PLUG = usb_plug()

DESIGNS = [
    ("twin", "Twin", head_twin),
    ("inline", "Inline", head_inline),
    ("angled", "Angled 45°", head_angled),
]

write_stl(ADAPTER, os.path.join(OUT, "print", "adapter.stl"))
write_stl(RING, os.path.join(OUT, "print", "centring-ring.stl"))
ADAPTER.exportStep(os.path.join(OUT, "step", "adapter.step"))
RING.exportStep(os.path.join(OUT, "step", "centring-ring.step"))
write_stl(ADAPTER, os.path.join(OUT, "render", "adapter.stl"), 0.05, 0.2)
write_stl(RING_PLACED, os.path.join(OUT, "render", "centring-ring.stl"), 0.05, 0.2)

specs = {"standoff": STANDOFF, "petg_g_mm3": PETG, "parts": {}, "designs": {}}
for name, shape in (("adapter", ADAPTER), ("centring-ring", RING)):
    specs["parts"][name] = {"volume": shape.Volume, "mass_solid": shape.Volume * PETG, "bbox": [shape.BoundBox.XLength, shape.BoundBox.YLength, shape.BoundBox.ZLength]}

viewer = {"adapter": viewer_mesh(ADAPTER), "ring": viewer_mesh(RING_PLACED), "d405": viewer_mesh(D405_SHAPE), "ard": viewer_mesh(ARD_SHAPE), "plug": viewer_mesh(PLUG), "heads": {}}

for key, title, build in DESIGNS:
    head, cams = build()
    if not head.isValid():
        head.fix(1e-4, 1e-4, 1e-4)
    solids = head.Solids
    print(f"{key}: valid={head.isValid()} solids={len(solids)} volume={head.Volume:.0f} mm3")
    # print orientation: back face (z = T_AD) on the bed
    printed = head.copy()
    printed.translate(V(0, 0, -T_AD))
    write_stl(printed, os.path.join(OUT, "print", f"head-{key}.stl"))
    head.exportStep(os.path.join(OUT, "step", f"head-{key}.step"))
    write_stl(head, os.path.join(OUT, "render", f"head-{key}.stl"), 0.05, 0.2)
    d405 = placed(D405_SHAPE, cams["d405"])
    ard_shape, plug_shape = ard_model(cams["usb"]), usb_plug(cams["usb"])
    ard = placed(ard_shape, cams["ard"])
    plug = placed(plug_shape, cams["ard"])
    write_stl(d405, os.path.join(OUT, "render", f"d405-{key}.stl"), 0.05, 0.2)
    write_stl(ard, os.path.join(OUT, "render", f"ard-{key}.stl"), 0.05, 0.2)
    write_stl(plug, os.path.join(OUT, "render", f"plug-{key}.stl"), 0.05, 0.2)
    Part.makeCompound([ADAPTER, RING_PLACED, head, d405, ard]).exportStep(os.path.join(OUT, "step", f"assembly-{key}.step"))
    doc = App.newDocument(f"bracket_{key}")
    for label, shape in (("FlangeAdapter", ADAPTER), ("CentringRing", RING_PLACED), (f"Head_{key}", head), ("D405_standin", d405), ("Arducam_standin", ard)):
        obj = doc.addObject("Part::Feature", label)
        obj.Shape = shape
    doc.recompute()
    doc.saveAs(os.path.join(OUT, "fcstd", f"bracket-{key}.FCStd"))
    App.closeDocument(doc.Name)
    # camera poses: the Arducam's optical axis and lens front, the TCP 120 mm out
    A = cams["ard"]
    lens = A.multiply(V(0, 0, ARD_FRONT))
    axis = V(A.A13, A.A23, A.A33)
    tcp = lens + axis * STANDOFF
    Dm = cams["d405"]
    d405_front = Dm.multiply(V(0, 0, D405["d"]))
    whole = Part.makeCompound([ADAPTER, RING_PLACED, head])
    with_cams = Part.makeCompound([ADAPTER, RING_PLACED, head, d405, ard])
    com_printed = sum((s.CenterOfMass * s.Volume for s in whole.Solids), V()) * (1.0 / whole.Volume)
    specs["designs"][key] = {
        "title": title,
        "head_volume": head.Volume,
        "head_mass_solid": head.Volume * PETG,
        "printed_volume": whole.Volume,
        "printed_com": [com_printed.x, com_printed.y, com_printed.z],
        "head_bbox": [head.BoundBox.XMin, head.BoundBox.YMin, head.BoundBox.ZMin, head.BoundBox.XMax, head.BoundBox.YMax, head.BoundBox.ZMax],
        "print_height": head.BoundBox.ZLength,
        "ard": mat_list(A),
        "d405": mat_list(Dm),
        "lens_front": [lens.x, lens.y, lens.z],
        "axis": [axis.x, axis.y, axis.z],
        "tcp": [tcp.x, tcp.y, tcp.z],
        "d405_front": [d405_front.x, d405_front.y, d405_front.z],
        "cloud": points_of(with_cams, 4.0),
    }
    viewer["heads"][key] = {"head": viewer_mesh(head), "d405": mat_list(cams["d405"]), "ard": mat_list(cams["ard"]), "ardMesh": viewer_mesh(ard_shape), "plugMesh": viewer_mesh(plug_shape)}

with open(os.path.join(OUT, "specs.json"), "w") as f:
    json.dump(specs, f)
with open(os.path.join(OUT, "viewer.json"), "w") as f:
    json.dump(viewer, f)
print("done")
