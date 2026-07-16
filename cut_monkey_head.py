"""
cut_monkey_head.py — run inside Blender (Scripting tab > Open > Run Script)

Configured for your file:
  - object name: "monkey_head"
  - scale: 2.5x  (0.137 m -> ~0.34 m wide head, ~34 cm in a cm stage)

What it does:
  1. Cuts the monkey head into two mirror halves (R = +X, L = -X) across
     its symmetry plane X = 0, and fills the cut faces (watertight).
  2. Adds an off-center key: peg on the R half, slightly oversized socket
     on the L half, so a 180-degree-flipped half physically cannot seat.
  3. Sets BOTH halves' origins to the assembled head's center (world 0,0,0)
     so T_mate between the halves is the identity matrix.
  4. Tags each half with a custom property piece_id = "L" / "R".
  5. Exports each half to its own .usd file in EXPORT_DIR.

Before running: Window > Toggle System Console (to see progress/warnings).
"""

import bpy
import bmesh
import os
from mathutils import Vector

# ----------------------------------------------------------------------
# PARAMETERS
# ----------------------------------------------------------------------
EXPORT_DIR   = r"C:\Users\User\Desktop\blender_to_usd"  # output folder
SOURCE_NAME  = "monkey_head"  # object name from your Outliner
HEAD_SCALE   = 2.5            # 0.137 m wide -> ~0.34 m (~34 cm in cm stage)
ADD_KEY      = True           # peg (R half) + socket (L half) on cut face
KEY_RADIUS   = 0.006          # peg radius, Blender units, PRE-scale
KEY_DEPTH    = 0.014          # peg length along X (half embeds each side)
KEY_POS      = Vector((0.0, 0.015, 0.012))  # off-center point ON the cut
                              # plane (keep X = 0). Off-center = flip-proof.
SOCKET_CLEAR = 1.12           # socket = peg scaled by this (clearance fit)

# NOTE: KEY_* values are sized for your 0.137 m head and get multiplied by
# HEAD_SCALE automatically below (final peg radius ~1.5 cm on a 34 cm head).

# ----------------------------------------------------------------------
# helpers
# ----------------------------------------------------------------------
def deselect_all():
    bpy.ops.object.select_all(action='DESELECT')

def activate(obj):
    deselect_all()
    obj.select_set(True)
    bpy.context.view_layer.objects.active = obj

def get_source_object():
    obj = bpy.data.objects.get(SOURCE_NAME)
    if obj is None:
        raise RuntimeError(
            f'No object named "{SOURCE_NAME}" found in this file. '
            f'Check the Outliner and update SOURCE_NAME.'
        )
    return obj

def apply_all_transforms(obj):
    activate(obj)
    bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)

def bisect_half(obj, keep_positive_x):
    """Bisect at X = 0, keep one side, fill the cut boundary with faces."""
    activate(obj)
    bpy.ops.object.mode_set(mode='EDIT')
    bm = bmesh.from_edit_mesh(obj.data)
    geom = bm.verts[:] + bm.edges[:] + bm.faces[:]
    bmesh.ops.bisect_plane(
        bm, geom=geom,
        plane_co=(0.0, 0.0, 0.0), plane_no=(1.0, 0.0, 0.0),
        clear_inner=keep_positive_x,      # remove the -X side
        clear_outer=not keep_positive_x,  # remove the +X side
        use_snap_center=False,
    )
    bmesh.update_edit_mesh(obj.data)
    # fill the open cut boundary so the half is watertight
    bpy.ops.mesh.select_all(action='SELECT')
    try:
        bpy.ops.mesh.edge_face_add()
    except RuntimeError:
        pass  # boundary may already be closed
    bpy.ops.mesh.select_all(action='SELECT')
    bpy.ops.mesh.normals_make_consistent(inside=False)
    bpy.ops.object.mode_set(mode='OBJECT')

def make_key_cylinder(clearance=1.0):
    """Cylinder crossing the cut plane, axis along X, at KEY_POS."""
    bpy.ops.mesh.primitive_cylinder_add(
        radius=KEY_RADIUS * clearance * HEAD_SCALE,
        depth=KEY_DEPTH * HEAD_SCALE,
        location=KEY_POS * HEAD_SCALE,
        rotation=(0.0, 1.5707963, 0.0),  # cylinder axis along X
        vertices=24,
    )
    return bpy.context.active_object

def boolean_op(obj, cutter, op):
    mod = obj.modifiers.new(name="key_bool", type='BOOLEAN')
    mod.object = cutter
    mod.operation = op            # 'UNION' or 'DIFFERENCE'
    mod.solver = 'EXACT'
    bpy.context.view_layer.objects.active = obj
    bpy.ops.object.modifier_apply(modifier=mod.name)

def set_origin_to_world_zero(obj):
    bpy.context.scene.cursor.location = (0.0, 0.0, 0.0)
    activate(obj)
    bpy.ops.object.origin_set(type='ORIGIN_CURSOR')

def export_usd(obj, filepath):
    activate(obj)
    kwargs = dict(
        filepath=filepath,
        selected_objects_only=True,
        export_materials=True,
    )
    # Try Y-up orientation conversion to match your Composer stage;
    # fall back if this Blender build's exporter lacks those options.
    try:
        bpy.ops.wm.usd_export(
            convert_orientation=True,
            export_global_forward_selection='NEGATIVE_Z',
            export_global_up_selection='Y',
            **kwargs,
        )
        print(f"  exported (Y-up converted): {filepath}")
    except TypeError:
        bpy.ops.wm.usd_export(**kwargs)
        print(f"  exported (default axes — fix orientation at the "
              f"reference prim if needed): {filepath}")

# ----------------------------------------------------------------------
# main
# ----------------------------------------------------------------------
print("=" * 60)
print("cut_monkey_head.py starting")

src = get_source_object()
apply_all_transforms(src)

if HEAD_SCALE != 1.0:
    src.scale = (HEAD_SCALE, HEAD_SCALE, HEAD_SCALE)
    apply_all_transforms(src)
    print(f"scaled source by {HEAD_SCALE}x; dimensions now "
          f"{tuple(round(d, 4) for d in src.dimensions)} m")

# make the two halves from duplicates of the source
halves = {}
for pid, keep_pos in (("R", True), ("L", False)):
    activate(src)
    bpy.ops.object.duplicate()
    dup = bpy.context.active_object
    dup.name = f"monkey_half_{pid}"
    bisect_half(dup, keep_positive_x=keep_pos)
    halves[pid] = dup
    print(f"created {dup.name}")

# hide the original, keep it as the master copy
src.hide_set(True)
src.hide_render = True

# optional flip-proof key: peg unioned into R, oversized socket cut from L
if ADD_KEY:
    try:
        peg = make_key_cylinder(1.0)
        boolean_op(halves["R"], peg, 'UNION')
        bpy.data.objects.remove(peg, do_unlink=True)

        socket = make_key_cylinder(SOCKET_CLEAR)
        boolean_op(halves["L"], socket, 'DIFFERENCE')
        bpy.data.objects.remove(socket, do_unlink=True)
        print("key added: peg on R, socket on L")
    except Exception as e:
        print("WARNING: key boolean failed (mesh not perfectly manifold?). "
              "Halves remain valid without the key.")
        print("  detail:", e)

# shared origin + piece tags + export
os.makedirs(EXPORT_DIR, exist_ok=True)
for pid, obj in halves.items():
    set_origin_to_world_zero(obj)
    obj["piece_id"] = pid  # becomes a custom USD attribute on export
    export_usd(obj, os.path.join(EXPORT_DIR, f"monkey_half_{pid}.usd"))

print("Done. Both halves share origin (0,0,0) = assembled head center,")
print("so T_mate between them is the identity matrix.")
print("=" * 60)
