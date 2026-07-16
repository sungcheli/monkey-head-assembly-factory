"""
factory_master.py — ONE-SHOT SETUP — run in Isaac Sim Script Editor.

Combines, in order, everything built so far:

  STEP 1  Factory layout        (build_factory_layout.py)
  STEP 2  Physics pass          (add_factory_physics.py)
  STEP 3  Half production       (half_production_spawner.py, functions only)
  STEP 4  State machine panel   (state_machine_ui.py v2, conveyor REAL)

Because everything now lives in one file, the panel's spawn buttons are
wired to the real spawner:
  Line rows "Spawn one"          -> produce_half("L"/"R")
  Master "Produce Part"          -> produce_pair() (one half on each line)
  Debug  "Spawn pair"            -> produce_pair()
  Line rows "Belt: ON/OFF"       -> real in-feed conveyor (surface velocity)
  Out-feed "Start/Stop conveyor" -> real out-feed conveyor
Everything else remains a stub for later wiring (arms, snap, deletion).

Safe to re-run: layout groups are rebuilt, physics re-application is
harmless, old panel/belts are cleaned up first.

After running: press PLAY, then "Produce Part" — one half spawns on each
belt; with belts ON they are carried into the collection pools.
"""

import time
import math
import re
import asyncio
import omni.usd
import omni.kit.app
import omni.kit.commands
import omni.ui as ui
from pxr import Usd, UsdGeom, UsdPhysics, UsdShade, Gf, Sdf

stage = omni.usd.get_context().get_stage()

# =================================================================
# CONFIG — all paths and layout constants in one place
# =================================================================
KUKA_USD_PATH = "C:/Users/User/Desktop/blender_to_usd/KukaArm1.usd"
ARM_SCALE     = 0.01   # reference-level scale: 0.01 = cm-authored asset -> meters stage.
                       # Tune and re-run: KR 10 should stand ~1.1-1.2 m tall,
                       # waist-high next to the 0.8 m table.
HALF_PATHS = {
    "L": "C:/Users/User/Desktop/blender_to_usd/monkey_half_L.usd",
    "R": "C:/Users/User/Desktop/blender_to_usd/monkey_half_R.usd",
}

TABLE_POS      = (0.0, 0.0)
TABLE_SIZE     = (0.75, 0.75, 0.8)  # enlarged from (0.6, 0.6, 0.8)
TABLE_RAIL_HEIGHT = 0.10  # guardrail height above table top (m) —
                          # low enough for the arm's vertical descent
                          # to clear easily, tall enough to stop a
                          # sliding/knocked half going over the edge
TABLE_RAIL_THICK  = 0.03
LINE_Y         = 1.1     # production lines / pools at y = +/-1.1 (unchanged)

# Arm bases (X, Y) — NOT a mirror pair. Tuned independently per arm
# because the KUKA asset's own origin isn't centered under its physical
# foot, so a single mirrored formula doesn't land both arms the same
# way relative to their pool/table. Edit these two numbers and re-run
# to move an arm — never hand-drag it in the viewport.
ARM_POSITIONS = {
    "L": (1.07, 1.01),
    "R": (1.07, -1.16),
}

# arm motion (STEP 5)
SEQ_FRAMES     = 60     # frames per single-joint move (one joint at a time)
A1_FRAMES      = 100    # base rotations are longer sweeps
HOVER_DZ       = 0.18   # hover height above the grip point (m)
PLACE_DZ       = 0.10   # release height above the place marker (m)
SAFE_TRANSIT_Z = 1.35   # world Z used for the safe-transit pose during
                        # any A1 sweep — clears the table top (0.8 m)
                        # and pool walls (top ~0.85 m) with real margin
GRAB_TOL       = 0.15   # suction attach tolerance: tip-to-part distance (m)
POOL_RADIUS    = 0.45   # search radius around PickZone for parts (m)
LOCATE_TIMEOUT_F = 300  # ~5 s: how long LOCATING waits/retries for a
                        # settled part before giving up (was a single
                        # check — two arms starting together often had
                        # one part settle a beat later than the other)

# settling gate: a part must move less than this between consecutive
# frames, for this many consecutive frames, before ANY arm action may
# target it (locate/approach/grip). Measured as PURE POSITION DRIFT,
# not the RigidBodyAPI velocity attribute — that attribute proved
# unreliable (stuck reporting "still moving" for 20+ seconds on parts
# that were visually motionless, likely contact/decomposition jitter
# never fully quieting to zero in PhysX's own velocity readback).
# Position drift can't lie about whether something visibly moved.
SETTLE_MOVE_TOL = 0.0015   # meters per frame (~9 cm/s if sustained)
SETTLE_FRAMES   = 15

# collection pool (one per production line) — where parts land and
# wait to be picked
CPOOL_CENTER_X = -0.75   # pool center, X (both lines share this X)
CPOOL_HALF_W   = 0.30    # inner half-width  (inner size = 2x this)
CPOOL_HALF_L   = 0.30    # inner half-length
CPOOL_WALL_H   = 0.25    # wall height above pool floor
CPOOL_WALL_T   = 0.05    # wall thickness
PICK_Z         = 0.15    # nominal world Z the arm aims for when locating

# in-feed belt: carries a spawned half from the machine to its pool
BELT_X_MIN     = -2.6    # belt start X (near the machine)
BELT_X_MAX     = -1.12   # belt end X (drops into the pool)
BELT_WIDTH     = 0.35
BELT_TOP       = 0.60    # belt surface height (world Z)
BELT_THICK     = 0.08

# production machine — visual placeholder only, no real "production" logic
MACHINE_POS_X  = -2.9
MACHINE_SIZE   = (0.4, 0.5, 1.2)


PLACE_Y_OFF    = 0.20   # each half's drop point, offset from TABLE_POS[1]
                        # (the table's center) — widened from 0.16 so each
                        # target sits solidly inside its own half (table
                        # half-width is 0.3 m) rather than near the line
PLACE_Z        = 0.88

OUTFEED_X_MIN  = TABLE_POS[0] + TABLE_SIZE[0] / 2  # abuts the table's
                                                    # +X edge exactly —
                                                    # no gap, computed
                                                    # from table geometry
OUTFEED_X_MAX  = 2.40
OUTFEED_WIDTH  = TABLE_SIZE[1]   # matches table width (was BELT_WIDTH,
                                 # 0.35 m) so a welded head rides straight
                                 # off without narrowing
OUTFEED_TOP    = 0.78
OUTFEED_THICK  = 0.08

# deletion pool — the welded head's final stop; deleted once it arrives
DPOOL_CENTER_X = 2.85
DPOOL_HALF_W   = 0.35
DPOOL_HALF_L   = 0.35
DPOOL_WALL_H   = 0.30
DPOOL_WALL_T   = 0.05

# shared display colors for every pool's floor/walls (both collection
# pools and the deletion pool use these)
POOL_FLOOR_COLOR = (0.4, 0.4, 0.6)
POOL_WALL_COLOR  = (0.6, 0.6, 0.8)

# where the 3 physics materials + part material live on the stage
# (created once in add_physics() / ensure_part_material())
MAT_ROOT   = "/World/PhysicsMaterials"
BELT_MAT   = MAT_ROOT + "/BeltMaterial"
POOL_MAT   = MAT_ROOT + "/PoolMaterial"
TABLE_MAT  = MAT_ROOT + "/TableMaterial"
PART_MAT   = MAT_ROOT + "/PartMaterial"

# where every spawned half lives, and how it's spawned
PARTS_ROOT   = "/World/Parts"
SPAWN_MARGIN = 0.15   # spawn point is this far in from the belt's machine end
SPAWN_DROP   = 0.25   # spawn height above the belt surface
SLIDE_SPEED  = 0.3    # small +X nudge at spawn (the belt itself does
                      # the real carrying once switched on)
PART_MASS    = 0.5    # kg, per half

# =================================================================
# STEP 1 — FACTORY LAYOUT
# =================================================================
_created = []

def make_xform(path, translate=(0, 0, 0)):
    """A plain group/parent prim at a world position. Every functional
    unit (LineL, ArmL_Group, AssemblyStation...) is one of these — drag
    it in the viewport and everything underneath moves with it."""
    prim = stage.DefinePrim(Sdf.Path(path), "Xform")
    xf = UsdGeom.Xformable(prim)
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(*translate))
    return prim

def make_box(path, center, size, color, opacity=1.0):
    """Creates a Cube prim scaled/positioned as an axis-aligned box.
    center/size are world units, not the Cube's own unit-cube scale."""
    prim = stage.DefinePrim(Sdf.Path(path), "Cube")
    cube = UsdGeom.Cube(prim)
    cube.CreateSizeAttr(1.0)
    xf = UsdGeom.Xformable(prim)
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(*center))
    xf.AddScaleOp().Set(Gf.Vec3f(*size))
    g = UsdGeom.Gprim(prim)
    g.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    if opacity < 1.0:
        g.CreateDisplayOpacityAttr([opacity])
    _created.append((path, center))
    return prim

def make_marker(path, pos, color=(1.0, 0.2, 0.2), radius=0.03):
    """Small semi-transparent sphere used as a non-physical bookmark
    (PickZone, PlaceTarget). Scripts read its world position at
    runtime instead of hardcoding coordinates, so dragging the parent
    group moves the target too."""
    prim = stage.DefinePrim(Sdf.Path(path), "Sphere")
    UsdGeom.Sphere(prim).CreateRadiusAttr(radius)
    xf = UsdGeom.Xformable(prim)
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(*pos))
    g = UsdGeom.Gprim(prim)
    g.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    g.CreateDisplayOpacityAttr([0.6])
    _created.append((path, pos))
    return prim

def make_pool(root, center_x, center_y, half_w, half_l, wall_h, wall_t):
    """A floor + 4 walls forming an open-top walled pool. Reused for
    both collection pools (one per production line) and the deletion
    pool — same shape, different size/position."""
    inner_w = half_w * 2
    inner_l = half_l * 2
    outer_w = inner_w + wall_t * 2
    wall_z  = wall_h / 2
    cx, cy  = center_x, center_y
    make_xform(root)
    make_box(f"{root}/Floor", (cx, cy, 0.005), (inner_w, inner_l, 0.01), POOL_FLOOR_COLOR)
    make_box(f"{root}/Wall_Front", (cx, cy + half_l + wall_t / 2, wall_z),
             (outer_w, wall_t, wall_h), POOL_WALL_COLOR)
    make_box(f"{root}/Wall_Back", (cx, cy - (half_l + wall_t / 2), wall_z),
             (outer_w, wall_t, wall_h), POOL_WALL_COLOR)
    make_box(f"{root}/Wall_Left", (cx - (half_w + wall_t / 2), cy, wall_z),
             (wall_t, inner_l, wall_h), POOL_WALL_COLOR)
    make_box(f"{root}/Wall_Right", (cx + (half_w + wall_t / 2), cy, wall_z),
             (wall_t, inner_l, wall_h), POOL_WALL_COLOR)

def clean(path):
    """Deletes a prim if it exists. Called before rebuilding any group
    so re-running the script never leaves duplicate/stale geometry."""
    if stage.GetPrimAtPath(path).IsValid():
        stage.RemovePrim(path)

def disable_inherited_collision(root_prim):
    """Explicitly disables collision on every Mesh under root_prim.
    Arm collision was deliberately deferred to a later pass — but the
    KUKA_USD_PATH asset likely ships with its OWN pre-authored collision
    meshes from the original source file, independent of anything we've
    added. That would let a swinging forearm physically shove parts
    around during transit/retreat, which matches what was observed
    (a half reported "settled" 0.89 m from where it was placed with
    millimeter-accurate IK — something hit it after the drop, not a
    placement error). This neutralizes that regardless of what the
    source file authored, until the real arm-collision pass is done."""
    for p in Usd.PrimRange(root_prim):
        if p.IsA(UsdGeom.Mesh):
            p.CreateAttribute("physics:collisionEnabled",
                              Sdf.ValueTypeNames.Bool).Set(False)

def build_layout():
    """Builds/rebuilds the whole static scene: ground, both production
    lines (machine+belt+pool+marker), both KUKA arm references, the
    assembly table+rails, the out-feed belt+rails, the deletion pool.
    Geometry only — add_physics() below is what makes it physical.
    Safe to re-run: clean() removes each group before rebuilding it."""
    if not stage.GetPrimAtPath("/World/GroundPlane").IsValid():
        make_box("/World/GroundPlane", (0, 0, -0.05), (8.0, 8.0, 0.1),
                 (0.35, 0.35, 0.38))

    belt_len = BELT_X_MAX - BELT_X_MIN
    belt_cx  = (BELT_X_MAX + BELT_X_MIN) / 2.0

    for pid, sign, tint in (("L", +1.0, (0.55, 0.45, 0.85)),
                            ("R", -1.0, (0.45, 0.55, 0.85))):
        grp = f"/World/Line{pid}"
        clean(grp)
        make_xform(grp)
        y = sign * LINE_Y
        make_box(f"{grp}/Machine", (MACHINE_POS_X, y, MACHINE_SIZE[2] / 2),
                 MACHINE_SIZE, tint)
        make_box(f"{grp}/Belt", (belt_cx, y, BELT_TOP - BELT_THICK / 2),
                 (belt_len, BELT_WIDTH, BELT_THICK), (0.25, 0.25, 0.25))
        # guardrails along the outlet belt — same style/height as the
        # table's rails — so a part can't slide off sideways en route
        # from machine to pool
        belt_rail_z = BELT_TOP + TABLE_RAIL_HEIGHT / 2
        make_box(f"{grp}/Belt_Rail_Front",
                 (belt_cx, y + BELT_WIDTH / 2 + TABLE_RAIL_THICK / 2, belt_rail_z),
                 (belt_len, TABLE_RAIL_THICK, TABLE_RAIL_HEIGHT), (0.55, 0.45, 0.25))
        make_box(f"{grp}/Belt_Rail_Back",
                 (belt_cx, y - BELT_WIDTH / 2 - TABLE_RAIL_THICK / 2, belt_rail_z),
                 (belt_len, TABLE_RAIL_THICK, TABLE_RAIL_HEIGHT), (0.55, 0.45, 0.25))
        make_pool(f"{grp}/CollectionPool", CPOOL_CENTER_X, y,
                  CPOOL_HALF_W, CPOOL_HALF_L, CPOOL_WALL_H, CPOOL_WALL_T)
        make_marker(f"{grp}/PickZone", (CPOOL_CENTER_X, y, PICK_Z), (1.0, 0.4, 0.1))

    # KUKA arms: referenced at each base with reference-level scaling.
    # Scale sits on the KukaArm CHILD prim (uniform, ARM_SCALE); the group
    # Xform stays scale-free so base translation and future child prims
    # (suction tip pad, markers) are unaffected.
    # Positions come from ARM_POSITIONS (independent per arm — see note
    # above config block) rather than a mirrored formula.
    for pid in ("L", "R"):
        grp = f"/World/Arm{pid}_Group"
        clean(grp)
        ax, ay = ARM_POSITIONS[pid]
        make_xform(grp, translate=(ax, ay, 0.0))
        arm = stage.DefinePrim(Sdf.Path(f"{grp}/KukaArm"), "Xform")
        arm.GetReferences().AddReference(KUKA_USD_PATH)
        disable_inherited_collision(arm)
        disable_inherited_collision(arm)
        axf = UsdGeom.Xformable(arm)
        axf.ClearXformOpOrder()
        axf.AddScaleOp().Set(Gf.Vec3f(ARM_SCALE, ARM_SCALE, ARM_SCALE))
        _created.append((f"{grp}/KukaArm (scale {ARM_SCALE})", (ax, ay, 0.0)))

    clean("/World/AssemblyStation")
    make_xform("/World/AssemblyStation")
    make_box("/World/AssemblyStation/Table",
             (TABLE_POS[0], TABLE_POS[1], TABLE_SIZE[2] / 2),
             (TABLE_SIZE[0], TABLE_SIZE[1], TABLE_SIZE[2]), (0.75, 0.6, 0.3))

    # guardrails: low walls on 3 sides (Y+, Y-, X- / machine side) to stop
    # a knocked or sliding half going over the edge. Arms still reach in
    # fine since they descend from ABOVE, same as the collection pools.
    # +X side (toward OutFeed) is left OPEN for the welded head to
    # eventually conveyor off — no rail there.
    rail_h = TABLE_RAIL_HEIGHT
    rail_t = TABLE_RAIL_THICK
    tx, ty = TABLE_POS
    hw, hl = TABLE_SIZE[0] / 2, TABLE_SIZE[1] / 2
    top = TABLE_SIZE[2]
    rail_color = (0.55, 0.45, 0.25)
    make_box("/World/AssemblyStation/Rail_Front",       # +Y side
             (tx, ty + hl + rail_t / 2, top + rail_h / 2),
             (TABLE_SIZE[0] + rail_t * 2, rail_t, rail_h), rail_color)
    make_box("/World/AssemblyStation/Rail_Back",        # -Y side
             (tx, ty - hl - rail_t / 2, top + rail_h / 2),
             (TABLE_SIZE[0] + rail_t * 2, rail_t, rail_h), rail_color)
    make_box("/World/AssemblyStation/Rail_MachineSide",  # -X side
             (tx - hw - rail_t / 2, ty, top + rail_h / 2),
             (rail_t, TABLE_SIZE[1], rail_h), rail_color)
    # (intentionally no rail on +X — that's the OutFeed exit)

    make_marker("/World/AssemblyStation/PlaceTarget_L",
                (TABLE_POS[0], TABLE_POS[1] + PLACE_Y_OFF, PLACE_Z), (0.2, 0.9, 0.3))
    make_marker("/World/AssemblyStation/PlaceTarget_R",
                (TABLE_POS[0], TABLE_POS[1] - PLACE_Y_OFF, PLACE_Z), (0.2, 0.9, 0.3))

    clean("/World/OutFeed")
    make_xform("/World/OutFeed")
    of_len = OUTFEED_X_MAX - OUTFEED_X_MIN
    of_cx  = (OUTFEED_X_MAX + OUTFEED_X_MIN) / 2.0
    make_box("/World/OutFeed/Belt", (of_cx, 0.0, OUTFEED_TOP - OUTFEED_THICK / 2),
             (of_len, OUTFEED_WIDTH, OUTFEED_THICK), (0.25, 0.25, 0.25))
    # side guardrails along the belt's full length, same style/height as
    # the table's rails, so a completed head can't slide off sideways
    of_rail_z = OUTFEED_TOP + TABLE_RAIL_HEIGHT / 2
    make_box("/World/OutFeed/Rail_Front",
             (of_cx, OUTFEED_WIDTH / 2 + TABLE_RAIL_THICK / 2, of_rail_z),
             (of_len, TABLE_RAIL_THICK, TABLE_RAIL_HEIGHT), (0.55, 0.45, 0.25))
    make_box("/World/OutFeed/Rail_Back",
             (of_cx, -(OUTFEED_WIDTH / 2 + TABLE_RAIL_THICK / 2), of_rail_z),
             (of_len, TABLE_RAIL_THICK, TABLE_RAIL_HEIGHT), (0.55, 0.45, 0.25))
    make_pool("/World/OutFeed/DeletionPool", DPOOL_CENTER_X, 0.0,
              DPOOL_HALF_W, DPOOL_HALF_L, DPOOL_WALL_H, DPOOL_WALL_T)
    make_box("/World/OutFeed/DeletionPool/DeletionTrigger",
             (DPOOL_CENTER_X, 0.0, 0.20),
             (DPOOL_HALF_W * 1.8, DPOOL_HALF_L * 1.8, 0.35),
             (0.9, 0.15, 0.15), opacity=0.30)
    print(f"[1/4] layout built ({len(_created)} prims)")

# =================================================================
# STEP 2 — PHYSICS
# =================================================================
def make_physics_material(path, restitution, dyn_fric, stat_fric,
                          rest_combine, fric_combine="average"):
    """Defines a PhysX material (bounciness + friction). rest_combine
    controls how two touching materials' restitution is combined —
    "min" means the LESS bouncy of the two always wins, which is what
    keeps parts from bouncing even if something else they touch is
    springy."""
    mat = UsdShade.Material.Define(stage, path)
    prim = mat.GetPrim()
    pm = UsdPhysics.MaterialAPI.Apply(prim)
    pm.CreateRestitutionAttr(restitution)
    pm.CreateDynamicFrictionAttr(dyn_fric)
    pm.CreateStaticFrictionAttr(stat_fric)
    try:
        from pxr import PhysxSchema
        px = PhysxSchema.PhysxMaterialAPI.Apply(prim)
        px.CreateRestitutionCombineModeAttr(rest_combine)
        px.CreateFrictionCombineModeAttr(fric_combine)
    except Exception:
        prim.CreateAttribute("physxMaterial:restitutionCombineMode",
                             Sdf.ValueTypeNames.Token).Set(rest_combine)
        prim.CreateAttribute("physxMaterial:frictionCombineMode",
                             Sdf.ValueTypeNames.Token).Set(fric_combine)
    return mat

def bind_physics_material(prim, mat_path):
    """Attaches an already-defined material (by path) to a prim."""
    mat = UsdShade.Material(stage.GetPrimAtPath(mat_path))
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(
        mat, UsdShade.Tokens.strongerThanDescendants, "physics")

def add_static_collider(path, mat_path):
    """Collision only, never moves. Used for walls, table, rails,
    ground — anything solid that isn't a conveyor and isn't a part."""
    prim = stage.GetPrimAtPath(path)
    if not prim.IsValid():
        return False
    UsdPhysics.CollisionAPI.Apply(prim)
    bind_physics_material(prim, mat_path)
    return True

def add_kinematic_body(path, mat_path):
    """Collision + kinematic rigid body. This is the prerequisite for
    a prim to later receive PhysX surface velocity (i.e. become a
    conveyor) — used for all 4 belts (3 in/out-feed + the table)."""
    prim = stage.GetPrimAtPath(path)
    if not prim.IsValid():
        return False
    UsdPhysics.CollisionAPI.Apply(prim)
    rb = UsdPhysics.RigidBodyAPI.Apply(prim)
    rb.CreateKinematicEnabledAttr(True)
    bind_physics_material(prim, mat_path)
    return True

def add_physics():
    """Makes everything build_layout() created physical: PhysicsScene,
    3 materials (belt/pool/table — each tuned for a different job:
    belts near-zero bounce so parts ride not bounce, pools higher
    friction so tumbled parts settle fast, table highest friction so
    halves sit dead still for the snap), then applies colliders to
    every relevant prim by path."""
    if not any(p.IsA(UsdPhysics.Scene) for p in stage.Traverse()):
        UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")

    stage.DefinePrim(Sdf.Path(MAT_ROOT), "Scope")
    make_physics_material(BELT_MAT,  0.05, 0.5, 0.6, "min")
    make_physics_material(POOL_MAT,  0.10, 0.8, 0.9, "multiply")
    make_physics_material(TABLE_MAT, 0.02, 0.9, 1.0, "min")

    add_static_collider("/World/GroundPlane", POOL_MAT)
    for pid in ("L", "R"):
        grp = f"/World/Line{pid}"
        add_static_collider(f"{grp}/Machine", BELT_MAT)
        add_kinematic_body(f"{grp}/Belt", BELT_MAT)
        for rail in ("Belt_Rail_Front", "Belt_Rail_Back"):
            add_static_collider(f"{grp}/{rail}", BELT_MAT)
        for wall in ("Floor", "Wall_Front", "Wall_Back", "Wall_Left", "Wall_Right"):
            add_static_collider(f"{grp}/CollectionPool/{wall}", POOL_MAT)
    add_kinematic_body("/World/AssemblyStation/Table", TABLE_MAT)  # now
                        # conveyor-ready (surface velocity), not just static
    for rail in ("Rail_Front", "Rail_Back", "Rail_MachineSide"):
        add_static_collider(f"/World/AssemblyStation/{rail}", TABLE_MAT)
    add_kinematic_body("/World/OutFeed/Belt", BELT_MAT)
    for rail in ("Rail_Front", "Rail_Back"):
        add_static_collider(f"/World/OutFeed/{rail}", BELT_MAT)
    for wall in ("Floor", "Wall_Front", "Wall_Back", "Wall_Left", "Wall_Right"):
        add_static_collider(f"/World/OutFeed/DeletionPool/{wall}", POOL_MAT)
    print("[2/4] physics applied (belts = kinematic, conveyor-ready)")

# =================================================================
# STEP 3 — HALF PRODUCTION (functions; UI lives in the panel)
# =================================================================
BELT_PRIMS = {"L": "/World/LineL/Belt", "R": "/World/LineR/Belt"}
counters = {"L": 0, "R": 0}

def ensure_part_material():
    """Creates PART_MAT once (idempotent) — shared by every spawned
    half so a single restitution/friction change here affects all of
    them, including already-welded pairs."""
    if stage.GetPrimAtPath(PART_MAT).IsValid():
        return
    # restitution lowered from 0.05 to 0.01 — less bounce on impact.
    # combine="min" already ensures the LOWER of any two contacting
    # materials' restitution wins, so this applies to every surface
    # the part touches (belts, pools, table) with no other changes.
    make_physics_material(PART_MAT, 0.01, 0.6, 0.7, "min")

def get_spawn_point(pid):
    """Reads the line's belt prim's ACTUAL world position/scale and
    returns a spawn point at its machine-end, just above the surface.
    Following the belt (instead of a hardcoded coordinate) means
    dragging /World/LineL or /World/LineR still spawns in the right
    place."""
    prim = stage.GetPrimAtPath(BELT_PRIMS[pid])
    if not prim.IsValid():
        return None
    xf = UsdGeom.Xformable(prim)
    m = xf.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    pos = m.ExtractTranslation()
    sx = sz = None
    for op in xf.GetOrderedXformOps():
        if op.GetOpType() == UsdGeom.XformOp.TypeScale:
            s = op.Get()
            sx, sz = s[0], s[2]
    if sx is None:
        sx, sz = 1.48, 0.08   # fallback if the belt has no scale op yet
    return (pos[0] - sx / 2.0 + SPAWN_MARGIN, pos[1],
            pos[2] + sz / 2.0 + SPAWN_DROP)

def produce_half(pid, slide_speed=None):
    """Spawns one monkey half (L or R) as a reference to its .usd
    file, tags it piece_id/head_id (read later by the arms and the
    snap logic), gives it a small +X nudge, and applies
    convexDecomposition collision — REQUIRED here: a plain convex hull
    would seal over the cut face and the peg/socket key, and the two
    halves could never physically mate."""
    if pid not in HALF_PATHS:
        return None
    spawn = get_spawn_point(pid)
    if spawn is None:
        _panel.log(f"Line {pid} belt not found")
        return None
    ensure_part_material()
    stage.DefinePrim(Sdf.Path(PARTS_ROOT), "Scope")

    counters[pid] += 1
    n = counters[pid]
    path = f"{PARTS_ROOT}/Half_{pid}_{n}"

    prim = stage.DefinePrim(Sdf.Path(path), "Xform")
    prim.GetReferences().AddReference(HALF_PATHS[pid])
    xf = UsdGeom.Xformable(prim)
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(*spawn))

    # piece_id: which line/side. head_id: this line's running count —
    # also doubles as the FIFO order number the arm's locate_part() reads.
    prim.CreateAttribute("piece_id", Sdf.ValueTypeNames.String).Set(pid)
    prim.CreateAttribute("head_id", Sdf.ValueTypeNames.Int).Set(n)

    v = SLIDE_SPEED if slide_speed is None else slide_speed
    rb = UsdPhysics.RigidBodyAPI.Apply(prim)
    rb.CreateVelocityAttr(Gf.Vec3f(v, 0.0, -0.2))
    UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(PART_MASS)

    for p in Usd.PrimRange(prim):
        if p.IsA(UsdGeom.Mesh):
            UsdPhysics.CollisionAPI.Apply(p)
            UsdPhysics.MeshCollisionAPI.Apply(p).CreateApproximationAttr(
                "convexDecomposition")

    bind_physics_material(prim, PART_MAT)
    _panel.set_line_status(pid, produced=counters[pid])
    _panel.log(f"produced Half_{pid}_{n}")
    return path

def produce_pair():
    """One half on each line — this is what "Produce Part" actually does."""
    produce_half("L")
    produce_half("R")

def clear_parts():
    """Deletes every prim under PARTS_ROOT and resets all counters/
    settle-tracking state. Used by "Clear all parts" and internally by
    the panel on re-run."""
    root = stage.GetPrimAtPath(PARTS_ROOT)
    if root.IsValid():
        for child in list(root.GetChildren()):
            stage.RemovePrim(child.GetPath())
    counters["L"] = 0
    counters["R"] = 0
    _settle_state.clear()
    _settle_last_pos.clear()
    for pid in ("L", "R"):
        _panel.set_line_status(pid, produced=0, in_pool=0)
    _panel.log("all parts cleared")

# =================================================================
# STEP 3b — SETTLING GATE
#
# One global subscription (NOT one per part — that pattern leaked
# forever-alive callbacks in the old ball-pool generator). Every
# frame it measures each part's WORLD POSITION and compares it to
# last frame's, counting consecutive frames under SETTLE_MOVE_TOL.
# is_settled() is the single source of truth every arm action must
# check before touching a part — this is what implements "all
# actions are prohibited before the part has come to rest."
#
# Deliberately NOT using RigidBodyAPI's velocity attribute: that
# attribute proved unreliable in testing — it read as "still moving"
# continuously for 20+ seconds on parts that were visually motionless
# on the table, most likely because PhysX's velocity readback never
# fully quiets to zero under persistent low-level contact/decomposition
# jitter. Position drift is a direct, physics-API-agnostic signal —
# it can't disagree with what's actually visible in the viewport.
# =================================================================
_settle_state = {}      # part path -> consecutive frames under threshold
_settle_last_pos = {}   # part path -> world position last frame
_settle_sub = None

def _settle_tick(e):
    root = stage.GetPrimAtPath(PARTS_ROOT)
    if not root.IsValid():
        return
    alive = set()
    for child in root.GetChildren():
        path = child.GetPath().pathString
        alive.add(path)
        w = UsdGeom.Xformable(child).ComputeLocalToWorldTransform(
            Usd.TimeCode.Default())
        pos = w.ExtractTranslation()
        prev = _settle_last_pos.get(path)
        _settle_last_pos[path] = pos
        if prev is None:
            _settle_state[path] = 0          # first frame seen: no data yet
            continue
        moved = (pos - prev).GetLength()
        if moved < SETTLE_MOVE_TOL:
            _settle_state[path] = _settle_state.get(path, 0) + 1
        else:
            _settle_state[path] = 0
    for stale in [p for p in _settle_state if p not in alive]:
        del _settle_state[stale]     # part deleted/moved: drop its count
        _settle_last_pos.pop(stale, None)

def start_settle_tracking():
    global _settle_sub
    if _settle_sub is None:
        _settle_sub = omni.kit.app.get_app().get_update_event_stream() \
            .create_subscription_to_pop(_settle_tick, name="settle_tracker")

def is_settled(part_path):
    return _settle_state.get(part_path, 0) >= SETTLE_FRAMES

# =================================================================
# STEP 4 — CONVEYOR CONTROL + STATE MACHINE PANEL
# =================================================================
class ConveyorController:
    """Drives all 4 belts via PhysX surface velocity: the belt
    geometry never moves, but its contact surface carries anything
    resting on it — the standard sim-conveyor mechanism. Requires each
    belt prim to already be a kinematic rigid body (add_kinematic_body
    in STEP 2)."""
    BELTS = {
        "outfeed": "/World/OutFeed/Belt",
        "inL":     "/World/LineL/Belt",
        "inR":     "/World/LineR/Belt",
        "table":   "/World/AssemblyStation/Table",  # 4th belt — automatically
                  # included in start_all()/stop_all() and the panel's
                  # "Start/Stop ALL belts" button just by being in this dict
    }
    DIRECTION = Gf.Vec3f(1.0, 0.0, 0.0)   # every belt carries toward +X

    def __init__(self, log_fn=print):
        self._log = log_fn
        self._running = {name: False for name in self.BELTS}

    def _prim(self, name):
        """Looks up a belt's prim by its short name (e.g. "inL")."""
        prim = stage.GetPrimAtPath(self.BELTS[name])
        if not prim.IsValid():
            self._log(f"conveyor '{name}': belt prim missing")
            return None
        return prim

    def _set_surface_velocity(self, prim, vel_vec, enabled):
        """Applies PhysxSurfaceVelocityAPI; falls back to raw
        attributes if that schema class isn't exposed in this build."""
        try:
            from pxr import PhysxSchema
            api = PhysxSchema.PhysxSurfaceVelocityAPI.Apply(prim)
            api.CreateSurfaceVelocityAttr(vel_vec)
            api.CreateSurfaceVelocityEnabledAttr(enabled)
        except Exception:
            prim.CreateAttribute("physxSurfaceVelocity:surfaceVelocity",
                                 Sdf.ValueTypeNames.Float3).Set(vel_vec)
            prim.CreateAttribute("physxSurfaceVelocity:surfaceVelocityEnabled",
                                 Sdf.ValueTypeNames.Bool).Set(enabled)

    def start(self, name, speed=0.4):
        """Turns one belt on at the given speed (m/s, +X)."""
        prim = self._prim(name)
        if prim is None:
            return False
        self._set_surface_velocity(prim, self.DIRECTION * float(speed), True)
        self._running[name] = True
        self._log(f"conveyor '{name}' START at {speed:.2f} m/s (+X)")
        return True

    def stop(self, name):
        """Turns one belt off."""
        prim = self._prim(name)
        if prim is None:
            return False
        self._set_surface_velocity(prim, Gf.Vec3f(0.0), False)
        self._running[name] = False
        self._log(f"conveyor '{name}' STOP")
        return True

    def start_all(self, speed=0.4):
        """Starts every belt in BELTS — this is "Start ALL belts"."""
        for name in self.BELTS:
            self.start(name, speed)

    def stop_all(self):
        """Stops every belt in BELTS — this is "Stop ALL belts"."""
        for name in self.BELTS:
            self.stop(name)

    def any_running(self):
        """True if at least one belt is on (drives the master button's
        Start/Stop label)."""
        return any(self._running.values())

    def is_running(self, name):
        return self._running.get(name, False)


# =================================================================
# STEP 5 — ARM MOTION (pick from pool -> place on table)
#
# Built on your 20260328_9.py pattern:
#   - smoothstep-eased matrix mutation on each joint's Transform op
#   - ONE JOINT AT A TIME per arm (both arms may run in parallel)
#   - suction = reparent under A6 with world-pose compensation
#     (part's rigid body is disabled while held, re-enabled on release)
#
# Upgrade over the reference: joint angles are COMPUTED, not fixed.
# At startup each arm calibrates a forward-kinematics model from the
# stage (joint init matrices + parent world transform), then solves
# "reverse calculation" (IK) for measured part positions by searching
# the FK model. No hardcoded link lengths, signs, or orientations —
# whatever correction transforms sit above the arm are automatically
# included, because the model is built from what is actually there.
# =================================================================
def _smooth(t):
    return t * t * (3.0 - 2.0 * t)

def _rot(axis, deg):
    return Gf.Matrix4d().SetRotate(Gf.Rotation(axis, deg))

def _reset_to_transform_op(prim):
    """Clear a prim's xform ops and give it a single fresh Transform op.
    Also removes any orphaned xformOp:translate/rotate/scale attributes
    left behind by ClearXformOpOrder() (which only clears the ORDER
    list, not the underlying attributes) — this is what was causing
    the 'cannot find xform op xformOp:translate' Hydra warning."""
    xf = UsdGeom.Xformable(prim)
    xf.ClearXformOpOrder()
    for stale in ("xformOp:translate", "xformOp:rotateXYZ",
                 "xformOp:rotateX", "xformOp:orient", "xformOp:scale"):
        if prim.HasAttribute(stale):
            prim.RemoveProperty(stale)
    return xf.AddTransformOp()

class ArmController:
    """One instance per arm (ARMS["L"], ARMS["R"]). Rather than
    hand-deriving link lengths/signs, each instance CALIBRATES its own
    forward-kinematics model directly from the stage at startup (reads
    the actual joint transform ops), then solves inverse kinematics by
    brute-force grid search over that model. Whatever scale/position
    correction sits above the arm in the hierarchy is automatically
    included, because the model is built from what's really there."""
    AXES = {"a1": Gf.Vec3d(0, 0, 1),   # from your reference script
            "a2": Gf.Vec3d(0, 1, 0),
            "a3": Gf.Vec3d(0, 1, 0)}

    def __init__(self, pid):
        """pid: "L" or "R". Just records paths/defaults — call
        calibrate() before using the arm for anything."""
        self.pid = pid
        base = (f"/World/Arm{pid}_Group/KukaArm/Geometry/ROOT_0/"
                f"KR_10_R1440_2_1")
        self.base_path = base
        self.paths = {"a1": f"{base}/A1_3",
                      "a2": f"{base}/A1_3/A2_5",
                      "a3": f"{base}/A1_3/A2_5/A3_7"}
        self.a6_path = f"{base}/A1_3/A2_5/A3_7/A4_91/A5_93/A6_95"
        self.ops, self.init = {}, {}
        self.tail = Gf.Matrix4d(1.0)   # A4*A5*A6 local (constant, not driven)
        self.parent_w = Gf.Matrix4d(1.0)
        self.angles = {"a1": 0.0, "a2": 0.0, "a3": 0.0}
        self.busy = False
        self.held = None               # (part_path, rel_matrix, xform_op)
        self.retries = 0
        self.ok = False

    # ---- calibration: build the FK model from the stage --------------
    def _op_and_init(self, path):
        """Finds (or creates) the Transform xform op on a joint and
        returns (op, its current matrix) — this matrix becomes the
        joint's "zero angle" reference for fk()/solve_ik()."""
        prim = stage.GetPrimAtPath(path)
        if not prim.IsValid():
            return None, None
        xf = UsdGeom.Xformable(prim)
        for op in xf.GetOrderedXformOps():
            if op.GetOpType() == UsdGeom.XformOp.TypeTransform:
                m = op.Get()
                return op, (m if m is not None else Gf.Matrix4d(1.0))
        # no Transform op: bake current local transform into one
        m = xf.GetLocalTransformation()
        xf.ClearXformOpOrder()
        op = xf.AddTransformOp()
        op.Set(m)
        return op, m

    def calibrate(self):
        """Reads A1/A2/A3's current transforms as the FK model's zero
        pose, plus the constant wrist "tail" (A4-A6, never driven) and
        the arm's world position. Must succeed (self.ok=True) before
        this arm can be used — logs and bails if any joint is missing."""
        for k, p in self.paths.items():
            op, init = self._op_and_init(p)
            if op is None:
                _panel.log(f"Arm {self.pid}: joint missing: {p}")
                self.ok = False
                return False
            self.ops[k], self.init[k] = op, init
        # constant wrist tail (A4, A5, A6 local transforms)
        tail = Gf.Matrix4d(1.0)
        node = stage.GetPrimAtPath(self.a6_path)
        while node.IsValid() and node.GetPath() != Sdf.Path(self.paths["a3"]):
            tail = tail * UsdGeom.Xformable(node).GetLocalTransformation()
            node = node.GetParent()
        self.tail = tail
        self.parent_w = UsdGeom.Xformable(
            stage.GetPrimAtPath(self.base_path)
        ).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        self.ok = True
        tip = self.fk(0, 0, 0)
        _panel.log(f"Arm {self.pid} calibrated, home tip at "
                   f"({tip[0]:+.2f}, {tip[1]:+.2f}, {tip[2]:+.2f})")
        return True

    # ---- forward kinematics (pure math, never touches the stage) -----
    def fk(self, a1, a2, a3):
        """Given 3 joint angles (degrees), returns the tool tip's
        world position. Pure math against the calibrated model —
        doesn't read or write the stage, so it's cheap to call
        thousands of times during an IK search."""
        m1 = _rot(self.AXES["a1"], a1) * self.init["a1"]
        m2 = _rot(self.AXES["a2"], a2) * self.init["a2"]
        m3 = _rot(self.AXES["a3"], a3) * self.init["a3"]
        m = self.tail * m3 * m2 * m1 * self.parent_w
        return m.Transform(Gf.Vec3d(0, 0, 0))     # A6 origin = tool tip

    # ---- inverse kinematics: search the FK model ---------------------
    def solve_ik(self, target):
        """Given a world point, finds (a1, a2, a3) that puts the tool
        tip there: coarse grid search over a1 (yaw toward the target),
        then a coarse 2D grid over a2/a3 refined by two shrinking
        passes. Returns (a1, a2, a3, residual_error_m)."""
        tx, ty, tz = target[0], target[1], target[2]

        def herr(a1):   # horizontal error with current a2/a3
            p = self.fk(a1, self.angles["a2"], self.angles["a3"])
            return math.hypot(p[0] - tx, p[1] - ty)

        best_a1 = min((a * 3.0 for a in range(-60, 61)), key=herr)
        best_a1 = min((best_a1 + d * 0.5 for d in range(-6, 7)), key=herr)

        def perr(a2, a3):
            p = self.fk(best_a1, a2, a3)
            return math.hypot(math.hypot(p[0] - tx, p[1] - ty), p[2] - tz)

        best, be = (0.0, 0.0), perr(0.0, 0.0)
        for a2 in range(-100, 101, 5):
            for a3 in range(-130, 131, 5):
                e = perr(a2, a3)
                if e < be:
                    best, be = (float(a2), float(a3)), e
        for step in (1.0, 0.25):
            b2, b3 = best
            for a2 in (b2 + i * step for i in range(-5, 6)):
                for a3 in (b3 + j * step for j in range(-5, 6)):
                    e = perr(a2, a3)
                    if e < be:
                        best, be = (a2, a3), e
        return best_a1, best[0], best[1], be

    def safe_transit_pose(self):
        """A2/A3 angles that put the tool well above BOTH the table
        (0.8 m) and the pool walls (0.25 m) — solved via IK against an
        explicit high point above this arm's own base, rather than
        assuming raw angles=0 happen to be tall/retracted enough.
        Using this before every A1 sweep (instead of a blind 0,0)
        is what actually gives clearance during the swing — the
        previous 'unrealistic/clipping' motion was sweeping at
        whatever height angles=0 produced, which was never verified
        to clear anything."""
        bx, by = ARM_POSITIONS[self.pid]
        # a point a modest distance in front of the base, well above
        # every obstacle in the scene; a1 result is discarded, only
        # a2/a3 (which set the height/retraction) are used
        target = Gf.Vec3d(bx * 0.5, by * 0.5, SAFE_TRANSIT_Z)
        _, a2, a3, _ = self.solve_ik(target)
        return a2, a3

    # ---- single-joint eased move (your reference pattern) ------------
    async def move_joint(self, key, target, frames=SEQ_FRAMES, side_guard=None):
        """side_guard: this arm's own-half sign relative to the table
        center (+1 for L, -1 for R). If set, computes the tool's Y each
        frame via FK and logs ONCE the instant it crosses (with 2 cm
        tolerance) into the other arm's half — concrete evidence of
        whether a sweep actually leaves its own territory, rather than
        inferring it from the outside."""
        start = self.angles[key]
        if abs(target - start) < 1e-3:
            return
        app = omni.kit.app.get_app()
        warned = False
        for i in range(frames + 1):
            t = _smooth(i / frames)
            a = start + (target - start) * t
            self.ops[key].Set(_rot(self.AXES[key], a) * self.init[key])
            if side_guard is not None and not warned:
                cur = dict(self.angles)
                cur[key] = a
                p = self.fk(cur["a1"], cur["a2"], cur["a3"])
                if p[1] * side_guard < -0.02:
                    _panel.log(f"Arm {self.pid}: WARNING tool crossed "
                              f"table centerline during {key} sweep "
                              f"(y={p[1]:.2f} m)")
                    warned = True
            if self.held:
                self._update_held_pose()   # follow the tip, no reparenting
            await app.next_update_async()
        self.angles[key] = target

    # ---- part search / suction ---------------------------------------
    def locate_part(self):
        """FIFO by trailing number in the part's name — the lowest-
        numbered candidate in this line's pool is always offered
        first, never skipped even if a later-numbered part settles
        sooner. Returns None (and logs why) if the pool is empty or
        the next-in-line part hasn't settled yet; no other action may
        target a part until this returns non-None for it."""
        zone = stage.GetPrimAtPath(f"/World/Line{self.pid}/PickZone")
        if not zone.IsValid():
            return None
        zp = UsdGeom.Xformable(zone).ComputeLocalToWorldTransform(
            Usd.TimeCode.Default()).ExtractTranslation()
        cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                                  [UsdGeom.Tokens.default_])
        root = stage.GetPrimAtPath(PARTS_ROOT)
        if not root.IsValid():
            return None

        candidates = []   # (number, path, grip_x, grip_y, grip_z)
        for child in root.GetChildren():
            attr = child.GetAttribute("piece_id")
            if not attr or attr.Get() != self.pid:
                continue
            rb_en = UsdPhysics.RigidBodyAPI(child).GetRigidBodyEnabledAttr()
            if rb_en and rb_en.Get() is False:
                continue        # currently attached to an arm — not available
            rng = cache.ComputeWorldBound(child).ComputeAlignedRange()
            if rng.IsEmpty():
                continue
            mn, mx = rng.GetMin(), rng.GetMax()
            cx, cy = (mn[0] + mx[0]) / 2, (mn[1] + mx[1]) / 2
            if math.hypot(cx - zp[0], cy - zp[1]) > POOL_RADIUS:
                continue        # not in this line's pool yet (e.g. on the belt)
            m = re.search(r'_(\d+)(?:_r\d+[A-Z])?$', child.GetName())
            n = int(m.group(1)) if m else 0
            candidates.append((n, child.GetPath().pathString, cx, cy, mx[2]))

        if not candidates:
            return None

        candidates.sort(key=lambda c: c[0])        # FIFO, no skipping
        n, path, cx, cy, top_z = candidates[0]

        if not is_settled(path):
            return None                    # present but still moving —
                                           # caller (run_retrieve) polls
                                           # and logs a throttled heartbeat

        return (path, Gf.Vec3d(cx, cy, top_z))     # top-center grip point

    def _tip_world(self):
        return UsdGeom.Xformable(
            stage.GetPrimAtPath(self.a6_path)
        ).ComputeLocalToWorldTransform(Usd.TimeCode.Default())

    def attach(self, part_path):
        """Suction attach WITHOUT reparenting. Reparenting a live rigid
        body (via MovePrim) while PhysX is stepping is a known crash
        vector — it forces a structural stage change mid-simulation.
        Instead: freeze the part's physics, switch its xform to a
        single Transform op, and drive that op every frame in
        move_joint() so it follows the tip. The part never leaves
        /World/Parts; only its own attribute is edited each frame."""
        prim = stage.GetPrimAtPath(part_path)
        if not prim.IsValid():
            return False
        rb = UsdPhysics.RigidBodyAPI(prim)
        rb.CreateRigidBodyEnabledAttr(False)          # freeze physics
        rb.CreateVelocityAttr(Gf.Vec3f(0.0))

        w = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(
            Usd.TimeCode.Default())
        rel = w * self._tip_world().GetInverse()      # part pose relative to tip

        op = _reset_to_transform_op(prim)
        op.Set(w)
        self.held = (part_path, rel, op)
        self._update_held_pose()                      # sync immediately
        return True

    def _update_held_pose(self):
        """Called every animation frame while holding a part: keeps it
        rigidly following the tip via a pure attribute Set (no prim
        hierarchy change, safe to call during active simulation)."""
        if not self.held:
            return
        _, rel, op = self.held
        op.Set(rel * self._tip_world())

    def release(self):
        if not self.held:
            return None
        part_path, rel, op = self.held
        prim = stage.GetPrimAtPath(part_path)
        if not prim.IsValid():
            self.held = None
            return None
        rb = UsdPhysics.RigidBodyAPI(prim)
        rb.CreateRigidBodyEnabledAttr(True)           # live again -> settles
        rb.CreateVelocityAttr(Gf.Vec3f(0.0))
        self.held = None
        return part_path

    # ---- the full retrieve cycle (one joint at a time) ----------------
    async def run_retrieve(self):
        if self.busy or not self.ok:
            _panel.log(f"Arm {self.pid}: not ready (busy or uncalibrated)")
            return False
        self.busy = True
        st = lambda s: _panel.set_arm_status(self.pid, state=s)
        try:
            st("LOCATING")
            app = omni.kit.app.get_app()
            found = None
            for frame_i in range(LOCATE_TIMEOUT_F):
                found = self.locate_part()
                if found is not None:
                    break
                if frame_i % 60 == 0 and frame_i > 0:
                    _panel.log(f"Arm {self.pid}: still waiting for a "
                              f"part to settle in the pool")
                await app.next_update_async()
            if found is None:
                _panel.log(f"Arm {self.pid}: no part settled within "
                          f"{LOCATE_TIMEOUT_F/60:.0f}s, giving up")
                st("WAITING")
                return False
            part_path, grip = found
            hover = Gf.Vec3d(grip[0], grip[1], grip[2] + HOVER_DZ)

            a1g, a2g, a3g, eg = self.solve_ik(grip)
            a1h, a2h, a3h, eh = self.solve_ik(hover)
            _panel.log(f"Arm {self.pid}: IK grip err {eg*1000:.0f} mm, "
                       f"hover err {eh*1000:.0f} mm")

            own_half = 1.0 if self.pid == "L" else -1.0
            st("APPROACHING")                    # one joint at a time:
            await self.move_joint("a1", a1g, A1_FRAMES, side_guard=own_half)
            await self.move_joint("a2", a2h)
            await self.move_joint("a3", a3h)
            await self.move_joint("a2", a2g)
            await self.move_joint("a3", a3g)

            # suction check against the part's CURRENT position
            tip = self._tip_world().ExtractTranslation()
            prim = stage.GetPrimAtPath(part_path)
            if not prim.IsValid():
                raise RuntimeError("part vanished")
            pw = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(
                Usd.TimeCode.Default()).ExtractTranslation()
            dist = math.hypot(math.hypot(tip[0] - pw[0], tip[1] - pw[1]),
                              tip[2] - pw[2])
            if dist > GRAB_TOL + 0.15:
                self.retries += 1
                _panel.set_arm_status(self.pid, retries=self.retries)
                _panel.log(f"Arm {self.pid}: suction miss "
                           f"({dist*100:.0f} cm), retreating")
                await self.go_home(step_status=st)
                return False

            st("GRIPPING")
            self.attach(part_path)
            _panel.set_arm_status(self.pid, grip="HOLDING")

            st("LIFTING")
            a2s, a3s = self.safe_transit_pose()
            await self.move_joint("a3", a3s)
            await self.move_joint("a2", a2s)

            marker = stage.GetPrimAtPath(
                f"/World/AssemblyStation/PlaceTarget_{self.pid}")
            mp = UsdGeom.Xformable(marker).ComputeLocalToWorldTransform(
                Usd.TimeCode.Default()).ExtractTranslation()
            place = Gf.Vec3d(mp[0], mp[1], mp[2] + PLACE_DZ)
            a1p, a2p, a3p, ep = self.solve_ik(place)
            _panel.log(f"Arm {self.pid}: IK place err {ep*1000:.0f} mm")

            st("TRANSITING")
            await self.move_joint("a1", a1p, A1_FRAMES, side_guard=own_half)

            st("PLACING")
            await self.move_joint("a2", a2p)
            await self.move_joint("a3", a3p)

            st("RELEASING")
            out = self.release()
            _panel.set_arm_status(self.pid, grip="OPEN")
            _panel.log(f"Arm {self.pid}: placed {out}")

            await self.go_home(step_status=st)
            return True
        except Exception as e:
            _panel.log(f"Arm {self.pid} FAULT: {e}")
            st("WAITING")
            return False
        finally:
            self.busy = False

    async def go_home(self, step_status=None):
        if step_status:
            step_status("RETREATING")
        a2s, a3s = self.safe_transit_pose()
        await self.move_joint("a3", a3s)
        await self.move_joint("a2", a2s)
        if step_status:
            step_status("HOMING")
        own_half = 1.0 if self.pid == "L" else -1.0
        await self.move_joint("a1", 0.0, A1_FRAMES, side_guard=own_half)
        if step_status:
            step_status("WAITING")


# =================================================================
# STEP 6 — AUTOMATIC SNAP & WELD
#
# Triggered automatically once both arms finish placing (no button).
# Fires only when BOTH halves are near their PlaceTarget marker AND
# settled (same is_settled() gate used for picking) — reuses the
# existing infrastructure rather than inventing a new one.
#
# Motion: L is the fixed anchor; R is kinematically interpolated
# (position lerp + rotation slerp, smoothstep timing) onto L's exact
# world transform. Because both halves were exported from Blender
# sharing one origin (the assembled head's center), "L's transform ==
# R's transform" IS the correct mate — no separate offset needed.
#
# Weld: a real UsdPhysics.FixedJoint. A joint is a relationship, not
# a reparent, so — unlike the suction MovePrim crash — it never
# touches stage hierarchy and is safe to create during simulation.
# =================================================================
PLACE_RADIUS   = 0.40   # search radius around each PlaceTarget marker (m)
                        # widened from 0.25 — measured placements landed
                        # at 0.30-0.32 m, just outside the old radius,
                        # even though both halves were fully settled
SNAP_FRAMES    = 90     # interpolation length (frames)
SNAP_TIMEOUT_F = 1200   # ~20 s at 60fps: give up waiting if a half never settles

_snap_task = None
_weld_count = 0
_snap_debug = {"L": None, "R": None}   # last diagnostic per side, for the heartbeat

def find_settled_on_table(pid):
    """Like locate_part, but scoped to this half's PlaceTarget marker
    instead of its pool. Returns a part path, or None if nothing
    matching/settled is there yet. Always records WHY in _snap_debug[pid]
    so the heartbeat can report a real number instead of just none/present."""
    marker = stage.GetPrimAtPath(f"/World/AssemblyStation/PlaceTarget_{pid}")
    if not marker.IsValid():
        _snap_debug[pid] = {"reason": "PlaceTarget marker missing"}
        return None
    mp = UsdGeom.Xformable(marker).ComputeLocalToWorldTransform(
        Usd.TimeCode.Default()).ExtractTranslation()
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    root = stage.GetPrimAtPath(PARTS_ROOT)
    if not root.IsValid():
        _snap_debug[pid] = {"reason": "no PARTS_ROOT"}
        return None
    found_any = False
    for child in root.GetChildren():
        attr = child.GetAttribute("piece_id")
        if not attr or attr.Get() != pid:
            continue
        welded_attr = child.GetAttribute("welded")
        if welded_attr and welded_attr.Get() is True:
            continue                       # already assembled — never re-offer
        found_any = True
        path = child.GetPath().pathString
        rb_en = UsdPhysics.RigidBodyAPI(child).GetRigidBodyEnabledAttr()
        held = bool(rb_en and rb_en.Get() is False)
        rng = cache.ComputeWorldBound(child).ComputeAlignedRange()
        if rng.IsEmpty():
            _snap_debug[pid] = {"path": path, "reason": "empty bbox", "held": held}
            continue
        mn, mx = rng.GetMin(), rng.GetMax()
        cx, cy = (mn[0] + mx[0]) / 2, (mn[1] + mx[1]) / 2
        dist = math.hypot(cx - mp[0], cy - mp[1])
        settled = is_settled(path)
        _snap_debug[pid] = {"path": path, "dist_m": round(dist, 3),
                            "held": held, "settled": settled,
                            "settle_frames": _settle_state.get(path, 0)}
        if held or dist > PLACE_RADIUS:
            continue                       # held, or too far — keep scanning
        if not settled:
            return None                    # present, in range, still moving

        return path
    if not found_any:
        _snap_debug[pid] = {"reason": f"no piece_id=={pid} child under {PARTS_ROOT}"}
    return None

def request_snap_check():
    """Start (or leave running) the watcher that fires the snap the
    moment both halves are present and settled."""
    global _snap_task
    if _snap_task is None or _snap_task.done():
        _snap_task = asyncio.ensure_future(_snap_watch())

async def _snap_watch():
    app = omni.kit.app.get_app()
    try:
        _panel.set_assembly_status(servo="WAITING", l_present=False, r_present=False)
        for frame_i in range(SNAP_TIMEOUT_F):
            lp = find_settled_on_table("L")
            rp = find_settled_on_table("R")
            _panel.set_assembly_status(l_present=lp is not None, r_present=rp is not None)
            if lp and rp:
                await do_snap(lp, rp)
                return
            if frame_i % 60 == 0 and frame_i > 0:   # heartbeat every ~1s
                _panel.log(f"Snap watching: L={_snap_debug['L']} "
                          f"R={_snap_debug['R']}")
            await app.next_update_async()
        _panel.log("Snap: timed out waiting for both halves to settle on the table")
        _panel.set_assembly_status(servo="INACTIVE")
        PIPE_COUNTERS["failed_snaps"] += 1
        _panel.set_counters(failed_snaps=PIPE_COUNTERS["failed_snaps"])
        _panel.set_pipeline_state("WAIT_FOR_PARTS")
    except Exception as e:
        # Catches EVERYTHING in the watcher, not just do_snap — a prior
        # version only wrapped the do_snap() call, so an exception
        # anywhere in the polling loop itself (e.g. inside
        # find_settled_on_table) still died silently with no timeout
        # message and nothing in the panel log. Now nothing can vanish.
        import traceback
        _panel.log(f"Snap watcher FAILED: {e}")
        print("Snap watcher traceback:")
        traceback.print_exc()
        _panel.set_assembly_status(servo="INACTIVE")
        PIPE_COUNTERS["failed_snaps"] += 1
        _panel.set_counters(failed_snaps=PIPE_COUNTERS["failed_snaps"])
        _panel.set_pipeline_state("WAIT_FOR_PARTS")

async def do_snap(l_path, r_path):
    global _weld_count
    _panel.log(f"Snap: engaging (anchor {l_path}, moving {r_path})")
    lprim = stage.GetPrimAtPath(l_path)
    rprim = stage.GetPrimAtPath(r_path)
    if not lprim.IsValid() or not rprim.IsValid():
        _panel.log("Snap: a half vanished, aborting")
        _panel.set_assembly_status(servo="INACTIVE")
        return False

    # freeze both: anchor must not drift, mover must not fight physics
    for p in (lprim, rprim):
        rb = UsdPhysics.RigidBodyAPI(p)
        rb.CreateRigidBodyEnabledAttr(False)
        rb.CreateVelocityAttr(Gf.Vec3f(0.0))

    target = Gf.Transform(UsdGeom.Xformable(lprim).ComputeLocalToWorldTransform(
        Usd.TimeCode.Default()))
    start = Gf.Transform(UsdGeom.Xformable(rprim).ComputeLocalToWorldTransform(
        Usd.TimeCode.Default()))
    p0, p1 = start.GetTranslation(), target.GetTranslation()
    rot0, rot1 = start.GetRotation(), target.GetRotation()
    ang0 = (rot1 * rot0.GetInverse()).GetAngle()
    # Gf.Slerp in this pxr build has no Gf.Rotation overload (confirmed by
    # the ArgumentError's own overload list) — it does accept Gf.Quaternion,
    # so slerp that instead and wrap the result back into Gf.Rotation for
    # SetTransform, which DOES accept Gf.Rotation (that part never errored).
    q0, q1 = rot0.GetQuaternion(), rot1.GetQuaternion()

    rop = _reset_to_transform_op(rprim)

    _panel.set_assembly_status(servo="ALIGNING")
    app = omni.kit.app.get_app()
    for i in range(SNAP_FRAMES + 1):
        if not lprim.IsValid() or not rprim.IsValid():
            raise RuntimeError("a half was removed/invalidated mid-snap "
                              "(stage Reset while snapping?)")
        t = _smooth(i / SNAP_FRAMES)
        pos = p0 + (p1 - p0) * t
        rot = Gf.Rotation(Gf.Slerp(t, q0, q1))
        m = Gf.Matrix4d().SetTransform(rot, pos)
        rop.Set(m)
        _panel.set_assembly_status(pos_err_mm=(p1 - pos).GetLength() * 1000.0,
                                   ang_err_deg=ang0 * (1.0 - t))
        await app.next_update_async()

    _panel.set_assembly_status(servo="CONVERGED", pos_err_mm=0.0, ang_err_deg=0.0)

    # weld: a relationship (joint), not a reparent — safe during simulation
    _weld_count += 1
    joint_path = f"{PARTS_ROOT}/Weld_{_weld_count}"
    joint = UsdPhysics.FixedJoint.Define(stage, joint_path)
    joint.CreateBody0Rel().SetTargets([Sdf.Path(l_path)])
    joint.CreateBody1Rel().SetTargets([Sdf.Path(r_path)])
    joint.CreateLocalPos0Attr(Gf.Vec3f(0, 0, 0))
    joint.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
    joint.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
    joint.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))

    # live again as one welded unit (kept together by the joint, not code)
    for p in (lprim, rprim):
        rb = UsdPhysics.RigidBodyAPI(p)
        rb.CreateRigidBodyEnabledAttr(True)
        rb.CreateVelocityAttr(Gf.Vec3f(0.0))
        # tag so find_settled_on_table never offers this pair again —
        # without this, a completed head left on the table (Deliver is
        # still a stub) gets rediscovered on the NEXT Retrieve cycle and
        # a second joint gets attempted on the same already-welded pair,
        # which is exactly what produced the "PxJoint::setActors: at
        # least one actor must be non-static" error in your last run.
        p.CreateAttribute("welded", Sdf.ValueTypeNames.Bool).Set(True)

    _panel.set_assembly_status(welded=True)
    _panel.log(f"Snap: welded {l_path} + {r_path} ({joint_path})")
    _panel.set_pipeline_state("WELDED")
    PIPE_COUNTERS["completed"] += 1
    _panel.set_counters(completed=PIPE_COUNTERS["completed"])
    return True


ARMS = {}
PIPE_COUNTERS = {"completed": 0, "grip_breaks": 0, "failed_snaps": 0}

def init_arms():
    for pid in ("L", "R"):
        ARMS[pid] = ArmController(pid)
        ARMS[pid].calibrate()

async def _retrieve_all():
    """Runs both arms in parallel; returns True if both placed
    successfully and the snap was triggered (not necessarily welded
    yet — caller awaits _snap_task separately to know that)."""
    ready = [a for a in ARMS.values() if a.ok and not a.busy]
    if not ready:
        _panel.log("Retrieve: no arm ready")
        return False
    _panel.set_pipeline_state("PICKING")
    results = await asyncio.gather(*[a.run_retrieve() for a in ready])
    if all(results):
        _panel.set_pipeline_state("SNAPPING")
        _panel.log("both halves placed — watching for settle, snap is automatic")
        request_snap_check()
        return True
    _panel.set_pipeline_state("WAIT_FOR_PARTS")
    return False

def start_retrieve():
    """Button handler — fire-and-forget wrapper around _retrieve_all()."""
    asyncio.ensure_future(_retrieve_all())

def home_arm(pid):
    arm = ARMS.get(pid)
    if arm and arm.ok and not arm.busy:
        asyncio.ensure_future(arm.go_home(
            step_status=lambda s: _panel.set_arm_status(pid, state=s)))
    else:
        _panel.log(f"Home ({pid}): arm busy or not ready")

# =================================================================
# STEP 7 — DELIVER (convey welded head to the deletion pool, delete it)
#          + AUTO-LOOP (the 8-step cycle, wired to Master control's
#          Start/Stop buttons; the existing "auto-loop" checkbox
#          decides single-cycle vs continuous, exactly as labeled)
# =================================================================
DELIVER_TIMEOUT_F = 1800   # ~30 s safety timeout for the conveyor ride
POOL_WAIT_TIMEOUT_F = 900  # ~15 s waiting for produced parts to settle

async def _wait_pool_settled(timeout_f=POOL_WAIT_TIMEOUT_F):
    """Polls both arms' own locate_part() (read-only) until each finds
    a settled part in its own pool. Reuses the exact same FIFO+settle
    logic the real pick uses, so 'settled' means the same thing here
    as it does to the arm that will actually retrieve it."""
    app = omni.kit.app.get_app()
    for _ in range(timeout_f):
        l_ready = ARMS["L"].locate_part() is not None
        r_ready = ARMS["R"].locate_part() is not None
        if l_ready and r_ready:
            return True
        await app.next_update_async()
    return False

def _find_current_welded_pair():
    """The pair tagged welded=True still sitting under PARTS_ROOT.
    Only one should exist at a time if Deliver runs before the next
    Produce (which the auto-loop guarantees, sequentially)."""
    root = stage.GetPrimAtPath(PARTS_ROOT)
    if not root.IsValid():
        return []
    found = []
    for child in root.GetChildren():
        w = child.GetAttribute("welded")
        if w and w.Get() is True:
            found.append(child.GetPath().pathString)
    return found

async def deliver_head():
    """Steps 6-8 of the cycle: start the table+outfeed conveyors, ride
    the welded head to the deletion pool, delete it (and its weld
    joint), stop the conveyors. Returns True on success."""
    pair = _find_current_welded_pair()
    if not pair:
        _panel.log("Deliver: no welded head found on the table")
        return False

    _panel.set_pipeline_state("CONVEYING")
    _panel.set_outfeed_status(head_on_belt=True, in_trigger=False)
    _panel.conveyor.start("table", 0.4)
    _panel.conveyor.start("outfeed", 0.4)
    _panel.log(f"Deliver: conveying {pair} toward the deletion pool")

    app = omni.kit.app.get_app()
    ref_path = pair[0]
    arrived = False
    for _ in range(DELIVER_TIMEOUT_F):
        prim = stage.GetPrimAtPath(ref_path)
        if not prim.IsValid():
            break
        x = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(
            Usd.TimeCode.Default()).ExtractTranslation()[0]
        _panel.set_outfeed_status(in_trigger=(x >= DPOOL_CENTER_X - DPOOL_HALF_W))
        if x >= DPOOL_CENTER_X:
            arrived = True
            break
        await app.next_update_async()

    _panel.conveyor.stop("table")
    _panel.conveyor.stop("outfeed")

    if not arrived:
        _panel.log("Deliver: timed out before reaching the deletion pool")
        _panel.set_outfeed_status(head_on_belt=False, in_trigger=False)
        _panel.set_pipeline_state("WAIT_FOR_PARTS")
        return False

    _panel.set_pipeline_state("DELETING")
    root = stage.GetPrimAtPath(PARTS_ROOT)
    for child in list(root.GetChildren()) if root.IsValid() else []:
        name = child.GetName()
        if name.startswith("Weld_"):
            rel = child.GetRelationship("physics:body0")
            targets = [str(t) for t in rel.GetTargets()] if rel else []
            if any(t in pair for t in targets):
                stage.RemovePrim(child.GetPath())
    for p in pair:
        if stage.GetPrimAtPath(p).IsValid():
            stage.RemovePrim(p)

    _panel.set_outfeed_status(head_on_belt=False, in_trigger=False)
    _panel.log(f"Deliver: deleted {pair}")
    _panel.set_pipeline_state("IDLE")
    return True

_auto_loop_task = None
_auto_loop_running = False

async def _auto_loop_body():
    global _auto_loop_running
    _auto_loop_running = True
    cycle = 0
    try:
        while _auto_loop_running:
            cycle += 1
            _panel.log(f"===== Auto-loop cycle {cycle} =====")

            # 1. produce material once
            _panel.set_pipeline_state("PRODUCING")
            produce_pair()

            # 2. start the conveyor belts
            _panel.conveyor.start_all(0.4)
            _panel.set_pipeline_state("WAIT_FOR_PARTS")

            # 3. wait until settled in the pool, then stop the belts
            settled = await _wait_pool_settled()
            _panel.conveyor.stop_all()
            if not settled:
                _panel.log("Auto-loop: parts never settled in the pool "
                          "— stopping loop")
                break

            # 4. start retrieve
            picked = await _retrieve_all()
            if not picked:
                _panel.log("Auto-loop: retrieve failed — stopping loop")
                break

            # 5. wait for the components to assemble completely
            if _snap_task is not None:
                await _snap_task
            welded_ok = (_panel._widgets["pipeline_state"].text == "WELDED")
            if not welded_ok:
                _panel.log("Auto-loop: snap did not complete — stopping loop")
                break

            # 6+7+8. start conveyor, deliver to deletion pool, stop conveyor
            delivered = await deliver_head()
            if not delivered:
                _panel.log("Auto-loop: deliver failed — stopping loop")
                break

            if not _panel._widgets["auto_loop"].model.get_value_as_bool():
                _panel.log("Auto-loop: single-cycle mode, one head done")
                break
    finally:
        _auto_loop_running = False
        _panel.log("Auto-loop stopped")

def start_auto_loop():
    global _auto_loop_task
    if _auto_loop_task is not None and not _auto_loop_task.done():
        _panel.log("Auto-loop already running")
        return
    _auto_loop_task = asyncio.ensure_future(_auto_loop_body())

def stop_auto_loop():
    global _auto_loop_running
    if not _auto_loop_running:
        _panel.log("Auto-loop is not running")
        return
    _auto_loop_running = False
    _panel.log("Auto-loop: stop requested — finishing the current step, "
              "will not start a new cycle")


COL_IDLE    = 0xFF888888
COL_ACTIVE  = 0xFF00A5FF
COL_OK      = 0xFF4CC44C
COL_FAULT   = 0xFF4444DD
COL_TEXT    = 0xFFCCCCCC

PIPELINE_STATES = ["IDLE", "PRODUCING", "WAIT_FOR_PARTS", "PICKING",
                   "PLACING", "SNAPPING", "WELDED", "CONVEYING",
                   "DELETING", "FAULT"]


class StateMachinePanel:
    """The whole UI window. Design rule: the panel NEVER contains
    logic — every status display has a set_*() method below that the
    coordinator functions call to update it, and every button's
    clicked_fn calls a real function defined earlier in the file (or
    self._stub() if that feature isn't wired up yet). _build() is
    pure layout; look there for what each button/label is."""
    def __init__(self):
        self._log_lines = []
        self._widgets = {}
        self.conveyor = ConveyorController(log_fn=self.log)
        self._build()

    def _stub(self, name):
        """Placeholder for any button not yet wired to real logic —
        just logs the press so you can see it registered."""
        self.log(f"[STUB] button pressed: {name}")

    # ---- event log ------------------------------------------------
    def log(self, msg):
        stamp = time.strftime("%H:%M:%S")
        line = f"[{stamp}] {msg}"
        self._log_lines.append(line)
        self._log_lines = self._log_lines[-200:]
        if "log_stack" in self._widgets:
            self._rebuild_log()
        print(line)

    def _rebuild_log(self):
        stack = self._widgets["log_stack"]
        stack.clear()
        with stack:
            with ui.VStack(spacing=1):
                for line in reversed(self._log_lines[-40:]):
                    ui.Label(line, height=14,
                             style={"font_size": 11, "color": COL_TEXT})

    def _clear_log(self):
        self._log_lines = []
        self._rebuild_log()

    # ---- real handlers --------------------------------------------
    def _outfeed_speed(self):
        return self._widgets["of_speed"].model.get_value_as_float()

    def _on_outfeed_start(self):
        if self.conveyor.start("outfeed", self._outfeed_speed()):
            self.set_outfeed_status(running=True)
        self._sync_all_belts_button()

    def _on_outfeed_stop(self):
        if self.conveyor.stop("outfeed"):
            self.set_outfeed_status(running=False)
        self._sync_all_belts_button()

    def _on_line_belt_toggle(self, pid):
        name = f"in{pid}"
        if self.conveyor.is_running(name):
            self.conveyor.stop(name)
            self._widgets[f"line{pid}_belt_btn"].text = "Belt: OFF"
        else:
            self.conveyor.start(name, self._outfeed_speed())
            self._widgets[f"line{pid}_belt_btn"].text = "Belt: ON"
        self._sync_all_belts_button()

    def _on_all_belts_toggle(self):
        """Single button: start or stop ALL three conveyors together."""
        if self.conveyor.any_running():
            self.conveyor.stop_all()
        else:
            self.conveyor.start_all(self._outfeed_speed())
        self._sync_belt_widgets()

    def _sync_belt_widgets(self):
        """Make every belt-related widget reflect actual conveyor state."""
        for pid in ("L", "R"):
            on = self.conveyor.is_running(f"in{pid}")
            self._widgets[f"line{pid}_belt_btn"].text = \
                f"Belt: {'ON' if on else 'OFF'}"
        self.set_outfeed_status(running=self.conveyor.is_running("outfeed"))
        self._sync_all_belts_button()

    def _sync_all_belts_button(self):
        btn = self._widgets.get("all_belts_btn")
        if btn:
            btn.text = ("Stop ALL belts" if self.conveyor.any_running()
                        else "Start ALL belts")

    def _on_produce_part(self):
        self.set_pipeline_state("PRODUCING")
        produce_pair()
        self.set_pipeline_state("WAIT_FOR_PARTS")

    # ---- setters ----------------------------------------------------
    def set_pipeline_state(self, state, fault=False):
        w = self._widgets["pipeline_state"]
        w.text = state
        w.style = {"font_size": 22,
                   "color": COL_FAULT if fault else
                            (COL_IDLE if state == "IDLE" else COL_ACTIVE)}

    def set_line_status(self, pid, in_pool=None, produced=None):
        if in_pool is not None:
            self._widgets[f"line{pid}_pool"].text = f"parts in pool: {in_pool}"
        if produced is not None:
            self._widgets[f"line{pid}_total"].text = f"produced: {produced}"

    def set_arm_status(self, pid, state=None, grip=None, retries=None):
        if state is not None:
            w = self._widgets[f"arm{pid}_state"]
            w.text = state
            w.style = {"font_size": 14,
                       "color": COL_IDLE if state in ("WAITING", "HOMING")
                                else COL_ACTIVE}
        if grip is not None:
            w = self._widgets[f"arm{pid}_grip"]
            w.text = f"grip: {grip}"
            w.style = {"font_size": 13,
                       "color": COL_FAULT if grip == "GRIP-BREAK" else
                                (COL_OK if grip == "HOLDING" else COL_TEXT)}
        if retries is not None:
            self._widgets[f"arm{pid}_retry"].text = f"retries: {retries}"

    def set_assembly_status(self, l_present=None, r_present=None,
                            servo=None, pos_err_mm=None, ang_err_deg=None,
                            welded=None):
        if l_present is not None:
            self._widgets["asm_l"].text = f"half L: {'PRESENT' if l_present else 'none'}"
        if r_present is not None:
            self._widgets["asm_r"].text = f"half R: {'PRESENT' if r_present else 'none'}"
        if servo is not None:
            self._widgets["asm_servo"].text = f"servo: {servo}"
        if pos_err_mm is not None and ang_err_deg is not None:
            self._widgets["asm_err"].text = \
                f"pose error: {pos_err_mm:.1f} mm / {ang_err_deg:.1f} deg"
        if welded is not None:
            w = self._widgets["asm_weld"]
            w.text = f"weld: {'CREATED' if welded else 'none'}"
            w.style = {"font_size": 14, "color": COL_OK if welded else COL_TEXT}

    def set_outfeed_status(self, running=None, head_on_belt=None, in_trigger=None):
        if running is not None:
            w = self._widgets["of_run"]
            w.text = f"conveyor: {'RUNNING' if running else 'STOPPED'}"
            w.style = {"font_size": 13, "color": COL_OK if running else COL_TEXT}
        if head_on_belt is not None:
            self._widgets["of_head"].text = f"head on belt: {'YES' if head_on_belt else 'none'}"
        if in_trigger is not None:
            self._widgets["of_trig"].text = f"in trigger zone: {'YES' if in_trigger else 'none'}"

    def set_counters(self, completed=None, last_cycle_s=None,
                     grip_breaks=None, failed_snaps=None):
        if completed is not None:
            self._widgets["cnt_done"].text = f"heads completed: {completed}"
        if last_cycle_s is not None:
            self._widgets["cnt_cycle"].text = f"last cycle: {last_cycle_s:.1f} s"
        if grip_breaks is not None:
            self._widgets["cnt_break"].text = f"grip-breaks: {grip_breaks}"
        if failed_snaps is not None:
            self._widgets["cnt_snapfail"].text = f"failed snaps: {failed_snaps}"

    # ---- build -------------------------------------------------------
    def _build(self):
        """Lays out the whole window. The CollapsableFrame titles below
        (Master control, Production lines, Arms, Assembly station,
        Out-feed, Counters, Event log, Debug) are the panel's actual
        visible sections — read top to bottom here in the same order
        you see them in the UI."""
        self.window = ui.Window("Assembly State Machine", width=430, height=880)
        with self.window.frame:
            with ui.ScrollingFrame(
                    horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_OFF):
                with ui.VStack(spacing=6):

                    with ui.CollapsableFrame("Master control", collapsed=False):
                        with ui.VStack(spacing=6):
                            lbl = ui.Label("IDLE", height=30,
                                           alignment=ui.Alignment.CENTER,
                                           style={"font_size": 22, "color": COL_IDLE})
                            self._widgets["pipeline_state"] = lbl

                            ui.Label("Cycle stages", style={"font_size": 12})
                            with ui.HStack(spacing=4, height=40):
                                ui.Button("Produce Part",
                                          clicked_fn=self._on_produce_part)
                                ui.Button("Retrieve",
                                          clicked_fn=lambda: start_retrieve())
                                ui.Button("Deliver",
                                          clicked_fn=lambda: self._stub("Deliver"))

                            ui.Label("Conveyors", style={"font_size": 12})
                            btn = ui.Button("Start ALL belts", height=32,
                                            clicked_fn=self._on_all_belts_toggle)
                            self._widgets["all_belts_btn"] = btn

                            ui.Label("Run control", style={"font_size": 12})
                            with ui.HStack(spacing=4, height=28):
                                ui.Button("Start", clicked_fn=lambda: start_auto_loop())
                                ui.Button("Pause", clicked_fn=lambda: self._stub("Pause"))
                                ui.Button("Stop", clicked_fn=lambda: stop_auto_loop())
                                ui.Button("Reset", clicked_fn=lambda: self._stub("Reset"))

                            with ui.HStack(height=20):
                                cb = ui.CheckBox(width=24)
                                self._widgets["auto_loop"] = cb
                                ui.Label("auto-loop (off = single cycle)",
                                         style={"font_size": 12})

                    with ui.CollapsableFrame("Production lines", collapsed=False):
                        with ui.VStack(spacing=4):
                            for pid in ("L", "R"):
                                with ui.HStack(spacing=6, height=24):
                                    ui.Label(f"Line {pid}", width=50,
                                             style={"font_size": 14})
                                    self._widgets[f"line{pid}_pool"] = ui.Label(
                                        "parts in pool: 0", width=110,
                                        style={"font_size": 12})
                                    self._widgets[f"line{pid}_total"] = ui.Label(
                                        "produced: 0", width=90,
                                        style={"font_size": 12})
                                    ui.Button("Spawn one", width=80,
                                              clicked_fn=lambda p=pid: produce_half(p))
                                    btn = ui.Button(
                                        "Belt: OFF", width=80,
                                        clicked_fn=lambda p=pid:
                                        self._on_line_belt_toggle(p))
                                    self._widgets[f"line{pid}_belt_btn"] = btn
                            ui.Button("Clear all parts", height=22,
                                      clicked_fn=lambda: clear_parts())

                    with ui.CollapsableFrame("Arms", collapsed=False):
                        with ui.VStack(spacing=4):
                            for pid in ("L", "R"):
                                with ui.HStack(spacing=6, height=24):
                                    ui.Label(f"Arm {pid}", width=50,
                                             style={"font_size": 14})
                                    self._widgets[f"arm{pid}_state"] = ui.Label(
                                        "WAITING", width=100,
                                        style={"font_size": 14, "color": COL_IDLE})
                                    self._widgets[f"arm{pid}_grip"] = ui.Label(
                                        "grip: OPEN", width=100,
                                        style={"font_size": 13, "color": COL_TEXT})
                                    self._widgets[f"arm{pid}_retry"] = ui.Label(
                                        "retries: 0", width=70,
                                        style={"font_size": 12})
                                with ui.HStack(spacing=4, height=22):
                                    ui.Spacer(width=50)
                                    ui.Button("Home", width=70,
                                              clicked_fn=lambda p=pid:
                                              home_arm(p))
                                    ui.Button("Force release", width=100,
                                              clicked_fn=lambda p=pid:
                                              self._stub(f"Force release ({p})"))
                                    ui.Button("Retry", width=70,
                                              clicked_fn=lambda p=pid:
                                              self._stub(f"Retry ({p})"))

                    with ui.CollapsableFrame("Assembly station", collapsed=False):
                        with ui.VStack(spacing=4):
                            with ui.HStack(spacing=10, height=20):
                                self._widgets["asm_l"] = ui.Label(
                                    "half L: none", width=110,
                                    style={"font_size": 13})
                                self._widgets["asm_r"] = ui.Label(
                                    "half R: none", width=110,
                                    style={"font_size": 13})
                            self._widgets["asm_servo"] = ui.Label(
                                "servo: INACTIVE", height=18,
                                style={"font_size": 13})
                            self._widgets["asm_err"] = ui.Label(
                                "pose error: -- mm / -- deg", height=18,
                                style={"font_size": 14, "color": COL_ACTIVE})
                            self._widgets["asm_weld"] = ui.Label(
                                "weld: none", height=18,
                                style={"font_size": 14, "color": COL_TEXT})
                            with ui.HStack(spacing=4, height=24):
                                ui.Button("Force weld",
                                          clicked_fn=lambda: self._stub("Force weld"))
                                ui.Button("Break weld",
                                          clicked_fn=lambda: self._stub("Break weld"))

                    with ui.CollapsableFrame("Out-feed", collapsed=False):
                        with ui.VStack(spacing=4):
                            with ui.HStack(spacing=10, height=20):
                                self._widgets["of_run"] = ui.Label(
                                    "conveyor: STOPPED", width=150,
                                    style={"font_size": 13})
                                self._widgets["of_head"] = ui.Label(
                                    "head on belt: none", width=130,
                                    style={"font_size": 13})
                            self._widgets["of_trig"] = ui.Label(
                                "in trigger zone: none", height=18,
                                style={"font_size": 13})
                            with ui.HStack(spacing=6, height=22):
                                ui.Label("belt speed (m/s):", width=110,
                                         style={"font_size": 12})
                                spd = ui.FloatField(width=60)
                                spd.model.set_value(0.4)
                                self._widgets["of_speed"] = spd
                            with ui.HStack(spacing=4, height=26):
                                ui.Button("Start conveyor",
                                          clicked_fn=self._on_outfeed_start)
                                ui.Button("Stop conveyor",
                                          clicked_fn=self._on_outfeed_stop)
                                ui.Button("Force delete",
                                          clicked_fn=lambda: self._stub("Force delete"))

                    with ui.CollapsableFrame("Counters", collapsed=False):
                        with ui.VStack(spacing=2):
                            with ui.HStack(spacing=10, height=18):
                                self._widgets["cnt_done"] = ui.Label(
                                    "heads completed: 0", width=150,
                                    style={"font_size": 12})
                                self._widgets["cnt_cycle"] = ui.Label(
                                    "last cycle: -- s", width=120,
                                    style={"font_size": 12})
                            with ui.HStack(spacing=10, height=18):
                                self._widgets["cnt_break"] = ui.Label(
                                    "grip-breaks: 0", width=150,
                                    style={"font_size": 12})
                                self._widgets["cnt_snapfail"] = ui.Label(
                                    "failed snaps: 0", width=120,
                                    style={"font_size": 12})

                    with ui.CollapsableFrame("Event log", collapsed=False):
                        with ui.VStack(spacing=2):
                            log_frame = ui.ScrollingFrame(
                                height=130,
                                horizontal_scrollbar_policy=
                                ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_OFF)
                            self._widgets["log_stack"] = log_frame
                            ui.Button("Clear log", height=20,
                                      clicked_fn=self._clear_log)

                    with ui.CollapsableFrame("Debug / injection", collapsed=True):
                        with ui.VStack(spacing=4):
                            with ui.HStack(spacing=4, height=26):
                                ui.Button("Spawn pair",
                                          clicked_fn=lambda: produce_pair())
                                ui.Button("Teleport-place both halves",
                                          clicked_fn=lambda:
                                          self._stub("Teleport-place both halves"))
                            with ui.HStack(spacing=4, height=24):
                                ui.Label("skip to:", width=55,
                                         style={"font_size": 12})
                                combo = ui.ComboBox(0, *PIPELINE_STATES)
                                self._widgets["skip_combo"] = combo
                                ui.Button("Go", width=40,
                                          clicked_fn=self._skip_stub)

        self._rebuild_log()

    def _skip_stub(self):
        idx = self._widgets["skip_combo"].model.get_item_value_model().get_value_as_int()
        self._stub(f"Skip to state: {PIPELINE_STATES[idx]}")


# =================================================================
# RUN ALL STEPS
# =================================================================
try:
    _panel.conveyor.stop_all()
    _panel.window.visible = False
except NameError:
    pass

build_layout()                       # step 1
add_physics()                        # step 2

# reach + clearance report per arm (KR 10 R1440 ~1.44 m reach)
print("-" * 64)
_pool_x_near = {"L": CPOOL_CENTER_X + (CPOOL_HALF_W + CPOOL_WALL_T),
                "R": CPOOL_CENTER_X + (CPOOL_HALF_W + CPOOL_WALL_T)}
for pid, sign in (("L", +1.0), ("R", -1.0)):
    ax, ay = ARM_POSITIONS[pid]
    print(f"arm {pid} base: ({ax:+.3f}, {ay:+.3f})")
    clearance = ax - _pool_x_near[pid]
    cflag = "  <-- may clip pool wall!" if clearance < 0.3 else ""
    print(f"  X-clearance to own pool's outer wall: {clearance:.2f} m{cflag}")
    for name, tx, ty in (
        ("own collection pool",   CPOOL_CENTER_X, sign * LINE_Y),
        ("own place target",      0.0,            sign * PLACE_Y_OFF),
        ("partner place target",  0.0,           -sign * PLACE_Y_OFF),
    ):
        d = math.hypot(tx - ax, ty - ay)
        flag = "  <-- OVER 1.44 m REACH!" if d > 1.44 else ""
        print(f"  -> {name:22s} {d:.2f} m{flag}")
print("-" * 64)
_panel = StateMachinePanel()         # step 4 (panel; step-3 functions above)
start_settle_tracking()              # step 5a (part-settling gate)
init_arms()                          # step 5b (calibrate FK from the stage)
_panel.log("factory ready: layout + physics + production + conveyors")
_panel.log("press PLAY, then Produce Part / Belt: ON to test the lines")

print("[3/4] production functions ready (wired to panel buttons)")
print("[4/4] state machine panel ready")
print("=" * 64)
print("ONE-SHOT SETUP COMPLETE")
print("  wired for real: Produce Part, Spawn one, Spawn pair, Clear all")
print("                  parts, Belt toggles, Start/Stop conveyor")
print("  wired for real: ...also Deliver, delete, Start/Stop (auto-loop)")
print("  still stubs:    Pause, Reset, Force weld/release/Retry")
print("=" * 64)
