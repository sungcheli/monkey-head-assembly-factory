# factory_master.py — Structure Guide

A map of the script (1851 lines), not a copy of it. Use this to find the
right place to look/edit; the real comments and logic live in the file
itself. Search for the `# STEP N` banners to jump between sections.

## The big picture

One Script Editor run builds the entire scene and wires the whole UI
panel. Nothing is manual after that — you press buttons, everything
else (physics, IK, settling, welding, delivery) runs itself.

```
Produce Part → belts carry halves into pools → arms Retrieve
   → automatic snap+weld → Deliver → deleted → loop
```

The **Start** button runs that whole loop for you (STEP 7). Every other
button lets you run one piece by hand for testing.

---

## STEP 1 — Factory Layout (`build_layout()`, ~line 173)

Builds every static/kinematic prim in the scene: ground, two production
lines (machine + belt + collection pool + pick marker), two KUKA arm
references, the assembly table + guardrails, the out-feed belt +
guardrails, the deletion pool. Pure geometry — no physics yet.

| Function | Purpose |
|---|---|
| `make_xform/make_box/make_marker` | thin wrappers for creating Xform/Cube/Sphere prims with a transform |
| `make_pool(...)` | floor + 4 walls, reused for both collection pools and the deletion pool |
| `clean(path)` | deletes a prim if it exists — every group is rebuilt fresh each run |
| `disable_inherited_collision(root_prim)` | strips any collision the KUKA source asset shipped with (a swinging forearm was knocking placed parts around before this) |
| `build_layout()` | calls all of the above in order; this is what actually runs |

**Key constants to tune:** `ARM_POSITIONS` (per-arm base X/Y — no longer
a strict mirror pair, tuned independently), `ARM_SCALE`, `TABLE_SIZE`,
`TABLE_POS`, `PLACE_Y_OFF` (each half's drop point, own side of table
center), `CPOOL_*` / `DPOOL_*` (pool geometry).

---

## STEP 2 — Physics (`add_physics()`, ~line 357)

Applies materials and colliders to everything STEP 1 built. Nothing
here creates geometry — it only makes existing prims physical.

| Function | Purpose |
|---|---|
| `make_physics_material(...)` | defines a `UsdPhysics.Material` + PhysX combine modes |
| `add_static_collider(path, mat)` | collision, never moves (walls, table, rails) |
| `add_kinematic_body(path, mat)` | collision + kinematic rigid body — **this is what makes a prim conveyor-capable** later |
| `add_physics()` | creates the PhysicsScene, 3 materials (Belt/Pool/Table), applies colliders to everything |

Belts (in-feed ×2, out-feed, **and now the table itself**) are all
`add_kinematic_body` — that's the prerequisite for STEP 4's conveyor
control.

---

## STEP 3 — Half Production (~line 432)

The spawner. No UI of its own anymore — the panel's Production Lines
section calls these directly.

| Function | Purpose |
|---|---|
| `produce_half(pid, slide_speed=None)` | spawns one `monkey_half_L/R.usd` reference at its line's belt, tags `piece_id`/`head_id`, sets convex-decomposition collision |
| `produce_pair()` | one call to `produce_half` for each side |
| `clear_parts()` | deletes everything under `/World/Parts`, resets counters |
| `get_spawn_point(pid)` | reads the belt's actual world position — spawn follows the belt if you drag `/World/LineL` etc. |

## STEP 3b — Settling Gate (~line 520)

**One global per-frame subscription**, not one per part. Tracks every
part's world position frame-to-frame; a part counts as `is_settled()`
once it's moved less than `SETTLE_MOVE_TOL` for `SETTLE_FRAMES` in a
row. This is the gate every arm action checks before touching anything
— deliberately position-based, not velocity-based (the velocity
attribute proved unreliable — it never stopped reporting "moving" even
on visibly still parts).

| Function | Purpose |
|---|---|
| `_settle_tick(e)` | the per-frame callback |
| `start_settle_tracking()` | subscribes it once |
| `is_settled(path)` | the check everything else calls |

---

## STEP 4 — Conveyor Control + the UI Panel (~line 577)

### `ConveyorController` class
Drives belts via PhysX surface velocity (the geometry never moves; the
contact surface does). `BELTS` dict has 4 entries: `outfeed`, `inL`,
`inR`, `table` — adding an entry here is all it takes to include a
belt in "Start/Stop ALL belts".

| Method | Purpose |
|---|---|
| `start(name, speed)` / `stop(name)` | one belt |
| `start_all()` / `stop_all()` | every belt in `BELTS` |
| `is_running(name)` / `any_running()` | state queries the UI reads |

### `StateMachinePanel` class (~line 1462, built AFTER the arm/snap/deliver
functions so its button closures can reference them)
The whole UI. Every **status display** has a `set_*` method the
coordinator calls (`set_pipeline_state`, `set_arm_status`,
`set_assembly_status`, `set_outfeed_status`, `set_counters`, `log`) —
the panel never contains logic itself, only these setters and the
button click handlers.

**Button → function map** (see the table at the very end of this guide).

---

## STEP 5 — Arm Motion (~line 647)

The core robotics. One arm = one `ArmController` instance.

**The big idea:** rather than hand-deriving link lengths, each arm
**calibrates its own forward-kinematics model from the stage** at
startup (reads the actual joint transforms), then solves inverse
kinematics by brute-force grid search over that model. No hardcoded
geometry — whatever is actually in the USD is what gets driven.

| Method | Purpose |
|---|---|
| `calibrate()` | reads joint init matrices, builds the FK model |
| `fk(a1,a2,a3)` | pure math — given 3 joint angles, where's the tool tip |
| `solve_ik(target)` | grid search: which a1/a2/a3 get closest to a world point |
| `safe_transit_pose()` | solves IK for a fixed high point — used before every A1 sweep so the arm clears the table/rails instead of sweeping at an unverified height |
| `move_joint(key, target, ..., side_guard=None)` | eases ONE joint from current angle to target (smoothstep). `side_guard` optionally logs a warning if the tool crosses into the other arm's half of the table mid-sweep |
| `locate_part()` | FIFO by trailing number in the part's name, gated on `is_settled()` — never skips ahead to a later-numbered part |
| `attach(part_path)` / `_update_held_pose()` / `release()` | suction: freezes the part's physics and makes it follow the tip via a plain per-frame attribute Set — **deliberately not a reparent** (an earlier version used `MovePrim` to carry the part, which crashed PhysX mid-simulation) |
| `run_retrieve()` | the full one-arm cycle: LOCATING → APPROACHING → GRIPPING → LIFTING → TRANSITING → PLACING → RELEASING → home. One joint moves at a time throughout. |
| `go_home()` | retreat to a safe pose, then yaw back to 0 |

`ARMS = {"L": ArmController, "R": ArmController}`, `init_arms()`
creates and calibrates both.

---

## STEP 6 — Automatic Snap & Weld (~line 1048)

No button — triggered the instant both arms finish placing.

**The trick:** both halves were exported from Blender sharing one
origin (the assembled head's center), so "L's transform == R's
transform" *is* correct assembly, with no separate offset to compute.

| Function | Purpose |
|---|---|
| `find_settled_on_table(pid)` | like `locate_part` but scoped to the `PlaceTarget_{pid}` marker; skips anything tagged `welded=True` (prevents re-welding an already-completed head still sitting on the table) |
| `request_snap_check()` | starts the watcher coroutine if not already running |
| `_snap_watch()` | polls every frame until both halves are present+settled, then calls `do_snap` |
| `do_snap(l_path, r_path)` | freezes both, **anchors on L**, kinematically lerps/slerps R onto L's exact pose, then creates a real `UsdPhysics.FixedJoint` once converged, tags both `welded=True` |

---

## STEP 7 — Deliver + Auto-Loop (~line 1286)

The piece that closes the whole cycle.

| Function | Purpose |
|---|---|
| `_retrieve_all()` | awaitable core of Retrieve — both the button and the auto-loop call this |
| `_wait_pool_settled()` | polls both arms' own `locate_part()` (read-only) until each has a settled candidate |
| `_find_current_welded_pair()` | finds the prim(s) tagged `welded=True` still on the table |
| `deliver_head()` | starts table+outfeed conveyors, tracks the head's X position until it reaches the deletion pool, deletes both halves + their weld joint, stops the conveyors |
| `_auto_loop_body()` | the 8-step cycle: produce → belts on → wait settle → belts off → retrieve → wait weld → deliver → (repeat if auto-loop checkbox is on, else stop) |
| `start_auto_loop()` / `stop_auto_loop()` | wired to the panel's Start/Stop buttons |

---

## Button → function map (Master control and below)

| Button | Calls | Real or stub? |
|---|---|---|
| Produce Part | `_on_produce_part` → `produce_pair()` | real |
| Retrieve | `start_retrieve()` → `_retrieve_all()` | real |
| Deliver | *(not a separate button — happens automatically inside the auto-loop / can be tested by calling `deliver_head()` directly)* | real, no dedicated button |
| Start | `start_auto_loop()` | real |
| Stop | `stop_auto_loop()` | real (graceful — finishes current step) |
| Pause / Reset | `self._stub(...)` | stub |
| Spawn one (per line) | `produce_half(pid)` | real |
| Belt: ON/OFF (per line) | `conveyor.start/stop("inL"/"inR")` | real |
| Clear all parts | `clear_parts()` | real |
| Home (per arm) | `home_arm(pid)` | real |
| Force release / Retry (per arm) | `self._stub(...)` | stub |
| Force weld / Break weld | `self._stub(...)` | stub |
| Start conveyor / Stop conveyor (out-feed) | `conveyor.start/stop("outfeed")` | real |
| Force delete | `self._stub(...)` | stub |
| Start ALL belts / Stop ALL belts | `conveyor.start_all()/stop_all()` | real |
| Spawn pair (debug) | `produce_pair()` | real |
| Teleport-place both halves (debug) | `self._stub(...)` | stub — never needed once real placement worked |
| Skip to state (debug) | `self._stub(...)` | stub |

---

## Constants worth knowing where to find

All near the top of the file (~line 30-170):
`ARM_POSITIONS`, `ARM_SCALE`, `TABLE_SIZE`/`TABLE_POS`, `PLACE_Y_OFF`,
`SETTLE_MOVE_TOL`/`SETTLE_FRAMES`, `PLACE_RADIUS`, `SAFE_TRANSIT_Z`,
`SNAP_FRAMES`, belt/pool geometry (`CPOOL_*`, `DPOOL_*`, `OUTFEED_*`).
