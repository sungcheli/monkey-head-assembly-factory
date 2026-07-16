# Monkey-Head Assembly Factory — Isaac Sim Robotics Pipeline

### 🎥 [Watch the demo video](https://youtube.com/shorts/-B6rWuDNboM?si=azGhsGsKBLWwppdV)

[![Demo video](https://img.youtube.com/vi/-B6rWuDNboM/hqdefault.jpg)](https://youtube.com/shorts/-B6rWuDNboM?si=azGhsGsKBLWwppdV)

A simulated two-arm robotic assembly cell built in NVIDIA Isaac Sim: two KUKA
KR10 arms pick mirror-halved mesh components from production lines, an
automatic pose-servo mechanism assembles them on a table, and a conveyor
delivers the finished part for disposal — all driven from a single Script
Editor file and a live state-machine control panel.

> **Note on this repo:** this is shared for code review, not for running.
> The scripts reference local file paths and a KUKA robot asset that isn't
> included. If you're reviewing this for an interview, `factory_master.py`
> is the file to read — see `docs/STRUCTURE_GUIDE.md` for a fast way to
> navigate it.

## What it does

1. **Production** — two lines spawn mirror-halved mesh components ("L" and
   "R" halves of a cut object) onto conveyor belts, which carry them into
   walled collection pools.
2. **Retrieval** — two robotic arms each locate the next part in their own
   pool (FIFO, gated on the part having come to physical rest), pick it via
   simulated suction, and place it on a shared assembly table.
3. **Assembly** — once both halves are present and settled, an automatic
   routine aligns and welds them into one rigid unit — no button, it just
   happens.
4. **Delivery** — the assembled unit rides a conveyor to a deletion zone
   and is removed, completing one cycle. A one-button auto-loop repeats the
   whole thing indefinitely.

Everything is controlled from one live UI panel: production, conveyors,
per-arm status, assembly progress, and a running event log.

## Why this is more than a scripted animation

The interesting engineering is in what breaks when you try to make a
robotics pipeline *physically real* instead of just visually plausible.
A few of the problems this project actually had to solve:

- **Self-calibrating inverse kinematics.** Rather than hardcoding link
  lengths and joint signs, each arm reads its own joint transforms
  directly from the USD stage at startup and builds a forward-kinematics
  model from what's actually there. Inverse kinematics is then a
  grid-search against that model — so the arm remains correct no matter
  what scale or position correction sits above it in the hierarchy.

- **A crash traced to a specific physics-engine mechanism.** Early suction
  logic reparented the held part into the arm's hierarchy mid-grasp. This
  intermittently crashed PhysX — traced to `MovePrim` forcing a structural
  stage change while the simulation was actively stepping. Fixed by never
  reparenting: the held part stays in place and is driven by a per-frame
  attribute update instead, which is safe during simulation.

- **A "settled" check that lied.** The initial rest-detection logic
  polled `RigidBodyAPI`'s velocity attribute, which reported "still
  moving" indefinitely on parts that were visibly motionless — a PhysX
  velocity-readback artifact. Replaced with direct frame-to-frame
  position-drift tracking, which can't disagree with what's actually
  happening in the viewport.

- **Assembly correctness by construction, not computation.** Both mesh
  halves are exported from Blender sharing a single origin — the
  assembled object's center. That reduces "is this correctly assembled?"
  to "do the two halves' world transforms match?", with no separate
  offset to compute or tune.

- **Concurrency with a hard constraint.** Both arms operate simultaneously,
  but each is only ever allowed to move one joint at a time (a
  requirement from the original spec, not a limitation of the approach) —
  satisfied via `asyncio` tasks per arm, each internally sequential.

## Files

| File | Purpose |
|---|---|
| `factory_master.py` | The full pipeline: scene construction, physics setup, production, arm kinematics, automatic assembly, delivery, and the UI panel — one file, run top to bottom in Isaac Sim's Script Editor |
| `factory_master_zh.py` | Same code, comments translated to Traditional Chinese |
| `cut_monkey_head.py` | Blender-side script that cuts the source mesh into mirrored halves with a poka-yoke alignment key and exports them sharing one origin |
| `docs/STRUCTURE_GUIDE.md` | A map of `factory_master.py` — every section, class, and function with a one-line purpose, plus a button-to-function table. Read this first. |

## Stack

NVIDIA Isaac Sim 4.5 (USD / PhysX), Python (`asyncio` for concurrent arm
control), Blender for asset preparation.
