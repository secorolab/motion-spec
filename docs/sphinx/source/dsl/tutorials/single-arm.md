# Hands-on: `pick_place_single`

This walkthrough follows the maintained
`models/pick_place_single/pick_place_single.robmot` model from authored files to
an observed run. It explains where each decision is made, so the example can be
changed without guessing which generated file to edit.

## 1. Identify the four source files

The example is composed rather than written as one large model:

| File | Responsibility |
|---|---|
| `pick_place_single.robmot` | quantities, guarded motions, controllers, and solvers |
| `pick_place_single.fsm` | task states, events, transitions, and reactions |
| `pick_place_single.scene` | abstract objects, workspace, agent, and scene |
| `pick_place_single.scenex` | kinematic instances, fixed joints, poses, and MJCF mappings |

The motion file imports the FSM and expanded scene:

```robmot
import "pick_place_single.fsm"
import "pick_place_single.scenex"
```

Nothing in the motion model depends on a parsed MuJoCo body name. It refers to
semantic scene entities such as `<kinova.base_link>` and
`<gripper.g_base.g_pinch>`.

## 2. Follow scene composition

The expanded scene instantiates one Kinova tree and one gripper tree:

```text
ktree inst (ns=pps_mjc) kinova of <kinova_tree>
ktree inst (ns=pps_mjc) gripper of <gripper_tree>
```

`kinova_2f85` joins the gripper interface to the robot pinch site. `world_tree`
then attaches the robot and table to the semantic world. Finally,
`pick_place_scene_mjc` maps those semantic instances to MJCF assets and adds the
cube at `(0.50, 0.0, 0.76) m`.

The execution context in the motion file selects that scene instance and fixes
the control period:

```robmot
exec-context (ns=app) pick-place-single-exec {
    runs-scene: <pick_place_scene_mjc>
    platform: simulation { name: "MuJoCo" }
    timestep: 1.0 ms
}
```

## 3. Read the shared world interface

The `shared` context is the interface between runtime observations and motion
logic. It declares:

- the TCP pose and twist relative to the Kinova base;
- the cube pose relative to the same base;
- the elbow pose;
- the gripper driver-joint position;
- common setpoints such as open/closed gripper angles and grasp orientation.

For example, this quantity says exactly what pose is observed and in which frame:

```robmot
pose pose-ee-base {
    of: <gripper.g_base.g_pinch>,
    wrt: <kinova.base_link>,
    as-seen-by: <kinova.base_link>
}
```

Later motions select typed components such as
`<shared.world.pose-ee-base>.position.z`; no string splitting is involved.

## 4. Understand the task sequence

The FSM maps one task state to each guarded motion. Monitors in the motion
handlers emit the events on the arrows:

```{graphviz}
:caption: Normal pick-and-place state sequence.

digraph pick_place {
  rankdir=LR;
  graph [bgcolor="white", nodesep=0.28, ranksep=0.55];
  node [shape=box, style="rounded", fontname="sans-serif"];
  edge [fontname="sans-serif", fontsize=9];

  start [label="START"];
  home [label="HOME"];
  above [label="PICK_ABOVE"];
  pick [label="PICK"];
  grasp [label="GRASP"];
  lift [label="LIFT"];
  place_above [label="PLACE_ABOVE"];
  place [label="PLACE"];
  open [label="OPEN"];
  retreat [label="RETREAT"];
  done [label="DONE"];
  recover [label="RECOVER", color="#c44"];

  start -> home [label="E_STEP"];
  home -> above [label="READY"];
  above -> pick [label="READY"];
  pick -> grasp [label="READY"];
  grasp -> lift [label="READY"];
  lift -> place_above [label="READY"];
  place_above -> place [label="READY"];
  place -> open [label="READY"];
  open -> retreat [label="READY"];
  retreat -> done [label="SETTLED"];
  lift -> recover [label="GRASP_LOST", color="#c44"];
  place_above -> recover [label="GRASP_LOST", color="#c44"];
  place -> recover [label="GRASP_LOST", color="#c44"];
  recover -> recover [label="E_RECOVER"];
}
```

The normal motions are:

| Motion | What it controls |
|---|---|
| `home` | snapshot and hold the startup TCP pose |
| `pick-above` | interpolate from the current pose to a pose above the cube |
| `pick` | lower to grasp height while supporting the elbow |
| `grasp-hold` | hold the TCP and close the gripper |
| `lift` | raise the cube while keeping the gripper closed |
| `place-above` | move over the placement position |
| `place` | lower to the placement height |
| `open-grasp-hold` | hold the TCP and open the gripper |
| `retreat` | move away while keeping the gripper open |
| `recover` | snapshot and hold the current pose after grasp loss |

## 5. Trace one motion end to end

`pick-above` captures the current TCP pose and the cube's initial x/y position,
then constructs a Cartesian goal:

```robmot
pose start-pose = snapshot of <shared.world.pose-ee-base>,
linear-velocity min-approach-speed = 0.005 m/s,
velocity-profile approach-profile = profile {
    max-velocity: 0.08 m/s,
    max-acceleration: 0.05 m/s^2,
    shape: trapezoidal
},
length start-cube-x = snapshot of <shared.world.pose-cube-base>.position.x,
length start-cube-y = snapshot of <shared.world.pose-cube-base>.position.y,
path approach-path = lerp {
    start: <spec.start-pose>,
    goal: <spec.goal-pose>
}
```

The motion gives path following three explicit constraint roles:

```robmot
while {
    follow-tan: moving <shared.world.pose-ee-base>
                along <spec.approach-path> with <spec.approach-profile>,
    follow-lat: keeping <shared.world.pose-ee-base>.position
                on <spec.approach-path>,
    follow-ori: keeping <shared.world.pose-ee-base>.orientation
                on <spec.approach-path>,
    advance: progress of <shared.world.pose-ee-base>
             along <spec.approach-path> more than <spec.min-approach-speed>
}
```

`follow-tan` commands the speed along the path, `follow-lat` and `follow-ori`
keep the TCP on its geometry, and `advance` confirms that measured tangential
progress stays above the minimum. The separate `when` constraint prevents motion
until the gripper is open.

`handler-pick-above` gives those declarative constraints runtime behavior:

- the monitor emits `E_PICK_ABOVE_READY` when `gripper-ready` activates;
- `otherwise hold <home>` supplies the safe motion before it activates;
- three PID controllers regulate the tangential, lateral, and orientation constraints;
- the handler reuses the ACHD arm solver from `handler-home`.

That event drives `T_HOME_PICK_ABOVE` in the FSM. The same chain—constraint,
monitor, event, transition—advances every subsequent stage.

## 6. See why the gripper closes

`grasp-hold` becomes eligible only when the TCP is within `0.015 m` of the cube.
Its `close-gripper` constraint targets `0.8 rad`:

```robmot
close-gripper: keeping <shared.world.gripper-pos>
               equal to <shared.spec.gripper-closed>
```

The matching handler uses a feed-forward controller and a
`command-forwarding` solver:

```robmot
feed-forward ctrl-cg-close-gripper {
    constraint: <grasp-hold.close-gripper>
}

solver gripper-command-solver {
    agent: <pickplace_agents.kinova_2f85>,
    algorithm: command-forwarding
}
```

The arm and gripper therefore use separate solver paths. Later lift/place
handlers reuse both solvers and keep commanding the closed gripper setpoint.

## 7. Generate without running

From the workspace root, stop after IR when inspecting semantic lowering:

```bash
motion-spec gen ir \
  src/motion-spec-dsl/models/pick_place_single/pick_place_single.robmot \
  -o generation/pick-place-ir
```

Open `generation/pick-place-ir/generated/model/ir.json`. Search for
`pick-above`, `ctrl-pa-follow-pos`, `E_PICK_ABOVE_READY`, and
`gripper-command-solver` to verify that the motion, handler, event, and solver
all reached IR.

Generate C++ when the code-generation toolchain is available:

```bash
motion-spec gen \
  src/motion-spec-dsl/models/pick_place_single/pick_place_single.robmot \
  -o generation/pick-place-generated
```

The authored snapshot is under `generated/source/`, RDF and IR under
`generated/model/`, and generated C++ under `generated/controller/`.

## 8. Run and inspect behavior

```bash
motion-spec run \
  src/motion-spec-dsl/models/pick_place_single/pick_place_single.robmot \
  -o generation/pick-place-run \
  --prefix /path/to/workspace/install \
  --run-id tutorial
```

The viewer should show this order: move above cube, lower, close, lift, move to
the placement side, lower, open, and retreat. Use `--headless` for an unattended
run.

Inspect the record:

```bash
motion-spec replay generation/pick-place-run/runs/tutorial
motion-spec replay generation/pick-place-run/runs/tutorial --verify
```

The run owns logs, runtime RDF, REC provenance, and its manifest. It references
the generation's source, model, controller, contract, and provenance instead of
copying them.

## 9. Make one observable change

Change `above-z` in `pick-above` from `0.26 m` to `0.30 m`, then run into a new
generation directory:

```bash
motion-spec run \
  src/motion-spec-dsl/models/pick_place_single/pick_place_single.robmot \
  -o generation/pick-place-higher \
  --prefix /path/to/workspace/install \
  --run-id tutorial
```

The pre-grasp waypoint is now 4 cm higher. A new generation is required because
the authored model, RDF, IR, generated controller, and provenance changed; more
runs of an unchanged generation would instead share those static artifacts.
