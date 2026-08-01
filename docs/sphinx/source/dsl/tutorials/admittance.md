# Arc motion and admittance

`models/admittance_arc_single` demonstrates force-aware motion with a wrist
force/torque sensor.

## 1. Generate and open the simulation

```bash
motion-spec run \
  src/motion-spec-dsl/models/admittance_arc_single/admittance_arc_single.robmot \
  -o /tmp/admittance-arc \
  --prefix /path/to/workspace/install \
  --run-id tutorial
```

The interactive viewer is useful here because the task transitions when external
force appears and disappears.

## 2. Bind the wrench observation

The shared world context binds a wrench to both its measurement frame and the scene
sensor:

```robmot
wrench ext-force {
    ref-point: <ft_tree.wrist_ft_body.wrist_ft_site>,
    as-seen-by: <kinova.base_link.base_link_origin>,
    ft-sensor: <wrist_ft>
}
```

The sensor is attached at the wrist, but `as-seen-by` requests force and torque
components expressed in the Kinova base frame. If it were omitted, the wrench
would remain expressed in the sensor's attached frame. `ref-point` independently
states the point about which torque is measured.

Constraints then select scalar components such as
`<shared.world.ext-force>.force.x`.

## 3. Trace the arc

`arc-motion` captures its start pose on `E_ARC_ENTERED`, constructs an end pose, and
uses an `arc` path with a plane normal and amplitude. Its `until all` condition
requires both table proximity and the target y interval.

The path constraints command tangential speed, keep position and orientation on the
arc, and monitor minimum measured progress. If force interrupts the motion, release
fires `E_ARC_ENTERED` again: the start snapshot is refreshed from the current TCP pose
and a new arc is constructed to the unchanged target pose. The robot does not return to
the interrupted path.

Force constraints use `outside` to detect either sign:

```robmot
force-x: <shared.world.ext-force>.force.x
         outside <shared.spec.neg-force-threshold>
         and <shared.spec.force-threshold>
```

Monitors debounce those constraints before emitting `E_FORCE_DETECTED`.

## 4. Convert force to velocity

The `compliance` motion declares one admittance reference per Cartesian axis:

```robmot
admittance velocity-x = {
    force: <shared.world.ext-force>.force.x,
    mass: 2.0,
    damping: 30.0,
    stiffness: 0.0,
    max-velocity: 0.30 m/s
}
```

Its `while` constraints command measured linear velocity to equal those generated
references while holding orientation. Two named `until` groups distinguish release
from table contact, allowing separate FSM transitions.

`mass`, `damping`, `stiffness`, and `max-velocity` are all required. The DSL supplies
no defaults; zero stiffness in this example explicitly disables the spring term.

## 5. Inspect the record

```bash
motion-spec replay /tmp/admittance-arc/runs/tutorial --verify
motion-spec replay /tmp/admittance-arc/runs/tutorial --jsonl
```

Use `--recover-runtime-ttl` when the archived runtime RDF needs to be regenerated
from the frame log.
