# DSL concepts

This page covers the complete `.robmot` authoring surface. Scene topology and FSM
syntax live in their respective DSLs; a motion model imports and references them.

## Model composition

A model contains imports, namespace declarations, and one or more top-level
specifications:

```robmot
import "task.fsm"
import "task.scenex"

ns app = "https://example.org/task/"
ns task = "https://example.org/task/fsm/"
```

The top-level specifications are:

- `exec-context`: scene, platform, and control period;
- `context`: reusable world and specification quantities;
- `guarded-motion`: context plus `when`, `while`, and `until` constraints;
- `constraint-handler`: monitors, controllers, and solvers for one motion.

Names become RDF identifiers under the declared namespace. Angle brackets refer
to named model elements, for example `<shared.spec.speed>` or
`<kinova.base_link>`.

Because a name is appended directly to its namespace, the declared URI must be
absolute and must end with `/` or `#`; it cannot carry a query or a fragment
name, and its path cannot contain an empty segment. `"https://example.org/task"`
would mint `https://example.org/taskspeed`, and `"https://example.org//task/"`
would mint an IRI with an empty segment, so both are rejected when the model is
validated.

## Execution context

Exactly which scene and backend a model runs against is explicit:

```robmot
exec-context (ns=app) task-exec {
    runs-scene: <task_scene_mjc>
    platform: simulation { name: "MuJoCo", version: "3.9" }
    timestep: 1.0 ms
}
```

Platforms are `simulation { name: ..., version: ... }` and `real-world`, optionally
with a name and version. The timestep must be positive.

## Context scopes

Context gives constraints typed inputs and separates observations from authored,
captured, and resulting values.

| Scope | Purpose |
|---|---|
| `world` | Runtime observations from the scene |
| `pre` | Values captured or required before a motion |
| `spec` | Goals, gains, thresholds, trajectories, and generated references |
| `post` | Values produced or asserted after a motion |

A top-level `context` is reusable. A motion or handler can declare local context or
alias a declaration with `<shared.spec>`.

### World quantities

World quantities are `pose`, `velocity-twist`, `wrench`, or `joint-position`.
Their geometric properties bind them to imported scene elements:

| Property | Target |
|---|---|
| `of`, `wrt`, `ref-point`, `as-seen-by` | scene frame |
| `joint` | revolute joint |
| `ft-sensor` | force/torque sensor specification |

```robmot
world {
    pose tcp-base {
        of: <gripper.g_base.g_pinch>,
        wrt: <kinova.base_link>,
        as-seen-by: <kinova.base_link>
    },
    joint-position gripper-pos { joint: <gripper.g_left_driver_joint> }
}
```

### Quantity types

Context quantities support:

- geometry: `pose`, `position`, `orientation`, `distance`, `angle`,
  `linear-distance`, `angular-distance`, `direction`, `free-vector`;
- motion: `velocity-twist`, `acceleration-twist`, `angular-velocity`,
  `linear-velocity`, `linear-acceleration`, `angular-acceleration`, `linear-jerk`;
- dynamics: `wrench`, `force`, `torque`;
- scalar/control: `dimensionless`, `duration`, `path-parameter`;
- generators: `velocity-profile`, `admittance`.

`linear-distance` and `angular-distance` are authoring aliases for `distance` and
`angle`.

### Values and references

Scalars and vectors carry units:

```robmot
linear-distance clearance = 0.10 m,
direction normal = { x: 0.0, y: 0.0, z: 1.0 }
```

References preserve type and may select a subspace or axis:

```robmot
<shared.world.tcp-base>.position.z
<shared.world.twist>.linvel.x
<shared.world.external-force>.force.y
```

`[quantity = value]` overrides a named quantity at the reference site. `pre [...]`,
`spec [...]`, and `post [...]` declare an inline scoped quantity. A bare measure,
such as `0.3 s`, is also a context reference where its type is unambiguous.

Supported view subspaces are `position`, `orientation`, `linvel`, `angvel`,
`linacc`, `angacc`, `force`, and `torque`. Linear axes are `x`, `y`, `z`; angular
axes are `roll`, `pitch`, `yaw`.

### Poses, snapshots, and derived references

A pose combines position and orientation components:

```robmot
pose goal = {
    position: { x: 0.40 m, y: <spec.goal-y>, z: 0.20 m },
    orientation: { roll: 3.14159 rad, pitch: 0.0 rad, yaw: 1.5708 rad }
}
```

Snapshots sample a world quantity or view. They may add an offset and may resample
on an FSM event:

```robmot
pose start = snapshot of <shared.world.tcp-base>,
linear-distance target-x = snapshot of <shared.world.tcp-base>.position.x
                           + <shared.spec.offset>,
pose entered = snapshot of <shared.world.tcp-base> on event task.E_ENTERED
```

A reference value can also be a named reference plus an optional offset:

```robmot
linear-distance shifted = <shared.spec.origin> + <shared.spec.offset>
```

### Velocity profiles

A velocity profile limits how a controller approaches a scalar reference:

```robmot
linear-velocity max-v = 0.08 m/s,
linear-acceleration max-a = 0.30 m/s2,
linear-jerk max-j = 1.0 m/s3,
velocity-profile lower-profile = profile {
    max-velocity: <spec.max-v>,
    max-acceleration: <spec.max-a>,
    measured-velocity: <shared.world.twist>.linvel.z,
    max-jerk: <spec.max-j>,
    shape: s-curve
}
```

Shapes are `trapezoidal` and `s-curve`. Maximum velocity and acceleration are
required and positive; jerk and measured velocity are optional. Attach the profile
to a controller with `profile: <...>`.

A profiled controller drives its constraint's reference as a target rather than tracking
it directly: the profile emits a setpoint that approaches the target within the limits,
recomputed every cycle, and the controller tracks that setpoint. The measured value it
starts from is the constraint's own quantity, so it is never restated.

### Admittance references

Admittance maps measured force to a bounded velocity reference:

```robmot
admittance comply-x = {
    force: <shared.world.external-force>.force.x,
    mass: 2.0,
    damping: 30.0,
    stiffness: 0.0,
    max-velocity: 0.30 m/s
}
```

Mass, damping, and maximum velocity must be positive; stiffness is optional.

## Views

A constraint reads one of four view forms:

```robmot
<shared.world.tcp-base>.position.x
distance between <shared.world.tcp-base> and <shared.world.object-base>
elapsed
<shared.world.gripper-pos>
```

Selectors determine the resulting type. For example, a pose is a pose,
`.position` is a position, and `.position.x` is a distance.

## Trajectories

A trajectory is a context value, normally driven by a `path-parameter`:

| Kind | Required fields | Optional fields |
|---|---|---|
| `lerp` | `start`, `goal`, `alpha` | `profile`: `linear`, `ease-in`, `ease-out`, `ease-in-out` |
| `circle` | `start`, `center`, `plane-normal`, `alpha` | — |
| `arc` | `start`, `end`, `amplitude`, `plane-normal`, `alpha` | — |
| `helix` | `start`, `center`, `axis`, `pitch`, `revolutions`, `alpha` | — |
| `figure8` | `anchor`, `radius`, `plane-normal`, `alpha` | `form`: `gerono` or `bernoulli` |

```robmot
trajectory path = lerp {
    start: <spec.start>,
    goal: <spec.goal>,
    alpha: <spec.progress>,
    profile: ease-in-out
}
```

A trajectory is emitted as two things: the geometry, a `geom-path:Path` of the matching
kind carrying only its shape parameters, and a `geom-op-ext:PathEvaluator` that traverses
it. The evaluator owns the path parameter and the easing, and produces the pose setpoint
the motion tracks. Timing is never on the geometry.

## Guarded motions

A guarded motion describes what must be true before activation, what is controlled
while active, and what can stop or redirect it:

```robmot
guarded-motion (ns=app) approach {
    move: "Move to the pre-grasp pose"
    context { spec { /* goals and snapshots */ } }
    when all { ready: <shared.world.gripper-pos> equal to <shared.spec.open> }
    while { follow: keeping <shared.world.tcp-base>.position equal to <spec.goal>.position }
    until any { timeout: elapsed greater than 5.0 s }
}
```

`when` and `until` accept `any` or `all`. `until` also supports named nested groups:

```robmot
until {
    released all {
        x: <shared.world.force>.force.x between -3.0 N and 3.0 N,
        y: <shared.world.force>.force.y between -3.0 N and 3.0 N
    }
}
```

Constraint names must be unique across all three sections. Prefix a constraint with
`@disable` to retain it in the source without emitting it. A constraint can alias
another constraint with `<motion.constraint>`; until groups can also be referenced.

### Constraint relations

| Relation | Syntax |
|---|---|
| Equality | `equal to REF` |
| Greater than | `greater than REF`, `is larger than REF`, `away from REF` |
| Less than | `less than REF`, `is smaller than REF`, `up to REF` |
| Inside interval | `between LOWER and UPPER` |
| Outside interval | `outside LOWER and UPPER` |

The view and references must resolve to compatible quantity types and units.

## Constraint handlers

Each handler names the motion it implements and contains optional local context,
optional monitors, controllers, and one or more solvers:

```robmot
constraint-handler (ns=app) handler-approach {
    handles: <approach>
    monitors { /* FSM wiring */ }
    controllers { /* control laws */ }
    solvers { /* robot algorithms */ }
}
```

Handlers can alias controllers and solvers from another handler instead of
redeclaring them.

### Monitors

A monitor observes a constraint, a motion's complete `.when`, or its complete
`.until`:

```robmot
ready: monitor <approach.when>
       on activation trigger event task.E_READY
       after active for 0.3 s
       otherwise hold <home>
```

Activation monitors can debounce with `after active for`, and a `when` monitor can
name a safe fallback motion with `otherwise hold`. A level monitor instead writes a
flag:

```robmot
contact: monitor <approach.contact> while active set flag touching
```

Events may be namespaced FSM events or standalone event names.

### Controllers

Controller kinds are `pid`, `impedance`, `abag`, and `feed-forward`.

| Field | Use |
|---|---|
| `constraint` | Controlled constraint; required |
| `profile` | Velocity-profile reference |
| `measured-derivative` | World view used by derivative action |
| `output-saturation` | `max` or `lower`/`upper` output bound |
| `integral-saturation` | `max` or `lower`/`upper` integral bound |
| `Kp`, `Ki`, `Kd`, `decay` | PID/ABAG parameters as applicable |
| `stiffness`, `damping` | Impedance parameters |

`as TYPE` declares the command quantity where inference is insufficient.
`apply at <body>` selects the rigid body for a force command. `via <solver>` selects
a solver explicitly; it may be omitted only when exactly one compatible solver is
available.

### Solvers

| Algorithm | Role |
|---|---|
| `ACHD` | Acceleration-level hybrid dynamics |
| `RNE` | Recursive Newton-Euler dynamics |
| `command-forwarding` | Forward joint or device commands directly |
| `velocity-distribution` | Distribute mobile-base velocity commands |
| `force-distribution` | Distribute base force commands |

Every solver binds an imported scene agent. Dynamics solvers can declare gravity:

```robmot
solver arm-solver {
    agent: <agents.kinova>,
    algorithm: ACHD,
    limits {
        torque: saturation { max: <shared.spec.max-torque> },
        linear-acceleration: saturation { lower: -2.0 m/s2, upper: 2.0 m/s2 }
    },
    gravity: { x: 0.0, y: 0.0, z: -9.81 m/s2 }
}
```

Supported limit targets are `torque`, `linear-acceleration`, and
`angular-acceleration`. Gravity may also reference a context quantity.

## Scene and FSM integration

The scene defines bodies, frames, joints, sensors, agents, asset mappings, and scene
instances. A `.robmot` model consumes those semantic identities; it does not use
MuJoCo names directly. Multiple instances of one kinematic tree remain distinct
through their instance names and generated runtime prefixes.

The FSM owns states, events, transitions, and reactions. Motion monitors emit those
events, and the generated controller runs only the handler associated with the
current FSM state. Event-triggered snapshots are sampled when their named event is
received.

## Units

Authored unit spellings are deliberately compact:

`rad/s2`, `rad/s`, `m/s3`, `m/s2`, `m/s`, `cm/s`, `deg/s`, `Nm`, `rad`, `deg`,
`cm`, `ms`, `m`, `s`, `N`, and `1`.

The generated RDF uses the corresponding QUDT terms.

## Generated outputs

`motion-spec gen` creates one self-contained generation:

| Directory | Contents |
|---|---|
| `generated/source/` | Snapshot of authored DSL inputs |
| `generated/model/` | JSON-LD graphs, FSM artifacts, and IR |
| `generated/controller/` | Generated C++ and CMake project |
| `generated/contract/` | Runtime schema, frame layout, and log protocol |
| `generated/provenance/` | DSL, coordinate, and motion-spec provenance |
| `build/` | Reusable compiled controller |
| `runs/<run-id>/` | Runtime logs, RDF, REC graph, and consumer manifest |

JSON-LD files use the `.ld.json` suffix. RDF data is handled as RDF graphs, not as
application-specific JSON objects.
