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

For an `ft-sensor` wrench, omitting `ref-point` or `as-seen-by` defaults that
property to the sensor's attached frame. Set `as-seen-by` explicitly when the
force and torque components must be expressed in another frame, such as the robot
base; the runtime transforms the observation before constraints consume its axes.
This sensor binding currently applies to force/torque observations, not RGB or
depth images.

### Quantity types

Context quantities support:

- geometry: `pose`, `position`, `orientation`, `length`, `distance`, `angle`,
  `direction`, `free-vector`;
- motion: `velocity-twist`, `acceleration-twist`, `angular-velocity`,
  `linear-velocity`, `linear-acceleration`, `angular-acceleration`, `linear-jerk`;
- dynamics: `wrench`, `force`, `torque`;
- scalar/control: `dimensionless`, `duration`, `path-parameter`;
- generators: `velocity-profile`, `admittance`.

`length` is the general scalar linear quantity (ISO 80000-3 3-1.1): it may be signed,
so it covers a coordinate or an offset as well as a separation. `distance` is the
non-negative length between two points. Use `length` unless the value really cannot be
negative.

### Values and references

Scalars and vectors carry units:

```robmot
length clearance = 0.10 m,
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
length target-x = snapshot of <shared.world.tcp-base>.position.x
                           + <shared.spec.offset>,
pose entered = snapshot of <shared.world.tcp-base> on event task.E_ENTERED
```

A reference value can also be a named reference plus an optional offset:

```robmot
length shifted = <shared.spec.origin> + <shared.spec.offset>
```

### Velocity profiles

A velocity profile limits how a controller approaches a scalar reference:

```robmot
linear-velocity max-v = 0.08 m/s,
linear-acceleration max-a = 0.30 m/s^2,
linear-jerk max-j = 1.0 m/s^3,
velocity-profile lower-profile = profile {
    max-velocity: <spec.max-v>,
    max-acceleration: <spec.max-a>,
    measured-velocity: <shared.world.twist>.linvel.z,
    max-jerk: <spec.max-j>,
    shape: s-curve
}
```

Shapes are `trapezoidal` and `s-curve`. Maximum velocity and acceleration are
required and positive; jerk and measured velocity are optional. Attach a scalar
reference profile to a controller with `profile: <...>`. A path-speed profile belongs
to the path driver instead, using `moving ... along ... with <profile>`.

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

All four parameters are required: there are no implicit controller defaults. Mass
and maximum velocity must be positive; damping and stiffness must be non-negative.

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

## Paths

A path is a pose-valued geometric function over a normalized parameter:

```{math}
:label: path-pose

\mathbf{T}(s) =
\begin{bmatrix}
\mathbf{R}(s) & \mathbf{p}(s) \\
\mathbf{0}^{\mathsf T} & 1
\end{bmatrix}
\in SE(3), \qquad s \in [0,1].
```

Here, {math}`\mathbf{p}(s)` is position and {math}`\mathbf{R}(s)` is orientation.
The parameter orders progress, but is neither time nor necessarily arc length. A trajectory
appears only when execution supplies a time law {math}`s(t)`:

```{math}
:label: path-trajectory

\mathbf{T}_d(t) = \mathbf{T}(s(t)).
```

This distinction is deliberate: a path fixes geometry while leaving timing free. It follows
the ordering in Bruyninckx, Sections 7.11 and 8.6, where runtime motion generation resolves
the degrees of freedom that the task did not constrain.

### Path kinds

The DSL provides these path functions:

| Kind | Required fields | Optional fields |
|---|---|---|
| `lerp` | `start`, `goal` | — |
| `circle` | `start`, `center`, `plane-normal` | — |
| `arc` | `start`, `end`, `amplitude`, `plane-normal` | — |
| `helix` | `start`, `center`, `axis`, `pitch`, `revolutions` | — |
| `figure8` | `anchor`, `radius`, `plane-normal` | `form`: `gerono` or `bernoulli` |

```robmot
path approach-path = lerp {
    start: <spec.start>,
    goal: <spec.goal>
}
```

A path is emitted as a `geom-path:Path` carrying only its shape parameters. The runtime
clamps {math}`s` to {math}`[0,1]` and evaluates the selected function below. In these
equations, {math}`\operatorname{Rot}(\mathbf{u},\theta)` rotates by {math}`\theta`
around unit axis {math}`\mathbf{u}`.

#### Linear interpolation (`lerp`)

For start pose {math}`(\mathbf{R}_0,\mathbf{p}_0)` and goal pose
{math}`(\mathbf{R}_1,\mathbf{p}_1)`:

```{math}
:label: path-lerp

\begin{aligned}
\mathbf{p}(s) &= (1-s)\mathbf{p}_0+s\mathbf{p}_1, \\
\mathbf{R}(s) &= \mathbf{R}_0
\operatorname{Exp}\!\left(s\operatorname{Log}
\left(\mathbf{R}_0^{\mathsf T}\mathbf{R}_1\right)\right).
\end{aligned}
```

The rotational expression is the shortest rotation interpolation used by the pose-difference
operation; it is not component-wise interpolation of Euler angles.

#### Circle

For center {math}`\mathbf{c}`, start position {math}`\mathbf{p}_0`, and plane normal
{math}`\mathbf{n}`:

```{math}
:label: path-circle

\mathbf{p}(s)=\mathbf{c}+
\operatorname{Rot}(\mathbf{n},2\pi s)(\mathbf{p}_0-\mathbf{c}),
\qquad \mathbf{R}(s)=\mathbf{R}_0.
```

One traversal covers a full revolution and returns to the start pose.

#### Arc

An arc is defined by its endpoints, plane normal, and positive sagitta (bulge amplitude)
{math}`a`. Let

```{math}
:label: path-arc-derived

\begin{aligned}
\mathbf{q} &= \mathbf{p}_1-\mathbf{p}_0, &
\ell &= \lVert\mathbf{q}\rVert, & m &= \ell/2, \\
\hat{\mathbf{q}} &= \mathbf{q}/\ell, &
\mathbf{b} &= \mathbf{n}\times\hat{\mathbf{q}}, \\
r &= \frac{a^2+m^2}{2a}, &
d &= \frac{a^2-m^2}{2a}, \\
\mathbf{c} &= \frac{\mathbf{p}_0+\mathbf{p}_1}{2}+d\mathbf{b}, &
\theta &= 2\cos^{-1}(-d/r).
\end{aligned}
```

Then

```{math}
:label: path-arc

\mathbf{p}(s)=\mathbf{c}+
\operatorname{Rot}(\mathbf{n},-\theta s)(\mathbf{p}_0-\mathbf{c}),
```

while orientation interpolates as in {eq}`path-lerp`. The special case
{math}`a=\ell/2` gives a semicircle. A zero chord or non-positive amplitude is degenerate;
the runtime holds the start pose instead of dividing by zero.

#### Helix

For axis {math}`\mathbf{u}`, center {math}`\mathbf{c}`, pitch {math}`h`, and number of
revolutions {math}`N`:

```{math}
:label: path-helix

\mathbf{p}(s)=\mathbf{c}+
\operatorname{Rot}(\mathbf{u},2\pi Ns)(\mathbf{p}_0-\mathbf{c})
+\mathbf{u}hNs,
\qquad \mathbf{R}(s)=\mathbf{R}_0.
```

#### Figure eight

Let {math}`\mathbf{a}` be the anchor position, {math}`\mathbf{n}` the plane normal,
and {math}`\mathbf{u}` the anchor frame's x-axis projected into that plane and normalized.
Set {math}`\mathbf{v}=\mathbf{n}\times\mathbf{u}` and
{math}`\tau=2\pi s+\pi/2`. Both forms start and finish at the anchor:

```{math}
:label: path-figure-eight

\begin{aligned}
\mathbf{p}_{\text{Gerono}}(s)
 &= \mathbf{a}+r\left(\cos\tau\,\mathbf{u}
 +\sin\tau\cos\tau\,\mathbf{v}\right), \\
\mathbf{p}_{\text{Bernoulli}}(s)
 &= \mathbf{a}+\frac{r}{1+\sin^2\tau}
 \left(\cos\tau\,\mathbf{u}
 +\sin\tau\cos\tau\,\mathbf{v}\right), \\
\mathbf{R}(s) &= \mathbf{R}_a.
\end{aligned}
```

### Projection and the moving path frame

Each control cycle projects the measured TCP position {math}`\mathbf{p}_{m,k}` onto a local
window {math}`\mathcal{W}_k\subseteq[0,1]` around the preceding projection:

```{math}
:label: path-projection

s_k^*=\underset{s\in\mathcal{W}_k}{\operatorname{argmin}}
\;\lVert\mathbf{p}(s)-\mathbf{p}_{m,k}\rVert^2.
```

The implementation derives {math}`\mathcal{W}_k` from the preceding {math}`s_{k-1}^*` and
the measured deviation. This preserves the current branch at a self-intersection
instead of jumping to another geometrically close branch.

The unit tangent is evaluated by a central difference with {math}`\delta_s=10^{-4}` and
endpoint-clamped parameters {math}`s_- = \max(0,s^*-\delta_s)` and
{math}`s_+ = \min(1,s^*+\delta_s)`:

```{math}
:label: path-tangent

\mathbf{t}(s^*)=
\frac{\mathbf{p}(s_+)-\mathbf{p}(s_-)}
{\left\lVert\mathbf{p}(s_+)-\mathbf{p}(s_-)\right\rVert}.
```

Two normals complete the local orthonormal frame. The first normal is parallel-transported
from the previous cycle and the second is its cross product with the tangent:

```{math}
:label: path-frame

\begin{aligned}
\tilde{\mathbf{n}}_1 &=(\mathbf{I}-\mathbf{t}\mathbf{t}^{\mathsf T})
\mathbf{n}_{1,k-1}, &
\mathbf{n}_{1,k}&=\tilde{\mathbf{n}}_1/
\lVert\tilde{\mathbf{n}}_1\rVert, \\
\mathbf{n}_{2,k}&=\mathbf{t}_k\times\mathbf{n}_{1,k}.
\end{aligned}
```

Transport avoids discontinuous normal-axis flips as the curve turns. At initialization or a
singularity, the runtime chooses the world axis least aligned with the tangent.

### Tangent, position, and orientation constraints

Path following is therefore expressed by ordinary motion constraints with three distinct
jobs:

```robmot
while {
    follow-tangent: moving <shared.world.tcp-base>
                    along <spec.approach-path> with <spec.approach-profile>,
    follow-position: keeping <shared.world.tcp-base>.position
                     on <spec.approach-path>,
    follow-orientation: keeping <shared.world.tcp-base>.orientation
                        on <spec.approach-path>,
    advance: progress of <shared.world.tcp-base>
             along <spec.approach-path> more than <spec.min-approach-speed>
}
```

Let the measured rigid-body twist be
{math}`\mathbf{V}_m=[\boldsymbol{\omega}_m^{\mathsf T}\;
\mathbf{v}_m^{\mathsf T}]^{\mathsf T}`, following Featherstone's spatial-velocity
decomposition (Section 2.2). The measured speed along the path is

```{math}
:label: path-along-speed

v_{\parallel}=\mathbf{t}^{\mathsf T}\mathbf{v}_m.
```

The three controllers occupy complementary directions:

- **Tangent:** `moving ... along ... with ...` regulates
  {math}`e_t=v_{\mathrm{cmd}}-v_{\parallel}` along {math}`\mathbf{t}`. It determines
  progress and timing, but does not pull the TCP toward an endpoint position.
- **Position:** `keeping ... .position on ...` uses
  {math}`\mathbf{e}_p=\mathbf{p}(s^*)-\mathbf{p}_m`, but contributes only the two lateral
  components
  {math}`[\mathbf{n}_1^{\mathsf T}\mathbf{e}_p\;
  \mathbf{n}_2^{\mathsf T}\mathbf{e}_p]^{\mathsf T}`. It corrects cross-track error without
  competing with the tangent-speed controller.
- **Orientation:** `keeping ... .orientation on ...` regulates the three-component rotation
  error
  {math}`\mathbf{e}_R=\operatorname{Log}
  (\mathbf{R}_m^{\mathsf T}\mathbf{R}(s^*))^\vee`. It aligns the body with the orientation
  stored at the same projected path parameter.

Together these form one six-dimensional task decomposition: one tangential linear direction,
two normal linear directions, and three angular directions. This is why tangent and position
are not duplicate Cartesian controllers.

`progress of ... along ...` observes {math}`v_{\parallel}` and acts only as a guard or
monitor; it contributes no solver row. Controllers bind to the driver and geometry constraints
in the normal way. For multiple arms, declare this constraint set once per moved quantity and
path.

### Path-speed profile

The velocity profile in `moving ... along ... with <profile>` is the single authority for
path speed. Its `max-velocity` is the cruise speed {math}`v_c`; there is no second `at` speed.
To brake before the endpoint, the runtime estimates the remaining arc length with 16 line
segments. For {math}`s_i=s^*+(1-s^*)i/16`:

```{math}
:label: path-remaining-length

L_{\mathrm{rem}}\approx
\sum_{i=1}^{16}\lVert\mathbf{p}(s_i)-\mathbf{p}(s_{i-1})\rVert.
```

It then applies the constant-acceleration bound

```{math}
:label: path-braking-speed

v_b=\sqrt{2a_{\max}L_{\mathrm{rem}}}, \qquad
v_{\mathrm{target}}=\operatorname{sgn}(v_c)
\min(|v_c|,v_b).
```

For a trapezoidal profile, one cycle of duration {math}`\Delta t` is

```{math}
:label: path-trapezoidal-profile

v_{k+1}=v_k+
\operatorname{clip}(v_{\mathrm{target}}-v_k,
-a_{\max}\Delta t,a_{\max}\Delta t).
```

For an S-curve profile, the acceleration change is additionally jerk-limited:

```{math}
:label: path-s-curve-profile

\begin{aligned}
a^* &= \operatorname{clip}
\left(\frac{v_{\mathrm{target}}-v_k}{\Delta t},-a_{\max},a_{\max}\right), \\
a_{k+1} &= a_k+\operatorname{clip}
(a^*-a_k,-j_{\max}\Delta t,j_{\max}\Delta t), \\
v_{k+1} &= \operatorname{clip}
(v_k+a_{k+1}\Delta t,-|v_c|,|v_c|).
\end{aligned}
```

The tangent PID tracks this generated {math}`v_{k+1}`. Position and orientation continue to
track the path geometry during acceleration and braking.

### References

- Herman Bruyninckx, *Composable and Explainable Systems of Systems*, Sections 7.11,
  8.6, and 8.6.3.
- Roy Featherstone, *Rigid Body Dynamics Algorithms*, Section 2.2,
  [doi:10.1007/978-1-4899-7560-7](https://doi.org/10.1007/978-1-4899-7560-7).

## Guarded motions

A guarded motion describes what must be true before activation, what is controlled
while active, and what can stop or redirect it:

```robmot
guarded-motion (ns=app) approach {
    description: "Move to the pre-grasp pose"
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
`.until`. While its motion is scheduled it is in one of two states each cycle:
`satisfied` or `violated`. Each state block lists what the monitor does while it is in
that state:

```robmot
ready: monitor <approach.when> {
    satisfied for 0.3 s { trigger: event <task.E_READY> },
    violated { hold: <home> },
}
```

`trigger` fires on entry into the state, and `for <Measure>` debounces that entry.
`hold` names a safe fallback motion and belongs in `violated`; `flag`, which writes a
boolean the model can read, belongs in `satisfied`:

```robmot
contact: monitor <approach.contact> { satisfied { flag: touching } }
```

Events may be namespaced FSM events or standalone event names.

`publish`, declared once per monitor rather than inside a state block, streams the
monitor's verdict onto a declared ROS topic; see [ROS topics](#ros-topics) below.

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

A solver is authored as one of three closed mechanism families -- never a generic
algorithm string. Each accepts only its own fields.

| Mechanism | Algorithm | Role |
|---|---|---|
| `serial-chain` | `achd` | Acceleration-level hybrid dynamics over one ordered kinematic chain |
| `serial-chain` | `rne` | Recursive Newton-Euler dynamics over one ordered kinematic chain |
| `mobile-platform` | `velocity-composition` | Reconstruct the platform twist from measured wheel/drive velocities |
| `mobile-platform` | `force-distribution` | Map a desired platform wrench to drive/wheel forces (control allocation) |
| `command-forwarding` | -- | Forward a feed-forward controller's command directly to a device |

Every solver binds an imported scene agent. A `serial-chain` entry can declare gravity
and, optionally, a torque limit:

```robmot
solvers {
    arm-solver: serial-chain {
        agent: <agents.kinova>,
        algorithm: achd,
        limits {
            torque: saturation { max: <shared.spec.max-torque> }
        },
        gravity: { x: 0.0, y: 0.0, z: -9.81 m/s2 }
    }
}
```

The only supported solver limit target is `torque`, and only `serial-chain` accepts a
`limits` block at all. Clamping a controller's output (e.g. a Cartesian acceleration)
is a controller-level concern: use `output-saturation` on the controller instead.
Gravity may also reference a context quantity.

A `mobile-platform` entry requires exactly one `configuration` (the backend lookup
key) and exactly one `quantity` context reference. The reference's kind must match the
algorithm: a `velocity-twist` for `velocity-composition`, a `wrench` for
`force-distribution`. There is one `mobile-platform` DSL class over the quantity x
operation matrix; only these two operations are backed by a vendored RDF class
(`slv:VelocityCompositionSolver`, `slv:ForceDistributionSolver` --
comp-rob2b `solver-specification.ttl:25,35`), so the `algorithm` enum stays closed to
them for now:

```robmot
solvers {
    platform-velocity: mobile-platform {
        agent: <agents.platform>,
        algorithm: velocity-composition,
        configuration: "hddc2b_example_vel",
        quantity: <world.platform-twist>
    },
    platform-force: mobile-platform {
        agent: <agents.platform>,
        algorithm: force-distribution,
        configuration: "hddc2b_example_frc_sc1",
        quantity: <world.platform-wrench>
    }
}
```

`command-forwarding` takes only an agent -- it is neither a dynamics solver nor a
platform kinematics/control-allocation solver:

```robmot
solvers {
    gripper-solver: command-forwarding { agent: <agents.gripper> }
}
```

## ROS topics

A model declares the topics it publishes on once, at the top level, and monitors
reference them by name:

```robmot
ros-topics (ns=task) {
    approach-done: topic "/motion/approach_done" message "bdd_ros2_interfaces/msg/TrinaryStamped",
}
```

A `publish` belongs to a monitor's state block, and states the fields it writes:

```robmot
done: monitor <approach.reached> {
    satisfied {
        trigger: event <task.E_DONE>,
        publish: to <task.approach-done> { trinary.value: TRUE },
    },
    violated { publish: to <task.approach-done> { trinary.value: FALSE } },
}
```

A field path is the dot-joined member path of the message. A value is a bare constant the
message class defines (`TRUE`), a quoted string, or a number; a string field takes a quoted
string, a numeric field a number. When the message type has exactly one payload field left
after the auto-filled ones are set aside, the sugar `publish: <value> to <topic>` names the
value alone and the field is resolved at generation:

```robmot
satisfied { publish: TRUE to <task.approach-done> },
```

A publish is live only while its state holds, and only while the monitor's motion is
scheduled: every control cycle in that state, the block's fields are written and the message
goes out. A state with no publish sends nothing, so a satisfied-only publish is a signal that
appears at satisfaction and is silent otherwise. While the motion is unscheduled the monitor
says nothing at all, and a subscriber reads that silence as "not being evaluated" -- so it
stays distinguishable from an evaluated false without a third published value.

Any message type works, as long as every authored path is one of its payload fields.
Timestamps and the `scenario_context_id` UUID are filled by the node, never authored: the
clock supplies the stamp, and the `scenario_context_id` node parameter supplies the scenario,
live-settable with `ros2 param set` while the controller runs.

A monitor publishes on at most one topic; a second topic is a second monitor, and a state
block publishes at most once.

A `satisfied` publish holds under the constraint the monitor watches; a `violated` publish
states no condition at all -- it is the otherwise, taken while the monitor evaluates and its
constraint does not hold. Any monitor may publish on `violated`, including one watching a whole
`until`/`when` section, but only alongside a `satisfied` publish: without the case it is
otherwise to, "otherwise" is not "violated".

### Joint states

The deployment, not the model, decides whether joint states are published. The section's
presence in the platform config turns the publisher on:

```toml
[ros.joint_states]
topic = "/joint_states"   # optional, default shown
rate = 100.0              # Hz, optional; omitted publishes every control cycle
```

One `sensor_msgs/msg/JointState` reports every chain joint the model drives, under its
runtime-qualified name. The topic and rate are read at launch, so retuning either needs
no regeneration.

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
