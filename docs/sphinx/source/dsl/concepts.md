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
- `tolerances`: model-wide satisfaction bands, one per quantity kind (see
  [Constraint relations](#constraint-relations));
- `ros`: the topics, subscriptions, and actions a model talks to (see
  [ROS topics](#ros-topics));
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
    platform: simulation { name: "MuJoCo" }
    config: "robot.toml"
    timestep: 1.0 ms
}
```

`platform` is `simulation { name: STRING }` or `real-world`, optionally with a
`{ <agent> realized by DEVICE, ... }` block binding scene agents to hardware kinds
(`KinovaGen3`, `KinovaGen3-2F85`, `Robotiq2F85`, `RobotiqFT300s`). `config` is an
optional path to the deployment TOML a real-world run reads addresses and poses
from; `timestep` must be positive.

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
direction normal = (0.0, 0.0, 1.0)
```

Vectors are always a parenthesized, positional `(x, y, z)` -- never `{x: .., y: .., z: ..}`.

References preserve type and may select a subspace or axis:

```robmot
<shared.world.tcp-base>.position.z
<shared.world.twist>.linvel.x
<shared.world.external-force>.force.y
```

A reference is either a named `<...>` with an optional selector tail, or a bare measure such
as `0.3 s`, which is itself a context reference where its type is unambiguous.

Supported view subspaces are `position`, `orientation`, `linvel`, `angvel`,
`linacc`, `angacc`, `force`, and `torque`. Every subspace's axis is `x`, `y`, or `z`; there is
no `roll`/`pitch`/`yaw` selector.

### Poses, snapshots, and derived references

A pose combines position and orientation components:

```robmot
pose goal = {
    position: (0.40, <spec.goal-y>, 0.20) m,
    orientation: euler { axes: xyz extrinsic, angles: (3.14159, 0.0, 1.5708) rad }
}
```

Position is a `(x, y, z)` tuple, each element independently a literal or a reference.
Orientation has no direct roll/pitch/yaw literal: it is `euler { axes: ..., angles: (...) }`
(`axes` one of the six axis orders, e.g. `xyz`, optionally `extrinsic`), a `quat { xyzw: (...) }`,
a `direction-cosine { x: ..., y: ..., z: ... }`, a plain `<...>` reference, or a rotation
relative to another orientation (`<ref> rotated by ...`).

Snapshots sample a world quantity or view. They may carry a trailing arithmetic expression
and may resample on an FSM event:

```robmot
pose start = snapshot of <shared.world.tcp-base>,
length target-x = snapshot of <shared.world.tcp-base>.position.x
                           + <shared.spec.offset>,
pose entered = snapshot of <shared.world.tcp-base> on event task.E_ENTERED
```

A reference value can also be a named reference under a full arithmetic expression -- standard
precedence, parentheses, mixed operators -- not just one repeated operator:

```robmot
length shifted   = <shared.spec.origin> + <shared.spec.offset>,
force  residual  = <shared.world.ext-force>.force.x - <spec.mass-k> * <shared.world.acc>.linacc.x
```

See [quantity expressions](expressions.md) for precedence, dimension-checking rules, and where
else (constraint views/thresholds/tolerances, saturation bounds, profile limits, solver
gravity) a parenthesized inline expression is accepted.

A value can instead be read from the deployment config at generation time, keyed by a dotted
path, taking its type from what the given view resolves to:

```robmot
pose home-pose = [config.poses.home] for <shared.world.pose-ee-base>
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

All five parameters are required: there are no implicit controller defaults. Mass
and maximum velocity must be positive; damping and stiffness must be non-negative.

## Views

A constraint reads one of these view forms:

```robmot
<shared.world.tcp-base>.position.x
<spec.residual>
(<shared.world.ext-force>.force.x - <spec.mass-k> * <shared.world.acc>.linacc.x)
distance between <shared.world.tcp-base> and <shared.world.object-base>
elapsed
progress of <shared.world.tcp-base> along <spec.approach-path>
moving <shared.world.tcp-base> along <spec.approach-path> with <spec.approach-profile>
<shared.world.tcp-base>.position on <spec.approach-path>
```

Selectors (`.position`, `.position.x`, ...) are optional on the first form and determine the
resulting type: a bare quantity like `<shared.world.tcp-base>` is a pose, `.position` is a
position, and `.position.x` is a distance. The same bare-quantity form also names a scalar
context quantity directly (`<spec.residual>`), and a parenthesized inline expression is
accepted too -- see [quantity expressions](expressions.md). `progress`, `moving`, and `on`
read against a `path` context quantity; see [Paths](#paths) for what each of them does.

## Paths

A path is a pose-valued function of a normalized parameter, not a trajectory: it fixes
shape and leaves timing free.

```{math}
:label: path-pose

\mathbf{T}(s) =
\begin{bmatrix}
\mathbf{R}(s) & \mathbf{p}(s) \\
\mathbf{0}^{\mathsf T} & 1
\end{bmatrix}
\in SE(3), \qquad s \in [0,1].
```

{math}`\mathbf{p}(s)` is position, {math}`\mathbf{R}(s)` is orientation, and {math}`s` orders
progress along the shape -- it is neither time nor necessarily arc length. A trajectory only
appears once execution supplies a time law {math}`s(t)`:

```{math}
:label: path-trajectory

\mathbf{T}_d(t) = \mathbf{T}(s(t)).
```

Keeping shape and timing separate follows Bruyninckx, Sections 7.11 and 8.6: runtime motion
generation resolves the degrees of freedom the task left unconstrained, rather than the model
committing to a timing choice up front.

### Path kinds

The DSL provides five path functions:

| Kind | Required fields | Optional fields |
|---|---|---|
| `lerp` | `start`, `goal` | — |
| `circle` | `start`, `center`, `plane-normal` | — |
| `arc` | `start`, `end`, `amplitude`, `plane-normal` | — |
| `helix` | `start`, `center`, `axis`, `pitch`, `revolutions` | — |
| `figure8` | `anchor`, `radius`, `plane-normal` | `form`: `gerono` or `bernoulli` |

A path is emitted as a `geom-path:Path` carrying only its shape parameters; the runtime
evaluates the selected function every control cycle. {numref}`fig-path-kinds` shows all five,
labeled with the variables their equations below use. {math}`\operatorname{Rot}(\mathbf{u},
\theta)` always means "rotate by {math}`\theta` around unit axis {math}`\mathbf{u}`", and the
runtime clamps {math}`s` to {math}`[0,1]` before evaluating.

```{figure} /_static/dsl-paths/path_kinds.svg
:name: fig-path-kinds
:width: 85%

The five path kinds and the variables their equations use, each plotted from its actual
closed-form equation below rather than a hand-drawn approximation.
```

Each subsection below reads the same way: a `.robmot` excerpt (authored input), a table mapping
each authored field to the symbol its equation uses, then the equation (output). Only the fields
in the table are authored -- every other symbol in an equation is computed from them.

#### Linear interpolation (`lerp`)

```robmot
path approach-path = lerp {
    start: <spec.start-pose>,
    goal:  <spec.goal-pose>
}
```

| `.robmot` field | equation symbol | meaning |
|---|---|---|
| `start` | {math}`(\mathbf{R}_0,\mathbf{p}_0)` | pose the path leaves from |
| `goal` | {math}`(\mathbf{R}_1,\mathbf{p}_1)` | pose the path arrives at |

```{image} /_static/dsl-paths/path_kind_lerp.svg
:width: 45%
```

The straight line between them: position interpolates linearly, orientation follows the
shortest rotation from start to goal.

```{math}
:label: path-lerp

\begin{aligned}
\mathbf{p}(s) &= (1-s)\mathbf{p}_0+s\mathbf{p}_1, \\
\mathbf{R}(s) &= \mathbf{R}_0
\operatorname{Exp}\!\left(s\operatorname{Log}
\left(\mathbf{R}_0^{\mathsf T}\mathbf{R}_1\right)\right).
\end{aligned}
```

**Output:** {math}`\mathbf{p}(s),\mathbf{R}(s)` -- the pose at parameter {math}`s`, read back
by whatever constraint follows this path (see
[Tangent, position, and orientation constraints](#tangent-position-and-orientation-constraints)
below). The rotational term is the same pose-difference operation used elsewhere in the DSL,
not component-wise Euler interpolation.

#### Circle

```robmot
path orbit-path = circle {
    start:        <spec.start-pose>,
    center:       <spec.center-pose>,
    plane-normal: <spec.plane-normal>
}
```

| `.robmot` field | equation symbol | meaning |
|---|---|---|
| `start` | {math}`(\mathbf{R}_0,\mathbf{p}_0)` | a point on the circle; also the fixed orientation held throughout |
| `center` | {math}`\mathbf{c}` | the circle's center |
| `plane-normal` | {math}`\mathbf{n}` | unit normal of the circle's plane |

```{image} /_static/dsl-paths/path_kind_circle.svg
:width: 45%
```

One full revolution around {math}`\mathbf{c}`, starting and ending at {math}`\mathbf{p}_0`:

```{math}
:label: path-circle

\mathbf{p}(s)=\mathbf{c}+
\operatorname{Rot}(\mathbf{n},2\pi s)(\mathbf{p}_0-\mathbf{c}),
\qquad \mathbf{R}(s)=\mathbf{R}_0.
```

**Output:** {math}`\mathbf{p}(s)` sweeps a circle of radius {math}`|\mathbf{p}_0-\mathbf{c}|`;
{math}`\mathbf{R}(s)` stays fixed at the start orientation rather than interpolating.

#### Arc

```robmot
path arc-path = arc {
    start:        <spec.start-pose>,
    end:          <spec.end-pose>,
    amplitude:    <spec.arc-height>,
    plane-normal: <spec.path-normal>
}
```

| `.robmot` field | equation symbol | meaning |
|---|---|---|
| `start` | {math}`(\mathbf{R}_0,\mathbf{p}_0)` | pose the arc leaves from |
| `end` | {math}`(\mathbf{R}_1,\mathbf{p}_1)` | pose the arc arrives at |
| `amplitude` | {math}`a` | sagitta: bulge height from the chord midpoint {math}`\mathbf{m}` to the arc, shown in {numref}`fig-path-kinds` |
| `plane-normal` | {math}`\mathbf{n}` | unit normal of the arc's plane |

```{image} /_static/dsl-paths/path_kind_arc.svg
:width: 45%
```

Nothing else here is authored: the chord {math}`\mathbf{q}`, its length {math}`\ell`, half-length
{math}`m`, radius {math}`r`, center {math}`\mathbf{c}`, and included angle {math}`\theta` are all
derived from the four fields above.

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

...then sweeps like the circle, over the included angle instead of a full turn:

```{math}
:label: path-arc

\begin{aligned}
\mathbf{p}(s) &=\mathbf{c}+
\operatorname{Rot}(\mathbf{n},-\theta s)(\mathbf{p}_0-\mathbf{c}), \\
\mathbf{R}(s) &= \mathbf{R}_0
\operatorname{Exp}\!\left(s\operatorname{Log}
\left(\mathbf{R}_0^{\mathsf T}\mathbf{R}_1\right)\right).
\end{aligned}
```

**Output:** {math}`\mathbf{p}(s)` sweeps the arc from {math}`\mathbf{p}_0` to
{math}`\mathbf{p}_1`; {math}`\mathbf{R}(s)` interpolates {math}`\mathbf{R}_0` to
{math}`\mathbf{R}_1` by the identical formula as {eq}`path-lerp`. {math}`a=\ell/2` gives a
semicircle. A zero chord or non-positive amplitude is degenerate; the runtime holds the start
pose instead of dividing by zero.

#### Helix

```robmot
path helix-path = helix {
    start:       <spec.start-pose>,
    center:      <spec.center-pose>,
    axis:        <spec.axis>,
    pitch:       <spec.pitch>,
    revolutions: <spec.revolutions>
}
```

| `.robmot` field | equation symbol | meaning |
|---|---|---|
| `start` | {math}`(\mathbf{R}_0,\mathbf{p}_0)` | a point on the helix; also the fixed orientation held throughout |
| `center` | {math}`\mathbf{c}` | center of the swept circle |
| `axis` | {math}`\mathbf{u}` | unit axis the helix climbs along |
| `pitch` | {math}`h` | rise per revolution, along {math}`\mathbf{u}` |
| `revolutions` | {math}`N` | number of turns over the whole path |

There is no separate `radius` field: like the circle, the swept radius is
{math}`|\mathbf{p}_0-\mathbf{c}|`, fixed by `start` and `center` together.

```{image} /_static/dsl-paths/path_kind_helix.svg
:width: 45%
```

A circle swept along its axis while it turns, so it both revolves and climbs:

```{math}
:label: path-helix

\mathbf{p}(s)=\mathbf{c}+
\operatorname{Rot}(\mathbf{u},2\pi Ns)(\mathbf{p}_0-\mathbf{c})
+\mathbf{u}hNs,
\qquad \mathbf{R}(s)=\mathbf{R}_0.
```

**Output:** {math}`\mathbf{p}(s)` completes {math}`N` revolutions of radius
{math}`|\mathbf{p}_0-\mathbf{c}|` around {math}`\mathbf{c}`, while climbing a total of
{math}`hN` along {math}`\mathbf{u}`; {math}`\mathbf{R}(s)` stays fixed at the start orientation,
as in the circle.

#### Figure eight

```robmot
path weave-path = figure8 {
    anchor:       <spec.anchor-pose>,
    radius:       <spec.radius>,
    plane-normal: <spec.plane-normal>,
    form:         gerono
}
```

| `.robmot` field | equation symbol | meaning |
|---|---|---|
| `anchor` | {math}`(\mathbf{R}_a,\mathbf{a})` | pose both lobes start and finish at |
| `radius` | {math}`r` | lobe size |
| `plane-normal` | {math}`\mathbf{n}` | unit normal of the figure's plane |
| `form` (optional) | selects the equation | `gerono` or `bernoulli` below |

```{image} /_static/dsl-paths/path_kind_figure8.svg
:width: 45%
```

{math}`\mathbf{u}` is not a separate field: it is the anchor frame's own in-plane x-axis, and
{math}`\mathbf{v}=\mathbf{n}\times\mathbf{u}` completes the frame. Set
{math}`\tau=2\pi s+\pi/2`; both forms start and finish at the anchor:

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

**Output:** {math}`\mathbf{p}(s)` traces two lobes through the anchor, shaped by whichever form
was selected; {math}`\mathbf{R}(s)` stays fixed at the anchor orientation.

### Projection and the moving path frame

A path knows its geometry as a function of {math}`s`, but execution only ever measures a
position {math}`\mathbf{p}_{m,k}` -- it does not know {math}`s` directly. Each control cycle
therefore finds the closest point on the path to what was measured, searching a small window
{math}`\mathcal{W}_k` around the previous cycle's answer -- sized from how far off the last
measurement was -- rather than the whole path, so tracking stays on the current branch through a
self-intersection instead of jumping to another geometrically close one:

```{math}
:label: path-projection

s_k^*=\underset{s\in\mathcal{W}_k}{\operatorname{argmin}}
\;\lVert\mathbf{p}(s)-\mathbf{p}_{m,k}\rVert^2.
```

{numref}`fig-path-frame` shows the result: the projected point {math}`\mathbf{p}(s^*)`, the
measured point {math}`\mathbf{p}_m`, and the local frame built at {math}`\mathbf{p}(s^*)` that
the next section's three constraints track against.

```{figure} /_static/dsl-paths/path_frame.svg
:name: fig-path-frame
:width: 75%

The projected point, the measured position, and the tangent/normal frame built there,
computed by running the actual projection, tangent, and frame equations below on an
illustrative curve.
```

The unit tangent {math}`\mathbf{t}` is a central difference around {math}`s^*`, clamped at the
path's endpoints ({math}`\delta_s=10^{-4}`):

```{math}
:label: path-tangent

\mathbf{t}(s^*)=
\frac{\mathbf{p}(s_+)-\mathbf{p}(s_-)}
{\left\lVert\mathbf{p}(s_+)-\mathbf{p}(s_-)\right\rVert},
\qquad s_\pm = \operatorname{clip}(s^*\pm\delta_s,\,0,\,1).
```

Two normals complete the frame: {math}`\mathbf{n}_1` is parallel-transported from the previous
cycle rather than recomputed from scratch, so it does not flip as the curve turns, and
{math}`\mathbf{n}_2` is simply {math}`\mathbf{t}\times\mathbf{n}_1`:

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

At initialization, or at a singularity where the transport is undefined, the runtime picks the
world axis least aligned with the tangent instead.

### Tangent, position, and orientation constraints

The frame in {numref}`fig-path-frame` is what path following actually tracks against. Three
ordinary motion constraints, each with one job, together cover all six degrees of freedom:

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

- **Tangent** (`moving ... along ... with ...`) drives along {math}`\mathbf{t}`. Given the
  measured twist {math}`\mathbf{V}_m=[\boldsymbol{\omega}_m^{\mathsf T}\;
  \mathbf{v}_m^{\mathsf T}]^{\mathsf T}` (Featherstone, Section 2.2), it regulates the along-path
  speed {math}`v_{\parallel}=\mathbf{t}^{\mathsf T}\mathbf{v}_m` toward the profile's commanded
  speed. It sets progress and timing but never pulls the TCP toward an endpoint position.
- **Position** (`keeping ... .position on ...`) regulates the position error
  {math}`\mathbf{e}_p=\mathbf{p}(s^*)-\mathbf{p}_m` -- the {math}`e_p` in
  {numref}`fig-path-frame` -- but contributes only its two lateral components
  {math}`[\mathbf{n}_1^{\mathsf T}\mathbf{e}_p\;\mathbf{n}_2^{\mathsf T}\mathbf{e}_p]^{\mathsf
  T}`. It corrects cross-track drift without fighting the tangent controller over speed.
- **Orientation** (`keeping ... .orientation on ...`) regulates the rotation error
  {math}`\mathbf{e}_R=\operatorname{Log}(\mathbf{R}_m^{\mathsf T}\mathbf{R}(s^*))^\vee` between
  the measured and path-stored orientation at the same {math}`s^*`.

One tangential direction, two lateral directions, three angular directions: six independent
controllers, not two competing Cartesian ones. `progress of ... along ...` only observes
{math}`v_{\parallel}` as a guard or monitor -- it contributes no solver row. Declare this set
once per moved quantity and path; for multiple arms, repeat it per arm.

### Path-speed profile

The velocity profile in `moving ... along ... with <profile>` is the only authority over path
speed -- there is no separate braking speed to configure. Its `max-velocity` is the cruise speed
{math}`v_c`; the runtime works out braking on its own by estimating the remaining arc length
with 16 line-segment samples ahead of {math}`s^*`:

```{math}
:label: path-remaining-length

L_{\mathrm{rem}}\approx
\sum_{i=1}^{16}\lVert\mathbf{p}(s_i)-\mathbf{p}(s_{i-1})\rVert,
\qquad s_i=s^*+(1-s^*)i/16,
```

and bounding the target speed so the profile can still stop within that distance at
{math}`a_{\max}`:

```{math}
:label: path-braking-speed

v_b=\sqrt{2a_{\max}L_{\mathrm{rem}}}, \qquad
v_{\mathrm{target}}=\operatorname{sgn}(v_c)
\min(|v_c|,v_b).
```

{numref}`fig-speed-profile` compares the two profile shapes this target then feeds. A trapezoidal
profile ramps at the acceleration limit alone, one cycle of duration {math}`\Delta t` at a time:

```{math}
:label: path-trapezoidal-profile

v_{k+1}=v_k+
\operatorname{clip}(v_{\mathrm{target}}-v_k,
-a_{\max}\Delta t,a_{\max}\Delta t).
```

```{figure} /_static/dsl-paths/speed_profile.svg
:name: fig-speed-profile
:width: 75%

Trapezoidal versus S-curve speed, each ramping to the cruise speed and braking from its own
actual remaining distance, simulated by running the equations below.
```

An S-curve profile additionally limits how fast the acceleration itself may change, rounding the
corners the trapezoidal profile leaves sharp:

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

The tangent PID tracks this generated {math}`v_{k+1}`; position and orientation keep tracking
the path geometry throughout, including during acceleration and braking.

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
| Greater than | `greater than REF`, `more than REF`, `is larger than REF`, `away from REF` |
| Less than | `less than REF`, `is smaller than REF`, `up to REF` |
| Inside interval | `between LOWER and UPPER` |
| Outside interval | `outside LOWER and UPPER` |

The view and references must resolve to compatible quantity types and units. Every `REF` above
also accepts a parenthesized inline expression (`outside (0.0 N - <spec.thr>) and <spec.thr>`);
see [quantity expressions](expressions.md).

A constraint may carry its own tolerance with a trailing `within REF`:

```robmot
hold-position: keeping <shared.world.tcp-base>.position
               equal to <spec.goal>.position
               within <shared.spec.satisfied-band>
```

A constraint that authors no `within` instead takes the model-wide default for the kind its
error carries, declared once at the top level:

```robmot
tolerances {
    position:         0.01 m,
    linear-velocity:  0.01 m/s,
    orientation:      5 deg
}
```

One entry per quantity kind that needs a default; a kind with no entry and no `within` on the
constraint has no band at all.

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

`publish`, declared inside the state block whose condition should drive it (`satisfied` or
`violated`, exactly like `trigger`, `hold`, and `flag`), streams the monitor's verdict onto a
declared ROS topic. A monitor answering a served ROS action instead uses `result` in the same
position. See [ROS topics](#ros-topics) below for both.

### Controllers

Controller kinds are `pid`, `impedance`, and `feed-forward`.

| Field | Use |
|---|---|
| `constraint` | Controlled constraint; required |
| `profile` | Velocity-profile reference |
| `measured-derivative` | World view used by derivative action |
| `output-saturation` | `max` or `lower`/`upper` output bound |
| `integral-saturation` | `max` or `lower`/`upper` integral bound |
| `Kp`, `Kd`, `decay` | PID only: `Kp` and `Kd` required, `decay` optional |
| `Ki` | Required on `pid`; optional anti-windup integral on `impedance` |
| `stiffness`, `damping` | Impedance only, at least one of the two required |

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
| `mobile-platform` | `velocity-distribution` | Map a desired platform twist to drive/wheel velocity commands (control allocation) |
| `mobile-platform` | `force-composition` | Reconstruct the platform wrench from measured wheel/drive forces |
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
        gravity: (0.0, 0.0, 9.81) m/s^2
    }
}
```

The only supported solver limit target is `torque`, and only `serial-chain` accepts a
`limits` block at all. Clamping a controller's output (e.g. a Cartesian acceleration)
is a controller-level concern: use `output-saturation` on the controller instead.
Gravity may also reference a context quantity.

A `mobile-platform` entry requires exactly one `configuration` (the backend lookup
key) and exactly one `quantity` context reference. The reference's kind must match the
algorithm's quantity axis: a `velocity-twist` for `velocity-composition`/`velocity-distribution`,
a `wrench` for `force-composition`/`force-distribution`. The operation axis (composition vs.
distribution) lives in the algorithm token, not a separate field, so no combination is
representable as illegal state:

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

A model declares everything it talks to over ROS once, at the top level, in one `ros` block.
Each of its five groups -- `publishers`, `subscribers`, `action-clients`, `action-servers`,
and `always` -- is its own scope, so an entry is referred to by what it is,
`<ros.publishers.table-anchor-pose>`, rather than by the namespace it happens to mint IRIs in:

```robmot
ros (ns=app) {
    publishers {
        table-anchor-pose: topic "/table_anchor_pose" message "geometry_msgs/msg/PoseStamped",
    },
    subscribers {
        table-anchor: topic "/recognized_objects" message "vision_msgs/msg/Detection3DArray" {
            observes { <shared.world.pose-table-anchor-cam> }
            pose from results
        },
    },
    action-servers {
        collab-bhv: action "/bdd/collab_bhv_server" type "control_msgs/action/GripperCommand" {
            on-goal: produce event <collab-coord.E_GOAL>,
        },
    },
    always {
        publish at 10.0 Hz to <ros.publishers.table-anchor-pose>
                with <shared.world.pose-table-anchor-cam>,
    },
}
```

All five groups are optional, and a model declares only the ones it needs.

### Publishing from a monitor

A `publish` belongs to a monitor's state block (`satisfied` or `violated`), and states the
fields it writes:

```robmot
done: monitor <approach.reached> {
    satisfied {
        trigger: event <task.E_DONE>,
        publish: to <ros.publishers.approach-done> { trinary.value: TRUE },
    },
    violated { publish: to <ros.publishers.approach-done> { trinary.value: FALSE } },
}
```

A field path is the dot-joined member path of the message. A value is a bare constant the
message class defines (`TRUE`), a quoted string, or a number; a string field takes a quoted
string, a numeric field a number. When the message type has exactly one payload field left
after the auto-filled ones are set aside, the sugar `publish: <value> to <topic>` names the
value alone and the field is resolved at generation:

```robmot
satisfied { publish: TRUE to <ros.publishers.approach-done> },
```

A third form fires one or more FSM events as the message instead of naming fields directly:

```robmot
satisfied { publish: events { <task.E_PICK_END> } to <ros.publishers.bdd-events> },
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

### Answering a served action

`action-servers` declares an action this model's own FSM serves; `on-goal` is the only field --
a new goal produces the named event and nothing else is authored on the server itself:

```robmot
action-servers {
    collab-bhv: action "/bdd/collab_bhv_server" type "control_msgs/action/GripperCommand" {
        on-goal: produce event <collab-coord.E_GOAL>,
    },
}
```

A monitor answers that goal with `result`, in the same position as `publish` inside a
`satisfied` or `violated` state block:

```robmot
mon-opened: monitor <hold-close.until> {
    satisfied {
        trigger: event <collab-coord.E_OBJ_GRASPED>,
        publish: events { <collab-coord.E_PICK_END> } to <ros.publishers.bdd-events>,
        result: succeeded <ros.action-servers.collab-bhv> { result.trinary.value: TRUE }
    },
}
```

A monitor may `publish` and `result` in the same state -- they answer different things. Only
`succeeded` and `aborted` are authorable outcomes: a cancel is the client's to ask for, and a
goal whose cancel is accepted is always reported `CANCELED`, whatever the model states. A run
that ends without an authored answer -- shutdown, or reaching the FSM end state with no
`result` -- aborts with the result type's own defaults; the FSM `end:` state itself answers
nothing, it only ends the loop. At most one monitor answers a given action.

### Subscribing and detecting

`subscribers` reads a topic into a world quantity. `observes` names the quantities the message
updates, and `pose from` says which message field carries the pose (`results` above is the
detection array's own field name, not a model-authored one):

```robmot
subscribers {
    table-anchor: topic "/recognized_objects" message "vision_msgs/msg/Detection3DArray" {
        observes { <shared.world.pose-table-anchor-cam> }
        pose from results
    },
}
```

`action-clients` declares an action this model calls out to, for `detect ... using <action>` to
drive:

```robmot
action-clients {
    find-table: action "/perception/find_table" type "vision_msgs/action/Detect3D",
}
```

```robmot
locate: detect <table.anchor_frame> using <ros.action-clients.find-table>
```

### Standing publications

`always` publishes independent of any monitor, at a fixed rate, for as long as the model runs.
`with <quantity>` reports one world quantity whole; `with { <subject>: <quantity>, ... }` tags
each entry when the message carries more than one:

```robmot
always {
    publish at 10.0 Hz to <ros.publishers.table-anchor-pose>
            with <shared.world.pose-table-anchor-cam>,
    publish at 10.0 Hz to <ros.publishers.seen-objects> with {
        <world_tree.robot-table-body>: <shared.world.pose-table-anchor-cam>,
    },
}
```

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

Authored unit spellings are deliberately compact; every exponentiated unit uses `^`, never a
bare digit:

`rad/s^2`, `rad/s`, `deg/s^2`, `deg/s`, `m/s^3`, `m/s^2`, `m/s`, `cm/s`, `Nm`, `N`, `rad`, `deg`,
`mm`, `cm`, `m`, `ms`, `s`, `Hz`, and `1`.

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
