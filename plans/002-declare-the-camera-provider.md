# 002 — Declare the camera's provider instead of guessing its topic

**Written against:** `8aab89d` (branch `dev`, repo `src/motion-spec`)
**Also touches:** `src/motion-spec-dsl` (grammar, classes, validation, RDF emission)
**Status:** TODO
**Effort:** M
**Depends on:** —

Before executing: run `git rev-parse --short HEAD` in `src/motion-spec` and in
`src/motion-spec-dsl`. If either has moved past the stamp above, re-read every file excerpted
here before trusting a line number.

---

## 1. Outcome and semantic contract

Today the dashboard invents the ROS topic a real robot's camera is published on. It builds the
string from the camera's name and a hardcoded suffix. This plan removes that invention: the
topic becomes something the model states, and a camera with no stated provider gets no live
pane at all.

### Authored syntax (the whole user-visible change)

In a `.robmot` `ros (ns=...)` block, a subscription may name a **scene sensor** as what it
observes, in which case it carries no pose path:

```
ros (ns=app) {
    subscribers {
        wrist-view: topic "/wrist/color" message "sensor_msgs/msg/Image" {
            observes { <collab-scene-simreal.wrist> }
        },
    },
}
```

The existing pose-observing form is unchanged and still requires its pose path:

```
        table-anchor: topic "/recognized_objects" message "vision_msgs/msg/Detection3DArray" {
            observes { <shared.world.pose-table-anchor-cam> }
            pose from results
        },
```

- **Cardinality:** a camera has at most one provider. Two subscriptions naming the same camera
  is an error (§5.2). A subscription naming several cameras is allowed only if you have a
  reason; the default rule is one target per camera subscription — reject more than one
  (§5.2) so nothing has to decide which camera the topic "really" carries.
- **Mixing:** one subscription observes either world poses or sensors, never both. A mixed
  `observes { }` is an error.
- **The target must be an rgb camera.** A force-torque or IMU sensor as the target is an
  error: nothing consumes an image stream for those, and `_cameras` (§4.5) drops non-rgb
  cameras already.

### RDF

The topic node gains nothing new. The existing predicate carries the link — this was decided
explicitly and **is not to be revisited**: the camera goes on `sosa:hasFeatureOfInterest`, the
same predicate a pose-observing subscription uses for its pose.

```turtle
<https://.../exec/ros/subscribers/wrist-view>
    a                         ros:Topic ;
    ros:channel-name          "/wrist/color" ;
    ros:type-name             "sensor_msgs/msg/Image" ;
    sosa:hasFeatureOfInterest <https://.../collab-scene-simreal/kinova/wrist> .
```

Note there is **no `ros:field-path`** on a camera subscription — that predicate says where in a
message a pose sits, and there is no pose.

### Runtime meaning

None. A camera subscription generates **no C++**: nothing in the control loop reads an image.
It exists to be recorded in the generation's contract and read by the dashboard. See §4.4 for
why this falls out of the existing code rather than needing a guard.

### Non-goals

- No new RDF term. No `sosa:madeBySensor`. Do not add one.
- No `views` clause, no second declaration form. The widened `subscribers` is the decision.
- No backward compatibility. There is no deployed model authoring a camera provider today.
- No change to `/api/ros-camera` (it already takes the topic as a query parameter) and no
  change to `ros_camera.py`.
- No support for depth or rgbd cameras — `_cameras` already drops them with a warning.
- Do not make the topic settable from `robot.toml`. That was considered and rejected: the
  decision is that the model states it.

---

## 2. Current state — what is being removed

`src/motion_spec/dashboard/frontend/run.js:234` onward, `showRosCamera`:

```js
async function showRosCamera(generationPath) {
  if (!generationPath) return;
  const generation = state.generation?.path === generationPath
    ? state.generation
    : await api(`/api/generation?path=${encodeURIComponent(generationPath)}`).catch(() => null);
  if (generation?.simulated !== false || state.replay?.generation !== generationPath) return;
  // The model names its cameras, and a published camera is <id>/color -- the external driver
  // for the same sensor is expected on the same name. Anything else is typed in.
  const declared = generation.cameras?.[0]?.id;
  const defaultTopic = declared ? `/${declared}/color` : "";
```

`/${declared}/color` is the invention. The comment admits it ("is expected on the same name").

Called from `openReplay` at `run.js:115`:

```js
  showVideos(path, state.replay.videos ?? []);
  // Nothing recorded: a real platform has a camera, but only ROS to reach it through.
  if (!state.replay.videos?.length) showRosCamera(state.replay.generation);
```

---

## 3. Reuse decision

| Reused as-is | Added locally | Not added |
|---|---|---|
| `sosa:hasFeatureOfInterest` — already how a subscription names what it informs the model about | Nothing in RDF | `sosa:madeBySensor` — correct SOSA sense, explicitly rejected by the user; do not introduce |
| `ros:Topic`, `ros:channel-name`, `ros:type-name` from `scene_dsl.rdf_parser.vocab.NS_MM_ROS` | Nothing | A `ros:` predicate for the camera link |
| textX built-in `OBJECT` reference class — already used by `DeviceBinding` (`model.tx:105`) to point at a scene sensor | Nothing | An abstract union rule over `WorldQuantity | Sensor`; unnecessary, `OBJECT` matches any model object |
| `URI_SENS_TYPE_CAMERA` / `get_node_types` from `resources.py` for "is this node a camera" | Nothing | A name- or suffix-based camera test |
| `CameraBinding` dataclass (`resources.py`) | two fields on it | A parallel structure for camera providers |

**No new RDF term is required by this plan.** If implementation reveals one is, **STOP** and
report — do not mint it.

---

## 4. Change map, in data-flow order

### 4.1 Grammar — `src/motion-spec-dsl/src/motion_spec_dsl/grammars/model.tx`

Current, line 63:

```
RosSubscriptionDecl:
    name=IRI_TRUNK ":" "topic" channel_name=STRING "message" type_name=STRING "{"
        "observes" "{" targets+=WorldQuantityRef[","] ","? "}"
        pose_field=ID "from" pose_container=ID ","?
    "}"
;

WorldQuantityRef: "<" ref=[WorldQuantity|FQN] ">" ;
```

Two edits:

1. The target ref must accept a scene sensor as well as a world quantity. Use textX's built-in
   `OBJECT` class, exactly as `DeviceBinding` on line 105 already does
   (`"<" target=[OBJECT|FQN] ">"`). Introduce a ref rule for subscription targets rather than
   widening `WorldQuantityRef`, which `RosStandingEntry` does **not** use but which reads as a
   promise about the referent's type:

```
    "observes" "{" targets+=SubscriptionTargetRef[","] ","? "}"
```
```
SubscriptionTargetRef: "<" ref=[OBJECT|FQN] ">" ;
```

2. The pose path becomes optional:

```
        (pose_field=ID "from" pose_container=ID ","?)?
```

Leave `WorldQuantityRef` in the file if another rule still uses it; delete it if nothing does
(check with `grep -n WorldQuantityRef src/motion_spec_dsl/grammars/*.tx`).

**Note on `OBJECT`:** it is a textX built-in meaning "any model object", not a rule defined
anywhere in this tree — `grep -rn "^OBJECT" --include='*.tx' src/` returns nothing while
`model.tx:105` uses it successfully. Widening to it therefore removes the grammar-level type
check, which is why §4.3's validation must carry it instead.

### 4.2 AST class — `src/motion-spec-dsl/src/motion_spec_dsl/classes/ros.py:44`

Current:

```python
@dataclass(eq=False)
class RosSubscriptionDecl(NamedNamespaceObject):
    """A standing subscription: the channel poses arrive on, the world poses it writes, and where
    in one detection each pose is found.
    """

    parent: object
    name: str
    channel_name: str
    type_name: str
    pose_field: str
    pose_container: str
    targets: list = field(default_factory=list)

    def __post_init__(self):
        super().__init__(parent=self.parent, name=self.name)

    @property
    def pose_path(self) -> str:
        """The dotted path from one detection to the pose it reports."""
        return f"{self.pose_container}.{self.pose_field}"
```

- `pose_field` and `pose_container` become optional (`str | None = None`). textX passes `None`
  or `""` for an unmatched optional assignment — **check which** by parsing a camera
  subscription and printing the value before writing the condition; do not assume.
- `pose_path` returns `None` (not `""`) when either half is absent, so §4.4's emission can test
  it directly.
- Update the docstring: it no longer only carries poses.

### 4.3 Validation — `src/motion-spec-dsl/src/motion_spec_dsl/classes/validation/detects.py:62`

Current:

```python
    for sub in get_children_of_type(RosSubscriptionDecl, model):
        for target in sub.targets:
            if target.ref.type != WorldQuantityType.Pose:
                raise semantic_error(
                    f"Subscription '{sub.name}' observes '{target.ref.name}', which is not a pose.",
                    sub,
                )
            if id(target.ref) in observed:
                raise semantic_error(
                    f"'{target.ref.name}' is observed by more than one source; a world pose has "
                    "one producer.",
                    sub,
                )
            observed.add(id(target.ref))
```

`target.ref.type` is a `WorldQuantityType` on a world quantity; on a scene camera the object is
a `CameraSensorSpec` (from `scene-dsl`, `grammars/sensors.tx`) and has no such attribute, so
this raises `AttributeError` rather than the intended error. Rewrite so the branch is chosen by
what the target **is**:

- Classify each target: world pose, camera sensor, or neither.
- **Neither** → the existing "which is not a pose" message, widened to say a subscription
  observes a world pose or a camera.
- **All targets are world poses** → the current rules apply unchanged, *plus*: `pose_path` must
  be present. A pose subscription with no `pose from ...` is an error naming the missing clause.
- **All targets are camera sensors** → `pose_path` must be absent (error if present: "a camera
  subscription reports no pose"); exactly one target; the camera must be `rgb`
  (`CameraSensorSpec.cam_type`); and the same one-producer rule applies — two subscriptions
  naming one camera is an error.
- **Mixed** → error.

Import `CameraSensorSpec` from wherever `scene-dsl` exposes it; find it with
`grep -rn "class CameraSensorSpec" src/scene-dsl/src/`. If it is not importable as a class,
classify with `type(target.ref).__name__ == "CameraSensorSpec"` **only as a last resort** and
say so in a comment.

Keep the file's existing `semantic_error(...)` style and message voice: lowercase after the
colon, states the rule rather than the fix.

### 4.4 RDF emission — `src/motion-spec-dsl/src/motion_spec_dsl/rdf/motion_spec.py:590`

Current:

```python
    def _emit_ros_subscription(self, subscription: RosSubscriptionDecl) -> None:
        node = URIRef(subscription.uri)
        self.graph.add((node, RDF.type, ROS.Topic))
        self.graph.add((node, ROS["channel-name"], Literal(subscription.channel_name)))
        self.graph.add((node, ROS["type-name"], Literal(subscription.type_name)))
        self.graph.add((node, ROS["field-path"], Literal(subscription.pose_path)))
        for target in subscription.targets:
            self.graph.add(
                (
                    node,
                    SOSA.hasFeatureOfInterest,
                    URIRef(str(target.ref.uri)),
                )
            )
```

One change: emit `ros:field-path` only when there is a pose path.

```python
        if subscription.pose_path is not None:
            self.graph.add((node, ROS["field-path"], Literal(subscription.pose_path)))
```

The `hasFeatureOfInterest` loop is unchanged — it already emits the camera's URI, because a
`CameraSensorSpec` carries `.uri` like any other named object. **Verify that it does** before
relying on it; if it does not, STOP and report rather than constructing a URI by hand.

### 4.5 Read side — `src/motion-spec/src/motion_spec/rdf_parser/quantities.py:851`

This is the one place that breaks. `perceived_written_poses` walks every `ros:Topic`'s features
of interest and resolves each as a pose (line ~880):

```python
    for act in sources:
        rows = []
        for target in sorted(graph.objects(act, SOSA.hasFeatureOfInterest), key=str):
            if NS_MM_ROS["Topic"] in get_node_types(graph, act):
                item = pose(model, target)
                target_iri = item.of.uri
                ...
```

A camera URI is not a `PoseCoordinate`, so `pose(model, target)` fails or returns nonsense.

Fix: skip targets that are cameras, by their type in the graph — not by name, not by URI
shape. `resources.py` already has the predicate for this:

```python
from motion_spec.rdf_parser.resources import URI_SENS_TYPE_CAMERA  # or wherever it is defined
...
            if URI_SENS_TYPE_CAMERA in get_node_types(graph, target):
                continue   # a camera is a provider for a viewer, not a pose the model reads
```

Confirm where `URI_SENS_TYPE_CAMERA` actually comes from — `grep -rn "URI_SENS_TYPE_CAMERA"
src/motion_spec/` — and import it from its origin rather than re-exporting it through
`resources.py` if that creates a cycle.

Extend the docstring: it currently says "A node that observes nothing — a published topic —
contributes no rows"; it must also say a topic that observes only a camera contributes none.

**Consequence, already verified — do not add a guard for it:**
`src/motion_spec/rdf_parser/communication.py:946` `ros_subscriptions` opens with

```python
    for node in sorted(graph.subjects(RDF["type"], NS_MM_ROS["Topic"]), key=str):
        rows = written.get(str(node)) or []
        if not rows:
            continue
```

so once §4.5 leaves a camera subscription with no rows, it is skipped and **no C++ subscriber,
include, member, setup or tick is generated for it**. Confirm this by inspecting the generated
`ros.stg` output in §6 — the camera's topic string must appear nowhere in `generated/controller`.
If it does appear, STOP: something else is consuming the subscription and the plan is wrong.

### 4.6 IR — the camera carries its provider

`src/motion-spec/src/motion_spec/rdf_parser/resources.py:372` `_cameras` builds each camera:

```python
        cameras.append(
            CameraBinding(
                id=f"{runtime_prefix}{local_name(sensor)}",
                width=int(graph.value(sensor, URI_SENS_PRED_RESOLUTION_WIDTH).toPython()),
                height=int(graph.value(sensor, URI_SENS_PRED_RESOLUTION_HEIGHT).toPython()),
                rate_hz=get_update_rate(graph, ModelBase(node_id=sensor, graph=graph)),
                uri=str(sensor),
                frame_id=f"{runtime_prefix}{local_name(sensor)}",
            )
        )
```

Add `topic` and `message` to `CameraBinding` (`resources.py:238` region — read the dataclass
before editing), defaulting to `None`, and fill them from the graph: the provider is the
`ros:Topic` node whose `sosa:hasFeatureOfInterest` is this sensor.

```python
    # The topic a viewer reads this camera off, as the model's subscription states it. A camera
    # nothing subscribes to has no provider and stays None.
```

Resolve it with one reverse lookup per camera:
`graph.subjects(SOSA.hasFeatureOfInterest, sensor)` filtered to nodes typed `ros:Topic`, then
read `NS_MM_ROS["channel-name"]` and `NS_MM_ROS["type-name"]` off it. Two providers for one
camera cannot occur — §4.3 rejects it — but if two are found, raise `ConstraintViolation` with
the camera's name rather than picking one.

**IR correctness note:** put the provider on the camera because that is what it is a fact
about — the camera is reached through this topic. Do not add a separate `communication.ros.views`
list shaped for the contract's convenience.

### 4.7 Contract — `src/motion-spec/src/motion_spec/generation/artifacts.py:451`

Current:

```python
    cameras = [
        {key: camera[key] for key in ("id", "width", "height", "rate_hz", "uri")}
        for camera in ir.get("composition", {}).get("scene", {}).get("cameras") or []
    ]
```

Add the two keys:

```python
        {key: camera[key] for key in ("id", "width", "height", "rate_hz", "uri", "topic", "message")}
```

Check whether `camera` here is a dataclass or a dict at this point — the comprehension indexes
with `[]`, and `test_camera_ir.py` reads `camera.id` off the IR, so something converts between
`resources.py` and here. Follow it and match; do not add a conversion.

Result in `generated/contract/frame_layout.json`:

```json
{"id": "wrist", "width": 640, "height": 480, "rate_hz": 30.0,
 "uri": "https://.../kinova/wrist",
 "topic": "/wrist/color", "message": "sensor_msgs/msg/Image"}
```

A camera with no provider keeps `"topic": null`.

### 4.8 Dashboard read — `src/motion_spec/dashboard/catalog.py:424`

```python
def generation_cameras(generation_dir: Path) -> list[dict]:
    """The cameras this generation can record, as its contract names them."""
    return [
        {"id": camera["id"], "width": camera.get("width"), "height": camera.get("height")}
        for camera in json_file(generation_dir / LAYOUT_REL).get("cameras") or []
        if camera.get("id")
    ]
```

Add `"topic": camera.get("topic")` and `"message": camera.get("message")` to the projection.

### 4.9 Run page — `src/motion_spec/dashboard/frontend/run.js`

In `showRosCamera` (line ~234), replace the invented default with the declared topic, and
return early when there is none:

```js
  // Where the camera is published is the model's to state -- a subscription naming this camera
  // -- not this page's to guess from its name.
  const camera = generation.cameras?.find((entry) => entry.topic);
  if (!camera) return;
  const declaredTopic = camera.topic;
```

Then:

- `topic.value = state.rosTopic ?? declaredTopic;` — the operator may still retype it for the
  session; only the invented default goes.
- The `.video-name` label added in `8aab89d` reads `"live camera"`; make it the camera's `id`,
  which is now a real name the model gave.
- Delete the `defaultTopic ? ... : ""` empty-string branch and every fallback that produced a
  pane with no topic.

At the call site (`run.js:115`) nothing changes — `showRosCamera` returning early is what
"no pane" means, and `.videos` stays `hidden` because only `showRosCamera` unhides it on this
path.

**Check:** `reserveVideoSpace()` must not be left setting `--video-h`/`--video-w` for a pane
that never appears. Confirm the early return happens before `panel.hidden = false`.

---

## 5. Tests

Run existing suites first to know your baseline (§6). New tests:

### 5.1 Fixture

`src/motion-spec/tests/fixtures/perception/perception.robmot` imports
`admittance_arc_single.scenex`, whose scene declares **no camera**, so it cannot host this case.
`pick_place_single.scenex:167` declares `camera wrist`.

Add a fixture `src/motion-spec/tests/fixtures/camera_view/` — model + `.fsm` + `robot.toml`,
copying the shape of `tests/fixtures/perception/` — that imports
`../../../../motion-spec-dsl/models/pick_place_single/pick_place_single.scenex` and declares:

```
ros (ns=app) {
    subscribers {
        wrist-view: topic "/wrist/color" message "sensor_msgs/msg/Image" {
            observes { <wrist> }
        },
    },
}
```

Resolve the correct reference form for the camera by reading how `pick_place_single.robmot`
refers to its own scene sensors; `collab_real.robmot:10` uses the scene-qualified form
`<collab-scene-simreal.wrist_ft>`, and `collab_real.robmot:52` the bare `<wrist_ft>`. Use
whichever resolves; if neither does, STOP and report — the reference syntax is not this plan's
to invent.

### 5.2 `src/motion-spec-dsl/tests` — grammar and validation

Follow the file naming and style already in that directory. Cases:

- a camera subscription parses with no `pose from` clause;
- a pose subscription still parses and still requires `pose from`;
- a pose subscription **without** `pose from` is rejected;
- a camera subscription **with** `pose from` is rejected;
- two subscriptions naming one camera are rejected;
- a subscription observing a force-torque sensor is rejected;
- a subscription mixing a pose and a camera is rejected.

### 5.3 `src/motion-spec/tests/test_subscription_ir.py`

This file builds topic nodes by hand (`_topic(graph, *observes)` at line ~43) — extend that
helper or add a sibling that types the target as a camera. Cases:

- a topic whose only feature of interest is a camera yields **no** rows from
  `perceived_written_poses` and **no** entry from `ros_subscriptions`;
- a topic observing a pose is unaffected (regression);
- a topic observing both is not something the graph can carry past validation, so do not test
  it here.

### 5.4 `src/motion-spec/tests/test_camera_ir.py`

Follow `test_declared_camera_lowers_with_its_authored_render_settings` (line ~34), which shells
out to `textx generate` and reads `composition.scene.cameras`. Cases:

- the `camera_view` fixture's camera carries `topic == "/wrist/color"` and
  `message == "sensor_msgs/msg/Image"`;
- `pick_place_single`'s camera, which nothing subscribes to, carries `topic is None`.

### 5.5 `src/motion-spec/tests/test_dashboard_*.py`

`generation_cameras` passes `topic`/`message` through, and a contract camera without a topic
comes back with `topic: None`. Put it wherever `generation_cameras` is already covered — find
it with `grep -rn generation_cameras tests/`.

There is no JS test harness; `run.js` is verified by §6.3.

---

## 6. Verification

### 6.1 Suites

```bash
cd /home/batsy/work/ms
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest src/motion-spec-dsl/tests -q
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest src/motion-spec/tests -q
```

Both green. `src/motion-spec/tests` was **447 passed, 13 skipped** at `8aab89d`; a lower passed
count means something was dropped, not fixed.

Format every touched Python file with the workspace ruff before finishing:

```bash
/home/batsy/work/ms/.venv/bin/ruff format <files>
/home/batsy/work/ms/.venv/bin/ruff check <files>
```

`cli.py` has two pre-existing `ruff check` findings (ISC004 line 209, PLW1510 line 535). Do not
fix them; they are not this plan's.

### 6.2 Three-model CLI gate

Per `plans/TEMPLATE-pipeline-feature.md` §5, all three maintained models must still generate,
check, build and **run to completion reaching `s_done`** — no `--steps` bound. Use a fresh
sibling generation directory, never the checked-in one:

```bash
cd /home/batsy/work/ms/src/motion-spec-dsl/models/pick_place_single
motion-spec gen code pick_place_single.robmot -o plan002_generation
motion-spec check plan002_generation/generated/model/pick_place_single-app.ld.json
motion-spec build plan002_generation --prefix "$INSTALL"
motion-spec run plan002_generation --cwd "$GRC_WS"
```

…and the same for `pick_place_dual` and `admittance_arc_single`.

`pick_place_single` is the one that matters most here: it has a camera and no subscription, so
it proves the no-provider path lowers cleanly and still records video in simulation.

**Then inspect the generated controller for the fixture:**

```bash
grep -rn "/wrist/color" plan002_generation/generated/controller/ || echo "no C++ subscriber — correct"
```

The topic string must not appear. If it does, §4.5's reasoning is wrong — STOP.

### 6.3 The run page

Per `CLAUDE.md`: run GUI examples in the normal session environment — `source setup-grc.zsh`
first, never `env -i`, never hand-set `DISPLAY`.

The live pane only renders when the generation reports `simulated: false`, so a simulated model
cannot exercise it. Two things to see:

1. **A real-world generation with a declared provider** shows the pane, and the topic field is
   prefilled with the model's topic — not `/<id>/color`.
2. **A real-world generation without one** shows no pane at all, and the page below is not left
   with a reserved gap where it would have been (check `--video-h` / `--video-w` are `0px` in
   the inspector).

The existing real-world generations under `/home/batsy/work/ms/generations/collab_real/` have
**0 runs**, so opening a run page for them is not possible without starting one. If no suitable
run exists, say so in the report rather than fabricating one.

---

## 7. Files in scope / out of scope

**In scope**

| Repo | File |
|---|---|
| motion-spec-dsl | `src/motion_spec_dsl/grammars/model.tx` |
| motion-spec-dsl | `src/motion_spec_dsl/classes/ros.py` |
| motion-spec-dsl | `src/motion_spec_dsl/classes/validation/detects.py` |
| motion-spec-dsl | `src/motion_spec_dsl/rdf/motion_spec.py` |
| motion-spec-dsl | `tests/` (new cases) |
| motion-spec | `src/motion_spec/rdf_parser/quantities.py` |
| motion-spec | `src/motion_spec/rdf_parser/resources.py` |
| motion-spec | `src/motion_spec/generation/artifacts.py` |
| motion-spec | `src/motion_spec/dashboard/catalog.py` |
| motion-spec | `src/motion_spec/dashboard/frontend/run.js` |
| motion-spec | `tests/fixtures/camera_view/`, `tests/test_subscription_ir.py`, `tests/test_camera_ir.py`, the dashboard test covering `generation_cameras` |

**Out of scope — do not touch**

- `src/bdd_collab_bhv_cpp/**` — **not our repo.** `collab_real.robmot` is the model that would
  most obviously gain a `wrist-view` subscription, and `collab.scenex:188` declares the camera
  it would name. Editing it needs its own approval. Write the one-paragraph diff you *would*
  apply into the final report and stop there.
- `src/motion_spec/dashboard/ros_camera.py` and the `/api/ros-camera` route — the topic is
  already a parameter.
- `src/motion_spec/rdf_parser/communication.py` `ros_subscriptions` — §4.5 makes it correct
  without an edit. If you find yourself adding a camera branch there, STOP.
- `src/motion_spec/templates/ros.stg` — no camera subscription reaches it.
- The recorded-video pane (`showVideos`) and the minimize/expand toggles.
- `src/scene-dsl/**` — the camera declaration is already sufficient; do not add `observes:` to
  `CameraSensorSpec`.

---

## 8. STOP conditions

Stop and report rather than deciding any of these yourself:

1. **A new RDF term looks necessary.** Never mint one. Name the closest existing candidates and
   why each fails.
2. **`CameraSensorSpec` has no `.uri`**, so `_emit_ros_subscription` cannot link to it. Do not
   construct the URI by string-building.
3. **The camera reference `<wrist>` / `<scene.wrist>` does not resolve** from a `.robmot`
   `subscribers` block. The reference syntax is not this plan's to invent.
4. **`textx generate` fails on the widened grammar** with a resolution error rather than a parse
   error — that means `OBJECT` does not behave as `DeviceBinding` implies, and §4.1 needs a
   different mechanism.
5. **The camera's topic appears in `generated/controller/`** after §6.2 — the subscription is
   reaching codegen through a path this plan did not find.
6. **Any of the three maintained models fails to reach `s_done`.** A truncated or errored run is
   a failure, not a flake — report the output.
7. **`perceived_written_poses` has a second caller** that also needs the camera skip. `grep -rn
   perceived_written_poses src/` before editing; if there is one beyond `ros_subscriptions` and
   the detect path, say so before changing shared behaviour.

---

## 9. Maintenance note

The load-bearing subtlety is that **a camera subscription is defined by absence**: it has no
pose path, and it produces no rows from `perceived_written_poses`, which is the only reason it
generates no C++. Anything that later makes `ros_subscriptions` iterate topics directly rather
than through `written` will silently start generating an image subscriber into the control
loop. If that refactor happens, it needs an explicit "skip camera providers" test.

The second is that `sosa:hasFeatureOfInterest` is now overloaded: on one topic it means "the
pose this channel writes", on another "the camera this channel carries". Every new reader of
that predicate must branch on the target's type. This was a deliberate choice; a future reader
wondering why should be pointed here rather than to a graph bug.
