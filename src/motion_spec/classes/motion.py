# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""The compiler's per-motion unit and everything scoped to one motion: its slice of a solver,
its snapshot captures, its regrouped pose errors, and the blackboard/publish records a motion's
schedule can reach.
"""

from __future__ import annotations

from dataclasses import dataclass, field

from motion_spec.classes.base import INTERNAL
from motion_spec.classes.handlers import ConstraintEvaluator, Controller, Monitor
from motion_spec.classes.qudt import Quantity, QuantityKind, Unit
from motion_spec.classes.solvers import MotionDrivers


@dataclass(eq=False)
class BlackboardValue:
    """A shared value the runtime writes that no model entity declares: the measured period, the
    FT tare state, a controller's internal state, a control parameter, a joint-space mirror.

    The blackboard publishes what a member is -- its id, its storage type, its initial value and
    the role it plays. What it was derived from is the introspection artifact's to report, so the
    descriptive fields are construction inputs that `communication.py` reads to build the row.
    """

    id: str
    type: str
    value: float | None = None
    role: str | None = None
    producer: dict | None = field(default=None, metadata=INTERNAL)
    quantity_kind: QuantityKind | None = field(default=None, metadata=INTERNAL)
    unit: Unit | None = field(default=None, metadata=INTERNAL)
    # Who the value belongs to, named the way its introspection row names it.
    owner: str | None = field(default=None, metadata=INTERNAL)
    parameter: str | None = field(default=None, metadata=INTERNAL)
    controller: str | None = field(default=None, metadata=INTERNAL)
    state: str | None = field(default=None, metadata=INTERNAL)
    runtime: str | None = field(default=None, metadata=INTERNAL)
    channel: str | None = field(default=None, metadata=INTERNAL)
    joint: str | None = field(default=None, metadata=INTERNAL)


@dataclass
class ForwardedCommandStep:
    """A robot command forwarded directly from a controller output."""

    id: str
    control_signal: Quantity
    target: str
    robot_id: str
    type: str = field(default="ForwardedCommandStep")


@dataclass
class PoseErrorComponent:
    """One per-axis component of a grouped pose error."""

    quantity: str
    error: str
    reference: str
    subspace: str
    axis: str
    eval_id: str
    type: str = field(default="PoseErrorComponent")


@dataclass
class PoseErrorRegroup:
    """Per-axis pose-error scalars regrouped into a single pose error."""

    id: str
    pose: str
    components: list[PoseErrorComponent]
    superobject_type: str = "Pose"
    has_angular: bool = False
    linear_x: str | None = None
    linear_y: str | None = None
    linear_z: str | None = None
    angular_x: str | None = None
    angular_y: str | None = None
    angular_z: str | None = None
    # Superobject-type flags folded from superobject_type (ST4 branch selectors).
    is_pose: bool = True
    is_twist: bool = False
    is_wrench: bool = False
    type: str = field(default="PoseErrorRegroup")


@dataclass
class SnapshotCapture:
    """A sample-and-hold capture of a fluent: always sampled once when its motion first
    runs, and re-sampled on every occurrence of `trigger_event` when one is declared.
    """

    target_id: str
    source_id: str
    source_closure_id: str | None = None
    trigger_event: str | None = None
    fsm_namespace: str | None = None
    type: str = field(default="SnapshotCapture")


@dataclass
class SceneRelativePose:
    """Continuous relative pose of an FK frame with respect to a scene-object body."""

    id: str
    fk_pose_id: str
    scene_pose_id: str
    base_seen: bool = False
    type: str = field(default="SceneRelativePose")


@dataclass
class RelativePoseCapture:
    """A pose captured relative to its start frame."""

    id: str
    fk_pose_id: str
    type: str = field(default="RelativePoseCapture")


@dataclass
class MotionSolverSlice:
    """A per-motion slice of a solver: what is genuinely per-motion, nothing copied off the
    solver. Everything else -- `algorithm`, `gravity`, `chain.root`, `torque_saturation` --
    is reached through `solver_id` into `resources.by_id`.
    """

    id: str  # the solver's id, so `state.<id>` keeps working
    solver_id: str  # what to look up for everything the solver owns
    output: list
    motion_driver: MotionDrivers
    read_only: bool = False
    gripper_joint_outputs: list = field(default_factory=list)
    joint_space_samples: list = field(default_factory=list)
    joint_space_cmd_samples: list = field(default_factory=list)
    type: str = field(default="MotionSolverSlice")


@dataclass
class MotionUnit:
    """A fully built motion unit: schedules, monitors, controllers, conditions, solvers and
    codegen flags. The compiler's per-motion unit -- `mot:GuardedMotion` is one row above it.
    """

    id: str
    handler: str = field(metadata=INTERNAL)
    name: str
    # Authored description split into lines: the doc comment it renders into is a per-line
    # construct, so the split belongs to the IR rather than to an escape in the renderer.
    description: list[str]

    # Evaluators
    when_evaluators: list[ConstraintEvaluator] = field(metadata=INTERNAL)
    while_evaluators: list[ConstraintEvaluator] = field(metadata=INTERNAL)
    until_evaluators: list[ConstraintEvaluator] = field(metadata=INTERNAL)

    # Controllers and Monitors
    controllers: list[Controller]
    when_monitors: list[Monitor]
    while_monitors: list[Monitor]
    until_monitors: list[Monitor]
    # Schedules: when_schedule runs in can_start (own Parser). while_/until_schedule are slices of
    # one shared active graph, so they share a Parser -- common steps emit once and dedup correctly.
    when_schedule: list[str]
    while_schedule: list[str]
    until_schedule: list[str]

    # Producers feeding a pose-axis error group's reference. The group emits its error inline
    # ahead of while_schedule, so these must run before it, not with the controllers.
    while_pre_schedule: list[str] = field(default_factory=list)
    has_elapsed: bool = field(default=False, metadata=INTERNAL)
    has_when_elapsed: bool = False
    has_active_elapsed: bool = False
    # The elapsed-duration coordinates this motion measures, per phase: the authored shared
    # value each timing constraint compares against, filled from the phase's start time.
    active_elapsed_ids: list[str] = field(default_factory=list)
    when_elapsed_ids: list[str] = field(default_factory=list)
    has_until_condition: bool = field(default=False, metadata=INTERNAL)
    # Derived join: the when phase is one disjunction.
    when_any: bool = False
    # Structured boolean terms (folded from evaluators/monitors); rendered to C++ by the
    # bool-condition template, joined by when_any. The *_present flag gates the
    # empty-default (JSON empty lists are truthy in the ST4 build).
    when_terms: list = field(default_factory=list)
    when_terms_present: bool = False
    path_projections: list[dict] = field(default_factory=list)
    # Declared pose components referenced by this motion (folded from pose_components).
    declared_pose_components: list = field(default_factory=list)
    # Per-function capability booleans (which context objects each generated function
    # needs). The C++ signatures/args are built from these by the sig-params/sig-args
    # templates (folded from schedules/monitors/solvers).
    can_start_needs_state: bool = False
    can_start_needs_shared: bool = False
    can_start_needs_robot: bool = False
    when_needs_state: bool = False
    when_needs_shared: bool = False
    when_needs_robot: bool = False
    until_needs_state: bool = False
    until_needs_shared: bool = False
    until_needs_robot: bool = False
    monitor_needs_state: bool = False
    monitor_needs_shared: bool = False
    monitor_needs_robot: bool = False
    apply_needs_state: bool = False
    apply_needs_shared: bool = False
    apply_needs_robot: bool = False
    # Whether the function records into the coordination event buffer (holds an edge monitor).
    when_needs_events: bool = False
    until_needs_events: bool = False
    monitor_needs_events: bool = False
    control_needs_events: bool = False
    step_needs_events: bool = False
    # FSM wiring (folded from the FSM named graph): the state this motion runs in,
    # and the WHEN-gated motions this one holds for as a fallback.
    fsm_state: str | None = None
    # The state the model says runs this motion, when it says so rather than leaving it derived.
    runs_in_state: str = field(default="", metadata=INTERNAL)
    fsm_when_gate_motions: list = field(default_factory=list)
    fsm_when_gate_calls: list = field(default_factory=list)

    # Solver Integration
    serial_chain_solvers: list[MotionSolverSlice] = field(default_factory=list)

    # Initial sample-and-hold captures.
    snapshots: list[SnapshotCapture] = field(default_factory=list)

    # Relative-from-start pose computations (e.g. pose_start_ee)
    relative_poses: list[RelativePoseCapture] = field(default_factory=list)

    # Continuous relative pose of FK frame wrt scene object body (e.g. pose_ee_wrt_cube)
    scene_relative_poses: list[SceneRelativePose] = field(default_factory=list)

    # Pose coordinate-view scalar constraints grouped back into one KDL::diff pose error.
    pose_axis_error_groups: list[PoseErrorRegroup] = field(default_factory=list)

    # Direct robot command forwarding driven by FeedForward controllers.
    forwarded_commands: list[ForwardedCommandStep] = field(default_factory=list)

    # The action goals this motion sends on entry and cancels on exit. The presence flag gates
    # the exit block: JSON empty lists are truthy in the ST4 build.
    action_clients: list = field(default_factory=list)
    has_action_clients: bool = False
    # Events fired by self-transitions on this motion's state: consuming one re-enters the
    # motion, so entry runs again (snapshots re-capture, goals re-send).
    reentry_events: list = field(default_factory=list)
    has_reentry_events: bool = False

    # This motion's introspection index: the single index space the frame log's active_motion,
    # the generated sample switch and schema["by_motion"] all share.
    index: int = -1

    type: str = field(default="MotionUnit")


@dataclass
class ComponentRef:
    """One pose component: a literal value, or the id of a value the backend reads."""

    value: str | None = None
    ref: str | None = None


@dataclass
class PoseComponents:
    """A pose's components, typed and published in full: every axis field is present, `null`
    where the pose has no such axis (DECISION 10). Completeness is checked against
    `representation`, not against "every key is set" -- position always; `orientation_x/y/z/w`
    for `quaternion`; one `orientation_*` per character of the Euler sequence for `euler`; none
    for `relative`.
    """

    representation: str
    position_x: ComponentRef | None = None
    position_y: ComponentRef | None = None
    position_z: ComponentRef | None = None
    orientation_x: ComponentRef | None = None
    orientation_y: ComponentRef | None = None
    orientation_z: ComponentRef | None = None
    orientation_w: ComponentRef | None = None
    orientation_operands: list | None = None
    euler_factors: list | None = None
