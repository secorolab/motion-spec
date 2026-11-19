# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""The compiler's per-motion unit and everything scoped to one motion: its slice of a solver,
its snapshot captures, its regrouped pose errors, and the data and publish records a motion's
schedule can reach.
"""

from __future__ import annotations

from dataclasses import dataclass, field

from motion_spec.classes.base import INTERNAL
from motion_spec.classes.handlers import ConstraintEvaluator, Controller, Monitor
from motion_spec.classes.qudt import Quantity, QuantityKind, Unit
from motion_spec.classes.solvers import MotionDrivers


@dataclass(eq=False)
class DataValue:
    """A D-block of the algorithm that no model entity declares: the measured period, the FT tare
    state, a controller's internal state, a control parameter, a joint-space mirror.

    The algorithm data states what it is -- its id, its storage type, its initial value and the
    role it plays. What it was derived from is provenance's to report, so the descriptive fields
    are construction inputs that `communication.py` reads to build the row.
    """

    id: str
    type: str
    value: float | None = None
    unset: bool = False
    role: str | None = None
    writer: dict | None = field(default=None, metadata=INTERNAL)
    quantity_kind: QuantityKind | None = field(default=None, metadata=INTERNAL)
    unit: Unit | None = field(default=None, metadata=INTERNAL)
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
    world_slot: int = 0
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
    is_pose: bool = True
    is_twist: bool = False
    is_wrench: bool = False
    type: str = field(default="PoseErrorRegroup")


@dataclass
class SnapshotCapture:
    """A sample-and-hold capture of a fluent, and when it is taken.

    `scope` is the declaration site made operative: a snapshot a motion declares is captured
    at the start of each of its activations, one the shared context declares with no trigger is
    captured once for the run, and one naming a trigger -- wherever it is declared -- is
    re-captured on every occurrence of that event.

    `captured_id` names the run-scoped latch guarding the once-for-the-run capture -- a shared
    value like any other, so whoever reads it is recorded rather than invented by a template.
    """

    target_id: str
    source_id: str
    scope: str = "entry"
    captured_id: str | None = None
    source_function_id: str | None = None
    trigger_event: str | None = None
    fsm_namespace: str | None = None
    type: str = field(default="SnapshotCapture")


@dataclass
class MotionSolverSlice:
    """A per-motion slice of a solver: what is genuinely per-motion, nothing copied off the
    solver. Everything else -- `algorithm`, `gravity`, `chain.root`, `torque_saturation` --
    is reached through `solver_id` into `resources.by_id`.
    """

    id: str  # the solver's id: `robot.<id>_state` names the shared state
    solver_id: str
    output: list
    motion_driver: MotionDrivers
    read_only: bool = False
    command_mode: str | None = None
    gripper_joint_outputs: list = field(default_factory=list)
    joint_space_samples: list = field(default_factory=list)
    joint_space_cmd_samples: list = field(default_factory=list)
    required_sensors: list[str] = field(default_factory=list)
    type: str = field(default="MotionSolverSlice")


@dataclass
class MotionUnit:
    """A fully built motion unit: schedules, monitors, controllers, conditions, solvers and
    codegen flags. The compiler's per-handler unit -- `mot:GuardedMotion` is one row above it,
    and several handlers may realize the same one, each with its own gains, solvers and state.
    """

    # handler id, one unit per handler; motion_id is not unique
    id: str
    motion_id: str = field(metadata=INTERNAL)
    name: str
    description: list[str]
    when_evaluators: list[ConstraintEvaluator] = field(metadata=INTERNAL)
    while_evaluators: list[ConstraintEvaluator] = field(metadata=INTERNAL)
    until_evaluators: list[ConstraintEvaluator] = field(metadata=INTERNAL)
    controllers: list[Controller]
    when_monitors: list[Monitor]
    while_monitors: list[Monitor]
    until_monitors: list[Monitor]
    when_schedule: list[str]
    while_schedule: list[str]
    until_schedule: list[str]

    # runs before while_schedule: feeds the inline pose-axis error groups
    while_pre_schedule: list[str] = field(default_factory=list)
    has_elapsed: bool = field(default=False, metadata=INTERNAL)
    active_elapsed_ids: list[str] = field(default_factory=list)
    when_elapsed_ids: list[str] = field(default_factory=list)
    when_observation_ages: list[dict] = field(default_factory=list)
    active_observation_ages: list[dict] = field(default_factory=list)
    has_until_condition: bool = field(default=False, metadata=INTERNAL)
    path_projections: list[dict] = field(default_factory=list)
    declared_pose_components: list = field(default_factory=list)
    when_needs_state: bool = False
    when_needs_data: bool = False
    when_needs_robot: bool = False
    until_needs_state: bool = False
    until_needs_data: bool = False
    until_needs_robot: bool = False
    apply_needs_state: bool = False
    apply_needs_data: bool = False
    apply_needs_robot: bool = False
    perturbations: list = field(default_factory=list)
    when_needs_events: bool = False
    until_needs_events: bool = False
    control_needs_events: bool = False
    step_needs_events: bool = False
    fsm_state: str | None = None
    runs_in_end_state: bool = False
    runs_in_state: str = field(default="", metadata=INTERNAL)
    fsm_when_gate_motions: list = field(default_factory=list)
    fsm_when_gate_calls: list = field(default_factory=list)
    has_when_gate: bool = False
    when_gate_hold: dict | None = None
    is_when_gate_hold: bool = False
    serial_chain_solvers: list[MotionSolverSlice] = field(default_factory=list)

    snapshots: list[SnapshotCapture] = field(default_factory=list)
    # split by scope: ST4 cannot filter
    entry_snapshots: list[SnapshotCapture] = field(default_factory=list)
    task_snapshots: list[SnapshotCapture] = field(default_factory=list)

    pose_axis_error_groups: list[PoseErrorRegroup] = field(default_factory=list)

    forwarded_commands: list[ForwardedCommandStep] = field(default_factory=list)

    action_clients: list = field(default_factory=list)
    reentry_events: list = field(default_factory=list)

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
