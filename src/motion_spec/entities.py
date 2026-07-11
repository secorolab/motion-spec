# SPDX-License-Identifier: MPL-2.0
"""Entity dataclasses and enums for the motion-spec IR.

Pure data model: no RDF access, no parsing logic. The stateful Parser in
ir_gen.py constructs these from the graph; codegen and the emitter consume
them via the IR dict.
"""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass, field, is_dataclass
from enum import Enum


class DataclassJSONEncoder(json.JSONEncoder):
    """Encode IR dataclasses without importing the RDF parser."""

    def default(self, o):
        if is_dataclass(o) and not isinstance(o, type):
            return asdict(o)
        return super().default(o)


class Subspace(str, Enum):
    # The 6D subspace of a spatial quantity: its linear (translational) or angular
    # (rotational) half. Which physical quantity it belongs to is carried by the
    # View's superobject type (Pose/VelocityTwist/AccelerationTwist/Wrench/PoseDifference),
    # so the subspace only needs to name the half -- matching the C++ runtime Subspace enum.
    Linear = "Linear"
    Angular = "Angular"


class Axis(str, Enum):
    X = "X"
    Y = "Y"
    Z = "Z"


class ControlMode(str, Enum):
    JointTorque = "JointTorque"


class UnilateralConstraintType(str, Enum):
    GreaterThan = "GreaterThan"
    LessThan = "LessThan"


class EvaluatorType(str, Enum):
    AssignmentEvaluator = "AssignmentEvaluator"
    ErrorEvaluator = "ErrorEvaluator"


@dataclass
class Point:
    id: str
    type: str = field(default="Point")


@dataclass
class Frame:
    id: str
    is_scene_object: bool = False
    type: str = field(default="Frame")


@dataclass
class QuantityKind:
    id: str
    type: str = field(default="QuantityKind")


@dataclass
class Unit:
    id: str
    type: str = field(default="Unit")


@dataclass
class Provenance:
    """Origin/role of a quantity's value, shared by all quantity-like entities.

    Not intrinsic to the physical quantity: whether the value was authored by the
    user (vs computed) and whether it is a runtime snapshot capture.
    """

    authored: bool = False
    snapshot: bool = False


@dataclass
class Quantity:
    id: str
    quantity_kind: QuantityKind
    unit: Unit
    value: float | None
    has_view: bool
    provenance: Provenance = field(default_factory=Provenance)
    reference_value: str | None = None
    type: str = field(default="Quantity")


@dataclass
class Saturation:
    id: str
    input_signal: Quantity
    output_signal: Quantity
    maximum: Quantity | None = None
    lower: Quantity | None = None
    upper: Quantity | None = None
    type: str = field(default="Saturation")


@dataclass
class FreeVector:
    id: str
    quantity_kind: QuantityKind
    unit: Unit
    vector: list[float] | None
    has_view: bool = False
    provenance: Provenance = field(default_factory=Provenance)
    type: str = field(default="FreeVector")


@dataclass
class Trajectory:
    id: str
    quantity_kind: QuantityKind
    unit: Unit
    has_view: bool
    provenance: Provenance = field(default_factory=Provenance)
    value_kind: str | None = None
    type: str = field(default="Trajectory")


@dataclass
class JointPosition:
    id: str
    joint_name: str
    type: str = field(default="JointPosition")


@dataclass
class SimplicialComplex:
    id: str
    is_scene_object: bool = False
    type: str = field(default="SimplicialComplex")


@dataclass
class SceneObject:
    id: str
    body: str = ""
    is_scene_object: bool = True
    type: str = field(default="SceneObject")


@dataclass
class Direction:
    id: str
    quantity_kind: list[QuantityKind]
    as_seen_by: Frame
    unit: list[Unit]
    direction: list[float] | None
    type: str = field(default="Direction")


@dataclass
class Position:
    id: str
    of: Point | None
    with_respect_to: Point | None
    quantity_kind: QuantityKind
    as_seen_by: Frame
    unit: Unit
    position: list[float] | None
    type: str = field(default="Position")


@dataclass
class Orientation:
    id: str
    of: Frame | SceneObject | None
    with_respect_to: Frame | SceneObject | None
    quantity_kind: QuantityKind
    as_seen_by: Frame | None
    unit: Unit
    euler_axes_sequence: str | None = None
    has_view: bool = False
    provenance: Provenance = field(default_factory=Provenance)
    type: str = field(default="Orientation")


@dataclass
class Pose:
    id: str
    of: SimplicialComplex | Frame | SceneObject | None
    with_respect_to: SimplicialComplex | Frame | None
    quantity_kind: list[QuantityKind]
    as_seen_by: Frame | None
    unit: list[Unit]
    direction_cosine_x: list[float] | None
    direction_cosine_y: list[float] | None
    direction_cosine_z: list[float] | None
    position: list[float] | None
    euler_axes_sequence: str | None = None
    provenance: Provenance = field(default_factory=Provenance)
    type: str = field(default="Pose")


@dataclass
class VelocityTwist:
    id: str
    of: SimplicialComplex
    with_respect_to: SimplicialComplex
    quantity_kind: list[QuantityKind]
    reference_point: Point
    as_seen_by: Frame
    unit: list[Unit]
    provenance: Provenance = field(default_factory=Provenance)
    type: str = field(default="VelocityTwist")


@dataclass
class AccelerationTwist:
    id: str
    quantity_kind: list[QuantityKind]
    reference_point: Point
    as_seen_by: Frame
    unit: list[Unit]
    provenance: Provenance = field(default_factory=Provenance)
    type: str = field(default="AccelerationTwist")


@dataclass
class PoseDifference:
    id: str
    quantity_kind: list[QuantityKind]
    reference_point: Point
    as_seen_by: Frame
    unit: list[Unit]
    provenance: Provenance = field(default_factory=Provenance)
    type: str = field(default="PoseDifference")


@dataclass
class Wrench:
    id: str
    quantity_kind: list[QuantityKind]
    reference_point: Point
    as_seen_by: Frame
    unit: list[Unit]
    provenance: Provenance = field(default_factory=Provenance)
    # Non-empty when this wrench is measured from a force/torque sensor (the FT-read
    # solver-output reads and tares this sensor into shared.<id>.force). Empty for
    # computed/commanded wrenches.
    sensor_name: str = ""
    type: str = field(default="Wrench")


@dataclass
class View:
    id: str
    superobject: Pose | VelocityTwist | AccelerationTwist | PoseDifference | Wrench
    subobject: Quantity
    subspace: Subspace
    axis: Axis | None
    type: str = field(default="View")


@dataclass
class EqualityConstraint:
    reference_value: Quantity
    type: str = field(default="EqualityConstraint")


@dataclass
class UnilateralConstraint:
    type_: UnilateralConstraintType
    threshold: Quantity
    type: str = field(default="UnilateralConstraint")


@dataclass
class BilateralConstraint:
    lower_threshold: Quantity
    upper_threshold: Quantity
    type: str = field(default="BilateralConstraint")


@dataclass
class OutsideConstraint:
    lower_threshold: Quantity
    upper_threshold: Quantity
    type: str = field(default="OutsideConstraint")


@dataclass
class Constraint:
    id: str
    quantity: Quantity
    parameter: EqualityConstraint | UnilateralConstraint | BilateralConstraint | OutsideConstraint
    type: str = field(default="Constraint")


@dataclass
class GuardedMotion:
    id: str
    when: list[Constraint]
    while_: list[Constraint]
    until: list[Constraint]
    until_any: bool = False
    when_any: bool = False
    type: str = field(default="GuardedMotion")


@dataclass
class ConstraintEvaluator:
    id: str
    type_: EvaluatorType
    constraint: Constraint
    error: Quantity | None
    is_elapsed: bool = False
    elapsed_op: str | None = None
    elapsed_threshold_s: float | None = None
    type: str = field(default="ConstraintEvaluator")


@dataclass
class PIDController:
    id: str
    control_signal: Quantity
    error_signal: Quantity | None = None
    measured_derivative: Quantity | None = None
    proportional_gain: float | None = None
    integral_gain: float | None = None
    derivative_gain: float | None = None
    decay_rate: float | None = None
    output_saturation: Saturation | None = None
    integral_saturation: Saturation | None = None
    type: str = "ProportionalIntegralDerivative"


@dataclass
class ImpedanceController:
    id: str
    control_signal: Quantity
    error_signal: Quantity | None = None
    integral_gain: float | None = None
    stiffness: float | None = None
    damping: float | None = None
    output_saturation: Saturation | None = None
    type: str = "ImpedanceController"


@dataclass
class FeedForwardController:
    id: str
    control_signal: Quantity
    reference_signal: Quantity | None = None
    output_saturation: Saturation | None = None
    type: str = "FeedForwardController"


Controller = PIDController | ImpedanceController | FeedForwardController


@dataclass
class ForwardedCommand:
    id: str
    control_signal: Quantity
    target: str
    type: str = field(default="ForwardedCommand")


@dataclass
class LevelMonitor:
    """A monitor that continuously sets a boolean flag from its constraint."""

    id: str
    monitor_type: str
    error: Quantity | None
    flag: str | None
    is_edge_triggered: bool = False
    is_until_aggregate: bool = False
    is_when_aggregate: bool = False
    debounce_steps: int | None = None
    active_condition: str | None = None
    type: str = field(default="LevelMonitor")


@dataclass
class EdgeMonitor:
    """A monitor that fires an FSM event on a rising edge of its constraint."""

    id: str
    monitor_type: str
    error: Quantity | None
    event: str | None
    event_idx: int | None
    is_edge_triggered: bool = True
    is_until_aggregate: bool = False
    is_when_aggregate: bool = False
    event_uri: str | None = None
    event_name: str | None = None
    fallback_motion: str | None = None
    # Authored debounce duration (s); converted to debounce_steps once the loop period is known.
    # Stays None (not 0) when absent -- ST4's <if(x)> is true even for integer 0.
    debounce_duration_s: float | None = None
    debounce_steps: int | None = None
    active_condition: str | None = None
    # FSM binding (folded when the monitor's event lives in the FSM namespace).
    fsm_namespace: str | None = None
    fsm_event_idx: int | None = None
    type: str = field(default="EdgeMonitor")


Monitor = LevelMonitor | EdgeMonitor


@dataclass
class PoseAxisErrorComponent:
    quantity: str
    error: str
    reference: str
    subspace: str
    axis: str
    eval_id: str
    type: str = field(default="PoseAxisErrorComponent")


@dataclass
class PoseAxisErrorGroup:
    id: str
    pose: str
    components: list[PoseAxisErrorComponent]
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
    type: str = field(default="PoseAxisErrorGroup")


@dataclass
class ConstraintHandler:
    id: str
    motion: GuardedMotion
    control_mode: str
    evaluators: list[ConstraintEvaluator]
    controllers: list[Controller]
    monitors: list[Monitor]
    order: int = 0
    type: str = field(default="ConstraintHandler")


@dataclass
class SnapshotCapture:
    target_id: str
    source_id: str
    source_closure_id: str | None = None
    # snap:sampled-on clock: "task" = sampled once, "entry" = re-sampled per entry.
    clock: str = "task"
    # persistent = shared-guarded so it survives the on-entry reset (see _snapshots_for_motion).
    persistent: bool = False
    type: str = field(default="SnapshotCapture")


@dataclass
class SceneRelativePose:
    id: str
    fk_pose_id: str
    scene_pose_id: str
    base_seen: bool = False
    type: str = field(default="SceneRelativePose")


@dataclass
class RelativePoseCapture:
    id: str
    fk_pose_id: str
    type: str = field(default="RelativePoseCapture")


@dataclass
class GuardedMotionBlock:
    id: str
    handler: str
    control_mode: str

    # Evaluators
    when_evaluators: list[ConstraintEvaluator]
    while_evaluators: list[ConstraintEvaluator]
    until_evaluators: list[ConstraintEvaluator]

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

    has_elapsed: bool = False
    has_when_elapsed: bool = False
    has_active_elapsed: bool = False
    has_until_condition: bool = False
    until_any: bool = False
    when_any: bool = False
    # Primary arm-solver id the motion commands (empty when the model has no arm).
    command_robot_id: str = ""
    # C++ boolean expressions (folded from the evaluators/monitors at build time).
    when_condition: str = "true"
    done_condition: str = "true"
    # Time-driven trajectory alpha ids (folded from while_schedule closures).
    time_trajectory_progress_ids: list = field(default_factory=list)
    # Declared pose components referenced by this motion (folded from pose_components).
    declared_pose_components: list = field(default_factory=list)
    # Generated C++ function signatures/args (folded from schedules/monitors/solvers).
    can_start_params: str = ""
    can_start_args: str = ""
    when_params: str = ""
    when_args: str = ""
    until_params: str = ""
    until_args: str = ""
    monitor_params: str = ""
    monitor_args: str = ""
    apply_params: str = ""
    apply_args: str = ""
    # FSM wiring (folded from the FSM named graph): the state this motion runs in,
    # and the WHEN-gated motions this one holds for as a fallback.
    fsm_state: str | None = None
    fsm_when_gate_motions: list = field(default_factory=list)
    fsm_when_gate_calls: list = field(default_factory=list)

    # Solver Integration
    arm_solvers: list = field(default_factory=list)

    # Snapshot captures (sample-and-hold of a fluent on a clock)
    snapshots: list = field(default_factory=list)

    # True iff any snapshot samples `on entry` -> motion is reset on FSM re-entry.
    has_entry_snapshot: bool = False

    # Relative-from-start pose computations (e.g. pose_start_ee)
    relative_poses: list = field(default_factory=list)

    # Continuous relative pose of FK frame wrt scene object body (e.g. pose_ee_wrt_cube)
    scene_relative_poses: list[SceneRelativePose] = field(default_factory=list)

    # Pose coordinate-view scalar constraints grouped back into one KDL::diff pose error.
    pose_axis_error_groups: list[PoseAxisErrorGroup] = field(default_factory=list)

    # Direct robot command forwarding driven by FeedForward controllers.
    forwarded_commands: list[ForwardedCommand] = field(default_factory=list)

    type: str = field(default="GuardedMotionBlock")


@dataclass
class AccelerationConstraint:
    id: str
    subspace: Subspace
    # Exactly one of axis (AxisAligned) / direction (DirectionAligned) is set.
    axis: Axis | None
    acceleration_energy: Quantity
    as_seen_by: Frame | None = None
    base_aligned: bool = True
    direction: "Direction | None" = None
    saturation: Saturation | None = None
    type: str = field(default="AccelerationConstraint")


@dataclass
class CartesianForceSpecification:
    id: str
    force: Wrench
    attached_to: SimplicialComplex
    type: str = field(default="CartesianForceSpecification")


@dataclass
class JointForceSpecification:
    id: str
    force_id: str
    joint_name: str
    type: str = field(default="JointForceSpecification")


@dataclass
class MotionDrivers:
    id: str
    acceleration_constraint: list[AccelerationConstraint]
    cartesian_force: list[CartesianForceSpecification]
    joint_force: list[JointForceSpecification] = field(default_factory=list)
    has_cartesian_force: bool = False
    type: str = field(default="MotionDrivers")


@dataclass
class HandlerArmSolver:
    id: str
    output: list
    motion_driver: MotionDrivers
    control_mode: str
    algorithm: str = ""
    algorithm_is_rne: bool = False
    root_acc: list[float] | None = None
    chain_root: str = ""
    chain_end: str = ""
    torque_saturation: Saturation | None = None
    type: str = field(default="HandlerArmSolver")


@dataclass
class SolverWithInputAndOutput:
    id: str
    motion_drivers: list[MotionDrivers]
    output: list
    algorithm: str = ""
    algorithm_is_rne: bool = False
    control_mode: str = ""
    urdf: str = ""
    chain_root: str = ""
    chain_end: str = ""
    chain_tip: str = ""
    robot_model: str = ""
    tool_body: str = ""
    tcp_site: str = ""
    ft_sensors: list[dict] = field(default_factory=list)
    root_acc: list[float] | None = None
    # DLS/Tikhonov regularization lambda, deduped across arm solvers into the
    # top-level IR key consumed by runtime_header.
    regularization: float | None = None
    torque_saturation: Saturation | None = None
    type: str = field(default="SolverWithInputAndOutput")


@dataclass
class SceneAttachment:
    id: str
    path: str
    attach_to: str
    attach_kind: str = "Body"
    prefix: str = ""
    pos: list[float] | None = None
    euler: list[float] | None = None
    actuator: str = ""
    type: str = field(default="SceneAttachment")


@dataclass
class SceneRobot:
    id: str
    path: str
    prefix: str = ""
    attach_kind: str = "World"
    attach_name: str = ""
    pos: list[float] | None = None
    euler: list[float] | None = None
    attachments: list[SceneAttachment] = field(default_factory=list)
    type: str = field(default="SceneRobot")


@dataclass
class SceneObjectSpec:
    id: str
    body: str
    path: str = ""
    attach_kind: str = "World"
    attach_name: str = ""
    pos: list[float] | None = None
    euler: list[float] | None = None
    fixed: bool = False
    shape: str | None = None
    size: list[float] | None = None
    color: list[float] | None = None
    mass: float | None = None
    friction: list[float] | None = None
    type: str = field(default="SceneObjectSpec")




@dataclass
class SceneSpec:
    robots: list[SceneRobot] = field(default_factory=list)
    objects: list[SceneObjectSpec] = field(default_factory=list)
    # Physics/control timestep from ENVIRONMENT.timestep; defaults to the backend
    # interval when the model omits it.
    timestep_s: float = 0.002
    type: str = field(default="SceneSpec")


@dataclass
class VelocityCompositionSolver:
    id: str
    configuration: str
    velocity: VelocityTwist
    type: str = field(default="VelocityCompositionSolver")


@dataclass
class ForceDistributionSolver:
    id: str
    configuration: str
    force: Wrench
    type: str = field(default="ForceDistributionSolver")
