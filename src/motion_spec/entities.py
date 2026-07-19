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
    """The linear (translational) or angular (rotational) half of a 6D spatial quantity.

    The physical quantity it belongs to is carried by the View's superobject type, so the
    subspace only names the half -- matching the C++ runtime Subspace enum.
    """

    Linear = "Linear"
    Angular = "Angular"


class Axis(str, Enum):
    """A Cartesian axis: X, Y or Z."""

    X = "X"
    Y = "Y"
    Z = "Z"


class UnilateralConstraintType(str, Enum):
    """Greater-than vs less-than kind of a unilateral constraint."""

    GreaterThan = "GreaterThan"
    LessThan = "LessThan"


class EvaluatorType(str, Enum):
    """Assignment vs error kind of a constraint evaluator."""

    AssignmentEvaluator = "AssignmentEvaluator"
    ErrorEvaluator = "ErrorEvaluator"


@dataclass
class Point:
    """A named point, such as a frame origin."""

    id: str
    type: str = field(default="Point")


@dataclass
class Frame:
    """A named reference frame (optionally backed by a scene object)."""

    id: str
    is_scene_object: bool = False
    type: str = field(default="Frame")


@dataclass
class QuantityKind:
    """A QUDT quantity kind."""

    id: str
    type: str = field(default="QuantityKind")


@dataclass
class Unit:
    """A QUDT unit."""

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
    """A scalar quantity with its kind, unit and optional value or view."""

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
    """Input/output saturation limits applied to a signal."""

    id: str
    input_signal: Quantity
    output_signal: Quantity
    maximum: Quantity | None = None
    lower: Quantity | None = None
    upper: Quantity | None = None
    type: str = field(default="Saturation")


@dataclass
class FreeVector:
    """A free (un-anchored) vector quantity."""

    id: str
    quantity_kind: QuantityKind
    unit: Unit
    vector: list[float] | None
    has_view: bool = False
    provenance: Provenance = field(default_factory=Provenance)
    type: str = field(default="FreeVector")


@dataclass
class Trajectory:
    """A trajectory-valued quantity."""

    id: str
    quantity_kind: QuantityKind
    unit: Unit
    has_view: bool
    provenance: Provenance = field(default_factory=Provenance)
    value_kind: str | None = None
    type: str = field(default="Trajectory")


@dataclass
class JointPosition:
    """A joint-position quantity for a named joint."""

    id: str
    joint_name: str
    type: str = field(default="JointPosition")


@dataclass
class SimplicialComplex:
    """A geometric body (optionally a scene object)."""

    id: str
    is_scene_object: bool = False
    type: str = field(default="SimplicialComplex")


@dataclass
class SceneObject:
    """A scene object referenced as a spatial endpoint."""

    id: str
    body: str = ""
    is_scene_object: bool = True
    type: str = field(default="SceneObject")


@dataclass
class Direction:
    """A unit-direction quantity."""

    id: str
    quantity_kind: list[QuantityKind]
    as_seen_by: Frame
    unit: list[Unit]
    direction: list[float] | None
    type: str = field(default="Direction")


@dataclass
class Position:
    """A position quantity of a point with respect to another."""

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
    """An orientation quantity of a frame/object with respect to another."""

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
    """A pose (position and orientation) quantity."""

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
    """A velocity-twist quantity."""

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
    """An acceleration-twist quantity."""

    id: str
    quantity_kind: list[QuantityKind]
    reference_point: Point
    as_seen_by: Frame
    unit: list[Unit]
    provenance: Provenance = field(default_factory=Provenance)
    type: str = field(default="AccelerationTwist")


@dataclass
class PoseDifference:
    """A pose-difference quantity."""

    id: str
    quantity_kind: list[QuantityKind]
    reference_point: Point
    as_seen_by: Frame
    unit: list[Unit]
    provenance: Provenance = field(default_factory=Provenance)
    type: str = field(default="PoseDifference")


@dataclass
class Wrench:
    """A wrench (force/torque) quantity, optionally read from a force/torque sensor."""

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
    """A scalar/axis view onto one subspace of a spatial superobject."""

    id: str
    superobject: Pose | VelocityTwist | AccelerationTwist | PoseDifference | Wrench
    subobject: Quantity
    subspace: Subspace
    axis: Axis | None
    type: str = field(default="View")


@dataclass
class EqualityConstraint:
    """Constraint parameter: equality to a reference value."""

    reference_value: Quantity
    type: str = field(default="EqualityConstraint")


@dataclass
class UnilateralConstraint:
    """Constraint parameter: a one-sided threshold."""

    type_: UnilateralConstraintType
    threshold: Quantity
    type: str = field(default="UnilateralConstraint")


@dataclass
class BilateralConstraint:
    """Constraint parameter: inside a lower/upper band."""

    lower_threshold: Quantity
    upper_threshold: Quantity
    type: str = field(default="BilateralConstraint")


@dataclass
class OutsideConstraint:
    """Constraint parameter: outside a lower/upper band."""

    lower_threshold: Quantity
    upper_threshold: Quantity
    type: str = field(default="OutsideConstraint")


@dataclass
class Constraint:
    """A constraint on a quantity together with its parameter."""

    id: str
    quantity: Quantity
    parameter: EqualityConstraint | UnilateralConstraint | BilateralConstraint | OutsideConstraint
    type: str = field(default="Constraint")


@dataclass
class GuardedMotion:
    """A guarded motion: its when/while/until constraint sets."""

    id: str
    when: list[Constraint]
    while_: list[Constraint]
    until: list[Constraint]
    until_any: bool = False
    when_any: bool = False
    type: str = field(default="GuardedMotion")


@dataclass
class ConstraintEvaluator:
    """Evaluates a constraint into an error signal (or an elapsed-timing predicate)."""

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
    """A proportional-integral-derivative controller."""

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
    # Abstract signal ids folded from the error-evaluator closure; the C++ access
    # expression is rendered backend-side by access-expr (shared_data.stg).
    measured_signal: str | None = None
    setpoint_signal: str | None = None
    type: str = "ProportionalIntegralDerivative"


@dataclass
class ImpedanceController:
    """An impedance controller."""

    id: str
    control_signal: Quantity
    error_signal: Quantity | None = None
    integral_gain: float | None = None
    stiffness: float | None = None
    damping: float | None = None
    output_saturation: Saturation | None = None
    measured_signal: str | None = None
    setpoint_signal: str | None = None
    type: str = "ImpedanceController"


@dataclass
class FeedForwardController:
    """A feed-forward controller."""

    id: str
    control_signal: Quantity
    reference_signal: Quantity | None = None
    output_saturation: Saturation | None = None
    measured_signal: str | None = None
    setpoint_signal: str | None = None
    type: str = "FeedForwardController"


Controller = PIDController | ImpedanceController | FeedForwardController


@dataclass
class ForwardedCommand:
    """A robot command forwarded directly from a controller output."""

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
    # Structured active-phase boolean terms (rendered to C++ by the bool-condition template).
    has_active: bool = False
    active_terms: list | None = None
    active_terms_present: bool = False
    active_any: bool = False
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
    # Structured active-phase boolean terms (rendered to C++ by the bool-condition template).
    has_active: bool = False
    active_terms: list | None = None
    active_terms_present: bool = False
    active_any: bool = False
    # FSM binding (folded when the monitor's event lives in the FSM namespace).
    fsm_namespace: str | None = None
    fsm_event_idx: int | None = None
    type: str = field(default="EdgeMonitor")


Monitor = LevelMonitor | EdgeMonitor


@dataclass
class PoseAxisErrorComponent:
    """One per-axis component of a grouped pose error."""

    quantity: str
    error: str
    reference: str
    subspace: str
    axis: str
    eval_id: str
    type: str = field(default="PoseAxisErrorComponent")


@dataclass
class PoseAxisErrorGroup:
    """Per-axis pose-error scalars regrouped into a single pose error."""

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
    """Binds a motion to its evaluators, controllers and monitors."""

    id: str
    motion: GuardedMotion
    evaluators: list[ConstraintEvaluator]
    controllers: list[Controller]
    monitors: list[Monitor]
    order: int = 0
    type: str = field(default="ConstraintHandler")


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
class GuardedMotionBlock:
    """A fully built motion unit: schedules, monitors, controllers, conditions, solvers and codegen flags."""

    id: str
    handler: str

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
    # Structured boolean terms (folded from evaluators/monitors); rendered to C++ by the
    # bool-condition template. WHEN joins with when_any, done joins with until_any. The
    # *_present flags gate the empty-default (JSON empty lists are truthy in the ST4 build).
    when_terms: list = field(default_factory=list)
    when_terms_present: bool = False
    done_terms: list = field(default_factory=list)
    done_terms_present: bool = False
    # Time-driven trajectory alpha ids (folded from while_schedule closures).
    time_trajectory_progress_ids: list = field(default_factory=list)
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
    # FSM wiring (folded from the FSM named graph): the state this motion runs in,
    # and the WHEN-gated motions this one holds for as a fallback.
    fsm_state: str | None = None
    fsm_when_gate_motions: list = field(default_factory=list)
    fsm_when_gate_calls: list = field(default_factory=list)

    # Solver Integration
    arm_solvers: list = field(default_factory=list)

    # Initial sample-and-hold captures.
    snapshots: list = field(default_factory=list)

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
    """An acceleration constraint (axis- or direction-aligned) on a solver."""

    id: str
    subspace: Subspace
    # Axis-aligned constraints carry an axis; derived constraint forms may leave it unset.
    axis: Axis | None
    acceleration_energy: Quantity
    as_seen_by: Frame | None = None
    base_aligned: bool = True
    direction: "Direction | None" = None
    saturation: Saturation | None = None
    type: str = field(default="AccelerationConstraint")


@dataclass
class CartesianForceSpecification:
    """A Cartesian force applied to a body."""

    id: str
    force: Wrench
    attached_to: SimplicialComplex
    type: str = field(default="CartesianForceSpecification")


@dataclass
class JointForceSpecification:
    """A joint-space force for a named joint."""

    id: str
    force_id: str
    joint_name: str
    type: str = field(default="JointForceSpecification")


@dataclass
class MotionDrivers:
    """The acceleration/Cartesian/joint force drivers of a solver."""

    id: str
    acceleration_constraint: list[AccelerationConstraint]
    cartesian_force: list[CartesianForceSpecification]
    joint_force: list[JointForceSpecification] = field(default_factory=list)
    has_cartesian_force: bool = False
    type: str = field(default="MotionDrivers")


@dataclass
class HandlerArmSolver:
    """An arm solver sliced to a single handler's motion driver."""

    id: str
    output: list
    motion_driver: MotionDrivers
    algorithm: str = ""
    algorithm_is_rne: bool = False
    root_acc: list[float] | None = None
    chain_root: str = ""
    chain_end: str = ""
    torque_saturation: Saturation | None = None
    runtime_id: str = ""
    runtime_owner: bool = False
    type: str = field(default="HandlerArmSolver")


@dataclass
class SolverWithInputAndOutput:
    """A full arm solver: chain, algorithm, drivers and outputs."""

    id: str
    motion_drivers: list[MotionDrivers]
    output: list
    algorithm: str = ""
    algorithm_is_rne: bool = False
    urdf: str = ""
    chain_root: str = ""
    chain_end: str = ""
    chain_tip: str = ""
    robot_model: str = ""
    tool_body: str = ""
    tcp_site: str = ""
    ft_sensors: list[dict] = field(default_factory=list)
    root_acc: list[float] | None = None
    torque_saturation: Saturation | None = None
    runtime_id: str = ""
    runtime_owner: bool = False
    type: str = field(default="SolverWithInputAndOutput")


@dataclass
class SceneAttachment:
    """An asset attached to a robot or object in the scene."""

    id: str
    path: str
    attach_to: str
    attach_kind: str = "Body"
    prefix: str = ""
    pos: list[float] | None = None
    euler: list[float] | None = None
    actuator: str = ""
    pos_x: float | None = None
    pos_y: float | None = None
    pos_z: float | None = None
    euler_x: float | None = None
    euler_y: float | None = None
    euler_z: float | None = None
    type: str = field(default="SceneAttachment")


@dataclass
class SceneRobot:
    """A robot placed in the scene, with its attachments."""

    id: str
    path: str
    prefix: str = ""
    attach_kind: str = "World"
    attach_name: str = ""
    pos: list[float] | None = None
    euler: list[float] | None = None
    attachments: list[SceneAttachment] = field(default_factory=list)
    pos_x: float | None = None
    pos_y: float | None = None
    pos_z: float | None = None
    euler_x: float | None = None
    euler_y: float | None = None
    euler_z: float | None = None
    type: str = field(default="SceneRobot")


@dataclass
class SceneObjectSpec:
    """A scene object's placement and (procedural or asset) geometry."""

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
    # Folded scalar expansions (pos/euler always; geometry only for non-path objects).
    pos_x: float | None = None
    pos_y: float | None = None
    pos_z: float | None = None
    euler_x: float | None = None
    euler_y: float | None = None
    euler_z: float | None = None
    has_path: bool = False
    size_x: float | None = None
    size_y: float | None = None
    size_z: float | None = None
    color_r: float | None = None
    color_g: float | None = None
    color_b: float | None = None
    color_a: float | None = None
    friction_slide: float | None = None
    friction_torsion: float | None = None
    friction_roll: float | None = None
    type: str = field(default="SceneObjectSpec")


@dataclass
class SceneSpec:
    """The scene: robots, objects and the control timestep."""

    robots: list[SceneRobot] = field(default_factory=list)
    objects: list[SceneObjectSpec] = field(default_factory=list)
    # Physics/control timestep from ENVIRONMENT.timestep; defaults to the backend
    # interval when the model omits it.
    timestep_s: float = 0.002
    type: str = field(default="SceneSpec")


@dataclass
class VelocityCompositionSolver:
    """A base velocity-composition solver."""

    id: str
    configuration: str
    velocity: VelocityTwist
    type: str = field(default="VelocityCompositionSolver")


@dataclass
class ForceDistributionSolver:
    """A base force-distribution solver."""

    id: str
    configuration: str
    force: Wrench
    type: str = field(default="ForceDistributionSolver")
