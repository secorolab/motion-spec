# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
# Author: Vamsi Kalagaturu
"""RDF namespaces used while parsing motion-spec manifests and models."""
from rdflib import URIRef
from rdflib.namespace import DefinedNamespace, Namespace
from rdf_utils.namespace import (
    URL_COMP_ROB2B,
    URL_SECORO_MM,
    NS_MM_QUDT,
    NS_MM_QUDT_QTY,
    NS_MM_GEOM,
    NS_MM_GEOM_REL,
    NS_MM_GEOM_COORD,
    NS_MM_KC,
    NS_MM_KC_STAT,
    NS_MM_EL,
    NS_MM_ENV,
    NS_MM_DYN_ENT,
    NS_MM_DYN_COORD,
)

URI_CR2B_MM = f"{URL_COMP_ROB2B}/metamodels"
URI_SECORO_MM = URL_SECORO_MM


class SOSA(DefinedNamespace):
    madeBySensor: URIRef
    hosts: URIRef

    _NS = Namespace("http://www.w3.org/ns/sosa/")


class SENSORS(DefinedNamespace):
    ForceTorqueSensor: URIRef
    frame: URIRef

    _NS = Namespace(f"{URI_SECORO_MM}/robot/sensors#")


class APP(DefinedNamespace):
    path: URIRef

    _extras = ["command-type", "constraints", "import", "iri-map", "order"]

    _NS = Namespace(f"{URI_CR2B_MM}/application/")


class ENV(DefinedNamespace):
    Object: URIRef
    Workspace: URIRef
    ModelledObject: URIRef
    ObjectModel: URIRef
    RigidObject: URIRef

    _extras = [
        "of-object",
        "has-object",
        "of-workspace",
        "has-workspace",
        "has-object-model",
    ]

    _NS = NS_MM_ENV


class EXEC(DefinedNamespace):
    ExecutionContext: URIRef
    Simulation: URIRef
    RealWorld: URIRef
    SystemResource: URIRef
    ResourceWithPath: URIRef

    path: URIRef

    _extras = [
        "has-config",
        "has-mapping",
        "maps",
        "has-body",
        "has-kinematic-tree",
        "model-entity",
        "runs-scene",
        "timestep",
        "platform-name",
        "platform-version",
    ]

    _NS = Namespace(f"{URI_SECORO_MM}/execution-context#")


class EXEC_TRACE(DefinedNamespace):
    Frame: URIRef
    StateOccurrence: URIRef
    TransitionOccurrence: URIRef
    EventOccurrence: URIRef
    MonitorOccurrence: URIRef

    atFrame: URIRef
    seq: URIRef
    activeState: URIRef
    step: URIRef
    state: URIRef
    transition: URIRef
    event: URIRef
    monitor: URIRef
    value: URIRef

    _NS = Namespace(f"{URI_SECORO_MM}/motion-spec/execution-trace/")


class EL(DefinedNamespace):
    EventLoop: URIRef
    Event: URIRef
    Flag: URIRef
    EventReaction: URIRef
    FlagReaction: URIRef

    _extras = [
        "has-event",
        "ref-event",
        "has-flag",
        "ref-flag",
        "has-evt-reaction",
        "has-flg-reaction",
    ]

    _NS = NS_MM_EL


class GEOM_ENT(DefinedNamespace):
    Point: URIRef
    Frame: URIRef
    KinematicTree: URIRef
    SimplicialComplex: URIRef
    RigidBody: URIRef
    start: URIRef
    end: URIRef
    origin: URIRef
    simplices: URIRef

    _NS = NS_MM_GEOM


class KC(DefinedNamespace):
    Joint: URIRef
    RevoluteJoint: URIRef

    _extras = ["between-attachments"]

    _NS = NS_MM_KC


class KC_STAT(DefinedNamespace):
    JointReference: URIRef
    JointPositionCoordinate: URIRef
    JointVelocityCoordinate: URIRef
    JointAccelerationCoordinate: URIRef
    JointForceCoordinate: URIRef
    JointForce: URIRef

    _extras = ["of-joint"]

    _NS = NS_MM_KC_STAT


class AGN(DefinedNamespace):
    Agent: URIRef
    AgentModel: URIRef
    ModelledAgent: URIRef

    _extras = ["of-agent", "has-agent", "has-agent-model"]

    _NS = Namespace(f"{URI_SECORO_MM}/agent#")


class QUDT_SCHEMA(DefinedNamespace):
    Quantity: URIRef
    hasQuantityKind: URIRef
    unit: URIRef
    value: URIRef

    _NS = NS_MM_QUDT


class QUDT_QKIND(DefinedNamespace):
    Distance: URIRef
    PlaneAngle: URIRef
    Position: URIRef
    AngularVelocity: URIRef
    LinearVelocity: URIRef
    AngularAcceleration: URIRef
    LinearAcceleration: URIRef
    AccelerationEnergy: URIRef
    Torque: URIRef
    Force: URIRef

    _NS = NS_MM_QUDT_QTY


class GEOM_REL(DefinedNamespace):
    LinearDistance: URIRef
    Orientation: URIRef
    Pose: URIRef
    Position: URIRef
    VelocityTwist: URIRef
    AccelerationTwist: URIRef

    of: URIRef

    _extras = ["with-respect-to", "reference-point", "between-entities"]

    _NS = NS_MM_GEOM_REL


class GEOM_COORD(DefinedNamespace):
    DirectionCoordinate: URIRef
    LinearDistanceCoordinate: URIRef
    OrientationCoordinate: URIRef
    PositionCoordinate: URIRef
    PoseCoordinate: URIRef
    PoseDifferenceCoordinate: URIRef
    VelocityTwistCoordinate: URIRef
    AccelerationTwistCoordinate: URIRef
    DirectionCosineXYZ: URIRef
    EulerAngles: URIRef
    Quaternion: URIRef
    VectorXYZ: URIRef

    x: URIRef
    y: URIRef
    z: URIRef
    alpha: URIRef
    beta: URIRef
    gamma: URIRef

    _extras = [
        "of-pose",
        "of-position",
        "of-orientation",
        "of-velocity",
        "of-acceleration",
        "as-seen-by",
        "direction-cosine-x",
        "direction-cosine-y",
        "direction-cosine-z",
        "axes-sequence",
        "angular-velocity",
        "linear-velocity",
        "angular-acceleration",
        "linear-acceleration",
        "linear",
        "angular",
    ]

    _NS = NS_MM_GEOM_COORD


class GEOM_OP(DefinedNamespace):
    RotateDirectionDistalToProximalWithPose: URIRef
    AddVelocityTwist: URIRef
    AddAccelerationTwist: URIRef
    ComposePose: URIRef
    TransformVelocityTwistToDistal: URIRef
    RotateVelocityTwistToProximalWithPose: URIRef
    TransformAccelerationTwistToDistal: URIRef
    PoseToAngleAroundAxis: URIRef
    PoseToLinearDistance: URIRef
    PoseToDirection: URIRef
    PlanarAngleFromDirections: URIRef
    InvertAngle: URIRef
    InvertPose: URIRef

    in1: URIRef
    in2: URIRef
    composite: URIRef
    pose: URIRef
    to: URIRef
    distance: URIRef
    direction: URIRef
    angle: URIRef
    out: URIRef
    axis: URIRef
    x: URIRef
    y: URIRef
    z: URIRef

    _extras = ["from", "absolute-velocity", "relative-velocity", "from-directions", "in"]

    _NS = Namespace(f"{URI_CR2B_MM}/geometry/spatial-operators#")


class GEOM_REL_EXT(DefinedNamespace):
    PoseDifference: URIRef

    _NS = Namespace(f"{URI_SECORO_MM}/geometry/spatial-relations#")


class GEOM_OP_EXT(DefinedNamespace):
    PoseDiffEvaluator: URIRef
    PoseToAngularDistance: URIRef
    PathProjection: URIRef
    RelativeOrientation: URIRef
    path: URIRef
    tangent: URIRef
    linear: URIRef

    _extras = [
        "angular-distance",
        "path-parameter",
        "normal-a",
        "normal-b",
        "along-speed",
    ]

    _NS = Namespace(f"{URI_SECORO_MM}/geometry/spatial-operators#")


class RBDYN_ENT(DefinedNamespace):
    Wrench: URIRef
    Mass: URIRef
    mass: URIRef

    _extras = ["reference-point", "acts-on", "of-body"]

    _NS = NS_MM_DYN_ENT


class RBDYN_COORD(DefinedNamespace):
    WrenchReference: URIRef
    WrenchCoordinate: URIRef
    UniformGravitationalFieldCoordinate: URIRef

    _extras = ["of-wrench", "as-seen-by", "force", "torque"]

    _NS = NS_MM_DYN_COORD


class RBDYN_OP(DefinedNamespace):
    TransformWrenchToProximal: URIRef
    RotateWrenchToDistalWithPose: URIRef
    RotateWrenchToProximalWithPose: URIRef
    WrenchFromPositionDirectionAndMagnitude: URIRef
    AddWrench: URIRef

    position: URIRef
    pose: URIRef
    to: URIRef
    magnitude: URIRef
    direction: URIRef
    wrench: URIRef
    in1: URIRef
    in2: URIRef
    out: URIRef

    _extras = ["from"]

    _NS = Namespace(f"{URI_CR2B_MM}/newtonian-rigid-body-dynamics/operators#")


class MAP(DefinedNamespace):
    View: URIRef
    PoseCoordinateView: URIRef
    VelocityTwistCoordinateView: URIRef
    AccelerationTwistCoordinateView: URIRef
    WrenchCoordinateView: URIRef

    superobject: URIRef
    subobject: URIRef
    subspace: URIRef
    position: URIRef
    torque: URIRef
    force: URIRef
    axis: URIRef
    x: URIRef
    y: URIRef
    z: URIRef
    w: URIRef

    _extras = ["angular-velocity", "linear-velocity", "angular-acceleration", "linear-acceleration"]

    _NS = Namespace(f"{URI_CR2B_MM}/task/map#")


class MAP_EXT(DefinedNamespace):
    PoseCoordinateView: URIRef
    VelocityTwistCoordinateView: URIRef
    AccelerationTwistCoordinateView: URIRef
    WrenchCoordinateView: URIRef
    PoseDifferenceView: URIRef
    position: URIRef
    orientation: URIRef
    linear: URIRef
    angular: URIRef

    _NS = Namespace(f"{URI_SECORO_MM}/task/map#")


class ALGO_EXT(DefinedNamespace):
    ProgressConstraint: URIRef
    Saturation: URIRef
    Addition: URIRef
    VelocityProfile: URIRef
    Snapshot: URIRef
    Admittance: URIRef
    limits: URIRef
    out: URIRef
    target: URIRef
    mass: URIRef
    damping: URIRef
    stiffness: URIRef
    sampling: URIRef
    trigger: URIRef
    shape: URIRef
    trapezoidal: URIRef
    parameter: URIRef
    path: URIRef
    progress: URIRef

    _extras = [
        "in",
        "maximum-absolute-value",
        "lower-bound",
        "upper-bound",
        "maximum-velocity",
        "maximum-acceleration",
        "maximum-jerk",
        "s-curve",
        "initial-sampling",
        "event-triggered-sampling",
    ]

    _NS = Namespace(f"{URI_SECORO_MM}/algorithm#")


class CSTR(DefinedNamespace):
    Constraint: URIRef
    EqualityConstraint: URIRef
    UnilateralConstraint: URIRef
    GreaterThanConstraint: URIRef
    LessThanConstraint: URIRef
    BilateralConstraint: URIRef
    AngularVelocityConstraint: URIRef
    LinearVelocityConstraint: URIRef
    TorqueConstraint: URIRef
    ForceConstraint: URIRef

    quantity: URIRef
    threshold: URIRef

    _extras = ["PositionConstraint", "reference-value", "lower-threshold", "upper-threshold"]

    _NS = Namespace(f"{URI_CR2B_MM}/task/constraint#")


class CSTR_EXT(DefinedNamespace):
    ConstraintExpression: URIRef
    ConstraintConjunction: URIRef
    ConstraintDisjunction: URIRef
    OutsideConstraint: URIRef
    AngleConstraint: URIRef
    AngularDistanceConstraint: URIRef
    TimeConstraint: URIRef
    ElapsedDurationCoordinate: URIRef

    tolerance: URIRef

    _extras = ["has-constraint"]

    _NS = Namespace(f"{URI_SECORO_MM}/task/constraint#")


class QKIND_EXT(DefinedNamespace):
    LinearJerk: URIRef

    _NS = Namespace(f"{URI_SECORO_MM}/qudt/quantity-kind#")


class TIME(DefinedNamespace):
    Instant: URIRef
    TimePosition: URIRef
    ProperInterval: URIRef
    Duration: URIRef
    TRS: URIRef
    unitSecond: URIRef

    inTimePosition: URIRef
    hasTRS: URIRef
    numericPosition: URIRef
    hasBeginning: URIRef
    hasEnd: URIRef
    hasDuration: URIRef
    numericDuration: URIRef
    unitType: URIRef

    _NS = Namespace("http://www.w3.org/2006/time#")


class MOT(DefinedNamespace):
    GuardedMotion: URIRef

    when: URIRef
    until: URIRef

    _extras = ["while"]

    _NS = Namespace(f"{URI_CR2B_MM}/task/motion-specification#")


class GEOM_PATH(DefinedNamespace):
    Path: URIRef
    LinearPath: URIRef
    Circle: URIRef
    Arc: URIRef
    Helix: URIRef
    Figure8: URIRef

    start: URIRef
    goal: URIRef
    end: URIRef
    anchor: URIRef
    center: URIRef
    axis: URIRef
    direction: URIRef
    radius: URIRef
    amplitude: URIRef
    pitch: URIRef
    revolutions: URIRef
    form: URIRef
    gerono: URIRef
    bernoulli: URIRef

    _extras = ["plane-normal"]

    _NS = Namespace(f"{URI_SECORO_MM}/geometry/path#")


class CSTR_HDL(DefinedNamespace):
    ConstraintHandler: URIRef
    ConstraintEvaluator: URIRef
    AssignmentEvaluator: URIRef
    ErrorEvaluator: URIRef
    Controller: URIRef
    ProportionalIntegralDerivative: URIRef
    ImpedanceController: URIRef
    DecayingIntegralTerm: URIRef
    Monitor: URIRef
    EdgeTriggeredMonitor: URIRef
    LevelTriggeredMonitor: URIRef
    motion: URIRef
    evaluators: URIRef
    monitors: URIRef
    controllers: URIRef
    constraint: URIRef
    error: URIRef
    event: URIRef
    flag: URIRef

    _extras = [
        "error-signal",
        "control-signal",
        "event-queue",
        "proportional-gain",
        "integral-gain",
        "derivative-gain",
        "decay-rate",
        "stiffness",
        "damping",
        "maximum-velocity",
        "measured-velocity",
    ]

    _NS = Namespace(f"{URI_CR2B_MM}/task/constraint-handler#")


class CSTR_HDL_EXT(DefinedNamespace):
    FeedForwardController: URIRef
    SetpointGenerator: URIRef
    solver: URIRef

    _extras = [
        "reference-signal",
        "mass",
        "damping",
        "stiffness",
        "force",
        "fallback-motion",
        "debounce-duration",
    ]

    _NS = Namespace(f"{URI_SECORO_MM}/task/constraint-handler#")


class SLV(DefinedNamespace):
    VelocityCompositionSolver: URIRef
    ForceDistributionSolver: URIRef
    SolverWithInputAndOutput: URIRef
    MotionDrivers: URIRef
    AccelerationConstraintSpecification: URIRef
    CartesianForceSpecification: URIRef
    JointForceSpecification: URIRef
    AccelerationConstraint: URIRef
    AxisAligned: URIRef
    PrioritizationLevel: URIRef
    AccelerationConstrainedHybridDynamicsAlgorithm: URIRef
    RecursiveNewtonEulerAlgorithm: URIRef

    constraints: URIRef
    force: URIRef
    subspace: URIRef
    axis: URIRef
    x: URIRef
    y: URIRef
    z: URIRef
    configuration: URIRef
    velocity: URIRef
    output: URIRef
    solver: URIRef
    root: URIRef
    gravity: URIRef

    _extras = [
        "motion-drivers",
        "cartesian-force",
        "joint-force",
        "acceleration-constraint",
        "acceleration-energy",
        "angular-acceleration",
        "linear-acceleration",
        "attached-to",
        "prioritization-hierarchy",
    ]

    _NS = Namespace(f"{URI_CR2B_MM}/task/solver-specification#")


class SLV_EXT(DefinedNamespace):
    CommandForwardingSolver: URIRef
    VelocityDistributionSolver: URIRef
    ForceCompositionSolver: URIRef

    _NS = Namespace(f"{URI_SECORO_MM}/task/solver-specification#")
