# SPDX-License-Identifier: MPL-2.0
from rdflib import URIRef
from rdflib.namespace import DefinedNamespace, Namespace

URI_CR2B_MM = "https://comp-rob2b.github.io/metamodels"
URI_SECORO_MM = "https://secorolab.github.io/metamodels"
URI_QUDT = "http://qudt.org"


class APP(DefinedNamespace):
    path: URIRef

    _extras = [
        "constraints",
        "import",
        "iri-map",
        "order",
    ]

    _NS = Namespace(f"{URI_CR2B_MM}/application/")


class SNAP(DefinedNamespace):
    Snapshot: URIRef

    _extras = ["snapshot-of"]

    _NS = Namespace(f"{URI_SECORO_MM}/snapshot#")


class VALUE_ROLE(DefinedNamespace):
    Measured: URIRef
    Declared: URIRef
    Computed: URIRef
    Snapshot: URIRef
    Reference: URIRef
    Error: URIRef
    Commanded: URIRef

    _NS = Namespace(f"{URI_SECORO_MM}/task/value-role#")


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

    _NS = Namespace(f"{URI_SECORO_MM}/environment#")


class SIM(DefinedNamespace):
    SystemResource: URIRef
    ResourceWithPath: URIRef

    path: URIRef

    _NS = Namespace(f"{URI_SECORO_MM}/simulation#")


class EXEC(DefinedNamespace):
    SystemResource: URIRef
    ResourceWithPath: URIRef

    path: URIRef

    _extras = [
        "has-config",
    ]

    _NS = Namespace(f"{URI_SECORO_MM}/execution-context#")


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

    _NS = Namespace(f"{URI_SECORO_MM}/behaviour/event_loop#")


class RT(DefinedNamespace):
    Runtime: URIRef
    MuJoCoRuntime: URIRef
    RealRobotRuntime: URIRef

    _extras = [
        "uses-runtime",
    ]

    _NS = Namespace(f"{URI_SECORO_MM}/runtime#")


class MJ(DefinedNamespace):
    MjcfModel: URIRef
    MuJoCoBody: URIRef
    MuJoCoSite: URIRef
    TrajectoryTrace: URIRef
    ColorRGBA: URIRef

    _extras = [
        "body-name",
        "site-name",
        "color",
        "has-trace",
        "trace-enabled",
        "trace-length",
        "trace-target",
        "attach-to-body",
        "attach-kind",
        "attach-name",
        "attach-prefix",
        "prefix",
        "attach-position",
        "attach-orientation",
        "actuator-name",
        "shape",
        "size",
        "mass",
        "color-r",
        "color-g",
        "color-b",
        "color-a",
        "friction",
        "friction-slide",
        "friction-torsion",
        "friction-roll",
        "tool-body",
        "tcp-site",
    ]

    _NS = Namespace(f"{URI_SECORO_MM}/simulation/mujoco#")


class POLY(DefinedNamespace):
    Polytope: URIRef
    Polygon: URIRef
    Polyhedron: URIRef
    Circle: URIRef
    Cuboid: URIRef
    CuboidWithSize: URIRef
    Cylinder: URIRef

    _extras = [
        "x-size",
        "y-size",
        "z-size",
        "radius",
        "diameter",
        "center",
        "base",
        "height",
        "axis",
        "points",
        "faces",
        "3DPolytope",
    ]

    _NS = Namespace(f"{URI_SECORO_MM}/geometry/polytope#")


class GEOM_ENT(DefinedNamespace):
    Point: URIRef
    Frame: URIRef
    SimplicialComplex: URIRef
    KinematicChain: URIRef
    UniformGravitationalField: URIRef
    RigidBody: URIRef
    start: URIRef
    end: URIRef

    _extras = ["kinematic-chain"]

    _NS = Namespace(f"{URI_CR2B_MM}/geometry/structural-entities#")

class KC(DefinedNamespace):
    Joint: URIRef

    _NS = Namespace(f"{URI_CR2B_MM}/kinematic-chain/structural-entities#")


class KC_STAT(DefinedNamespace):
    JointPositionCoordinate: URIRef
    JointVelocityCoordinate: URIRef
    JointAccelerationCoordinate: URIRef
    JointForceCoordinate: URIRef

    _NS = Namespace(f"{URI_CR2B_MM}/kinematic-chain/state#")


class QUDT_SCHEMA(DefinedNamespace):
    Quantity: URIRef
    hasQuantityKind: URIRef
    unit: URIRef
    value: URIRef

    _extras = ["quantity-kind"]

    _NS = Namespace(f"{URI_QUDT}/schema/qudt/")

class QUDT_QKIND(DefinedNamespace):
    Angle: URIRef
    AngularDistance: URIRef
    Length: URIRef
    Distance: URIRef
    FreeVector: URIRef
    Dimensionless: URIRef
    PlaneAngle: URIRef
    Position: URIRef
    Direction: URIRef
    AngularVelocity: URIRef
    LinearVelocity: URIRef
    AngularAcceleration: URIRef
    LinearAcceleration: URIRef
    AccelerationEnergy: URIRef
    Torque: URIRef
    Force: URIRef
    Time: URIRef

    _NS = Namespace(f"{URI_QUDT}/vocab/quantitykind/")

class QUDT_UNIT(DefinedNamespace):
    UNITLESS: URIRef
    M: URIRef
    N: URIRef

    _extras = [
        "M-PER-SEC",
        "M-PER-SEC2",
        "N-M",
        "N-M2-PER-SEC2",
        "RAD-PER-SEC",
        "RAD-PER-SEC2",
        "RAD",
        "DEG",
        "DEG-PER-SEC",
        "CentiM",
        "CentiM-PER-SEC",
        "SEC",
        "MilliSEC",
    ]

    _NS = Namespace(f"{URI_QUDT}/vocab/unit/")

class GEOM_REL(DefinedNamespace):
    LinearDistance: URIRef
    Orientation: URIRef
    Pose: URIRef
    Direction: URIRef
    Position: URIRef
    VelocityTwist: URIRef
    AccelerationTwist: URIRef

    of: URIRef

    _extras = [ 
        "with-respect-to",
        "reference-point"
    ]

    _NS = Namespace(f"{URI_CR2B_MM}/geometry/spatial-relations#")

class GEOM_COORD(DefinedNamespace):
    DirectionCoordinate: URIRef
    LinearDistanceCoordinate: URIRef
    OrientationCoordinate: URIRef
    PositionCoordinate: URIRef
    PoseCoordinate: URIRef
    VelocityTwistCoordinate: URIRef
    AccelerationTwistCoordinate: URIRef
    DirectionCosineXYZ: URIRef
    EulerAngles: URIRef
    AnglesABG: URIRef
    VectorXYZ: URIRef

    x: URIRef
    y: URIRef
    z: URIRef
    alpha: URIRef
    beta: URIRef
    gamma: URIRef

    _extras = [ 
        "of-pose",
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
        "angle-axis",
        "has-coordinate",
    ]

    _NS = Namespace(f"{URI_CR2B_MM}/geometry/coordinates#")

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
    PoseDiffEvaluator: URIRef
    InvertPose: URIRef
    PlanarAngleFromDirections: URIRef
    InvertAngle: URIRef

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

    _extras = [
        "from",
        "absolute-velocity",
        "relative-velocity",
        "from-directions",
        "in",
    ]

    _NS = Namespace(f"{URI_CR2B_MM}/geometry/spatial-operators#")

class RBDYN_ENT(DefinedNamespace):
    Wrench: URIRef
    Mass: URIRef
    mass: URIRef

    _extras = [
        "reference-point",
        "of-body",
    ]

    _NS = Namespace(f"{URI_CR2B_MM}/newtonian-rigid-body-dynamics/structural-entities#")

class RBDYN_COORD(DefinedNamespace):
    WrenchCoordinate: URIRef
    UniformGravitationalFieldCoordinate: URIRef
    #VectorXYZ: URIRef

    _extras = [
        "as-seen-by"
    ]

    _NS = Namespace(f"{URI_CR2B_MM}/newtonian-rigid-body-dynamics/coordinates#")

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

    _extras = [
        "from"
    ]

    _NS = Namespace(f"{URI_CR2B_MM}/newtonian-rigid-body-dynamics/operators#")

class RBDYN_OP_EXT(DefinedNamespace):
    # Secorolab extension to the upstream comp-rob2b rigid-body-dynamics operators:
    # a generic element-wise quantity addition. Lives in the secorolab namespace
    # rather than squatting in comp-rob2b's operators#; upstream rbdyn-op:
    # predicates (in1/in2/out) are reused.
    AddQuantity: URIRef

    _NS = Namespace(f"{URI_SECORO_MM}/newtonian-rigid-body-dynamics/operators#")

class MAP(DefinedNamespace):
    View: URIRef
    DirectionCoordinateView: URIRef
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

    _extras = [
        "angular-velocity",
        "linear-velocity",
        "angular-acceleration",
        "linear-acceleration"
    ]

    _NS = Namespace(f"{URI_CR2B_MM}/task/map#")

class MAP_EXT(DefinedNamespace):
    # Secorolab coordinate-view and operator extensions to the upstream comp-rob2b
    # `map` vocabulary. New classes/terms live here rather than squatting in
    # comp-rob2b's `task/map#`; the map: predicates (superobject/subobject/subspace/
    # axis) are reused from upstream. Mirrors how `mot-ext` shadows `mot`.
    PoseOrientationView: URIRef
    PosePositionView: URIRef
    ComputeRotationFromPose: URIRef
    rotation: URIRef
    pose: URIRef

    _NS = Namespace(f"{URI_SECORO_MM}/task/map#")

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
    AngleConstraint: URIRef
    DistanceConstraint: URIRef

    quantity: URIRef
    threshold: URIRef

    _extras = [
        "PositionConstraint",
        "reference-value",
        "lower-threshold",
        "upper-threshold",
    ]

    _NS = Namespace(f"{URI_CR2B_MM}/task/constraint#")

class MOT(DefinedNamespace):
    GuardedMotion: URIRef

    when: URIRef
    until: URIRef

    _extras = ["while"]

    _NS = Namespace(f"{URI_CR2B_MM}/task/motion-specification#")


class TRAJ(DefinedNamespace):
    Trajectory: URIRef
    Lerp: URIRef
    Circle: URIRef
    Arc: URIRef
    Helix: URIRef
    Figure8: URIRef
    Progress: URIRef

    start: URIRef
    goal: URIRef
    alpha: URIRef
    trajectory: URIRef
    anchor: URIRef
    center: URIRef
    radius: URIRef
    amplitude: URIRef
    end: URIRef
    axis: URIRef
    pitch: URIRef
    revolutions: URIRef
    orientation: URIRef
    form: URIRef
    profile: URIRef

    _extras = ["plane-normal"]

    _NS = Namespace("https://secorolab.github.io/metamodels/task/trajectory#")


class MOT_EXT(DefinedNamespace):
    ConstraintConjunction: URIRef
    ConstraintDisjunction: URIRef

    _extras = ["has-constraint"]

    _NS = Namespace(f"{URI_SECORO_MM}/task/motion-specification#")

class CSTR_HDL(DefinedNamespace):
    ConstraintHandler: URIRef
    JointTorque: URIRef
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
        "control-mode",
        "event-queue",
        "proportional-gain",
        "integral-gain",
        "derivative-gain",
        "decay-rate",
        "stiffness",
        "damping",
    ]

    _NS = Namespace(f"{URI_CR2B_MM}/task/constraint-handler#")

class CSTR_HDL_EXT(DefinedNamespace):
    # Secorolab extension to the upstream comp-rob2b constraint-handler.
    # New classes/predicates live here rather than squatting in comp-rob2b's
    # task/constraint-handler#; upstream cstr_hdl predicates are reused.
    FeedForwardController: URIRef

    _extras = [
        "reference-signal",
        "monitors-until",
        "monitors-when",
        "fallback-motion",
        "control-period",
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
    NewtonEulerAlgorithm: URIRef

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
    # Secorolab extension to the upstream comp-rob2b solver-specification: a
    # pass-through "command forwarding" solver (used for gripper actuation), plus
    # the control-signal and gravity-value terms that upstream does not define.
    # These new classes/predicates live in the secorolab namespace rather than
    # squatting in comp-rob2b's task/solver-specification#; upstream slv:
    # predicates (solver, attached-to) are reused.
    CommandForwardingSolver: URIRef
    CommandForwardingSpecification: URIRef
    CommandForwardingAlgorithm: URIRef

    # "robot" links a solver to the environment robot whose kinematic chain it
    # drives, so per-robot chain setups can be resolved in multi-robot scenes.
    _extras = ["command-forwarding", "control-signal", "gravity-value", "robot"]

    _NS = Namespace(f"{URI_SECORO_MM}/task/solver-specification#")
