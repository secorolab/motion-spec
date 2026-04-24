# SPDX-License-Identifier: MPL-2.0
from rdflib import URIRef
from rdflib.namespace import DefinedNamespace, Namespace

URI_CR2B_MM = "https://comp-rob2b.github.io/metamodels"
URI_QUDT = "http://qudt.org"


class APP(DefinedNamespace):
    constraints: URIRef
    path: URIRef

    _extras = [
        "import",
        "entry-point",
        "reasoning-rules",
        "iri-map"
    ]

    _NS = Namespace(f"{URI_CR2B_MM}/application/")

class GEOM_ENT(DefinedNamespace):
    Point: URIRef
    Frame: URIRef
    SimplicialComplex: URIRef
    KinematicChain: URIRef
    UniformGravitationalField: URIRef

    _NS = Namespace(f"{URI_CR2B_MM}/geometry/structural-entities#")

class KC(DefinedNamespace):
    Joint: URIRef

    _NS = Namespace(f"{URI_CR2B_MM}/kinematic-chain/structural-entities#")

class QUDT_SCHEMA(DefinedNamespace):
    Quantity: URIRef
    hasQuantityKind: URIRef
    unit: URIRef
    value: URIRef

    _extras = [
        "quantity-kind",
    ]

    _NS = Namespace(f"{URI_QUDT}/schema/qudt/")

class QUDT_QKIND(DefinedNamespace):
    Angle: URIRef
    AngularDistance: URIRef
    Length: URIRef
    Distance: URIRef
    Vector: URIRef
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
    ]

    _NS = Namespace(f"{URI_QUDT}/vocab/unit/")

class GEOM_REL(DefinedNamespace):
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
    PositionCoordinate: URIRef
    PoseCoordinate: URIRef
    VelocityTwistCoordinate: URIRef
    AccelerationTwistCoordinate: URIRef
    DirectionCosineXYZ: URIRef
    VectorXYZ: URIRef

    x: URIRef
    y: URIRef
    z: URIRef

    _extras = [ 
        "of-pose",
        "of-velocity",
        "of-acceleration",
        "as-seen-by",
        "direction-cosine-x",
        "direction-cosine-y",
        "direction-cosine-z",
        "angular-velocity",
        "linear-velocity",
        "angular-acceleration",
        "linear-acceleration"
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

    _extras = [
        "from",
        "absolute-velocity",
        "relative-velocity",
        "from-directions",
        "in"
    ]

    _NS = Namespace(f"{URI_CR2B_MM}/geometry/spatial-operators#")

class RBDYN_ENT(DefinedNamespace):
    Wrench: URIRef

    _extras = [
        "reference-point"
    ]

    _NS = Namespace(f"{URI_CR2B_MM}/newtonian-rigid-body-dynamics/structural-entities#")

class RBDYN_COORD(DefinedNamespace):
    WrenchCoordinate: URIRef
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
        "rotation",
        "pose",
        "ComputeRotationFromPose",
        "angular-velocity",
        "linear-velocity",
        "angular-acceleration",
        "linear-acceleration"
    ]

    _NS = Namespace(f"{URI_CR2B_MM}/task/map#")

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

    _extras = [
        "while"
    ]

    _NS = Namespace(f"{URI_CR2B_MM}/task/motion-specification#")

class CSTR_HDL(DefinedNamespace):
    ConstraintHandler: URIRef
    ConstraintEvaluator: URIRef
    AssignmentEvaluator: URIRef
    ErrorEvaluator: URIRef
    Controller: URIRef
    ProportionalIntegralDerivative: URIRef
    DecayingIntegralTerm: URIRef
    Monitor: URIRef
    EdgeTriggeredMonitor: URIRef
    LevelTriggeredMonitor: URIRef
    Event: URIRef
    Flag: URIRef

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
        "proportional-gain",
        "integral-gain",
        "derivative-gain",
        "decay-rate",
    ]

    _NS = Namespace(f"{URI_CR2B_MM}/task/constraint-handler#")

class SLV(DefinedNamespace):
    VelocityCompositionSolver: URIRef
    ForceDistributionSolver: URIRef
    SolverWithInputAndOutput: URIRef
    MotionDrivers: URIRef
    AccelerationConstraintSpecification: URIRef
    CartesianForceSpecification: URIRef
    JointForce: URIRef
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
        "kinematic-chain",
        "prioritization-hierarchy",
    ]

    _NS = Namespace(f"{URI_CR2B_MM}/task/solver-specification#")
