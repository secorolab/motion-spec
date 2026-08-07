# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""RDF graph parsing: the dataset loader, `Parser`, the DSL operator tables and the
derived-IRI registry. Nothing here is published; these are the lowering's own indexes."""

from __future__ import annotations

import collections
import itertools
import math
import weakref
from dataclasses import (
    dataclass, field,
)
from enum import Enum
from pathlib import Path
from urllib.parse import urlsplit
import rdflib
from rdf_utils.constraints import ConstraintViolation
from scene_dsl.rdf_parser.vocab import NS_MM_ROS
from rdf_utils.models.geom_coord import (
    OrientCoordModel, PoseCoordModel, PositionCoordModel, get_coord_vectorxyz,
    get_orientation_coord_vals,
)
from rdf_utils.models.geom_rel import (
    OrientationModel, PositionModel,
)
from rdf_utils.models.common import (
    ModelBase, get_node_types,
)
# fmt: off
from rdf_utils.models.vocab import (
    URI_GEOM_PRED_AXES_SEQ, URI_GEOM_PRED_ALPHA, URI_GEOM_PRED_BETA,
    URI_GEOM_PRED_DIRECTION_COSINE_X, URI_GEOM_PRED_DIRECTION_COSINE_Y,
    URI_GEOM_PRED_DIRECTION_COSINE_Z, URI_GEOM_PRED_GAMMA, URI_GEOM_PRED_W, URI_GEOM_PRED_X,
    URI_GEOM_PRED_Y, URI_GEOM_PRED_Z, URI_GEOM_TYPE_ANGLES_ABG,
    URI_GEOM_TYPE_DIRECTION_COSINE_XYZ, URI_GEOM_TYPE_EULER_ANGLES, URI_GEOM_TYPE_INTRINSIC,
    URI_GEOM_TYPE_POSE_COORD, URI_GEOM_TYPE_POSE, URI_GEOM_TYPE_POSITION,
    URI_GEOM_TYPE_POSITION_COORD, URI_GEOM_TYPE_ORIENT, URI_GEOM_TYPE_ORIENT_COORD,
    URI_GEOM_TYPE_QUATERNION, URI_QUDT_UNIT_CM, URI_QUDT_UNIT_DEG, URI_QUDT_UNIT_M,
    URI_QUDT_UNIT_MM, URI_QUDT_UNIT_RAD,
)
# fmt: on
from rdf_utils.namespace import (
    NS_MM_QUDT_QTY as QUDT_QTY, NS_MM_QUDT_UNIT as QUDT_UNIT,
)
from rdf_utils.resolver import (
    IriToFileResolver, install_resolver,
)
from rdflib import URIRef
from rdflib.namespace import (
    RDF, SDO, split_uri,
)
# fmt: off
from motion_spec.classes.entities import (
    AccelerationTwist, Axis, BilateralConstraint, CartesianForceSpecification, Constraint,
    ConstraintEvaluator, ConstraintHandler, Direction, EdgeMonitor, EqualityConstraint,
    EvaluatorType, ForceDistributionSolver, Frame, FreeVector, GuardedMotion,
    JointForceSpecification, JointPosition, LevelMonitor, MotionDrivers, Orientation,
    OutsideConstraint, Point, Pose, PoseDifference, Position, Provenance, Quantity, QuantityKind,
    Saturation, SceneObject, Setpoint, SimplicialComplex, SolverWithInputAndOutput, Subspace,
    UnilateralConstraint, UnilateralConstraintType, Unit, VelocityCompositionSolver,
    VelocityTwist, View, Wrench,
)
# fmt: on
# fmt: off
from motion_spec_dsl.rdf_parser.vocab import (
    ALGO_EXT, APP, CSTR, CSTR_EXT, CSTR_HDL, CSTR_HDL_EXT, ENV, GEOM_COORD, GEOM_ENT, GEOM_OP,
    GEOM_OP_EXT, GEOM_PATH, GEOM_REL, KC_STAT, MAP, MAP_EXT, MOT, QUDT_QKIND, QUDT_SCHEMA,
    RBDYN_COORD, RBDYN_ENT, RBDYN_OP, SLV, SLV_EXT, SENSORS, SOSA, TIME,
)
# fmt: on
from motion_spec_dsl.rdf_parser.manifest import (
    build_url_map, metamodel_url_map,
)
from rdf_utils.models.vocab import (
    URI_KC_EXT_PRED_OF_JOINT, URI_KC_EXT_TYPE_JOINT_LIMIT, URI_KC_STAT_JNT_POSITION,
    URI_KC_TYPE_REVOLUTE_JOINT, URI_KC_TYPE_REVOLUTE_JOINT_ORIENTED_AXIS,
)

from motion_spec.rdf_parser.records import (
    _dedupe_by_id, _kebab, _ros_type_parts, escape, memoize,
)

class AccelerationInputKind(str, Enum):
    """Physical acceleration input accepted by a solver algorithm."""

    None_ = "None"
    ConstraintEnergy = "ConstraintEnergy"
    CartesianAcceleration = "CartesianAcceleration"
@dataclass(frozen=True)
class SolverSemantics:
    """Algorithm-specific meanings needed while deriving executable solver inputs."""

    acceleration_input: AccelerationInputKind
    signal_prefix: str | None = None
    codegen_name: str = ""
SOLVER_SEMANTICS_BY_ALGORITHM = {
    SLV["AccelerationConstrainedHybridDynamicsAlgorithm"]: SolverSemantics(
        AccelerationInputKind.ConstraintEnergy, "eacc", "ACHD"
    ),
    SLV["RecursiveNewtonEulerAlgorithm"]: SolverSemantics(
        AccelerationInputKind.CartesianAcceleration, "acc", "RNE"
    ),
}
COMMAND_FORWARDING_SEMANTICS = SolverSemantics(AccelerationInputKind.None_)
def _is_elapsed_constraint(g, cstr_node) -> bool:
    """Whether a constraint node is a timing (elapsed) constraint."""
    return cstr_node is not None and CSTR_EXT["TimeConstraint"] in get_node_types(g, cstr_node)
def _duration_seconds(g, node) -> float:
    """The value in seconds of a Duration node, converting from the unit it was written in."""
    return _seconds(float(g.value(node, QUDT_SCHEMA["value"])), g.value(node, QUDT_SCHEMA["unit"]))
def _term_name(node) -> str | None:
    if node is None:
        return None
    return split_uri(str(node))[1]
def _resolve_solver_semantics(g, solver: URIRef) -> SolverSemantics:
    """Resolve a solver resource to explicit input semantics or reject it."""
    if SLV_EXT.CommandForwardingSolver in get_node_types(g, solver):
        return COMMAND_FORWARDING_SEMANTICS
    algorithm = g.value(solver, SLV["solver"])
    # No algorithm authored: a monitor-only solver, nothing accepts acceleration input.
    if algorithm is None:
        return SolverSemantics(AccelerationInputKind.None_)
    try:
        return SOLVER_SEMANTICS_BY_ALGORITHM[algorithm]
    except KeyError as exc:
        raise ValueError(f"Solver '{solver}' has unsupported algorithm '{algorithm}'.") from exc
def parse_argument(g, closure_id, argument, to_id, resolve_value=False):
    """Resolve a closure argument (input/output/parameter) to id(s): a lone value collapses to a
    scalar and a qudt:Quantity constant resolves to its scalar value.
    """
    entry = list(g[closure_id:argument])
    if len(entry) == 0:
        return None

    def resolve(e):
        """Resolve one graph value to its id, unwrapping a qudt:Quantity constant to its scalar.

        Parameters are baked into generated code as literal text, so the constant must collapse
        to a scalar here; bare literals and IRI refs pass through.
        """
        if resolve_value and not isinstance(e, rdflib.Literal):
            qval = g.value(e, QUDT_SCHEMA["value"])
            if qval is not None:
                return qval
        return to_id(e)

    ids = [resolve(e) for e in entry]
    unique = list(dict.fromkeys(ids))  # deduplicate, preserving order
    if len(unique) == 1:
        return unique[0]
    return unique
def _reference_inputs(g, node):
    """Inputs a referenced data structure contributes to whoever reads it. A path produces nothing,
    so the output-to-input walk never reaches its parameters as a producer's outputs.
    """
    if GEOM_PATH.Path not in get_node_types(g, node):
        return ()
    return tuple(obj for pred, obj in g.predicate_objects(node) if pred != RDF["type"])
def _fill_closure_args(g, closure, to_id, op, closure_id, input_subject) -> None:
    """Fill a closure's input/output/parameter slots; inputs may hang off another subject.
    Insertion order is the emitted JSON key order.
    """
    for input_ in op.input:
        closure[to_id(input_)] = parse_argument(g, input_subject, input_, to_id)
    for output in op.output:
        closure[to_id(output)] = parse_argument(g, closure_id, output, to_id)
    for param in op.parameters:
        closure[to_id(param)] = parse_argument(g, closure_id, param, to_id, resolve_value=True)
def _operator_inputs(g, operator_id, inputs) -> set:
    """Data-structure nodes feeding an operator call's inputs, plus any path parameters."""
    data_structures = set()
    for in_ in inputs:
        for data_in in g.objects(operator_id, in_):
            data_structures.add(data_in)
            data_structures.update(_reference_inputs(g, data_in))
    return data_structures
def _constraint_types(g, node) -> set:
    """Types of the constraint a handler call points at; empty when it points at nothing."""
    constraint_id = g.value(node, CSTR_HDL["constraint"])
    return get_node_types(g, constraint_id) if constraint_id is not None else set()
def _constraint_inputs(g, operator_id, inputs) -> set:
    """Data-structure nodes reached through a handler call's constraint."""
    return {
        data_in
        for in_ in inputs
        for data_in in g.objects(operator_id, CSTR_HDL["constraint"] / in_)
    }
@dataclass
class Operator:
    """A schedulable RDF computation: maps graph inputs/outputs/parameters to a closure and
    participates in output-to-input scheduling.
    """

    type_: URIRef
    input: list[URIRef]
    output: list[URIRef]
    parameters: list = field(default_factory=list)
    schedulable: bool = True

    def closure_step(self, g, to_id, closure_id):
        """Build the closure dict for this operator's call at closure_id."""
        closure = {"id": to_id(closure_id), "type": to_id(self.type_)}
        _fill_closure_args(g, closure, to_id, self, closure_id, closure_id)
        return closure

    def from_operator_to_input(self, g, operator_id):
        """Data-structure nodes feeding this operator call's inputs."""
        return _operator_inputs(g, operator_id, self.input)

    def scheduler_step(self, g, data_out):
        """Input data structures and schedulable calls producing data_out for this operator."""
        data_structures = set()
        schedule = []

        for out in self.output:
            # sorted(): unordered here, and append order becomes the emitted schedule order.
            for call in sorted(g[:out:data_out]):
                if self.type_ not in get_node_types(g, call):
                    continue
                # A call with no input cannot be scheduled.
                inputs = _operator_inputs(g, call, self.input)
                if inputs:
                    data_structures |= inputs
                    schedule.append(call)

        return {"data_structures": data_structures, "schedule": schedule}
@dataclass
class Specification:
    """Not a computation: no closure and no schedule entry, but it propagates outputs to inputs so
    further computations are found.
    """

    type_: URIRef
    input: list[URIRef]
    output: list[URIRef]
    parameters: list = field(default_factory=list)
    schedulable: bool = False

    def from_operator_to_input(self, g, operator_id):
        """Data-structure nodes feeding this specification's inputs."""
        return _operator_inputs(g, operator_id, self.input)

    def scheduler_step(self, g, data_out):
        """Input data structures for this specification (never schedulable)."""
        data_structures = set()
        for out in self.output:
            for call in g.subjects(out, data_out):
                for in_ in self.input:
                    for data_in in g.objects(call, in_):
                        data_structures.add(data_in)

        return {"data_structures": data_structures, "schedule": []}
def _continuous_joint_leaves(g) -> set[str]:
    """Leaf names of revolute joints with no authored position limit: continuous joints,
    whose position error lives on the circle. Read from absence -- scene-dsl emits a
    kc-ext:JointLimit per authored bound and nothing for a missing one.
    """
    revolute = set(g.subjects(RDF.type, URI_KC_TYPE_REVOLUTE_JOINT)) | set(
        g.subjects(RDF.type, URI_KC_TYPE_REVOLUTE_JOINT_ORIENTED_AXIS)
    )
    position_limited = {
        g.value(limit, URI_KC_EXT_PRED_OF_JOINT)
        for limit in g.subjects(RDF.type, URI_KC_EXT_TYPE_JOINT_LIMIT)
        if URI_KC_STAT_JNT_POSITION in get_node_types(g, limit)
    }
    return {_term_name(joint) for joint in revolute - position_limited}
class ErrorEvaluator:
    """Constraint-handler operator emitting a constraint error signal, dispatching on the constraint
    type (equality/greater/less/bilateral/outside).
    """

    def __init__(self):
        """Register the constraint operators this error evaluator dispatches over."""
        self.type_ = CSTR_HDL["ErrorEvaluator"]
        self.schedulable = False
        self.cstr_op = [
            Operator(
                type_=CSTR["EqualityConstraint"],
                input=[CSTR["quantity"], CSTR["reference-value"]],
                output=[CSTR_HDL["error"]],
            ),
            Operator(
                type_=CSTR["GreaterThanConstraint"],
                input=[CSTR["quantity"], CSTR["threshold"]],
                output=[CSTR_HDL["error"]],
            ),
            Operator(
                type_=CSTR["LessThanConstraint"],
                input=[CSTR["quantity"], CSTR["threshold"]],
                output=[CSTR_HDL["error"]],
            ),
            Operator(
                type_=CSTR["BilateralConstraint"],
                input=[CSTR["quantity"], CSTR["lower-threshold"], CSTR["upper-threshold"]],
                output=[CSTR_HDL["error"]],
            ),
            Operator(
                type_=CSTR_EXT["OutsideConstraint"],
                input=[CSTR["quantity"], CSTR["lower-threshold"], CSTR["upper-threshold"]],
                output=[CSTR_HDL["error"]],
            ),
        ]

    def closure_step(self, g, to_id, closure_id):
        """Build the error-evaluator closure, dispatching on the constraint type."""
        constraint_id = g.value(closure_id, CSTR_HDL["constraint"])

        for operator in self.cstr_op:
            if operator.type_ not in get_node_types(g, constraint_id):
                continue

            closure = {
                "id": to_id(closure_id),
                "type": "ErrorEvaluator",
                "constraint": to_id(operator.type_),
            }

            # An equality error on a continuous joint wraps to the shortest arc.
            if operator.type_ == CSTR["EqualityConstraint"]:
                quantity = g.value(constraint_id, CSTR["quantity"])
                joint = (
                    g.value(quantity, KC_STAT["of-joint"]) if quantity is not None else None
                )
                if joint is not None and _term_name(joint) in _continuous_joint_leaves(g):
                    closure["angular_wrap"] = True

            _fill_closure_args(g, closure, to_id, operator, closure_id, constraint_id)
            return closure  # first matching constraint type wins

        return None

    def from_operator_to_input(self, g, operator_id):
        """Data-structure nodes feeding the matching constraint's inputs."""
        constraint_types = _constraint_types(g, operator_id)
        data_structures = set()
        for op in self.cstr_op:
            if op.type_ in constraint_types:
                data_structures |= _constraint_inputs(g, operator_id, op.input)
        return data_structures

    def scheduler_step(self, g, data_out):
        """Input data structures and schedulable calls producing the error data_out."""
        data_structures = set()
        schedule = []

        for op in self.cstr_op:
            for out in op.output:
                # sorted(): see Parser.schedule -- append order becomes schedule order.
                for call in sorted(g.subjects(out, data_out)):
                    if op.type_ not in _constraint_types(g, call):
                        continue
                    # A call with no input cannot be scheduled.
                    inputs = _constraint_inputs(g, call, op.input)
                    if inputs:
                        data_structures |= inputs
                        schedule.append(call)

        return {"data_structures": data_structures, "schedule": schedule}
class AssignmentEvaluator:
    """Constraint operator that assigns a reference value to a quantity (no error output)."""

    def __init__(self):
        """Register the equality-constraint operator this assignment evaluator uses."""
        self.type_ = CSTR_HDL["AssignmentEvaluator"]
        self.schedulable = True
        self.cstr_op = Operator(
            type_=CSTR["EqualityConstraint"],
            input=[CSTR["quantity"], CSTR["reference-value"]],
            output=[],
        )

    def closure_step(self, g, to_id, closure_id):
        """Build the assignment-evaluator closure (equality constraint only)."""
        constraint_id = g.value(closure_id, CSTR_HDL["constraint"])

        if self.cstr_op.type_ not in get_node_types(g, constraint_id):
            return None

        closure = {
            "id": to_id(closure_id),
            "type": "AssignmentEvaluator",
            "constraint": to_id(self.cstr_op.type_),
        }

        _fill_closure_args(g, closure, to_id, self.cstr_op, closure_id, constraint_id)
        return closure

    def from_operator_to_input(self, g, operator_id):
        """Data-structure nodes feeding the assignment's inputs."""
        if self.cstr_op.type_ not in _constraint_types(g, operator_id):
            return set()
        return _constraint_inputs(g, operator_id, self.cstr_op.input)
def _op_output_preds(op):
    """Output predicates an operator's scheduler queries; empty when the operator can never match,
    so its scheduler step can be skipped.
    """
    if isinstance(op, ErrorEvaluator):
        return {p for sub in op.cstr_op for p in sub.output}
    if isinstance(op, AssignmentEvaluator):
        return set()
    return set(op.output)
# A path is geometry: data with no output, so it is never found by the output-to-input walk
# and yields no closure of its own. The evaluator that traverses it is the computation, and
# folds the path's geometry into its call.
ops_path = [
    Specification(
        type_=GEOM_PATH["LinearPath"],
        input=[GEOM_PATH["start"], GEOM_PATH["goal"]],
        output=[],
    ),
    Specification(
        type_=GEOM_PATH["Circle"],
        input=[GEOM_PATH["start"], GEOM_PATH["center"], GEOM_PATH["plane-normal"]],
        output=[],
    ),
    Specification(
        type_=GEOM_PATH["Arc"],
        input=[
            GEOM_PATH["start"],
            GEOM_PATH["end"],
            GEOM_PATH["amplitude"],
            GEOM_PATH["plane-normal"],
        ],
        output=[],
    ),
    Specification(
        type_=GEOM_PATH["Helix"],
        input=[
            GEOM_PATH["start"],
            GEOM_PATH["center"],
            GEOM_PATH["axis"],
            GEOM_PATH["pitch"],
            GEOM_PATH["revolutions"],
        ],
        output=[],
    ),
    Specification(
        type_=GEOM_PATH["Figure8"],
        input=[
            GEOM_PATH["anchor"],
            GEOM_PATH["radius"],
            GEOM_PATH["plane-normal"],
        ],
        output=[],
        parameters=[GEOM_PATH["form"]],
    ),
]
ops_generic = [
    Operator(
        type_=GEOM_OP["RotateDirectionDistalToProximalWithPose"],
        input=[GEOM_OP["pose"], GEOM_OP["from"]],
        output=[GEOM_OP["to"]],
    ),
    Operator(
        type_=GEOM_OP["ComposePose"],
        input=[GEOM_OP["in1"], GEOM_OP["in2"]],
        output=[GEOM_OP["composite"]],
    ),
    Operator(type_=GEOM_OP["InvertPose"], input=[GEOM_OP["pose"]], output=[GEOM_OP["out"]]),
    Operator(
        type_=GEOM_OP["RotateVelocityTwistToProximalWithPose"],
        input=[GEOM_OP["pose"], GEOM_OP["from"]],
        output=[GEOM_OP["to"]],
    ),
    Operator(
        type_=GEOM_OP["PoseToAngleAroundAxis"],
        input=[GEOM_OP["pose"]],
        output=[GEOM_OP["angle"]],
        parameters=[GEOM_OP["axis"]],
    ),
    Operator(
        type_=GEOM_OP["PoseToLinearDistance"], input=[GEOM_OP["pose"]], output=[GEOM_OP["distance"]]
    ),
    Operator(
        type_=GEOM_OP["PoseToDirection"], input=[GEOM_OP["pose"]], output=[GEOM_OP["direction"]]
    ),
    Operator(
        type_=GEOM_OP["PlanarAngleFromDirections"],
        input=[GEOM_OP["from-directions"]],
        output=[GEOM_OP["angle"]],
    ),
    Operator(type_=GEOM_OP["InvertAngle"], input=[GEOM_OP["in"]], output=[GEOM_OP["out"]]),
    Operator(
        type_=RBDYN_OP["AddWrench"],
        input=[RBDYN_OP["in1"], RBDYN_OP["in2"]],
        output=[RBDYN_OP["out"]],
    ),
    Operator(
        type_=ALGO_EXT.Addition,
        input=[ALGO_EXT["in"]],
        output=[ALGO_EXT.out],
    ),
    Operator(
        type_=RBDYN_OP["RotateWrenchToDistalWithPose"],
        input=[RBDYN_OP["pose"], RBDYN_OP["from"]],
        output=[RBDYN_OP["to"]],
    ),
    Operator(
        type_=RBDYN_OP["RotateWrenchToProximalWithPose"],
        input=[RBDYN_OP["pose"], RBDYN_OP["from"]],
        output=[RBDYN_OP["to"]],
    ),
    Operator(
        type_=RBDYN_OP["TransformWrenchToProximal"],
        input=[RBDYN_OP["pose"], RBDYN_OP["from"]],
        output=[RBDYN_OP["to"]],
    ),
    Operator(
        type_=RBDYN_OP["WrenchFromPositionDirectionAndMagnitude"],
        input=[RBDYN_OP["magnitude"], RBDYN_OP["direction"], RBDYN_OP["position"]],
        output=[RBDYN_OP["wrench"]],
    ),
    Specification(type_=MAP["View"], input=[MAP["superobject"]], output=[MAP["subobject"]]),
    *ops_path,
    Operator(
        type_=GEOM_OP_EXT.PathProjection,
        input=[GEOM_OP_EXT.path, GEOM_OP["pose"]],
        output=[GEOM_OP_EXT["path-parameter"]],
    ),
    Operator(
        type_=GEOM_OP_EXT.PathTangentFrame,
        input=[GEOM_OP_EXT.path, GEOM_OP_EXT["path-parameter"]],
        output=[GEOM_OP_EXT.tangent, GEOM_OP_EXT["normal-a"], GEOM_OP_EXT["normal-b"]],
    ),
    Operator(
        type_=GEOM_OP_EXT.TwistToLinearVelocityAlong,
        input=[GEOM_OP["in"], GEOM_OP["direction"]],
        output=[GEOM_OP_EXT["along-speed"]],
    ),
    Operator(
        type_=GEOM_OP_EXT.PathEvaluator,
        input=[GEOM_OP_EXT.path, GEOM_OP_EXT["path-parameter"]],
        output=[GEOM_OP["out"]],
    ),
    Operator(
        type_=ALGO_EXT["VelocityProfile"],
        input=[
            ALGO_EXT["target"],
            ALGO_EXT["in"],
            ALGO_EXT["maximum-velocity"],
            ALGO_EXT["maximum-acceleration"],
            ALGO_EXT["maximum-jerk"],
        ],
        output=[ALGO_EXT["out"]],
        parameters=[ALGO_EXT["shape"]],
    ),
    Operator(
        type_=ALGO_EXT["Admittance"],
        input=[ALGO_EXT["in"]],
        output=[ALGO_EXT["out"]],
        parameters=[
            ALGO_EXT["mass"],
            ALGO_EXT["damping"],
            ALGO_EXT["stiffness"],
            CSTR_HDL["maximum-velocity"],
        ],
    ),
]
ops_cstr_hdl = [
    AssignmentEvaluator(),
    ErrorEvaluator(),
]
ops_slv = [
    Specification(type_=SLV["CartesianForceSpecification"], input=[SLV["force"]], output=[]),
    Specification(type_=SLV["JointForceSpecification"], input=[SLV["force"]], output=[]),
    Specification(type_=SLV["ForceDistributionSolver"], input=[SLV["force"]], output=[]),
]
class Parser:
    # A context quantity's id is its URI's last segment, so a name reused across motions
    # collapses to one id. Qualify only ambiguous names (same segment, >1 owner).
    """Parses a motion-spec RDF dataset into IR pieces: ids, closures, views, data structures,
    handlers and solvers.
    """

    # scan depends only on the graph; memoize per-graph
    _ambiguous_cache: "weakref.WeakKeyDictionary" = weakref.WeakKeyDictionary()

    def __init__(self, g):
        """Bind the parser to an RDF graph and initialize its memoization cache."""
        self.cache = dict()
        self.g = g
        self.sched = set()
        cached = Parser._ambiguous_cache.get(g)
        if cached is None:
            cached = self._compute_ambiguous_context_ids()
            Parser._ambiguous_cache[g] = cached
        self._ambiguous_context_ids = cached
        self._id_sources: dict[str, set[str]] = {}
        self._id_cache: dict = {}

    def _expect_type(self, id_, type_):
        """Raise if `id_` lacks the expected rdf:type. Replaces bare asserts so
        the check survives `python -O` and names the offending node."""
        if type_ not in get_node_types(self.g, id_):
            raise ConstraintViolation(
                "motion-spec", f"Node '{id_}' is missing expected rdf:type '{type_}'"
            )

    def _compute_ambiguous_context_ids(self):
        """Local names shared by more than one node, which need model-scope prefixing."""
        sources_by_id: dict[str, set[str]] = {}
        for s in set(self.g.subjects()):
            if self._context_scope(s) is None:
                continue
            try:
                local = escape(self.g.compute_qname(s)[2])
            except Exception:
                continue
            sources_by_id.setdefault(local, set()).add(str(s))
        return {lid for lid, sources in sources_by_id.items() if len(sources) > 1}

    @staticmethod
    def _context_scope(node) -> tuple[str, str, tuple[str, ...]] | None:
        """Return the owner, section and member path of a context quantity IRI."""
        parts = tuple(part for part in urlsplit(str(node)).path.split("/") if part)
        for index in range(1, len(parts) - 1):
            if parts[index] in ("spec", "world"):
                return parts[index - 1], parts[index], parts[index + 1 :]
        return None

    def id(self, x):
        """Stable local id for a URI/node (scoped when the bare name is ambiguous)."""
        cached = self._id_cache.get(x)
        if cached is not None:
            return cached
        try:
            q = self.g.compute_qname(x)
            local = escape(q[2])
        except Exception:
            self._id_cache[x] = x
            return x
        # Only context quantities become `shared.*` fields and can merge silently; constraint
        # names, metamodel predicates and aliases legitimately share an id.
        scope = self._context_scope(x)
        if scope:
            owner, _section, member_path = scope
            if local in self._ambiguous_context_ids:
                local = escape("-".join((owner, *member_path)))
            self._id_sources.setdefault(local, set()).add(str(x))
        self._id_cache[x] = local
        return local

    def assert_no_id_collisions(self):
        """Fail loudly if two distinct context-quantity URIs collapse to one generated id: that would
        silently merge unrelated `shared.*` fields, and a genuinely shared quantity has one URI.
        """
        collisions = {i: sorted(u) for i, u in self._id_sources.items() if len(u) > 1}
        if collisions:
            details = "\n".join(f"  '{i}' <- {', '.join(u)}" for i, u in sorted(collisions.items()))
            raise ValueError(
                "id collision(s): distinct URIs map to one generated id and would be silently "
                "merged (e.g. a context-quantity name reused across motions with different "
                "definitions). Give them distinct names, or extend the motion-qualified id "
                f"scoping in Parser.id to cover their URI shape:\n{details}"
            )

    def label(self, x):
        """Human-readable label of a node, if the graph carries one."""
        try:
            q = self.g.compute_qname(x)
            return q[2]
        except Exception:
            return str(x)

    @memoize
    def velocity_composition_solver(self, id_):
        """Parse a VelocityCompositionSolver at node."""
        self._expect_type(id_, SLV["VelocityCompositionSolver"])
        conf = self.id(self.g.value(id_, SLV["configuration"]))
        velocity = self.velocity_twist(self.g.value(id_, SLV["velocity"]))

        return VelocityCompositionSolver(self.id(id_), conf, velocity)

    @memoize
    def force_distribution_solver(self, id_):
        """Parse a ForceDistributionSolver at node."""
        self._expect_type(id_, SLV["ForceDistributionSolver"])
        conf = self.id(self.g.value(id_, SLV["configuration"]))
        force = self.wrench(self.g.value(id_, SLV["force"]))

        return ForceDistributionSolver(self.id(id_), conf, force)

    @memoize
    def solver_with_input_and_output(self, id_):
        """Parse a SolverWithInputAndOutput (chain, algorithm, drivers, outputs) at node."""
        self._expect_type(id_, SLV["SolverWithInputAndOutput"])
        io_dispatcher = [
            (GEOM_COORD["PoseCoordinate"], self.pose),
            (GEOM_COORD["VelocityTwistCoordinate"], self.velocity_twist),
            (KC_STAT["JointPositionCoordinate"], self.joint_position),
            (RBDYN_COORD["WrenchCoordinate"], self.wrench),
        ]

        drv = []
        for motion_driver in self.g[id_ : SLV["motion-drivers"]]:
            drv.append(self.motion_drivers(motion_driver))
        out = []
        for o in self.g[id_ : SLV["output"]]:
            for type_, func in io_dispatcher:
                if type_ in get_node_types(self.g, o):
                    out.append(func(o))

        # The Vereshchagin root acceleration, passed to ACHD as-is; RNE's opposite-sign
        # gravity is derived in _annotate_rne_gravity.
        gravity_node = self.g.value(id_, SLV.gravity)
        root_acc = self.parse_xyz(gravity_node) if gravity_node else None
        algorithm_node = self.g.value(id_, SLV["solver"])
        if algorithm_node is None:
            algorithm = ""
        else:
            try:
                algorithm = SOLVER_SEMANTICS_BY_ALGORITHM[algorithm_node].codegen_name
            except KeyError as exc:
                raise ValueError(
                    f"Solver '{id_}' has unsupported algorithm '{algorithm_node}'."
                ) from exc
        torque_saturation_node = next(
            (
                limit
                for limit in self.g.objects(id_, ALGO_EXT.limits)
                if QUDT_QKIND.Torque
                in self.g[self.g.value(limit, ALGO_EXT["in"]) : QUDT_SCHEMA.hasQuantityKind]
            ),
            None,
        )

        return SolverWithInputAndOutput(
            id=self.id(id_),
            motion_drivers=drv,
            output=out,
            algorithm=algorithm,
            root_acc=root_acc,
            torque_saturation=(
                self.saturation(torque_saturation_node)
                if torque_saturation_node is not None
                else None
            ),
        )

    @memoize
    def motion_drivers(self, id_):
        """Parse authored Cartesian and joint-force drivers at node."""
        self._expect_type(id_, SLV["MotionDrivers"])
        spec_frc = []
        spec_jf = []

        for f in self.g[id_ : SLV["cartesian-force"]]:
            spec_frc.append(self.cartesian_force_specification(f))

        for jf in self.g[id_ : SLV["joint-force"]]:
            spec_jf.append(self.joint_force_specification(jf))

        return MotionDrivers(
            id=self.id(id_),
            acceleration_constraint=[],
            cartesian_force=spec_frc,
            joint_force=spec_jf,
            has_cartesian_force=bool(spec_frc),
        )

    def joint_force_specification(self, id_):
        """Parse a JointForceSpecification at node."""
        self._expect_type(id_, SLV["JointForceSpecification"])
        force_node = self.g.value(id_, SLV["force"])
        force_id = self.id(force_node) if force_node is not None else ""
        joint_node = self.g.value(id_, SLV["attached-to"])
        joint_name = self.label(joint_node) if joint_node is not None else ""
        return JointForceSpecification(self.id(id_), force_id, joint_name)

    @memoize
    def cartesian_force_specification(self, id_):
        """Parse a CartesianForceSpecification at node."""
        self._expect_type(id_, SLV["CartesianForceSpecification"])
        force = self.wrench(self.g.value(id_, SLV["force"]))
        attached_to = self.simplicial_complex(self.g.value(id_, SLV["attached-to"]))

        return CartesianForceSpecification(self.id(id_), force, attached_to)

    @memoize
    def saturation(self, id_):
        """Parse a Saturation (input/output limits) at node."""
        self._expect_type(id_, ALGO_EXT.Saturation)
        input_signal = self.quantity(self.g.value(id_, ALGO_EXT["in"]))
        output_signal = self.quantity(self.g.value(id_, ALGO_EXT.out))
        maximum_node = self.g.value(id_, ALGO_EXT["maximum-absolute-value"])
        lower_node = self.g.value(id_, ALGO_EXT["lower-bound"])
        upper_node = self.g.value(id_, ALGO_EXT["upper-bound"])
        return Saturation(
            self.id(id_),
            input_signal,
            output_signal,
            self.quantity(maximum_node) if maximum_node is not None else None,
            self.quantity(lower_node) if lower_node is not None else None,
            self.quantity(upper_node) if upper_node is not None else None,
        )

    @memoize
    def subspace(self, id_):
        """Parse the Subspace (linear/angular half) of node."""
        d = {
            MAP["position"]: Subspace.Linear,
            MAP_EXT["position"]: Subspace.Linear,
            MAP_EXT["orientation"]: Subspace.Angular,
            MAP["angular-velocity"]: Subspace.Angular,
            MAP["linear-velocity"]: Subspace.Linear,
            MAP["angular-acceleration"]: Subspace.Angular,
            MAP["linear-acceleration"]: Subspace.Linear,
            MAP["torque"]: Subspace.Angular,
            MAP["force"]: Subspace.Linear,
            SLV["angular-acceleration"]: Subspace.Angular,
            SLV["linear-acceleration"]: Subspace.Linear,
            MAP_EXT["linear"]: Subspace.Linear,
            MAP_EXT["angular"]: Subspace.Angular,
        }
        if id_ not in d:
            raise ValueError(f"unknown subspace {id_}")

        return d[id_]

    @memoize
    def axis(self, id_):
        """Parse the Axis (X/Y/Z) of node."""
        d = {
            MAP["x"]: Axis.X,
            MAP["y"]: Axis.Y,
            MAP["z"]: Axis.Z,
            MAP["w"]: Axis.W,
            SLV["x"]: Axis.X,
            SLV["y"]: Axis.Y,
            SLV["z"]: Axis.Z,
        }
        if id_ not in d:
            raise ValueError(f"unknown axis {id_}")

        return d[id_]

    def constraint_handler(self, id_):
        """Parse a ConstraintHandler (evaluators, controllers, monitors) at node."""
        self._expect_type(id_, CSTR_HDL["ConstraintHandler"])
        motion = self.guarded_motion(self.g.value(id_, CSTR_HDL["motion"]))
        evaluators = []
        for e in self.g[id_ : CSTR_HDL["evaluators"]]:
            evaluators.append(self.constraint_evaluator(e))

        monitors = []
        for m in self.g[id_ : CSTR_HDL["monitors"]]:
            monitors.append(self.monitor_entry(m))

        order_value = self.g.value(id_, APP["order"])
        order = int(order_value.value) if order_value is not None else 0

        return ConstraintHandler(self.id(id_), motion, evaluators, [], monitors, order)

    @memoize
    def monitor_entry(self, id_):
        """Parse a monitor (level flag or edge event) at node."""
        self._expect_type(id_, CSTR_HDL["Monitor"])
        handler = next(self.g.subjects(CSTR_HDL.monitors, id_), None)
        motion = self.g.value(handler, CSTR_HDL.motion) if handler is not None else None
        monitored = set(self.g.objects(id_, CSTR_HDL.constraint))
        when = set(self.g.objects(motion, MOT.when)) if motion is not None else set()
        until = set(self.g.objects(motion, MOT.until)) if motion is not None else set()
        is_aggregate = len(monitored) > 1 or any(
            _is_constraint_aggregate(self.g, node) for node in monitored
        )
        is_until_aggregate = is_aggregate and monitored == until
        is_when_aggregate = is_aggregate and monitored == when
        # A named group is one of several until conditions, so it is not the whole section:
        # carry its members and logic instead, and let the terms be built from those.
        group_constraint_ids: list[str] = []
        group_any = False
        if not is_until_aggregate and not is_when_aggregate:
            group_node = next(
                (n for n in monitored if _is_constraint_aggregate(self.g, n)), None
            )
            if group_node is not None:
                group_constraint_ids = sorted(
                    self.id(c) for c in self.g[group_node : CSTR_EXT["has-constraint"]]
                )
                group_any = CSTR_EXT.ConstraintDisjunction in get_node_types(self.g, group_node)
        error_node = self.g.value(id_, CSTR_HDL["error"])
        error = (
            None
            if is_until_aggregate or is_when_aggregate or group_constraint_ids or error_node is None
            else self.quantity(error_node)
        )
        # The band belongs to the constraint, so a monitor carries it only when it watches one.
        tolerance_node = (
            self.g.value(next(iter(monitored)), CSTR_EXT["tolerance"])
            if error is not None and len(monitored) == 1
            else None
        )
        tolerance = self.quantity(tolerance_node) if tolerance_node is not None else None

        if CSTR_HDL["LevelTriggeredMonitor"] in get_node_types(self.g, id_):
            flag = self.id(self.g.value(id_, CSTR_HDL["flag"]))
            return LevelMonitor(
                self.id(id_),
                "LevelTriggeredMonitor",
                error,
                flag,
                tolerance=tolerance,
                is_until_aggregate=is_until_aggregate,
                is_when_aggregate=is_when_aggregate,
                group_constraint_ids=group_constraint_ids,
                group_any=group_any,
            )

        event_node = self.g.value(id_, CSTR_HDL["event"])
        event = self.id(event_node)
        fallback_node = self.g.value(id_, CSTR_HDL_EXT["fallback-motion"])
        fallback_motion = self.id(fallback_node) if fallback_node is not None else None
        debounce_duration_s = self._optional_seconds(id_, CSTR_HDL_EXT["debounce-duration"])
        ros_kwargs = {}
        ros_channel = self.g.value(id_, NS_MM_ROS["channel-name"])
        if ros_channel is not None:
            ros_type = str(self.g.value(id_, NS_MM_ROS["type-name"]) or "")
            pkg, include, cpp_type = _ros_type_parts(ros_type)
            ros_kwargs = dict(
                ros_channel=str(ros_channel),
                ros_type=ros_type,
                ros_pkg=pkg,
                ros_include=include,
                ros_cpp_type=cpp_type,
                ros_pub_id=f"{self.id(id_)}_pub".replace("-", "_"),
            )
        return EdgeMonitor(
            self.id(id_),
            "EdgeTriggeredMonitor",
            error,
            event,
            None,
            tolerance=tolerance,
            is_until_aggregate=is_until_aggregate,
            is_when_aggregate=is_when_aggregate,
            group_constraint_ids=group_constraint_ids,
            group_any=group_any,
            event_uri=str(event_node),
            event_name=event.upper(),
            fallback_motion=fallback_motion,
            debounce_duration_s=debounce_duration_s,
            **ros_kwargs,
        )

    @memoize
    def constraint_evaluator(self, id_):
        """Parse a ConstraintEvaluator (constraint, error, elapsed timing) at node."""
        self._expect_type(id_, CSTR_HDL["ConstraintEvaluator"])
        constraint_node = self.g.value(id_, CSTR_HDL["constraint"])
        constraint = self.constraint(constraint_node)

        if CSTR_HDL["AssignmentEvaluator"] in get_node_types(self.g, id_):
            t = EvaluatorType.AssignmentEvaluator
            error = None
        else:
            t = EvaluatorType.ErrorEvaluator
            error = self.quantity(self.g.value(id_, CSTR_HDL["error"]))

        # Timing constraint: no solver error; codegen compares the world clock to the threshold.
        is_elapsed = False
        elapsed_op = None
        elapsed_threshold_s = None
        elapsed_tolerance_s = None
        if _is_elapsed_constraint(self.g, constraint_node):
            is_elapsed = True
            types = get_node_types(self.g, constraint_node)
            if CSTR["GreaterThanConstraint"] in types:
                elapsed_op = ">="
                thr = self.g.value(constraint_node, CSTR["threshold"])
                elapsed_threshold_s = _duration_seconds(self.g, thr)
            elif CSTR["EqualityConstraint"] in types:
                elapsed_op = "=="
                thr = self.g.value(constraint_node, CSTR["reference-value"])
                elapsed_threshold_s = _duration_seconds(self.g, thr)
                tol = self.g.value(constraint_node, CSTR_EXT["tolerance"])
                elapsed_tolerance_s = _duration_seconds(self.g, tol)
            else:
                elapsed_op = "<"
                thr = self.g.value(constraint_node, CSTR["threshold"])
                elapsed_threshold_s = _duration_seconds(self.g, thr)

        # An authored band on a spatial equality; the elapsed branch reads its own, in seconds,
        # because a duration's magnitude rides on qudt rather than on a shared value.
        tolerance_node = None if is_elapsed else self.g.value(constraint_node, CSTR_EXT["tolerance"])
        tolerance = self.quantity(tolerance_node) if tolerance_node is not None else None

        return ConstraintEvaluator(
            self.id(id_),
            t,
            constraint,
            error,
            tolerance=tolerance,
            is_elapsed=is_elapsed,
            elapsed_op=elapsed_op,
            elapsed_threshold_s=elapsed_threshold_s,
            elapsed_tolerance_s=elapsed_tolerance_s,
        )

    def _optional_float(self, subject, predicate) -> float | None:
        """Read an optional float-valued property, or None when absent."""
        value = self.g.value(subject, predicate)
        if value is None:
            return None
        literal = (
            value if isinstance(value, rdflib.Literal) else self.g.value(value, QUDT_SCHEMA.value)
        )
        if literal is None:
            raise ValueError(
                f"Controller '{self.id(subject)}' property '{self.id(predicate)}' must be a literal or a node with qudt:value."
            )
        return float(literal.value)

    def _optional_seconds(self, subject, predicate) -> float | None:
        """Read an optional duration in seconds, converting from the unit it was written in."""
        node = self.g.value(subject, predicate)
        if node is None:
            return None
        value = self._optional_float(subject, predicate)
        return None if value is None else _seconds(value, self.g.value(node, QUDT_SCHEMA.unit))

    def _required_float(self, subject, predicate) -> float:
        """Read a required float-valued property, raising when absent."""
        value = self._optional_float(subject, predicate)
        if value is None:
            raise ValueError(
                f"Controller '{self.id(subject)}' is missing required property '{self.id(predicate)}'."
            )
        return value

    @memoize
    def guarded_motion(self, id_):
        """Parse a GuardedMotion (when/while/until constraint sets) at node."""
        self._expect_type(id_, MOT["GuardedMotion"])
        when = []
        when_any = False
        for c in self.g[id_ : MOT["when"]]:
            if CSTR_EXT.ConstraintDisjunction in get_node_types(self.g, c):
                when_any = True
                for member in self.g[c : CSTR_EXT["has-constraint"]]:
                    when.append(self.constraint(member))
            else:
                when.append(self.constraint(c))

        while_ = []
        for c in self.g[id_ : MOT["while"]]:
            while_.append(self.constraint(c))

        until = []
        until_any = False
        for c in self.g[id_ : MOT["until"]]:
            if _is_constraint_aggregate(self.g, c):
                # A section-wide disjunction makes the whole until 'any'; a named group keeps
                # its own logic on the monitor that targets it.
                if CSTR_EXT.ConstraintDisjunction in get_node_types(self.g, c):
                    until_any = True
                for member in self.g[c : CSTR_EXT["has-constraint"]]:
                    until.append(self.constraint(member))
            else:
                until.append(self.constraint(c))

        name_literal = self.g.value(id_, SDO.name)
        if name_literal is None:
            raise ValueError(f"GuardedMotion {id_} has no schema:name triple")
        name = str(name_literal)
        description = self.g.value(id_, SDO.description)
        return GuardedMotion(
            self.id(id_), when, while_, until, until_any, when_any,
            name=name, description=str(description) if description is not None else None,
        )

    @memoize
    def constraint(self, id_):
        """Parse a Constraint (quantity plus its parameter) at node."""
        self._expect_type(id_, CSTR["Constraint"])
        quantity = self.quantity(self.g.value(id_, CSTR["quantity"]))

        parameter = None
        if CSTR["EqualityConstraint"] in get_node_types(self.g, id_):
            parameter = self.equality_constraint(id_)
        elif CSTR["UnilateralConstraint"] in get_node_types(self.g, id_):
            parameter = self.unilateral_constraint(id_)
        elif CSTR_EXT["OutsideConstraint"] in get_node_types(self.g, id_):
            parameter = self.outside_constraint(id_)
        else:
            parameter = self.bilateral_constraint(id_)

        return Constraint(self.id(id_), quantity, parameter)

    @memoize
    def equality_constraint(self, id_):
        """Parse an EqualityConstraint at node."""
        self._expect_type(id_, CSTR["EqualityConstraint"])
        reference_value = self.quantity(self.g.value(id_, CSTR["reference-value"]))

        return EqualityConstraint(reference_value)

    @memoize
    def unilateral_constraint(self, id_):
        """Parse a UnilateralConstraint (greater/less threshold) at node."""
        self._expect_type(id_, CSTR["UnilateralConstraint"])
        threshold = self.quantity(self.g.value(id_, CSTR["threshold"]))
        type_ = UnilateralConstraintType.LessThan
        if CSTR["GreaterThanConstraint"] in get_node_types(self.g, id_):
            type_ = UnilateralConstraintType.GreaterThan

        return UnilateralConstraint(type_, threshold)

    @memoize
    def bilateral_constraint(self, id_):
        """Parse a BilateralConstraint (lower/upper threshold) at node."""
        self._expect_type(id_, CSTR["BilateralConstraint"])
        lower_threshold = self.quantity(self.g.value(id_, CSTR["lower-threshold"]))
        upper_threshold = self.quantity(self.g.value(id_, CSTR["upper-threshold"]))

        return BilateralConstraint(lower_threshold, upper_threshold)

    @memoize
    def outside_constraint(self, id_):
        """Parse an OutsideConstraint (lower/upper threshold) at node."""
        self._expect_type(id_, CSTR_EXT["OutsideConstraint"])
        lower_threshold = self.quantity(self.g.value(id_, CSTR["lower-threshold"]))
        upper_threshold = self.quantity(self.g.value(id_, CSTR["upper-threshold"]))

        return OutsideConstraint(lower_threshold, upper_threshold)

    @memoize
    def direction(self, id_):
        """Parse a Direction quantity at node."""
        self._expect_type(id_, GEOM_COORD["DirectionCoordinate"])
        self._expect_type(id_, GEOM_COORD["VectorXYZ"])
        quantity_kind = []
        for k in self.g[id_ : QUDT_SCHEMA["hasQuantityKind"]]:
            quantity_kind.append(self.id(k))
        as_seen_by = self.frame(self.g.value(id_, GEOM_COORD["as-seen-by"]))
        unit = self.id(self.g.value(id_, QUDT_SCHEMA["unit"]))
        direction = self.parse_xyz(id_)

        return Direction(self.id(id_), quantity_kind, as_seen_by, [Unit(unit)], direction)

    def parse_xyz(self, node):
        """Parse x/y/z scalar coordinates from node on SI, or None."""
        x = self.g.value(node, GEOM_COORD["x"])
        y = self.g.value(node, GEOM_COORD["y"])
        z = self.g.value(node, GEOM_COORD["z"])

        if x is None or y is None or z is None:
            return None

        return _si_all((v.value for v in (x, y, z)), self.g.value(node, QUDT_SCHEMA["unit"]))

    def _derived_reference_frames(self, id_):
        """Derive a reference value's frames from its RDF use or snapshot source."""
        source = next(
            (
                self.g.value(snapshot, ALGO_EXT["in"])
                for snapshot in self.g.subjects(ALGO_EXT["out"], id_)
            ),
            None,
        )
        if source is None:
            constraint = next(self.g.subjects(CSTR["reference-value"], id_), None)
            quantity = self.g.value(constraint, CSTR.quantity) if constraint is not None else None
            view = next(self.g.subjects(MAP.subobject, quantity), None)
            source = self.g.value(view, MAP.superobject) if view is not None else quantity
        if source is None:
            return None, None, None
        return (
            self.g.value(source, GEOM_REL.of),
            self.g.value(source, GEOM_REL["with-respect-to"]),
            self.g.value(source, GEOM_COORD["as-seen-by"]),
        )

    @memoize
    def position(self, id_):
        """Parse a Position quantity at node."""
        if URI_GEOM_TYPE_POSITION_COORD in get_node_types(self.g, id_):
            coordinate = PositionCoordModel(id_, self.g)
            relation = coordinate.position
        else:
            relation = PositionModel(id_, self.g)
            if len(relation.coordinate_ids) != 1:
                raise ValueError(f"Position '{id_}' needs exactly one coordinate")
            coordinate = PositionCoordModel(next(iter(relation.coordinate_ids)), self.g, relation)
        of = self.position_reference(relation.of_id)
        wrt = self.position_reference(relation.wrt_id)
        quantity_kind = self.id(self.g.value(id_, QUDT_SCHEMA["hasQuantityKind"]))
        as_seen_by = self.frame(coordinate.as_seen_by)
        length_unit = _length_unit(coordinate)
        unit = self.id(_si_unit(length_unit))
        values = get_coord_vectorxyz(coordinate, self.g)
        pos = _si_all(values, length_unit) if values is not None else None

        return Position(
            self.id(id_), of, wrt, QuantityKind(quantity_kind), as_seen_by, Unit(unit), pos
        )

    @memoize
    def orientation(self, id_):
        """Parse an Orientation quantity at node."""
        if URI_GEOM_TYPE_ORIENT_COORD in get_node_types(self.g, id_):
            coordinate = OrientCoordModel(id_, self.g)
            relation = coordinate.relation
        else:
            relation = OrientationModel(id_, self.g)
            if len(relation.coordinate_ids) != 1:
                raise ValueError(f"Orientation '{id_}' needs exactly one coordinate")
            coordinate = OrientCoordModel(next(iter(relation.coordinate_ids)), self.g, relation)

        def optional_pose_ref(node):
            """Resolve an optional pose reference (endpoint or bare pose) at node."""
            if node is None:
                return None
            if ENV.RigidObject in get_node_types(self.g, node):
                return self.scene_object(node)
            if GEOM_ENT.Frame in get_node_types(self.g, node):
                return self.frame(node)
            return None

        of = optional_pose_ref(relation.of_id)
        wrt = optional_pose_ref(relation.wrt_id)
        quantity_kind = self.id(self.g.value(id_, QUDT_SCHEMA["hasQuantityKind"]))
        as_seen_by = self.frame(coordinate.as_seen_by.id)
        units = set(self.g.objects(coordinate.id, QUDT_SCHEMA["unit"]))
        if self._orientation_composition(coordinate.id) is not None or coordinate.types & {
            URI_GEOM_TYPE_QUATERNION,
            URI_GEOM_TYPE_DIRECTION_COSINE_XYZ,
        }:
            unit = self.id(QUDT_UNIT.UNITLESS)
        else:
            angular_units = units & {URI_QUDT_UNIT_RAD, URI_QUDT_UNIT_DEG}
            if len(angular_units) != 1:
                raise ConstraintViolation(
                    "geometry",
                    f"OrientationCoordinate '{coordinate.id}' needs exactly one angular unit, "
                    f"found {angular_units}",
                )
            unit = self.id(_si_unit(next(iter(angular_units))))
        axes = self.g.value(coordinate.id, URI_GEOM_PRED_AXES_SEQ)
        provenance = self.quantity_provenance(id_)
        return Orientation(
            self.id(id_),
            of,
            wrt,
            QuantityKind(quantity_kind),
            as_seen_by,
            Unit(unit),
            str(axes) if axes is not None else "xyz",
            (id_, ~MAP["subobject"], None) in self.g,
            provenance=provenance,
        )

    def position_reference(self, id_):
        """A Position is of a Point with respect to a Point (geometry metamodel)."""
        if id_ is None:
            return None
        if GEOM_ENT.Point in get_node_types(self.g, id_):
            return self.point(id_)
        if GEOM_ENT.Frame in get_node_types(self.g, id_):
            return Point(self.id(id_))
        raise ValueError(f"Position reference must be a Point, got: {id_}")

    @memoize
    def _pose_endpoint(self, node):
        """Resolve a pose endpoint (frame/scene-object) to its id."""
        if node is None:
            return None
        if ENV.RigidObject in get_node_types(self.g, node):
            return self.scene_object(node)
        return self.frame(node)

    def _bare_pose(self, id_):
        # A geom-rel:Pose with no PoseCoordinate: a snapshot/reference pose filled at runtime.
        # Same IR shape as a coordinate pose, with frame endpoints but no authored values.
        """Parse a bare Pose (no view) at node."""
        of_node = self.g.value(id_, GEOM_REL["of"])
        wrt_node = self.g.value(id_, GEOM_REL["with-respect-to"])
        as_seen_by_node = self.g.value(id_, GEOM_COORD["as-seen-by"])
        if of_node is None or wrt_node is None or as_seen_by_node is None:
            inherited_of, inherited_wrt, inherited_as_seen_by = self._derived_reference_frames(id_)
            of_node = of_node or inherited_of
            wrt_node = wrt_node or inherited_wrt
            as_seen_by_node = as_seen_by_node or inherited_as_seen_by
        provenance = self.quantity_provenance(id_)
        return Pose(
            self.id(id_),
            self._pose_endpoint(of_node),
            self._pose_endpoint(wrt_node),
            [self.id(k) for k in self.g[id_ : QUDT_SCHEMA["hasQuantityKind"]]],
            self.frame(as_seen_by_node) if as_seen_by_node is not None else None,
            [self.id(u) for u in self.g[id_ : QUDT_SCHEMA["unit"]]],
            None,
            None,
            None,
            None,
            provenance=provenance,
        )

    def pose(self, id_):
        """Parse a Pose quantity (endpoints, orientation, position) at node."""
        coordinate = PoseCoordModel(id_, self.g)
        relation = coordinate.relation
        of = self._pose_endpoint(relation.of_id)
        wrt = self._pose_endpoint(relation.wrt_id)
        quantity_kind = [
            self.id(k) for k in self.g[relation.id : QUDT_SCHEMA["hasQuantityKind"]]
        ]
        as_seen_by = self.frame(coordinate.as_seen_by.id)
        unit = list(
            dict.fromkeys(
                self.id(_si_unit(u))
                for component in (coordinate.position_coord.id, coordinate.orientation_coord.id)
                for u in self.g[component : QUDT_SCHEMA["unit"]]
            )
        )
        length_unit = _length_unit(coordinate.position_coord)
        position_values = get_coord_vectorxyz(coordinate.position_coord, self.g)
        pos = _si_all(position_values, length_unit) if position_values is not None else None
        orientation_node = coordinate.orientation_coord.id
        representation = self.orientation_representation(orientation_node)
        # A symbolic triple keeps its convention: the backend composes per-axis quaternions,
        # and the sequence decides both the axes and the order they multiply in.
        euler_axes_sequence = None
        euler_intrinsic = False
        if representation == "euler":
            axes = self.g.value(orientation_node, URI_GEOM_PRED_AXES_SEQ)
            euler_axes_sequence = str(axes) if axes is not None else None
            euler_intrinsic = URI_GEOM_TYPE_INTRINSIC in coordinate.orientation_coord.types

        provenance = self.quantity_provenance(id_)
        if not provenance.snapshot:
            provenance = Provenance(
                authored=any(
                    self._is_authored(component.id)
                    for component in (coordinate.position_coord, coordinate.orientation_coord)
                )
                or (
                    bool(
                        coordinate.orientation_coord.types
                        & {
                            URI_GEOM_TYPE_EULER_ANGLES,
                            URI_GEOM_TYPE_QUATERNION,
                            URI_GEOM_TYPE_DIRECTION_COSINE_XYZ,
                        }
                    )
                    or self._orientation_composition(coordinate.orientation_coord.id) is not None
                    and any(
                        self.g.value(view, MAP["axis"]) is not None
                        for view in self.g.subjects(MAP["superobject"], id_)
                    )
                ),
                snapshot=False,
            )
        rel_operands = None
        if representation == "relative":
            rel_operands = self._relative_orientation(orientation_node)
        return Pose(
            self.id(id_),
            of,
            wrt,
            quantity_kind,
            as_seen_by,
            unit,
            pos,
            euler_axes_sequence,
            euler_intrinsic,
            representation,
            orientation_operands=rel_operands,
            provenance=provenance,
        )

    def _orientation_composition(self, orientation_node):
        """The `geom-op-ext:ComposeOrientation` operator writing into this orientation, if any."""
        if orientation_node is None:
            return None
        return next(
            (
                operation
                for operation in self.g.subjects(GEOM_OP["composite"], orientation_node)
                if GEOM_OP_EXT.ComposeOrientation in get_node_types(self.g, operation)
            ),
            None,
        )

    def _relative_orientation(self, orientation_node):
        """The composition's two operands, in `geom-op:in1`/`in2` order: each is either
        `{"pose": <id>}` (the base, by id) or `{"delta": [...], "representation": ...}` (the
        delta's ordered component values and rotation representation)."""
        composition = self._orientation_composition(orientation_node)
        if composition is None:
            raise ValueError(
                f"Relative orientation '{orientation_node}' has no composition operator."
            )
        in1 = self.g.value(composition, GEOM_OP["in1"])
        in2 = self.g.value(composition, GEOM_OP["in2"])
        if in1 is None or in2 is None:
            raise ValueError(
                f"Orientation composition '{composition}' must declare both operands."
            )

        def _operand(node):
            types = get_node_types(self.g, node)
            if URI_GEOM_TYPE_POSE_COORD in types:
                return {"pose": self.id(node)}
            representation_types = {
                URI_GEOM_TYPE_ANGLES_ABG,
                URI_GEOM_TYPE_QUATERNION,
                URI_GEOM_TYPE_DIRECTION_COSINE_XYZ,
            }
            if types & representation_types:
                # A delta is literal by construction, so it folds to a quaternion here.
                rotation = get_orientation_coord_vals(ModelBase(node_id=node, graph=self.g), self.g)
                if rotation is None:
                    raise ValueError(
                        f"Relative orientation delta '{node}' has no literal components"
                    )
                return {
                    "delta": [{"value": float(value)} for value in rotation.as_quat()],
                    "representation": "quaternion",
                }
            raise ValueError(
                f"Relative orientation operand '{node}' is neither a pose nor an orientation"
            )

        operands = [_operand(in1), _operand(in2)]
        if sum("pose" in op for op in operands) != 1 or sum("delta" in op for op in operands) != 1:
            raise ValueError(
                f"Relative orientation '{orientation_node}' must compose exactly one base pose "
                "and one delta rotation."
            )
        return operands

    def orientation_representation(self, id_):
        """How an orientation's components arrive, not how the model wrote them.

        All-literal rotations resolve to the same quaternion however they were authored. What cannot be
        resolved ahead of time keeps its own shape: `euler` is a triple whose angles arrive at runtime,
        `relative` composes around a runtime pose.
        """
        if id_ is None:
            return "quaternion"
        types = get_node_types(self.g, id_)
        if self._orientation_composition(id_) is not None:
            return "relative"
        if URI_GEOM_TYPE_EULER_ANGLES in types and URI_GEOM_TYPE_ANGLES_ABG not in types:
            return "euler"
        return "quaternion"

    @memoize
    def velocity_twist(self, id_):
        """Parse a VelocityTwist quantity at node."""
        self._expect_type(id_, GEOM_COORD["VelocityTwistCoordinate"])
        self._expect_type(id_, GEOM_COORD["VectorXYZ"])
        of = self.simplicial_complex(self.g.value(id_, GEOM_REL["of"]))
        wrt = self.simplicial_complex(self.g.value(id_, GEOM_REL["with-respect-to"]))
        quantity_kind = []
        for k in self.g[id_ : QUDT_SCHEMA["hasQuantityKind"]]:
            quantity_kind.append(self.id(k))
        reference_point = self.point(self.g.value(id_, GEOM_REL["reference-point"]))
        as_seen_by = self.frame(self.g.value(id_, GEOM_COORD["as-seen-by"]))
        unit = []
        for u in self.g[id_ : QUDT_SCHEMA["unit"]]:
            unit.append(self.id(u))

        provenance = self.quantity_provenance(id_)
        return VelocityTwist(
            self.id(id_),
            of,
            wrt,
            quantity_kind,
            reference_point,
            as_seen_by,
            unit,
            provenance=provenance,
        )

    def _spatial_coordinate_fields(self, id_):
        """Shared field extraction for the 6D coordinate quantities: AccelerationTwist, PoseDifference
        and Wrench share one coordinate structure and differ only in RDF namespace.
        """
        quantity_kind = [self.id(k) for k in self.g[id_ : QUDT_SCHEMA["hasQuantityKind"]]]
        reference_point = self.point(self.g.value(id_, GEOM_REL["reference-point"]))
        as_seen_by = self.frame(self.g.value(id_, GEOM_COORD["as-seen-by"]))
        unit = [self.id(u) for u in self.g[id_ : QUDT_SCHEMA["unit"]]]
        provenance = self.quantity_provenance(id_)
        return quantity_kind, reference_point, as_seen_by, unit, provenance

    @memoize
    def acceleration_twist(self, id_):
        """Parse an AccelerationTwist quantity at node."""
        self._expect_type(id_, GEOM_COORD["AccelerationTwistCoordinate"])
        self._expect_type(id_, GEOM_COORD["VectorXYZ"])
        qk, ref, seen, unit, provenance = self._spatial_coordinate_fields(id_)
        return AccelerationTwist(self.id(id_), qk, ref, seen, unit, provenance=provenance)

    @memoize
    def pose_difference(self, id_):
        """Parse a PoseDifference quantity at node."""
        self._expect_type(id_, GEOM_COORD["PoseDifferenceCoordinate"])
        self._expect_type(id_, GEOM_COORD["VectorXYZ"])
        qk, ref, seen, unit, provenance = self._spatial_coordinate_fields(id_)
        return PoseDifference(self.id(id_), qk, ref, seen, unit, provenance=provenance)

    @memoize
    def wrench(self, id_):
        """Parse a Wrench quantity (with any FT sensor) at node."""
        self._expect_type(id_, RBDYN_COORD["WrenchCoordinate"])
        relation = self.g.value(id_, RBDYN_COORD["of-wrench"])
        if relation is None or RBDYN_ENT.Wrench not in get_node_types(self.g, relation):
            raise ConstraintViolation(
                "dynamics", f"WrenchCoordinate '{id_}' has no valid of-wrench relation"
            )
        qk = [self.id(k) for k in self.g[relation : QUDT_SCHEMA["hasQuantityKind"]]]
        reference = self.g.value(relation, RBDYN_ENT["reference-point"])
        seen_by = self.g.value(id_, RBDYN_COORD["as-seen-by"])
        if reference is None or seen_by is None:
            raise ConstraintViolation(
                "dynamics", f"WrenchCoordinate '{id_}' is missing reference-point/as-seen-by"
            )
        ref = self.point(reference)
        seen = self.frame(seen_by)
        unit = [self.id(u) for u in self.g[id_ : QUDT_SCHEMA["unit"]]]
        provenance = self.quantity_provenance(id_)
        sensor = self.g.value(id_, SOSA.madeBySensor)
        sensor_name = self.id(sensor) if sensor is not None else ""
        sensor_frame_node = self.g.value(sensor, SENSORS.frame) if sensor is not None else None
        if sensor is not None and sensor_frame_node is None:
            raise ConstraintViolation(
                "dynamics", f"WrenchCoordinate '{id_}' sensor '{sensor}' has no physical frame"
            )
        sensor_frame = self.frame(sensor_frame_node) if sensor_frame_node is not None else None
        return Wrench(
            self.id(id_), qk, ref, seen, unit, provenance, sensor_frame, sensor_name
        )

    def _is_duration(self, id_):
        """Authored durations carry the OWL-Time type; runtime elapsed time is a Time-kind
        quantity the clock fills, so it has a kind but no value."""
        if TIME["Duration"] in get_node_types(self.g, id_):
            return True
        return self.g.value(id_, QUDT_SCHEMA.hasQuantityKind) == QUDT_QTY["Time"]

    @memoize
    def quantity(self, id_):
        """Parse the quantity at node, dispatching on its RDF type."""
        if self._is_duration(id_):
            return self.duration_quantity(id_)
        self._expect_type(id_, QUDT_SCHEMA["Quantity"])
        quantity_kind_node = self.g.value(id_, QUDT_SCHEMA.hasQuantityKind)
        quantity_kind = self.id(quantity_kind_node)

        # Values below are converted, so the unit reported alongside them is the SI one.
        unit = self.id(_si_unit(self.g.value(id_, QUDT_SCHEMA["unit"])))
        has_view = (id_, ~MAP["subobject"], None) in self.g

        types = get_node_types(self.g, id_)
        if URI_GEOM_TYPE_POSE_COORD in types:
            return self.pose(id_)
        if URI_GEOM_TYPE_POSE in types:
            return self._bare_pose(id_)
        if URI_GEOM_TYPE_POSITION in types:
            return self.position(id_)
        if URI_GEOM_TYPE_ORIENT in types:
            return self.orientation(id_)
        if CSTR_HDL_EXT["SetpointGenerator"] in get_node_types(self.g, id_):
            value_kind_node = next(
                (k for k in self.g[id_ : QUDT_SCHEMA["hasQuantityKind"]] if k != CSTR_HDL_EXT.SetpointGenerator),
                None,
            )
            provenance = self.quantity_provenance(id_)
            return Setpoint(
                self.id(id_),
                QuantityKind(quantity_kind),
                Unit(unit),
                has_view,
                provenance=provenance,
                value_kind=self.id(value_kind_node) if value_kind_node is not None else None,
            )

        if quantity_kind == "FreeVector" and GEOM_COORD["VectorXYZ"] in get_node_types(
            self.g, id_
        ):
            provenance = self.quantity_provenance(id_)
            return FreeVector(
                self.id(id_),
                QuantityKind(quantity_kind),
                Unit(unit),
                self.parse_xyz(id_),
                has_view,
                provenance=provenance,
            )

        value = None
        if (id_, QUDT_SCHEMA["value"], None) in self.g:
            authored = self.g.value(id_, QUDT_SCHEMA["value"])
            value = _si(float(authored), self.g.value(id_, QUDT_SCHEMA["unit"]))
        reference_value = self.g.value(id_, CSTR["reference-value"])
        provenance = self.quantity_provenance(id_)
        return Quantity(
            self.id(id_),
            QuantityKind(quantity_kind),
            Unit(unit),
            value,
            has_view,
            provenance=provenance,
            reference_value=self.id(reference_value) if reference_value is not None else None,
        )

    @memoize
    def duration_quantity(self, id_):
        """Parse an authored duration or runtime elapsed-duration coordinate."""
        if not self._is_duration(id_):
            raise ValueError(f"Expected a duration at '{id_}'")
        # An elapsed coordinate has no authored value; the clock fills it at runtime.
        value_node = self.g.value(id_, QUDT_SCHEMA["value"])
        value = (
            None
            if value_node is None
            else _seconds(float(value_node), self.g.value(id_, QUDT_SCHEMA["unit"]))
        )
        provenance = self.quantity_provenance(id_)
        return Quantity(
            self.id(id_), QuantityKind("Duration"), Unit("Second"), value, False,
            provenance=provenance,
        )

    @memoize
    def joint_position(self, id_):
        """Parse a JointPosition quantity at node."""
        self._expect_type(id_, KC_STAT["JointPositionCoordinate"])
        self._expect_type(id_, KC_STAT["JointReference"])
        joint_node = self.g.value(id_, KC_STAT["of-joint"])
        if not isinstance(joint_node, URIRef):
            raise ConstraintViolation(
                "kinematic-chain", f"JointPositionCoordinate '{id_}' has no of-joint URI"
            )
        joint_name = self.label(joint_node) if joint_node is not None else ""
        return JointPosition(self.id(id_), joint_name)

    def quantity_provenance(self, id_):
        # authored and snapshot are mutually exclusive: snapshot wins. authored == carries an
        # authored value/coordinate and is not a runtime snapshot.
        """Parse a quantity's Provenance (authored / snapshot) at node."""
        snapshot = ALGO_EXT.Snapshot in get_node_types(self.g, id_)
        authored = (not snapshot) and self._is_authored(id_)
        return Provenance(authored=authored, snapshot=snapshot)

    def _is_authored(self, id_):
        """True when a quantity's value was authored by the user (not computed)."""
        # A path parameter's value is only its starting point; the traversal drives it per tick.
        if (None, GEOM_OP_EXT["path-parameter"], id_) in self.g:
            return False
        if (id_, QUDT_SCHEMA["value"], None) in self.g:
            return True
        if (id_, CSTR["reference-value"], None) in self.g:
            return True
        if any(
            (id_, predicate, None) in self.g
            for predicate in (
                URI_GEOM_PRED_X,
                URI_GEOM_PRED_Y,
                URI_GEOM_PRED_Z,
                URI_GEOM_PRED_W,
                URI_GEOM_PRED_ALPHA,
                URI_GEOM_PRED_BETA,
                URI_GEOM_PRED_GAMMA,
                URI_GEOM_PRED_DIRECTION_COSINE_X,
                URI_GEOM_PRED_DIRECTION_COSINE_Y,
                URI_GEOM_PRED_DIRECTION_COSINE_Z,
            )
        ):
            return True
        return False

    @memoize
    def simplicial_complex(self, id_):
        """Parse a SimplicialComplex, mapping a body-origin frame to its runtime body."""
        if not any(
            type_ in get_node_types(self.g, id_)
            for type_ in (GEOM_ENT.SimplicialComplex, GEOM_ENT.Frame)
        ):
            raise ValueError(f"Expected a rigid body or frame, got: {id_}")
        body = next(
            (
                owner
                for owner in self.g.subjects(GEOM_ENT.simplices, id_)
                if GEOM_ENT.RigidBody in get_node_types(self.g, owner)
            ),
            None,
        )
        if body is not None and self.id(id_) == f"{self.id(body)}_origin":
            return SimplicialComplex(self.id(body))
        return SimplicialComplex(self.id(id_))

    @memoize
    def scene_object(self, id_):
        """Parse a SceneObject at node."""
        self._expect_type(id_, ENV.RigidObject)
        return SceneObject(self.id(id_), self.id(id_))

    @memoize
    def frame(self, id_):
        """Parse a frame, mapping a scene-dsl body-origin frame to its runtime body."""
        self._expect_type(id_, GEOM_ENT["Frame"])
        body = next(
            (
                owner
                for owner in self.g.subjects(GEOM_ENT.simplices, id_)
                if GEOM_ENT.RigidBody in get_node_types(self.g, owner)
            ),
            None,
        )
        if body is not None and self.id(id_) == f"{self.id(body)}_origin":
            return Frame(self.id(body))
        return Frame(self.id(id_))

    @memoize
    def point(self, id_):
        """Parse a Point at node."""
        if not {GEOM_ENT.Point, GEOM_ENT.Frame} & get_node_types(self.g, id_):
            raise ValueError(f"Expected a point or frame, got: {id_}")
        return Point(self.id(id_))

    def view(self):
        """Parse a View (superobject, subobject, subspace, axis) at node."""
        dispatcher = [
            (MAP_EXT["PoseCoordinateView"], self.pose),
            (MAP_EXT["VelocityTwistCoordinateView"], self.velocity_twist),
            (MAP_EXT["AccelerationTwistCoordinateView"], self.acceleration_twist),
            (MAP_EXT["PoseDifferenceView"], self.pose_difference),
            (MAP_EXT["WrenchCoordinateView"], self.wrench),
        ]

        view_map = {}
        # Sorted: _views_for_access keeps the first view seen for a subobject, so an unordered
        # walk would publish a different (equivalent) view id on every generation.
        for view in sorted(self.g[: RDF["type"] : MAP["View"]]):
            superobject_id = self.g.value(view, MAP["superobject"])
            superobject = None
            for type_, func in dispatcher:
                if type_ not in get_node_types(self.g, view):
                    continue

                superobject = func(superobject_id)
                break

            if superobject is None:
                superobject = self.quantity(superobject_id)

            subobject = self.quantity(self.g.value(view, MAP["subobject"]))
            subspace = self.subspace(self.g.value(view, MAP["subspace"]))
            axis_node = self.g.value(view, MAP["axis"])
            axis = self.axis(axis_node) if axis_node is not None else None

            if superobject is None:
                raise ValueError(
                    f"MAP view {view} has an unrecognized type; no view dispatcher matched"
                )
            view_map[self.id(view)] = View(
                self.id(view), superobject, subobject, subspace, axis
            )

        return view_map

    def data_structures(self):
        """Parse every data-structure entity in the graph."""
        dispatcher = [
            (GEOM_COORD["DirectionCoordinate"], self.direction),
            # A combined PoseCoordinate is also a PositionCoordinate and an
            # OrientationCoordinate; dispatch the most specific type first.
            (GEOM_COORD["PoseCoordinate"], self.pose),
            (GEOM_COORD["PositionCoordinate"], self.position),
            (GEOM_COORD["OrientationCoordinate"], self.orientation),
            (GEOM_COORD["VelocityTwistCoordinate"], self.velocity_twist),
            (GEOM_COORD["AccelerationTwistCoordinate"], self.acceleration_twist),
            (GEOM_COORD["PoseDifferenceCoordinate"], self.pose_difference),
            (RBDYN_COORD["WrenchCoordinate"], self.wrench),
            (QUDT_SCHEMA["Quantity"], self.quantity),
        ]

        data_structures = []
        for type_, func in dispatcher:
            # Sort within the type group: keeps the most-specific-first dispatch the dedupe
            # relies on, while making the published order reproducible.
            for dstruct in sorted(self.g[: RDF["type"] : type_]):
                data_structures.append(func(dstruct))

        return _dedupe_by_id(data_structures)

    def _path_fields(self, path_node):
        """The path's geometry, as fields of the evaluator call that traverses it: the shape decides the
        maths, so the closure takes the path's type and carries its parameters directly.
        """
        path_types = get_node_types(self.g, path_node)
        spec = next((s for s in ops_path if s.type_ in path_types), None)
        if spec is None:
            return {}
        fields = {"type": self.id(spec.type_)}
        for input_ in spec.input:
            fields[self.id(input_)] = parse_argument(self.g, path_node, input_, self.id)
        for param in spec.parameters:
            fields[self.id(param)] = parse_argument(
                self.g, path_node, param, self.id, resolve_value=True
            )
        return fields

    def closures(self, operators):
        """Build the closure for each call of the given operators."""
        closures = {}
        for operator in operators:
            for closure in self.g.subjects(RDF["type"], operator.type_):
                cl = (
                    operator.closure_step(self.g, self.id, closure)
                    if hasattr(operator, "closure_step")
                    else None
                )
                if cl:
                    if operator.type_ in {
                        GEOM_OP_EXT.PathProjection,
                        GEOM_OP_EXT.PathTangentFrame,
                        GEOM_OP_EXT.PathEvaluator,
                    }:
                        if operator.type_ == GEOM_OP_EXT.PathEvaluator:
                            reference = self.g.value(closure, GEOM_OP.out)
                            if reference is not None:
                                cl["setpoint"] = self.id(reference)
                        fields = self._path_fields(self.g.value(closure, GEOM_OP_EXT.path))
                        # The geometry decides the maths; each caller samples the same curve.
                        cl["shape"] = fields.pop("type")
                        cl.update(fields)
                    if operator.type_ in {ALGO_EXT.VelocityProfile, ALGO_EXT.Admittance}:
                        reference = self.g.value(closure, ALGO_EXT.out)
                        constraint = next(
                            self.g.subjects(CSTR["reference-value"], reference), None
                        )
                        if constraint is None:
                            raise ValueError(
                                f"{self.id(operator.type_)} '{self.id(closure)}' output is not bound to a constraint."
                            )
                        # Evaluators carry cstr-hdl:constraint too, and can win this lookup;
                        # filter state belongs to the controller.
                        controller = next(
                            (
                                node
                                for node in self.g.subjects(CSTR_HDL.constraint, constraint)
                                if CSTR_HDL.ConstraintEvaluator
                                not in get_node_types(self.g, node)
                            ),
                            None,
                        )
                        if controller is None:
                            raise ValueError(
                                f"{self.id(operator.type_)} '{self.id(closure)}' constraint has no controller."
                            )
                        cl["controller"] = self.id(controller)
                        if operator.type_ == ALGO_EXT.VelocityProfile:
                            # A physical profile starts from the constraint's measured quantity.
                            cl["measured"] = self.id(self.g.value(constraint, CSTR.quantity))
                            cl["goal"] = cl.pop("target")
                    closures[self.id(closure)] = cl

        return closures

    def schedule(self, start, ops):
        """Build the dependency-ordered schedule for the given operators."""
        q = collections.deque()
        data_structures = set()
        sched = []
        scheduled_nodes = {}
        for v in start:
            v_types = get_node_types(self.g, v)
            for op in ops:
                if op.type_ not in v_types:
                    continue

                call = self.id(v)
                if op.schedulable and call not in self.sched:
                    sched.append(call)
                    scheduled_nodes[call] = v
                    self.sched.add(call)

                # sorted(): set order over rdflib nodes varies between processes, and traversal
                # order decides the emitted schedule order.
                for data_in in sorted(op.from_operator_to_input(self.g, v)):
                    q.append(data_in)
                    data_structures.add(data_in)

        # data_in --(in)--> call --(out)--> data_out, traversed backward.
        op_preds = [(_op_output_preds(op), op) for op in ops]
        while len(q) > 0:
            data_out = q.pop()
            preds_into = set(self.g.predicates(None, data_out))
            # Only ops whose output predicate points into data_out can match here.
            for out_set, op in op_preds:
                if out_set.isdisjoint(preds_into):
                    continue
                res = op.scheduler_step(self.g, data_out)

                for call_node in res["schedule"]:
                    call = self.id(call_node)
                    if call and call not in self.sched:
                        sched.append(call)
                        scheduled_nodes[call] = call_node
                        self.sched.add(call)

                for data_in in sorted(res["data_structures"]):
                    if data_in in data_structures:
                        continue

                    q.append(data_in)
                    data_structures.add(data_in)

            # An inline/declared Pose is no operator's output, so follow its per-axis views to
            # schedule the closures producing its components. sorted() for the reason above.
            view_subobjects = (
                self.g.value(view, MAP["subobject"])
                for view in sorted(self.g.subjects(MAP["superobject"], data_out))
                if self.g.value(view, MAP["axis"]) is not None
            )
            for successor in itertools.chain(
                (node for node in view_subobjects if node is not None),
                sorted(self.g.objects(data_out, CSTR["reference-value"])),
            ):
                if successor not in data_structures:
                    q.append(successor)
                    data_structures.add(successor)

        sched.reverse()
        return self._topological_schedule(sched, scheduled_nodes, ops)

    def _operator_outputs(self, node, op):
        """Output data ids produced by an operator call."""
        outputs = set()
        for out in getattr(op, "output", []):
            outputs.update(self.g.objects(node, out))
        return outputs

    def _topological_schedule(self, sched, scheduled_nodes, ops):
        """Order scheduled calls so producers run before their consumers."""
        if len(sched) < 2:
            return sched

        order = {call: index for index, call in enumerate(sched)}
        call_inputs: dict[str, set] = {}
        output_producer = {}

        for call in sched:
            node = scheduled_nodes.get(call)
            if node is None:
                continue
            inputs = set()
            outputs = set()
            node_types = get_node_types(self.g, node)
            for op in ops:
                if op.type_ not in node_types:
                    continue
                inputs.update(op.from_operator_to_input(self.g, node))
                outputs.update(self._operator_outputs(node, op))
            call_inputs[call] = inputs
            for output in outputs:
                output_producer.setdefault(output, call)

        deps = {
            call: {
                output_producer[data]
                for data in inputs
                if output_producer.get(data) is not None and output_producer[data] != call
            }
            for call, inputs in call_inputs.items()
        }

        result = []
        temporary = set()
        permanent = set()

        def visit(call):
            """Recurse operators feeding a data node, appending schedulable calls in dependency order."""
            if call in permanent:
                return
            if call in temporary:
                return
            temporary.add(call)
            for dep in sorted(deps.get(call, ()), key=lambda item: order.get(item, 0)):
                visit(dep)
            temporary.remove(call)
            permanent.add(call)
            result.append(call)

        for call in sched:
            visit(call)
        return result
_LENGTH_UNITS = {URI_QUDT_UNIT_M, URI_QUDT_UNIT_CM, URI_QUDT_UNIT_MM}
_TEMPORAL_UNITS = {QUDT_UNIT["SEC"], QUDT_UNIT["MilliSEC"]}
# The DSL records the unit a model was written in and never rescales a value, so putting one
# on SI is the reader's job -- codegen emits metres, radians and seconds. A unit absent here
# is already SI (N, N-M, M-PER-SEC2, KiloGM, UNITLESS, ...).
_SI_EQUIVALENT = {
    QUDT_UNIT["CentiM"]: (QUDT_UNIT["M"], 1e-2),
    QUDT_UNIT["MilliM"]: (QUDT_UNIT["M"], 1e-3),
    QUDT_UNIT["DEG"]: (QUDT_UNIT["RAD"], math.pi / 180.0),
    QUDT_UNIT["CentiM-PER-SEC"]: (QUDT_UNIT["M-PER-SEC"], 1e-2),
    QUDT_UNIT["DEG-PER-SEC"]: (QUDT_UNIT["RAD-PER-SEC"], math.pi / 180.0),
    QUDT_UNIT["DEG-PER-SEC2"]: (QUDT_UNIT["RAD-PER-SEC2"], math.pi / 180.0),
    QUDT_UNIT["MilliSEC"]: (QUDT_UNIT["SEC"], 1e-3),
}
def _si_unit(unit):
    """The SI unit `unit` converts to; `unit` itself when it already is SI."""
    return _SI_EQUIVALENT.get(unit, (unit, 1.0))[0]
def _length_unit(coordinate):
    """A position coordinate's length unit, rejecting anything that is not one."""
    if coordinate.unit not in _LENGTH_UNITS:
        raise ConstraintViolation(
            "geometry",
            f"Position coordinate '{coordinate.id}' has an unrecognized length "
            f"unit '{coordinate.unit}'.",
        )
    return coordinate.unit
def _si(value: float, unit) -> float:
    """`value`, authored in `unit`, on SI."""
    return value * _SI_EQUIVALENT.get(unit, (unit, 1.0))[1]
def _si_all(values, unit) -> list[float]:
    """Each of `values`, authored in `unit`, on SI."""
    return [_si(float(value), unit) for value in values]
def _seconds(value: float, unit) -> float:
    """A duration the model authored, in seconds."""
    if unit not in _TEMPORAL_UNITS:
        raise ConstraintViolation("units", f"'{unit}' is not a duration this can read")
    return _si(value, unit)
def _is_constraint_aggregate(g, node) -> bool:
    """True for an until/when group node: a conjunction or disjunction of constraints."""
    types = get_node_types(g, node)
    return bool({CSTR_EXT.ConstraintDisjunction, CSTR_EXT.ConstraintConjunction} & types)
def _uri_table(id_nodes):
    """Sorted [{id, uri}] rows for every id that maps to a URIRef."""
    return [
        {"id": id_, "uri": str(node)}
        for id_, node in sorted(id_nodes, key=lambda item: (item[0], str(item[1])))
        if isinstance(node, URIRef)
    ]
class DerivedIriRegistry:
    """IRIs for codegen-derived entities, minted as a path segment under the parent they came from.

    An id is a lossy projection of its IRI (Parser.id keeps only the local name), so a derived
    entity's IRI has to be recorded where the derivation happens, against the parent still in hand.
    """

    SPECIALIZATION = "specializationOf"
    DERIVATION = "wasDerivedFrom"

    def __init__(self, id_nodes):
        self._authored = {
            id_: str(node) for id_, node in id_nodes if isinstance(node, URIRef)
        }
        self._derived: dict[str, dict] = {}

    def iri_of(self, id_):
        """IRI for an id, authored or already derived; None when neither."""
        entry = self._derived.get(id_)
        return entry["uri"] if entry else self._authored.get(id_)

    def register(self, id_, parent_iri, suffix, relation, types=()):
        """Mint <parent_iri>/<suffix> for id_ and record how it relates to its parent."""
        if not id_ or not parent_iri:
            return None
        # An authored node always wins: a derived IRI must never shadow a model's own.
        authored = self._authored.get(id_)
        if authored is not None:
            return authored
        uri = f"{parent_iri.rstrip('/')}/{_kebab(suffix)}"
        existing = self._derived.get(id_)
        if existing is not None:
            if existing["uri"] != uri:
                raise RuntimeError(
                    f"derived IRI collision: '{id_}' minted as both "
                    f"{existing['uri']} and {uri}"
                )
            return uri
        self._derived[id_] = {
            "uri": uri,
            "parent": parent_iri,
            "relation": relation,
            "types": list(types),
        }
        return uri

    def rows(self):
        """[{id, uri}] rows for the introspection uris table, sorted by id."""
        return [
            {"id": id_, "uri": entry["uri"]}
            for id_, entry in sorted(self._derived.items())
        ]

    def nodes(self):
        """Derivation-graph nodes: what each derived entity is, and what it came from."""
        return [
            {
                "id": entry["uri"],
                "types": ["prov:Entity", *entry["types"]],
                "relation": entry["relation"],
                "parent": entry["parent"],
            }
            for _, entry in sorted(self._derived.items())
        ]
def _id_ref(value):
    """Normalize a value to its id string (str/Enum/.id), or None."""
    if value is None:
        return None
    if isinstance(value, str):
        return value
    if isinstance(value, Enum):
        return value.value
    return getattr(value, "id", None)
def _resolve_import_location(location: str, url_map: dict[str, str]) -> str:
    """Map an import-location URL to its local file path via the url map."""
    for base, root in sorted(url_map.items(), key=lambda item: len(item[0]), reverse=True):
        if location.startswith(base):
            return str((Path(root) / location[len(base) :]).resolve())
    return location
def _load_graph(manifest_path):
    """Load the app manifest and its imports into one dataset; return (app path, graph, imported
    models, imported provenance).
    """
    app_model_path = Path(manifest_path).resolve()
    g = rdflib.Dataset(default_union=True)
    install_resolver(IriToFileResolver(metamodel_url_map(), download=False))
    g.parse(str(app_model_path), format="json-ld")

    url_map = build_url_map(g, app_model_path)
    install_resolver(IriToFileResolver({**metamodel_url_map(), **url_map}, download=False))

    imported_files = list(dict.fromkeys(str(model) for model in g.objects(predicate=APP["import"])))
    _PROV_SUFFIX = "/provenance/dsl.ld.json"
    imported_provenance = [
        _resolve_import_location(item, url_map)
        for item in imported_files
        if item.endswith(_PROV_SUFFIX)
    ]
    imported_model_locations = [i for i in imported_files if not i.endswith(_PROV_SUFFIX)]
    imported_models = [_resolve_import_location(item, url_map) for item in imported_model_locations]
    for model in imported_model_locations:
        g.parse(location=model, format="json-ld")
    return app_model_path, g, imported_models, imported_provenance
def _node_indexes(g, p: Parser):
    """Build the (node_by_id, id_nodes) lookup indexes from the parser."""
    node_by_id = {}
    id_nodes = []
    for node in sorted(g.subjects(), key=lambda item: str(item)):
        try:
            id_ = p.id(node)
        except Exception:
            continue
        id_nodes.append((id_, node))
        node_by_id.setdefault(id_, node)
    return node_by_id, id_nodes
