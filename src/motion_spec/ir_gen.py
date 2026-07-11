# SPDX-License-Identifier: MPL-2.0
"""Intermediate representation (IR) generator for motion specification models.

This module parses RDF graphs containing motion specification models and generates
a JSON intermediate representation suitable for code generation.
"""

from __future__ import annotations

import argparse
import collections
import hashlib
import itertools
import json
import math
import os
import re
import sys
import weakref
import xml.etree.ElementTree as ET
from dataclasses import asdict, dataclass, field, replace
from enum import Enum
from functools import wraps
from pathlib import Path

import rdflib
from rdf_utils.naming import get_valid_var_name
from rdf_utils.resolver import IriToFileResolver, install_resolver
from rdf_utils.uri import local_name
from rdflib import URIRef
from rdflib.namespace import RDF

from motion_spec.entities import (
    AccelerationConstraint,
    AccelerationTwist,
    Axis,
    BilateralConstraint,
    CartesianForceSpecification,
    Constraint,
    ConstraintEvaluator,
    ConstraintHandler,
    ControlMode,
    DataclassJSONEncoder,
    Direction,
    EdgeMonitor,
    EqualityConstraint,
    EvaluatorType,
    FeedForwardController,
    ForceDistributionSolver,
    ForwardedCommand,
    Frame,
    FreeVector,
    GuardedMotion,
    GuardedMotionBlock,
    HandlerArmSolver,
    ImpedanceController,
    JointForceSpecification,
    JointPosition,
    LevelMonitor,
    MotionDrivers,
    Orientation,
    OutsideConstraint,
    PIDController,
    Point,
    Pose,
    PoseAxisErrorComponent,
    PoseAxisErrorGroup,
    PoseDifference,
    Position,
    Provenance,
    Quantity,
    QuantityKind,
    RelativePoseCapture,
    Saturation,
    SceneAttachment,
    SceneObject,
    SceneObjectSpec,
    SceneRelativePose,
    SceneRobot,
    SceneSpec,
    SimplicialComplex,
    SnapshotCapture,
    SolverWithInputAndOutput,
    Subspace,
    Trajectory,
    UnilateralConstraint,
    UnilateralConstraintType,
    Unit,
    VelocityCompositionSolver,
    VelocityTwist,
    View,
    Wrench,
)
from motion_spec.manifest import build_url_map, metamodel_url_map
from motion_spec.namespace import (
    APP,
    CSTR,
    CSTR_EXT,
    CSTR_HDL,
    CSTR_HDL_EXT,
    ENV,
    EXEC,
    GEOM_COORD,
    GEOM_COORD_EXT,
    GEOM_ENT,
    GEOM_OP,
    GEOM_OP_EXT,
    GEOM_REL,
    KC_STAT,
    MAP,
    MAP_EXT,
    MJ,
    MOT,
    POLY,
    QUDT_QKIND,
    QUDT_SCHEMA,
    QUDT_UNIT,
    RBDYN_COORD,
    RBDYN_ENT,
    RBDYN_OP,
    RBDYN_OP_EXT,
    RT,
    SLV,
    SLV_EXT,
    SNAP,
    TRAJ,
)


# ---------------------------------------------------------------------------
# DSL operators and specifications
# ---------------------------------------------------------------------------
def parse_argument(g, closure_id, argument, to_id, resolve_value=False):
    # For each of the key differentiate if there is one or more associated value
    """Resolve a closure argument (input/output/parameter) from the graph to id(s); a lone value
    collapses to a scalar and qudt:Quantity constants resolve to their scalar value.
    """
    entry = list(g[closure_id:argument])
    if len(entry) == 0:
        return None

    def resolve(e):
        # Parameters are baked into generated code as literal text, so a qudt:Quantity-wrapped
        # constant must resolve to its scalar qudt:value here (bare literals / IRI refs pass through).
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


@dataclass
class Operator:
    """A schedulable RDF computation mapping graph inputs/outputs/parameters to a closure and
    participating in output-to-input scheduling.
    """

    type_: URIRef
    input: list[URIRef]
    output: list[URIRef]
    parameters: list = field(default_factory=list)
    schedulable: bool = True

    def closure_step(self, g, to_id, closure_id):
        closure = {"id": to_id(closure_id), "type": to_id(self.type_)}

        for input in self.input:
            closure[to_id(input)] = parse_argument(g, closure_id, input, to_id)
        for output in self.output:
            closure[to_id(output)] = parse_argument(g, closure_id, output, to_id)
        for param in self.parameters:
            closure[to_id(param)] = parse_argument(g, closure_id, param, to_id, resolve_value=True)

        return closure

    def from_operator_to_input(self, g, operator_id):
        data_structures = set()

        for in_ in self.input:
            for data_in in g.objects(operator_id, in_):
                data_structures.add(data_in)

        return data_structures

    def from_output_to_operator(self, g, data_out):
        return [
            op
            for out in self.output
            for op in g.subjects(out, data_out)
            if g[op : RDF["type"] : self.type_]
        ]

    def scheduler_step(self, g, data_out):
        data_structures = set()
        schedule = []

        for out in self.output:
            for call in g[:out:data_out]:
                if not g[call : RDF["type"] : self.type_]:
                    continue

                # We will only record this call if it has any input
                has_any_input = False

                for in_ in self.input:
                    for data_in in g.objects(call, in_):
                        data_structures.add(data_in)
                        has_any_input = True

                if has_any_input:
                    schedule.append(call)

        return {"data_structures": data_structures, "schedule": schedule}


@dataclass
class Specification:
    """
    A specification is not a computation and, hence, has neither a closure nor
    an entry in a schedule but contributes in finding further computations by
    propagating from outputs to inputs.
    """

    type_: URIRef
    input: list[URIRef]
    output: list[URIRef]
    parameters: list = field(default_factory=list)
    schedulable: bool = False

    def from_operator_to_input(self, g, operator_id):
        data_structures = set()

        for in_ in self.input:
            for data_in in g.objects(operator_id, in_):
                data_structures.add(data_in)

        return data_structures

    def from_output_to_operator(self, g, data_out):
        return [
            op
            for out in self.output
            for op in g.subjects(out, data_out)
            if g[op : RDF["type"] : self.type_]
        ]

    def scheduler_step(self, g, data_out):
        data_structures = set()
        for out in self.output:
            for call in g.subjects(out, data_out):
                for in_ in self.input:
                    for data_in in g.objects(call, in_):
                        data_structures.add(data_in)

        return {"data_structures": data_structures, "schedule": []}


class ErrorEvaluator:
    """Constraint-handler operator that emits a constraint error signal, dispatching on the
    constraint type (equality/greater/less/bilateral/outside).
    """

    def __init__(self):
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
        constraint_id = g.value(closure_id, CSTR_HDL["constraint"])

        for operator in self.cstr_op:
            if operator.type_ not in g[constraint_id : RDF["type"]]:
                continue

            closure = {
                "id": to_id(closure_id),
                "type": "ErrorEvaluator",
                "constraint": to_id(operator.type_),
            }

            for input in operator.input:
                closure[to_id(input)] = parse_argument(g, constraint_id, input, to_id)
            for output in operator.output:
                closure[to_id(output)] = parse_argument(g, closure_id, output, to_id)
            for param in operator.parameters:
                closure[to_id(param)] = parse_argument(
                    g, closure_id, param, to_id, resolve_value=True
                )

            # Only return the first matching type
            return closure

        # Should never happen
        return None

    def from_operator_to_input(self, g, operator_id):
        data_structures = set()

        for op in self.cstr_op:
            if op.type_ not in g[operator_id : CSTR_HDL["constraint"] / RDF["type"]]:
                continue

            for in_ in op.input:
                for data_in in g.objects(operator_id, CSTR_HDL["constraint"] / in_):
                    data_structures.add(data_in)

        return data_structures

    def from_output_to_operator(self, g, data_out):
        outputs = {out for operator in self.cstr_op for out in operator.output}
        return [
            op
            for out in outputs
            for op in g.subjects(out, data_out)
            if g[op : RDF["type"] : self.type_]
        ]

    def scheduler_step(self, g, data_out):
        data_structures = set()
        schedule = []

        for op in self.cstr_op:
            for out in op.output:
                for call in g.subjects(out, data_out):
                    if op.type_ not in g[call : CSTR_HDL["constraint"] / RDF["type"]]:
                        continue

                    # We will only record this call if it has any input
                    has_any_input = False

                    for in_ in op.input:
                        for data_in in g.objects(call, CSTR_HDL["constraint"] / in_):
                            data_structures.add(data_in)
                            has_any_input = True

                    if has_any_input:
                        schedule.append(call)

        return {"data_structures": data_structures, "schedule": schedule}


class AssignmentEvaluator:
    """Constraint operator that assigns a reference value to a quantity (no error output)."""

    def __init__(self):
        self.type_ = CSTR_HDL["AssignmentEvaluator"]
        self.schedulable = True
        self.cstr_op = Operator(
            type_=CSTR["EqualityConstraint"],
            input=[CSTR["quantity"], CSTR["reference-value"]],
            output=[],
        )

    def closure_step(self, g, to_id, closure_id):
        constraint_id = g.value(closure_id, CSTR_HDL["constraint"])

        if self.cstr_op.type_ not in g[constraint_id : RDF["type"]]:
            return None

        closure = {
            "id": to_id(closure_id),
            "type": "AssignmentEvaluator",
            "constraint": to_id(self.cstr_op.type_),
        }

        for input in self.cstr_op.input:
            closure[to_id(input)] = parse_argument(g, constraint_id, input, to_id)
        for param in self.cstr_op.parameters:
            closure[to_id(param)] = parse_argument(g, closure_id, param, to_id, resolve_value=True)

        # Only return the first matching type
        return closure

    def from_operator_to_input(self, g, operator_id):
        if self.cstr_op.type_ not in g[operator_id : CSTR_HDL["constraint"] / RDF["type"]]:
            return set()

        data_structures = set()
        for in_ in self.cstr_op.input:
            for data_in in g.objects(operator_id, CSTR_HDL["constraint"] / in_):
                data_structures.add(data_in)

        return data_structures


def _op_output_preds(op):
    # Predicates op.scheduler_step queries against data_out; if none point into a
    # node, scheduler_step can only return empty, so it is safe to skip.
    """Output predicates an operator's scheduler queries; empty when the operator can never match,
    so its scheduler step can be skipped.
    """
    if isinstance(op, ErrorEvaluator):
        return {p for sub in op.cstr_op for p in sub.output}
    if isinstance(op, AssignmentEvaluator):
        return set()
    return set(op.output)


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
        type_=RBDYN_OP_EXT["AddQuantity"],
        input=[RBDYN_OP["in1"], RBDYN_OP["in2"]],
        output=[RBDYN_OP["out"]],
    ),
    Operator(type_=RBDYN_OP_EXT["Norm"], input=[RBDYN_OP["in1"]], output=[RBDYN_OP["out"]]),
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
    Operator(
        type_=GEOM_OP_EXT["PoseDiffEvaluator"],
        input=[GEOM_OP["in1"], GEOM_OP["in2"]],
        output=[GEOM_OP_EXT["out"]],
    ),
    Operator(
        type_=TRAJ["Lerp"],
        input=[TRAJ["start"], TRAJ["goal"], TRAJ["alpha"]],
        output=[TRAJ["trajectory"]],
        parameters=[TRAJ["profile"]],
    ),
    Operator(
        type_=CSTR_HDL_EXT["VelocityProfile"],
        input=[
            CSTR_HDL_EXT["goal"],
            CSTR_HDL_EXT["measured"],
            TRAJ["measured-velocity"],
            TRAJ["max-velocity"],
            TRAJ["max-acceleration"],
            TRAJ["max-jerk"],
        ],
        output=[CSTR_HDL_EXT["reference"]],
        parameters=[TRAJ["shape"], CSTR_HDL_EXT["controller"]],
    ),
    Operator(
        type_=CSTR_HDL_EXT["Admittance"],
        input=[CSTR_HDL_EXT["force"]],
        output=[CSTR_HDL_EXT["reference"]],
        parameters=[
            CSTR_HDL_EXT["mass"],
            CSTR_HDL_EXT["damping"],
            CSTR_HDL_EXT["stiffness"],
            CSTR_HDL_EXT["max-velocity"],
            CSTR_HDL_EXT["controller"],
        ],
    ),
    Operator(
        type_=TRAJ["Circle"],
        input=[TRAJ["start"], TRAJ["center"], TRAJ["plane-normal"], TRAJ["alpha"]],
        output=[TRAJ["trajectory"]],
    ),
    Operator(
        type_=TRAJ["Arc"],
        input=[TRAJ["start"], TRAJ["end"], TRAJ["amplitude"], TRAJ["plane-normal"], TRAJ["alpha"]],
        output=[TRAJ["trajectory"]],
    ),
    Operator(
        type_=TRAJ["Helix"],
        input=[
            TRAJ["start"],
            TRAJ["center"],
            TRAJ["axis"],
            TRAJ["pitch"],
            TRAJ["revolutions"],
            TRAJ["alpha"],
        ],
        output=[TRAJ["trajectory"]],
    ),
    Operator(
        type_=TRAJ["Figure8"],
        input=[TRAJ["anchor"], TRAJ["radius"], TRAJ["plane-normal"], TRAJ["alpha"]],
        output=[TRAJ["trajectory"]],
        parameters=[TRAJ["form"]],
    ),
]

ops_cstr_hdl = [
    Operator(
        type_=CSTR_HDL["Controller"],
        input=[
            CSTR_HDL["error-signal"],
            CSTR_HDL_EXT["reference-signal"],
            CSTR_HDL_EXT["measured-derivative"],
        ],
        output=[CSTR_HDL["control-signal"]],
        parameters=[
            CSTR_HDL["proportional-gain"],
            CSTR_HDL["integral-gain"],
            CSTR_HDL["derivative-gain"],
            CSTR_HDL["decay-rate"],
        ],
    ),
    AssignmentEvaluator(),
    ErrorEvaluator(),
]

ops_slv = [
    Specification(type_=SLV["CartesianForceSpecification"], input=[SLV["force"]], output=[]),
    Specification(type_=SLV["JointForceSpecification"], input=[SLV["force"]], output=[]),
    Specification(
        type_=SLV["AccelerationConstraint"],
        # `slv-ext:direction` is only present on DirectionAligned constraints; absent
        # on AxisAligned ones, where g.objects() simply yields nothing for it.
        input=[SLV["acceleration-energy"], SLV_EXT["direction"]],
        output=[],
    ),
    Specification(type_=SLV["ForceDistributionSolver"], input=[SLV["force"]], output=[]),
]


# ---------------------------------------------------------------------------
# RDF parsing
# ---------------------------------------------------------------------------
def memoize(func):
    """Decorator caching a Parser method's result per instance, keyed by its arguments."""

    @wraps(func)
    def decorator(self, *args, **kwargs):
        # Scope by func identity so e.g. position(uri) and quantity(uri)
        # don't collide on the same (uri,) cache key.
        key = (func.__qualname__,) + args + tuple(kwargs.items())
        if key not in self.cache:
            self.cache[key] = func(self, *args, **kwargs)
        return self.cache[key]

    return decorator


def escape(s):
    """Return a graph-safe identifier for a local name."""
    s = re.sub(r"[^0-9A-Za-z_]", "_", str(s))
    if s and s[0].isdigit():
        s = f"_{s}"
    return s


class LocalIdMap:
    """Stable generated ids for RDF nodes: local name when unique, scoped name on collision."""

    def __init__(self, graph, nodes):
        self.graph = graph
        grouped: dict[str, list] = {}
        for node in sorted({n for n in nodes if n is not None}, key=str):
            grouped.setdefault(get_valid_var_name(local_name(node)), []).append(node)
        self.ids = {}
        for base, members in grouped.items():
            if len(members) == 1:
                self.ids[members[0]] = base
                continue
            used = set()
            for node in members:
                scoped = self._scoped_id(node, base)
                if scoped in used:
                    scoped = f"{scoped}_{hashlib.sha1(str(node).encode()).hexdigest()[:8]}"
                used.add(scoped)
                self.ids[node] = scoped

    def _scoped_id(self, node, base: str) -> str:
        try:
            prefix, _namespace, local = self.graph.compute_qname(node)
        except Exception:
            prefix, local = "", base
        if prefix:
            return get_valid_var_name(f"{prefix}_{local}")
        return f"{base}_{hashlib.sha1(str(node).encode()).hexdigest()[:8]}"

    def __getitem__(self, node) -> str:
        if node not in self.ids:
            self.ids[node] = get_valid_var_name(local_name(node))
        return self.ids[node]


class Parser:
    # A context quantity's id is its URI's last segment, so a name reused across motions' specs
    # (e.g. `support-z`) collapses to one id. Qualify only ambiguous names (same segment, >1 owner).
    """Parses a motion-spec RDF dataset into IR pieces: ids, closures, views, data structures,
    handlers and solvers.
    """

    _SPEC_OWNER_RE = re.compile(r"/([^/]+)/(?:Spec/spec|World/world)/")

    # scan depends only on the graph; memoize per-graph
    _ambiguous_cache: "weakref.WeakKeyDictionary" = weakref.WeakKeyDictionary()

    def __init__(self, g):
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
        if type_ not in self.g[id_ : RDF["type"]]:
            raise ValueError(f"node {id_} is missing expected rdf:type {type_}")

    def _compute_ambiguous_context_ids(self):
        owners_by_id: dict[str, set[str]] = {}
        for s in set(self.g.subjects()):
            m = self._SPEC_OWNER_RE.search(str(s))
            if not m:
                continue
            try:
                local = escape(self.g.compute_qname(s)[2])
            except Exception:
                continue
            owners_by_id.setdefault(local, set()).add(m.group(1))
        return {lid for lid, owners in owners_by_id.items() if len(owners) > 1}

    def id(self, x):
        cached = self._id_cache.get(x)
        if cached is not None:
            return cached
        try:
            q = self.g.compute_qname(x)
            local = escape(q[2])
        except Exception:
            self._id_cache[x] = x
            return x
        # Only context quantities (a motion's or the shared context's `Spec/spec` / `World/world`
        # members) become `shared.*` data fields and are vulnerable to the silent merge; constraint
        # names, metamodel predicates and aliases legitimately share an id and are scoped elsewhere.
        m = self._SPEC_OWNER_RE.search(str(x))
        if m:
            if local in self._ambiguous_context_ids:
                local = escape(f"{m.group(1)}-{q[2]}")
            self._id_sources.setdefault(local, set()).add(str(x))
        self._id_cache[x] = local
        return local

    def assert_no_id_collisions(self):
        """Fail loudly if two distinct context-quantity URIs collapse to one generated id. That
        would silently merge unrelated `shared.*` fields (the class of bug that hid the support-z
        collision) -- a genuinely-shared quantity has a single URI, so >1 URI per id is a real
        collision the motion-qualified id scoping failed to resolve."""
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
        try:
            q = self.g.compute_qname(x)
            return q[2]
        except Exception:
            return str(x)

    @memoize
    def velocity_composition_solver(self, id_):
        self._expect_type(id_, SLV["VelocityCompositionSolver"])
        conf = self.id(self.g.value(id_, SLV["configuration"]))
        velocity = self.velocity_twist(self.g.value(id_, SLV["velocity"]))

        return VelocityCompositionSolver(self.id(id_), conf, velocity)

    @memoize
    def force_distribution_solver(self, id_):
        self._expect_type(id_, SLV["ForceDistributionSolver"])
        conf = self.id(self.g.value(id_, SLV["configuration"]))
        force = self.wrench(self.g.value(id_, SLV["force"]))

        return ForceDistributionSolver(self.id(id_), conf, force)

    @memoize
    def solver_with_input_and_output(self, id_):
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
                if type_ in self.g[o : RDF["type"]]:
                    out.append(func(o))

        # Authored gravity-value IS the solver root acceleration (KDL Vereshchagin
        # root_acc.vel) — taken as truth, no sign flip. If a MuJoCo env ever needs a
        # different world gravity than the solver, hardcode it there with a TODO;
        # do not re-derive it from this by negation.
        gravity_node = self.g.value(id_, SLV_EXT["gravity-value"])
        root_acc = self.parse_xyz(gravity_node) if gravity_node else None
        algorithm_node = self.g.value(id_, SLV["solver"])
        algorithm = {
            SLV["AccelerationConstrainedHybridDynamicsAlgorithm"]: "ACHD",
            SLV["RecursiveNewtonEulerAlgorithm"]: "RNE",
        }.get(algorithm_node, self.id(algorithm_node) if algorithm_node else "")
        torque_saturation_node = next(
            (
                node
                for node in self.g.objects(id_, CSTR_HDL_EXT["limits"])
                if SLV_EXT["TorqueSaturation"] in self.g[node : RDF["type"]]
            ),
            None,
        )

        return SolverWithInputAndOutput(
            id=self.id(id_),
            motion_drivers=drv,
            output=out,
            algorithm=algorithm,
            algorithm_is_rne=algorithm == "RNE",
            root_acc=root_acc,
            regularization=self._optional_float(id_, SLV_EXT["regularization"]),
            torque_saturation=(
                self.saturation(torque_saturation_node)
                if torque_saturation_node is not None
                else None
            ),
        )

    @memoize
    def motion_drivers(self, id_):
        self._expect_type(id_, SLV["MotionDrivers"])
        spec_acc = []
        spec_frc = []
        spec_jf = []

        for a in self.g[id_ : SLV["acceleration-constraint"]]:
            self._expect_type(a, SLV["AccelerationConstraintSpecification"])
            for c in self.g[a : SLV["constraints"]]:
                spec_acc.append(self.acceleration_constraint(c))

        for f in self.g[id_ : SLV["cartesian-force"]]:
            spec_frc.append(self.cartesian_force_specification(f))

        for jf in self.g[id_ : SLV["joint-force"]]:
            spec_jf.append(self.joint_force_specification(jf))

        return MotionDrivers(
            self.id(id_), spec_acc, spec_frc, spec_jf, has_cartesian_force=bool(spec_frc)
        )

    def joint_force_specification(self, id_):
        self._expect_type(id_, SLV["JointForceSpecification"])
        force_node = self.g.value(id_, SLV["force"])
        force_id = self.id(force_node) if force_node is not None else ""
        joint_node = self.g.value(id_, SLV["attached-to"])
        joint_name = self.label(joint_node) if joint_node is not None else ""
        return JointForceSpecification(self.id(id_), force_id, joint_name)

    @memoize
    def cartesian_force_specification(self, id_):
        self._expect_type(id_, SLV["CartesianForceSpecification"])
        force = self.wrench(self.g.value(id_, SLV["force"]))
        attached_to = self.simplicial_complex(self.g.value(id_, SLV["attached-to"]))

        return CartesianForceSpecification(self.id(id_), force, attached_to)

    @memoize
    def saturation(self, id_):
        self._expect_type(id_, CSTR_HDL_EXT["Saturation"])
        input_signal = self.quantity(self.g.value(id_, CSTR_HDL_EXT["input-signal"]))
        output_signal = self.quantity(self.g.value(id_, CSTR_HDL_EXT["output-signal"]))
        maximum_node = self.g.value(id_, CSTR_HDL_EXT["maximum-absolute-value"])
        lower_node = self.g.value(id_, CSTR_HDL_EXT["lower-limit"])
        upper_node = self.g.value(id_, CSTR_HDL_EXT["upper-limit"])
        return Saturation(
            self.id(id_),
            input_signal,
            output_signal,
            self.quantity(maximum_node) if maximum_node is not None else None,
            self.quantity(lower_node) if lower_node is not None else None,
            self.quantity(upper_node) if upper_node is not None else None,
        )

    @memoize
    def acceleration_constraint(self, id_):
        self._expect_type(id_, SLV["AccelerationConstraint"])
        subspace = self.subspace(self.g.value(id_, SLV["subspace"]))
        e_acc = self.quantity(self.g.value(id_, SLV["acceleration-energy"]))
        saturation_node = next(
            (
                node
                for node in self.g.objects(id_, CSTR_HDL_EXT["limits"])
                if SLV_EXT["AccelerationSaturation"] in self.g[node : RDF["type"]]
            ),
            None,
        )
        saturation = self.saturation(saturation_node) if saturation_node is not None else None
        as_seen_by_node = self.g.value(id_, GEOM_COORD["as-seen-by"])
        as_seen_by = self.frame(as_seen_by_node) if as_seen_by_node else None

        if SLV_EXT["DirectionAligned"] in self.g[id_ : RDF["type"]]:
            direction = self.direction(self.g.value(id_, SLV_EXT["direction"]))
            return AccelerationConstraint(
                self.id(id_),
                subspace,
                None,
                e_acc,
                as_seen_by,
                direction=direction,
                saturation=saturation,
            )

        self._expect_type(id_, SLV["AxisAligned"])
        axis = self.axis(self.g.value(id_, SLV["axis"]))
        return AccelerationConstraint(
            self.id(id_), subspace, axis, e_acc, as_seen_by, saturation=saturation
        )

    @memoize
    def subspace(self, id_):
        d = {
            MAP["position"]: Subspace.Linear,
            MAP_EXT["rotation"]: Subspace.Angular,
            MAP_EXT["orientation"]: Subspace.Angular,
            MAP["angular-velocity"]: Subspace.Angular,
            MAP["linear-velocity"]: Subspace.Linear,
            MAP["angular-acceleration"]: Subspace.Angular,
            MAP["linear-acceleration"]: Subspace.Linear,
            MAP["torque"]: Subspace.Angular,
            MAP["force"]: Subspace.Linear,
            SLV["angular-acceleration"]: Subspace.Angular,
            SLV["linear-acceleration"]: Subspace.Linear,
            GEOM_COORD_EXT["linear"]: Subspace.Linear,
            GEOM_COORD_EXT["angular"]: Subspace.Angular,
        }
        if id_ not in d:
            raise ValueError(f"unknown subspace {id_}")

        return d[id_]

    @memoize
    def axis(self, id_):
        d = {
            MAP["x"]: Axis.X,
            MAP["y"]: Axis.Y,
            MAP["z"]: Axis.Z,
            SLV["x"]: Axis.X,
            SLV["y"]: Axis.Y,
            SLV["z"]: Axis.Z,
        }
        if id_ not in d:
            raise ValueError(f"unknown axis {id_}")

        return d[id_]

    @memoize
    def constraint_handler(self, id_):
        self._expect_type(id_, CSTR_HDL["ConstraintHandler"])
        motion = self.guarded_motion(self.g.value(id_, CSTR_HDL["motion"]))
        control_mode_node = self.g.value(id_, CSTR_HDL["control-mode"])
        if control_mode_node is None:
            raise ValueError(f"Constraint handler '{self.id(id_)}' is missing control-mode.")
        control_mode = self.id(control_mode_node)
        try:
            ControlMode(control_mode)
        except ValueError as exc:
            raise ValueError(
                f"Constraint handler '{self.id(id_)}' uses unsupported control mode "
                f"'{control_mode}'."
            ) from exc

        evaluators = []
        for e in self.g[id_ : CSTR_HDL["evaluators"]]:
            if GEOM_OP_EXT["PoseDiffEvaluator"] in self.g[e : RDF["type"]]:
                continue  # handled via schedule traversal
            evaluators.append(self.constraint_evaluator(e))

        controllers = []
        for c in self.g[id_ : CSTR_HDL["controllers"]]:
            controllers.append(self.controller(c))

        monitors = []
        for m in self.g[id_ : CSTR_HDL["monitors"]]:
            monitors.append(self.monitor_entry(m))

        order_value = self.g.value(id_, APP["order"])
        order = int(order_value.value) if order_value is not None else 0

        return ConstraintHandler(
            self.id(id_), motion, control_mode, evaluators, controllers, monitors, order
        )

    @memoize
    def monitor_entry(self, id_):
        self._expect_type(id_, CSTR_HDL["Monitor"])
        is_until_aggregate = self.g.value(id_, CSTR_HDL_EXT["monitors-until"]) is not None
        is_when_aggregate = self.g.value(id_, CSTR_HDL_EXT["monitors-when"]) is not None
        error_node = self.g.value(id_, CSTR_HDL["error"])
        error = (
            None
            if is_until_aggregate or is_when_aggregate or error_node is None
            else self.quantity(error_node)
        )

        if CSTR_HDL["LevelTriggeredMonitor"] in self.g[id_ : RDF["type"]]:
            flag = self.id(self.g.value(id_, CSTR_HDL["flag"]))
            return LevelMonitor(
                self.id(id_),
                "LevelTriggeredMonitor",
                error,
                flag,
                is_until_aggregate=is_until_aggregate,
                is_when_aggregate=is_when_aggregate,
            )

        event_node = self.g.value(id_, CSTR_HDL["event"])
        event = self.id(event_node)
        fallback_node = self.g.value(id_, CSTR_HDL_EXT["fallback-motion"])
        fallback_motion = self.id(fallback_node) if fallback_node is not None else None
        debounce_duration_s = self._optional_float(id_, CSTR_HDL_EXT["debounce-duration"])
        return EdgeMonitor(
            self.id(id_),
            "EdgeTriggeredMonitor",
            error,
            event,
            None,
            is_until_aggregate=is_until_aggregate,
            is_when_aggregate=is_when_aggregate,
            event_uri=str(event_node),
            event_name=event.upper(),
            fallback_motion=fallback_motion,
            debounce_duration_s=debounce_duration_s,
        )

    @memoize
    def constraint_evaluator(self, id_):
        self._expect_type(id_, CSTR_HDL["ConstraintEvaluator"])
        constraint_node = self.g.value(id_, CSTR_HDL["constraint"])
        constraint = self.constraint(constraint_node)

        if CSTR_HDL["AssignmentEvaluator"] in self.g[id_ : RDF["type"]]:
            t = EvaluatorType.AssignmentEvaluator
            error = None
        else:
            t = EvaluatorType.ErrorEvaluator
            error = self.quantity(self.g.value(id_, CSTR_HDL["error"]))

        # Timing constraint: measured quantity is the motion-state elapsed time (kind Time).
        # No solver error — codegen compares the world clock against the threshold directly.
        is_elapsed = False
        elapsed_op = None
        elapsed_threshold_s = None
        qnode = self.g.value(constraint_node, CSTR["quantity"])
        if (
            qnode is not None
            and QUDT_QKIND["Time"] in self.g[qnode : QUDT_SCHEMA["hasQuantityKind"]]
        ):
            is_elapsed = True
            elapsed_op = (
                ">="
                if CSTR["GreaterThanConstraint"] in self.g[constraint_node : RDF["type"]]
                else "<"
            )
            thr = self.g.value(constraint_node, CSTR["threshold"])
            thr_val = float(self.g.value(thr, QUDT_SCHEMA["value"]))
            thr_unit = self.g.value(thr, QUDT_SCHEMA["unit"])
            elapsed_threshold_s = thr_val * (0.001 if thr_unit == QUDT_UNIT["MilliSEC"] else 1.0)

        return ConstraintEvaluator(
            self.id(id_),
            t,
            constraint,
            error,
            is_elapsed=is_elapsed,
            elapsed_op=elapsed_op,
            elapsed_threshold_s=elapsed_threshold_s,
        )

    @memoize
    def controller(self, id_):
        is_pid = CSTR_HDL["ProportionalIntegralDerivative"] in self.g[id_ : RDF["type"]]
        is_impedance = CSTR_HDL["ImpedanceController"] in self.g[id_ : RDF["type"]]
        is_feedforward = CSTR_HDL_EXT["FeedForwardController"] in self.g[id_ : RDF["type"]]
        if not (is_pid or is_impedance or is_feedforward):
            raise ValueError(
                f"Controller {id_} must be ProportionalIntegralDerivative, ImpedanceController, "
                "or FeedForwardController"
            )

        error_node = self.g.value(id_, CSTR_HDL["error-signal"])
        ref_node = self.g.value(id_, CSTR_HDL_EXT["reference-signal"])
        measured_derivative_node = self.g.value(id_, CSTR_HDL_EXT["measured-derivative"])
        error_signal = self.quantity(error_node) if error_node is not None else None
        reference_signal = self.quantity(ref_node) if ref_node is not None else None
        measured_derivative = (
            self.quantity(measured_derivative_node)
            if measured_derivative_node is not None
            else None
        )
        control_signal = self.quantity(self.g.value(id_, CSTR_HDL["control-signal"]))
        output_saturation_node = next(
            (
                node
                for node in self.g.objects(id_, CSTR_HDL_EXT["limits"])
                if CSTR_HDL_EXT["IntegralSaturation"] not in self.g[node : RDF["type"]]
            ),
            None,
        )
        integral_saturation_node = next(
            (
                node
                for node in self.g.objects(id_, CSTR_HDL_EXT["limits"])
                if CSTR_HDL_EXT["IntegralSaturation"] in self.g[node : RDF["type"]]
            ),
            None,
        )
        output_saturation = (
            self.saturation(output_saturation_node) if output_saturation_node is not None else None
        )
        integral_saturation = (
            self.saturation(integral_saturation_node)
            if integral_saturation_node is not None
            else None
        )

        if is_pid:
            if error_signal is None:
                raise ValueError(
                    f"PID controller '{self.id(id_)}' must have cstr-hdl:error-signal."
                )
            if reference_signal is not None:
                raise ValueError(
                    f"PID controller '{self.id(id_)}' must not have cstr-hdl-ext:reference-signal."
                )
            decay_rate = None
            if CSTR_HDL["DecayingIntegralTerm"] in self.g[id_ : RDF["type"]]:
                decay_rate = self.g.value(id_, CSTR_HDL["decay-rate"]).value
            return PIDController(
                id=self.id(id_),
                control_signal=control_signal,
                error_signal=error_signal,
                measured_derivative=measured_derivative,
                proportional_gain=self._required_float(id_, CSTR_HDL["proportional-gain"]),
                integral_gain=self._required_float(id_, CSTR_HDL["integral-gain"]),
                derivative_gain=self._required_float(id_, CSTR_HDL["derivative-gain"]),
                decay_rate=decay_rate,
                output_saturation=output_saturation,
                integral_saturation=integral_saturation,
                type=self.id(CSTR_HDL.ProportionalIntegralDerivative),
            )
        if is_impedance:
            if measured_derivative is not None:
                raise ValueError(
                    f"Impedance controller '{self.id(id_)}' must not have cstr-hdl-ext:measured-derivative."
                )
            if error_signal is None:
                raise ValueError(
                    f"Impedance controller '{self.id(id_)}' must have cstr-hdl:error-signal."
                )
            if reference_signal is not None:
                raise ValueError(
                    f"Impedance controller '{self.id(id_)}' must not have cstr-hdl-ext:reference-signal."
                )
            return ImpedanceController(
                id=self.id(id_),
                control_signal=control_signal,
                error_signal=error_signal,
                stiffness=self._required_float(id_, CSTR_HDL["stiffness"]),
                damping=self._required_float(id_, CSTR_HDL["damping"]),
                integral_gain=self._optional_float(id_, CSTR_HDL["integral-gain"]),
                output_saturation=output_saturation,
                type=self.id(CSTR_HDL.ImpedanceController),
            )
        if reference_signal is None:
            raise ValueError(
                f"FeedForward controller '{self.id(id_)}' must have cstr-hdl-ext:reference-signal."
            )
        if error_signal is not None:
            raise ValueError(
                f"FeedForward controller '{self.id(id_)}' must not have cstr-hdl:error-signal."
            )
        if measured_derivative is not None:
            raise ValueError(
                f"FeedForward controller '{self.id(id_)}' must not have cstr-hdl-ext:measured-derivative."
            )
        return FeedForwardController(
            id=self.id(id_),
            control_signal=control_signal,
            reference_signal=reference_signal,
            output_saturation=output_saturation,
            type=self.id(CSTR_HDL_EXT.FeedForwardController),
        )

    @memoize
    def forwarded_command(self, id_):
        self._expect_type(id_, SLV_EXT["ForwardedCommand"])
        command_signal = self.quantity(self.g.value(id_, SLV_EXT["command-signal"]))
        target_node = self.g.value(id_, SLV["attached-to"])
        return ForwardedCommand(
            self.id(id_), command_signal, self.label(target_node) if target_node is not None else ""
        )

    def _optional_float(self, subject, predicate) -> float | None:
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

    def _required_float(self, subject, predicate) -> float:
        value = self._optional_float(subject, predicate)
        if value is None:
            raise ValueError(
                f"Controller '{self.id(subject)}' is missing required property '{self.id(predicate)}'."
            )
        return value

    @memoize
    def guarded_motion(self, id_):
        self._expect_type(id_, MOT["GuardedMotion"])
        when = []
        when_any = False
        for c in self.g[id_ : MOT["when"]]:
            if CSTR_EXT.ConstraintDisjunction in self.g[c : RDF["type"]]:
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
            if CSTR_EXT.ConstraintDisjunction in self.g[c : RDF["type"]]:
                until_any = True
                for member in self.g[c : CSTR_EXT["has-constraint"]]:
                    until.append(self.constraint(member))
            else:
                until.append(self.constraint(c))

        return GuardedMotion(self.id(id_), when, while_, until, until_any, when_any)

    @memoize
    def constraint(self, id_):
        self._expect_type(id_, CSTR["Constraint"])
        quantity = self.quantity(self.g.value(id_, CSTR["quantity"]))

        parameter = None
        if CSTR["EqualityConstraint"] in self.g[id_ : RDF["type"]]:
            parameter = self.equality_constraint(id_)
        elif CSTR["UnilateralConstraint"] in self.g[id_ : RDF["type"]]:
            parameter = self.unilateral_constraint(id_)
        elif CSTR_EXT["OutsideConstraint"] in self.g[id_ : RDF["type"]]:
            parameter = self.outside_constraint(id_)
        else:
            parameter = self.bilateral_constraint(id_)

        return Constraint(self.id(id_), quantity, parameter)

    @memoize
    def equality_constraint(self, id_):
        self._expect_type(id_, CSTR["EqualityConstraint"])
        reference_value = self.quantity(self.g.value(id_, CSTR["reference-value"]))

        return EqualityConstraint(reference_value)

    @memoize
    def unilateral_constraint(self, id_):
        self._expect_type(id_, CSTR["UnilateralConstraint"])
        threshold = self.quantity(self.g.value(id_, CSTR["threshold"]))
        type_ = UnilateralConstraintType.LessThan
        if CSTR["GreaterThanConstraint"] in self.g[id_ : RDF["type"]]:
            type_ = UnilateralConstraintType.GreaterThan

        return UnilateralConstraint(type_, threshold)

    @memoize
    def bilateral_constraint(self, id_):
        self._expect_type(id_, CSTR["BilateralConstraint"])
        lower_threshold = self.quantity(self.g.value(id_, CSTR["lower-threshold"]))
        upper_threshold = self.quantity(self.g.value(id_, CSTR["upper-threshold"]))

        return BilateralConstraint(lower_threshold, upper_threshold)

    @memoize
    def outside_constraint(self, id_):
        self._expect_type(id_, CSTR_EXT["OutsideConstraint"])
        lower_threshold = self.quantity(self.g.value(id_, CSTR["lower-threshold"]))
        upper_threshold = self.quantity(self.g.value(id_, CSTR["upper-threshold"]))

        return OutsideConstraint(lower_threshold, upper_threshold)

    @memoize
    def direction(self, id_):
        self._expect_type(id_, GEOM_COORD["DirectionCoordinate"])
        self._expect_type(id_, GEOM_COORD["VectorXYZ"])
        quantity_kind = []
        for k in self.g[id_ : QUDT_SCHEMA["hasQuantityKind"]]:
            quantity_kind.append(self.id(k))
        for k in self.g[id_ : QUDT_SCHEMA["quantity-kind"]]:
            quantity_kind.append(self.id(k))
        as_seen_by = self.frame(self.g.value(id_, GEOM_COORD["as-seen-by"]))
        unit = self.id(self.g.value(id_, QUDT_SCHEMA["unit"]))
        direction = self.parse_xyz(id_)

        return Direction(self.id(id_), quantity_kind, as_seen_by, [Unit(unit)], direction)

    def parse_vector3(self, node):
        from rdflib import collection

        items = list(collection.Collection(self.g, node))
        if len(items) != 3:
            return None

        # Get the Python representation of the associated RDF literal
        return [float(v.toPython()) for v in items]

    def parse_xyz(self, node):
        x = self.g.value(node, GEOM_COORD["x"])
        y = self.g.value(node, GEOM_COORD["y"])
        z = self.g.value(node, GEOM_COORD["z"])

        if x is None or y is None or z is None:
            return None

        return [float(v.value) for v in (x, y, z)]

    @memoize
    def position(self, id_):
        self._expect_type(id_, GEOM_COORD["PositionCoordinate"])
        self._expect_type(id_, GEOM_COORD["VectorXYZ"])
        of = self.position_reference(self.g.value(id_, GEOM_REL["of"]))
        wrt = self.position_reference(self.g.value(id_, GEOM_REL["with-respect-to"]))
        quantity_kind = self.id(self.g.value(id_, QUDT_SCHEMA["hasQuantityKind"]))
        as_seen_by = self.frame(self.g.value(id_, GEOM_COORD["as-seen-by"]))
        unit = self.id(self.g.value(id_, QUDT_SCHEMA["unit"]))
        pos = self.parse_xyz(id_)

        return Position(
            self.id(id_), of, wrt, QuantityKind(quantity_kind), as_seen_by, Unit(unit), pos
        )

    @memoize
    def orientation(self, id_):
        self._expect_type(id_, GEOM_COORD["OrientationCoordinate"])

        def optional_pose_ref(node):
            if node is None:
                return None
            if ENV.RigidObject in self.g[node : RDF["type"]]:
                return self.scene_object(node)
            if GEOM_ENT.Frame in self.g[node : RDF["type"]]:
                return self.frame(node)
            return None

        of = optional_pose_ref(self.g.value(id_, GEOM_REL["of"]))
        wrt = optional_pose_ref(self.g.value(id_, GEOM_REL["with-respect-to"]))
        quantity_kind = self.id(self.g.value(id_, QUDT_SCHEMA["hasQuantityKind"]))
        as_seen_by_node = self.g.value(id_, GEOM_COORD["as-seen-by"])
        as_seen_by = self.frame(as_seen_by_node) if as_seen_by_node is not None else None
        unit = self.id(self.g.value(id_, QUDT_SCHEMA["unit"]))
        axes = self.g.value(id_, GEOM_COORD["axes-sequence"])
        provenance = self.quantity_provenance(id_)
        return Orientation(
            self.id(id_),
            of,
            wrt,
            QuantityKind(quantity_kind),
            as_seen_by,
            Unit(unit),
            str(axes) if axes is not None else None,
            (id_, ~MAP["subobject"], None) in self.g,
            provenance=provenance,
        )

    def position_reference(self, id_):
        """A Position is of a Point with respect to a Point (geometry metamodel)."""
        if id_ is None:
            return None
        if GEOM_ENT.Point in self.g[id_ : RDF["type"]]:
            return self.point(id_)
        raise ValueError(f"Position reference must be a Point, got: {id_}")

    @memoize
    def _pose_endpoint(self, node):
        if node is None:
            return None
        if ENV.RigidObject in self.g[node : RDF["type"]]:
            return self.scene_object(node)
        return self.frame(node)

    def _bare_pose(self, id_):
        # A geom-rel:Pose with no PoseCoordinate: a snapshot/reference pose whose
        # KDL::Frame is filled at runtime. Same IR shape as a coordinate pose, with
        # its frame endpoints but no authored coordinate values.
        as_seen_by_node = self.g.value(id_, GEOM_COORD["as-seen-by"])
        provenance = self.quantity_provenance(id_)
        return Pose(
            self.id(id_),
            self._pose_endpoint(self.g.value(id_, GEOM_REL["of"])),
            self._pose_endpoint(self.g.value(id_, GEOM_REL["with-respect-to"])),
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
        self._expect_type(id_, GEOM_COORD["PoseCoordinate"])
        self._expect_type(id_, GEOM_COORD["VectorXYZ"])
        of = self._pose_endpoint(self.g.value(id_, GEOM_REL["of"]))
        wrt = self._pose_endpoint(self.g.value(id_, GEOM_REL["with-respect-to"]))
        quantity_kind = []
        for k in self.g[id_ : QUDT_SCHEMA["hasQuantityKind"]]:
            quantity_kind.append(self.id(k))
        as_seen_by_node = self.g.value(id_, GEOM_COORD["as-seen-by"])
        as_seen_by = self.frame(as_seen_by_node) if as_seen_by_node is not None else None
        unit = []
        for u in self.g[id_ : QUDT_SCHEMA["unit"]]:
            unit.append(self.id(u))
        dc_x = self.parse_vector3(self.g.value(id_, GEOM_COORD["direction-cosine-x"]))
        dc_y = self.parse_vector3(self.g.value(id_, GEOM_COORD["direction-cosine-y"]))
        dc_z = self.parse_vector3(self.g.value(id_, GEOM_COORD["direction-cosine-z"]))
        pos = self.parse_xyz(id_)
        euler_axes_sequence = None
        for coord in self.g.objects(id_, GEOM_COORD["has-coordinate"]):
            if GEOM_COORD["EulerAngles"] in self.g[coord : RDF["type"]]:
                axes = self.g.value(coord, GEOM_COORD["axes-sequence"])
                euler_axes_sequence = str(axes) if axes is not None else None
                break

        provenance = self.quantity_provenance(id_)
        return Pose(
            self.id(id_),
            of,
            wrt,
            quantity_kind,
            as_seen_by,
            unit,
            dc_x,
            dc_y,
            dc_z,
            pos,
            euler_axes_sequence,
            provenance=provenance,
        )

    @memoize
    def velocity_twist(self, id_):
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

    def _spatial_coordinate_fields(self, id_, ref_point_pred, as_seen_by_pred):
        """Shared field extraction for the 6D coordinate quantities.

        AccelerationTwist, PoseDifference and Wrench are distinct concepts with an
        identical coordinate structure; only the RDF predicates differ (geometry vs
        rigid-body-dynamics namespaces).
        """
        quantity_kind = [self.id(k) for k in self.g[id_ : QUDT_SCHEMA["hasQuantityKind"]]]
        reference_point = self.point(self.g.value(id_, ref_point_pred))
        as_seen_by = self.frame(self.g.value(id_, as_seen_by_pred))
        unit = [self.id(u) for u in self.g[id_ : QUDT_SCHEMA["unit"]]]
        provenance = self.quantity_provenance(id_)
        return quantity_kind, reference_point, as_seen_by, unit, provenance

    @memoize
    def acceleration_twist(self, id_):
        self._expect_type(id_, GEOM_COORD["AccelerationTwistCoordinate"])
        self._expect_type(id_, GEOM_COORD["VectorXYZ"])
        qk, ref, seen, unit, provenance = self._spatial_coordinate_fields(
            id_, GEOM_REL["reference-point"], GEOM_COORD["as-seen-by"]
        )
        return AccelerationTwist(self.id(id_), qk, ref, seen, unit, provenance=provenance)

    @memoize
    def pose_difference(self, id_):
        self._expect_type(id_, GEOM_COORD_EXT["PoseDifferenceCoordinate"])
        self._expect_type(id_, GEOM_COORD["VectorXYZ"])
        qk, ref, seen, unit, provenance = self._spatial_coordinate_fields(
            id_, GEOM_REL["reference-point"], GEOM_COORD["as-seen-by"]
        )
        return PoseDifference(self.id(id_), qk, ref, seen, unit, provenance=provenance)

    @memoize
    def wrench(self, id_):
        self._expect_type(id_, RBDYN_COORD["WrenchCoordinate"])
        qk, ref, seen, unit, provenance = self._spatial_coordinate_fields(
            id_, RBDYN_ENT["reference-point"], RBDYN_COORD["as-seen-by"]
        )
        sensor_name = str(self.g.value(id_, MJ["ft-sensor-ref"]) or "")
        return Wrench(
            self.id(id_), qk, ref, seen, unit, provenance=provenance, sensor_name=sensor_name
        )

    @memoize
    def quantity(self, id_):
        self._expect_type(id_, QUDT_SCHEMA["Quantity"])
        quantity_kind_node = self.g.value(id_, QUDT_SCHEMA["hasQuantityKind"]) or self.g.value(
            id_, QUDT_SCHEMA["quantity-kind"]
        )
        quantity_kind = self.id(quantity_kind_node)

        unit = self.id(self.g.value(id_, QUDT_SCHEMA["unit"]))
        has_view = (id_, ~MAP["subobject"], None) in self.g

        if GEOM_REL["Pose"] in self.g[id_ : RDF["type"]]:
            if GEOM_COORD["PoseCoordinate"] in self.g[id_ : RDF["type"]]:
                return self.pose(id_)
            return self._bare_pose(id_)
        if (
            GEOM_REL["Position"] in self.g[id_ : RDF["type"]]
            and GEOM_COORD["PositionCoordinate"] in self.g[id_ : RDF["type"]]
        ):
            return self.position(id_)
        if (
            GEOM_REL["Orientation"] in self.g[id_ : RDF["type"]]
            and GEOM_COORD["OrientationCoordinate"] in self.g[id_ : RDF["type"]]
        ):
            return self.orientation(id_)
        if TRAJ["Trajectory"] in self.g[id_ : RDF["type"]]:
            value_kind_node = next(
                (k for k in self.g[id_ : QUDT_SCHEMA["hasQuantityKind"]] if k != TRAJ.Trajectory),
                None,
            )
            provenance = self.quantity_provenance(id_)
            return Trajectory(
                self.id(id_),
                QuantityKind(quantity_kind),
                Unit(unit),
                has_view,
                provenance=provenance,
                value_kind=self.id(value_kind_node) if value_kind_node is not None else None,
            )

        if quantity_kind == "FreeVector" and GEOM_COORD["VectorXYZ"] in self.g[id_ : RDF["type"]]:
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
            value = float(self.g.value(id_, QUDT_SCHEMA["value"]))
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
    def joint_position(self, id_):
        self._expect_type(id_, KC_STAT["JointPositionCoordinate"])
        joint_node = self.g.value(id_, GEOM_REL["of"])
        joint_name = self.label(joint_node) if joint_node is not None else ""
        return JointPosition(self.id(id_), joint_name)

    def quantity_provenance(self, id_):
        # Provenance(authored, snapshot), mutually exclusive: snapshot wins (mirrors old roles() elif).
        # authored == carries an authored value/coordinate and is not a runtime snapshot.
        snapshot = SNAP.Snapshot in self.g[id_ : RDF["type"]]
        authored = (not snapshot) and self._is_authored(id_)
        return Provenance(authored=authored, snapshot=snapshot)

    def _is_authored(self, id_):
        if (id_, QUDT_SCHEMA["value"], None) in self.g:
            return True
        if (id_, CSTR["reference-value"], None) in self.g:
            return True
        if any(
            (id_, GEOM_COORD[c], None) in self.g
            for c in (
                "x",
                "y",
                "z",
                "direction-cosine-x",
                "direction-cosine-y",
                "direction-cosine-z",
            )
        ):
            return True
        if any(True for _ in self.g.objects(id_, GEOM_COORD["has-coordinate"])):
            return True
        return False

    @memoize
    def simplicial_complex(self, id_):
        self._expect_type(id_, GEOM_ENT["SimplicialComplex"])
        # rdf.py's _frame_body() mints this off a Frame (mj:attached-body) so Twist.of/wrt don't
        # fuse Frame and body onto one URI. Follow the link back so the id matches its name.
        owner = self.g.value(predicate=MJ["attached-body"], object=id_)
        return SimplicialComplex(self.id(owner if owner is not None else id_))

    @memoize
    def scene_object(self, id_):
        self._expect_type(id_, ENV.RigidObject)
        body = str(self.g.value(id_, MJ["body-name"]) or self.id(id_))
        return SceneObject(self.id(id_), body)

    @memoize
    def frame(self, id_):
        self._expect_type(id_, GEOM_ENT["Frame"])
        return Frame(self.id(id_))

    @memoize
    def point(self, id_):
        self._expect_type(id_, GEOM_ENT["Point"])
        # rdf.py's _frame_origin_point() mints this off a Frame (geom-ent:origin) so Position.of/wrt
        # don't fuse Frame and Point onto one URI. Follow the link back so the id matches its name.
        owner = self.g.value(predicate=GEOM_ENT["origin"], object=id_)
        return Point(self.id(owner if owner is not None else id_))

    def view(self):
        dispatcher = [
            (MAP["DirectionCoordinateView"], self.direction),
            (MAP["PoseCoordinateView"], self.pose),
            (MAP_EXT["PoseOrientationView"], self.pose),
            (MAP_EXT["PosePositionView"], self.pose),
            (MAP["VelocityTwistCoordinateView"], self.velocity_twist),
            (MAP["AccelerationTwistCoordinateView"], self.acceleration_twist),
            (MAP_EXT["PoseDifferenceView"], self.pose_difference),
            (MAP["WrenchCoordinateView"], self.wrench),
            (MAP_EXT["WrenchVectorView"], self.wrench),
        ]

        view_map = {}
        for view in self.g[: RDF["type"] : MAP["View"]]:
            superobject = None
            for type_, func in dispatcher:
                if type_ not in self.g[view : RDF["type"]]:
                    continue

                superobject_id = self.g.value(view, MAP["superobject"])
                superobject = func(superobject_id)
                break

            subobject = self.quantity(self.g.value(view, MAP["subobject"]))
            subspace = self.subspace(self.g.value(view, MAP["subspace"]))
            axis_node = self.g.value(view, MAP["axis"])
            axis = self.axis(axis_node) if axis_node is not None else None

            if superobject is None:
                raise ValueError(
                    f"MAP view {view} has an unrecognized type; no view dispatcher matched"
                )
            view_map[self.id(subobject.id)] = View(
                self.id(view), superobject, subobject, subspace, axis
            )

        return view_map

    def data_structures(self):
        dispatcher = [
            (GEOM_COORD["DirectionCoordinate"], self.direction),
            (GEOM_COORD["PositionCoordinate"], self.position),
            (GEOM_COORD["OrientationCoordinate"], self.orientation),
            (GEOM_COORD["PoseCoordinate"], self.pose),
            (GEOM_COORD["VelocityTwistCoordinate"], self.velocity_twist),
            (GEOM_COORD["AccelerationTwistCoordinate"], self.acceleration_twist),
            (GEOM_COORD_EXT["PoseDifferenceCoordinate"], self.pose_difference),
            (RBDYN_COORD["WrenchCoordinate"], self.wrench),
            (QUDT_SCHEMA["Quantity"], self.quantity),
        ]

        data_structures = []
        for type_, func in dispatcher:
            for dstruct in self.g[: RDF["type"] : type_]:
                data_structures.append(func(dstruct))

        return _dedupe_by_id(data_structures)

    def closures(self, operators):
        closures = {}
        for operator in operators:
            for closure in self.g.subjects(RDF["type"], operator.type_):
                cl = (
                    operator.closure_step(self.g, self.id, closure)
                    if hasattr(operator, "closure_step")
                    else None
                )
                if cl:
                    if operator.type_ == CSTR_HDL["Controller"]:
                        # output-saturation lives on cstr-hdl-ext:limits (the non-integral
                        # SignalLimiter); the closure the step template renders is built from
                        # input/output/param edges, so attach the clamp explicitly.
                        sat_node = next(
                            (
                                n
                                for n in self.g.objects(closure, CSTR_HDL_EXT["limits"])
                                if CSTR_HDL_EXT["IntegralSaturation"] not in self.g[n : RDF["type"]]
                            ),
                            None,
                        )
                        if sat_node is not None:
                            cl["output_saturation"] = self.saturation(sat_node)
                    closures[self.id(closure)] = cl

        return closures

    def schedule(self, start, ops):
        # Start at a SolverWithInputAndOutput
        # Then traverse along data structure and collect function blocks
        q = collections.deque()
        data_structures = set()
        sched = []
        scheduled_nodes = {}
        for v in start:
            v_types = set(self.g[v : RDF["type"]])
            for op in ops:
                if op.type_ not in v_types:
                    continue

                call = self.id(v)
                if op.schedulable and call not in self.sched:
                    sched.append(call)
                    scheduled_nodes[call] = v
                    self.sched.add(call)

                for data_in in op.from_operator_to_input(self.g, v):
                    q.append(data_in)
                    data_structures.add(data_in)

        # An operator/call: data_in --(in)--> call --(out)--> data_out (traversed backward here;
        # there may be multiple inputs/outputs).
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

                for data_in in res["data_structures"]:
                    # We have already visited this data structure,
                    # so skip it
                    if data_in in data_structures:
                        continue

                    q.append(data_in)
                    data_structures.add(data_in)

            # An inline/declared Pose is no operator's output, so follow its coordinate /
            # reference-value edges to schedule the closures producing its scalar components --
            # else an Arc/Lerp ending in a declared pose assembles from zeros -> wrong endpoint.
            for successor in itertools.chain(
                self.g.objects(data_out, GEOM_COORD["has-coordinate"]),
                self.g.objects(data_out, CSTR["reference-value"]),
            ):
                if successor not in data_structures:
                    q.append(successor)
                    data_structures.add(successor)

        sched.reverse()
        return self._topological_schedule(sched, scheduled_nodes, ops)

    def _operator_outputs(self, node, op):
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
            node_types = set(self.g[node : RDF["type"]])
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


# ---------------------------------------------------------------------------
# Motion units
# ---------------------------------------------------------------------------
def _upstream_dependencies(data_id: str, closure_input_map: dict[str, set[str]]) -> set[str]:
    """Transitive set of data ids that feed the given id through the closure input map."""
    result: set[str] = set()
    pending = list(closure_input_map.get(data_id, set()))
    while pending:
        item = pending.pop()
        if item in result:
            continue
        result.add(item)
        pending.extend(closure_input_map.get(item, set()))
    return result


def _body_name(name: str | None) -> str | None:
    """Strip a frame_/link_ prefix to the bare MuJoCo body name."""
    if name is None:
        return None
    for prefix in ("frame_", "frame-", "link_", "link-"):
        if name.startswith(prefix):
            return name[len(prefix) :]
    return name


def _mark_acceleration_constraint_frames(solver):
    """Flag each of a solver's acceleration constraints as base-aligned when its axis frame is the
    chain root (or unset).
    """
    root_body = _body_name(getattr(solver, "chain_root", None))
    for driver in solver.motion_drivers:
        for constraint in driver.acceleration_constraint:
            axis_frame = getattr(constraint.as_seen_by, "id", None)
            constraint.base_aligned = axis_frame is None or _body_name(axis_frame) == root_body


def _filtered_motion_driver(driver, handler_output_ids: set[str], closure_input_map):
    """Return the slice of a solver driver fed by the current handler outputs."""
    acceleration_constraints = [
        ac
        for ac in driver.acceleration_constraint
        if ac.acceleration_energy.id in handler_output_ids
    ]

    cartesian_forces = []
    for force_spec in driver.cartesian_force:
        force_id = force_spec.force.id
        upstream = {force_id} | _upstream_dependencies(force_id, closure_input_map)
        if upstream & handler_output_ids:
            cartesian_forces.append(force_spec)

    joint_forces = [
        jf_spec for jf_spec in driver.joint_force if jf_spec.force_id in handler_output_ids
    ]

    if not acceleration_constraints and not cartesian_forces and not joint_forces:
        return None

    return replace(
        driver,
        acceleration_constraint=acceleration_constraints,
        cartesian_force=cartesian_forces,
        joint_force=joint_forces,
        has_cartesian_force=bool(cartesian_forces),
    )


def _arm_solvers_for_handler(handler, slv_arm, closure_input_map=None):
    """Find arm solvers whose motion drivers consume this handler's controller outputs.

    A handler drives an arm solver through controller ``control_signal`` quantities.
    Those quantities are referenced by the solver graph either as
    ``acceleration-energy`` entries in acceleration constraints or as ``force``
    entries in cartesian-force specifications.
    """
    closure_input_map = closure_input_map or {}
    handler_output_ids = {c.control_signal.id for c in handler.controllers}
    if not handler_output_ids:
        return []

    result = []
    motion_driver_id = f"driver_{handler.motion.id.removeprefix('motion_')}"
    for solver in slv_arm:
        matched = []
        for driver in solver.motion_drivers:
            filtered = _filtered_motion_driver(driver, handler_output_ids, closure_input_map)
            if filtered is not None:
                matched.append(filtered)
        if not matched:
            continue
        selected = next((driver for driver in matched if driver.id == motion_driver_id), matched[0])
        result.append(
            HandlerArmSolver(
                id=solver.id,
                output=solver.output,
                motion_driver=selected,
                control_mode=handler.control_mode,
                algorithm=solver.algorithm,
                algorithm_is_rne=solver.algorithm_is_rne,
                root_acc=solver.root_acc,
                chain_root=solver.chain_root,
                chain_end=solver.chain_end,
                torque_saturation=solver.torque_saturation,
            )
        )

    return result


def _relative_poses_for_motion(evaluators, view_map, arm_solvers):
    """Detect Pose quantities whose wrt frame ends in _start and pair them with FK outputs."""
    fk_poses: dict[str, str] = {}  # of_id → FK pose id
    for solver in arm_solvers:
        for out in solver.output:
            if getattr(out, "type", "") == "Pose":
                of_id = getattr(getattr(out, "of", None), "id", None)
                if of_id:
                    fk_poses[of_id] = out.id

    start_rel_poses: dict[str, object] = {}
    for ev in evaluators:
        qty = getattr(getattr(ev, "constraint", None), "quantity", None)
        if qty is None or not getattr(qty, "has_view", False):
            continue
        view = view_map.get(qty.id)
        if view is None:
            continue
        wrt = getattr(getattr(view, "superobject", None), "with_respect_to", None)
        if wrt and getattr(wrt, "id", "").endswith("_start"):
            pose_id = view.superobject.id
            start_rel_poses[pose_id] = view.superobject

    result = []
    for pose_id, pose in start_rel_poses.items():
        of_id = getattr(getattr(pose, "of", None), "id", None)
        fk_pose_id = fk_poses.get(of_id) if of_id else None
        if fk_pose_id:
            result.append(RelativePoseCapture(id=pose_id, fk_pose_id=fk_pose_id))
    return result


def _constraint_reference_value_id(constraint):
    """Id of a constraint's reference-value parameter, or None."""
    param = getattr(constraint, "parameter", None)
    ref = getattr(param, "reference_value", None) if param else None
    return getattr(ref, "id", None)


def _snapshot_reference_value_ids(evaluators, constraints):
    """Set of reference-value ids across the given evaluators and constraints."""
    ref_val_ids = set()
    for ev in evaluators:
        if ev.constraint is None:
            continue
        ref_id = _constraint_reference_value_id(ev.constraint)
        if ref_id:
            ref_val_ids.add(ref_id)
    for constraint in constraints:
        ref_id = _constraint_reference_value_id(constraint)
        if ref_id:
            ref_val_ids.add(ref_id)
    return ref_val_ids


def _snapshots_for_motion(
    evaluators,
    constraints,
    snapshot_source_map,
    view_map,
    closure_output_map,
    data_reference_map=None,
    schedule=None,
    closures=None,
    snapshot_clock_map=None,
):
    """Build a motion's snapshot captures (sampled source, clock and persistence) from its
    evaluators, constraints and reference maps.
    """
    snapshot_clock_map = snapshot_clock_map or {}
    ref_val_ids = _snapshot_reference_value_ids(evaluators, constraints)
    closures = closures or {}
    data_reference_map = data_reference_map or {}

    def object_id(value):
        if isinstance(value, dict):
            return value.get("id")
        return getattr(value, "id", None)

    def object_field(value, field):
        if isinstance(value, dict):
            return value.get(field)
        return getattr(value, field, None)

    subobjects_by_super: dict[str, list[str]] = {}
    supers_by_subobject: dict[str, list[str]] = {}
    for view in view_map.values():
        super_id = object_id(object_field(view, "superobject"))
        subobject_id = object_id(object_field(view, "subobject"))
        if super_id and subobject_id:
            subobjects_by_super.setdefault(super_id, []).append(subobject_id)
            supers_by_subobject.setdefault(subobject_id, []).append(super_id)

    def add_reference(ref_id):
        if not isinstance(ref_id, str):
            return
        pending = [ref_id]
        expanded = set()
        while pending:
            current = pending.pop()
            if current in expanded:
                continue
            expanded.add(current)
            ref_val_ids.add(current)
            referenced = data_reference_map.get(current)
            if referenced:
                pending.append(referenced)
            # superobject -> subobject (forward decomposition), and subobject ->
            # superobject (a composite pose's snapshot is only referenced through its
            # scalar components, so climb back to capture the composite).
            pending.extend(subobjects_by_super.get(current, ()))
            pending.extend(supers_by_subobject.get(current, ()))

    for ref_id in list(ref_val_ids):
        add_reference(ref_id)
    for step in schedule or []:
        closure = closures.get(step) or {}
        for value in closure.values():
            if isinstance(value, str):
                add_reference(value)
            elif isinstance(value, list):
                for item in value:
                    add_reference(item)
    result = []
    seen = set()
    for target_id in sorted(ref_val_ids):
        if target_id not in snapshot_source_map or target_id in seen:
            continue
        seen.add(target_id)
        source_id = snapshot_source_map[target_id]
        source_closure_id = None if source_id in view_map else closure_output_map.get(source_id)
        result.append(
            SnapshotCapture(
                target_id=target_id,
                source_id=source_id,
                source_closure_id=source_closure_id,
                clock=snapshot_clock_map.get(target_id, "task"),
            )
        )

    # If the motion resets on re-entry (has any `on entry` sample), its `on task`
    # samples must survive that reset -> persistent. Otherwise nothing persists.
    has_entry = any(s.clock == "entry" for s in result)
    for s in result:
        s.persistent = has_entry and s.clock == "task"
    return result


_CLOSURE_OUTPUT_FIELDS = {
    "PoseToAngleAroundAxis": "angle",
    "PoseToLinearDistance": "distance",
    "PoseToDirection": "direction",
    "RotateDirectionDistalToProximalWithPose": "to",
    "ComposePose": "composite",
    "InvertPose": "out",
    "PoseDiffEvaluator": "out",
    "RotateVelocityTwistToProximalWithPose": "to",
    "InvertAngle": "out",
    "AddWrench": "out",
    "AddQuantity": "out",
    "Norm": "out",
    "RotateWrenchToDistalWithPose": "to",
    "RotateWrenchToProximalWithPose": "to",
    "TransformWrenchToProximal": "to",
    "WrenchFromPositionDirectionAndMagnitude": "wrench",
    "VelocityProfile": "reference",
    "Admittance": "reference",
}


def _scene_relative_poses_for_motion(view_map, arm_solvers, data_structures=None, evaluators=None):
    """For each view whose wrt-frame is a scene object, emit the requested relative pose."""
    fk_pose_by_frame: dict[str, str] = {}
    # Keys are the scene-object's id (the "of" of scene-object solver pose outputs).
    # A wrt_id lookup asks: "is this frame the subject of a tracked scene-object pose?"
    scene_pose_by_id: dict[str, tuple[str, str | None]] = {}
    solver_output_ids: set[str] = set()
    for solver in arm_solvers:
        for out in solver.output:
            if getattr(out, "type", "") != "Pose":
                continue
            solver_output_ids.add(out.id)
            of = getattr(out, "of", None)
            if of is None:
                chain_end = getattr(solver, "chain_end", None)
                if chain_end:
                    fk_pose_by_frame.setdefault(chain_end, out.id)
                continue
            if getattr(of, "is_scene_object", False):
                scene_pose_by_id[of.id] = (
                    out.id,
                    getattr(getattr(out, "with_respect_to", None), "id", None),
                )
                body = getattr(of, "body", None)
                if body:
                    scene_pose_by_id[body] = scene_pose_by_id[of.id]
            else:
                fk_pose_by_frame[of.id] = out.id

    for view in view_map.values():
        so = getattr(view, "superobject", None)
        if so is None or not hasattr(so, "of") or not hasattr(so, "with_respect_to"):
            continue
        if getattr(so, "id", None) not in solver_output_ids:
            continue
        of = getattr(so, "of", None)
        if of is None or getattr(of, "is_scene_object", False):
            continue
        fk_pose_by_frame.setdefault(getattr(of, "id", ""), so.id)

    candidate_poses = [
        getattr(view, "superobject", None)
        for view in view_map.values()
        if not getattr(getattr(view, "superobject", None), "authored", False)
    ]
    for evaluator in evaluators or []:
        constraint = getattr(evaluator, "constraint", None)
        quantity = getattr(constraint, "quantity", None)
        if (
            quantity is not None
            and hasattr(quantity, "of")
            and hasattr(quantity, "with_respect_to")
        ):
            candidate_poses.append(quantity)

    seen: set[str] = set()
    result: list[SceneRelativePose] = []
    for so in candidate_poses:
        if so is None:
            continue
        pose_id = getattr(so, "id", None)
        if not pose_id or pose_id in seen:
            continue
        of = getattr(so, "of", None)
        wrt = getattr(so, "with_respect_to", None)
        if of is None or wrt is None:
            continue
        of_id = getattr(of, "id", "")
        wrt_id = getattr(wrt, "id", "")
        if getattr(of, "is_scene_object", False) or of_id in scene_pose_by_id:
            continue
        scene_pose = scene_pose_by_id.get(wrt_id)
        if scene_pose is None and not getattr(wrt, "is_scene_object", False):
            continue
        fk_pose_id = fk_pose_by_frame.get(of_id)
        if fk_pose_id and scene_pose:
            scene_pose_id, scene_wrt_id = scene_pose
            as_seen_by_id = getattr(getattr(so, "as_seen_by", None), "id", None)
            seen.add(pose_id)
            result.append(
                SceneRelativePose(
                    id=pose_id,
                    fk_pose_id=fk_pose_id,
                    scene_pose_id=scene_pose_id,
                    base_seen=as_seen_by_id == scene_wrt_id,
                )
            )
    return result


_GROUPABLE_SO_TYPES = {"Pose", "VelocityTwist", "AccelerationTwist", "Wrench"}

_SUBSPACE_TO_GROUP_AXIS: dict[Subspace, tuple[str, bool]] = {
    Subspace.Linear: ("linear", False),
    Subspace.Angular: ("angular", True),
}


def _pose_axis_error_groups_for_motion(eval_nodes, p, view_map):
    """Group a motion's per-axis pose error evaluators into PoseAxisErrorGroups, one per superobject
    pose.
    """
    groups: dict[str, PoseAxisErrorGroup] = {}
    for eval_node in eval_nodes:
        if GEOM_OP_EXT["PoseDiffEvaluator"] in p.g[eval_node : RDF["type"]]:
            continue
        if CSTR_HDL["ErrorEvaluator"] not in p.g[eval_node : RDF["type"]]:
            continue

        evaluator = p.constraint_evaluator(eval_node)
        if not isinstance(evaluator.constraint.parameter, EqualityConstraint):
            continue
        if evaluator.error is None:
            continue

        quantity = evaluator.constraint.quantity
        view = view_map.get(quantity.id)
        if view is None:
            continue
        so_type = getattr(view.superobject, "type", None)
        if so_type not in _GROUPABLE_SO_TYPES:
            continue

        mapping = _SUBSPACE_TO_GROUP_AXIS.get(view.subspace)
        if mapping is None:
            qkind = getattr(getattr(quantity, "quantity_kind", None), "id", "")
            quantity_id = getattr(quantity, "id", "")
            if so_type == "Pose" and (
                "Angle" in qkind or "angle" in qkind.lower() or "rotation" in quantity_id
            ):
                mapping = ("angular", True)
            else:
                continue
        subspace, is_angular = mapping

        # A whole-subspace view (e.g. keeping <pose>.position as a unit) has no per-axis
        # component, so it cannot join a per-axis pose-error group; it is controlled by its
        # own equality-constraint controller instead. Skip it rather than deref a None axis.
        if view.axis is None:
            continue

        so_id = view.superobject.id
        group = groups.setdefault(
            so_id,
            PoseAxisErrorGroup(
                id=f"pose_axis_error_{so_id}", pose=so_id, components=[], superobject_type=so_type
            ),
        )
        if is_angular:
            group.has_angular = True
        group.components.append(
            PoseAxisErrorComponent(
                quantity=evaluator.constraint.quantity.id,
                error=evaluator.error.id,
                reference=evaluator.constraint.parameter.reference_value.id,
                subspace=subspace,
                axis=view.axis.value,
                eval_id=evaluator.id,
            )
        )
        setattr(
            group,
            f"{subspace}_{view.axis.value.lower()}",
            evaluator.constraint.parameter.reference_value.id,
        )

    return [group for group in groups.values() if len(group.components) > 1]


def build_motion_units(
    g,
    p,
    handlers,
    node_by_id,
    slv_arm,
    snapshot_source_map=None,
    view_map=None,
    closure_output_map=None,
    data_reference_map=None,
    closure_input_map=None,
    closures=None,
    data_structures=None,
    snapshot_clock_map=None,
    pose_components=None,
    fsm=None,
):
    """Build the per-motion IR units (one motion per handler), each complete with schedules,
    monitors, controllers, conditions, declared poses, FSM wiring and function-interface flags;
    returns (motions, fsm_meta).
    """
    snapshot_clock_map = snapshot_clock_map or {}
    snapshot_source_map = snapshot_source_map or {}
    view_map = view_map or {}
    closure_output_map = closure_output_map or {}
    data_reference_map = data_reference_map or {}
    closure_input_map = closure_input_map or {}
    motions = []
    primary_robot_id = next((s.id for s in slv_arm if getattr(s, "id", "")), "")

    for handler in handlers:
        handler_node = node_by_id[handler.id]
        motion_node = g.value(handler_node, CSTR_HDL["motion"])
        motion = handler.motion

        # Classify constraints by motion phase via RDF traversal
        _raw_when = set(g[motion_node : MOT["when"]])
        when_constraint_nodes = set()
        for node in _raw_when:
            if CSTR_EXT.ConstraintDisjunction in g[node : RDF["type"]]:
                when_constraint_nodes.update(g[node : CSTR_EXT["has-constraint"]])
            else:
                when_constraint_nodes.add(node)
        while_constraint_nodes = set(g[motion_node : MOT["while"]])
        _raw_until = set(g[motion_node : MOT["until"]])
        until_constraint_nodes = set()
        for node in _raw_until:
            if CSTR_EXT.ConstraintDisjunction in g[node : RDF["type"]]:
                until_constraint_nodes.update(g[node : CSTR_EXT["has-constraint"]])
            else:
                until_constraint_nodes.add(node)

        # Classify evaluators by which phase their constraint belongs to
        when_eval_nodes, while_eval_nodes, until_eval_nodes = [], [], []
        for eval_node in g[handler_node : CSTR_HDL["evaluators"]]:
            cstr_node = g.value(eval_node, CSTR_HDL["constraint"])
            if cstr_node in when_constraint_nodes:
                when_eval_nodes.append(eval_node)
            elif cstr_node in while_constraint_nodes:
                while_eval_nodes.append(eval_node)
            elif cstr_node in until_constraint_nodes:
                until_eval_nodes.append(eval_node)

        def _is_elapsed_eval(eval_node):
            # Timing evaluator: measured quantity is kind Time. No kinematic computation,
            # so it must stay out of the schedule (the monitor reads the clock directly).
            cnode = g.value(eval_node, CSTR_HDL["constraint"])
            if cnode is None:
                return False
            qnode = g.value(cnode, CSTR["quantity"])
            return (
                qnode is not None
                and QUDT_QKIND["Time"] in g[qnode : QUDT_SCHEMA["hasQuantityKind"]]
            )

        # Classify controllers: driven by while-evaluator error outputs.
        # PoseDiffEvaluator exposes its components as normal MAP views over the
        # operator output twist, so collect those view subobjects here.
        while_error_nodes = set()
        while_pose_eval_nodes = []
        for n in while_eval_nodes:
            if GEOM_OP_EXT["PoseDiffEvaluator"] in g[n : RDF["type"]]:
                while_pose_eval_nodes.append(n)
                pose_diff_out = g.value(n, GEOM_OP_EXT["out"])
                for view_node in g.subjects(MAP["superobject"], pose_diff_out):
                    error_node = g.value(view_node, MAP["subobject"])
                    if error_node is not None:
                        while_error_nodes.add(error_node)
                continue
            error_node = g.value(n, CSTR_HDL["error"])
            if error_node is not None:
                while_error_nodes.add(error_node)
        while_error_nodes.discard(None)
        ctrl_nodes = [
            n
            for n in g[handler_node : CSTR_HDL["controllers"]]
            if (
                g.value(n, CSTR_HDL["error-signal"]) in while_error_nodes
                or g.value(n, CSTR_HDL["constraint"]) in while_constraint_nodes
            )
        ]

        # Classify monitors by the constraint they watch (via cstr-hdl:constraint).
        # Fall back to error-signal bucketing for monitors without a constraint link.
        when_cstr_nodes = {g.value(n, CSTR_HDL["constraint"]) for n in when_eval_nodes}
        while_cstr_nodes = {g.value(n, CSTR_HDL["constraint"]) for n in while_eval_nodes}
        until_cstr_nodes = {g.value(n, CSTR_HDL["constraint"]) for n in until_eval_nodes}
        when_cstr_nodes.discard(None)
        while_cstr_nodes.discard(None)
        until_cstr_nodes.discard(None)

        when_error_nodes = {g.value(n, CSTR_HDL["error"]) for n in when_eval_nodes}
        until_error_nodes = {g.value(n, CSTR_HDL["error"]) for n in until_eval_nodes}
        when_error_nodes.discard(None)
        until_error_nodes.discard(None)

        when_mon_nodes, while_mon_nodes, until_mon_nodes = [], [], []
        for mon_node in g[handler_node : CSTR_HDL["monitors"]]:
            if g.value(mon_node, CSTR_HDL_EXT["monitors-when"]) is not None:
                when_mon_nodes.append(mon_node)
                continue
            if g.value(mon_node, CSTR_HDL_EXT["monitors-until"]) is not None:
                until_mon_nodes.append(mon_node)
                continue
            mon_cstr = g.value(mon_node, CSTR_HDL["constraint"])
            if mon_cstr is not None:
                if mon_cstr in when_cstr_nodes:
                    when_mon_nodes.append(mon_node)
                elif mon_cstr in while_cstr_nodes:
                    while_mon_nodes.append(mon_node)
                elif mon_cstr in until_cstr_nodes:
                    until_mon_nodes.append(mon_node)
            else:
                mon_error = g.value(mon_node, CSTR_HDL["error"])
                if mon_error in when_error_nodes:
                    when_mon_nodes.append(mon_node)
                elif mon_error in while_error_nodes:
                    while_mon_nodes.append(mon_node)
                elif mon_error in until_error_nodes:
                    until_mon_nodes.append(mon_node)

        # Validate classified sets are subsets of what the handler declares. Evaluators/controllers
        # not linked to any motion phase are silently excluded from all schedules (e.g. sc1's extras).
        all_eval_nodes = set(g[handler_node : CSTR_HDL["evaluators"]])
        classified_eval_nodes = set(when_eval_nodes) | set(while_eval_nodes) | set(until_eval_nodes)
        if not classified_eval_nodes <= all_eval_nodes:
            raise ValueError(
                f"Handler {handler.id}: classified evaluators not a subset of handler evaluators"
            )

        all_ctrl_nodes = set(g[handler_node : CSTR_HDL["controllers"]])
        if not set(ctrl_nodes) <= all_ctrl_nodes:
            raise ValueError(
                f"Handler {handler.id}: classified controllers not a subset of handler controllers"
            )

        all_mon_nodes = set(g[handler_node : CSTR_HDL["monitors"]])
        classified_mon_nodes = set(when_mon_nodes) | set(while_mon_nodes) | set(until_mon_nodes)
        if not classified_mon_nodes <= all_mon_nodes:
            raise ValueError(
                f"Handler {handler.id}: classified monitors not a subset of handler monitors"
            )

        # when_schedule runs in can_start (own Parser + dedup set); while_/until share p_active
        # so steps evaluated in both phases emit once (see the _build_ir schedules note).
        p_when = Parser(g)
        when_schedule = p_when.schedule(
            [n for n in when_eval_nodes if not _is_elapsed_eval(n)], ops_generic + ops_cstr_hdl
        )

        handler_arm_solvers = _arm_solvers_for_handler(handler, slv_arm, closure_input_map)
        handler_output_ids = {c.control_signal.id for c in handler.controllers}
        cartesian_force_nodes = []
        for solver in handler_arm_solvers:
            driver_node = node_by_id.get(solver.motion_driver.id)
            if driver_node is None:
                continue
            for cf_node in g[driver_node : SLV["cartesian-force"]]:
                cf_force = g.value(cf_node, SLV["force"])
                if cf_force is None:
                    continue
                cf_force_id = p.id(cf_force)
                upstream = {cf_force_id} | _upstream_dependencies(cf_force_id, closure_input_map)
                if upstream & handler_output_ids:
                    cartesian_force_nodes.append(cf_node)

        # Direction-aligned ACHD constraints: their runtime direction (PoseToDirection) is no
        # evaluator/controller input, so seed the walk from the constraint nodes themselves.
        direction_constraint_nodes = []
        for solver in handler_arm_solvers:
            driver_node = node_by_id.get(solver.motion_driver.id)
            if driver_node is None:
                continue
            for spec_node in g[driver_node : SLV["acceleration-constraint"]]:
                for acc_node in g[spec_node : SLV["constraints"]]:
                    if SLV_EXT["DirectionAligned"] in g[acc_node : RDF["type"]]:
                        direction_constraint_nodes.append(acc_node)

        p_active = Parser(g)
        pose_axis_error_groups = _pose_axis_error_groups_for_motion(
            while_eval_nodes, p_active, view_map
        )
        pose_axis_error_eval_ids = {
            component.eval_id for group in pose_axis_error_groups for component in group.components
        }
        pose_axis_error_quantity_ids = {
            component.quantity for group in pose_axis_error_groups for component in group.components
        }
        pose_axis_error_compute_ids = {
            closure_output_map[quantity_id]
            for quantity_id in pose_axis_error_quantity_ids
            if quantity_id in closure_output_map
        }
        grouped_while_eval_nodes = {
            node for node in while_eval_nodes if p_active.id(node) in pose_axis_error_eval_ids
        }
        while_schedule = p_active.schedule(
            [
                n
                for n in while_eval_nodes
                if n not in while_pose_eval_nodes
                and n not in grouped_while_eval_nodes
                and not _is_elapsed_eval(n)
            ]
            + ctrl_nodes,
            ops_generic + ops_cstr_hdl,
        )
        while_schedule = [
            step
            for step in while_schedule
            if step not in pose_axis_error_eval_ids and step not in pose_axis_error_compute_ids
        ]
        while_schedule.extend(
            p_active.schedule(cartesian_force_nodes, ops_generic + ops_slv + ops_cstr_hdl)
        )
        while_schedule.extend(
            p_active.schedule(direction_constraint_nodes, ops_generic + ops_slv + ops_cstr_hdl)
        )

        # While evaluators watched only by a monitor (no controller consumes their error) aren't
        # reached by backward discovery; append them after their deps so their evaluate call emits.
        for n in while_eval_nodes:
            if GEOM_OP_EXT["PoseDiffEvaluator"] in g[n : RDF["type"]]:
                continue
            if _is_elapsed_eval(n):
                continue
            eval_id = p.id(n)
            if eval_id not in p_active.sched:
                while_schedule.append(eval_id)
                p_active.sched.add(eval_id)

        until_schedule = p_active.schedule(
            [n for n in until_eval_nodes if not _is_elapsed_eval(n)], ops_generic + ops_cstr_hdl
        )

        # Until evaluators have no controllers whose error-signal would drive their
        # backward discovery. Append them explicitly after their dependencies so the
        # template emits the computation calls in the correct order.
        for n in until_eval_nodes:
            if _is_elapsed_eval(n):
                continue
            eval_id = p.id(n)
            if eval_id not in p_active.sched:
                until_schedule.append(eval_id)
                p_active.sched.add(eval_id)

        # Same for when evaluators: the can_start template inlines them via
        # when_evaluators, but any prerequisite generic ops still need scheduling.
        # Append when evaluators that were not discovered through backward traversal.
        for n in when_eval_nodes:
            if _is_elapsed_eval(n):
                continue
            eval_id = p.id(n)
            if eval_id not in p_when.sched:
                when_schedule.append(eval_id)
                p_when.sched.add(eval_id)

        # Build Python objects from classified RDF nodes
        when_evaluators = [p.constraint_evaluator(n) for n in when_eval_nodes]
        while_evaluators = [
            p.constraint_evaluator(n)
            for n in while_eval_nodes
            if GEOM_OP_EXT["PoseDiffEvaluator"] not in g[n : RDF["type"]]
        ]
        until_evaluators = [p.constraint_evaluator(n) for n in until_eval_nodes]
        controllers = [p.controller(n) for n in ctrl_nodes]
        controller_output_ids = {
            c.control_signal.id for c in controllers if c.control_signal is not None
        }
        forwarded_commands = [
            p.forwarded_command(n)
            for n in g.subjects(RDF.type, SLV_EXT["ForwardedCommand"])
            if p.id(g.value(n, SLV_EXT["command-signal"])) in controller_output_ids
        ]
        when_monitors = [p.monitor_entry(n) for n in when_mon_nodes]
        while_monitors = [p.monitor_entry(n) for n in while_mon_nodes]
        until_monitors = [p.monitor_entry(n) for n in until_mon_nodes]

        has_when_elapsed = any(getattr(e, "is_elapsed", False) for e in when_evaluators)
        has_active_elapsed = any(
            getattr(e, "is_elapsed", False) for e in while_evaluators + until_evaluators
        )
        has_elapsed = has_when_elapsed or has_active_elapsed

        motions.append(
            GuardedMotionBlock(
                id=handler.motion.id,
                handler=handler.id,
                control_mode=handler.control_mode,
                command_robot_id=primary_robot_id,
                has_when_elapsed=has_when_elapsed,
                has_active_elapsed=has_active_elapsed,
                when_evaluators=when_evaluators,
                while_evaluators=while_evaluators,
                until_evaluators=until_evaluators,
                controllers=controllers,
                when_monitors=when_monitors,
                while_monitors=while_monitors,
                until_monitors=until_monitors,
                when_schedule=when_schedule,
                while_schedule=while_schedule,
                until_schedule=until_schedule,
                has_elapsed=has_elapsed,
                has_until_condition=bool(until_evaluators),
                until_any=handler.motion.until_any,
                when_any=handler.motion.when_any,
                arm_solvers=handler_arm_solvers,
                relative_poses=_relative_poses_for_motion(
                    while_evaluators + when_evaluators + until_evaluators,
                    view_map,
                    _arm_solvers_for_handler(handler, slv_arm),
                ),
                scene_relative_poses=_scene_relative_poses_for_motion(
                    view_map,
                    handler_arm_solvers,
                    data_structures,
                    while_evaluators + when_evaluators + until_evaluators,
                ),
                pose_axis_error_groups=pose_axis_error_groups,
                forwarded_commands=forwarded_commands,
                snapshots=(
                    _motion_snapshots := _snapshots_for_motion(
                        while_evaluators + when_evaluators + until_evaluators,
                        motion.while_ + motion.when + motion.until,
                        snapshot_source_map,
                        view_map,
                        closure_output_map,
                        data_reference_map,
                        while_schedule + when_schedule + until_schedule,
                        closures,
                        snapshot_clock_map,
                    )
                ),
                has_entry_snapshot=any(s.clock == "entry" for s in _motion_snapshots),
            )
        )

    # Invariant: one motion maps to exactly one constraint handler. A repeated
    # motion id (the same motion driven by two handlers) is rejected rather than
    # silently merged — that ambiguity is a modelling error, not a compose feature.
    handler_by_motion: dict[str, str] = {}
    for motion in motions:
        if motion.id in handler_by_motion:
            raise ValueError(
                f"Motion '{motion.id}' is governed by more than one constraint handler "
                f"('{handler_by_motion[motion.id]}' and '{motion.handler}'). Each motion "
                f"must map to exactly one handler; split the motion or merge the handlers."
            )
        handler_by_motion[motion.id] = motion.handler

    ordered = sorted(
        motions, key=lambda motion: next(h.order for h in handlers if h.id == motion.handler)
    )
    data_by_id = _index_by_id(data_structures or [])
    for motion in ordered:
        _set_motion_conditions(motion)
        _add_group_type_flags(motion.pose_axis_error_groups)
        _set_motion_trajectory_progress(motion, closures or {}, data_by_id)
        # Controller signal ids + this motion's declared-pose components.
        _annotate_controller_signals(motion.controllers, closures or {})
        motion_refs = collect_motion_references(motion, closures or {})
        motion.declared_pose_components = declared_pose_component_entries(
            data_structures or [], pose_components or {}, motion_refs
        )
    # FSM wiring tags monitors/motions and yields the header/step meta; then the
    # function-interface capability booleans, then the gate calls (which read them).
    fsm_meta = _apply_fsm_wiring(ordered, fsm)
    add_motion_function_interfaces(ordered)
    _apply_fsm_gate_calls(ordered, fsm_meta["fsm_namespace"])
    return ordered, fsm_meta


# ---------------------------------------------------------------------------
# Scene, geometry and robot setups
# ---------------------------------------------------------------------------
def _filter_shared_data(data_structures, schedule, closures, view_map=None, fk_output_ids=None):
    """Select the data structures that become shared_data fields (scheduled/closure/FK outputs),
    excluding view sub-objects.
    """
    referenced: set[str] = set(schedule)
    for c in closures.values():
        if isinstance(c, dict):
            for v in c.values():
                if isinstance(v, str):
                    referenced.add(v)
    if view_map:
        for view in view_map.values():
            so = getattr(view, "superobject", None)
            if so:
                referenced.add(so.id)
    if fk_output_ids:
        referenced.update(fk_output_ids)

    result = []
    for item in data_structures:
        if item.type == "Quantity" and item.has_view and item.id not in referenced:
            continue
        if (
            item.type == "Quantity"
            and getattr(item, "value", None) is None
            and getattr(getattr(item, "quantity_kind", None), "id", "x") is None
            and item.id not in referenced
        ):
            continue
        if item.type in ("Pose", "VelocityTwist") and item.id not in referenced:
            continue
        result.append(item)
    return _dedupe_by_id(result)


def _dedupe_by_id(items):
    """Deduplicate items by id, keeping the first occurrence."""
    result = []
    seen = set()
    for item in items:
        key = getattr(item, "id", None)
        if key is None:
            result.append(item)
            continue
        if key in seen:
            continue
        seen.add(key)
        result.append(item)
    return result


def _xyz_or_none(g, node):
    """Read a coordinate node's x/y/z as floats, or None if any axis is missing."""
    if node is None:
        return None
    values = [g.value(node, GEOM_COORD[axis]) for axis in ("x", "y", "z")]
    if any(v is None for v in values):
        return None
    return [float(v.value) for v in values]


def _orientation_degrees(g, node):
    """Read an orientation node's roll/pitch/yaw in degrees, or None if incomplete."""
    result: dict[str, float | None] = {"roll": None, "pitch": None, "yaw": None}
    if node is None:
        return None
    for coord in g.objects(node, GEOM_COORD["has-coordinate"]):
        axis = str(g.value(coord, GEOM_COORD["angle-axis"]) or "")
        if axis not in result:
            continue
        value_node = g.value(coord, QUDT_SCHEMA.value)
        if value_node is None:
            continue
        value = float(value_node.value)
        unit = str(g.value(coord, QUDT_SCHEMA.unit) or "")
        if unit.endswith("RAD"):
            value = math.degrees(value)
        result[axis] = value
    if any(value is None for value in result.values()):
        return None
    return [result["roll"], result["pitch"], result["yaw"]]


def _position_of(g, obj_node):
    # geom-rel:Position.of targets the frame's origin Point (geom-ent:origin),
    # not the frame node itself -- see rdf.py's _frame_origin_point().
    """World position [x, y, z] of an object's origin frame, or None."""
    origin = g.value(obj_node, GEOM_ENT["origin"]) or obj_node
    for pos_node in g.subjects(GEOM_REL["of"], origin):
        if GEOM_COORD["PositionCoordinate"] in g[pos_node : RDF.type]:
            return _xyz_or_none(g, pos_node)
    return None


def _orientation_of(g, obj_node):
    """World orientation [roll, pitch, yaw] (degrees) of an object, or None."""
    for orient_node in g.subjects(GEOM_REL["of"], obj_node):
        if GEOM_COORD["OrientationCoordinate"] in g[orient_node : RDF.type]:
            return _orientation_degrees(g, orient_node)
    return None


def _path_of_model(g, model_node):
    """Asset path (exec:path) of a model node, or the empty string."""
    if not model_node:
        return ""
    return str(g.value(model_node, EXEC.path) or "")


def _resolve_existing_path(path: str) -> Path | None:
    """Resolve an asset path against cwd, menagerie and cache roots; None if not found."""
    if not path:
        return None
    candidate = Path(path)
    if candidate.exists():
        return candidate
    for root in [Path.cwd(), *Path.cwd().parents]:
        candidate = root / path
        if candidate.exists():
            return candidate
    text = Path(path).as_posix()
    cache_root = None
    if os.environ.get("XDG_CACHE_HOME"):
        cache_root = Path(os.environ["XDG_CACHE_HOME"]) / "mj_kdl_wrapper"
    elif os.environ.get("HOME"):
        cache_root = Path(os.environ["HOME"]) / ".cache" / "mj_kdl_wrapper"

    def cache_path(marker: str, cache_subdir: str) -> Path | None:
        pos = text.find(marker)
        if pos == -1 or cache_root is None:
            return None
        return cache_root / cache_subdir / text[pos + len(marker) :]

    menagerie_marker = "third_party/menagerie/"
    pos = text.find(menagerie_marker)
    if pos != -1 and os.environ.get("MJ_KDL_MENAGERIE"):
        candidate = Path(os.environ["MJ_KDL_MENAGERIE"]) / text[pos + len(menagerie_marker) :]
        if candidate.exists():
            return candidate
    for candidate in (
        cache_path(menagerie_marker, "menagerie"),
        cache_path("src/mj_kdl_wrapper/assets/", "assets"),
        cache_path("src/examples/assets/", "assets"),
    ):
        if candidate is not None and candidate.exists():
            return candidate
    return None


def _mjcf_body_containing_site(path: str, site_name: str) -> str:
    """Name of the MJCF body that owns the named site, or the empty string."""
    resolved = _resolve_existing_path(path)
    if resolved is None or not site_name:
        return ""
    try:
        root = ET.parse(resolved).getroot()
    except ET.ParseError:
        return ""

    def visit_body(body) -> str:
        for site in body.findall("site"):
            if site.get("name") == site_name:
                return body.get("name") or ""
        for child in body.findall("body"):
            found = visit_body(child)
            if found:
                return found
        return ""

    for worldbody in root.findall("worldbody"):
        for body in worldbody.findall("body"):
            found = visit_body(body)
            if found:
                return found
    return ""


def _derive_tool_body_from_attachment(attachment: SceneAttachment, tcp_site: str) -> str:
    """Prefixed tool-body owning the tcp site within an attachment's MJCF, or empty string."""
    if not tcp_site:
        return ""
    local_site = tcp_site
    if attachment.prefix and tcp_site.startswith(attachment.prefix):
        local_site = tcp_site[len(attachment.prefix) :]
    local_body = _mjcf_body_containing_site(attachment.path, local_site)
    if not local_body:
        return ""
    return f"{attachment.prefix}{local_body}"


def _scene_id_nodes(g, env_node):
    """Graph nodes needing local ids for a workspace (objects, their models and attach targets)."""
    nodes = set()
    for obj_node in g.objects(env_node, ENV["has-object"]):
        nodes.add(obj_node)
        nodes.add(g.value(obj_node, ENV["has-object-model"]))
        nodes.add(g.value(obj_node, SLV["attached-to"]))
    return nodes


def _attach_target_of(g, obj_node, ids: LocalIdMap):
    """Attachment (kind, name) of an object, scoping a site name to its target object."""
    kind = str(g.value(obj_node, MJ["attach-kind"]) or "world").title()
    if kind not in {"World", "Body", "Site", "Frame"}:
        kind = "World"
    name = str(g.value(obj_node, MJ["attach-name"]) or "")
    target = g.value(obj_node, SLV["attached-to"])
    if kind == "Site" and target is not None and ENV.Object in g[target : RDF.type]:
        target_name = ids[target]
        if target_name and name and not name.startswith(f"{target_name}_{target_name}_"):
            name = f"{target_name}_{name}"
    return kind, name


def _transitive_attachment_nodes(g, env_node, robot_node):
    """Attachment nodes reachable from the robot through SLV:attached-to, parent-first.

    Supports chained attachments (e.g. gripper -> ft-sensor -> robot), not just those
    bolted directly to the robot. Ordered so a parent always precedes its children,
    which is the order MuJoCo needs (a child's target site only exists once its parent
    is attached).
    """
    candidates = []
    for c in g.objects(env_node, ENV["has-object"]):
        path = _path_of_model(g, c) or _path_of_model(g, g.value(c, ENV["has-object-model"]))
        if path:
            candidates.append((c, g.value(c, SLV["attached-to"])))
    ordered = []
    resolved = {robot_node}
    progress = True
    while progress:
        progress = False
        for c, target in candidates:
            if c in resolved or target not in resolved:
                continue
            ordered.append(c)
            resolved.add(c)
            progress = True
    return ordered


def _attachments_for_robot(g, env_node, robot_node, tool_body, ids: LocalIdMap):
    """Ordered SceneAttachment list bolted onto a robot, resolving chained (parent-first)
    attachments.
    """
    attachments = []
    prefix_by_node = {}
    for candidate in _transitive_attachment_nodes(g, env_node, robot_node):
        path = _path_of_model(g, candidate)
        if not path:
            path = _path_of_model(g, g.value(candidate, ENV["has-object-model"]))
        if not path:
            continue
        attach_node = g.value(candidate, MJ["attach-to-body"])
        attach_kind_lit = g.value(candidate, MJ["attach-kind"])
        if attach_kind_lit is None:
            raise ValueError(
                f"Attachment '{candidate}' is missing mj:attach-kind; "
                f"add an 'attach-to:' entry to the .robmot model."
            )
        attach_kind = str(attach_kind_lit).title()
        if attach_kind not in {"Body", "Site", "Frame"}:
            raise ValueError(
                f"Attachment '{candidate}' has unsupported attach-kind '{attach_kind}'; "
                f"expected Body, Site, or Frame."
            )
        attach_to = (
            str(g.value(attach_node, MJ["site-name"]) or "")
            if attach_kind == "Site"
            else str(g.value(attach_node, MJ["body-name"]) or "")
        )
        prefix_lit = g.value(candidate, MJ["attach-prefix"])
        prefix = str(prefix_lit) if prefix_lit is not None else ""
        # When attached to another (prefixed) attachment, that parent's sites/bodies
        # carry its prefix in the compiled model, so prefix the target name to match.
        parent_prefix = prefix_by_node.get(g.value(candidate, SLV["attached-to"]), "")
        if parent_prefix and attach_to and not attach_to.startswith(parent_prefix):
            attach_to = parent_prefix + attach_to
        pos = _xyz_or_none(g, g.value(candidate, MJ["attach-position"]))
        euler = _orientation_degrees(g, g.value(candidate, MJ["attach-orientation"]))
        actuator = str(g.value(candidate, MJ["actuator-name"]) or "")
        attachments.append(
            SceneAttachment(
                id=ids[candidate],
                path=path,
                attach_to=attach_to,
                attach_kind=attach_kind,
                prefix=prefix,
                pos=pos,
                euler=euler,
                actuator=actuator,
            )
        )
        prefix_by_node[candidate] = prefix
    return attachments


def _color_rgba(g, owner_node):
    """Read a grouped mj:ColorRGBA value node (via mj:color) as [r, g, b, a].

    Returns None when the owner has no colour, so callers can fall back to their
    own defaults. Shared by scene objects and the trajectory-trace overlay.
    """
    color_node = g.value(owner_node, MJ["color"])
    if color_node is None:
        return None
    channels = [g.value(color_node, MJ[f"color-{ch}"]) for ch in ("r", "g", "b", "a")]
    if any(c is None for c in channels):
        return None
    return [float(c.toPython()) for c in channels]


def _trace_from_graph(g):
    """Read the optional MuJoCo trajectory-trace overlay config from the graph.

    The trace is a viewer-only overlay (a polyline of recent EE positions). When
    no TRACE block is declared it stays disabled, so headless runs and non-MuJoCo
    runtimes cost nothing. Defaults match the wrapper's built-in warm orange.
    """
    trace = {
        "enabled": False,
        "length": 4096,
        "color_r": 1.0,
        "color_g": 0.5,
        "color_b": 0.1,
        "color_a": 1.0,
        "targets": [],
        "has_targets": False,
    }
    trace_node = next(g.objects(predicate=MJ["has-trace"]), None)
    if trace_node is None:
        return trace

    enabled = g.value(trace_node, MJ["trace-enabled"])
    if enabled is not None:
        trace["enabled"] = bool(enabled.toPython())
    length = g.value(trace_node, MJ["trace-length"])
    if length is not None:
        trace["length"] = int(length.toPython())
    color = _color_rgba(g, trace_node)
    if color is not None:
        trace["color_r"], trace["color_g"], trace["color_b"], trace["color_a"] = color
    for target in g.objects(trace_node, MJ["trace-target"]):
        trace["targets"].append({"link": str(target)})
    trace["has_targets"] = bool(trace["targets"])
    return trace


def _scene_from_graph(g):
    """Build the scene (robots, objects, placement and geometry) from the workspace graph."""
    scene = SceneSpec()
    for env_node in g.subjects(RDF.type, ENV.Workspace):
        ids = LocalIdMap(g, _scene_id_nodes(g, env_node))
        _timestep = g.value(env_node, MJ["timestep"])
        if _timestep is not None:
            scene.timestep_s = float(_timestep.toPython())
        robot_nodes = []
        for obj_node in g.objects(env_node, ENV["has-object"]):
            if g.value(obj_node, GEOM_ENT["kinematic-chain"]) is not None:
                robot_nodes.append(obj_node)

        for robot_node in robot_nodes:
            model_node = g.value(robot_node, ENV["has-object-model"])
            robot_path = _path_of_model(g, model_node)
            if not robot_path:
                continue

            tool_body_node = g.value(robot_node, MJ["tool-body"])
            tool_body = (
                str(g.value(tool_body_node, MJ["body-name"]) or "") if tool_body_node else ""
            )
            attachments = _attachments_for_robot(g, env_node, robot_node, tool_body, ids)
            attach_kind, attach_name = _attach_target_of(g, robot_node, ids)

            scene.robots.append(
                SceneRobot(
                    id=ids[robot_node],
                    path=robot_path,
                    prefix=str(g.value(robot_node, MJ["prefix"]) or ""),
                    attach_kind=attach_kind,
                    attach_name=attach_name,
                    pos=_position_of(g, robot_node),
                    euler=_orientation_of(g, robot_node),
                    attachments=attachments,
                )
            )

        for obj_node in g.objects(env_node, ENV["has-object"]):
            if obj_node in robot_nodes:
                continue
            if ENV.RigidObject not in g[obj_node : RDF.type]:
                continue
            model_node = g.value(obj_node, ENV["has-object-model"])
            path = _path_of_model(g, model_node)
            body = str(g.value(obj_node, MJ["body-name"]) or "") or ids[obj_node]
            obj_types = set(g[obj_node : RDF.type])
            if POLY.CuboidWithSize in obj_types:
                shape = "BOX"
            else:
                shape_lit = g.value(obj_node, MJ["shape"])
                shape = str(shape_lit).upper() if shape_lit is not None else None
            size_x = g.value(obj_node, POLY["x-size"])
            size_y = g.value(obj_node, POLY["y-size"])
            size_z = g.value(obj_node, POLY["z-size"])
            size = (
                [float(size_x.value), float(size_y.value), float(size_z.value)]
                if size_x is not None and size_y is not None and size_z is not None
                else None
            )
            mass_value = None
            for mass_node in g.subjects(RBDYN_ENT["of-body"], obj_node):
                if RBDYN_ENT.Mass in g[mass_node : RDF.type]:
                    mv = g.value(mass_node, RBDYN_ENT.mass)
                    if mv is not None:
                        mass_value = float(mv.value)
                        break
            attach_kind, attach_name = _attach_target_of(g, obj_node, ids)
            f_slide = g.value(obj_node, MJ["friction-slide"])
            f_torsion = g.value(obj_node, MJ["friction-torsion"])
            f_roll = g.value(obj_node, MJ["friction-roll"])
            friction = (
                [float(f_slide.value), float(f_torsion.value), float(f_roll.value)]
                if f_slide is not None and f_torsion is not None and f_roll is not None
                else None
            )
            color = _color_rgba(g, obj_node)
            scene.objects.append(
                SceneObjectSpec(
                    id=ids[obj_node],
                    body=body,
                    path=path,
                    attach_kind=attach_kind,
                    attach_name=attach_name,
                    pos=_position_of(g, obj_node),
                    fixed=bool(path),
                    shape=shape,
                    size=size,
                    color=color,
                    mass=mass_value,
                    friction=friction,
                )
            )
        break
    _expand_scene_geometry(scene)
    return scene


def _expand_scene_geometry(scene) -> None:
    """Expand placement vectors and (present) procedural geometry onto the native scene
    items so the scene is codegen-complete at construction. Env placement shorthand: an
    omitted position/orientation means zero/identity. Path-backed objects take geometry
    from their MJCF/URDF asset, so their flat size/color/friction fields stay unset.
    Missing required geometry is not raised here (that is ``_validate_scene``) so building a
    scene never depends on a downstream pass."""
    for robot in scene.robots:
        expand_vector_fields(robot, "pos")
        expand_vector_fields(robot, "euler")
        for attachment in robot.attachments:
            expand_vector_fields(attachment, "pos")
            expand_vector_fields(attachment, "euler")
    for obj in scene.objects:
        expand_vector_fields(obj, "pos")
        expand_vector_fields(obj, "euler")
        obj.has_path = bool(obj.path)
        if obj.has_path:
            continue
        if obj.size is not None:
            obj.size_x, obj.size_y, obj.size_z = (
                float(obj.size[0]),
                float(obj.size[1]),
                float(obj.size[2]),
            )
        if obj.color is not None:
            obj.color_r, obj.color_g, obj.color_b, obj.color_a = (
                float(obj.color[0]),
                float(obj.color[1]),
                float(obj.color[2]),
                float(obj.color[3]),
            )
        if obj.friction is not None:
            obj.friction_slide, obj.friction_torsion, obj.friction_roll = (
                float(obj.friction[0]),
                float(obj.friction[1]),
                float(obj.friction[2]),
            )


def _validate_scene(scene) -> None:
    """Every procedural (non-path) scene object must carry full geometry — the model must
    declare it; silent defaults are not applied. Runs at construction time in generate_ir."""
    for obj in scene.objects:
        if bool(obj.path):
            continue
        require_field(obj.id, "size", obj.size)
        require_field(obj.id, "color", obj.color)
        require_field(obj.id, "friction", obj.friction)
        require_field(obj.id, "shape", obj.shape)
        require_field(obj.id, "mass", obj.mass)


def _robot_setups_from_graph(g):
    """Per-robot solver chain setups for the whole workspace.

    Returns ``(setups_by_node, ordered)`` where ``setups_by_node`` maps each
    robot's graph node to its setup tuple
    ``(urdf, chain_root, chain_end, chain_tip, robot_model, tool_body, tcp_site, ft_sensors)``
    and ``ordered`` is the same tuples in declaration order. Names derived from
    the robot's own MJCF (``chain_tip``, ``tool_body``) get the robot's prefix
    prepended so they resolve in the prefixed, multi-robot MuJoCo scene;
    already-prefixed (authored) names are left untouched.
    """
    setups_by_node = {}
    ordered = []
    for env_node in g.subjects(RDF.type, ENV.Workspace):
        ids = LocalIdMap(g, _scene_id_nodes(g, env_node))
        for obj_node in g.objects(env_node, ENV["has-object"]):
            chain = g.value(obj_node, GEOM_ENT["kinematic-chain"])
            if chain is None:
                continue
            prefix = str(g.value(obj_node, MJ["prefix"]) or "")
            start = g.value(chain, GEOM_ENT.start)
            end = g.value(chain, GEOM_ENT.end)
            chain_root = str(g.value(start, MJ["body-name"]) or "")
            chain_end = str(g.value(end, MJ["body-name"]) or "")
            model_node = g.value(obj_node, ENV["has-object-model"])
            urdf = _path_of_model(g, model_node)
            robot_model = ids[model_node] if model_node else ""
            tool_body_node = g.value(obj_node, MJ["tool-body"])
            tcp_site_node = g.value(obj_node, MJ["tcp-site"])
            tool_body = (
                str(g.value(tool_body_node, MJ["body-name"]) or "") if tool_body_node else ""
            )
            tcp_site = str(g.value(tcp_site_node, MJ["site-name"]) or "") if tcp_site_node else ""
            ft_sensors = sorted(
                (
                    {
                        "name": str(g.value(ft_node, MJ["sensor-name"]) or ""),
                        "frame_site": str(g.value(ft_node, MJ["frame-site"]) or ""),
                    }
                    for ft_node in g.objects(obj_node, MJ["ft-sensor"])
                ),
                key=lambda s: s["name"],
            )
            attachments = _attachments_for_robot(g, env_node, obj_node, tool_body, ids)
            for attachment in _transitive_attachment_nodes(g, env_node, obj_node):
                tool_body_node = g.value(attachment, MJ["tool-body"])
                tcp_site_node = g.value(attachment, MJ["tcp-site"])
                tool_body = tool_body or (
                    str(g.value(tool_body_node, MJ["body-name"]) or "") if tool_body_node else ""
                )
                tcp_site = tcp_site or (
                    str(g.value(tcp_site_node, MJ["site-name"]) or "") if tcp_site_node else ""
                )
            if not tool_body and tcp_site:
                for attachment in attachments:
                    tool_body = _derive_tool_body_from_attachment(attachment, tcp_site)
                    if tool_body:
                        break
            chain_tip = chain_end
            if (
                tcp_site
                and chain_end == tcp_site
                and attachments
                and attachments[0].attach_kind == "Body"
            ):
                chain_tip = attachments[0].attach_to
            elif (
                tcp_site
                and chain_end == tcp_site
                and attachments
                and attachments[0].attach_kind == "Site"
            ):
                model_path = _path_of_model(g, model_node)
                site_body = _mjcf_body_containing_site(model_path, attachments[0].attach_to)
                if not site_body:
                    raise ValueError(
                        f"Cannot derive robot chain tip body: site '{attachments[0].attach_to}' "
                        f"was not found in model '{model_path}'."
                    )
                chain_tip = site_body
            if prefix:
                if chain_tip and not chain_tip.startswith(prefix):
                    chain_tip = prefix + chain_tip
                if tool_body and not tool_body.startswith(prefix):
                    tool_body = prefix + tool_body
            if not (chain_root or chain_end or urdf):
                continue
            setup = (
                urdf,
                chain_root,
                chain_end,
                chain_tip,
                robot_model,
                tool_body,
                tcp_site,
                ft_sensors,
            )
            setups_by_node[obj_node] = setup
            ordered.append(setup)
    return setups_by_node, ordered


# ---------------------------------------------------------------------------
# Introspection artifact
# ---------------------------------------------------------------------------
def _uri_table(id_nodes):
    """Sorted [{id, uri}] rows for every id that maps to a URIRef."""
    return [
        {"id": id_, "uri": str(node)}
        for id_, node in sorted(id_nodes, key=lambda item: (item[0], str(item[1])))
        if isinstance(node, URIRef)
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


def _id_refs(value):
    """Map _id_ref over a list of values."""
    if value is None:
        return []
    if isinstance(value, list):
        return [ref for item in value if (ref := _id_ref(item))]
    ref = _id_ref(value)
    return [ref] if ref else []


def _dedupe_dicts(entries, key="id"):
    """Deduplicate dict rows by id, keeping the first occurrence."""
    result = []
    seen = set()
    for entry in entries:
        value = entry.get(key)
        if value in seen:
            continue
        seen.add(value)
        result.append(entry)
    return result


def _build_introspection(
    *,
    app_model_path,
    imported_models,
    imported_provenance,
    id_nodes,
    node_by_id,
    motions,
    data_structures,
    control_period_ns,
    backend,
    scene,
    closures,
    views,
    shared_data,
):
    """Build the introspection artifact (uris, motions, controllers, monitors, quantities,
    provenance) and fold in the controller-state and frame-log samples.
    """
    uri_rows = _uri_table(id_nodes)
    uri_by_id = {row["id"]: row["uri"] for row in uri_rows}

    controllers = []
    monitors = []
    signals = []
    for motion in motions:
        for controller in motion.controllers:
            controller_entry = {
                "id": controller.id,
                "uri": uri_by_id.get(controller.id),
                "motion": motion.id,
                "type": controller.type,
                "proportional_gain": getattr(controller, "proportional_gain", None),
                "integral_gain": getattr(controller, "integral_gain", None),
                "derivative_gain": getattr(controller, "derivative_gain", None),
                "decay_rate": getattr(controller, "decay_rate", None),
                "stiffness": getattr(controller, "stiffness", None),
                "damping": getattr(controller, "damping", None),
                "error_signal": _id_ref(getattr(controller, "error_signal", None)),
                "reference_signal": _id_ref(getattr(controller, "reference_signal", None)),
                "measured_derivative": _id_ref(getattr(controller, "measured_derivative", None)),
                "output_signal": _id_ref(controller.control_signal),
            }
            controllers.append(
                {k: v for k, v in controller_entry.items() if v is not None and v != []}
            )
            for role in (
                "error_signal",
                "reference_signal",
                "measured_derivative",
                "control_signal",
            ):
                quantity_id = _id_ref(getattr(controller, role, None))
                if quantity_id:
                    signal_entry = {
                        "id": f"{controller.id}.{role}",
                        "uri": uri_by_id.get(quantity_id),
                        "quantity": quantity_id,
                        "role": role,
                        "owner": controller.id,
                    }
                    signals.append(
                        {k: v for k, v in signal_entry.items() if v is not None and v != []}
                    )
        for phase in ("when", "while", "until"):
            for monitor in getattr(motion, f"{phase}_monitors"):
                monitor_entry = {
                    "id": monitor.id,
                    "uri": uri_by_id.get(monitor.id),
                    "motion": motion.id,
                    "phase": phase,
                    "type": monitor.monitor_type,
                    "trigger": "edge" if monitor.is_edge_triggered else "level",
                    "event": getattr(monitor, "event", None),
                    "event_uri": getattr(monitor, "event_uri", None),
                    "event_name": getattr(monitor, "event_name", None),
                    "flag": getattr(monitor, "flag", None),
                    "error_signal": _id_ref(monitor.error),
                    "fallback_motion": getattr(monitor, "fallback_motion", None),
                    "debounce_duration_s": getattr(monitor, "debounce_duration_s", None),
                    "debounce_steps": getattr(monitor, "debounce_steps", None),
                }
                monitors.append(
                    {k: v for k, v in monitor_entry.items() if v is not None and v != []}
                )
                if monitor.error is not None:
                    signal_entry = {
                        "id": f"{monitor.id}.error",
                        "uri": uri_by_id.get(monitor.error.id),
                        "quantity": monitor.error.id,
                        "role": "monitor_error",
                        "owner": monitor.id,
                    }
                    signals.append(
                        {k: v for k, v in signal_entry.items() if v is not None and v != []}
                    )

    quantities = []
    for item in data_structures:
        quantity_entry = {
            "id": item.id,
            "uri": uri_by_id.get(item.id),
            "type": item.type,
            "unit": _id_refs(getattr(item, "unit", None)),
            "quantity_kind": _id_refs(getattr(item, "quantity_kind", None)),
            # Reference frame the spatial value is expressed in — the frame_id for the
            # pose/twist/wrench spatial samples.
            "reference_frame": _id_ref(getattr(item, "as_seen_by", None))
            or _id_ref(getattr(item, "with_respect_to", None)),
            "reference_value": getattr(item, "reference_value", None),
            "value": getattr(item, "value", None),
            "authored": getattr(getattr(item, "provenance", None), "authored", False),
            "snapshot": getattr(getattr(item, "provenance", None), "snapshot", False),
        }
        quantities.append({k: v for k, v in quantity_entry.items() if v is not None and v != []})

    runtime_type = {"mj_kdl": "rt:MuJoCoRuntime", "robif2b": "rt:RealRobotRuntime"}.get(
        backend, "rt:Runtime"
    )
    runtime_id = "agent:runtime:mujoco" if backend == "mj_kdl" else "agent:runtime:real_robot"
    runtime_activity_type = (
        "bdd:SimulatedExecution" if backend == "mj_kdl" else "bdd:ScenarioExecution"
    )

    entities = [
        {
            "id": "entity:app_manifest",
            "types": ["prov:Entity"],
            "role": "app_manifest",
            "path": str(app_model_path),
        },
        {
            "id": "entity:motion_spec_ir",
            "types": ["prov:Entity"],
            "role": "motion_spec_ir",
            "wasGeneratedBy": "activity:motion_spec_ir_generation",
            "wasDerivedFrom": "entity:app_manifest",
        },
    ]
    entities.extend(
        {
            "id": f"entity:imported_graph:{idx}",
            "types": ["prov:Entity"],
            "role": "imported_model_graph",
            "source": source,
        }
        for idx, source in enumerate(imported_models)
    )
    entities.extend(
        {
            "id": f"entity:imported_provenance:{idx}",
            "types": ["prov:Entity"],
            "role": "imported_provenance",
            "source": source,
        }
        for idx, source in enumerate(imported_provenance)
    )

    agents = [
        {
            "id": "agent:motion_spec_ir_gen",
            "types": ["prov:SoftwareAgent", "obs:ObservationProvider"],
            "role": "ir_generator",
        },
        {"id": runtime_id, "types": ["prov:SoftwareAgent", runtime_type], "role": "runtime_runner"},
        {
            "id": "agent:controller_process",
            "types": ["prov:SoftwareAgent"],
            "role": "controller_process",
            "actedOnBehalfOf": runtime_id,
        },
    ]
    agents.extend(
        {
            "id": f"agent:modelled:{robot.id}",
            "types": ["prov:Agent", "agn:ModelledAgent"],
            "role": "robot",
            "model": robot.path,
        }
        for robot in scene.robots
    )

    introspection = {
        "contract_version": 1,
        "control_period_ns": control_period_ns,
        "uris": uri_rows,
        "motions": [
            {
                k: v
                for k, v in {
                    "id": motion.id,
                    "uri": uri_by_id.get(motion.id),
                    "handler": motion.handler,
                    "handler_uri": uri_by_id.get(motion.handler),
                    "control_mode": motion.control_mode,
                    "controllers": [controller.id for controller in motion.controllers],
                    "monitors": [
                        monitor.id
                        for group in (
                            motion.when_monitors,
                            motion.while_monitors,
                            motion.until_monitors,
                        )
                        for monitor in group
                    ],
                }.items()
                if v is not None and v != []
            }
            for motion in motions
        ],
        "states": [],
        "controllers": _dedupe_dicts(controllers),
        "monitors": _dedupe_dicts(monitors),
        "quantities": _dedupe_dicts(quantities),
        "signals": _dedupe_dicts(signals),
        "provenance": {
            # Only id/uri are consumed; the canonical uri is the published IRI, which
            # the rdf-utils resolver maps to a local checkout. No local source/shape
            # paths are baked in — they were dead metadata and non-portable.
            "contexts": [
                {"id": "prov", "uri": "http://www.w3.org/ns/prov#"},
                {
                    "id": "bdd",
                    "uri": "https://secorolab.github.io/metamodels/acceptance-criteria/bdd#",
                },
                {"id": "agent", "uri": "https://secorolab.github.io/metamodels/agent#"},
                {"id": "observation", "uri": "https://secorolab.github.io/metamodels/observation#"},
                {"id": "runtime", "uri": str(RT.Runtime)},
            ],
            "entities": entities,
            "activities": [
                {
                    "id": "activity:motion_spec_ir_generation",
                    "types": ["prov:Activity"],
                    "used": [
                        entity["id"] for entity in entities if entity["role"] != "motion_spec_ir"
                    ],
                    "wasAssociatedWith": "agent:motion_spec_ir_gen",
                    "role": "motion_spec_ir_generation",
                },
                {
                    "id": "activity:controller_execution",
                    "types": ["prov:Activity", runtime_activity_type],
                    "used": ["entity:motion_spec_ir"],
                    "wasAssociatedWith": "agent:controller_process",
                    "role": "controller_execution",
                },
            ],
            "agents": agents,
        },
    }

    # Fold the introspection-facing derivations into the introspection piece itself:
    # controller signal ids, controller-internal-state logging (which grows shared_data),
    # then the frame-log quantity/spatial samples that read them.
    _annotate_controller_signals(introspection["controllers"], closures)
    add_controller_internal_state_logging(closures, shared_data, introspection, motions)
    add_quantity_samples(introspection, shared_data, views)
    add_spatial_samples(introspection, shared_data)
    return introspection


# ---------------------------------------------------------------------------
# Graph loading
# ---------------------------------------------------------------------------
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
    imported_provenance = []
    imported_model_locations = []
    for item in imported_files:
        if item.endswith("/provenance/dsl.jsonld"):
            imported_provenance.append(_resolve_import_location(item, url_map))
        else:
            imported_model_locations.append(item)
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


# ---------------------------------------------------------------------------
# Solver sections
# ---------------------------------------------------------------------------
def _solver_sections(g, p: Parser, setups_by_node: dict, default_setup):
    """Parse the solver/handler sections: base-velocity, arm and base-force solvers with their
    schedules.
    """
    sched1 = []
    slv_base_vel = []
    sched2 = []
    hdl = []
    sched3 = []
    slv_arm = []
    sched4 = []
    slv_base_frc = []

    for s in g.subjects(RDF.type, SLV["VelocityCompositionSolver"]):
        slv_base_vel.append(p.velocity_composition_solver(s))
        sched1.extend(p.schedule([s], ops_generic + ops_slv))

    for h in g.subjects(RDF.type, CSTR_HDL["ConstraintHandler"]):
        hdl.append(p.constraint_handler(h))
        start = g[h : CSTR_HDL["evaluators"] | CSTR_HDL["controllers"]]
        sched2.extend(p.schedule(start, ops_generic + ops_cstr_hdl))

    for s in g.subjects(RDF.type, SLV["SolverWithInputAndOutput"]):
        solver = p.solver_with_input_and_output(s)
        robot_node = g.value(s, SLV_EXT["robot"])
        (
            solver.urdf,
            solver.chain_root,
            solver.chain_end,
            solver.chain_tip,
            solver.robot_model,
            solver.tool_body,
            solver.tcp_site,
            solver.ft_sensors,
        ) = setups_by_node.get(robot_node, default_setup)
        _mark_acceleration_constraint_frames(solver)
        slv_arm.append(solver)
        start = g[
            s : SLV["motion-drivers"]
            / (
                (SLV["acceleration-constraint"] / SLV["constraints"])
                | (SLV["cartesian-force"])
                | (SLV["joint-force"])
            )
        ]
        sched3.extend(p.schedule(start, ops_generic + ops_slv))

    for s in g.subjects(RDF.type, SLV["ForceDistributionSolver"]):
        slv_base_frc.append(p.force_distribution_solver(s))
        sched4.extend(p.schedule([s], ops_generic + ops_slv))

    return slv_base_vel, sched1, hdl, sched2, slv_arm, sched3, slv_base_frc, sched4


def _assign_monitor_event_indexes(handlers) -> None:
    """Assign each monitor a stable per-handler event index."""
    event_idx = 0
    for handler in handlers:
        for monitor in handler.monitors:
            if monitor.monitor_type == "EdgeTriggeredMonitor":
                monitor.event_idx = event_idx
                event_idx += 1


def _snapshot_maps(g, p: Parser) -> tuple[dict[str, str], dict[str, str]]:
    """Build the (snapshot_source_map, snapshot_clock_map) lookups from the graph."""
    snapshot_source_map: dict[str, str] = {}
    snapshot_clock_map: dict[str, str] = {}
    for snap_node in g.subjects(RDF.type, SNAP.Snapshot):
        source_node = g.value(snap_node, SNAP["snapshot-of"])
        if source_node is not None:
            snapshot_source_map[p.id(snap_node)] = p.id(source_node)
        clock_node = g.value(snap_node, SNAP["sampled-on"])
        snapshot_clock_map[p.id(snap_node)] = (
            "entry" if clock_node == SNAP["entry-clock"] else "task"
        )
    return snapshot_source_map, snapshot_clock_map


def _data_reference_map(data_structures, closures: dict) -> dict[str, str]:
    """Map each data id to the ids it references through closures."""
    data_reference_map: dict[str, str] = {}
    for item in data_structures:
        ref = getattr(item, "reference_value", None)
        ref_id = ref if isinstance(ref, str) else getattr(ref, "id", None)
        if ref_id:
            data_reference_map[item.id] = ref_id
    for c in closures.values():
        if not isinstance(c, dict) or c.get("type") != "AssignmentEvaluator":
            continue
        quantity_id = c.get("quantity")
        ref_id = c.get("reference_value")
        if isinstance(quantity_id, str) and isinstance(ref_id, str):
            data_reference_map[quantity_id] = ref_id
    return data_reference_map


def _closure_maps(closures: dict) -> tuple[dict[str, str], dict[str, set[str]]]:
    """Build (closure_output_map, closure_input_map): data id to the closures producing/consuming
    it.
    """
    closure_output_map: dict[str, str] = {}
    closure_input_map: dict[str, set[str]] = {}
    for cid, c in closures.items():
        if not isinstance(c, dict):
            continue
        inputs = {v for k, v in c.items() if k not in {"id", "type"} and isinstance(v, str)}
        out_field = _CLOSURE_OUTPUT_FIELDS.get(c.get("type", ""))
        if out_field:
            out_val = c.get(out_field)
            if isinstance(out_val, str):
                closure_output_map[out_val] = cid
                closure_input_map[out_val] = {v for v in inputs if v != out_val}
    return closure_output_map, closure_input_map


def _apply_solver_control_modes(slv_arm, motions) -> None:
    """Propagate each motion's control mode onto its arm solvers."""
    for solver in slv_arm:
        control_modes = {
            arm_solver.control_mode
            for motion in motions
            for arm_solver in motion.arm_solvers
            if arm_solver.id == solver.id and arm_solver.control_mode
        }
        if len(control_modes) == 1:
            solver.control_mode = next(iter(control_modes))
        elif len(control_modes) > 1:
            raise ValueError(
                f"Solver '{solver.id}' is used with multiple control modes: "
                f"{', '.join(sorted(control_modes))}."
            )
        else:
            raise ValueError(f"Solver '{solver.id}' is not associated with a control mode.")


def _backend_from_graph(g) -> str:
    """Runtime backend ('mj_kdl' or 'robif2b') selected by the graph's runtime type."""
    runtime_to_backend = {str(RT.MuJoCoRuntime): "mj_kdl", str(RT.RealRobotRuntime): "robif2b"}
    for runtime_iri in g.objects(predicate=RT["uses-runtime"]):
        mapped = runtime_to_backend.get(str(runtime_iri))
        if mapped:
            return mapped
    return "robif2b"


def _single_solver_value(slv_arm, attr, label, default):
    """The single value of an attribute across arm solvers (one control loop); raises if they
    disagree.
    """
    values = {v for s in slv_arm if (v := getattr(s, attr)) is not None}
    if len(values) > 1:
        raise ValueError(
            f"Multiple {label} values found across arm solvers, but generated code has one "
            "control loop."
        )
    return next(iter(values), default)


def _apply_monitor_debounce(handlers, control_period_ns: int) -> None:
    """Convert each monitor's debounce duration to a step count from the control period."""
    for handler in handlers:
        for monitor in handler.monitors:
            if getattr(monitor, "debounce_duration_s", None) is not None:
                monitor.debounce_steps = round(
                    monitor.debounce_duration_s / (control_period_ns * 1e-9)
                )


def _shared_runtime_members(slv_arm, motions) -> list[dict]:
    """Extra shared_data members for runtime state (FT bias/settle counters, snapshot-captured
    flags).
    """
    members = []
    seen_ft_ids = set()
    for s in slv_arm:
        for out in s.output:
            if getattr(out, "type", None) == "Wrench" and getattr(out, "sensor_name", ""):
                if out.id in seen_ft_ids:
                    continue
                seen_ft_ids.add(out.id)
                members.append({"id": f"{out.id}_ft_bias", "type": "FreeVector"})
                members.append({"id": f"{out.id}_ft_settle", "type": "IntCounter"})

    seen_captured = set()
    for motion in motions:
        for snap in getattr(motion, "snapshots", []):
            if getattr(snap, "persistent", False) and snap.target_id not in seen_captured:
                seen_captured.add(snap.target_id)
                members.append({"id": f"{snap.target_id}_captured", "type": "Bool"})
    return members


# ---------------------------------------------------------------------------
# Codegen-facing helpers. Each is invoked while constructing the piece it belongs to
# (scene, solvers, closures, motions, introspection) so generate_ir builds a complete IR
# in one forward pass — the assembled ir dict is final and is never re-processed.
# ---------------------------------------------------------------------------


SUPPORTED_ROBOT_MODELS = {"KinovaGen3"}


def _validate_solvers(arm_solvers, backend: str) -> None:
    """Reject unsupported robot models, and scene-object pose sync on the robif2b backend."""
    unsupported = {
        _field(s, "robot_model")
        for s in arm_solvers
        if _field(s, "robot_model") and _field(s, "robot_model") not in SUPPORTED_ROBOT_MODELS
    }
    if unsupported:
        raise RuntimeError(
            f"Unsupported robot model(s): {', '.join(sorted(unsupported))}. "
            f"Supported: {', '.join(sorted(SUPPORTED_ROBOT_MODELS))}"
        )

    if backend != "robif2b":
        return

    for solver in arm_solvers:
        for out in _field(solver, "output", []):
            if _field(out, "type") != "Pose":
                continue
            entity = _field(out, "of") or {}
            if _field(entity, "is_scene_object"):
                obj_id = _field(entity, "id") or _field(entity, "body") or _field(out, "id")
                raise RuntimeError(
                    "robif2b backend cannot sync scene-object pose output "
                    f"'{_field(out, 'id')}' for '{obj_id}'; world/scene object pose sync "
                    "is only implemented for mj_kdl."
                )


def _runtime_signature(solver, backend: str) -> tuple:
    """Identity tuple of a solver's runtime (backend, chain, tool) for deduplicating runtimes."""
    return (
        backend,
        _field(solver, "robot_model", ""),
        _field(solver, "urdf", ""),
        _field(solver, "chain_root", ""),
        _field(solver, "chain_tip") or _field(solver, "chain_end", ""),
        _field(solver, "tool_body", ""),
        _field(solver, "tcp_site", ""),
    )


def _annotate_runtime_robots(arm_solvers, motions, backend: str) -> None:
    """Assign runtime_id/runtime_owner across solvers sharing a runtime and normalize empty tool
    fields.
    """
    runtime_by_signature: dict[tuple, str] = {}
    owner_by_runtime: dict[str, str] = {}
    solvers_by_id = {_field(solver, "id"): solver for solver in arm_solvers}

    for solver in arm_solvers:
        solver_id = _field(solver, "id", "")
        signature = _runtime_signature(solver, backend)
        runtime_id = runtime_by_signature.setdefault(signature, solver_id)
        owner_by_runtime.setdefault(runtime_id, solver_id)
        _set_field(solver, "runtime_id", runtime_id)
        _set_field(solver, "runtime_owner", solver_id == owner_by_runtime[runtime_id])
        # ST4's <if(x)> treats "" as truthy. Convert empty strings to None so
        # the template's <if(solver.tool_body)> branch is correctly skipped
        # for bare robots (no gripper / tool attached).
        if not _field(solver, "tool_body"):
            _set_field(solver, "tool_body", None)
        if not _field(solver, "tcp_site"):
            _set_field(solver, "tcp_site", None)

    for motion in motions:
        for solver in _field(motion, "arm_solvers", []):
            canonical = solvers_by_id.get(_field(solver, "id"))
            if canonical is None:
                continue
            _set_field(
                solver, "runtime_id", _field(canonical, "runtime_id") or _field(solver, "id", "")
            )
            _set_field(solver, "runtime_owner", _field(canonical, "runtime_owner", True))


def _add_group_type_flags(groups: list) -> list:
    """Set is_pose/is_twist/is_wrench on pose-axis error groups from their superobject type."""
    for g in groups:
        so_type = _field(g, "superobject_type", "Pose")
        _set_field(g, "is_pose", so_type == "Pose")
        _set_field(g, "is_twist", so_type in ("VelocityTwist", "AccelerationTwist"))
        _set_field(g, "is_wrench", so_type == "Wrench")
    return groups


# ---------------------------------------------------------------------------
# Codegen-facing helpers
# ---------------------------------------------------------------------------
def _field(obj, key, default=None):
    """Read a field from either a dict or a dataclass instance, so derivations can run
    on the native IR (dataclasses) without a dict round-trip. str-Enum values are
    normalized to their string value so native access matches the serialized dict."""
    if obj is None:
        return default
    val = obj.get(key, default) if isinstance(obj, dict) else getattr(obj, key, default)
    return val.value if isinstance(val, Enum) else val


def _set_field(obj, key, value) -> None:
    """Set a field on either a dict or a dataclass instance (the field must exist on the
    dataclass for it to serialize)."""
    if isinstance(obj, dict):
        obj[key] = value
    else:
        setattr(obj, key, value)


def _as_dict(obj) -> dict:
    """Plain-dict view of a dict or dataclass (for building rows from all fields)."""
    return obj if isinstance(obj, dict) else asdict(obj)


def _motion_done_terms(motion) -> list:
    """Structured UNTIL-member terms that end a motion (edge monitors → event flag, level
    monitors → their boolean flag). Joined by until_any and rendered by bool-condition."""
    mid = _field(motion, "id")
    terms = []
    for monitor in _field(motion, "until_monitors", []):
        if _field(monitor, "is_edge_triggered"):
            terms.append({"kind": "event", "motion_id": mid, "monitor_id": _field(monitor, "id")})
        else:
            terms.append({"kind": "flag", "motion_id": mid, "flag": _field(monitor, "flag")})
    return terms


def expand_vector_fields(item, field: str) -> None:
    """Expand a 3-vector field into <field>_x/_y/_z components (an omitted value means zero)."""
    values = _field(item, field)
    if values is None:
        # Env placement shorthand: omitted position/orientation means zero/identity.
        values = [0.0, 0.0, 0.0]
    if not isinstance(values, list) or len(values) != 3:
        item_id = _field(item, "id", "<unknown>")
        raise ValueError(f"Scene item '{item_id}' has invalid '{field}'; expected three values.")
    _set_field(item, f"{field}_x", values[0])
    _set_field(item, f"{field}_y", values[1])
    _set_field(item, f"{field}_z", values[2])


def require_field(obj_id: str, field: str, value):
    """Return value, raising if a required procedural scene-object field is missing."""
    if value is None:
        raise ValueError(
            f"Procedural scene object '{obj_id}' is missing required field "
            f"'{field}'. Add it to the .robmot model — silent defaults are no "
            f"longer applied."
        )
    return value


def _pose_component(component_id: str, data_by_id: dict) -> dict:
    """Structured pose component: either a literal ``value`` or a ``ref`` id that the
    backend template renders via access-expr. Backend-agnostic — no C++/KDL here."""
    component = data_by_id.get(component_id)
    reference_value = _field(component, "reference_value")
    if reference_value:
        return {"value": None, "ref": reference_value}
    value = _field(component, "value")
    if value is not None:
        return {"value": str(value), "ref": None}
    return {"value": None, "ref": component_id}


def _signal_id(value):
    """Id string of a signal value (a str or an object with an id)."""
    if isinstance(value, str):
        return value
    return _field(value, "id")


def _annotate_controller_signals(controllers, closures: dict) -> None:
    """Fold the measured/setpoint signal ids onto each controller from its error-evaluator
    closure. Emits abstract ids only; the C++ access expression is rendered backend-side by
    access-expr (shared_data.stg). Runs during construction of the controllers' piece."""
    error_sources = {
        closure.get("error"): closure
        for closure in closures.values()
        if isinstance(closure, dict)
        and closure.get("type") == "ErrorEvaluator"
        and closure.get("error")
    }
    for controller in controllers:
        error_id = _signal_id(_field(controller, "error_signal"))
        source = error_sources.get(error_id) or {}
        measured_id = source.get("quantity")
        setpoint_id = _signal_id(_field(controller, "reference_signal")) or source.get(
            "reference_value"
        )
        if measured_id:
            _set_field(controller, "measured_signal", measured_id)
        if setpoint_id:
            _set_field(controller, "setpoint_signal", setpoint_id)


def add_controller_internal_state_logging(
    closures: dict, shared_data: list, introspection: dict, motions
) -> None:
    """Log stateful controllers' internal state (error integral, previous error, first-sample flag)
    as shared_data items and introspection quantities.
    """
    quantities = introspection.setdefault("quantities", [])
    shared_ids = {_field(item, "id") for item in shared_data if _field(item, "id")}
    quantity_ids = {_field(item, "id") for item in quantities if _field(item, "id")}

    def add_shared(item_id: str, item_type: str, controller_id: str, state_name: str) -> None:
        if item_id not in shared_ids:
            shared_data.append(
                {
                    "id": item_id,
                    "type": item_type,
                    "controller": controller_id,
                    "role": "controller_internal_state",
                    "state": state_name,
                }
            )
            shared_ids.add(item_id)

    def add_quantity(item_id: str, controller_id: str, state_name: str) -> None:
        if item_id not in quantity_ids:
            quantities.append(
                {
                    "id": item_id,
                    "type": "Quantity",
                    "controller": controller_id,
                    "role": "controller_internal_state",
                    "state": state_name,
                }
            )
            quantity_ids.add(item_id)

    stateful_types = {"ProportionalIntegralDerivative", "ImpedanceController"}
    stateful_ids = {
        _field(controller, "id")
        for source in (motions, [{"controllers": introspection.get("controllers", [])}])
        for entry in source
        for controller in (_field(entry, "controllers", []) or [])
        if _field(controller, "type") in stateful_types and _field(controller, "id")
    }

    for closure in closures.values():
        if not isinstance(closure, dict) or closure.get("type") != "Controller":
            continue
        controller_id = closure.get("id")
        if controller_id not in stateful_ids:
            continue
        samples = [
            ("error_integral", "Quantity", "error_integral"),
            ("previous_error", "Quantity", "previous_error"),
            ("first_sample", "Bool", "is_first_sample"),
        ]
        closure_samples = []
        for state_name, item_type, getter in samples:
            item_id = f"{controller_id}_{state_name}"
            add_shared(item_id, item_type, controller_id, state_name)
            if item_type == "Quantity":
                add_quantity(item_id, controller_id, state_name)
            closure_samples.append({"id": item_id, "getter": getter})
        closure["internal_state_samples"] = closure_samples


def add_quantity_samples(introspection: dict, shared_data: list, views: dict) -> None:
    """Build the per-quantity frame-log sample descriptors from the introspection quantities and
    shared data.
    """
    shared_ids = {_field(item, "id") for item in shared_data if _field(item, "id")}
    samples = []

    # Each sample carries a backend-agnostic descriptor (kind + ids/axis); the C++
    # sample expression is rendered by the sample-expr template (shared_data.stg).
    def add(source, component: str, desc: dict) -> None:
        src = _as_dict(source)
        row = {key: value for key, value in src.items() if key != "index"}
        source_id = src.get("id")
        row.update(
            {
                "id": source_id if not component else f"{source_id}.{component}",
                "source_id": source_id,
                "component": component or None,
                "type": "Scalar",
                "source_type": src.get("type"),
                "sample_desc": desc,
            }
        )
        samples.append(row)

    def add_axes(source, prefix: str, make_desc) -> None:
        for idx, axis in enumerate(("x", "y", "z")):
            add(source, f"{prefix}.{axis}" if prefix else axis, make_desc(idx))

    def scalar_view(data_id: str) -> bool:
        view = views.get(data_id)
        return not view or _field(view, "axis") is not None

    for quantity in introspection.get("quantities", []):
        qid = quantity.get("id")
        if not qid:
            continue
        qtype = quantity.get("type")
        if qtype == "Quantity":
            if quantity.get("value") is not None and qid not in shared_ids and qid not in views:
                add(quantity, "", {"kind": "literal", "value": str(quantity["value"])})
            elif qid in views and scalar_view(qid):
                # A scalar view resolves to a composite-member access only for these
                # superobject types; other superobjects (e.g. PoseDifference) sample the
                # quantity's own shared field instead.
                so_type = _field(_field(views.get(qid), "superobject"), "type")
                if so_type in {"Pose", "Wrench", "VelocityTwist", "AccelerationTwist"}:
                    add(quantity, "", {"kind": "access", "ref": qid})
                else:
                    add(quantity, "", {"kind": "shared", "id": qid})
            elif qid in shared_ids and qid not in views:
                add(quantity, "", {"kind": "shared", "id": qid})
        elif qtype in {"Position", "Direction", "FreeVector"} and qid in shared_ids:
            add_axes(quantity, "", lambda i, q=qid: {"kind": "vec", "id": q, "axis": i})
        elif qtype == "Orientation" and qid in shared_ids:
            add_axes(quantity, "", lambda i, q=qid: {"kind": "orientation", "id": q, "axis": i})
        elif qtype in {"Pose", "Trajectory"} and qid in shared_ids:
            add_axes(
                quantity, "position", lambda i, q=qid: {"kind": "pose_pos", "id": q, "axis": i}
            )
            add_axes(
                quantity,
                "orientation",
                lambda i, q=qid: {"kind": "pose_orient", "id": q, "axis": i},
            )
        elif (
            qtype in {"VelocityTwist", "AccelerationTwist", "PoseDifference"} and qid in shared_ids
        ):
            add_axes(
                quantity,
                "angular",
                lambda i, q=qid: {"kind": "member", "id": q, "member": "rot", "axis": i},
            )
            add_axes(
                quantity,
                "linear",
                lambda i, q=qid: {"kind": "member", "id": q, "member": "vel", "axis": i},
            )
        elif qtype == "Wrench" and qid in shared_ids:
            add_axes(
                quantity,
                "torque",
                lambda i, q=qid: {"kind": "member", "id": q, "member": "torque", "axis": i},
            )
            add_axes(
                quantity,
                "force",
                lambda i, q=qid: {"kind": "member", "id": q, "member": "force", "axis": i},
            )

    sampled_ids = {sample.get("source_id") for sample in samples}
    for item in shared_data:
        item_id = _field(item, "id")
        if not item_id or item_id in sampled_ids:
            continue
        if _field(item, "type") == "Bool":
            add(item, "", {"kind": "bool", "id": item_id})
        elif _field(item, "type") == "IntCounter":
            add(item, "", {"kind": "int", "id": item_id})

    introspection["quantity_samples"] = samples


def add_spatial_samples(introspection: dict, shared_data: list) -> None:
    """Add per-object pose, velocity-twist and wrench frame-log samples."""
    shared_ids = {_field(item, "id") for item in shared_data if _field(item, "id")}
    kinds = {"Pose": "poses", "VelocityTwist": "twists", "Wrench": "wrenches"}
    spatial = {"poses": [], "twists": [], "wrenches": []}
    for item in shared_data:
        iid = _field(item, "id")
        pool = kinds.get(_field(item, "type"))
        if not iid or iid not in shared_ids or pool is None:
            continue
        spatial[pool].append({"id": iid, "index": len(spatial[pool])})
    introspection["spatial_samples"] = spatial


def _index_by_id(items: list) -> dict:
    """Index IR items (dicts or dataclasses) by their id (skips id-less entries)."""
    out = {}
    for item in items:
        iid = _field(item, "id")
        if iid:
            out[iid] = item
    return out


def build_pose_components(views: dict, data: list) -> dict:
    """Resolve declared/inline poses into per-axis structured components (a literal value or a
    reference id).
    """
    data_by_id = _index_by_id(data)
    components: dict[str, dict] = {}
    for view in views.values():
        superobject = _field(view, "superobject")
        so_type = _field(superobject, "type")
        so_prov = _field(superobject, "provenance") or {}
        is_declared_pose = bool(_field(so_prov, "authored") or _field(so_prov, "snapshot"))
        if so_type != "Pose":
            continue
        if not (is_declared_pose or _field(superobject, "euler_axes_sequence")):
            continue
        # Only include inline-defined poses (those where components have values/references).
        subobject_id = _field(_field(view, "subobject"), "id")
        subobject_data = data_by_id.get(subobject_id)
        if (
            not _field(subobject_data, "reference_value")
            and _field(subobject_data, "value") is None
        ):
            continue
        pose_id = _field(superobject, "id")
        entry = components.setdefault(
            pose_id,
            {
                "position_x": None,
                "position_y": None,
                "position_z": None,
                "orientation_x": None,
                "orientation_y": None,
                "orientation_z": None,
            },
        )
        axis = str(_field(view, "axis") or "").lower()
        if axis not in {"x", "y", "z"}:
            continue
        subobject = _field(_field(view, "subobject"), "id")
        if not subobject:
            continue
        prefix = "position" if _field(view, "subspace") == "Linear" else "orientation"
        entry[f"{prefix}_{axis}"] = _pose_component(subobject, data_by_id)
    for pose_id, parts in components.items():
        missing = [name for name, value in parts.items() if value is None]
        if missing:
            raise ValueError(
                f"Declared pose '{pose_id}' is missing required components: {', '.join(missing)}."
            )
    return components


def resolve_lerp_closures(closures: dict, pose_components: dict) -> None:
    """Fold each Lerp closure's goal into structured pose components or a shared-signal ref."""
    for closure in closures.values():
        if closure.get("type") != "Lerp":
            continue
        goal = closure.get("goal")
        if not isinstance(goal, str):
            continue
        if goal in pose_components:
            # Emit the structured pose components; the template builds the pose frame.
            closure["goal_components"] = pose_components[goal]
            closure["assign_goal"] = True
        else:
            # goal is a shared signal id (already on the closure as closure["goal"]).
            closure["assign_goal"] = False


def resolve_arc_closures(closures: dict, data: list) -> None:
    """Validate that each Arc closure's end is a Pose (the template renders its
    position/orientation).
    """
    data_by_id = _index_by_id(data)

    def is_pose(data) -> bool:
        qkind = _field(data, "quantity_kind")
        qkind_ids = qkind if isinstance(qkind, list) else [qkind]
        return _field(data, "type") == "Pose" or any(
            _field(item, "id") == "Pose" for item in qkind_ids
        )

    for closure in closures.values():
        if closure.get("type") != "Arc":
            continue
        end = closure.get("end")
        end_data = data_by_id.get(end)
        if not isinstance(end, str) or not is_pose(end_data):
            raise ValueError("Arc trajectory end must be a Pose quantity.")
        # end is a validated Pose shared signal; the template renders shared.<end>.p/.M.


def declared_pose_component_entries(
    data: list, pose_components: dict, referenced_ids: set[str] | None = None
) -> list[dict]:
    """Authored declared-pose component entries, optionally restricted to the referenced ids."""
    data_by_id = _index_by_id(data)
    entries = []
    for pose_id, parts in pose_components.items():
        if referenced_ids is not None and pose_id not in referenced_ids:
            continue
        item = data_by_id.get(pose_id)
        item_prov = _field(item, "provenance") or {}
        if not _field(item_prov, "authored") or _field(item_prov, "snapshot"):
            continue
        entries.append({"id": pose_id, **parts})
    return entries


def collect_motion_references(motion, closures: dict) -> set[str]:
    """Every id a motion references, including through its scheduled closures."""
    refs: set[str] = set()

    def visit(value):
        if isinstance(value, str):
            refs.add(value)
        elif isinstance(value, dict):
            for item in value.values():
                visit(item)
        elif isinstance(value, list):
            for item in value:
                visit(item)

    visit(_as_dict(motion))
    for schedule_name in ("when_schedule", "while_schedule", "until_schedule"):
        for step in _field(motion, schedule_name, []):
            closure = closures.get(step)
            if closure:
                visit(closure)
    return refs


def _set_motion_trajectory_progress(motion, closures: dict, data_by_id: dict) -> None:
    """Fold the time-driven trajectory alpha ids (Progress-kind, non-Arc while-schedule
    closures) onto a motion (dict or dataclass)."""
    ids: list[str] = []
    for step in _field(motion, "while_schedule", []):
        closure = closures.get(step)
        if not closure or _field(closure, "type") not in {
            "Lerp",
            "Circle",
            "Arc",
            "Helix",
            "Figure8",
        }:
            continue
        alpha_id = _field(closure, "alpha")
        alpha_data = data_by_id.get(alpha_id)
        qkind = _field(_field(alpha_data, "quantity_kind"), "id")
        if qkind == "Progress" and _field(closure, "type") != "Arc" and alpha_id not in ids:
            ids.append(alpha_id)
    _set_field(motion, "time_trajectory_progress_ids", ids)


def _evaluator_term(e, start_field: str) -> dict:
    """Structured boolean term for an evaluator: an elapsed timing predicate (world clock
    vs threshold from the selected state timestamp) or a solver constraint-satisfied check.
    Rendered to C++ by the bool-condition template."""
    if _field(e, "is_elapsed"):
        op = _field(e, "elapsed_op") or ">="
        thr = _field(e, "elapsed_threshold_s") or 0.0
        # Pre-format the threshold (fixed 6-decimal) so the emitted literal is stable.
        return {"kind": "elapsed", "start_field": start_field, "op": op, "threshold": f"{thr:.6f}"}
    return {"kind": "constraint", "error_id": _field(_field(e, "error"), "id")}


def _set_monitor_conditions(
    motion, evaluators_key: str, monitors_key: str, start_field: str, any_key: str
) -> None:
    """Stamp the structured active-phase terms onto the aggregate monitor + any
    elapsed-error monitors. Rendered to C++ by the bool-condition template."""
    evaluators = _field(motion, evaluators_key, [])
    terms = [
        _evaluator_term(e, start_field)
        for e in evaluators
        if _field(e, "error") or _field(e, "is_elapsed")
    ]
    any_flag = bool(_field(motion, any_key))
    elapsed_terms_by_error = {
        _field(_field(e, "error"), "id"): _evaluator_term(e, start_field)
        for e in evaluators
        if _field(e, "is_elapsed") and _field(e, "error")
    }
    aggregate_key = "is_until_aggregate" if any_key == "until_any" else "is_when_aggregate"
    for monitor in _field(motion, monitors_key, []):
        if _field(monitor, aggregate_key):
            _set_field(monitor, "active_terms", terms)
            _set_field(monitor, "active_terms_present", bool(terms))
            _set_field(monitor, "active_any", any_flag)
            _set_field(monitor, "has_active", True)
            continue
        error_id = _field(_field(monitor, "error"), "id")
        if error_id in elapsed_terms_by_error:
            _set_field(monitor, "active_terms", [elapsed_terms_by_error[error_id]])
            _set_field(monitor, "active_terms_present", True)
            _set_field(monitor, "active_any", False)
            _set_field(monitor, "has_active", True)


def _set_motion_conditions(motion) -> None:
    """Fold the UNTIL/WHEN/done structured boolean terms onto a motion (rendered to C++ by
    the bool-condition template). WHEN joins with when_any, done with until_any."""
    _set_monitor_conditions(
        motion, "until_evaluators", "until_monitors", "motion_start_time", "until_any"
    )
    when_terms = [
        _evaluator_term(e, "when_start_time")
        for e in _field(motion, "when_evaluators", [])
        if _field(e, "error") or _field(e, "is_elapsed")
    ]
    _set_field(motion, "when_terms", when_terms)
    _set_field(motion, "when_terms_present", bool(when_terms))
    _set_monitor_conditions(
        motion, "when_evaluators", "when_monitors", "when_start_time", "when_any"
    )
    done_terms = _motion_done_terms(motion)
    _set_field(motion, "done_terms", done_terms)
    _set_field(motion, "done_terms_present", bool(done_terms))


def add_motion_function_interfaces(motions: list) -> None:
    """Fold per-motion capability booleans (which context objects — state, shared, robot —
    each generated function needs). The C++ signatures and call args are built from these
    by the sig-params / sig-args templates; ir_gen carries no C++ type names."""
    for motion in motions:
        has_when_elapsed = any(
            _field(e, "is_elapsed") for e in _field(motion, "when_evaluators", [])
        )
        has_when_logic = bool(_field(motion, "when_schedule") or _field(motion, "when_evaluators"))
        when_mons = _field(motion, "when_monitors") or []
        until_mons = _field(motion, "until_monitors") or []
        has_pose = bool(_field(motion, "declared_pose_components"))
        when_sched = bool(_field(motion, "when_schedule"))
        until_sched = bool(_field(motion, "until_schedule"))
        when_fsm = any(_field(m, "fsm_namespace") for m in when_mons)
        until_fsm = any(_field(m, "fsm_namespace") for m in until_mons)
        has_forwarded_commands = bool(_field(motion, "forwarded_commands"))
        has_arm = bool(_field(motion, "arm_solvers"))

        _set_field(motion, "can_start_needs_state", has_when_elapsed)
        _set_field(motion, "can_start_needs_shared", has_when_logic)
        _set_field(motion, "can_start_needs_robot", False)

        _set_field(motion, "when_needs_state", has_when_elapsed or bool(when_mons))
        _set_field(
            motion,
            "when_needs_shared",
            has_when_elapsed or has_pose or when_sched or bool(when_mons),
        )
        _set_field(motion, "when_needs_robot", when_fsm)

        _set_field(motion, "until_needs_state", bool(until_mons))
        _set_field(motion, "until_needs_shared", until_sched or bool(until_mons))
        _set_field(motion, "until_needs_robot", until_fsm)

        _set_field(motion, "monitor_needs_state", bool(when_mons) or bool(until_mons))
        _set_field(
            motion,
            "monitor_needs_shared",
            when_sched or bool(when_mons) or until_sched or bool(until_mons),
        )
        _set_field(motion, "monitor_needs_robot", when_fsm or until_fsm)

        _set_field(motion, "apply_needs_state", has_arm)
        _set_field(motion, "apply_needs_shared", has_forwarded_commands)
        _set_field(motion, "apply_needs_robot", has_arm or has_forwarded_commands)


_FSM_NS = "https://secorolab.github.io/metamodels/behaviour/fsm#"


# ---------------------------------------------------------------------------
# FSM wiring
# ---------------------------------------------------------------------------
def _fsm_from_graph(g) -> dict | None:
    """Frame the FSM named graph (states/events/transitions/reactions, folded into the
    model dataset by motion-spec-dsl) into the same dict shape the standalone .hpp uses,
    so codegen needs no fsm_ir.json read. None when the model imports no .fsm."""
    FSM = rdflib.Namespace(_FSM_NS)
    fsm_ref = next(iter(g.subjects(RDF["type"], FSM["FSM"])), None)
    if fsm_ref is None:
        return None

    def ident(uri):
        return get_valid_var_name(local_name(str(uri))).upper()

    states, state_uris = [], {}
    for s in g.objects(fsm_ref, FSM["states"]):
        key = ident(s)
        states.append(key)
        state_uris[key] = str(s)
    events, event_uris = [], {}
    for e in g.objects(fsm_ref, FSM["events"]):
        key = ident(e)
        events.append(key)
        event_uris[key] = str(e)

    transitions_table = []
    for tr in g.objects(fsm_ref, FSM["transitions"]):
        transitions_table.append(
            {
                "id": ident(tr),
                "uri": str(tr),
                "from_state": ident(g.value(tr, FSM["transition-from"])),
                "to_state": ident(g.value(tr, FSM["transition-to"])),
            }
        )
    reactions_table = []
    for rx in g.objects(fsm_ref, FSM["reactions"]):
        fires = [ident(ev) for ev in g.objects(rx, FSM["fires-events"])]
        reactions_table.append(
            {
                "id": ident(rx),
                "uri": str(rx),
                "when_event": ident(g.value(rx, FSM["when-event"])),
                "do_transition": ident(g.value(rx, FSM["do-transition"])),
                "fires_events": fires,
                "num_fires": len(fires),
            }
        )

    description_node = g.value(fsm_ref, FSM["description"])
    # Event/state IRIs share the FSM node's parent path (…/<model>/fsm/); is_fsm_event
    # matches monitor event IRIs against it.
    namespace_uri = str(fsm_ref).rsplit("/", 1)[0] + "/"
    return {
        "name": str(g.value(fsm_ref, FSM["name"])),
        "description": str(description_node) if description_node is not None else None,
        "start_state": ident(g.value(fsm_ref, FSM["start-state"])),
        "end_state": ident(g.value(fsm_ref, FSM["end-state"])),
        "states": states,
        "state_uris": state_uris,
        "events": events,
        "event_uris": event_uris,
        "transitions_table": transitions_table,
        "reactions_table": reactions_table,
        "namespace_uri": namespace_uri,
    }


def _event_to_state(fsm: dict) -> dict[str, str]:
    """Map each FSM event token to the state it transitions out of (the state the motion
    runs in): the from-state of the transition the event's reaction fires."""
    transition_from = {t["id"]: t["from_state"] for t in fsm["transitions_table"]}
    return {
        r["when_event"]: transition_from[r["do_transition"]]
        for r in fsm["reactions_table"]
        if r["do_transition"] in transition_from
    }


def is_fsm_event(monitor, fsm_ns_uri: str | None) -> bool:
    # A monitor fires the FSM only when its event lives in the FSM's namespace;
    # standalone (monitor-owned) events keep the existing warn stub.
    """True when a monitor fires an FSM event: edge-triggered with an event in the FSM namespace."""
    return bool(
        fsm_ns_uri
        and _field(monitor, "is_edge_triggered")
        and (_field(monitor, "event_uri") or "").startswith(fsm_ns_uri)
    )


def _apply_fsm_wiring(motions, fsm) -> dict:
    """Tag FSM-event monitors + their motions from the framed FSM, and return the FSM
    header/step meta fields. Runs before the function-interface pass so the FSM-added robot
    param is picked up. No tagging when the model has no FSM. Runs during motion construction."""
    fsm_namespace = fsm["name"].lower() if fsm else None
    events = fsm.get("events", []) if fsm else []
    fsm_event_index = {event: idx for idx, event in enumerate(events)}
    fsm_step_event = "E_STEP" if "E_STEP" in events else None
    meta = {
        "fsm_namespace": fsm_namespace,
        "fsm_header": f"{fsm['name']}.hpp" if fsm else None,
        "fsm_step_event": fsm_step_event,
        "fsm_step_event_idx": fsm_event_index.get(fsm_step_event, -1),
    }
    if fsm_namespace is None:
        return meta

    fsm_ns_uri = fsm.get("namespace_uri")
    event_state = _event_to_state(fsm)
    by_id = {_field(m, "id"): m for m in motions}

    def tag_run_state(motion, monitors):
        for monitor in monitors:
            if is_fsm_event(monitor, fsm_ns_uri):
                _set_field(monitor, "fsm_namespace", fsm_namespace)
                _set_field(
                    monitor,
                    "fsm_event_idx",
                    fsm_event_index.get(_field(monitor, "event_name") or "", -1),
                )
                state = event_state.get(_field(monitor, "event_name") or "")
                if state and not _field(motion, "fsm_state"):
                    _set_field(motion, "fsm_state", state)

    for motion in motions:
        tag_run_state(
            motion, _field(motion, "until_monitors", []) + _field(motion, "while_monitors", [])
        )
        for monitor in _field(motion, "when_monitors", []):
            if not is_fsm_event(monitor, fsm_ns_uri):
                continue
            _set_field(monitor, "fsm_namespace", fsm_namespace)
            _set_field(
                monitor,
                "fsm_event_idx",
                fsm_event_index.get(_field(monitor, "event_name") or "", -1),
            )
            fallback_id = _field(monitor, "fallback_motion")
            if not fallback_id:
                raise ValueError(
                    f"WHEN monitor '{_field(monitor, 'id')}' on FSM-wired motion "
                    f"'{_field(motion, 'id')}' must declare a fallback hold motion "
                    f"(e.g. '... when active fallback <hold-motion>'). A WHEN precondition "
                    f"without a fallback would leave the arm uncommanded while waiting."
                )
            fallback = by_id.get(fallback_id)
            if fallback is None:
                raise ValueError(
                    f"WHEN monitor '{_field(monitor, 'id')}' names unknown fallback motion "
                    f"'{fallback_id}'."
                )
            state = event_state.get(_field(monitor, "event_name") or "")
            if state and not _field(fallback, "fsm_state"):
                _set_field(fallback, "fsm_state", state)
            gates = _field(fallback, "fsm_when_gate_motions")
            if gates is None:
                gates = []
                _set_field(fallback, "fsm_when_gate_motions", gates)
            if _field(motion, "id") not in gates:
                gates.append(_field(motion, "id"))
    return meta


def _apply_fsm_gate_calls(motions, fsm_namespace) -> None:
    """Fold each fallback state's WHEN-evaluation gate calls: the gated motion id plus its
    when-signature capability booleans. The C++ ``monitor_when_<id>(<args>)`` call is
    rendered by the template via sig-args. Runs after function interfaces so when_needs_*
    are available."""
    if fsm_namespace is None:
        return
    by_id = {_field(m, "id"): m for m in motions}
    for fallback in motions:
        gate_ids = _field(fallback, "fsm_when_gate_motions")
        if not gate_ids:
            continue
        _set_field(
            fallback,
            "fsm_when_gate_calls",
            [
                {
                    "gid": gate_id,
                    "needs_state": _field(by_id[gate_id], "when_needs_state", False),
                    "needs_shared": _field(by_id[gate_id], "when_needs_shared", False),
                    "needs_robot": _field(by_id[gate_id], "when_needs_robot", False),
                }
                for gate_id in gate_ids
                if gate_id in by_id
            ],
        )


# ---------------------------------------------------------------------------
# Top-level orchestration
# ---------------------------------------------------------------------------
def generate_ir(manifest_path):
    """Build the complete IR for a model manifest in one forward pass and return it as a dict."""
    app_model_path, g, imported_models, imported_provenance = _load_graph(manifest_path)

    p = Parser(g)
    node_by_id, id_nodes = _node_indexes(g, p)
    setups_by_node, ordered_setups = _robot_setups_from_graph(g)
    default_setup = ordered_setups[0] if ordered_setups else ("", "", "", "", "", "", "", [])
    # Derive backend + FSM up front: both are pure functions of the graph and are inputs to
    # downstream construction (solver validation, runtime-robot annotation, motion FSM wiring).
    backend = _backend_from_graph(g)
    fsm = _fsm_from_graph(g)
    scene = _scene_from_graph(g)
    _validate_scene(scene)

    (slv_base_vel, sched1, hdl, sched2, slv_arm, sched3, slv_base_frc, sched4) = _solver_sections(
        g, p, setups_by_node, default_setup
    )
    _assign_monitor_event_indexes(hdl)

    closures = p.closures(ops_generic + ops_slv + ops_cstr_hdl)
    view_map = p.view()
    data_structures = p.data_structures()
    snapshot_source_map, snapshot_clock_map = _snapshot_maps(g, p)
    data_reference_map = _data_reference_map(data_structures, closures)
    closure_output_map, closure_input_map = _closure_maps(closures)

    # Resolve declared-pose components and trajectory goals from views/data/closures before
    # motions are built (per-motion declared poses reference them).
    pose_components = build_pose_components(view_map, data_structures)
    resolve_lerp_closures(closures, pose_components)
    resolve_arc_closures(closures, data_structures)

    wrench_outputs = _dedupe_by_id(
        [
            item
            for item in data_structures
            if item.type == "Wrench" and item.id not in closure_output_map
        ]
    )

    motions, fsm_meta = build_motion_units(
        g,
        p,
        hdl,
        node_by_id,
        slv_arm,
        snapshot_source_map=snapshot_source_map,
        snapshot_clock_map=snapshot_clock_map,
        view_map=view_map,
        closure_output_map=closure_output_map,
        data_reference_map=data_reference_map,
        closure_input_map=closure_input_map,
        closures=closures,
        data_structures=data_structures,
        pose_components=pose_components,
        fsm=fsm,
    )
    _apply_solver_control_modes(slv_arm, motions)
    _validate_solvers(slv_arm, backend)
    _annotate_runtime_robots(slv_arm, motions, backend)

    if scene.timestep_s <= 0:
        raise ValueError("ENVIRONMENT timestep must be positive.")
    control_period_ns = int(round(scene.timestep_s * 1e9))
    rne_damping_lambda = _single_solver_value(
        slv_arm, "regularization", "Solver regularization", 0.05
    )
    _apply_monitor_debounce(hdl, control_period_ns)

    # Safeguard: no two distinct URIs may collapse to one generated id (would silently merge).
    p.assert_no_id_collisions()

    shared_data = _filter_shared_data(
        data_structures,
        sched1 + sched2 + sched3 + sched4,
        closures,
        view_map=view_map,
        fk_output_ids={out.id for s in slv_arm for out in s.output},
    )
    shared_data = shared_data + _shared_runtime_members(slv_arm, motions)

    introspection = _build_introspection(
        app_model_path=app_model_path,
        imported_models=imported_models,
        imported_provenance=imported_provenance,
        id_nodes=id_nodes,
        node_by_id=node_by_id,
        motions=motions,
        data_structures=data_structures,
        control_period_ns=control_period_ns,
        backend=backend,
        scene=scene,
        closures=closures,
        views=view_map,
        shared_data=shared_data,
    )

    schedule = sched1 + sched2 + sched3 + sched4
    shared_schedule = sched1 + sched3 + sched4
    ir = {
        "slv_arm": slv_arm,
        "slv_base_vel": slv_base_vel,
        "slv_base_frc": slv_base_frc,
        "cstr_hdl": hdl,
        "motions": motions,
        "data": data_structures,
        "closures": closures,
        "shared_schedule": shared_schedule,
        "schedule": schedule,
        "views": view_map,
        "shared_data": shared_data,
        "pose_components": pose_components,
        "declared_pose_components": declared_pose_component_entries(
            data_structures, pose_components
        ),
        "wrench_outputs": wrench_outputs,
        "has_arm": bool(slv_arm),
        "has_mobile_base": bool(slv_base_vel or slv_base_frc),
        # Elapsed constraints compare seconds from the runtime clock (MuJoCo sim seconds /
        # real monotonic wall clock).
        "needs_clock_time": any(m.has_elapsed for m in motions),
        "control_period_ns": control_period_ns,
        "rne_damping_lambda": rne_damping_lambda,
        "arm_solvers": slv_arm,
        "base_velocity_solvers": slv_base_vel,
        "base_force_solvers": slv_base_frc,
        "backend": backend,
        "scene": scene,
        "trace": _trace_from_graph(g),
        "uris": introspection["uris"],
        "introspection": introspection,
        # FSM (states/events/transitions/reactions) framed from the FSM named graph that
        # motion-spec-dsl folds into the model dataset; None when no .fsm is imported.
        "fsm": fsm,
        **fsm_meta,
    }
    # ir is complete by construction — every codegen-facing field was computed while its
    # piece was built (scene / solvers / closures / motions / introspection). Codegen only
    # loads ir.json and renders; there is no post-assembly derivation pass.
    return ir


def main():
    """Generate intermediate representation (IR) from motion specification models."""
    parser = argparse.ArgumentParser(
        description="Generate intermediate representation (IR) JSON from motion specification models",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s manifest.json --console           # Print IR to console
  %(prog)s manifest.json -o output.json     # Save IR to file
  %(prog)s manifest.json --output -         # Print IR to console (alternative)
        """,
    )

    parser.add_argument("manifest", help="Path to the application manifest JSON file")

    output_group = parser.add_mutually_exclusive_group(required=True)
    output_group.add_argument(
        "-o", "--output", metavar="FILE", help="Output file path (use '-' for stdout)"
    )
    output_group.add_argument(
        "-c", "--console", action="store_true", help="Print output to console"
    )

    args = parser.parse_args()

    # Determine output destination
    if args.console or (args.output and args.output == "-"):
        output_file = None  # stdout
    else:
        output_file = Path(args.output)

    ir = generate_ir(args.manifest)

    # Output IR to file or stdout
    ir_json = json.dumps(ir, cls=DataclassJSONEncoder, indent=4)

    if output_file is None:
        # Output to stdout
        print(ir_json)
    else:
        # Output to file
        try:
            output_file.parent.mkdir(parents=True, exist_ok=True)
            with open(output_file, "w") as f:
                f.write(ir_json)
            print(f"IR written to {output_file}", file=sys.stderr)
        except IOError as e:
            print(f"Error writing to {output_file}: {e}", file=sys.stderr)
            sys.exit(1)


if __name__ == "__main__":
    main()
