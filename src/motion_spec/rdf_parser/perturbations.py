# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""The disturbances a simulation run applies to bodies in the scene.

A perturbation is a wrench the world pushes the robot with, not one the robot commands, so it
never reaches a solver: it is composed by the same ops a force command is built from, rotated
into the world frame, and written straight into the simulator's applied-force slots.
"""

from __future__ import annotations

from motion_spec_dsl.rdf_parser.vocab import (
    CSTR_EXT,
    CSTR_HDL,
    GEOM_OP,
    RBDYN_COORD,
    RBDYN_OP,
    SIM,
    SLV,
    TIME,
)
from rdf_utils.constraints import ConstraintViolation
from rdf_utils.models.common import get_node_types
from rdflib.namespace import PROV, RDF

from motion_spec.classes.handlers import Perturbation
from motion_spec.rdf_parser import quantities
from motion_spec.rdf_parser.model import local_name


def nodes(model, handler_node) -> list:
    """Every perturbation the handler arms, in a stable order."""
    graph = model.graph
    return sorted(
        (
            node
            for node in graph.subjects(RDF.type, SIM["Perturbation"])
            if graph.value(node, PROV.wasDerivedFrom) == handler_node
        ),
        key=str,
    )


def wrench_node(model, node):
    """The wrench coordinate the perturbation's ops compose."""
    wrench = model.graph.value(node, SLV["force"])
    if wrench is None:
        raise ConstraintViolation(
            "perturbation", f"perturbation '{model.id(node)}' composes no wrench"
        )
    return wrench


def compose_ops(model, node) -> list:
    """The op nodes writing the perturbation's wrench: one per authored component, or the sum
    that folds them. Walking back from these picks up the magnitudes and directions they read.
    """
    graph = model.graph
    wrench = wrench_node(model, node)
    return sorted(
        set(graph.subjects(RBDYN_OP["wrench"], wrench))
        | set(graph.subjects(RBDYN_OP["out"], wrench)),
        key=str,
    )


def _target_pose(model, node):
    """The pose the force direction is normalized from, or None.

    A direction computed between two frames is that pose's translation made unit, so the pose
    still carries how far away the target is -- which is what a line drawn to it needs. An
    authored direction is normalized from nothing and aims at no particular point.
    """
    graph = model.graph
    for op in graph.subjects(RBDYN_OP["wrench"], wrench_node(model, node)):
        for direction in graph.objects(op, RBDYN_OP["direction"]):
            for source in graph.subjects(GEOM_OP["direction"], direction):
                if GEOM_OP["PoseToDirection"] in get_node_types(graph, source):
                    return graph.value(source, GEOM_OP["pose"])
    return None


def gate_constraints(model, node) -> tuple[list, bool]:
    """The constraints opening the perturbation's window, and whether any of them suffices.

    An `all`/`any` gate of more than one member goes through an expression node, exactly as a
    motion's when section does; a single condition is held directly.
    """
    graph = model.graph
    held = sorted(graph.objects(node, CSTR_EXT["has-constraint"]), key=str)
    if len(held) == 1:
        inner = sorted(graph.objects(held[0], CSTR_EXT["has-constraint"]), key=str)
        if inner:
            return inner, CSTR_EXT["ConstraintDisjunction"] in get_node_types(graph, held[0])
    return held, False


def evaluator_nodes(model, node) -> list:
    """The handler evaluators computing this perturbation's gate conditions."""
    graph = model.graph
    constraints, _gate_any = gate_constraints(model, node)
    return [
        evaluator
        for constraint in constraints
        for evaluator in sorted(graph.subjects(CSTR_HDL["constraint"], constraint), key=str)
        if CSTR_HDL["ConstraintEvaluator"] in get_node_types(graph, evaluator)
    ]


def _body_name(body, chains) -> str:
    """The simulator's name for a scene body: its local name under the runtime prefix the chain
    carrying it was merged in with.
    """
    for _slice_id, solver in chains:
        if str(body) in (solver.chain.world_segments or {}):
            return f"{solver.runtime.prefix}{local_name(body)}"
    raise ConstraintViolation(
        "perturbation",
        f"perturbation body '{local_name(body)}' is not part of the scene this run plays in, "
        "so the simulator has no body to apply the wrench to",
    )


def _rotating_chain(model, wrench, chains) -> str:
    """The chain whose root frame the wrench is stated in.

    The simulator reads applied wrenches in the world frame; that chain already tracks its own
    world-to-root pose every cycle, so rotating through it needs no second kinematics read.
    """
    frame = model.graph.value(wrench, RBDYN_COORD["as-seen-by"])
    if frame is None:
        raise ConstraintViolation(
            "perturbation", f"perturbation wrench '{model.id(wrench)}' states no frame"
        )
    body = quantities.body_of(model, frame)
    for slice_id, solver in chains:
        if solver.chain.root == f"{solver.runtime.prefix}{local_name(body)}":
            return slice_id
    raise ConstraintViolation(
        "perturbation",
        f"a perturbation's direction is seen by '{local_name(body)}', which is not the root of "
        "any chain this motion runs; state it in the frame the arm is based in, so the applied "
        "wrench can be rotated into the world frame the simulator reads it in",
    )


def read(model, node, chains) -> Perturbation:
    """One perturbation record, less the gate terms its handler's evaluators supply.

    `chains` pairs the runtime member name a motion addresses a chain by with the solver record
    holding what that chain is: the slice carries neither its root frame nor its merge prefix.
    """
    graph = model.graph
    identifier = model.id(node)
    wrench = wrench_node(model, node)
    body = graph.value(node, SLV["attached-to"])
    if body is None:
        raise ConstraintViolation(
            "perturbation", f"perturbation '{identifier}' names no body to act on"
        )
    duration = graph.value(node, TIME["hasDuration"])
    constraints, gate_any = gate_constraints(model, node)
    target_pose = _target_pose(model, node)

    return Perturbation(
        id=identifier,
        body=_body_name(body, chains),
        robot_id=_rotating_chain(model, wrench, chains),
        wrench_id=model.id(wrench),
        applied_id=f"{identifier}_applied",
        active_id=f"{identifier}_active",
        duration_id=None if duration is None else model.id(duration),
        target_pose_id=None if target_pose is None else model.id(target_pose),
        has_gate=bool(constraints),
        gate_any=gate_any,
    )
