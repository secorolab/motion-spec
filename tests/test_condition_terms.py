# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""A condition that lowers to no terms renders as a constant, and the constant is the opposite of
what the model asked for: a monitor that can never fire. It may not reach the generated program."""

from __future__ import annotations

from pathlib import Path

import pytest
from motion_spec_dsl.rdf_parser.vocab import CSTR, CSTR_EXT, CSTR_HDL, QUDT_SCHEMA, TIME
from rdf_utils.constraints import ConstraintViolation
from rdf_utils.namespace import NS_MM_QUDT_QTY as QUDT_QKIND
from rdf_utils.namespace import NS_MM_QUDT_UNIT as QUDT_UNIT
from rdflib import Dataset, Literal, URIRef
from rdflib.namespace import RDF, XSD

from motion_spec.classes.constraints import Constraint
from motion_spec.classes.handlers import ConstraintEvaluator, EvaluatorType, LevelMonitor
from motion_spec.classes.motion import MotionUnit
from motion_spec.rdf_parser.coordination import constraint_evaluator, set_motion_conditions
from motion_spec.rdf_parser.model import Model

NS = "https://example.test/"


def test_a_monitor_watching_nothing_evaluable_is_rejected() -> None:
    # It would render as a constant false: the event never fires and the FSM never leaves.
    watcher = LevelMonitor(
        id="mon_held", monitor_type="LevelMonitor", error=None, flag="held", is_until_aggregate=True
    )
    unit = MotionUnit(
        id="motion_probe",
        motion_id="",
        name="probe",
        description=[],
        when_evaluators=[],
        while_evaluators=[],
        until_evaluators=[
            ConstraintEvaluator(
                id="eval_c_held",
                type_=EvaluatorType.AssignmentEvaluator,
                constraint=Constraint(id="c_held", quantity=None, parameter=None),
                error=None,
            )
        ],
        controllers=[],
        when_monitors=[],
        while_monitors=[],
        until_monitors=[watcher],
        when_schedule=[],
        while_schedule=[],
        until_schedule=[],
    )
    with pytest.raises(ConstraintViolation, match="constant false"):
        set_motion_conditions(unit)


def test_an_elapsed_equality_reads_its_tolerance_in_seconds() -> None:
    """The graph keeps the unit a duration was written in; the reader turns 10 ms into 0.01 s."""
    graph = Dataset(default_union=True)
    constraint = URIRef(f"{NS}wait-eq")
    durations = {}
    for name, value, unit in (
        ("elapsed", None, "SEC"),
        ("reference", 5.0, "SEC"),
        ("tolerance", 10.0, "MilliSEC"),
    ):
        node = URIRef(f"{NS}wait-eq-{name}")
        graph.add((node, QUDT_SCHEMA.hasQuantityKind, QUDT_QKIND["Time"]))
        graph.add((node, QUDT_SCHEMA.unit, QUDT_UNIT[unit]))
        if value is None:
            graph.add((node, RDF.type, CSTR_EXT.ElapsedDurationCoordinate))
        else:
            graph.add((node, RDF.type, TIME.Duration))
            graph.add((node, QUDT_SCHEMA.value, Literal(value, datatype=XSD.double)))
        durations[name] = node
    interval = URIRef(f"{NS}wait-eq-interval")
    graph.add((interval, RDF.type, TIME.ProperInterval))
    graph.add((constraint, TIME.hasTime, interval))
    for predicate, instant in ((TIME.hasBeginning, "entry"), (TIME.hasEnd, "now")):
        graph.add((interval, predicate, URIRef(f"{interval}-{instant}")))
        graph.add((URIRef(f"{interval}-{instant}"), RDF.type, TIME.Instant))
    graph.add((constraint, RDF.type, CSTR.Constraint))
    graph.add((constraint, RDF.type, CSTR_EXT.TimeConstraint))
    graph.add((constraint, RDF.type, CSTR.EqualityConstraint))
    graph.add((constraint, CSTR.quantity, durations["elapsed"]))
    graph.add((constraint, CSTR["reference-value"], durations["reference"]))
    graph.add((constraint, CSTR_EXT.tolerance, durations["tolerance"]))
    evaluator = URIRef(f"{constraint}-eval")
    graph.add((evaluator, RDF.type, CSTR_HDL.ConstraintEvaluator))
    graph.add((evaluator, RDF.type, CSTR_HDL.ErrorEvaluator))
    graph.add((evaluator, CSTR_HDL.constraint, constraint))
    graph.add((evaluator, CSTR_HDL.error, durations["elapsed"]))

    ev = constraint_evaluator(Model(graph=graph, app_path=Path("model-app.ld.json")), evaluator)

    assert (ev.elapsed_op, ev.elapsed_threshold_s, ev.elapsed_tolerance_s) == ("==", 5.0, 0.01)
