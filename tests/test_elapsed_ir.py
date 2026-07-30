# SPDX-License-Identifier: MPL-2.0
"""IR parsing for elapsed (timing) constraints: native OWL-Time Duration dispatch."""

from __future__ import annotations

from rdflib import Graph, Literal, URIRef
from rdflib.namespace import RDF, XSD

from motion_spec.entities import ConstraintEvaluator, EvaluatorType
from motion_spec.ir_gen import Parser, _evaluator_term
from motion_spec.namespace import CSTR, CSTR_EXT, CSTR_HDL, TIME

NS = "https://example.test/"


def _instant(g: Graph, name: str) -> URIRef:
    node = URIRef(f"{NS}{name}")
    g.add((node, RDF.type, TIME.Instant))
    return node


def _duration(g: Graph, name: str, seconds: float | None = None) -> URIRef:
    node = URIRef(f"{NS}{name}")
    g.add((node, RDF.type, TIME.Duration))
    if seconds is not None:
        g.add((node, TIME.numericDuration, Literal(seconds, datatype=XSD.decimal)))
        g.add((node, TIME.unitType, TIME.unitSecond))
    return node


def _interval(g: Graph, name: str, duration: URIRef) -> None:
    node = URIRef(f"{NS}{name}")
    g.add((node, RDF.type, TIME.ProperInterval))
    g.add((node, TIME.hasBeginning, _instant(g, f"{name}-entry")))
    g.add((node, TIME.hasEnd, _instant(g, f"{name}-now")))
    g.add((node, TIME.hasDuration, duration))


def _evaluator(g: Graph, cstr_node: URIRef, measured: URIRef) -> URIRef:
    """A ConstraintEvaluator wired the way builder.py wires an elapsed constraint's own
    (self-referential) error, matching `_emit_error_evaluator`."""
    eval_node = URIRef(f"{cstr_node}-eval")
    g.add((eval_node, RDF.type, CSTR_HDL.ConstraintEvaluator))
    g.add((eval_node, RDF.type, CSTR_HDL.ErrorEvaluator))
    g.add((eval_node, CSTR_HDL.constraint, cstr_node))
    g.add((eval_node, CSTR_HDL.error, measured))
    return eval_node


def test_elapsed_greater_than_reads_native_duration_threshold() -> None:
    g = Graph()
    cstr = URIRef(f"{NS}wait5")
    measured = _duration(g, "wait5-elapsed")
    _interval(g, "wait5-interval", measured)
    threshold = _duration(g, "wait5-threshold", seconds=5.0)
    g.add((cstr, RDF.type, CSTR.Constraint))
    g.add((cstr, RDF.type, CSTR_EXT.TimeConstraint))
    g.add((cstr, RDF.type, CSTR.UnilateralConstraint))
    g.add((cstr, RDF.type, CSTR.GreaterThanConstraint))
    g.add((cstr, CSTR.quantity, measured))
    g.add((cstr, CSTR.threshold, threshold))
    eval_node = _evaluator(g, cstr, measured)

    ev = Parser(g).constraint_evaluator(eval_node)

    assert ev.is_elapsed is True
    assert ev.elapsed_op == ">="
    assert ev.elapsed_threshold_s == 5.0
    assert ev.elapsed_tolerance_s is None


def test_elapsed_equality_reads_reference_and_tolerance_normalized_to_seconds() -> None:
    g = Graph()
    cstr = URIRef(f"{NS}wait-eq")
    measured = _duration(g, "wait-eq-elapsed")
    _interval(g, "wait-eq-interval", measured)
    reference = _duration(g, "wait-eq-reference", seconds=5.0)
    tolerance = _duration(g, "wait-eq-tolerance", seconds=0.01)  # 10 ms normalized
    g.add((cstr, RDF.type, CSTR.Constraint))
    g.add((cstr, RDF.type, CSTR_EXT.TimeConstraint))
    g.add((cstr, RDF.type, CSTR.EqualityConstraint))
    g.add((cstr, CSTR.quantity, measured))
    g.add((cstr, CSTR["reference-value"], reference))
    g.add((cstr, CSTR_EXT.tolerance, tolerance))
    eval_node = _evaluator(g, cstr, measured)

    ev = Parser(g).constraint_evaluator(eval_node)

    assert ev.is_elapsed is True
    assert ev.elapsed_op == "=="
    assert ev.elapsed_threshold_s == 5.0
    assert ev.elapsed_tolerance_s == 0.01


def test_non_time_constraint_is_not_mistaken_for_elapsed() -> None:
    """A constraint whose quantity happens to be Time-kind QUDT (not a TimeConstraint)
    must not be dispatched as elapsed -- the class, not the quantity kind, decides."""
    from motion_spec.namespace import QUDT_SCHEMA
    from rdf_utils.namespace import NS_MM_QUDT_QTY, NS_MM_QUDT_UNIT

    g = Graph()
    cstr = URIRef(f"{NS}some-timer-quantity")
    qty = URIRef(f"{NS}some-timer-quantity-qty")
    g.add((qty, RDF.type, QUDT_SCHEMA.Quantity))
    g.add((qty, QUDT_SCHEMA.hasQuantityKind, NS_MM_QUDT_QTY["Time"]))
    g.add((qty, QUDT_SCHEMA.unit, NS_MM_QUDT_UNIT["SEC"]))
    g.add((qty, QUDT_SCHEMA.value, Literal(1.0, datatype=XSD.double)))
    g.add((cstr, RDF.type, CSTR.Constraint))
    g.add((cstr, RDF.type, CSTR.UnilateralConstraint))
    g.add((cstr, RDF.type, CSTR.GreaterThanConstraint))
    g.add((cstr, CSTR.quantity, qty))
    threshold = URIRef(f"{NS}some-timer-threshold")
    g.add((threshold, RDF.type, QUDT_SCHEMA.Quantity))
    g.add((threshold, QUDT_SCHEMA.hasQuantityKind, NS_MM_QUDT_QTY["Time"]))
    g.add((threshold, QUDT_SCHEMA.unit, NS_MM_QUDT_UNIT["SEC"]))
    g.add((threshold, QUDT_SCHEMA.value, Literal(0.5, datatype=XSD.double)))
    g.add((cstr, CSTR.threshold, threshold))
    eval_node = _evaluator(g, cstr, qty)

    ev = Parser(g).constraint_evaluator(eval_node)

    assert ev.is_elapsed is False


def test_evaluator_term_renders_equality_as_abs_within_tolerance() -> None:
    ev = ConstraintEvaluator(
        id="wait5",
        type_=EvaluatorType.ErrorEvaluator,
        constraint=None,
        error=None,
        is_elapsed=True,
        elapsed_op="==",
        elapsed_threshold_s=5.0,
        elapsed_tolerance_s=0.01,
    )

    term = _evaluator_term(ev, "wait5_start_time")

    assert term == {
        "kind": "elapsed-eq",
        "start_field": "wait5_start_time",
        "threshold": "5.000000",
        "tolerance": "0.010000",
    }
