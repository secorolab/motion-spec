# SPDX-License-Identifier: MPL-2.0
"""IR parsing for elapsed (timing) constraints: native OWL-Time Duration dispatch.

The graph keeps the unit a duration was written in, so the reader is what turns 10 ms
into seconds for codegen.
"""

from __future__ import annotations

from rdflib import Graph, Literal, URIRef
from rdflib.namespace import RDF, XSD

from motion_spec.classes.entities import ConstraintEvaluator, EvaluatorType
from motion_spec.rdf_parser.ir import Parser, _evaluator_term
from motion_spec_dsl.rdf_parser.vocab import CSTR, CSTR_EXT, CSTR_HDL, QUDT_SCHEMA, TIME
from rdf_utils.namespace import NS_MM_QUDT_QTY as QUDT_QKIND, NS_MM_QUDT_UNIT as QUDT_UNIT

NS = "https://example.test/"


def _instant(g: Graph, name: str) -> URIRef:
    node = URIRef(f"{NS}{name}")
    g.add((node, RDF.type, TIME.Instant))
    return node


def _duration(g: Graph, name: str, value: float | None = None, unit: str = "SEC") -> URIRef:
    node = URIRef(f"{NS}{name}")
    g.add((node, QUDT_SCHEMA.hasQuantityKind, QUDT_QKIND["Time"]))
    g.add((node, QUDT_SCHEMA.unit, QUDT_UNIT[unit]))
    if value is None:
        g.add((node, RDF.type, CSTR_EXT.ElapsedDurationCoordinate))
    else:
        g.add((node, RDF.type, TIME.Duration))
        g.add((node, QUDT_SCHEMA.value, Literal(value, datatype=XSD.double)))
    return node


def _interval(g: Graph, name: str, duration: URIRef) -> None:
    node = URIRef(f"{NS}{name}")
    g.add((node, RDF.type, TIME.ProperInterval))
    g.add((node, TIME.hasBeginning, _instant(g, f"{name}-entry")))
    g.add((node, TIME.hasEnd, _instant(g, f"{name}-now")))


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
    threshold = _duration(g, "wait5-threshold", 5.0)
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
    reference = _duration(g, "wait-eq-reference", 5.0)
    tolerance = _duration(g, "wait-eq-tolerance", 10.0, unit="MilliSEC")
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


def test_evaluator_term_renders_equality_as_abs_within_tolerance() -> None:
    ev = ConstraintEvaluator(
        id="wait5",
        type_=EvaluatorType.ErrorEvaluator,
        constraint=None,
        error={"id": "wait5_elapsed"},
        is_elapsed=True,
        elapsed_op="==",
        elapsed_threshold_s=5.0,
        elapsed_tolerance_s=0.01,
    )

    term = _evaluator_term(ev)

    assert term == {
        "kind": "elapsed-eq",
        "elapsed_id": "wait5_elapsed",
        "threshold": "5.000000",
        "tolerance": "0.010000",
    }


def test_a_monitor_condition_reads_nothing_but_shared() -> None:
    """The introspection sample renders a monitor's activation outside the motion it belongs
    to, where neither the motion's state nor its instance exists. Every term a monitor
    condition can carry must therefore resolve through shared alone -- the elapsed term read
    `state.motion_start_time` and would not have compiled in that scope, and no model authors
    one, so nothing rendered it.
    """
    import re
    from pathlib import Path

    template = (
        Path(__file__).resolve().parents[1] / "src" / "motion_spec" / "templates" / "domain_monitors.stg"
    ).read_text()
    evaluators = [
        ConstraintEvaluator(
            id="wait",
            type_=EvaluatorType.ErrorEvaluator,
            constraint=None,
            error={"id": "wait_elapsed"},
            is_elapsed=True,
            elapsed_op=op,
            elapsed_threshold_s=1.0,
            elapsed_tolerance_s=0.01,
        )
        for op in (">=", "==", "<")
    ]
    evaluators.append(
        ConstraintEvaluator(
            id="reached",
            type_=EvaluatorType.ErrorEvaluator,
            constraint=None,
            error=None,
        )
    )
    kinds = {_evaluator_term(ev)["kind"] for ev in evaluators}
    assert kinds == {"elapsed", "elapsed-eq", "constraint"}
    for kind in sorted(kinds):
        body = re.search(rf"^cond-term-{kind}\(t\) ::= <<(.*?)^>>", template, re.S | re.M)
        assert body, f"cond-term-{kind} is no longer in domain_monitors.stg"
        assert "state." not in body.group(1), f"cond-term-{kind} reaches outside shared"
