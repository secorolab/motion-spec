# SPDX-License-Identifier: MPL-2.0
"""IR parsing for elapsed (timing) constraints: native OWL-Time Duration dispatch.

The graph keeps the unit a duration was written in, so the reader is what turns 10 ms
into seconds for codegen.
"""

from __future__ import annotations

from pathlib import Path

import pytest
from motion_spec_dsl.rdf_parser.vocab import CSTR, CSTR_EXT, CSTR_HDL, QUDT_SCHEMA, SOSA, TIME
from rdf_utils.constraints import ConstraintViolation
from rdf_utils.namespace import NS_MM_QUDT_QTY as QUDT_QKIND
from rdf_utils.namespace import NS_MM_QUDT_UNIT as QUDT_UNIT
from rdflib import Dataset, Graph, Literal, URIRef
from rdflib.namespace import RDF, XSD

from motion_spec.classes.handlers import ConstraintEvaluator, EvaluatorType
from motion_spec.classes.qudt import Quantity, QuantityKind, Unit
from motion_spec.rdf_parser import quantities
from motion_spec.rdf_parser.coordination import constraint_evaluator, evaluator_term
from motion_spec.rdf_parser.model import Model

NS = "https://example.test/"


def _elapsed(id_: str) -> Quantity:
    """The duration coordinate an elapsed constraint measures into."""
    return Quantity(id_, QuantityKind("Time"), Unit("SEC"), None, False)


def _model(g: Graph) -> Model:
    return Model(
        graph=g, app_path=Path("model-app.ld.json"), imported_models=[], imported_provenance=[]
    )


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


def _interval(g: Graph, name: str, cstr: URIRef) -> None:
    node = URIRef(f"{NS}{name}")
    g.add((node, RDF.type, TIME.ProperInterval))
    g.add((cstr, TIME.hasTime, node))
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
    g = Dataset(default_union=True)
    cstr = URIRef(f"{NS}wait5")
    measured = _duration(g, "wait5-elapsed")
    _interval(g, "wait5-interval", cstr)
    threshold = _duration(g, "wait5-threshold", 5.0)
    g.add((cstr, RDF.type, CSTR.Constraint))
    g.add((cstr, RDF.type, CSTR_EXT.TimeConstraint))
    g.add((cstr, RDF.type, CSTR.UnilateralConstraint))
    g.add((cstr, RDF.type, CSTR.GreaterThanConstraint))
    g.add((cstr, CSTR.quantity, measured))
    g.add((cstr, CSTR.threshold, threshold))
    eval_node = _evaluator(g, cstr, measured)

    ev = constraint_evaluator(_model(g), eval_node)

    assert ev.is_elapsed is True
    assert ev.elapsed_op == ">="
    assert ev.elapsed_threshold_s == 5.0
    assert ev.elapsed_tolerance_s is None
    # No observation instant on the interval's beginning: this clock counts from motion entry.
    assert ev.observed_at_id is None


def test_elapsed_equality_reads_reference_and_tolerance_normalized_to_seconds() -> None:
    g = Dataset(default_union=True)
    cstr = URIRef(f"{NS}wait-eq")
    measured = _duration(g, "wait-eq-elapsed")
    _interval(g, "wait-eq-interval", cstr)
    reference = _duration(g, "wait-eq-reference", 5.0)
    tolerance = _duration(g, "wait-eq-tolerance", 10.0, unit="MilliSEC")
    g.add((cstr, RDF.type, CSTR.Constraint))
    g.add((cstr, RDF.type, CSTR_EXT.TimeConstraint))
    g.add((cstr, RDF.type, CSTR.EqualityConstraint))
    g.add((cstr, CSTR.quantity, measured))
    g.add((cstr, CSTR["reference-value"], reference))
    g.add((cstr, CSTR_EXT.tolerance, tolerance))
    eval_node = _evaluator(g, cstr, measured)

    ev = constraint_evaluator(_model(g), eval_node)

    assert ev.is_elapsed is True
    assert ev.elapsed_op == "=="
    assert ev.elapsed_threshold_s == 5.0
    assert ev.elapsed_tolerance_s == 0.01


def _observed_interval(g: Graph, name: str, cstr: URIRef, pose: URIRef) -> URIRef:
    """An age interval: the constraint states it, and it begins at the pose's last reading."""
    node = URIRef(f"{NS}{name}")
    instant = _instant(g, f"{name}-observed")
    g.add((node, RDF.type, TIME.ProperInterval))
    g.add((cstr, TIME.hasTime, node))
    g.add((node, TIME.hasBeginning, instant))
    g.add((node, TIME.hasEnd, _instant(g, f"{name}-now")))
    g.add((pose, SOSA.phenomenonTime, instant))
    return instant


def _age_constraint(g: Graph, pose: URIRef) -> tuple[URIRef, URIRef]:
    cstr = URIRef(f"{NS}seen")
    measured = _duration(g, "seen-elapsed")
    instant = _observed_interval(g, "seen-interval", cstr, pose)
    threshold = _duration(g, "seen-threshold", 1.0)
    g.add((cstr, RDF.type, CSTR.Constraint))
    g.add((cstr, RDF.type, CSTR_EXT.TimeConstraint))
    g.add((cstr, RDF.type, CSTR.UnilateralConstraint))
    g.add((cstr, RDF.type, CSTR.LessThanConstraint))
    g.add((cstr, CSTR.quantity, measured))
    g.add((cstr, CSTR.threshold, threshold))
    return _evaluator(g, cstr, measured), instant


def test_observation_age_resolves_the_instant_its_pose_is_observed_at(monkeypatch) -> None:
    g = Dataset(default_union=True)
    pose = URIRef(f"{NS}pose-table-cam")
    eval_node, instant = _age_constraint(g, pose)
    model = _model(g)
    monkeypatch.setattr(
        quantities, "perceived_written_poses", lambda _: {"table": [{"pose_id": model.id(pose)}]}
    )

    ev = constraint_evaluator(model, eval_node)

    assert ev.is_elapsed is True
    assert ev.observed_at_id == model.id(instant)
    assert quantities.observation_ages([ev]) == [
        {"coordinate": "seen_elapsed", "observed_at": model.id(instant)}
    ]
    assert quantities.elapsed_coordinate_ids([ev]) == []


def test_observation_age_of_an_unperceived_pose_is_rejected() -> None:
    g = Dataset(default_union=True)
    eval_node, _ = _age_constraint(g, URIRef(f"{NS}pose-table-cam"))

    with pytest.raises(ConstraintViolation, match="nothing observes it"):
        constraint_evaluator(_model(g), eval_node)


def test_evaluator_term_renders_equality_as_abs_within_tolerance() -> None:
    ev = ConstraintEvaluator(
        id="wait5",
        type_=EvaluatorType.ErrorEvaluator,
        constraint=None,
        error=_elapsed("wait5_elapsed"),
        is_elapsed=True,
        elapsed_op="==",
        elapsed_threshold_s=5.0,
        elapsed_tolerance_s=0.01,
    )

    term = evaluator_term(ev)

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
        Path(__file__).resolve().parents[1]
        / "src"
        / "motion_spec"
        / "templates"
        / "domain_monitors.stg"
    ).read_text()
    evaluators = [
        ConstraintEvaluator(
            id="wait",
            type_=EvaluatorType.ErrorEvaluator,
            constraint=None,
            error=_elapsed("wait_elapsed"),
            is_elapsed=True,
            elapsed_op=op,
            elapsed_threshold_s=1.0,
            elapsed_tolerance_s=0.01,
        )
        for op in (">=", "==", "<")
    ]
    evaluators.append(
        ConstraintEvaluator(
            id="reached", type_=EvaluatorType.ErrorEvaluator, constraint=None, error=None
        )
    )
    kinds = {evaluator_term(ev)["kind"] for ev in evaluators}
    assert kinds == {"elapsed", "elapsed-eq", "constraint"}
    for kind in sorted(kinds):
        body = re.search(
            rf"^cond-term-{kind}\(t\) ::= <<(.*?)^>>", template, re.DOTALL | re.MULTILINE
        )
        assert body, f"cond-term-{kind} is no longer in domain_monitors.stg"
        assert "state." not in body.group(1), f"cond-term-{kind} reaches outside shared"
