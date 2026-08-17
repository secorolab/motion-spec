# SPDX-License-Identifier: MPL-2.0
"""IR scheduling for quantity-expression op chains: the generic Addition/Subtraction/
Multiplication/Division operators (already registered in `OPS_GENERIC`) walk a nested chain
output-to-input, so a monitor reading a mixed-op expression's root schedules every interior
op, producers before consumers, with no expression-specific IR code.
"""

from __future__ import annotations

from pathlib import Path

from motion_spec_dsl.rdf_parser.vocab import ALGO_EXT, CSTR, CSTR_HDL, QUDT_SCHEMA
from rdflib import Dataset, Graph, Literal, URIRef
from rdflib.namespace import RDF, XSD

from motion_spec.rdf_parser.model import Model
from motion_spec.rdf_parser.operations import OPS_GENERIC, OPS_HANDLER, Schedule, build_closures

NS = "https://example.test/"


def _model(g: Graph) -> Model:
    return Model(
        graph=g, app_path=Path("model-app.ld.json"), imported_models=[], imported_provenance=[]
    )


def _quantity(g: Graph, name: str, value: float | None = None) -> URIRef:
    node = URIRef(f"{NS}{name}")
    g.add((node, RDF.type, QUDT_SCHEMA.Quantity))
    if value is not None:
        g.add((node, QUDT_SCHEMA.value, Literal(value, datatype=XSD.double)))
    return node


def _evaluator(g: Graph, cstr_node: URIRef, error: URIRef) -> URIRef:
    """A ConstraintEvaluator wired the way builder.py wires a constraint's own error
    (matching `_emit_error_evaluator`, mirrored from `test_elapsed_ir.py`)."""
    eval_node = URIRef(f"{cstr_node}-eval")
    g.add((eval_node, RDF.type, CSTR_HDL.ConstraintEvaluator))
    g.add((eval_node, RDF.type, CSTR_HDL.ErrorEvaluator))
    g.add((eval_node, CSTR_HDL.constraint, cstr_node))
    g.add((eval_node, CSTR_HDL.error, error))
    return eval_node


def _residual_graph() -> tuple[Graph, URIRef, URIRef, URIRef, URIRef]:
    """`residual = f - k * a`: a Subtraction reading a Multiplication's own output, monitored
    by a constraint's error evaluator -- the fixture `<f> - <k> * <a>` lowers to in DSL tests.
    """
    g = Dataset(default_union=True)
    f = _quantity(g, "f", 10.0)
    k = _quantity(g, "k", 1.2)
    a = _quantity(g, "a", 9.81)

    mul = URIRef(f"{NS}residual-multiply")
    mul_out = _quantity(g, "residual-multiply-out")
    g.add((mul, RDF.type, ALGO_EXT.Multiplication))
    g.add((mul, ALGO_EXT["in"], k))
    g.add((mul, ALGO_EXT["in"], a))
    g.add((mul, ALGO_EXT.out, mul_out))

    sub = URIRef(f"{NS}residual-subtract")
    residual = _quantity(g, "residual")
    g.add((sub, RDF.type, ALGO_EXT.Subtraction))
    g.add((sub, ALGO_EXT.minuend, f))
    g.add((sub, ALGO_EXT.subtrahend, mul_out))
    g.add((sub, ALGO_EXT.out, residual))

    reference = _quantity(g, "residual-reference", 0.0)
    cstr = URIRef(f"{NS}residual-band")
    g.add((cstr, RDF.type, CSTR.Constraint))
    g.add((cstr, RDF.type, CSTR.EqualityConstraint))
    g.add((cstr, CSTR.quantity, residual))
    g.add((cstr, CSTR["reference-value"], reference))

    return g, cstr, mul, sub, residual


def test_a_monitor_on_a_mixed_op_expression_schedules_every_interior_op() -> None:
    g, cstr, mul, sub, residual = _residual_graph()
    model = _model(g)
    eval_node = _evaluator(g, cstr, residual)

    steps = Schedule(model).of([eval_node], OPS_GENERIC + OPS_HANDLER)

    assert model.id(mul) in steps
    assert model.id(sub) in steps
    assert steps.index(model.id(mul)) < steps.index(model.id(sub))


def test_each_op_in_the_chain_renders_its_own_closure() -> None:
    g, _cstr, mul, sub, _residual = _residual_graph()
    model = _model(g)

    closures = build_closures(model, OPS_GENERIC)

    assert closures[model.id(mul)]["type"] == model.id(ALGO_EXT.Multiplication)
    assert closures[model.id(sub)]["type"] == model.id(ALGO_EXT.Subtraction)


def test_domain_closures_template_has_an_emit_call_for_every_algo_op() -> None:
    """`domain_closures.stg` renders each op kind via its own `emit-call-<Type>` template."""
    template = (
        Path(__file__).resolve().parents[1]
        / "src"
        / "motion_spec"
        / "templates"
        / "domain_closures.stg"
    ).read_text()
    for op in ("Addition", "Subtraction", "Multiplication", "Division"):
        assert f"emit-call-{op}(" in template
