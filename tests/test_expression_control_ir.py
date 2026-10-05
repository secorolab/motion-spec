# SPDX-License-Identifier: MPL-2.0
"""Solver rows for a controlled quantity expression.

A constraint whose quantity is an expression names no view of its own: the DSL emits its
gradient as the same arithmetic over its terms' gradients, and the expression's top operator
names the unit gradient and the norm it was taken from. The row runs along that shared vector, in
the half its terms move in, and the control law divides its error by that norm.
"""

from __future__ import annotations

from pathlib import Path

import pytest
from motion_spec_dsl.rdf_parser.vocab import (
    ALGO_EXT,
    CSTR,
    CSTR_HDL,
    GEOM_COORD,
    GEOM_ENT,
    GEOM_OP_EXT,
    MAP,
    MAP_EXT,
    QUDT_SCHEMA,
)
from rdf_utils.constraints import ConstraintViolation
from rdflib import Dataset, URIRef

from motion_spec.classes.geometry import Subspace
from motion_spec.rdf_parser import constraint_handler
from motion_spec.rdf_parser.model import Model
from motion_spec.rdf_parser.operations import OPS_GENERIC, OPS_HANDLER, Schedule

NS = "https://example.test/"

# One pose seen by one frame, with the per-axis views of it the DSL emits, and the literals and
# operations the expressions below combine them with.
GRAPH = (
    f"""
@prefix ex: <{NS}> .
@prefix algo-ext: <{ALGO_EXT}> .
@prefix cstr: <{CSTR}> .
@prefix cstr-hdl: <{CSTR_HDL}> .
@prefix geom-coord: <{GEOM_COORD}> .
@prefix geom-ent: <{GEOM_ENT}> .
@prefix geom-op-ext: <{GEOM_OP_EXT}> .
@prefix map: <{MAP}> .
@prefix map-ext: <{MAP_EXT}> .
@prefix qudt: <{QUDT_SCHEMA}> .
@prefix xsd: <http://www.w3.org/2001/XMLSchema#> .

ex:base-frame a geom-ent:Frame .
ex:pose-ee-base a geom-coord:PoseCoordinate ; geom-coord:as-seen-by ex:base-frame .
"""
    + "".join(
        f"""
ex:pose-ee-base.{subspace}.{axis} a qudt:Quantity .
ex:pose-ee-base.{subspace}.{axis}-view a map:View ; map:superobject ex:pose-ee-base ;
    map:subobject ex:pose-ee-base.{subspace}.{axis} ; map:subspace map-ext:{subspace} ;
    map:axis map:{axis} .
"""
        for subspace in ("position", "orientation")
        for axis in ("x", "y", "z")
    )
    + "".join(
        f"""
ex:{name} a qudt:Quantity ; qudt:value "{value}"^^xsd:double ;
    qudt:unit <https://qudt.org/vocab/unit/M> .
"""
        for name, value in (("zero", 0.0), ("f", 10.0), ("k", 1.2), ("a", 9.81))
    )
)

# A PID-driven equality constraint on `ex:expr`, attached to a handler.
CONTROLLED = """
ex:expr a qudt:Quantity .
ex:hold a cstr:Constraint, cstr:EqualityConstraint ; cstr:quantity ex:expr .
ex:ctrl-hold a cstr-hdl:ProportionalIntegralDerivative ; cstr-hdl:constraint ex:hold ;
    cstr-hdl:proportional-gain "200.0"^^xsd:double .
ex:handler a cstr-hdl:ConstraintHandler ; cstr-hdl:controllers ex:ctrl-hold .
ex:expr-gradient a geom-coord:DirectionCoordinate ; geom-coord:as-seen-by ex:base-frame .
ex:expr-gradient-norm a qudt:Quantity .
"""


@pytest.mark.parametrize(
    ("subspace", "half"),
    [("position", Subspace.Linear), ("orientation", Subspace.Angular)],
    ids=["linear", "angular"],
)
def test_an_expression_runs_along_the_gradient_its_operator_names(subspace, half) -> None:
    graph = Dataset(default_union=True)
    graph.default_graph.parse(
        data=GRAPH
        + CONTROLLED
        + f"ex:expr-op a algo-ext:Subtraction ; algo-ext:minuend ex:pose-ee-base.{subspace}.x ;\n"
        f"    algo-ext:subtrahend ex:pose-ee-base.{subspace}.y ; algo-ext:out ex:expr ;\n"
        "    geom-op-ext:gradient ex:expr-gradient ; geom-op-ext:norm ex:expr-gradient-norm .\n",
        format="turtle",
    )
    model = Model(graph=graph, app_path=Path("model-app.ld.json"))
    directions = constraint_handler._authored_controller_axes(model)[URIRef(f"{NS}ctrl-hold")]
    (axis,) = directions.axes
    assert axis.subspace == half
    assert axis.direction == URIRef(f"{NS}expr-gradient")
    assert directions.gradient_norm == URIRef(f"{NS}expr-gradient-norm")
    assert directions.gradient_frame == URIRef(f"{NS}base-frame")


def test_a_controlled_expression_naming_no_gradient_is_refused() -> None:
    graph = Dataset(default_union=True)
    graph.default_graph.parse(
        data=GRAPH
        + CONTROLLED
        + "ex:expr-op a algo-ext:Addition ; algo-ext:in ex:pose-ee-base.position.x, ex:zero ;\n"
        "    algo-ext:out ex:expr .\n",
        format="turtle",
    )
    with pytest.raises(ConstraintViolation, match="names no gradient"):
        constraint_handler._authored_controller_axes(
            Model(graph=graph, app_path=Path("model-app.ld.json"))
        )


def test_a_monitor_on_a_mixed_op_expression_schedules_every_interior_op() -> None:
    """`residual = f - k * a` read by an error evaluator schedules the product before the
    difference, or the monitor reads a value one tick stale."""
    graph = Dataset(default_union=True)
    graph.default_graph.parse(
        data=GRAPH
        + """
ex:residual-product a qudt:Quantity .
ex:residual-product-op a algo-ext:Multiplication ; algo-ext:in ex:k, ex:a ;
    algo-ext:out ex:residual-product .
ex:residual a qudt:Quantity .
ex:residual-subtract a algo-ext:Subtraction ; algo-ext:minuend ex:f ;
    algo-ext:subtrahend ex:residual-product ; algo-ext:out ex:residual .
ex:residual-band a cstr:Constraint, cstr:EqualityConstraint ;
    cstr:quantity ex:residual ; cstr:reference-value ex:zero .
ex:residual-band-eval a cstr-hdl:ConstraintEvaluator, cstr-hdl:ErrorEvaluator ;
    cstr-hdl:constraint ex:residual-band ; cstr-hdl:error ex:residual .
""",
        format="turtle",
    )
    model = Model(graph=graph, app_path=Path("model-app.ld.json"))

    steps = Schedule(model).of([URIRef(f"{NS}residual-band-eval")], OPS_GENERIC + OPS_HANDLER)

    multiply = model.id(URIRef(f"{NS}residual-product-op"))
    subtract = model.id(URIRef(f"{NS}residual-subtract"))
    assert steps.index(multiply) < steps.index(subtract)
