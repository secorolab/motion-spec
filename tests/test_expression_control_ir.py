# SPDX-License-Identifier: MPL-2.0
"""Solver rows for a controlled quantity expression.

A constraint whose quantity is an expression names no view of its own, so its alpha column is
the expression's gradient over the views it is built from: one coefficient per measured axis,
normalized, with the norm carried into the gains so the loop gain stays the authored one. A
single positive coefficient on one axis has to come back as exactly the row the same constraint
gets when it is written as a plain view.
"""

from __future__ import annotations

import math
from pathlib import Path

import pytest
from motion_spec_dsl.rdf_parser.vocab import (
    ALGO_EXT,
    CSTR,
    CSTR_HDL,
    GEOM_COORD,
    GEOM_ENT,
    MAP,
    MAP_EXT,
    QUDT_SCHEMA,
)
from rdf_utils.constraints import ConstraintViolation
from rdflib import Dataset, URIRef

from motion_spec.classes.geometry import Subspace
from motion_spec.rdf_parser import constraint_handler, quantities
from motion_spec.rdf_parser.model import Model
from motion_spec.rdf_parser.operations import OPS_GENERIC, OPS_HANDLER, Schedule

NS = "https://example.test/"

# One pose seen by one frame, with the per-axis views of it the DSL emits, and the literals and
# operations the expressions below combine them with.
GRAPH = f"""
@prefix ex: <{NS}> .
@prefix algo-ext: <{ALGO_EXT}> .
@prefix cstr: <{CSTR}> .
@prefix cstr-hdl: <{CSTR_HDL}> .
@prefix geom-coord: <{GEOM_COORD}> .
@prefix geom-ent: <{GEOM_ENT}> .
@prefix map: <{MAP}> .
@prefix map-ext: <{MAP_EXT}> .
@prefix qudt: <{QUDT_SCHEMA}> .
@prefix xsd: <http://www.w3.org/2001/XMLSchema#> .

ex:base-frame a geom-ent:Frame .
ex:pose-ee-base a geom-coord:PoseCoordinate ; geom-coord:as-seen-by ex:base-frame .
""" + "".join(
    f"""
ex:pose-ee-base.{axis} a qudt:Quantity .
ex:pose-ee-base.{axis}-view a map:View ; map:superobject ex:pose-ee-base ;
    map:subobject ex:pose-ee-base.{axis} ; map:subspace map-ext:position ; map:axis map:{axis} .
"""
    for axis in ("x", "y", "z")
) + "".join(
    f"""
ex:{name} a qudt:Quantity ; qudt:value "{value}"^^xsd:double ;
    qudt:unit <https://qudt.org/vocab/unit/M> .
"""
    for name, value in (("zero", 0.0), ("two", 2.0), ("f", 10.0), ("k", 1.2), ("a", 9.81))
)

# A PID-driven equality constraint on `ex:expr`, attached to a handler.
CONTROLLED = """
ex:expr a qudt:Quantity .
ex:hold a cstr:Constraint, cstr:EqualityConstraint ; cstr:quantity ex:expr .
ex:ctrl-hold a cstr-hdl:ProportionalIntegralDerivative ; cstr-hdl:constraint ex:hold ;
    cstr-hdl:proportional-gain "200.0"^^xsd:double .
ex:handler a cstr-hdl:ConstraintHandler ; cstr-hdl:controllers ex:ctrl-hold .
"""


@pytest.mark.parametrize(
    "expression",
    [
        "ex:expr-view a map:View ; map:superobject ex:pose-ee-base ; map:subobject ex:expr ;\n"
        "    map:subspace map-ext:position ; map:axis map:z .\n",
        "ex:expr-op a algo-ext:Addition ; algo-ext:in ex:pose-ee-base.z, ex:zero ;\n"
        "    algo-ext:out ex:expr .\n",
    ],
    ids=["plain-view", "unit-coefficient"],
)
def test_a_unit_coefficient_expression_drives_the_row_its_plain_view_drives(expression) -> None:
    graph = Dataset(default_union=True)
    graph.default_graph.parse(data=GRAPH + CONTROLLED + expression, format="turtle")
    model = Model(graph=graph, app_path=Path("model-app.ld.json"))
    directions = constraint_handler._authored_controller_axes(model)[URIRef(f"{NS}ctrl-hold")]
    assert directions.axes == (quantities.SpatialAxis(Subspace.Linear, "z"),)
    assert directions.gradient_norm == 1.0


def test_a_scaled_single_leaf_keeps_its_axis_and_carries_the_scale_into_the_gains() -> None:
    graph = Dataset(default_union=True)
    graph.default_graph.parse(
        data=GRAPH
        + CONTROLLED
        + "ex:expr-op a algo-ext:Multiplication ; algo-ext:in ex:pose-ee-base.z, ex:two ;\n"
        "    algo-ext:out ex:expr .\n",
        format="turtle",
    )
    model = Model(graph=graph, app_path=Path("model-app.ld.json"))
    controller = URIRef(f"{NS}ctrl-hold")
    directions = constraint_handler._authored_controller_axes(model)[controller]
    assert directions.axes == (quantities.SpatialAxis(Subspace.Linear, "z"),)
    assert directions.gradient_norm == 2.0
    plan = constraint_handler.ControllerDerivation(
        handler=None,
        motion=None,
        controller=controller,
        solver=None,
        constraint=None,
        quantity=None,
        view=None,
        axes=directions.axes,
        gradient_norm=directions.gradient_norm,
    )
    assert constraint_handler._gain(model, plan, CSTR_HDL["proportional-gain"]) == 100.0


def test_two_measured_axes_give_one_column_along_the_normalized_gradient() -> None:
    graph = Dataset(default_union=True)
    graph.default_graph.parse(
        data=GRAPH
        + CONTROLLED
        + "ex:expr-op a algo-ext:Addition ; algo-ext:in ex:pose-ee-base.x, ex:pose-ee-base.y ;\n"
        "    algo-ext:out ex:expr .\n",
        format="turtle",
    )
    model = Model(graph=graph, app_path=Path("model-app.ld.json"))
    directions = constraint_handler._authored_controller_axes(model)[URIRef(f"{NS}ctrl-hold")]
    (axis,) = directions.axes
    assert axis.subspace == Subspace.Linear
    assert axis.frame_axis is None
    assert directions.gradient_norm == pytest.approx(math.sqrt(2.0))
    gradient = quantities.direction(model, axis.direction)
    assert list(gradient.direction) == pytest.approx([0.5**0.5, 0.5**0.5, 0.0])


def test_a_gradient_that_cancels_names_no_direction_to_drive() -> None:
    graph = Dataset(default_union=True)
    graph.default_graph.parse(
        data=GRAPH
        + CONTROLLED
        + "ex:expr-subtract a algo-ext:Subtraction ; algo-ext:minuend ex:pose-ee-base.z ;\n"
        "    algo-ext:subtrahend ex:pose-ee-base.z ; algo-ext:out ex:expr .\n",
        format="turtle",
    )
    with pytest.raises(ConstraintViolation, match="no measured view a solver moves"):
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
