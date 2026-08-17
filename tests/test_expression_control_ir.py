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
import unittest
from pathlib import Path

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
from rdflib import Dataset, Graph, Literal, URIRef
from rdflib.namespace import RDF, XSD

from motion_spec.classes.geometry import Subspace
from motion_spec.rdf_parser import constraint_handler, quantities
from motion_spec.rdf_parser.model import Model

NS = "https://example.test/"


def _model(graph: Graph) -> Model:
    return Model(
        graph=graph, app_path=Path("model-app.ld.json"), imported_models=[], imported_provenance=[]
    )


def _scene(graph: Graph) -> URIRef:
    """One pose seen by one frame, which every measured view below cuts an axis out of."""
    frame = URIRef(f"{NS}base-frame")
    graph.add((frame, RDF.type, GEOM_ENT.Frame))
    pose = URIRef(f"{NS}pose-ee-base")
    graph.add((pose, RDF.type, GEOM_COORD.PoseCoordinate))
    graph.add((pose, GEOM_COORD["as-seen-by"], frame))
    return pose


def _view(graph: Graph, pose: URIRef, axis: str, subspace=MAP_EXT["position"]) -> URIRef:
    """A per-axis view of `pose`, shaped the way the DSL emits one."""
    scalar = URIRef(f"{NS}{pose.split('/')[-1]}.{axis}")
    graph.add((scalar, RDF.type, QUDT_SCHEMA.Quantity))
    view = URIRef(f"{scalar}-view")
    graph.add((view, RDF.type, MAP.View))
    graph.add((view, MAP.superobject, pose))
    graph.add((view, MAP.subobject, scalar))
    graph.add((view, MAP.subspace, subspace))
    graph.add((view, MAP.axis, MAP[axis]))
    return scalar


def _literal(graph: Graph, name: str, value: float) -> URIRef:
    node = URIRef(f"{NS}{name}")
    graph.add((node, RDF.type, QUDT_SCHEMA.Quantity))
    graph.add((node, QUDT_SCHEMA.value, Literal(value, datatype=XSD.double)))
    graph.add((node, QUDT_SCHEMA.unit, URIRef("https://qudt.org/vocab/unit/M")))
    return node


def _sum(graph: Graph, name: str, operands) -> URIRef:
    operation = URIRef(f"{NS}{name}-add")
    out = URIRef(f"{NS}{name}")
    graph.add((out, RDF.type, QUDT_SCHEMA.Quantity))
    graph.add((operation, RDF.type, ALGO_EXT.Addition))
    for operand in operands:
        graph.add((operation, ALGO_EXT["in"], operand))
    graph.add((operation, ALGO_EXT.out, out))
    return out


def _product(graph: Graph, name: str, operands) -> URIRef:
    operation = URIRef(f"{NS}{name}-multiply")
    out = URIRef(f"{NS}{name}")
    graph.add((out, RDF.type, QUDT_SCHEMA.Quantity))
    graph.add((operation, RDF.type, ALGO_EXT.Multiplication))
    for operand in operands:
        graph.add((operation, ALGO_EXT["in"], operand))
    graph.add((operation, ALGO_EXT.out, out))
    return out


def _difference(graph: Graph, name: str, minuend: URIRef, subtrahend: URIRef) -> URIRef:
    operation = URIRef(f"{NS}{name}-subtract")
    out = URIRef(f"{NS}{name}")
    graph.add((out, RDF.type, QUDT_SCHEMA.Quantity))
    graph.add((operation, RDF.type, ALGO_EXT.Subtraction))
    graph.add((operation, ALGO_EXT.minuend, minuend))
    graph.add((operation, ALGO_EXT.subtrahend, subtrahend))
    graph.add((operation, ALGO_EXT.out, out))
    return out


def _controlled(graph: Graph, quantity: URIRef, name: str = "hold") -> URIRef:
    """A PID-driven equality constraint on `quantity`, attached to a handler."""
    constraint = URIRef(f"{NS}{name}")
    graph.add((constraint, RDF.type, CSTR.Constraint))
    graph.add((constraint, RDF.type, CSTR.EqualityConstraint))
    graph.add((constraint, CSTR.quantity, quantity))
    controller = URIRef(f"{NS}ctrl-{name}")
    graph.add((controller, RDF.type, CSTR_HDL.ProportionalIntegralDerivative))
    graph.add((controller, CSTR_HDL.constraint, constraint))
    graph.add((controller, CSTR_HDL["proportional-gain"], Literal(200.0, datatype=XSD.double)))
    handler = URIRef(f"{NS}handler")
    graph.add((handler, RDF.type, CSTR_HDL.ConstraintHandler))
    graph.add((handler, CSTR_HDL.controllers, controller))
    return controller


def _directions(build) -> tuple:
    """`(ControlDirections, model, controller)` for a graph `build` populates."""
    graph = Dataset(default_union=True)
    controller = build(graph)
    model = _model(graph)
    return constraint_handler._authored_controller_axes(model)[controller], model, controller


class ExpressionSolverRows(unittest.TestCase):
    def test_a_unit_coefficient_expression_drives_the_row_its_plain_view_drives(self) -> None:
        def plain(graph):
            return _controlled(graph, _view(graph, _scene(graph), "z"))

        def expression(graph):
            scalar = _view(graph, _scene(graph), "z")
            return _controlled(graph, _sum(graph, "expr", [scalar, _literal(graph, "zero", 0.0)]))

        plain_directions, _, _ = _directions(plain)
        expression_directions, _, _ = _directions(expression)
        self.assertEqual(plain_directions.axes, (quantities.SpatialAxis(Subspace.Linear, "z"),))
        self.assertEqual(expression_directions.axes, plain_directions.axes)
        self.assertEqual(expression_directions.gradient_norm, 1.0)

    def test_a_scaled_single_leaf_keeps_its_axis_and_carries_the_scale_into_the_gains(self) -> None:
        def build(graph):
            scalar = _view(graph, _scene(graph), "z")
            return _controlled(
                graph, _product(graph, "expr", [scalar, _literal(graph, "two", 2.0)])
            )

        directions, model, controller = _directions(build)
        self.assertEqual(directions.axes, (quantities.SpatialAxis(Subspace.Linear, "z"),))
        self.assertEqual(directions.gradient_norm, 2.0)
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
        self.assertEqual(
            constraint_handler._gain(model, plan, CSTR_HDL["proportional-gain"]), 100.0
        )

    def test_two_measured_axes_give_one_column_along_the_normalized_gradient(self) -> None:
        def build(graph):
            pose = _scene(graph)
            return _controlled(
                graph, _sum(graph, "expr", [_view(graph, pose, "x"), _view(graph, pose, "y")])
            )

        directions, model, _ = _directions(build)
        (axis,) = directions.axes
        self.assertEqual(axis.subspace, Subspace.Linear)
        self.assertIsNone(axis.frame_axis)
        self.assertAlmostEqual(directions.gradient_norm, math.sqrt(2.0))
        gradient = quantities.direction(model, axis.direction)
        for component, expected in zip(gradient.direction, (0.5**0.5, 0.5**0.5, 0.0)):
            self.assertAlmostEqual(component, expected)

    def test_a_gradient_that_cancels_names_no_direction_to_drive(self) -> None:
        def build(graph):
            scalar = _view(graph, _scene(graph), "z")
            return _controlled(graph, _difference(graph, "expr", scalar, scalar))

        with self.assertRaisesRegex(ConstraintViolation, "no measured view a solver moves"):
            _directions(build)

    def test_measured_views_of_two_frames_state_no_single_gradient(self) -> None:
        def build(graph):
            other = URIRef(f"{NS}pose-ee-forearm")
            graph.add((other, RDF.type, GEOM_COORD.PoseCoordinate))
            graph.add((other, GEOM_COORD["as-seen-by"], URIRef(f"{NS}forearm-frame")))
            return _controlled(
                graph,
                _sum(graph, "expr", [_view(graph, _scene(graph), "x"), _view(graph, other, "y")]),
            )

        with self.assertRaisesRegex(ConstraintViolation, "different frames"):
            _directions(build)

    def test_a_gradient_across_both_subspaces_needs_one_constraint_per_subspace(self) -> None:
        def build(graph):
            pose = _scene(graph)
            return _controlled(
                graph,
                _sum(
                    graph,
                    "expr",
                    [
                        _view(graph, pose, "x"),
                        _view(graph, pose, "y", subspace=MAP_EXT["orientation"]),
                    ],
                ),
            )

        with self.assertRaisesRegex(ConstraintViolation, "linear and the angular subspace"):
            _directions(build)

    def test_a_product_of_two_measured_views_states_no_gradient(self) -> None:
        def build(graph):
            pose = _scene(graph)
            return _controlled(
                graph, _product(graph, "expr", [_view(graph, pose, "x"), _view(graph, pose, "y")])
            )

        with self.assertRaisesRegex(ConstraintViolation, "multiplies two measured views"):
            _directions(build)


if __name__ == "__main__":
    unittest.main()
