# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
# Author: Vamsi Kalagaturu

from __future__ import annotations

from pathlib import Path

import pytest
from motion_spec_dsl.rdf_parser.vocab import CSTR, ENV, EXEC, GEOM_ENT, GEOM_REL
from rdf_utils.constraints import ConstraintViolation
from rdflib import Dataset, URIRef
from rdflib.namespace import RDF

from motion_spec.classes.handlers import ConstraintEvaluator, EvaluatorType
from motion_spec.classes.qudt import Quantity, QuantityKind, Unit
from motion_spec.rdf_parser import deployment
from motion_spec.rdf_parser.coordination import evaluator_term
from motion_spec.rdf_parser.model import Model

CONSTRAINED_SCENE_OBJECT = f"""
@prefix ex: <https://example.test/> .
@prefix env: <{ENV}> .
@prefix exec: <{EXEC}> .
@prefix geom-ent: <{GEOM_ENT}> .
@prefix cstr: <{CSTR}> .
ex:modelled-cube a env:ModelledObject ; env:has-object-model ex:cube-asset .
ex:cube-asset exec:has-mapping ex:cube-mapping .
ex:cube-mapping exec:maps ex:cube-body .
ex:cube-body a geom-ent:RigidBody ; geom-ent:simplices ex:cube-frame .
ex:cube-frame a geom-ent:Frame .
ex:near-cube a cstr:Constraint ; cstr:quantity ex:pose-cube-position .
ex:pose-cube-position <{GEOM_REL.of}> ex:cube-frame .
"""


def test_shared_or_nested_names_get_distinct_ids_and_a_fold_onto_one_id_is_rejected() -> None:
    graph = Dataset(default_union=True)
    nodes = [
        URIRef("https://example.test/motion/spec/path1/reference"),
        URIRef("https://example.test/motion/spec/path2/reference"),
        URIRef("https://example.test/motion/handler-home/hold-position"),
        URIRef("https://example.test/motion/handler-hold/hold-position"),
    ]
    for node in nodes:
        graph.add((node, RDF.type, RDF.Property))
    model = Model(
        graph=graph,
        app_path=Path("/tmp/app.json"),
        namespaces=("https://example.test/", "https://example.test/motion/"),
    )

    assert len({model.id(node) for node in nodes}) == len(nodes)
    model.id(URIRef("https://example.test/hold-position"))
    with pytest.raises(ConstraintViolation, match="both name the id 'hold_position'"):
        model.id(URIRef("https://example.test/hold_position"))


def test_real_world_execution_rejects_a_constrained_scene_object() -> None:
    """A scene object's pose comes from the simulator; on hardware nothing measures it."""
    graph = Dataset(default_union=True)
    graph.default_graph.parse(data=CONSTRAINED_SCENE_OBJECT, format="turtle")
    with pytest.raises(ConstraintViolation, match="constrains scene object"):
        deployment.reject_scene_objects_on_hardware(
            Model(graph=graph, app_path=Path("/tmp/app.json")),
            URIRef("https://example.test/real-exec"),
        )


def test_an_authored_band_rides_the_constraint_term() -> None:
    """The term is what every reader renders from -- the motion's condition, the level monitor and
    the telemetry sample -- so the authored band has to travel with it, not the global default."""
    error = Quantity("near_err", QuantityKind("Length"), Unit("M"), None, False)
    band = Quantity("pos_band", QuantityKind("Length"), Unit("M"), 0.002, False)
    evaluator = ConstraintEvaluator(
        id="near", type_=EvaluatorType.ErrorEvaluator, constraint=None, error=error, tolerance=band
    )
    assert evaluator_term(evaluator) == {
        "kind": "constraint",
        "error_id": "near_err",
        "tolerance_id": "pos_band",
    }
