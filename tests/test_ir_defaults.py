# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
# Author: Vamsi Kalagaturu

from __future__ import annotations

import pytest
from motion_spec_dsl.rdf_parser.vocab import CSTR, ENV, EXEC, GEOM_ENT, GEOM_REL
from rdf_utils.constraints import ConstraintViolation
from rdf_utils.models.vocab import (
    URI_ENV_PRED_OF_OBJ,
    URI_EXEC_PRED_HAS_MODELLED_OBJ,
    URI_EXEC_TYPE_SCENE_INST,
)
from rdflib import Dataset, URIRef
from rdflib.namespace import RDF
from scene_dsl.rdf_parser.vocab import URI_BDD_PRED_OF_SCENE, URI_BDD_TYPE_SCENE

from motion_spec.rdf_parser import deployment
from motion_spec.rdf_parser.model import Model

CONSTRAINED_SCENE_OBJECT = f"""
@prefix ex: <https://example.test/> .
@prefix env: <{ENV}> .
@prefix exec: <{EXEC}> .
@prefix geom-ent: <{GEOM_ENT}> .
@prefix cstr: <{CSTR}> .
ex:scene a <{URI_BDD_TYPE_SCENE}> .
ex:scene-instance a <{URI_EXEC_TYPE_SCENE_INST}> ; <{URI_BDD_PRED_OF_SCENE}> ex:scene ;
    <{URI_EXEC_PRED_HAS_MODELLED_OBJ}> ex:modelled-cube .
ex:modelled-cube a env:ModelledObject ; <{URI_ENV_PRED_OF_OBJ}> ex:cube ;
    env:has-object-model ex:cube-asset .
ex:cube-asset a env:ObjectModel ; exec:has-mapping ex:cube-mapping .
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
    model = Model(graph=graph, namespaces=("https://example.test/", "https://example.test/motion/"))

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
            Model(graph=graph), URIRef("https://example.test/real-exec")
        )

