# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)

from pathlib import Path

import pytest
from motion_spec_dsl.rdf_parser.vocab import CSTR, CSTR_HDL, KC_STAT
from rdf_utils.models.vocab import (
    URI_KC_EXT_PRED_OF_JOINT,
    URI_KC_EXT_TYPE_JOINT_LIMIT,
    URI_KC_STAT_JNT_POSITION,
    URI_KC_TYPE_REVOLUTE_JOINT,
)
from rdflib import Dataset, Namespace

from motion_spec.rdf_parser.model import Model
from motion_spec.rdf_parser.operations import ErrorEvaluator, continuous_joint_leaves

EX = Namespace("https://example.org/")

JOINT_EQUALITY = f"""
@prefix ex: <{EX}> .
@prefix cstr: <{CSTR}> .
ex:joint_1 a <{URI_KC_TYPE_REVOLUTE_JOINT}> .
ex:eval <{CSTR_HDL["constraint"]}> ex:cstr .
ex:cstr a cstr:EqualityConstraint ; cstr:quantity ex:q ; cstr:reference-value ex:ref .
ex:q <{KC_STAT["of-joint"]}> ex:joint_1 .
"""

POSITION_LIMIT = f"""
ex:limit a <{URI_KC_EXT_TYPE_JOINT_LIMIT}>, <{URI_KC_STAT_JNT_POSITION}> ;
    <{URI_KC_EXT_PRED_OF_JOINT}> ex:joint_1 .
"""


@pytest.mark.parametrize(
    ("limit", "leaves", "wraps"),
    [("", {"joint_1"}, True), (POSITION_LIMIT, set(), None)],
    ids=["continuous", "position-limited"],
)
def test_only_a_continuous_joint_equality_error_wraps(limit: str, leaves: set, wraps) -> None:
    graph = Dataset(default_union=True)
    graph.default_graph.parse(data=JOINT_EQUALITY + limit, format="turtle")
    model = Model(graph=graph, app_path=Path("model-app.ld.json"))
    assert continuous_joint_leaves(model) == leaves
    closure = ErrorEvaluator().closure_step(model, EX.eval)
    assert closure is not None
    assert closure.get("angular_wrap") is wraps
