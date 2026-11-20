# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)

import pytest
from motion_spec_dsl.vocab import CSTR, CSTR_HDL, KC_STAT
from rdf_utils.models.vocab import (
    URI_KC_EXT_PRED_OF_JOINT,
    URI_KC_EXT_TYPE_JOINT_LIMIT,
    URI_KC_STAT_JNT_POSITION,
    URI_KC_TYPE_REVOLUTE_JOINT,
)
from rdflib import Dataset, Namespace

from motion_spec.rdf_parser.model import Model
from motion_spec.rdf_parser.operations import ErrorEvaluator, continuous_joints

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
    ("limit", "joints", "wraps"),
    [("", {EX.joint_1}, True), (POSITION_LIMIT, set(), None)],
    ids=["continuous", "position-limited"],
)
def test_only_a_continuous_joint_equality_error_wraps(limit: str, joints: set, wraps) -> None:
    graph = Dataset(default_union=True)
    graph.default_graph.parse(data=JOINT_EQUALITY + limit, format="turtle")
    model = Model(graph=graph)
    assert continuous_joints(model) == joints
    function = ErrorEvaluator().function_step(model, EX.eval)
    assert function is not None
    assert function.get("angular_wrap") is wraps
