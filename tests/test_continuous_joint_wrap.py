# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)

from rdflib import Graph, Namespace
from rdflib.namespace import RDF, split_uri

from motion_spec.rdf_parser.graph import ErrorEvaluator, _continuous_joint_leaves
from motion_spec_dsl.rdf_parser.vocab import CSTR, CSTR_HDL, KC_STAT
from rdf_utils.models.vocab import (
    URI_KC_EXT_PRED_OF_JOINT,
    URI_KC_EXT_TYPE_JOINT_LIMIT,
    URI_KC_STAT_JNT_POSITION,
    URI_KC_TYPE_REVOLUTE_JOINT,
)

EX = Namespace("https://example.org/")


def _graph(with_position_limit: bool) -> Graph:
    g = Graph()
    g.add((EX.joint_1, RDF.type, URI_KC_TYPE_REVOLUTE_JOINT))
    if with_position_limit:
        g.add((EX.limit, RDF.type, URI_KC_EXT_TYPE_JOINT_LIMIT))
        g.add((EX.limit, RDF.type, URI_KC_STAT_JNT_POSITION))
        g.add((EX.limit, URI_KC_EXT_PRED_OF_JOINT, EX.joint_1))
    g.add((EX.eval, CSTR_HDL["constraint"], EX.cstr))
    g.add((EX.cstr, RDF.type, CSTR["EqualityConstraint"]))
    g.add((EX.cstr, CSTR["quantity"], EX.q))
    g.add((EX.cstr, CSTR["reference-value"], EX.ref))
    g.add((EX.q, KC_STAT["of-joint"], EX.joint_1))
    return g


def test_continuous_joint_equality_error_wraps() -> None:
    g = _graph(with_position_limit=False)
    assert _continuous_joint_leaves(g) == {"joint_1"}
    closure = ErrorEvaluator().closure_step(g, lambda node: split_uri(str(node))[1], EX.eval)
    assert closure is not None
    assert closure["angular_wrap"] is True


def test_position_limited_joint_error_does_not_wrap() -> None:
    g = _graph(with_position_limit=True)
    assert _continuous_joint_leaves(g) == set()
    closure = ErrorEvaluator().closure_step(g, lambda node: split_uri(str(node))[1], EX.eval)
    assert closure is not None
    assert "angular_wrap" not in closure
