# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)

from pathlib import Path

from motion_spec_dsl.rdf_parser.vocab import CSTR, CSTR_HDL, KC_STAT
from rdf_utils.models.vocab import (
    URI_KC_EXT_PRED_OF_JOINT,
    URI_KC_EXT_TYPE_JOINT_LIMIT,
    URI_KC_STAT_JNT_POSITION,
    URI_KC_TYPE_REVOLUTE_JOINT,
)
from rdflib import Dataset, Namespace
from rdflib.namespace import RDF

from motion_spec.rdf_parser.model import Model
from motion_spec.rdf_parser.operations import ErrorEvaluator, continuous_joint_leaves

EX = Namespace("https://example.org/")


def _model(with_position_limit: bool) -> Model:
    g = Dataset(default_union=True)
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
    return Model(
        graph=g, app_path=Path("model-app.ld.json"), imported_models=[], imported_provenance=[]
    )


def test_continuous_joint_equality_error_wraps() -> None:
    model = _model(with_position_limit=False)
    assert continuous_joint_leaves(model) == {"joint_1"}
    closure = ErrorEvaluator().closure_step(model, EX.eval)
    assert closure is not None
    assert closure["angular_wrap"] is True


def test_position_limited_joint_error_does_not_wrap() -> None:
    model = _model(with_position_limit=True)
    assert continuous_joint_leaves(model) == set()
    closure = ErrorEvaluator().closure_step(model, EX.eval)
    assert closure is not None
    assert "angular_wrap" not in closure
