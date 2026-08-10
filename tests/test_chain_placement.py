# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""Where a frame sits on a solver's chain is resolved while generating, so a frame the chain
never reaches is an error here rather than a dead controller on its first tick."""

from __future__ import annotations

import pytest
from rdf_utils.constraints import ConstraintViolation

from motion_spec.classes.bindings import ChainBinding
from motion_spec.classes.geometry import Frame, SimplicialComplex
from motion_spec.rdf_parser.resources import _place_on_chain

SITE = "https://example.test/ft_tree/wrist_ft_body/wrist_ft_site"
BODY = "https://example.test/ft_tree/wrist_ft_body"


def _chain() -> ChainBinding:
    return ChainBinding(
        root="base_link",
        end="tip",
        tip="tip",
        tree="tree",
        name="chain",
        joints=[],
        frames={SITE: {"index": 8, "offset": None}},
        bodies={BODY: 8},
    )


def test_a_frame_that_is_no_segment_resolves_through_the_body_carrying_it() -> None:
    frame = Frame("wrist_ft_site", uri=SITE)
    _place_on_chain(_chain(), frame, "solver")
    assert frame.segment == 8


def test_a_body_resolves_to_the_segment_standing_for_it() -> None:
    body = SimplicialComplex("wrist_ft_body", uri=BODY)
    _place_on_chain(_chain(), body, "solver")
    assert body.segment == 8


def test_a_frame_the_chain_never_reaches_fails_while_generating() -> None:
    with pytest.raises(ConstraintViolation, match="not on the chain"):
        _place_on_chain(_chain(), Frame("elbow", uri="https://example.test/other/elbow"), "solver")
