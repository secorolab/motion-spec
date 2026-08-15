# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""One motion specification, several constraint handlers.

A motion says which constraints hold; a handler says with which controllers, solvers and
monitors, and in which coordination state. The same specification realized in two states is two
motion units, and nothing either of them owns may be shared with the other.
"""

from __future__ import annotations

from pathlib import Path

import pytest
from motion_spec_dsl.gens import _gen_graph
from motion_spec_dsl.langs import motion_spec_metamodel

from motion_spec.rdf_parser.ir import generate_ir

from conftest import requires_workspace

MODEL = Path(__file__).parent / "fixtures" / "shared_motion"
SCENE = Path(__file__).parents[2] / "motion-spec-dsl" / "models" / "admittance_arc_single"
METAMODELS = Path(__file__).resolve().parents[2] / "metamodels"

pytestmark = requires_workspace(SCENE, METAMODELS)


@pytest.fixture(scope="module")
def shared_motion_ir(tmp_path_factory: pytest.TempPathFactory) -> dict:
    """The IR of a model whose S_FIRST and S_SECOND states realize the same hold.

    A fixture of its own rather than a demo model: the invariant is about the lowering, and a
    demo restructured for reasons of its own has twice left it untested.
    """
    tmp_path = tmp_path_factory.mktemp("shared_motion")
    with pytest.MonkeyPatch.context() as mp:
        mp.setenv("METAMODELS_PATH", str(METAMODELS))
        metamodel = motion_spec_metamodel()
        model = metamodel.model_from_file(MODEL / "shared_motion.robmot")
        _gen_graph(metamodel, model, tmp_path, overwrite=True, debug=False)

    return generate_ir(tmp_path / "shared_motion-app.ld.json")


def _shared_units(ir: dict) -> dict[str, list]:
    """The units grouped by the specification they realize, keeping only the shared ones."""
    by_specification: dict[str, list] = {}
    for unit in ir["coordination"]["motions"]:
        by_specification.setdefault(unit.motion_id, []).append(unit)

    return {spec: units for spec, units in by_specification.items() if len(units) > 1}


def test_each_handler_realizing_one_motion_is_its_own_unit(shared_motion_ir: dict) -> None:
    """A unit is named by its handler, so two of them never collapse onto one state struct."""
    shared = _shared_units(shared_motion_ir)
    assert shared, "the model no longer gives any motion two handlers"
    handlers = {unit.id for units in shared.values() for unit in units}
    assert len(handlers) == sum(len(units) for units in shared.values())
    for units in shared.values():
        # Each runs in its own coordination state: that is what makes them distinct realizations
        # rather than two names for one.
        assert len({unit.fsm_state for unit in units}) == len(units)


def test_units_sharing_a_motion_share_nothing_they_command_with(shared_motion_ir: dict) -> None:
    """Controllers, solver slices and drivers are the handler's, not the specification's.

    One driver carrying both handlers' rows would feed the solver constraints whose signals the
    inactive handler never writes.
    """
    for units in _shared_units(shared_motion_ir).values():
        for attribute, ids in (
            ("controllers", [c.id for unit in units for c in unit.controllers]),
            ("solvers", [s.id for unit in units for s in unit.serial_chain_solvers]),
            ("drivers", [s.motion_driver.id for unit in units for s in unit.serial_chain_solvers]),
        ):
            assert len(ids) == len(set(ids)), f"units sharing a motion share {attribute}: {ids}"
