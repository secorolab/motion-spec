# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""An alignment constraint's controller mints the measured-derivative it references.

The alignment branch of `augment_data` builds its error views from the rotation vector and skips
the pose-difference branch entirely. It used to skip that branch's derivative minting with it, so a
`pid` on an `angle between` view carrying `measured-derivative:` emitted `shared.<ctrl>_measured_
derivative_ang_<axis>` into the handler while nothing ever declared it -- the model generated and
only failed at C++ compile time.
"""

from __future__ import annotations

import shutil
from pathlib import Path

import pytest
from conftest import requires_workspace
from motion_spec_dsl.gens import _gen_graph
from motion_spec_dsl.langs import motion_spec_metamodel

from motion_spec.rdf_parser.ir import generate_ir

MODELS = Path(__file__).parents[2] / "motion-spec-dsl" / "models"
METAMODELS = Path(__file__).resolve().parents[2] / "metamodels"

pytestmark = requires_workspace(MODELS, METAMODELS)

# The stock motion holds full orientation and leaves its alignment cone monitored-only, so the
# alignment claims no solver row. Driving it with a pid instead makes it two angular rows, which
# the orientation hold already owns -- hence dropping that hold as well, or the solve is rejected
# for repeating an acceleration axis.
WHILE_ORI = (
    "        comply-ori: keeping <shared.world.pose-ee-base>.orientation equal to "
    "<spec.hold-orientation>.orientation within <shared.spec.satisfied-band-rot>,\n"
)
CTRL_ORI = (
    "        pid ctrl-comply-ori {\n"
    "            constraint: <compliance.comply-ori>,\n"
    "            measured-derivative: <shared.world.twist-ee-base>.angvel,\n"
    "            Kp: 240,\n"
    "            Ki: 0,\n"
    "            Kd: 160,\n"
    "            decay: 0\n"
    "        },\n"
)
ELBOW_CTRL = "        pid ctrl-comply-elbow {"
ROW_ALIGN = (
    "        pid ctrl-comply-align-forearm { constraint: <compliance.align-forearm>, "
    "measured-derivative: <shared.world.twist-ee-base>.angvel, "
    "Kp: 240, Ki: 0, Kd: 32, decay: 0 },\n"
)


def _model_driving_alignment_with_a_pid(tmp_path: Path) -> Path:
    """admittance_arc_single, with its compliance alignment moved onto a pid that reads angvel."""
    source = MODELS / "admittance_arc_single"
    # The scenex reaches for the ktree beside the model directory, so the siblings come too.
    for entry in source.parent.iterdir():
        if entry.is_file():
            shutil.copy2(entry, tmp_path / entry.name)
    model_dir = tmp_path / source.name
    shutil.copytree(source, model_dir)

    path = model_dir / "admittance_arc_single.robmot"
    text = path.read_text()
    for anchor in (WHILE_ORI, CTRL_ORI, ELBOW_CTRL):
        assert anchor in text, f"model no longer carries the anchor:\n{anchor}"
    text = text.replace(WHILE_ORI, "", 1).replace(CTRL_ORI, "", 1)
    path.write_text(text.replace(ELBOW_CTRL, ROW_ALIGN + ELBOW_CTRL, 1))
    return path


def test_alignment_pid_declares_its_measured_derivative(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setenv("METAMODELS_PATH", str(METAMODELS))
    metamodel = motion_spec_metamodel()
    path = _model_driving_alignment_with_a_pid(tmp_path)
    generated = tmp_path / "gen"
    generated.mkdir()
    _gen_graph(metamodel, metamodel.model_from_file(path), generated, overwrite=True, debug=False)

    ir = generate_ir(generated / "admittance_arc_single-app.ld.json")
    declared = {item.id for item in ir["computation"]["shared_data"] if getattr(item, "id", None)}

    # One per controlled axis, and only those: the reference direction is the base's z, so the
    # rotation about it carries no error and claims no controller.
    assert {
        name for name in declared if "ctrl_comply_align_forearm_measured_derivative" in name
    } == {
        "ctrl_comply_align_forearm_measured_derivative_ang_x",
        "ctrl_comply_align_forearm_measured_derivative_ang_y",
    }
