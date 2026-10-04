# SPDX-License-Identifier: MPL-2.0
"""What robot.toml must state for a real run, checked before the run rather than failing as a
zero pose or an unreached device on hardware."""

from __future__ import annotations

import json
from pathlib import Path

import pytest
from motion_spec_dsl.rdf_parser.vocab import EXEC
from rdf_utils.constraints import ConstraintViolation
from rdflib import Dataset

from motion_spec.rdf_parser.model import Model
from motion_spec.rdf_parser.resources import config_poses
from motion_spec.runs.runner import RunnerError, _validate_robot_config

APP = "https://secorolab.github.io/models/demo/"

# A pose the model reads from the execution context's config, at `poses.table`.
CONFIG_POSE = f"""
@prefix app: <{APP}> .
@prefix exec: <{EXEC}> .
app:demo-exec a exec:ExecutionContext ; exec:has-resource app:demo-exec.config .
app:demo-exec.config exec:path "/somewhere/robot.toml" .
<{APP}shared/spec/look-at-table> exec:has-resource app:demo-exec.config ;
    <https://schema.org/identifier> "poses.table" .
"""


@pytest.mark.parametrize(
    ("config", "message"),
    [
        ({}, "does not state"),
        (
            {"poses": {"look-at-table": {"position": [0, 0, 0], "orientation": [0, 0, 0]}}},
            "does not state",
        ),
        ({"poses": {"table": {"position": [0.4, 0.24, 0.22]}}}, "orientation"),
        ({"poses": {"table": {"position": [0.4, 0.24], "orientation": [0, 0, 0]}}}, "position"),
    ],
)
def test_a_config_that_does_not_state_the_pose_is_rejected(config: dict, message: str) -> None:
    graph = Dataset(default_union=True)
    graph.default_graph.parse(data=CONFIG_POSE, format="turtle")
    model = Model(graph=graph, app_path=Path("model-app.ld.json"), namespaces=(f"{APP}shared/spec/",))
    with pytest.raises(ConstraintViolation, match=message):
        config_poses(model, config)


_ARM_SECTION = (
    "[agents.arm1]\nip='10.0.0.1'\nuser='u'\npassword='p'\nport=1\n"
    "port_real_time=2\nsession_timeout_ms=3\nconnection_timeout_ms=4\n"
)
_FT_SECTION = "[arm1.wrist_ft]\nport='ttyUSB0'\nbaudrate=19200\nslave_address=9\n"
_GRIPPER_SECTION = "[agents.gripper1]\nport='ttyUSB1'\nbaudrate=115200\nslave_address=9\n"
_FT = {"kind": "RobotiqFT300s", "config_key": "arm1.wrist_ft", "drives": "wrist_ft"}
_ARM = {"kind": "KinovaGen3", "config_key": "agents.arm1", "drives": ""}


REAL_PLATFORM = {"simulated": False, "config": "robot.toml"}
ARM_ROBOT = {"id": "arm_solver", "kind": "serial_chain", "config_key": "agents.arm1"}


@pytest.fixture
def source(tmp_path: Path) -> Path:
    """A generation's generated/ tree; each case writes the IR binding its devices."""
    (tmp_path / "generated" / "model").mkdir(parents=True)
    return tmp_path / "generated"


def test_robot_config_must_cover_every_bound_device(tmp_path: Path, source: Path) -> None:
    (source / "model" / "ir.json").write_text(
        json.dumps(
            {
                "configuration": {"platform": REAL_PLATFORM, "config_poses": []},
                "resources": {"robots": [{**ARM_ROBOT, "devices": [_ARM, _FT]}]},
            }
        )
    )
    config = tmp_path / "robot.toml"

    with pytest.raises(RunnerError, match="robot config not found"):
        _validate_robot_config(source, tmp_path)

    config.write_text("[agents.arm1]\nip = '10.0.0.1'\n")
    with pytest.raises(RunnerError, match="is missing user"):
        _validate_robot_config(source, tmp_path)

    config.write_text(_ARM_SECTION)
    with pytest.raises(RunnerError, match=r"no \[arm1.wrist_ft\] section for the bound Robotiq"):
        _validate_robot_config(source, tmp_path)

    # The FT sensor is a serial device: the arm's network keys say nothing about it.
    config.write_text(_ARM_SECTION + "[arm1.wrist_ft]\nport='ttyUSB0'\n")
    with pytest.raises(RunnerError, match=r"\[arm1.wrist_ft\] is missing baudrate"):
        _validate_robot_config(source, tmp_path)

    config.write_text(_ARM_SECTION + _FT_SECTION)
    _validate_robot_config(source, tmp_path)

    config.write_text(_ARM_SECTION + _FT_SECTION + _GRIPPER_SECTION)
    with pytest.raises(RunnerError, match=r"\[agents.gripper1\] configures nothing"):
        _validate_robot_config(source, tmp_path)


def test_a_pose_the_model_reads_binds_its_section(tmp_path: Path, source: Path) -> None:
    """`config_poses` refuses to generate a model whose pose section is absent, so the run must
    not refuse it for being present -- between them, no real-world model could run at all."""
    (source / "model" / "ir.json").write_text(
        json.dumps(
            {
                "configuration": {
                    "platform": REAL_PLATFORM,
                    "config_poses": [{"id": "home", "config_key": "poses.home"}],
                },
                "resources": {"robots": [{**ARM_ROBOT, "devices": [_ARM]}]},
            }
        )
    )
    config = tmp_path / "robot.toml"
    pose = "[poses.home]\nposition = [0.1, 0.2, 0.3]\norientation = [0.0, 0.0, 0.0]\n"

    config.write_text(_ARM_SECTION + pose)
    _validate_robot_config(source, tmp_path)

    # The numbers may be retuned without regenerating, so their shape is checked on the way in.
    config.write_text(
        _ARM_SECTION + "[poses.home]\nposition = [0.1, 0.2]\norientation = [0, 0, 0]\n"
    )
    with pytest.raises(RunnerError, match=r"\[poses.home\] states no three-number `position`"):
        _validate_robot_config(source, tmp_path)

    # A deployment may keep more poses than the model reads; an unread one is skipped.
    config.write_text(_ARM_SECTION + pose + "[poses.spare]\nposition = [0, 0, 0]\n")
    _validate_robot_config(source, tmp_path)


def test_a_home_is_rejected_on_a_real_device(tmp_path: Path, source: Path) -> None:
    """A home stated for a real run is a number the deployment believes in and nothing acts on."""
    (source / "model" / "ir.json").write_text(
        json.dumps(
            {
                "configuration": {"platform": REAL_PLATFORM, "config_poses": []},
                "resources": {"robots": [{**ARM_ROBOT, "devices": [_ARM]}]},
            }
        )
    )
    config = tmp_path / "robot.toml"

    config.write_text(_ARM_SECTION + "home = [0.0, 0.6, 3.14, -2.0, 0.0, 1.1, 1.57]\n")
    with pytest.raises(RunnerError, match=r"`home` in \[agents.arm1\]"):
        _validate_robot_config(source, tmp_path)

    config.write_text(_ARM_SECTION)
    _validate_robot_config(source, tmp_path)


@pytest.mark.parametrize(
    ("devices", "unreachable", "missing"),
    [
        (
            [{"kind": "KinovaGen3-2F85", "config_key": "agents.arm1", "drives": ""}, _FT],
            r"\[agents.gripper1\] configures nothing",
            None,
        ),
        (
            [_ARM, {"kind": "Robotiq2F85", "config_key": "agents.gripper1", "drives": ""}, _FT],
            None,
            r"no \[agents.gripper1\] section",
        ),
    ],
    ids=["interconnect", "separate"],
)
def test_the_authored_device_decides_which_sections_the_config_needs(
    tmp_path: Path, source: Path, devices: list, unreachable: str | None, missing: str | None
) -> None:
    """The gripper's route is authored, not inferred: one section under the arm's device, two
    under separate ones. A section the run cannot reach is as wrong as a missing one."""
    (source / "model" / "ir.json").write_text(
        json.dumps(
            {
                "configuration": {"platform": REAL_PLATFORM, "config_poses": []},
                "resources": {"robots": [{**ARM_ROBOT, "devices": devices}]},
            }
        )
    )
    config = tmp_path / "robot.toml"
    for sections, rejection in (
        (_ARM_SECTION + _FT_SECTION, missing),
        (_ARM_SECTION + _FT_SECTION + _GRIPPER_SECTION, unreachable),
    ):
        config.write_text(sections)
        if rejection:
            with pytest.raises(RunnerError, match=rejection):
                _validate_robot_config(source, tmp_path)
        else:
            _validate_robot_config(source, tmp_path)
