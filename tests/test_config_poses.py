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

from motion_spec.generation.codegen import CONFIG_CHECK, render_template
from motion_spec.rdf_parser.deployment import (
    AGENT_HOME_KEY,
    CONFIG_POSE_FIELDS,
    config_poses,
    platform_facts,
)
from motion_spec.rdf_parser.model import Model
from motion_spec.runs.runner import RunnerError, validate_robot_config
from motion_spec.setup import find_stst

APP = "https://secorolab.github.io/models/demo/"
ROBIF2B = "backend/robif2b/main"

# A pose the model reads from the execution context's config, at `poses.table`.
CONFIG_POSE = f"""
@prefix app: <{APP}> .
@prefix exec: <{EXEC}> .
@prefix prov: <http://www.w3.org/ns/prov#> .
app:demo-exec a exec:ExecutionContext ; prov:wasDerivedFrom app:demo-exec.config .
app:demo-exec.config a prov:Entity ; prov:atLocation <file:///somewhere/robot.toml> .
<{APP}shared/spec/look-at-table> prov:wasDerivedFrom app:demo-exec.config ;
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
    model = Model(graph=graph, namespaces=(f"{APP}shared/spec/",))
    with pytest.raises(ConstraintViolation, match=message):
        config_poses(model, config)


def test_the_platform_is_read_from_the_model_with_the_config_it_states() -> None:
    graph = Dataset(default_union=True)
    graph.default_graph.parse(
        data=f"""
        @prefix app: <{APP}> .
        @prefix exec: <{EXEC}> .
        @prefix prov: <http://www.w3.org/ns/prov#> .
        app:demo-exec a exec:RealWorld ; prov:wasDerivedFrom app:demo-exec.config .
        app:demo-exec.config a prov:Entity ; prov:atLocation <file:///somewhere/robot.toml> .
        """,
        format="turtle",
    )
    assert platform_facts(graph) == {
        "uri": f"{APP}demo-exec",
        "name": None,
        "simulated": False,
        "backend": "robif2b",
        "config": "/somewhere/robot.toml",
    }


_ARM_SECTION = (
    "[agents.arm1]\nip='10.0.0.1'\nuser='u'\npassword='p'\nport=1\n"
    "port_real_time=2\nsession_timeout_ms=3\nconnection_timeout_ms=4\n"
)
_FT_SECTION = "[arm1.wrist_ft]\nport='ttyUSB0'\nbaudrate=19200\nslave_address=9\n"
_FT = {"kind": "RobotiqFT300s", "config_key": "arm1.wrist_ft", "drives": "wrist_ft"}
_ARM = {"kind": "KinovaGen3", "config_key": "agents.arm1", "drives": ""}


# The codegen payload a real-hardware program's config check renders from.
CONFIGURATION = {
    "name": "demo",
    "platform": {"simulated": False, "config": "robot.toml"},
    "config_check": {"home_key": AGENT_HOME_KEY, "pose_fields": list(CONFIG_POSE_FIELDS)},
}
ARM_ROBOT = {"id": "arm_solver", "runtime": {"owner": True}}
STST = find_stst()


@pytest.fixture
def source(tmp_path: Path) -> Path:
    """A generation's generated/ tree; each case renders the check its devices bind."""
    (tmp_path / "generated").mkdir()
    return tmp_path / "generated"


@pytest.mark.skipif(STST is None, reason="no stst; run `motion-spec setup`")
def test_robot_config_must_cover_every_bound_device(tmp_path: Path, source: Path) -> None:
    payload = tmp_path / "payload.json"
    payload.write_text(
        json.dumps(
            {
                "configuration": {**CONFIGURATION, "config_poses": []},
                "resources": {"by_kind": {"serial_chain": [{**ARM_ROBOT, "devices": [_ARM, _FT]}]}},
            }
        )
    )
    render_template(STST, ROBIF2B, "config_check", payload, source / "controller" / CONFIG_CHECK)
    config = tmp_path / "robot.toml"

    with pytest.raises(RunnerError, match="robot config not found"):
        validate_robot_config(source, tmp_path)

    config.write_text("[agents.arm1]\nip = '10.0.0.1'\n")
    with pytest.raises(RunnerError, match="is missing user"):
        validate_robot_config(source, tmp_path)

    config.write_text(_ARM_SECTION)
    with pytest.raises(RunnerError, match=r"no \[arm1.wrist_ft\] section for the bound Robotiq"):
        validate_robot_config(source, tmp_path)

    # The FT sensor is a serial device: the arm's network keys say nothing about it.
    config.write_text(_ARM_SECTION + "[arm1.wrist_ft]\nport='ttyUSB0'\n")
    with pytest.raises(RunnerError, match=r"\[arm1.wrist_ft\] is missing baudrate"):
        validate_robot_config(source, tmp_path)

    config.write_text(_ARM_SECTION + _FT_SECTION)
    validate_robot_config(source, tmp_path)


@pytest.mark.skipif(STST is None, reason="no stst; run `motion-spec setup`")
def test_a_pose_the_model_reads_binds_its_section(tmp_path: Path, source: Path) -> None:
    """`config_poses` refuses to generate a model whose pose section is absent, so the run must
    not refuse it for being present -- between them, no real-world model could run at all."""
    payload = tmp_path / "payload.json"
    payload.write_text(
        json.dumps(
            {
                "configuration": {
                    **CONFIGURATION,
                    "config_poses": [{"id": "home", "config_key": "poses.home"}],
                },
                "resources": {"by_kind": {"serial_chain": [{**ARM_ROBOT, "devices": [_ARM]}]}},
            }
        )
    )
    render_template(STST, ROBIF2B, "config_check", payload, source / "controller" / CONFIG_CHECK)
    config = tmp_path / "robot.toml"
    pose = "[poses.home]\nposition = [0.1, 0.2, 0.3]\norientation = [0.0, 0.0, 0.0]\n"

    config.write_text(_ARM_SECTION + pose)
    validate_robot_config(source, tmp_path)

    # The numbers may be retuned without regenerating, so their shape is checked on the way in.
    config.write_text(
        _ARM_SECTION + "[poses.home]\nposition = [0.1, 0.2]\norientation = [0, 0, 0]\n"
    )
    with pytest.raises(RunnerError, match=r"\[poses.home\] states no three-number `position`"):
        validate_robot_config(source, tmp_path)

    # A deployment may keep more poses than the model reads; an unread one is skipped.
    config.write_text(_ARM_SECTION + pose + "[poses.spare]\nposition = [0, 0, 0]\n")
    validate_robot_config(source, tmp_path)


@pytest.mark.skipif(STST is None, reason="no stst; run `motion-spec setup`")
def test_a_home_is_rejected_on_a_real_device(tmp_path: Path, source: Path) -> None:
    """A home stated for a real run is a number the deployment believes in and nothing acts on."""
    payload = tmp_path / "payload.json"
    payload.write_text(
        json.dumps(
            {
                "configuration": {**CONFIGURATION, "config_poses": []},
                "resources": {"by_kind": {"serial_chain": [{**ARM_ROBOT, "devices": [_ARM]}]}},
            }
        )
    )
    render_template(STST, ROBIF2B, "config_check", payload, source / "controller" / CONFIG_CHECK)
    config = tmp_path / "robot.toml"

    config.write_text(_ARM_SECTION + "home = [0.0, 0.6, 3.14, -2.0, 0.0, 1.1, 1.57]\n")
    with pytest.raises(RunnerError, match=r"`home` in \[agents.arm1\]"):
        validate_robot_config(source, tmp_path)

    config.write_text(_ARM_SECTION)
    validate_robot_config(source, tmp_path)


@pytest.mark.parametrize(
    ("devices", "missing"),
    [
        ([{"kind": "KinovaGen3-2F85", "config_key": "agents.arm1", "drives": ""}, _FT], None),
        (
            [_ARM, {"kind": "Robotiq2F85", "config_key": "agents.gripper1", "drives": ""}, _FT],
            r"no \[agents.gripper1\] section",
        ),
    ],
    ids=["interconnect", "separate"],
)
@pytest.mark.skipif(STST is None, reason="no stst; run `motion-spec setup`")
def test_the_authored_device_decides_which_sections_the_config_needs(
    tmp_path: Path, source: Path, devices: list, missing: str | None
) -> None:
    """The gripper's route is authored, not inferred: one section under the arm's device, two
    under separate ones."""
    payload = tmp_path / "payload.json"
    payload.write_text(
        json.dumps(
            {
                "configuration": {**CONFIGURATION, "config_poses": []},
                "resources": {"by_kind": {"serial_chain": [{**ARM_ROBOT, "devices": devices}]}},
            }
        )
    )
    render_template(STST, ROBIF2B, "config_check", payload, source / "controller" / CONFIG_CHECK)
    config = tmp_path / "robot.toml"
    config.write_text(_ARM_SECTION + _FT_SECTION)
    if missing:
        with pytest.raises(RunnerError, match=missing):
            validate_robot_config(source, tmp_path)
    else:
        validate_robot_config(source, tmp_path)
