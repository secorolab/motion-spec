# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""What the robif2b templates promise about hardware they cannot be run against here: the
gripper command scaling, and one writer per sensor reading."""

from __future__ import annotations

import json
import re
import shutil
import subprocess
from pathlib import Path

import pytest
from support import EXAMPLES

from motion_spec.generation.pipeline import generate_model
from motion_spec.setup import find_stst

TEMPLATES = Path(__file__).resolve().parents[1] / "src" / "motion_spec" / "templates"


def test_a_commanded_joint_position_reaches_each_route_as_that_devices_units(tmp_path) -> None:
    """Compile the shipped helper and check both ends of the stroke.

    An inverted or off-by-one mapping here closes a real gripper on whatever is between its
    fingers, so the endpoints, the middle and the clamp are checked against the real code
    rather than a reimplementation of it.
    """
    robif2b = (TEMPLATES / "backend" / "robif2b" / "robot.stg").read_text()
    travel = re.search(r'gripper-travel-Robotiq2F85\(\) ::= "([\d.]+)"', robif2b)
    assert travel, "the 2F-85 travel constant is no longer in backend/robif2b/robot.stg"
    # The 2F-85's driver joint opens over 0.8 rad; another value re-scales every gripper command.
    assert float(travel.group(1)) == 0.8
    compiler = shutil.which("g++") or shutil.which("c++")
    if compiler is None:
        pytest.skip("no C++ compiler")
    helper = re.search(r"inline double gripper_closed_fraction.*?\n\}", robif2b, re.DOTALL)
    assert helper, "gripper_closed_fraction is no longer in backend/robif2b/robot.stg"
    source = tmp_path / "gripper.cpp"
    source.write_text(
        "#include <algorithm>\n#include <cassert>\n#include <cmath>\n#include <cstdint>\n"
        "#include <stdexcept>\n"
        f"{helper.group(0)}\n"
        f"constexpr double kTravel = {travel.group(1)};\n"
        "int main() {\n"
        "    assert(gripper_closed_fraction(0.0, kTravel) == 0.0);\n"
        "    assert(gripper_closed_fraction(kTravel, kTravel) == 1.0);\n"
        "    assert(std::fabs(gripper_closed_fraction(0.5 * kTravel, kTravel) - 0.5) < 1e-12);\n"
        "    assert(gripper_closed_fraction(2.0 * kTravel, kTravel) == 1.0);\n"
        "    assert(gripper_closed_fraction(-1.0, kTravel) == 0.0);\n"
        # the interconnect route sends a percentage of travel, the serial route a byte
        "    assert(static_cast<float>(100.0 * gripper_closed_fraction(kTravel, kTravel)) == 100.0f);\n"
        "    assert(static_cast<uint8_t>(std::lround(255.0 * gripper_closed_fraction(kTravel, kTravel))) == 255);\n"
        "    assert(static_cast<uint8_t>(std::lround(255.0 * gripper_closed_fraction(0.5 * kTravel, kTravel))) == 128);\n"
        "    try { gripper_closed_fraction(0.1, 0.0); return 1; } catch (const std::runtime_error &) {}\n"
        "    return 0;\n"
        "}\n"
    )
    binary = tmp_path / "gripper"
    subprocess.run([compiler, "-std=c++20", "-O0", str(source), "-o", str(binary)], check=True)
    subprocess.run([str(binary)], check=True)


@pytest.mark.skipif(find_stst() is None, reason="no stst; run `motion-spec setup`")
@pytest.mark.parametrize(
    ("name", "ft_required_by_motion"),
    [("real_gripper_cycle_ft_monitoring", [0, 1]), ("real_arm_pose_hold", [])],
)
def test_a_sensor_reading_has_one_writer_and_only_its_readers_need_it(
    name: str, ft_required_by_motion: list[int], tmp_path: Path
) -> None:
    """A second writer of a sensor reading neither fails to compile nor fails a run: it silently
    replaces the reading -- one such copy left the external wrench at zero for whole runs. And
    only motions whose computation reads the FT wrench depend on FT health."""
    generated = generate_model(EXAMPLES[name] / f"{name}.robmot", tmp_path)
    ir = json.loads((generated / "model" / "ir.json").read_text())
    telemetry = ir["communication"]["telemetry"]
    readings = [
        row["id"]
        for row in telemetry["spatial_samples"]["wrenches"]
        if ir["computation"]["data_access"][row["id"]]["write"]["kind"] == "sensor"
    ]
    assert readings, "the real-world models carry an FT sensor; this checks nothing without one"
    for header in (generated / "controller" / "headers").glob("motion_*.hpp"):
        text = header.read_text()
        for reading in readings:
            assert f"shared.{reading} =" not in text, f"{header.name} writes '{reading}'"
    (ft,) = [
        device
        for solver in ir["resources"]["by_kind"]["serial_chain"]
        if solver["runtime"]["owner"]
        for device in solver["devices"]
        if device["kind"] == "RobotiqFT300s"
    ]
    assert ft["required_by_motion"] == ft_required_by_motion
