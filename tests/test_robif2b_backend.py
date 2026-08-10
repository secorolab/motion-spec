# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""What the robif2b templates promise about hardware they cannot be run against here:
the gripper command scaling, that no address or credential is authored into them, and that
each real-world model still generates a controller that compiles."""

from __future__ import annotations

import re
import shutil
import subprocess
from pathlib import Path

import pytest
from conftest import requires_workspace

TEMPLATES = Path(__file__).resolve().parents[1] / "src" / "motion_spec" / "templates"
ROBIF2B = (TEMPLATES / "backend_robif2b.stg").read_text()


def _gripper_travel() -> float:
    travel = re.search(r'gripper-travel-Robotiq2F85\(\) ::= "([\d.]+)"', ROBIF2B)
    assert travel, "the 2F-85 travel constant is no longer in backend_robif2b.stg"
    return float(travel.group(1))


def test_the_2f85_travel_is_the_devices_stroke() -> None:
    # The 2F-85's driver joint opens over 0.8 rad. Changing this silently re-scales every
    # gripper command, on both routes.
    assert _gripper_travel() == 0.8


def test_a_commanded_joint_position_reaches_each_route_as_that_devices_units(tmp_path) -> None:
    """Compile the shipped helper and check both ends of the stroke.

    An inverted or off-by-one mapping here closes a real gripper on whatever is between its
    fingers, so the endpoints, the middle and the clamp are checked against the real code
    rather than a reimplementation of it.
    """
    compiler = shutil.which("g++") or shutil.which("c++")
    if compiler is None:
        pytest.skip("no C++ compiler")
    helper = re.search(
        r"inline double gripper_closed_fraction.*?\n\}",
        (TEMPLATES / "runtime.stg").read_text(),
        re.S,
    )
    assert helper, "gripper_closed_fraction is no longer in runtime.stg"
    travel = _gripper_travel()
    source = tmp_path / "gripper.cpp"
    source.write_text(
        "#include <algorithm>\n#include <cassert>\n#include <cmath>\n#include <cstdint>\n"
        "#include <stdexcept>\n"
        f"{helper.group(0)}\n"
        f"constexpr double kTravel = {travel};\n"
        "int main() {\n"
        "    assert(gripper_closed_fraction(0.0, kTravel) == 0.0);\n"
        "    assert(gripper_closed_fraction(kTravel, kTravel) == 1.0);\n"
        "    assert(std::fabs(gripper_closed_fraction(0.5 * kTravel, kTravel) - 0.5) < 1e-12);\n"
        "    assert(gripper_closed_fraction(2.0 * kTravel, kTravel) == 1.0);\n"
        "    assert(gripper_closed_fraction(-1.0, kTravel) == 0.0);\n"
        # the interconnect route sends a percentage of travel, the serial route a byte
        "    assert(static_cast<float>(100.0 * gripper_closed_fraction(0.0, kTravel)) == 0.0f);\n"
        "    assert(static_cast<float>(100.0 * gripper_closed_fraction(kTravel, kTravel)) == 100.0f);\n"
        "    assert(static_cast<uint8_t>(std::lround(255.0 * gripper_closed_fraction(0.0, kTravel))) == 0);\n"
        "    assert(static_cast<uint8_t>(std::lround(255.0 * gripper_closed_fraction(kTravel, kTravel))) == 255);\n"
        "    assert(static_cast<uint8_t>(std::lround(255.0 * gripper_closed_fraction(0.5 * kTravel, kTravel))) == 128);\n"
        "    try { gripper_closed_fraction(0.1, 0.0); return 1; } catch (const std::runtime_error &) {}\n"
        "    return 0;\n"
        "}\n"
    )
    binary = tmp_path / "gripper"
    subprocess.run([compiler, "-std=c++20", "-O0", str(source), "-o", str(binary)], check=True)
    subprocess.run([str(binary)], check=True)


def test_no_deployment_detail_is_authored_into_a_template() -> None:
    """Addresses and credentials come from the config file at startup, so a template that
    carries one would connect somewhere the deployment never named."""
    literals = re.compile(r"192\.168|127\.0\.0\.1|ttyUSB|\badmin\b")
    for template in sorted(TEMPLATES.glob("*.stg")):
        found = literals.findall(template.read_text())
        assert not found, f"{template.name} carries deployment literals: {sorted(set(found))}"


MODELS = Path(__file__).parents[2] / "motion-spec-dsl" / "models"
# One per route the gripper can take: its own serial line, or the arm's interconnect.
REAL_WORLD_MODELS = ("real_demo_monitor", "real_demo_hold")

pytestmark = requires_workspace(MODELS)


@pytest.mark.parametrize("name", REAL_WORLD_MODELS)
def test_a_real_world_model_generates_and_compiles(name: str, tmp_path: Path) -> None:
    """The maintained-model gate runs simulated models; hardware cannot be run here, so
    compiling is the strongest check the real path still has -- and the one that catches a
    template rule that exists for mj_kdl and not for robif2b."""
    if shutil.which("cmake") is None:
        pytest.skip("no cmake")
    generation = tmp_path / "gen"
    subprocess.run(
        ["motion-spec", "gen", "code", f"{name}.robmot", "-o", str(generation)],
        cwd=MODELS / name,
        check=True,
    )
    built = next(generation.glob(f"{name}/*"))
    build = subprocess.run(
        ["motion-spec", "build", str(built), "--prefix", str(_install_prefix())],
        capture_output=True,
        text=True,
        check=False,
    )
    if build.returncode and "robif2b" in build.stderr:
        pytest.skip("robif2b is not built in this workspace")
    assert build.returncode == 0, build.stderr
    assert (built / "build" / "main").is_file()


def _install_prefix() -> Path:
    """The colcon install tree the generated project links against."""
    return Path(__file__).parents[3] / "install"
