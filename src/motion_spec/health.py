# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
# Author: Vamsi Kalagaturu

"""Health checks for installed motion-spec capabilities."""

from __future__ import annotations

import ctypes
import importlib.util
import shutil
import subprocess
import tempfile
from dataclasses import dataclass
from pathlib import Path

PROFILE_IMPORTS = {
    "base": ("click", "rdflib", "rdf_utils"),
    "validation": ("pyshacl",),
    "introspection": ("pyshacl", "rec", "google.protobuf"),
    "dsl": ("textx", "motion_spec_dsl", "coord_dsl", "scene_dsl"),
}
PROFILES = (*PROFILE_IMPORTS, "codegen", "build", "runtime")
# Every generated CMakeLists asks for these, whichever backend it targets.
GENERAL_BUILD_PACKAGES = ("coord2b", "Eigen3", "tomlplusplus")
# A model that publishes, sends a goal or answers one links these; one that talks to nothing
# does not, so a report for a purely offline model can show these missing and still build.
ROS_BUILD_PACKAGES = ("rclcpp", "realtime_tools", "action_msgs", "rclcpp_action")
# `(package, version)`; the generated CMakeLists asks for that version, so a check that ignores
# it passes on an install the build then rejects.
MUJOCO_BUILD_PACKAGES = ("orocos_kdl", "kdl_parser", ("mj_kdl_wrapper", "0.3.6"))
# Reading a ROS message's shape is what turns a declared type into fields, headers and packages.
# rosidl spells its case-conversion helper differently across distros; either will do.
CODEGEN_IMPORTS = ("rosidl_runtime_py",)
CODEGEN_ALTERNATIVES = (("rosidl_pycommon", "rosidl_cmake"),)
ROBIF2B_BUILD_PACKAGES = (
    "robif2b",
    "Eigen3",
    "orocos_kdl",
    "urdfdom_headers",
    "urdfdom",
    "kdl_parser",
    # The gripper and the force-torque sensor are their own drivers; robif2b wraps them, and
    # builds neither wrapper unless it finds them.
    "serial",
    "robotiq_driver_noros",
    "robotiq_ft",
)
# TODO: Check hddc2b only when the generated model selects an HDDC2B base solver.

_WORKSPACE = "$GRC_WS"
# What to do about a missing dependency: the one command that gets it. "Install it" is not an
# instruction, so every dependency this checks names its own source -- an apt package, a
# workspace package, or the flag whose absence left it unbuilt.
_REMEDIES = {
    "stst": f"motion-spec setup --prefix {_WORKSPACE}",
    "cmake": "apt install cmake",
    "c++": "apt install build-essential",
    "Eigen3": "apt install libeigen3-dev",
    "orocos_kdl": "apt install liborocos-kdl-dev",
    "urdfdom": "apt install liburdfdom-dev",
    "urdfdom_headers": "apt install liburdfdom-headers-dev",
    "tomlplusplus": "apt install libtomlplusplus-dev",
    "Protobuf": "apt install libprotobuf-dev protobuf-compiler",
    "rclcpp": "apt install ros-$ROS_DISTRO-rclcpp",
    "realtime_tools": "apt install ros-$ROS_DISTRO-realtime-tools",
    "action_msgs": "apt install ros-$ROS_DISTRO-action-msgs",
    "rclcpp_action": "apt install ros-$ROS_DISTRO-rclcpp-action",
    "rosidl_runtime_py": "source /opt/ros/$ROS_DISTRO/setup.bash",
    "rosidl_pycommon": "source /opt/ros/$ROS_DISTRO/setup.bash",
}
# Everything else is a workspace package: grc_meta lists where each one comes from.
_WORKSPACE_PACKAGES = (
    "coord2b",
    "kdl_parser",
    "mj_kdl_wrapper",
    "robif2b",
    "serial",
    "robotiq_driver_noros",
    "robotiq_ft",
)
# robif2b builds a device wrapper only when told to. Missing here means the flag was off, not
# that the package is absent, so the fix is a rebuild rather than a checkout.
_ROBIF2B_DEVICE_FLAGS = {
    "robif2b::robotiq_gripper": "ENABLE_ROBOTIQ_GRIPPER",
    "robif2b::robotiq_ft_sensor": "ENABLE_ROBOTIQ_FT",
    "robif2b::kinova_gen3": "ENABLE_KORTEX",
}


def _remedy(dependency: str) -> str:
    """The command that gets `dependency`, for a report a reader can act on."""
    if dependency in _REMEDIES:
        return _REMEDIES[dependency]
    if dependency in _WORKSPACE_PACKAGES:
        return (
            f"vcs import src < src/grc_meta/grc_meta.repos && "
            f"colcon build --packages-up-to {dependency}"
        )

    return f"install {dependency} and expose its prefix through CMAKE_PREFIX_PATH"


def _device_remedy(cmake_target: str) -> str:
    """The rebuild that turns a robif2b device wrapper on."""
    flag = _ROBIF2B_DEVICE_FLAGS.get(cmake_target)
    if flag is None:
        return _remedy(cmake_target.partition("::")[0])

    return (
        f"colcon build --packages-select robif2b --cmake-args -D{flag}=ON "
        f"(grc_meta's colcon.meta sets this for the whole workspace)"
    )


@dataclass(frozen=True)
class HealthCheck:
    """One installed capability and its actionable result."""

    profile: str
    dependency: str
    what: str
    path: str | None
    ok: bool
    detail: str


def _module_path(name: str) -> str | None:
    try:
        spec = importlib.util.find_spec(name)
    except ModuleNotFoundError:
        return None
    if spec is None:
        return None
    if spec.origin:
        return spec.origin
    return next(iter(spec.submodule_search_locations or ()), None)


def _cmake_package_path(
    name: str, *, load_target: str | None = None, version: str | None = None
) -> str | None:
    cmake = shutil.which("cmake")
    if not cmake:
        return None
    with tempfile.TemporaryDirectory(prefix="motion-spec-health-") as directory:
        root = Path(directory)
        target_probe = (
            f'file(GENERATE OUTPUT "${{CMAKE_BINARY_DIR}}/target" '
            f'CONTENT "$<TARGET_FILE:{load_target}>")\n'
            if load_target
            else ""
        )
        package_probe = (
            f'file(WRITE "${{CMAKE_BINARY_DIR}}/package" "${{{name}_CONFIG}}")\n'
            f"if(NOT {name}_CONFIG)\n"
            f'  file(WRITE "${{CMAKE_BINARY_DIR}}/package" "${{{name}_DIR}}")\n'
            "endif()\n"
        )
        (root / "CMakeLists.txt").write_text(
            "cmake_minimum_required(VERSION 3.16)\n"
            "project(motion_spec_health LANGUAGES CXX)\n"
            "find_package(Python3 COMPONENTS Interpreter REQUIRED)\n"
            f"find_package({name} {version or ''} REQUIRED)\n"
            f"{package_probe}"
            f"{target_probe}"
        )
        build = root / "build"
        configured = subprocess.run(
            [cmake, "-S", str(root), "-B", str(build)],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            check=False,
        ).returncode
        if configured:
            return None
        if not load_target:
            return (build / "package").read_text().strip() or str(build)
        try:
            target = (build / "target").read_text().strip()
            ctypes.CDLL(target)
        except (OSError, FileNotFoundError):
            return None
        return target


def _named(package) -> tuple[str, str | None]:
    """A build package as `(name, required version)`, whichever way it was written."""
    return package if isinstance(package, tuple) else (package, None)


def check_health(profiles: tuple[str, ...], targets: tuple[str, ...] = ()) -> list[HealthCheck]:
    """Check only the selected installation profiles and target-specific dependencies."""
    selected = PROFILES if "all" in profiles else tuple(dict.fromkeys(("base", *profiles)))
    checks = []
    for profile in selected:
        for module in PROFILE_IMPORTS.get(profile, ()):
            path = _module_path(module)
            checks.append(
                HealthCheck(
                    profile,
                    module,
                    "Python module",
                    path,
                    path is not None,
                    "pip install motion_spec"
                    if profile == "base"
                    else f"motion-spec install {profile}",
                )
            )
        if profile == "introspection":
            # The Python module reads a recorded run; the generated C++ writes one, and links
            # the C++ library to do it.
            path = _cmake_package_path("Protobuf")
            checks.append(
                HealthCheck(
                    profile,
                    "Protobuf",
                    "CMake package",
                    path,
                    path is not None,
                    _remedy("Protobuf"),
                )
            )
    if "codegen" in selected:
        for module in CODEGEN_IMPORTS:
            path = _module_path(module)
            checks.append(
                HealthCheck(
                    "codegen", module, "Python module", path, path is not None, _remedy(module)
                )
            )
        for alternatives in CODEGEN_ALTERNATIVES:
            path = next((found for name in alternatives if (found := _module_path(name))), None)
            checks.append(
                HealthCheck(
                    "codegen",
                    " or ".join(alternatives),
                    "Python module",
                    path,
                    path is not None,
                    _remedy(alternatives[0]),
                )
            )
        from motion_spec.setup import find_stst

        path = find_stst()
        checks.append(
            HealthCheck(
                "codegen", "stst", "executable", path, path is not None, "install STSTv4 on PATH"
            )
        )
    if "build" in selected:
        for executable in ("cmake", "c++"):
            path = shutil.which(executable)
            checks.append(
                HealthCheck(
                    "build",
                    executable,
                    "executable",
                    path,
                    path is not None,
                    f"install {executable} on PATH",
                )
            )
        for package in (*GENERAL_BUILD_PACKAGES, *ROS_BUILD_PACKAGES):
            name, version = _named(package)
            path = _cmake_package_path(name, version=version)
            checks.append(
                HealthCheck("build", name, "CMake package", path, path is not None, _remedy(name))
            )
        for target, packages in (
            ("mujoco", MUJOCO_BUILD_PACKAGES),
            ("robif2b", ROBIF2B_BUILD_PACKAGES),
        ):
            if target not in targets:
                continue
            for package in packages:
                name, version = _named(package)
                path = _cmake_package_path(name, version=version)
                checks.append(
                    HealthCheck(
                        f"build[{target}]",
                        f"{name} {version}" if version else name,
                        "CMake package",
                        path,
                        path is not None,
                        _remedy(name),
                    )
                )
    if "runtime" in selected:
        path = _cmake_package_path("coord2b", load_target="coord2b")
        checks.append(
            HealthCheck(
                "runtime", "coord2b", "shared library", path, path is not None, _remedy("coord2b")
            )
        )
        runtime_targets = {
            "mujoco": (("mj_kdl_wrapper", "mj_kdl_wrapper::mj_kdl_wrapper"),),
            # One per device the robif2b backend drives: the arm, the gripper, and the
            # force-torque sensor are separate libraries, and a model binding any of them
            # links that one.
            "robif2b": (
                ("robif2b", "robif2b::kinova_gen3"),
                ("robif2b", "robif2b::robotiq_gripper"),
                ("robif2b", "robif2b::robotiq_ft_sensor"),
            ),
        }
        for target in targets:
            for package, cmake_target in runtime_targets[target]:
                path = _cmake_package_path(package, load_target=cmake_target)
                checks.append(
                    HealthCheck(
                        f"runtime[{target}]",
                        cmake_target if package != cmake_target else package,
                        "shared library",
                        path,
                        path is not None,
                        _device_remedy(cmake_target),
                    )
                )
    return checks
