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
GENERAL_BUILD_PACKAGES = ("coord2b",)
MUJOCO_BUILD_PACKAGES = ("orocos_kdl", "kdl_parser", "mj_kdl_wrapper")
ROBIF2B_BUILD_PACKAGES = (
    "robif2b",
    "Eigen3",
    "orocos_kdl",
    "urdfdom_headers",
    "urdfdom",
    "kdl_parser",
)
# TODO: Check hddc2b only when the generated model selects an HDDC2B base solver.


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


def _cmake_package_path(name: str, *, load_target: str | None = None) -> str | None:
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
            f"find_package({name} REQUIRED)\n"
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
    if "codegen" in selected:
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
        for package in GENERAL_BUILD_PACKAGES:
            path = _cmake_package_path(package)
            checks.append(
                HealthCheck(
                    "build",
                    package,
                    "CMake package",
                    path,
                    path is not None,
                    "install it and expose its prefix through CMAKE_PREFIX_PATH",
                )
            )
        for target, packages in (
            ("mujoco", MUJOCO_BUILD_PACKAGES),
            ("robif2b", ROBIF2B_BUILD_PACKAGES),
        ):
            if target not in targets:
                continue
            for package in packages:
                path = _cmake_package_path(package)
                checks.append(
                    HealthCheck(
                        f"build[{target}]",
                        package,
                        "CMake package",
                        path,
                        path is not None,
                        "install it and expose its prefix through CMAKE_PREFIX_PATH",
                    )
                )
    if "runtime" in selected:
        path = _cmake_package_path("coord2b", load_target="coord2b")
        checks.append(
            HealthCheck(
                "runtime",
                "coord2b",
                "shared library",
                path,
                path is not None,
                "install coord2b and expose its runtime libraries",
            )
        )
        runtime_targets = {
            "mujoco": (("mj_kdl_wrapper", "mj_kdl_wrapper::mj_kdl_wrapper"),),
            "robif2b": (("robif2b", "robif2b::kinova_gen3"),),
        }
        for target in targets:
            for package, cmake_target in runtime_targets[target]:
                path = _cmake_package_path(package, load_target=cmake_target)
                checks.append(
                    HealthCheck(
                        f"runtime[{target}]",
                        package,
                        "shared library",
                        path,
                        path is not None,
                        f"install {package} and expose its runtime libraries",
                    )
                )
    return checks
