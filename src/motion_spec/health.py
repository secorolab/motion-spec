# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
# Author: Vamsi Kalagaturu

"""Health checks for installed motion-spec capabilities."""

from __future__ import annotations

import ctypes
import dataclasses
import importlib.util
import json
import os
import shlex
import shutil
import subprocess
import sys
import tempfile
from dataclasses import dataclass
from pathlib import Path

from motion_spec.setup import BUILD_TYPE, COMPONENTS_BY_NAME, MJ_KDL_REF

PROFILE_IMPORTS = {
    # motion_spec_dsl and scene_dsl are imported at module scope by the generation pipeline:
    # hard dependencies in pyproject, not a profile someone opts into.
    "base": (
        "click",
        "rdflib",
        "rdf_utils",
        "jinja2",
        "motion_spec_dsl",
        "scene_dsl",
        "pyshacl",
        "rec",
        "google.protobuf",
    ),
    # Everything Python it needed is required now; what is left is the C++ library below.
    "introspection": (),
    "dsl": ("textx", "coord_dsl"),
}
PROFILES = (*PROFILE_IMPORTS, "codegen", "ros", "build", "runtime")
# Every generated CMakeLists asks for these, whichever backend it targets: the kinematics and
# the frame types are the same on a simulator and on a real arm.
GENERAL_BUILD_PACKAGES = ("coord2b", "Eigen3", "orocos_kdl", "tomlplusplus")
# Only a model that publishes, sends a goal or answers one links these; an installation without
# ROS reports them absent rather than missing, and generates, builds and runs regardless.
ROS_BUILD_PACKAGES = ("rclcpp", "realtime_tools", "action_msgs", "rclcpp_action")


def mujoco_build_packages() -> tuple[tuple[str, str], ...]:
    """`(package, version)` for the MuJoCo target, from the manifest this workspace uses.

    The generated CMakeLists asks for that version, so a check ignoring it passes on an
    install the build rejects. Read per call: `[setup] repos` can put a different pin in
    force, and a value frozen at import would answer for the shipped one instead.
    """
    from motion_spec.setup import manifest_in_force

    declared = _configured_repos()
    pinned = manifest_in_force(declared).get("mj_kdl_wrapper")
    return (("mj_kdl_wrapper", (pinned.version if pinned else MJ_KDL_REF).lstrip("v")),)


def _configured_repos() -> Path | None:
    """The manifest `[setup] repos` names, if a workspace config names one."""
    from motion_spec.config import settings

    try:
        configured, _ = settings()
    except ValueError:
        return None
    declared = configured.get("setup", {}).get("repos")
    return Path(declared) if declared else None


# Reading a ROS message's shape is what turns a declared type into fields, headers and packages.
# rosidl spells its case-conversion helper differently across distros; either will do.
# ament_index_python resolves a scene asset that names a package rather than a path, so a
# generation reaches for it long before anything ROS-shaped appears in the model.
# ament_package: ament's cmake scripts import it, so find_package(rclcpp) needs it too.
# yaml: rosidl_runtime_py imports it, and apt supplies it, not /opt/ros -- so it is the one
# ROS dependency a venv can miss while every module above resolves.
ROS_IMPORTS = ("rosidl_runtime_py", "ament_index_python", "ament_package", "yaml")
ROS_ALTERNATIVES = (("rosidl_pycommon", "rosidl_cmake"),)
# `stst` is a Java program built by ant, and `protoc` compiles the frame-log schema every
# generation carries: the generator shells out to all three.
CODEGEN_EXECUTABLES = ("java", "ant", "protoc")
ROBIF2B_BUILD_PACKAGES = ("robif2b", "urdfdom_headers", "urdfdom", "serial", "robotiq_driver_noros")
# Present only when the workspace was built with that device wrapper enabled. A model that
# binds none of them builds and runs regardless, so a miss here is a note, not a failure.
OPTIONAL_BUILD_PACKAGES = frozenset({"serial", "robotiq_driver_noros"})
# TODO: Check hddc2b only when the generated model selects an HDDC2B base solver.

ROS_ROOT = Path("/opt/ros")
# What motion-spec reads, in the order a reader cares about it.
ENVIRONMENT_VARIABLES = (
    "MOTION_SPEC_WS",
    "MOTION_SPEC_GEN",
    "MOTION_SPEC_ENV",
    "MOTION_SPEC_PREFIX",
    "MOTION_SPEC_BUILD_TYPE",
    "MOTION_SPEC_JOURNAL",
    "ROS_DISTRO",
    "ROS_VERSION",
    "CMAKE_PREFIX_PATH",
    "LD_LIBRARY_PATH",
)
# Unset is no warning where motion-spec falls back to a value of its own.
ENVIRONMENT_DEFAULTS = {"MOTION_SPEC_BUILD_TYPE": BUILD_TYPE}
# The remedies apt answers, which the report gathers into one install line.
APT_REMEDY = "apt install "
# What to do about a missing dependency: the one command that gets it. "Install it" is not an
# instruction, so every dependency this checks names its own source -- an apt package, a
# workspace package, or the flag whose absence left it unbuilt.
_REMEDIES = {
    "stst": "motion-spec setup stst",
    "cmake": "apt install cmake",
    "c++": "apt install build-essential",
    "Eigen3": "apt install libeigen3-dev",
    "java": "apt install default-jdk",
    "ant": "apt install ant",
    "protoc": "apt install protobuf-compiler",
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
    "ament_index_python": "source /opt/ros/$ROS_DISTRO/setup.bash",
    "ament_package": "source /opt/ros/$ROS_DISTRO/setup.bash",
    "yaml": "uv venv --python /usr/bin/python3 --system-site-packages <venv>",
}
# Not installed by `setup`: only a model that binds them needs them.
_DEVICE_PACKAGES = ("robif2b", "serial", "robotiq_driver_noros")
# robif2b builds a device wrapper only when told to. Missing here means the flag was off, not
# that the package is absent, so the fix is a rebuild rather than a checkout.
_ROBIF2B_DEVICE_FLAGS = {
    "robif2b::robotiq_gripper": "ENABLE_ROBOTIQ_GRIPPER",
    "robif2b::robotiq_ft_sensor": "ENABLE_ROBOTIQ_FT",
    "robif2b::kinova_gen3": "ENABLE_KORTEX",
}
_PREFIX = "$MOTION_SPEC_PREFIX"


def system_site_packages(env: dict[str, str] | None = None) -> bool:
    """Whether the distribution's own Python packages are reachable.

    Asked of a module rather than of pyvenv.cfg: a venv built on an interpreter that is not
    the system one sets `include-system-site-packages` and still reaches nothing from apt.
    """
    return _module_path("yaml", env) is not None


def installed_ros_distros() -> list[str]:
    """Every ROS distribution installed under /opt/ros, whether or not one is sourced."""
    if not ROS_ROOT.is_dir():
        return []
    return sorted(entry.name for entry in ROS_ROOT.iterdir() if (entry / "setup.bash").is_file())


def active_ros_distro(env: dict[str, str] | None = None) -> str | None:
    """The distribution this environment is sourced against, when it is installed."""
    distro = (env if env is not None else os.environ).get("ROS_DISTRO")
    return distro if distro and (ROS_ROOT / distro / "setup.bash").is_file() else None


def _ros_distro(env: dict[str, str] | None = None) -> str | None:
    """The distribution a remedy can name: the sourced one, the configured one, or the only one."""
    active = active_ros_distro(env)
    if active:
        return active
    declared = _configured_distro()
    if declared:
        return declared
    installed = installed_ros_distros()
    return installed[0] if len(installed) == 1 else None


def _configured_distro() -> str | None:
    """The distribution the workspace's config names, when it names one."""
    from motion_spec.config import settings

    try:
        configured, _ = settings()
    except (OSError, ValueError):
        return None
    return configured.get("ros", {}).get("distro") or None


def environment_values(env: dict[str, str] | None = None) -> dict[str, str | None]:
    """What each variable motion-spec reads is set to, in the environment being reported on."""
    source = env if env is not None else os.environ
    return {name: source.get(name) or None for name in ENVIRONMENT_VARIABLES}


def ros_summary(env: dict[str, str] | None = None) -> str:
    """What this machine has to say about ROS, for the line above the ROS checks.

    `ABSENT` alone cannot say whether ROS is missing or merely unsourced.
    """
    installed = installed_ros_distros()
    active = active_ros_distro(env)
    if not installed:
        return "no distribution under /opt/ros"
    if active:
        release = (env if env is not None else os.environ).get("ROS_VERSION")
        others = [name for name in installed if name != active]
        sourced = f"{active} sourced (ROS {release})" if release else f"{active} sourced"
        return f"{sourced}; {', '.join(others)} also installed" if others else sourced
    if len(installed) == 1:
        return f"{installed[0]} installed, not sourced: source /opt/ros/{installed[0]}/setup.bash"
    return (
        f"{', '.join(installed)} installed, none sourced: "
        f"source /opt/ros/<distro>/setup.bash for the one this workspace builds against"
    )


def _remedy(dependency: str, env: dict[str, str] | None = None) -> str:
    """The command that gets `dependency`, for a report a reader can act on."""
    if dependency in _REMEDIES:
        # `$ROS_DISTRO` is only an instruction when a shell already set it.
        return _REMEDIES[dependency].replace("$ROS_DISTRO", _ros_distro(env) or "$ROS_DISTRO")
    if dependency in COMPONENTS_BY_NAME:
        return f"motion-spec setup {dependency}"

    return f"install {dependency} and expose its prefix through CMAKE_PREFIX_PATH"


def _cmake_build(package: str, *options: str) -> str:
    """Configure, build and install one source checkout into the motion-spec prefix."""
    flags = "".join(f" -D{option}" for option in options)
    return (
        f"cmake -S <{package} checkout> -B build/{package} "
        f"-DCMAKE_INSTALL_PREFIX={_PREFIX} -DCMAKE_PREFIX_PATH={_PREFIX}{flags} && "
        f"cmake --build build/{package} --target install"
    )


def _device_remedy(cmake_target: str) -> str:
    """The rebuild that turns a robif2b device wrapper on."""
    flag = _ROBIF2B_DEVICE_FLAGS.get(cmake_target)
    if flag is None:
        return _remedy(cmake_target.partition("::")[0])

    return f"motion-spec setup robif2b --force --cmake-arg -D{flag}=ON"


def _by_hand(dependency: str) -> str:
    """The same install without motion-spec, for a reader who would rather run it themselves."""
    flag = _ROBIF2B_DEVICE_FLAGS.get(dependency)
    if flag:
        return _cmake_build("robif2b", "ENABLE_INSTALL_TARGETS=ON", f"{flag}=ON")
    name = dependency.partition("::")[0]
    if name in COMPONENTS_BY_NAME:
        return _cmake_build(name, *_installed_options(name))
    return ""


def _installed_options(dependency: str) -> tuple[str, ...]:
    return tuple(option.lstrip("-D") for option in COMPONENTS_BY_NAME[dependency].options)


@dataclass(frozen=True)
class HealthCheck:
    """One installed capability and its actionable result."""

    profile: str
    dependency: str
    what: str
    path: str | None
    ok: bool
    detail: str
    # A failure a reader can act on says all three: the job the dependency does, where it
    # lives, and how it arrives. Filled from DETAILS for every dependency it names.
    why: str = ""
    source: str = ""
    # The same install without motion-spec, for a reader who would rather do it themselves.
    alternative: str = ""
    # Absent by choice rather than by mistake: reported, but not counted against the install.
    optional: bool = False


# What each dependency is for and where it comes from -- the words every "not found" carries,
# here once, read by the CLI's boxes and the dashboard's health page alike.
DETAILS: dict[str, dict[str, str]] = {
    "click": {"why": "the CLI itself runs on it", "source": "https://github.com/pallets/click"},
    "rdflib": {
        "why": "every model is an RDF graph; parsing, querying and serializing run on it",
        "source": "https://github.com/RDFLib/rdflib",
    },
    "rdf_utils": {
        "why": "shared RDF loaders, resolvers and vocabularies every secorolab tool uses",
        "source": "https://github.com/minhnh/rdf-utils",
    },
    "pyshacl": {
        "why": "validates a generated model graph against the published SHACL shapes",
        "source": "https://github.com/RDFLib/pySHACL",
    },
    "rec": {
        "why": "the archive contract: what a recorded run keeps and how it is read back",
        "source": "https://github.com/secorolab/rec",
    },
    "google.protobuf": {
        "why": "decodes the frame log every run writes",
        "source": "https://github.com/protocolbuffers/protobuf",
    },
    "jinja2": {
        "why": "renders the scene's KDL headers from the shipped templates",
        "source": "https://github.com/pallets/jinja",
    },
    "java": {
        "why": "runs stst, the StringTemplate engine the C++ generator drives",
        "source": "https://openjdk.org",
    },
    "ant": {
        "why": "builds STSTv4 from source when `motion-spec setup` installs it",
        "source": "https://ant.apache.org",
    },
    "protoc": {
        "why": "compiles the frame-log schema into the C++ every recorded run links",
        "source": "https://github.com/protocolbuffers/protobuf",
    },
    "textx": {
        "why": "parses the DSL grammars; every .robmot compile starts in it",
        "source": "https://github.com/textX/textX",
    },
    "motion_spec_dsl": {
        "why": "compiles .robmot models into the RDF graphs every later stage reads",
        "source": "https://github.com/secorolab/motion-spec-dsl",
    },
    "coord_dsl": {
        "why": "compiles .fsm coordination models into the FSM the runtime dispatches",
        "source": "https://github.com/secorolab/coord-dsl",
    },
    "scene_dsl": {
        "why": "compiles .scenex/.ktree scenes into the kinematic tree and simulator assets",
        "source": "https://github.com/secorolab/scene-dsl",
    },
    "rosidl_runtime_py": {
        "why": "reads a ROS message's shape, turning declared types into fields and headers",
        "source": "https://github.com/ros2/rosidl_runtime_py",
    },
    "ament_package": {
        "why": "ament's cmake scripts import it, so find_package(rclcpp) fails without it",
        "source": "https://github.com/ament/ament_package",
    },
    "yaml": {
        "why": "rosidl_runtime_py reads message shapes through it; apt supplies it, not ROS",
        "source": "https://pyyaml.org",
    },
    "stst": {
        "why": "renders the generated C++ from the packaged StringTemplate groups",
        "source": "https://github.com/jsnyders/STSTv4",
    },
    "cmake": {"why": "configures every generated controller build", "source": "https://cmake.org"},
    "c++": {"why": "compiles the generated controller", "source": "https://gcc.gnu.org"},
    "Protobuf": {
        "why": "the generated runtime links it to write the frame log",
        "source": "https://github.com/protocolbuffers/protobuf",
    },
    "coord2b": {
        "why": "the FSM event loop the generated controller links and dispatches through",
        "source": "https://github.com/secorolab/coord2b",
    },
    "Eigen3": {
        "why": "the linear algebra under KDL's kinematics",
        "source": "https://gitlab.com/libeigen/eigen",
    },
    "tomlplusplus": {
        "why": "reads the deployment's robot.toml at controller startup",
        "source": "https://github.com/marzer/tomlplusplus",
    },
    "orocos_kdl": {
        "why": "chains, solvers and frames: the kinematics the generated control math runs on",
        "source": "https://github.com/orocos/orocos_kinematics_dynamics",
    },
    "mj_kdl_wrapper": {
        "why": "the MuJoCo simulation the generated controller drives, and its camera publisher",
        "source": "https://github.com/vamsikalagaturu/mj_kdl_wrapper",
    },
    "rclcpp": {
        "why": "the ROS node a model with a ros block publishes and serves through",
        "source": "https://github.com/ros2/rclcpp",
    },
    "realtime_tools": {
        "why": "lock-free publishers, so the control loop hands off messages without blocking",
        "source": "https://github.com/ros-controls/realtime_tools",
    },
    "action_msgs": {
        "why": "the goal/result plumbing under every served behaviour action",
        "source": "https://github.com/ros2/rcl_interfaces",
    },
    "rclcpp_action": {
        "why": "serves the behaviour action a coordinator sends goals to",
        "source": "https://github.com/ros2/rclcpp",
    },
    "rosidl_pycommon": {
        "why": "reads a ROS message's shape, turning declared types into fields and headers",
        "source": "https://github.com/ros2/rosidl",
    },
    "robif2b": {
        "why": "the real-robot hardware drivers the robif2b backend generates against",
        "source": "https://github.com/secorolab/robif2b",
    },
    "urdfdom": {
        "why": "parses the robot's URDF for the real-platform chain",
        "source": "https://github.com/ros/urdfdom",
    },
    "urdfdom_headers": {
        "why": "parses the robot's URDF for the real-platform chain",
        "source": "https://github.com/ros/urdfdom_headers",
    },
    "serial": {
        "why": "the serial line the Robotiq devices are driven over",
        "source": "https://github.com/wjwwood/serial",
    },
    "robotiq_driver_noros": {
        "why": "drives the Robotiq gripper and force-torque sensor on a real platform",
        "source": "https://github.com/secorolab/robotiq_driver_noros",
    },
}


def _enrich(check: HealthCheck) -> HealthCheck:
    """The check, carrying its dependency's why and source when DETAILS knows them.

    A dependency may be named with a version ("mj_kdl_wrapper 0.3.11"), as a cmake target
    ("robif2b::kinova_gen3") or as alternatives ("rosidl_pycommon or rosidl_cmake"); the
    details belong to the bare name either way.
    """
    bare = check.dependency.partition(" or ")[0].split()[0]
    name = bare.partition("::")[0]
    spec = DETAILS.get(check.dependency) or DETAILS.get(name) or {}
    return dataclasses.replace(
        check,
        why=spec.get("why", check.why),
        source=spec.get("source", check.source),
        alternative=check.alternative or _by_hand(bare),
    )


_PROBE = """
import importlib.util, json, sys

found = {}
for name in sys.argv[1:]:
    try:
        spec = importlib.util.find_spec(name)
    except (ImportError, ValueError):
        spec = None
    origin = None
    if spec is not None:
        origin = spec.origin or next(iter(spec.submodule_search_locations or ()), None)
    found[name] = origin
print(json.dumps(found))
"""


def _module_path(name: str, env: dict[str, str] | None = None) -> str | None:
    if env is not None:
        return _module_paths((name,), env)[name]
    try:
        spec = importlib.util.find_spec(name)
    except ModuleNotFoundError:
        return None
    if spec is None:
        return None
    if spec.origin:
        return spec.origin
    return next(iter(spec.submodule_search_locations or ()), None)


def _module_paths(names: tuple[str, ...], env: dict[str, str]) -> dict[str, str | None]:
    """Where each module resolves under ENV, asked of an interpreter started in it.

    A sourced ROS distribution puts its Python packages on the path through the environment,
    so importing them here would answer for this process instead of for the one the build
    and the run will use. One subprocess answers for all of them.
    """
    done = subprocess.run(
        [sys.executable, "-c", _PROBE, *names],
        env=env,
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
        check=False,
        text=True,
    )
    if done.returncode:
        return dict.fromkeys(names)
    try:
        return json.loads(done.stdout)
    except json.JSONDecodeError:
        return dict.fromkeys(names)


def _module_what(path: str | None) -> str:
    """`Python module`, marked editable when it imports from a source tree.

    A pip editable install and a checkout on PYTHONPATH both resolve outside any
    site-packages, and both mean the same thing to a reader: edits here take effect.
    The distribution's own `direct_url.json` names only the first of the two.
    """
    if path is None:
        return "Python module"
    parts = Path(path).parts
    installed = "site-packages" in parts or "dist-packages" in parts
    return "Python module" if installed else "Python module, editable"


def _stst_jar(launcher: str) -> bool:
    """Whether the launcher still has the jar it runs; one without it renders nothing."""
    try:
        text = Path(launcher).read_text(errors="replace")
    except OSError:
        return False
    home = next(
        (line.partition("=")[2] for line in text.splitlines() if line.startswith("STST_HOME=")),
        None,
    )
    if home is None:
        return True  # not a launcher this tool wrote, so not its place to judge
    return (Path(shlex.split(home)[0]) / "build" / "jar" / "stst.jar").is_file()


def _which(executable: str, env: dict[str, str] | None = None) -> str | None:
    """`executable` on PATH -- the one ENV gives, when the caller named an environment."""
    if env and "PATH" in env:
        return shutil.which(executable, path=env["PATH"])
    return shutil.which(executable)


def _cmake_package_path(
    name: str,
    *,
    load_target: str | None = None,
    version: str | None = None,
    env: dict[str, str] | None = None,
) -> str | None:
    cmake = _which("cmake", env)
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
            env=env,
        ).returncode
        if configured:
            return None
        if not load_target:
            return (build / "package").read_text().strip() or str(build)
        try:
            target = (build / "target").read_text().strip()
        except (OSError, FileNotFoundError):
            return None
        if env is not None:
            # ENV's LD_LIBRARY_PATH decides this, not ours.
            loaded = subprocess.run(
                [sys.executable, "-c", "import ctypes, sys; ctypes.CDLL(sys.argv[1])", target],
                env=env,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                check=False,
            )
            return target if loaded.returncode == 0 else None
        try:
            ctypes.CDLL(target)
        except OSError:
            return None
        return target


# What a target needs before it can be generated, built and run. ROS and introspection are
# left out: whether a model needs them is a property of the model, not of the target.
VERDICT_PROFILES = ("base", "dsl", "codegen", "build", "runtime")
VERDICTS = {"mujoco": "sim", "robif2b": "real"}


def verdicts(checks: list[HealthCheck]) -> dict[str, list[str]]:
    """Per target, the dependencies still missing before it can run. Empty means ready.

    A target is absent from the result unless every profile it needs was checked: a partial
    run cannot say "ready" about probes it never made.
    """
    profiles = {check.profile for check in checks}
    answer = {}
    for target, name in VERDICTS.items():
        wanted = {*VERDICT_PROFILES, f"build[{target}]", f"runtime[{target}]"}
        if wanted - profiles:
            continue
        answer[name] = [
            check.dependency
            for check in checks
            if check.profile in wanted and not check.ok and not check.optional
        ]
    return answer


def apt_packages(checks: list[HealthCheck]) -> list[str]:
    """The apt packages that would fix every missing check, in the order they were reported.

    A remedy per row is a row-by-row installation: eight of them are eight apt invocations
    to assemble by hand. The ones apt provides are the same command, so they are collected
    into one, and each stays named where it was reported.
    """
    packages: list[str] = []
    for check in checks:
        if check.ok or check.optional or not check.detail.startswith(APT_REMEDY):
            continue
        for package in check.detail[len(APT_REMEDY) :].split():
            if package not in packages:
                packages.append(package)
    return packages


def _named(package) -> tuple[str, str | None]:
    """A build package as `(name, required version)`, whichever way it was written."""
    return package if isinstance(package, tuple) else (package, None)


def check_health(
    profiles: tuple[str, ...],
    targets: tuple[str, ...] = (),
    env: dict[str, str] | None = None,
    on_progress=None,
) -> list[HealthCheck]:
    """Check only the selected installation profiles and target-specific dependencies.

    ENV is the environment to check under; without it every probe answers for this process.
    ON_PROGRESS is called with (done, dependency) before each probe: configuring a CMake
    project takes seconds, and several of these do.
    """
    selected = PROFILES if "all" in profiles else tuple(dict.fromkeys(("base", *profiles)))
    checks = []

    def announce(dependency: str) -> None:
        if on_progress:
            on_progress(len(checks), dependency)

    for profile in selected:
        for module in PROFILE_IMPORTS.get(profile, ()):
            announce(module)
            path = _module_path(module, env)
            checks.append(
                HealthCheck(
                    profile,
                    module,
                    _module_what(path),
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
            announce("Protobuf")
            path = _cmake_package_path("Protobuf", env=env)
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
        from motion_spec.setup import find_stst

        announce("stst")
        path = (
            find_stst(path=env.get("PATH", ""), workspace=env.get("MOTION_SPEC_WS"))
            if env is not None
            else find_stst()
        )
        runs = path is not None and _stst_jar(path)
        checks.append(
            HealthCheck(
                "codegen",
                "stst",
                "executable" if runs else "executable, jar missing",
                path,
                runs,
                "motion-spec setup stst --force" if path else _remedy("stst"),
            )
        )
        for executable in CODEGEN_EXECUTABLES:
            announce(executable)
            path = _which(executable, env)
            checks.append(
                HealthCheck(
                    "codegen", executable, "executable", path, path is not None, _remedy(executable)
                )
            )
    if "ros" in selected:
        # First: every ROS row below is a consequence of it.
        distro = active_ros_distro(env)
        installed = installed_ros_distros()
        checks.append(
            HealthCheck(
                "ros",
                "ROS distribution",
                "sourced" if distro else "installed" if installed else "not found",
                str(ROS_ROOT / distro)
                if distro
                else ", ".join(str(ROS_ROOT / name) for name in installed) or None,
                distro is not None,
                ros_summary(env),
                optional=True,
            )
        )
        for module in ROS_IMPORTS:
            announce(module)
            path = _module_path(module, env)
            checks.append(
                HealthCheck(
                    "ros",
                    module,
                    _module_what(path),
                    path,
                    path is not None,
                    _remedy(module, env),
                    optional=True,
                )
            )
        for alternatives in ROS_ALTERNATIVES:
            announce(alternatives[0])
            path = next(
                (found for name in alternatives if (found := _module_path(name, env))), None
            )
            checks.append(
                HealthCheck(
                    "ros",
                    " or ".join(alternatives),
                    _module_what(path),
                    path,
                    path is not None,
                    _remedy(alternatives[0], env),
                    optional=True,
                )
            )
        for package in ROS_BUILD_PACKAGES:
            name, version = _named(package)
            announce(name)
            path = _cmake_package_path(name, version=version, env=env)
            checks.append(
                HealthCheck(
                    "ros",
                    name,
                    "CMake package",
                    path,
                    path is not None,
                    _remedy(name, env),
                    optional=True,
                )
            )
    if "build" in selected:
        for executable in ("cmake", "c++"):
            announce(executable)
            path = _which(executable, env)
            checks.append(
                HealthCheck(
                    "build", executable, "executable", path, path is not None, _remedy(executable)
                )
            )
        for package in GENERAL_BUILD_PACKAGES:
            name, version = _named(package)
            announce(name)
            path = _cmake_package_path(name, version=version, env=env)
            checks.append(
                HealthCheck("build", name, "CMake package", path, path is not None, _remedy(name))
            )
        for target, packages in (
            ("mujoco", mujoco_build_packages()),
            ("robif2b", ROBIF2B_BUILD_PACKAGES),
        ):
            if target not in targets:
                continue
            for package in packages:
                name, version = _named(package)
                announce(name)
                path = _cmake_package_path(name, version=version, env=env)
                checks.append(
                    HealthCheck(
                        f"build[{target}]",
                        f"{name} {version}" if version else name,
                        "CMake package",
                        path,
                        path is not None,
                        _remedy(name),
                        optional=name in OPTIONAL_BUILD_PACKAGES,
                    )
                )
    if "runtime" in selected:
        announce("coord2b")
        path = _cmake_package_path("coord2b", load_target="coord2b", env=env)
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
                announce(cmake_target)
                path = _cmake_package_path(package, load_target=cmake_target, env=env)
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
    return [_enrich(check) for check in checks]
