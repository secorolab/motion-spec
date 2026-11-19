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

from motion_spec.setup import BUILD_TYPE

PROFILE_IMPORTS = {
    # motion_spec_dsl and scene_dsl: hard dependencies of the generation pipeline
    "base": (
        "click",
        "rdflib",
        "rdf_utils",
        "motion_spec_dsl",
        "scene_dsl",
        "pyshacl",
        "rec",
        "google.protobuf",
    ),
    "telemetry": (),
    "dsl": ("textx", "coord_dsl"),
}
PROFILES = (*PROFILE_IMPORTS, "codegen", "ros", "build", "runtime")
GENERAL_BUILD_PACKAGES = ("coord2b", "Eigen3", "orocos_kdl", "tomlplusplus")
ROS_BUILD_PACKAGES = ("rclcpp", "realtime_tools", "action_msgs", "rclcpp_action")


# yaml comes from apt, not /opt/ros: a venv can miss it
ROS_IMPORTS = ("rosidl_runtime_py", "ament_index_python", "ament_package", "yaml")
ROS_ALTERNATIVES = (("rosidl_pycommon", "rosidl_cmake"),)
CODEGEN_EXECUTABLES = ("java", "ant", "protoc")
ROBIF2B_BUILD_PACKAGES = ("robif2b", "urdfdom_headers", "urdfdom", "serial", "robotiq_driver_noros")
# built only with that device wrapper enabled
OPTIONAL_BUILD_PACKAGES = {"serial", "robotiq_driver_noros"}
# TODO: Check hddc2b only when the generated model selects an HDDC2B base solver.

ROS_ROOT = Path("/opt/ros")
ENVIRONMENT_VARIABLES = (
    "MOTION_SPEC_WS",
    "MOTION_SPEC_GEN",
    "MOTION_SPEC_ENV",
    "MOTION_SPEC_PREFIX",
    "MOTION_SPEC_BUILD_TYPE",
    "ROS_DISTRO",
    "ROS_VERSION",
    "CMAKE_PREFIX_PATH",
    "LD_LIBRARY_PATH",
)
ENVIRONMENT_DEFAULTS = {"MOTION_SPEC_BUILD_TYPE": BUILD_TYPE}
APT_PACKAGES = {
    "cmake": ("cmake",),
    "c++": ("build-essential",),
    "Eigen3": ("libeigen3-dev",),
    "java": ("default-jdk",),
    "ant": ("ant",),
    "protoc": ("protobuf-compiler",),
    "urdfdom": ("liburdfdom-dev",),
    "urdfdom_headers": ("liburdfdom-headers-dev",),
    "tomlplusplus": ("libtomlplusplus-dev",),
    "Protobuf": ("libprotobuf-dev", "protobuf-compiler"),
    "glfw3": ("libglfw3-dev",),
    "OpenGL": ("libgl-dev",),
    "EGL": ("libegl-dev",),
    "ffmpeg": ("ffmpeg",),
    "rclcpp": ("ros-$ROS_DISTRO-rclcpp",),
    "realtime_tools": ("ros-$ROS_DISTRO-realtime-tools",),
    "action_msgs": ("ros-$ROS_DISTRO-action-msgs",),
    "rclcpp_action": ("ros-$ROS_DISTRO-rclcpp-action",),
}
_REMEDIES = {
    "rosidl_runtime_py": "source /opt/ros/$ROS_DISTRO/setup.bash",
    "rosidl_pycommon": "source /opt/ros/$ROS_DISTRO/setup.bash",
    "ament_index_python": "source /opt/ros/$ROS_DISTRO/setup.bash",
    "ament_package": "source /opt/ros/$ROS_DISTRO/setup.bash",
    "yaml": "uv venv --python /usr/bin/python3 --system-site-packages <venv>",
}
_PROVIDED_AS = {"STSTv4": "stst", "orocos_kinematics_dynamics": "orocos_kdl"}
SETUP_PROVIDES: dict[str, str] = {}
_DEVICE_PACKAGES: set[str] = set()
# missing means the flag was off: rebuild, not checkout
_ROBIF2B_DEVICE_FLAGS = {
    "robif2b::robotiq_gripper": "ENABLE_ROBOTIQ_GRIPPER",
    "robif2b::robotiq_ft_sensor": "ENABLE_ROBOTIQ_FT",
    "robif2b::kinova_gen3": "ENABLE_KORTEX",
}
_PREFIX = "$MOTION_SPEC_PREFIX"


def installed_ros_distros() -> list[str]:
    """Every ROS distribution installed under /opt/ros, whether or not one is sourced."""
    if not ROS_ROOT.is_dir():
        return []
    return [entry.name for entry in ROS_ROOT.iterdir() if (entry / "setup.bash").is_file()]


def active_ros_distro(env: dict[str, str] | None = None) -> str | None:
    """The distribution this environment is sourced against, when it is installed."""
    distro = (env if env is not None else os.environ).get("ROS_DISTRO")
    return distro if distro and (ROS_ROOT / distro / "setup.bash").is_file() else None


def ros_distro(env: dict[str, str] | None = None) -> str | None:
    """The distribution a remedy can name: the sourced one, the configured one, or the only one."""
    active = active_ros_distro(env)
    if active:
        return active
    from motion_spec.config import configured

    try:
        declared = configured("ros.distro") or None
    except (OSError, ValueError):
        declared = None
    if declared:
        return declared
    installed = installed_ros_distros()
    return installed[0] if len(installed) == 1 else None


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


def remedy(dependency: str, env: dict[str, str] | None = None) -> str:
    """The command that gets `dependency`, for a report a reader can act on."""
    if dependency in APT_PACKAGES:
        packages = " ".join(APT_PACKAGES[dependency])
        return f"apt install {packages}".replace("$ROS_DISTRO", ros_distro(env) or "$ROS_DISTRO")
    if dependency in _REMEDIES:
        return _REMEDIES[dependency].replace("$ROS_DISTRO", ros_distro(env) or "$ROS_DISTRO")
    if dependency in SETUP_PROVIDES:
        real = "--real " if dependency in _DEVICE_PACKAGES else ""
        return f"motion-spec setup {real}{SETUP_PROVIDES[dependency]}"

    return f"install {dependency} and expose its prefix through CMAKE_PREFIX_PATH"


def cmake_build(package: str, *options: str) -> str:
    """Configure, build and install one source checkout into the motion-spec prefix."""
    flags = "".join(f" {option}" for option in options)
    return (
        f"cmake -S <{package} checkout> -B build/{package} "
        f"-DCMAKE_INSTALL_PREFIX={_PREFIX} -DCMAKE_PREFIX_PATH={_PREFIX}{flags} && "
        f"cmake --build build/{package} --target install"
    )


def installed_options(package: str) -> tuple[str, ...]:
    """PACKAGE's cmake arguments from the workspace colcon.meta, else the shipped one."""
    from motion_spec.setup import COLCON_META, cmake_arguments, workspace

    try:
        meta = workspace() / COLCON_META
    except RuntimeError:
        meta = Path(__file__).parent / COLCON_META
    if not meta.is_file():
        meta = Path(__file__).parent / COLCON_META
    return cmake_arguments(meta, package)


@dataclass(frozen=True)
class HealthCheck:
    """One installed capability and its actionable result."""

    profile: str
    dependency: str
    what: str
    path: str | None
    ok: bool
    detail: str
    why: str = ""
    source: str = ""
    alternative: str = ""
    # absent by choice: not counted against the install
    optional: bool = False
    name: str = ""
    apt: tuple[str, ...] = ()


DETAILS: dict[str, dict[str, str]] = {
    "click": {"why": "the CLI itself runs on it", "source": "https://github.com/pallets/click"},
    "rdflib": {
        "why": "every model is an RDF graph; parsing, querying and serializing run on it",
        "source": "https://github.com/RDFLib/rdflib",
    },
    "rdf_utils": {
        "why": "shared RDF loaders, resolvers and vocabularies every secorolab tool uses"
    },
    "pyshacl": {
        "why": "validates a generated model graph against the published SHACL shapes",
        "source": "https://github.com/RDFLib/pySHACL",
    },
    "rec": {"why": "the archive contract: what a recorded run keeps and how it is read back"},
    "google.protobuf": {
        "why": "decodes the frame log every run writes",
        "source": "https://github.com/protocolbuffers/protobuf",
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
        "why": "compiles .robmot models into the RDF graphs every later stage reads"
    },
    "coord_dsl": {"why": "compiles .fsm coordination models into the FSM the runtime dispatches"},
    "scene_dsl": {
        "why": "compiles .scenex/.ktree scenes into the kinematic tree and simulator assets"
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
    "stst": {"why": "renders the generated C++ from the packaged StringTemplate groups"},
    "cmake": {"why": "configures every generated controller build", "source": "https://cmake.org"},
    "c++": {"why": "compiles the generated controller", "source": "https://gcc.gnu.org"},
    "Protobuf": {
        "why": "the generated runtime links it to write the frame log",
        "source": "https://github.com/protocolbuffers/protobuf",
    },
    "coord2b": {"why": "the FSM event loop the generated controller links and dispatches through"},
    "Eigen3": {
        "why": "the linear algebra under KDL's kinematics",
        "source": "https://gitlab.com/libeigen/eigen",
    },
    "tomlplusplus": {
        "why": "reads the deployment's robot.toml at controller startup",
        "source": "https://github.com/marzer/tomlplusplus",
    },
    "orocos_kdl": {
        "why": "chains, solvers and frames: the kinematics the generated control math runs on"
    },
    "glfw3": {
        "why": "the window and input layer of mj_kdl_wrapper's MuJoCo viewer",
        "source": "https://www.glfw.org",
    },
    "OpenGL": {
        "why": "mj_kdl_wrapper renders the scene through it, on screen and off",
        "source": "https://www.khronos.org/opengl",
    },
    "EGL": {
        "why": "the headless context mj_kdl_wrapper's video recorder renders into",
        "source": "https://www.khronos.org/egl",
    },
    "ffmpeg": {
        "why": "encodes the recorder's frames and the ROS camera recordings into MP4",
        "source": "https://ffmpeg.org",
    },
    "mj_kdl_wrapper": {
        "why": "the MuJoCo simulation the generated controller drives, and its camera publisher"
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
    "robif2b": {"why": "the real-robot hardware drivers the robif2b backend generates against"},
    "urdfdom": {
        "why": "parses the robot's URDF for the real-platform chain",
        "source": "https://github.com/ros/urdfdom",
    },
    "urdfdom_headers": {
        "why": "parses the robot's URDF for the real-platform chain",
        "source": "https://github.com/ros/urdfdom_headers",
    },
    "serial": {"why": "the serial line the Robotiq devices are driven over"},
    "robotiq_driver_noros": {
        "why": "drives the Robotiq gripper and force-torque sensor on a real platform"
    },
}


def manifest_sources() -> None:
    """Fill in where each dependency `setup` provides comes from, as the manifests pin it."""
    from motion_spec.setup import manifest_files, manifest_in_force, read_manifest

    real = {r.name for r in read_manifest(manifest_files(real=True)[-1])}
    for repository in manifest_in_force(manifest_files(real=True)):
        dependency = _PROVIDED_AS.get(repository.name, repository.name.replace("-", "_"))
        SETUP_PROVIDES[dependency] = repository.name
        DETAILS[dependency]["source"] = repository.url.removesuffix(".git")
        if repository.name in real:
            _DEVICE_PACKAGES.add(dependency)


manifest_sources()


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


def module_path(name: str, env: dict[str, str] | None = None) -> str | None:
    if env is not None:
        # a sourced ROS puts its packages on the path only through ENV
        done = subprocess.run(
            [sys.executable, "-c", _PROBE, name],
            env=env,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            check=False,
            text=True,
        )
        if done.returncode:
            return None
        try:
            found = json.loads(done.stdout)
        except json.JSONDecodeError:
            return None
        return found[name]
    try:
        spec = importlib.util.find_spec(name)
    except ModuleNotFoundError:
        return None
    if spec is None:
        return None
    if spec.origin:
        return spec.origin
    return next(iter(spec.submodule_search_locations or ()), None)


def module_what(path: str | None) -> str:
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


def stst_jar(launcher: str) -> bool:
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
        return True  # not a launcher this tool wrote
    return (Path(shlex.split(home)[0]) / "build" / "jar" / "stst.jar").is_file()


def which(executable: str, env: dict[str, str] | None = None) -> str | None:
    """`executable` on PATH -- the one ENV gives, when the caller named an environment."""
    if env and "PATH" in env:
        return shutil.which(executable, path=env["PATH"])
    return shutil.which(executable)


def cmake_package_path(
    name: str,
    *,
    load_target: str | None = None,
    version: str | None = None,
    env: dict[str, str] | None = None,
) -> str | None:
    cmake = which("cmake", env)
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
            # ENV's LD_LIBRARY_PATH decides, not ours
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
            check.name or check.dependency
            for check in checks
            if check.profile in wanted and not check.ok and not check.optional
        ]
    return answer


def cmake_library_path(name: str, env: dict[str, str] | None = None) -> str | None:
    """Where `find_library({name})` lands, the lookup mj_kdl_wrapper's CMake makes itself."""
    cmake = which("cmake", env)
    if not cmake:
        return None
    with tempfile.TemporaryDirectory(prefix="motion-spec-health-") as directory:
        root = Path(directory)
        (root / "CMakeLists.txt").write_text(
            "cmake_minimum_required(VERSION 3.16)\n"
            "project(motion_spec_health LANGUAGES CXX)\n"
            f"find_library(LIBRARY {name})\n"
            "if(NOT LIBRARY)\n"
            f'  message(FATAL_ERROR "{name} not found")\n'
            "endif()\n"
            'file(WRITE "${CMAKE_BINARY_DIR}/library" "${LIBRARY}")\n'
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
        return (build / "library").read_text().strip() or None


def apt_packages(checks: list[HealthCheck]) -> list[str]:
    """The apt packages that would fix every missing check, in the order they were reported.

    A remedy per row is a row-by-row installation: eight of them are eight apt invocations
    to assemble by hand. The ones apt provides are the same command, so they are collected
    into one, and each stays named where it was reported.
    """
    return list(
        dict.fromkeys(
            package
            for check in checks
            if not check.ok and not check.optional
            for package in check.apt
        )
    )


def check_health(
    profiles: tuple[str, ...],
    targets: tuple[str, ...] = (),
    env: dict[str, str] | None = None,
    *,
    on_progress,
) -> list[HealthCheck]:
    """Check only the selected installation profiles and target-specific dependencies.

    ENV is the environment to check under; without it every probe answers for this process.
    ON_PROGRESS is called with (done, dependency) before each probe: configuring a CMake
    project takes seconds, and several of these do.
    """
    selected = tuple(
        profile
        for profile in PROFILES
        if "all" in profiles or profile == "base" or profile in profiles
    )
    checks = []
    for profile in selected:
        for module in PROFILE_IMPORTS.get(profile, ()):
            on_progress(len(checks), module)
            path = module_path(module, env)
            checks.append(
                HealthCheck(
                    profile,
                    module,
                    module_what(path),
                    path,
                    path is not None,
                    "pip install motion_spec"
                    if profile == "base"
                    else f"motion-spec install {profile}",
                )
            )
        if profile == "telemetry":
            on_progress(len(checks), "Protobuf")
            path = cmake_package_path("Protobuf", env=env)
            checks.append(
                HealthCheck(
                    profile, "Protobuf", "CMake package", path, path is not None, remedy("Protobuf")
                )
            )
    if "codegen" in selected:
        from motion_spec.setup import find_stst

        on_progress(len(checks), "stst")
        path = (
            find_stst(path=env.get("PATH", ""), workspace=env.get("MOTION_SPEC_WS"))
            if env is not None
            else find_stst()
        )
        runs = path is not None and stst_jar(path)
        checks.append(
            HealthCheck(
                "codegen",
                "stst",
                "executable" if runs else "executable, jar missing",
                path,
                runs,
                "motion-spec setup stst --force" if path else remedy("stst"),
            )
        )
        for executable in CODEGEN_EXECUTABLES:
            on_progress(len(checks), executable)
            path = which(executable, env)
            checks.append(
                HealthCheck(
                    "codegen", executable, "executable", path, path is not None, remedy(executable)
                )
            )
    if "ros" in selected:
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
            on_progress(len(checks), module)
            path = module_path(module, env)
            checks.append(
                HealthCheck(
                    "ros",
                    module,
                    module_what(path),
                    path,
                    path is not None,
                    remedy(module, env),
                    optional=True,
                )
            )
        for alternatives in ROS_ALTERNATIVES:
            on_progress(len(checks), alternatives[0])
            path = next(
                (found for name in alternatives if (found := module_path(name, env))), None
            )
            checks.append(
                HealthCheck(
                    "ros",
                    " or ".join(alternatives),
                    module_what(path),
                    path,
                    path is not None,
                    remedy(alternatives[0], env),
                    optional=True,
                    name=alternatives[0],
                )
            )
        for name in ROS_BUILD_PACKAGES:
            on_progress(len(checks), name)
            path = cmake_package_path(name, env=env)
            checks.append(
                HealthCheck(
                    "ros",
                    name,
                    "CMake package",
                    path,
                    path is not None,
                    remedy(name, env),
                    optional=True,
                )
            )
    if "build" in selected:
        for executable in ("cmake", "c++"):
            on_progress(len(checks), executable)
            path = which(executable, env)
            checks.append(
                HealthCheck(
                    "build", executable, "executable", path, path is not None, remedy(executable)
                )
            )
        for name in GENERAL_BUILD_PACKAGES:
            on_progress(len(checks), name)
            path = cmake_package_path(name, env=env)
            checks.append(
                HealthCheck("build", name, "CMake package", path, path is not None, remedy(name))
            )
        if "mujoco" in targets:
            # mj_kdl_wrapper's configure needs these
            on_progress(len(checks), "glfw3")
            path = cmake_package_path("glfw3", env=env)
            checks.append(
                HealthCheck(
                    "build[mujoco]",
                    "glfw3",
                    "CMake package",
                    path,
                    path is not None,
                    remedy("glfw3"),
                )
            )
            for name, library in (("OpenGL", "GL"), ("EGL", "EGL")):
                on_progress(len(checks), name)
                path = cmake_library_path(library, env=env)
                checks.append(
                    HealthCheck(
                        "build[mujoco]",
                        name,
                        "shared library",
                        path,
                        path is not None,
                        remedy(name),
                    )
                )
            on_progress(len(checks), "ffmpeg")
            path = which("ffmpeg", env)
            checks.append(
                HealthCheck(
                    "build[mujoco]",
                    "ffmpeg",
                    "executable",
                    path,
                    path is not None,
                    remedy("ffmpeg"),
                )
            )
        from motion_spec.setup import shipped_pin

        # the generated CMakeLists requires the pinned version
        for target, packages in (
            ("mujoco", (("mj_kdl_wrapper", shipped_pin("mj_kdl_wrapper").version.lstrip("v")),)),
            ("robif2b", ROBIF2B_BUILD_PACKAGES),
        ):
            if target not in targets:
                continue
            for package in packages:
                name, version = package if isinstance(package, tuple) else (package, None)
                on_progress(len(checks), name)
                path = cmake_package_path(name, version=version, env=env)
                checks.append(
                    HealthCheck(
                        f"build[{target}]",
                        f"{name} {version}" if version else name,
                        "CMake package",
                        path,
                        path is not None,
                        remedy(name),
                        optional=name in OPTIONAL_BUILD_PACKAGES,
                        name=name,
                    )
                )
    if "runtime" in selected:
        on_progress(len(checks), "coord2b")
        path = cmake_package_path("coord2b", load_target="coord2b", env=env)
        checks.append(
            HealthCheck(
                "runtime", "coord2b", "shared library", path, path is not None, remedy("coord2b")
            )
        )
        runtime_targets = {
            "mujoco": (("mj_kdl_wrapper", "mj_kdl_wrapper::mj_kdl_wrapper"),),
            "robif2b": (
                ("robif2b", "robif2b::kinova_gen3"),
                ("robif2b", "robif2b::robotiq_gripper"),
                ("robif2b", "robif2b::robotiq_ft_sensor"),
            ),
        }
        for target in targets:
            for package, cmake_target in runtime_targets[target]:
                on_progress(len(checks), cmake_target)
                path = cmake_package_path(package, load_target=cmake_target, env=env)
                flag = _ROBIF2B_DEVICE_FLAGS.get(cmake_target)
                checks.append(
                    HealthCheck(
                        f"runtime[{target}]",
                        cmake_target if package != cmake_target else package,
                        "shared library",
                        path,
                        path is not None,
                        remedy(cmake_target.partition("::")[0])
                        if flag is None
                        else f'add "-D{flag}=ON" to robif2b in colcon.meta, then '
                        "motion-spec setup --real --force robif2b",
                    )
                )
    enriched = []
    for check in checks:
        bare = check.name or check.dependency
        name = bare.partition("::")[0]
        spec = DETAILS.get(check.dependency) or DETAILS.get(name) or {}
        alternative = check.alternative
        if not alternative:
            flag = _ROBIF2B_DEVICE_FLAGS.get(bare)
            if flag:
                alternative = cmake_build("robif2b", *installed_options("robif2b"), f"-D{flag}=ON")
            elif name in SETUP_PROVIDES:
                alternative = cmake_build(name, *installed_options(name))
        enriched.append(
            dataclasses.replace(
                check,
                why=spec.get("why", check.why),
                source=spec.get("source", check.source),
                alternative=alternative,
                apt=tuple(
                    package.replace("$ROS_DISTRO", ros_distro(env) or "$ROS_DISTRO")
                    for package in APT_PACKAGES.get(name, ())
                ),
            )
        )
    return enriched
