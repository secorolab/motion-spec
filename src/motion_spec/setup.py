# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
# Author: Vamsi Kalagaturu

"""Install the external tools and libraries every motion-spec installation builds against.

Sources in src/, builds in build/, everything installed into install/; only what colcon cannot
build goes under src/thirdparty/. A source directory that already exists is built only when
clean and on the pinned commit, never moved.
"""

from __future__ import annotations

import importlib.util
import os
import shlex
import shutil
import subprocess
import sys
from dataclasses import dataclass, field
from pathlib import Path
from urllib.request import urlretrieve

from motion_spec.utils import tee, total_memory, trash_if_present, usable_cores

MANIFEST = "motion_spec.repos"
STST_REPOSITORY = "thirdparty/STSTv4"
JARS = {
    "ST4-4.3.4.jar": "https://repo1.maven.org/maven2/org/antlr/ST4/4.3.4/ST4-4.3.4.jar",
    "antlr-runtime-3.5.3.jar": (
        "https://repo1.maven.org/maven2/org/antlr/antlr-runtime/3.5.3/antlr-runtime-3.5.3.jar"
    ),
}
WORKSPACE_VARIABLE = "MOTION_SPEC_WS"
# The layout `vcs import` produces, so either route gives the same workspace.
SOURCE_DIRECTORY = "src"
# Where a plain install puts the sources it must build: hidden, because they are setup's to
# fetch and delete, not the operator's to edit. `--dev` uses src/ for everything instead.
MANAGED_SOURCE_DIRECTORY = ".ms-sources"
BUILD_DIRECTORY = "build"
INSTALL_DIRECTORY = "install"
GENERATION_DIRECTORY = "generations"
GENERATION_VARIABLE = "MOTION_SPEC_GEN"
# For sources colcon has no business building; `setup` marks it with a COLCON_IGNORE.
THIRDPARTY_DIRECTORY = "thirdparty"
COLCON_IGNORE = "COLCON_IGNORE"
MANAGED = Path("share") / "motion-spec"
BUILD_TYPE = "RelWithDebInfo"
BUILD_TYPE_VARIABLE = "MOTION_SPEC_BUILD_TYPE"
# A marker origin, beside cloned and adopted: pip fetched it, so there is no source to clean.
PIP_ORIGIN = "pip"


@dataclass(frozen=True)
class Source:
    """Where one pinned repository comes from."""

    url: str
    version: str


def read_manifest(path: Path | None = None) -> dict[str, Source]:
    """The repositories the shipped .repos manifest declares, by directory name.

    A line outside vcstool's shape raises: a pin read wrong is a build of the wrong version.
    """
    manifest = path or Path(__file__).parent / MANIFEST
    entries: dict[str, dict[str, str]] = {}
    current: dict[str, str] | None = None
    for number, raw in enumerate(manifest.read_text().splitlines(), start=1):
        line = raw.split("#", 1)[0].rstrip()
        if not line or line == "repositories:":
            continue
        if line.startswith("    ") and current is not None:
            key, separator, value = line.strip().partition(":")
            if not separator:
                raise ValueError(f"{manifest}:{number}: expected `key: value`, got {raw!r}")
            current[key] = value.strip()
        elif line.startswith("  ") and not line.startswith("   ") and line.endswith(":"):
            current = entries.setdefault(line.strip().rstrip(":"), {})
        else:
            raise ValueError(f"{manifest}:{number}: not a repository or a field: {raw!r}")

    sources = {}
    for name, fields in entries.items():
        missing = {"type", "url", "version"} - fields.keys()
        if missing:
            raise ValueError(f"{manifest}: {name} declares no {', '.join(sorted(missing))}")
        if fields["type"] != "git":
            raise ValueError(f"{manifest}: {name} is {fields['type']}; only git is supported")
        sources[name] = Source(fields["url"], fields["version"])
    return sources


SOURCES = read_manifest()
STST_REPO = SOURCES[STST_REPOSITORY].url
STST_REF = SOURCES[STST_REPOSITORY].version
# The version the generated CMakeLists asks for; health checks the same one.
MJ_KDL_REF = SOURCES["mj_kdl_wrapper"].version


@dataclass(frozen=True)
class Component:
    """One library installed from its own source into the shared prefix.

    The name is what a user asks for and what health reports; `repository` is its entry in
    the manifest, which is also the directory it is checked out into -- the two differ where
    a repository carries more than the one package.
    """

    name: str
    repository: str
    source: str = ""
    options: tuple[str, ...] = field(default_factory=tuple)
    # Also install the checkout's Python package; its extension is a separate build.
    bindings: bool = False
    # A Python package, not a CMake one: pip installs the checkout and nothing is built.
    python: bool = False
    requires: tuple[str, ...] = field(default_factory=tuple)
    # Installed only when named: a model that binds no device never links these.
    on_request: bool = False
    why: str = ""


# Dependency order: mj_kdl_wrapper links orocos_kdl.
COMPONENTS = (
    # Before scene_dsl: motion_spec_dsl requires it from git, and pip would pull that over a
    # checkout already installed, undoing the local one.
    Component(
        "motion_spec_dsl",
        "motion-spec-dsl",
        python=True,
        why="compiles .robmot models into the RDF graphs every later stage reads",
    ),
    Component(
        "scene_dsl",
        "scene-dsl",
        python=True,
        why="compiles .scenex/.ktree scenes into the kinematic tree and simulator assets",
    ),
    Component(
        "orocos_kdl",
        "orocos_kinematics_dynamics",
        source="orocos_kdl",
        why="the secorolab fork: the Vereshchagin solvers with fixed joints the templates call",
    ),
    Component(
        "coord2b",
        "coord2b",
        why="the FSM event loop the generated controller dispatches through",
    ),
    Component(
        "mj_kdl_wrapper",
        "mj_kdl_wrapper",
        options=(
            # The robot models a generated scene names by package path.
            "-DMJ_KDL_FETCH_MENAGERIE=ON",
            # Left off, it builds the KDL fork again: two liborocos-kdl, one SONAME, one process.
            "-DMJ_KDL_OROCOS_KDL_FROM_PACKAGE=ON",
        ),
        bindings=True,
        requires=("orocos_kdl",),
        why="the MuJoCo simulation the generated controller drives, and its camera publisher",
    ),
    Component(
        "serial",
        "serial",
        on_request=True,
        why="the serial line the Robotiq devices are driven over",
    ),
    Component(
        "robotiq_driver_noros",
        "robotiq_driver_noros",
        on_request=True,
        why="drives the Robotiq gripper and force-torque sensor on a real platform",
    ),
    Component(
        "robif2b",
        "robif2b",
        # Each device wrapper stays off until its own flag is passed; health names which.
        options=("-DENABLE_INSTALL_TARGETS=ON",),
        on_request=True,
        why="the real-robot hardware drivers the robif2b backend generates against",
    ),
)
COMPONENTS_BY_NAME = {component.name: component for component in COMPONENTS}
# Ant builds this one, but its source is fetched by the same rules.
STST_COMPONENT = Component(
    "stst", STST_REPOSITORY, why="renders the generated C++ from the packaged StringTemplate groups"
)
COMPONENT_NAMES = ("stst", *COMPONENTS_BY_NAME)
# Name to the cmake options motion-spec already passes, for the config sample to show.
COMPONENT_OPTIONS = {
    "stst": STST_COMPONENT.options,
    **{component.name: component.options for component in COMPONENTS if not component.python},
}
# What a bare `motion-spec setup` installs; the rest are named or not built.
DEFAULT_COMPONENTS = ("stst", *(c.name for c in COMPONENTS if not c.on_request))
ENVIRONMENT_FILES = ("setup-motion-spec.bash", "setup-motion-spec.zsh")
ENVIRONMENT_VARIABLE = "MOTION_SPEC_ENV"
# Archived with a run. Not the whole environment: that is mostly the operator's shell.
PROVENANCE_VARIABLES = (
    "MOTION_SPEC_PREFIX",
    "PATH",
    "CMAKE_PREFIX_PATH",
    "LD_LIBRARY_PATH",
    "PYTHONPATH",
    "ROS_DISTRO",
    "AMENT_PREFIX_PATH",
    "RMW_IMPLEMENTATION",
    "ROS_DOMAIN_ID",
)


def workspace(argument: Path | None = None) -> Path:
    """The workspace: --workspace, else $MOTION_SPEC_WS, else the config file, else an error.

    Neither is an error, not a guess: this installs a toolchain.
    """
    from motion_spec.config import CONFIG_FILE, settings

    configured, path = settings()
    # The file's own directory is the workspace unless it says otherwise, so a workspace with
    # one needs nothing in the shell.
    declared = configured.get("workspace", {}).get("root") or (str(path.parent) if path else None)
    named = os.environ.get(WORKSPACE_VARIABLE)
    chosen = argument or (Path(named) if named else None) or (Path(declared) if declared else None)
    if chosen is None:
        raise RuntimeError(
            f"no workspace: set {WORKSPACE_VARIABLE}, pass --workspace <path>, or put a "
            f"{CONFIG_FILE} in it"
        )
    root = chosen.expanduser()
    if not root.is_dir():
        raise RuntimeError(f"workspace is not a directory: {root}")
    return root.resolve()


def install_prefix(root: Path) -> Path:
    """Where a workspace's builds are installed."""
    return root / INSTALL_DIRECTORY


def generations_root() -> Path:
    """Where generations are written: $MOTION_SPEC_GEN, else the workspace's generations/.

    Never the working directory: `rerun` and the dashboard look in one place.
    """
    from motion_spec.config import settings

    named = os.environ.get(GENERATION_VARIABLE, "").strip()
    if named:
        return Path(named).expanduser()
    configured, _ = settings()
    try:
        root = workspace()
    except RuntimeError as exc:
        raise RuntimeError(
            f"no generation directory: set {GENERATION_VARIABLE}, or {WORKSPACE_VARIABLE} to "
            f"use its {GENERATION_DIRECTORY}/"
        ) from exc
    return generations_directory(root, configured)


def generations_directory(root: Path, configured: dict) -> Path:
    """Where ROOT keeps its generations, with no environment variable in the way."""
    declared = configured.get("workspace", {}).get("generations")
    return Path(declared) if declared else root / GENERATION_DIRECTORY


def source_root(root: Path, dev: bool = True) -> Path:
    """Where sources are checked out: src/ is the developer's, and only theirs to edit."""
    return root / (SOURCE_DIRECTORY if dev else MANAGED_SOURCE_DIRECTORY)


def thirdparty_directory(root: Path, dev: bool = True) -> Path:
    """The subtree colcon leaves alone: what it could not build, or must not."""
    return source_root(root, dev) / THIRDPARTY_DIRECTORY


def source_directory(root: Path, repository: str, dev: bool = True) -> Path:
    """Where a workspace keeps one repository's source, as the manifest spells its path."""
    return source_root(root, dev) / repository


def checked_out(root: Path, repository: str) -> Path:
    """The tree a repository is actually in, whichever mode put it there."""
    for dev in (True, False):
        candidate = source_directory(root, repository, dev)
        if candidate.exists():
            return candidate
    return source_directory(root, repository)


def _ignore_thirdparty(root: Path, dev: bool = True) -> Path:
    """Create the third-party subtree, marked so `colcon build` does not descend into it."""
    directory = thirdparty_directory(root, dev)
    directory.mkdir(parents=True, exist_ok=True)
    marker = directory / COLCON_IGNORE
    if not marker.exists():
        marker.touch()
    return directory


def build_directory(root: Path, name: str) -> Path:
    """Where a workspace builds one component."""
    return root / BUILD_DIRECTORY / name


def required_profiles(components: list[str], ros: bool = False) -> tuple[str, ...]:
    """The health profiles the named COMPONENTS need before any of them can be installed.

    Scoped, so `setup coord2b` is not refused for want of the Ant that only stst uses.
    """
    named = [COMPONENTS_BY_NAME[name] for name in components if name in COMPONENTS_BY_NAME]
    profiles = {"codegen"} if "stst" in components else set()
    if any(not component.python for component in named):
        profiles.add("build")
    return (*sorted(profiles), *(("ros",) if ros and profiles else ()))


def missing_prerequisites(
    components: list[str], ros: bool = False, targets: tuple[str, ...] = ("mujoco",)
) -> tuple[list[str], list[str]]:
    """What must be installed before setup starts: `(apt packages, other requirements)`.

    Only what setup cannot supply itself. Everything it does install carries a
    `motion-spec setup <name>` remedy instead, and is absent here by construction.
    """
    from motion_spec.health import (
        _ros_distro,
        apt_packages,
        check_health,
        installed_ros_distros,
        system_site_packages,
    )

    profiles = required_profiles(components, ros)
    if not profiles:
        return [], []
    packages = apt_packages(check_health(profiles, targets if "build" in profiles else ()))
    others = []
    if ros:
        # The environment file is written last, so an unresolved distro would surface only
        # after everything is built -- and the build would have used whatever was sourced.
        if _ros_distro() is None:
            installed = ", ".join(installed_ros_distros()) or "none under /opt/ros"
            others.append(
                f"a ROS distribution: source one, or set [ros] distro; installed: {installed}"
            )
        if shutil.which("colcon") is None:
            others.append("colcon: apt install python3-colcon-common-extensions")
        if not system_site_packages():
            others.append(
                "the distribution's Python packages: recreate the environment with "
                "`uv venv --python /usr/bin/python3 --system-site-packages`"
            )
    return packages, others


def build_jobs(requested: int | None = None) -> int:
    """How many compilers to run at once; memory caps it, not cores."""
    if requested is not None:
        return max(1, requested)
    cores = usable_cores()
    memory = total_memory()
    if memory is None:
        return max(1, cores // 2)
    return max(1, min(cores, memory // (2 * 1024**3)))


@dataclass(frozen=True)
class SourceState:
    """What was found at a component's source path, and whether setup will build it."""

    path: Path
    # Only a checkout setup cloned is setup's to delete.
    cloned: bool
    usable: bool
    reason: str = ""
    # What to name as the source; empty means the path.
    origin: str = ""


def find_stst(path: str | None = None) -> str | None:
    """Prefer the STST on PATH, falling back to the one the sourced workspace installed.

    PATH is the search path to look along, for a caller asking about an environment other
    than this process's.
    """
    on_path = shutil.which("stst", path=path) if path else shutil.which("stst")
    if on_path:
        return on_path
    # Derived, not a variable of its own: one more to keep in step is one more to disagree.
    named = os.environ.get(WORKSPACE_VARIABLE)
    managed = install_prefix(Path(named).expanduser()) / "bin" / "stst" if named else None
    return str(managed) if managed and managed.is_file() else None


def remove_stst(root: Path, prefix: Path | None = None) -> bool:
    """Remove an STST installation this tool made, and only a source it cloned itself."""
    prefix = prefix or install_prefix(root)
    launcher = prefix / "bin" / "stst"
    marker = _marker("stst", prefix)
    if not (launcher.exists() or marker.exists()):
        return False
    if not marker.is_file():
        raise RuntimeError(f"refusing to clean an unmanaged STST installation under {prefix}")
    if _recorded_origin(marker) == "cloned":
        trash_if_present(checked_out(root, STST_REPOSITORY))
    trash_if_present(launcher)
    marker.unlink(missing_ok=True)
    try:
        (prefix / MANAGED).rmdir()
    except OSError:
        pass
    return True


def component_installed(component: Component, prefix: Path, dev: bool | None = None) -> bool:
    """Whether PREFIX has COMPONENT at the pinned ref, by the route DEV asks for."""
    marker = _marker(component.name, prefix)
    version = SOURCES[component.repository].version
    if not (marker.is_file() and marker.read_text().split("\n")[0] == version):
        return False
    if component.python and dev is not None:
        return (_recorded_origin(marker) == PIP_ORIGIN) != dev
    return True


def stst_installed(root: Path, prefix: Path | None = None, dev: bool = True) -> bool:
    """Whether PREFIX carries a usable stst: a launcher without its jar is a half-finished one."""
    prefix = prefix or install_prefix(root)
    if not (prefix / "bin" / "stst").is_file():
        return False
    # Either tree counts: the launcher runs the jar wherever the install that made it put one.
    return any(
        (source_directory(root, STST_REPOSITORY, where) / "build" / "jar" / "stst.jar").is_file()
        for where in {dev, True, False}
    )


def install_stst(
    root: Path,
    prefix: Path | None = None,
    force: bool = False,
    log: Path | None = None,
    dev: bool = True,
) -> Path:
    """Build the pinned STSTv4 from its checkout and install its launcher under PREFIX/bin."""
    prefix = prefix or install_prefix(root)
    launcher = prefix / "bin" / "stst"
    source = source_directory(root, STST_REPOSITORY, dev)
    marker = _marker("stst", prefix)
    if not force and stst_installed(root, prefix, dev):
        return launcher

    missing = [command for command in ("git", "ant", "java") if shutil.which(command) is None]
    if missing:
        raise RuntimeError(
            f"required command{'s' if len(missing) > 1 else ''} missing: {', '.join(missing)}"
        )

    # Ant writes inside the source tree, so an adopted checkout must be clean and on the pin.
    previous = _recorded_origin(marker)
    state = prepare_source(STST_COMPONENT, root, log, dev)
    if not state.usable:
        raise RuntimeError(f"{state.path} {state.reason}")
    marker.parent.mkdir(parents=True, exist_ok=True)
    marker.write_text(f"{STST_REF}\n{_origin(previous, state)}\n")
    tee(["ant", "-f", str(source / "build.xml")], log=log)

    lib = source / "lib"
    lib.mkdir(exist_ok=True)
    for filename, url in JARS.items():
        path = lib / filename
        if not path.exists():
            urlretrieve(url, path)

    launcher.parent.mkdir(parents=True, exist_ok=True)
    launcher.write_text(
        "#!/usr/bin/env bash\n"
        "set -euo pipefail\n"
        f"STST_HOME={shlex.quote(str(source))}\n"
        'CP="$STST_HOME/build/jar/stst.jar:$STST_HOME/lib/ST4-4.3.4.jar:'
        '$STST_HOME/lib/antlr-runtime-3.5.3.jar"\n'
        'exec java -cp "$CP" jjs.stst.STStandaloneTool "$@"\n'
    )
    launcher.chmod(0o755)
    return launcher


def shadowing_stst(prefix: Path) -> tuple[Path, bool] | None:
    """An `stst` PATH reaches before PREFIX's, and whether its jar is still there.

    Earlier motion-spec versions installed the launcher into `~/.local/bin`, which most PATHs
    put ahead of a workspace. Left alone it wins and codegen dies on its missing jar.
    """
    found = shutil.which("stst")
    if not found or Path(found) == prefix / "bin" / "stst":
        return None
    other = Path(found)
    try:
        body = other.read_text()
    except OSError:
        return None
    home = next(
        (line.split("=", 1)[1].strip() for line in body.splitlines() if line.startswith("STST_HOME=")),
        None,
    )
    if home is None:
        return None
    return other, (Path(shlex.split(home)[0]) / "build" / "jar" / "stst.jar").is_file()


def is_installed(component: Component, prefix: Path) -> bool:
    """Whether PREFIX carries an installation of COMPONENT this tool made."""
    marker = _marker(component.name, prefix)
    return marker.is_file() and marker.read_text().strip() != "installing"


def _trash_installed(manifest: Path, prefix: Path, label: str) -> None:
    """Trash the files an install left in PREFIX, as one entry rather than hundreds."""
    if not manifest.is_file():
        return
    staging = prefix.parent / f".removed-{label}"
    for line in manifest.read_text().splitlines():
        installed = Path(line.strip())
        if not line.strip() or not (installed.exists() or installed.is_symlink()):
            continue
        try:
            destination = staging / installed.relative_to(prefix)
        except ValueError:
            # Outside the prefix: keep it by name.
            destination = staging / "elsewhere" / installed.name
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.move(str(installed), str(destination))
    trash_if_present(staging)


def _recorded_origin(marker: Path) -> str:
    """Whether an earlier install cloned this source. Sticky: a rebuild would read it as adopted.

    A one-line marker is the v1 format, written before `setup` could adopt a checkout, so
    everything it recorded was cloned.
    """
    if not marker.is_file():
        return ""
    lines = marker.read_text().splitlines()
    if len(lines) < 2:
        return "cloned" if lines else ""
    return lines[-1]


def _origin(previous: str, state: SourceState) -> str:
    return "cloned" if state.cloned or previous == "cloned" else "adopted"


def _marker(name: str, prefix: Path) -> Path:
    return prefix / MANAGED / f".{name}-managed"


def _git(repository: Path, *arguments: str) -> str | None:
    """The output of a read-only git command, or None when it fails."""
    done = subprocess.run(
        ["git", "-C", str(repository), *arguments],
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
        text=True,
        check=False,
    )
    return done.stdout.strip() if done.returncode == 0 else None


def _pinned_commit(repository: Path, ref: str) -> str | None:
    """The commit REF names, preferring the remote: a local branch still points where it did."""
    for candidate in (f"origin/{ref}", ref):
        commit = _git(repository, "rev-parse", "--verify", "--quiet", f"{candidate}^{{commit}}")
        if commit:
            return commit
    return None


def prepare_source(
    component: Component, root: Path, log: Path | None = None, dev: bool = True
) -> SourceState:
    """Put COMPONENT's source in place without ever moving a checkout already there.

    A `checkout --detach` in a tree someone works in loses the branch they were on.
    """
    spec = SOURCES[component.repository]
    repository = source_directory(root, component.repository, dev)

    if not (repository / ".git").is_dir():
        if repository.exists() and any(repository.iterdir()):
            return SourceState(repository, False, False, "is not a git checkout")
        if component.repository.startswith(f"{THIRDPARTY_DIRECTORY}/"):
            _ignore_thirdparty(root, dev)
        repository.parent.mkdir(parents=True, exist_ok=True)
        tee(["git", "clone", spec.url, str(repository)], log=log)
        tee(["git", "-C", str(repository), "fetch", "--tags", "origin"], log=log)
        commit = _pinned_commit(repository, spec.version)
        tee(
            ["git", "-C", str(repository), "checkout", "--detach", commit or spec.version],
            log=log,
        )
        return SourceState(repository, True, True)

    subprocess.run(
        ["git", "-C", str(repository), "fetch", "--tags", "origin"],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        check=False,
    )
    commit = _pinned_commit(repository, spec.version)
    head = _git(repository, "rev-parse", "HEAD")
    if commit is None:
        return SourceState(repository, False, False, f"knows no ref {spec.version}")
    if head != commit:
        described = _git(repository, "describe", "--all", "--always", "HEAD") or (head or "?")
        return SourceState(
            repository, False, False, f"is at {described}, and the manifest pins {spec.version}"
        )
    if _git(repository, "status", "--porcelain", "--untracked-files=no"):
        return SourceState(repository, False, False, "has uncommitted changes")
    return SourceState(repository, False, True)


def _cmake_prefix_path(prefix: Path) -> str:
    """PREFIX first, then whatever the caller's environment already pointed at.

    Dropping the inherited value would hide a dependency installed by apt or by another
    prefix, and the configure would fail on a machine where the library is in fact there.
    """
    inherited = os.environ.get("CMAKE_PREFIX_PATH", "")
    return f"{prefix}{os.pathsep}{inherited}" if inherited else str(prefix)


def install_component(
    component: Component,
    root: Path,
    prefix: Path | None = None,
    *,
    force: bool = False,
    build_type: str = BUILD_TYPE,
    log: Path | None = None,
    options: tuple[str, ...] | None = None,
    ros: bool = False,
    editable: bool = False,
    jobs: int | None = None,
    dev: bool = False,
) -> SourceState:
    """Build COMPONENT from ROOT/src into ROOT/build and install it into PREFIX.

    The marker carries the ref installed, so a rerun is a no-op until the pin moves. A source
    this tool will not touch is returned unbuilt. Without DEV a Python component is not a
    checkout at all: pip fetches the pinned ref itself.
    """
    prefix = prefix or install_prefix(root)
    source_spec = SOURCES[component.repository]
    marker = _marker(component.name, prefix)
    if not force and component_installed(component, prefix, dev):
        return SourceState(
            source_directory(root, component.repository, dev), False, True, "installed"
        )

    if component.python and not dev:
        return _install_python_from_git(component, root, source_spec, marker, log)

    needed = ("git",) if component.python else ("git", "cmake")
    missing = [command for command in needed if shutil.which(command) is None]
    if missing:
        raise RuntimeError(
            f"required command{'s' if len(missing) > 1 else ''} missing: {', '.join(missing)}"
        )

    previous = _recorded_origin(marker)
    state = prepare_source(component, root, log, dev)
    if not state.usable:
        return state

    marker.parent.mkdir(parents=True, exist_ok=True)
    # Before the build: a half-built installation is still this tool's.
    marker.write_text(f"installing\n{_origin(previous, state)}\n")

    if component.python:
        # No ament here: a pure-Python component has no cmake, and disabling isolation would
        # strand every source dependency pip resolves for it without its own build backend.
        _pip_install(state.path, log, editable)
        marker.write_text(f"{source_spec.version}\n{_origin(previous, state)}\n")
        return state

    if ros:
        _colcon_build(component, root, build_type, log, build_jobs(jobs), dev)
        if component.bindings:
            _pip_install(state.path, log, editable, ros)
        marker.write_text(f"{source_spec.version}\n{_origin(previous, state)}\n")
        return state

    build = build_directory(root, component.name)
    source = state.path / component.source if component.source else state.path
    cmake = shutil.which("cmake")
    tee(
        [
            cmake,
            "-S",
            str(source),
            "-B",
            str(build),
            f"-DCMAKE_INSTALL_PREFIX={prefix}",
            f"-DCMAKE_PREFIX_PATH={_cmake_prefix_path(prefix)}",
            f"-DCMAKE_BUILD_TYPE={build_type}",
            *(component.options if options is None else options),
        ],
        log=log,
    )
    # A bare --parallel is make -j: unlimited, and it overrides the env and MAKEFLAGS too.
    tee([cmake, "--build", str(build), "--parallel", str(build_jobs(jobs))], log=log)
    tee([cmake, "--install", str(build)], log=log)

    if component.bindings:
        _pip_install(state.path, log, editable, ros)

    marker.write_text(f"{source_spec.version}\n{_origin(previous, state)}\n")
    return state


def installer() -> list[str]:
    """How to install a Python package into this environment.

    A virtual environment uv made has no pip in it at all, so `python -m pip` there fails with
    "No module named pip"; uv installs into the interpreter it is pointed at instead.
    """
    if importlib.util.find_spec("pip") is not None:
        return [sys.executable, "-m", "pip", "install"]
    uv = shutil.which("uv")
    if uv is None:
        raise RuntimeError(
            f"neither pip nor uv is available to install with: {sys.executable} has no pip "
            f"module, and no `uv` is on PATH"
        )
    return [uv, "pip", "install", "--python", sys.executable]


def _build_requirements(source: Path) -> list[str]:
    """What PEP 518 says this checkout needs to build, since nothing else will install them."""
    manifest = source / "pyproject.toml"
    if not manifest.is_file():
        return []
    import tomllib

    return tomllib.loads(manifest.read_text()).get("build-system", {}).get("requires", [])


def _pip_install(source: Path, log: Path | None, editable: bool, ros: bool = False) -> None:
    """Install a checkout. Editable points site-packages back at it, so `--clean` would orphan
    the installation along with the source it removes.

    ROS is for an extension whose cmake finds ament: ament's scripts import ament_package,
    which reaches the interpreter over PYTHONPATH from the sourced distro, and pip replaces
    PYTHONPATH with its own for an isolated build. Isolation off means pip installs no build
    backend at all, for this checkout or for anything it resolves, so the checkout's own
    requirements go in first and the flag stays off everything that does not need it.
    """
    arguments = ["--editable", str(source)] if editable else [str(source)]
    if ros:
        requires = _build_requirements(source)
        if requires:
            tee([*installer(), *requires], log=log)
        arguments = ["--no-build-isolation", *arguments]
    tee([*installer(), *arguments], log=log)


def git_requirement(source: Source) -> str:
    """The pip requirement for a pinned repository."""
    return f"git+{source.url}@{source.version}"


def _install_python_from_git(
    component: Component, root: Path, source: Source, marker: Path, log: Path | None
) -> SourceState:
    """Let pip fetch COMPONENT itself, leaving nothing under src/."""
    marker.parent.mkdir(parents=True, exist_ok=True)
    requirement = git_requirement(source)
    # Before the install: a half-installed component is still this tool's.
    marker.write_text(f"installing\n{PIP_ORIGIN}\n")
    tee([*installer(), requirement], log=log)
    marker.write_text(f"{source.version}\n{PIP_ORIGIN}\n")
    return SourceState(
        source_directory(root, component.repository), False, True, origin=requirement
    )


def remove_component(component: Component, root: Path, prefix: Path | None = None) -> bool:
    """Remove a COMPONENT installation this tool made, and only the source it cloned itself."""
    prefix = prefix or install_prefix(root)
    marker = _marker(component.name, prefix)
    build = build_directory(root, component.name)
    if not (build.exists() or marker.exists()):
        return False
    if not marker.is_file():
        raise RuntimeError(
            f"refusing to clean an unmanaged {component.name} installation under {prefix}"
        )
    # The only record of what landed in PREFIX; without it find_package keeps finding it.
    _trash_installed(build / "install_manifest.txt", prefix, f"{component.name}-install")
    trash_if_present(build)
    if _recorded_origin(marker) == "cloned":
        trash_if_present(checked_out(root, component.repository))
    marker.unlink(missing_ok=True)
    for directory in (prefix / MANAGED, root / BUILD_DIRECTORY):
        try:
            directory.rmdir()
        except OSError:
            pass
    return True


def _activation() -> str:
    """Activate the environment setup ran in, unless this shell is already in it.

    Sourcing the file is the one step between a fresh shell and a working workspace, and a
    `motion-spec: command not found` right after it helps nobody.
    """
    activate = Path(sys.prefix) / "bin" / "activate"
    if not (Path(sys.prefix) / "pyvenv.cfg").is_file() or not activate.is_file():
        return ""
    quoted = shlex.quote(str(activate))
    return f'[ "${{VIRTUAL_ENV:-}}" = {shlex.quote(sys.prefix)} ] || . {quoted}\n'


def write_environment(root: Path, prefix: Path | None = None, ros: bool | None = None) -> Path:
    """Write ROOT's environment file, pointing at the prefix its builds were installed into.

    One file, for the shell in force. `motion-spec mutate` finds a workspace by it.
    """
    from motion_spec.config import settings, shell

    prefix = prefix or install_prefix(root)
    configured, _ = settings(root)
    using = shell(configured)
    path = root / f"setup-motion-spec.{using}"
    if configured.get("ros", {}).get("workspace") if ros is None else ros:
        return _write_ros_environment(root, prefix, configured, using, path)
    body = (
        "# Written by `motion-spec setup`. Source it before generating, building or running.\n"
        + _activation()
        + f"export {WORKSPACE_VARIABLE}={shlex.quote(str(root))}\n"
        f"export {GENERATION_VARIABLE}={shlex.quote(str(generations_directory(root, configured)))}\n"
        f"export {ENVIRONMENT_VARIABLE}={shlex.quote(str(path))}\n"
        f"export MOTION_SPEC_PREFIX={shlex.quote(str(prefix))}\n"
        'export PATH="$MOTION_SPEC_PREFIX/bin${PATH:+:$PATH}"\n'
        'export CMAKE_PREFIX_PATH="$MOTION_SPEC_PREFIX'
        '${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"\n'
        'export LD_LIBRARY_PATH="$MOTION_SPEC_PREFIX/lib'
        '${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"\n'
    )
    root.mkdir(parents=True, exist_ok=True)
    path.write_text(body)
    path.chmod(0o755)
    return path


def _write_ros_environment(
    root: Path, prefix: Path, configured: dict, using: str, path: Path
) -> Path:
    """The two sourcings a colcon workspace needs, rather than paths exported by hand."""
    from motion_spec.health import _ros_distro

    distro = _ros_distro()
    if distro is None:
        raise RuntimeError(
            "no ROS distribution: set [ros] distro, or source one, for a [ros] workspace"
        )
    overlay = prefix / f"setup.{using}"
    quoted_overlay = shlex.quote(str(overlay))
    body = (
        "# Written by `motion-spec setup`. Source it before generating, building or running.\n"
        + _activation()
        # Each sourcing is skipped when this shell has already done it: sourcing a distro
        # twice is noise, and sourcing an overlay twice repeats it on every path it sets.
        + f'[ "${{ROS_DISTRO:-}}" = {distro} ] || . /opt/ros/{distro}/setup.{using}\n'
        + f"case \":${{COLCON_PREFIX_PATH:-}}:\" in *:{prefix}:*) ;; *)"
        f" [ -f {quoted_overlay} ] && . {quoted_overlay} ;; esac\n"
        + f"export {WORKSPACE_VARIABLE}={shlex.quote(str(root))}\n"
        f"export {GENERATION_VARIABLE}={shlex.quote(str(generations_directory(root, configured)))}\n"
        f"export {ENVIRONMENT_VARIABLE}={shlex.quote(str(path))}\n"
        f"export MOTION_SPEC_PREFIX={shlex.quote(str(prefix))}\n"
        # stst is ant-built into the prefix, so it is in no colcon package and on no overlay.
        'export PATH="$MOTION_SPEC_PREFIX/bin${PATH:+:$PATH}"\n'
    )
    root.mkdir(parents=True, exist_ok=True)
    path.write_text(body)
    path.chmod(0o755)
    return path


def _colcon_build(
    component: Component, root: Path, build_type: str, log: Path | None, jobs: int, dev: bool
) -> None:
    """Build one package with colcon, in the order motion-spec knows and colcon cannot derive.

    One package per call rather than one `colcon build`: coord2b and mj_kdl_wrapper carry no
    package.xml, so colcon has no dependency to order them by. Options come from colcon.meta.
    """
    if shutil.which("colcon") is None:
        raise RuntimeError("required command missing: colcon (this workspace is [ros] workspace)")
    tee(
        [
            "colcon",
            "build",
            "--packages-select",
            component.name,
            "--base-paths",
            str(source_root(root, dev)),
            "--metas",
            str(root / "colcon.meta"),
            "--cmake-args",
            f"-DCMAKE_BUILD_TYPE={build_type}",
        ],
        log=log,
        cwd=root,
        # colcon derives -j from the core count unless MAKEFLAGS already names one.
        env={**os.environ, "MAKEFLAGS": f"-j{jobs} -l{jobs}"},
    )


def write_colcon_meta(root: Path, options: dict[str, tuple[str, ...]]) -> Path:
    """Write the cmake options each package needs, for a colcon workspace to build them itself.

    Without it `colcon build` misses -DMJ_KDL_OROCOS_KDL_FROM_PACKAGE=ON and builds a second
    Orocos KDL, and misses -DENABLE_INSTALL_TARGETS=ON and installs no robif2b.
    """
    import json

    named = {name: {"cmake-args": list(args)} for name, args in options.items() if args}
    path = root / "colcon.meta"
    path.write_text(json.dumps({"names": named}, indent=4) + "\n")
    return path


def remove_environment(root: Path) -> list[Path]:
    """Trash ROOT's environment files, which describe an installation being cleaned away."""
    removed = []
    for name in ENVIRONMENT_FILES:
        path = root / name
        if path.is_file():
            trash_if_present(path)
            removed.append(path)
    return removed


def find_environment(start: Path | None = None) -> Path | None:
    """The environment file a command should run under, or None to inherit the shell.

    ``$MOTION_SPEC_ENV`` names one outright; otherwise the nearest one above START and then
    above the working directory. Discovery is what lets an installed prefix work without a
    flag: `setup --prefix ws` writes the file at the workspace root, and every generation
    underneath finds it from there.
    """
    from motion_spec.config import settings

    configured, _ = settings()
    named = os.environ.get(ENVIRONMENT_VARIABLE)
    if named:
        path = Path(named).expanduser()
        if not path.is_file():
            raise RuntimeError(f"{ENVIRONMENT_VARIABLE} names no file: {path}")
        return path
    # The config's is a workspace default, which `setup` has not necessarily written yet.
    declared = configured.get("workspace", {}).get("environment")
    if declared and Path(declared).expanduser().is_file():
        return Path(declared).expanduser()
    from motion_spec.config import shell

    preferred = f"setup-motion-spec.{shell(configured)}"
    ordered = (preferred, *(name for name in ENVIRONMENT_FILES if name != preferred))
    roots = [start.resolve()] if start else []
    roots.append(Path.cwd())
    for root in roots:
        for parent in (root, *root.parents):
            for name in ordered:
                if (parent / name).is_file():
                    return parent / name
    return None


def capture_environment(script: Path) -> dict[str, str]:
    """The environment a shell is left with after sourcing SCRIPT."""
    shell = "zsh" if script.suffix == ".zsh" else "bash"
    if shutil.which(shell) is None:
        # A .zsh file is still a POSIX export list; the extension should not strand it.
        shell = next((found for found in ("bash", "sh") if shutil.which(found)), None)
        if shell is None:
            raise RuntimeError(f"no shell to source {script} with")
    # stdout redirected so echoes stay out of the dump; env -0 because a value may hold newlines.
    done = subprocess.run(
        [shell, "-c", f". {shlex.quote(str(script))} >/dev/null && env -0"],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )
    if done.returncode:
        detail = done.stderr.decode(errors="replace").strip().splitlines()
        reason = detail[-1] if detail else f"it exited {done.returncode} without saying why"
        raise RuntimeError(f"sourcing {script} failed: {reason}")

    environment = {}
    for entry in done.stdout.split(b"\0"):
        key, separator, value = entry.decode(errors="replace").partition("=")
        if separator:
            environment[key] = value
    return environment


def environment_provenance(script: Path, environment: dict[str, str]) -> dict:
    """What a run archives about the environment it happened in.

    The path alone dates badly -- the file it names can be edited or deleted -- so the
    variables a build and a run actually depend on are recorded with it.
    """
    return {
        "script": str(script),
        "variables": {
            name: environment[name] for name in PROVENANCE_VARIABLES if name in environment
        },
    }
