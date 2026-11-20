# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
# Author: Vamsi Kalagaturu

"""Install what the .repos manifests list, in their order, with the workspace's colcon.meta.

The sources are the operator's: `vcs import` the manifests into src/ first, and each checkout
is built as it stands. Python packages go into one environment, CMake ones into install/, and
src/thirdparty/ holds what colcon must not build.
"""

from __future__ import annotations

import os
import shlex
import shutil
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path
from urllib.request import urlretrieve

from motion_spec.utils import tee, total_memory, trash, usable_cores

# editable install: the manifests are at the clone's root
CHECKOUT = Path(__file__).resolve().parents[2]
MANIFEST = "motion_spec.repos"
REAL_MANIFEST = "motion_spec.real.repos"
COLCON_META = "colcon.meta"
STST_REPOSITORY = "thirdparty/STSTv4"
JARS = {
    "ST4-4.3.4.jar": "https://repo1.maven.org/maven2/org/antlr/ST4/4.3.4/ST4-4.3.4.jar",
    "antlr-runtime-3.5.3.jar": (
        "https://repo1.maven.org/maven2/org/antlr/antlr-runtime/3.5.3/antlr-runtime-3.5.3.jar"
    ),
}
WORKSPACE_VARIABLE = "MOTION_SPEC_WS"
SOURCE_DIRECTORY = "src"
BUILD_DIRECTORY = "build"
INSTALL_DIRECTORY = "install"
COLCON_LOG_DIRECTORY = "log"
GENERATION_DIRECTORY = "generations"
GENERATION_VARIABLE = "MOTION_SPEC_GEN"
THIRDPARTY_DIRECTORY = "thirdparty"
COLCON_IGNORE = "COLCON_IGNORE"
MANAGED = Path("share") / "motion-spec"
BUILD_TYPE = "RelWithDebInfo"
BUILD_TYPE_VARIABLE = "MOTION_SPEC_BUILD_TYPE"
VENV_DIRECTORY = ".venv"


@dataclass(frozen=True)
class Repository:
    """One manifest entry: where it is checked out, where from, and at which version."""

    path: str
    url: str
    version: str

    @property
    def name(self) -> str:
        return Path(self.path).name


def read_manifest(path: Path) -> list[Repository]:
    """PATH's repositories, in the order it lists them, which is the order they install in.

    Only vcstool's fields are read; a pin read wrong is a build of the wrong version, so a
    malformed entry raises instead of being skipped.
    """
    import yaml

    loaded = yaml.safe_load(path.read_text()) or {}
    entries = loaded.get("repositories") if isinstance(loaded, dict) else None
    if not isinstance(entries, dict):
        raise ValueError(f"{path}: no `repositories:` mapping")  # noqa: TRY004 -- file content
    repositories = []
    for key, fields in entries.items():
        if not isinstance(fields, dict):
            raise ValueError(f"{path}: {key} is not a mapping")  # noqa: TRY004 -- file content
        missing = {"type", "url", "version"} - fields.keys()
        if missing:
            raise ValueError(f"{path}: {key} declares no {', '.join(missing)}")
        if fields["type"] != "git":
            raise ValueError(f"{path}: {key} is {fields['type']}; only git is supported")
        repositories.append(Repository(str(key), str(fields["url"]), str(fields["version"])))
    return repositories


def manifest_files(real: bool = False) -> list[Path]:
    """The manifests a setup reads: the base one, then the real layer."""
    return [
        packaged if (packaged := Path(__file__).parent / name).exists() else CHECKOUT / name
        for name in ([MANIFEST, REAL_MANIFEST] if real else [MANIFEST])
    ]


def manifest_in_force(files: list[Path]) -> list[Repository]:
    """Every repository FILES list, file after file; one path listed twice is an error."""
    listed: dict[str, Path] = {}
    repositories = []
    for path in files:
        for repository in read_manifest(path):
            if repository.path in listed:
                raise ValueError(
                    f"{repository.path} is listed in both {listed[repository.path]} and {path}"
                )
            listed[repository.path] = path
            repositories.append(repository)
    return repositories


def shipped_pin(path: str) -> Repository:
    """A repository as the shipped manifests pin it."""
    for repository in manifest_in_force(manifest_files(real=True)):
        if repository.path == path:
            return repository
    raise KeyError(f"{path} is in no shipped manifest")


@dataclass(frozen=True)
class Package:
    """One buildable directory in a checkout, and how it builds: CMake, pip, or both."""

    name: str
    path: Path
    cmake: bool
    python: bool


def package_at(directory: Path) -> Package | None:
    cmake = (directory / "CMakeLists.txt").is_file()
    python = (directory / "pyproject.toml").is_file() or (directory / "setup.py").is_file()
    if not (cmake or python):
        return None
    return Package(package_name(directory, cmake), directory, cmake, python)


def package_name(directory: Path, cmake: bool) -> str:
    """The name colcon and the markers know it by: package.xml, else project(), else pyproject."""
    import re
    import xml.etree.ElementTree as ET

    manifest = directory / "package.xml"
    if manifest.is_file():
        name = (ET.parse(manifest).getroot().findtext("name") or "").strip()
        if name:
            return name
    if cmake:
        found = re.search(
            r"^\s*project\s*\(\s*([A-Za-z0-9_.+-]+)",
            (directory / "CMakeLists.txt").read_text(),
            re.IGNORECASE | re.MULTILINE,
        )
        if found:
            return found.group(1)
    if (directory / "pyproject.toml").is_file():
        import tomllib

        declared = tomllib.loads((directory / "pyproject.toml").read_text())
        return declared.get("project", {}).get("name") or directory.name
    return directory.name


def discover_packages(checkout: Path) -> list[Package]:
    """The packages a checkout holds, as colcon finds them: its root, else each subdirectory.

    Subdirectories come in package.xml dependency order, so python_orocos_kdl follows orocos_kdl.
    """
    import xml.etree.ElementTree as ET

    at_root = package_at(checkout)
    if at_root is not None:
        return [at_root]
    found = [
        package
        for directory in checkout.iterdir()
        if directory.is_dir() and not directory.name.startswith(".")
        if (package := package_at(directory)) is not None
    ]
    dependencies = {}
    for package in found:
        manifest = package.path / "package.xml"
        elements = ET.parse(manifest).getroot().iter() if manifest.is_file() else ()
        dependencies[package] = {
            (element.text or "").strip()
            for element in elements
            if element.tag in {"depend", "build_depend"}
        }
    ordered: list[Package] = []
    pending = found
    while pending:
        names = {package.name for package in pending}
        ready = [p for p in pending if not (dependencies[p] & names)] or pending[:1]
        ordered.extend(ready)
        pending = [p for p in pending if p not in ready]
    return ordered


def cmake_arguments(meta: Path, package: str) -> tuple[str, ...]:
    """The cmake-args META holds for PACKAGE: what colcon would pass it, for a plain build too."""
    import json

    names = json.loads(meta.read_text()).get("names", {})
    return tuple(names.get(package, {}).get("cmake-args", []))


ENVIRONMENT_FILES = ("setup-motion-spec.bash", "setup-motion-spec.zsh")
ENVIRONMENT_VARIABLE = "MOTION_SPEC_ENV"


def workspace(argument: Path | None = None) -> Path:
    """The workspace: --workspace, else $MOTION_SPEC_WS, else the config file, else an error.

    Neither is an error, not a guess: this installs a toolchain.
    """
    from motion_spec.config import CONFIG_FILE, find_config, setting

    chosen = setting("workspace.root", argument).value
    if chosen is None and (path := find_config()) is not None:
        chosen = path.parent
    if chosen is None:
        raise RuntimeError(
            f"no workspace: set {WORKSPACE_VARIABLE}, pass --workspace <path>, or put a "
            f"{CONFIG_FILE} in it"
        )
    root = Path(str(chosen)).expanduser()
    if not root.is_dir():
        raise RuntimeError(f"workspace is not a directory: {root}")
    return root.resolve()


def generations_root() -> Path:
    """Where generations are written: $MOTION_SPEC_GEN, and nowhere else.

    Never the working directory: `rerun`, the dashboard and every provenance location name one
    place.
    """
    from motion_spec.config import setting

    chosen = setting("workspace.generations").value
    if chosen is None:
        raise RuntimeError(f"no generation directory: set {GENERATION_VARIABLE}")
    return Path(str(chosen)).expanduser()


def ignore_thirdparty(root: Path) -> None:
    """Create the third-party subtree, marked so `colcon build` does not descend into it."""
    directory = root / SOURCE_DIRECTORY / THIRDPARTY_DIRECTORY
    directory.mkdir(parents=True, exist_ok=True)
    (directory / COLCON_IGNORE).touch()


def missing_prerequisites(
    repositories: list[Repository], ros: bool = False
) -> tuple[list[str], list[str]]:
    """What must be installed before setup starts: `(apt packages, other requirements)`.

    Only what setup cannot supply itself. Everything it does install carries a
    `motion-spec setup <name>` remedy instead, and is absent here by construction.
    """
    from motion_spec.health import (
        apt_packages,
        check_health,
        installed_ros_distros,
        module_path,
        ros_distro,
    )

    # scoped: only STST needs Ant
    profiles = []
    if any(
        not Path(repository.path).is_relative_to(THIRDPARTY_DIRECTORY)
        for repository in repositories
    ):
        profiles.append("build")
    if any(repository.path == STST_REPOSITORY for repository in repositories):
        profiles.append("codegen")
    if ros and profiles:
        profiles.append("ros")
    packages = (
        apt_packages(
            check_health(
                profiles,
                ("mujoco",) if "build" in profiles else (),
                on_progress=lambda done, dependency: None,
            )
        )
        if profiles
        else []
    )
    others = []
    if ros:
        if ros_distro() is None:
            installed = ", ".join(installed_ros_distros()) or "none under /opt/ros"
            others.append(
                f"a ROS distribution: source one, or set [ros] distro; installed: {installed}"
            )
        if shutil.which("colcon") is None:
            others.append("colcon: apt install python3-colcon-common-extensions")
        # not pyvenv.cfg: system site packages of a non-system interpreter lack apt's
        if module_path("yaml") is None:
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
    usable: bool
    reason: str = ""
    ref: str = ""
    drift: str = ""


def find_stst(path: str | None = None, workspace: str | None = None) -> str | None:
    """Prefer the STST on PATH, falling back to the one the sourced workspace installed.

    PATH and WORKSPACE can describe an environment other than this process's.
    """
    on_path = shutil.which("stst", path=path) if path is not None else shutil.which("stst")
    if on_path:
        return on_path
    if workspace is None and path is None:
        workspace = os.environ.get(WORKSPACE_VARIABLE)
    if not workspace:
        return None
    managed = Path(workspace).expanduser() / INSTALL_DIRECTORY / "bin" / "stst"
    return str(managed) if managed.is_file() else None


def clean_workspace(root: Path) -> list[tuple[Path, Path | None]]:
    """Trash what setup wrote into ROOT: builds, installs, colcon logs and environment files.

    Each trashed path with where the trash keeps it. Sources and generations stay.
    """
    trashed = []
    for name in [BUILD_DIRECTORY, INSTALL_DIRECTORY, COLCON_LOG_DIRECTORY] + list(ENVIRONMENT_FILES):
        path = root / name
        if path.exists() or path.is_symlink():
            trashed.append((path, trash(path)))
    return trashed


def package_installed(name: str, prefix: Path, checkout: Path) -> bool:
    """Whether PREFIX already has package NAME built from CHECKOUT as it stands now.

    Against the checkout's HEAD, so an adopted source at another ref is a no-op until that tree
    moves, rather than rebuilt on every run for not being the pin.
    """
    # uncommitted edits: nothing recorded can prove a match
    if not (checkout / ".git").is_dir() or git(
        checkout, "status", "--porcelain", "--untracked-files=no"
    ):
        return False
    head = git(checkout, "rev-parse", "HEAD")
    marker = prefix / MANAGED / f".{name}-managed"
    return head is not None and marker.is_file() and marker.read_text().split("\n")[0] == head


def stst_installed(root: Path) -> bool:
    """Whether ROOT carries a usable stst: a launcher without its jar is a half-finished one."""
    prefix = root / INSTALL_DIRECTORY
    return (
        (prefix / "bin" / "stst").is_file()
        and (prefix / MANAGED / ".stst-managed").is_file()
        and (root / SOURCE_DIRECTORY / STST_REPOSITORY / "build" / "jar" / "stst.jar").is_file()
    )


def install_stst(
    root: Path, state: SourceState, force: bool = False, log: Path | None = None
) -> Path:
    """Build the pinned STSTv4 from its checkout and install its launcher under ROOT/install/bin."""
    prefix = root / INSTALL_DIRECTORY
    launcher = prefix / "bin" / "stst"
    marker = prefix / MANAGED / ".stst-managed"
    if not force and stst_installed(root):
        return launcher

    missing = [command for command in ("ant", "java") if shutil.which(command) is None]
    if missing:
        raise RuntimeError(
            f"required command{'s' if len(missing) > 1 else ''} missing: {', '.join(missing)}"
        )

    if not state.usable:
        raise RuntimeError(f"{state.path} {state.reason}")
    # a half-built install must not read as installed
    marker.unlink(missing_ok=True)
    source = state.path
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
    marker.parent.mkdir(parents=True, exist_ok=True)
    marker.write_text(f"{state.ref}\n")
    return launcher


def git(repository: Path, *arguments: str) -> str | None:
    """The output of a read-only git command, or None when it fails."""
    done = subprocess.run(
        ["git", "-C", str(repository), *arguments],
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
        text=True,
        check=False,
    )
    return done.stdout.strip() if done.returncode == 0 else None


def source_state(pinned: Repository, root: Path) -> SourceState:
    """What PINNED's checkout holds now, read-only: the commit to record and any drift."""
    repository = root / SOURCE_DIRECTORY / pinned.path
    if not (repository / ".git").is_dir():
        reason = (
            "is not a git checkout"
            if repository.exists()
            else "is missing; vcs import the manifests into src/ first"
        )
        return SourceState(repository, False, reason)
    head = git(repository, "rev-parse", "HEAD")
    if head is None:
        return SourceState(repository, False, "is a git checkout with no commit")
    if git(repository, "status", "--porcelain", "--untracked-files=no"):
        return SourceState(
            repository,
            True,
            ref=head,
            drift="has uncommitted changes; building them, and rebuilding on every run",
        )
    # remote first: a local branch may be stale
    commit = git(
        repository, "rev-parse", "--verify", "--quiet", f"origin/{pinned.version}^{{commit}}"
    ) or git(repository, "rev-parse", "--verify", "--quiet", f"{pinned.version}^{{commit}}")
    # an existing checkout is built as it stands; drift is reported, never reset
    if commit is None or head != commit:
        described = git(repository, "describe", "--all", "--always", "HEAD") or head
        return SourceState(
            repository,
            True,
            ref=head,
            drift=f"is at {described}, not the pinned {pinned.version}; building it as it stands",
        )
    return SourceState(repository, True, ref=head)


def cmake_prefix_path(prefix: Path) -> str:
    """PREFIX first, then whatever the caller's environment already pointed at.

    Dropping the inherited value would hide a dependency installed by apt or by another
    prefix, and the configure would fail on a machine where the library is in fact there.
    """
    inherited = os.environ.get("CMAKE_PREFIX_PATH", "")
    return f"{prefix}{os.pathsep}{inherited}" if inherited else str(prefix)


def install_package(
    package: Package,
    state: SourceState,
    root: Path,
    python: Path,
    *,
    fresh: bool = False,
    build_type: str = BUILD_TYPE,
    log: Path | None = None,
    ros: bool = False,
    jobs: int | None = None,
    dev: bool = False,
) -> None:
    """Build PACKAGE from its prepared checkout and install it into ROOT/install and PYTHON's env.

    CMake first when it has a CMakeLists, with the workspace colcon.meta's arguments either
    way; then pip when it is a Python package too, editable under --dev. FRESH clears the CMake
    cache first.
    """
    prefix = root / INSTALL_DIRECTORY
    marker = prefix / MANAGED / f".{package.name}-managed"
    marker.unlink(missing_ok=True)
    meta = root / COLCON_META
    if not meta.exists():
        shutil.copyfile(Path(__file__).parent / COLCON_META, meta)
    configured = cmake_arguments(meta, package.name)
    interpreter = extension_options(python, log) if package.cmake else ()

    if package.cmake and ros:
        # one package per call: coord2b and robif2b have no package.xml to order by
        parallel = build_jobs(jobs)
        if shutil.which("colcon") is None:
            raise RuntimeError(
                "required command missing: colcon (this workspace is [ros] workspace)"
            )
        tee(
            [
                "colcon",
                "build",
                "--packages-select",
                package.name,
                "--base-paths",
                str(root / SOURCE_DIRECTORY),
                "--build-base",
                str(root / BUILD_DIRECTORY),
                "--install-base",
                str(prefix),
                "--metas",
                str(meta),
                *(["--cmake-clean-cache"] if fresh else []),
                "--cmake-args",
                f"-DCMAKE_BUILD_TYPE={build_type}",
                *interpreter,
            ],
            log=log,
            cwd=root,
            # colcon derives -j from the core count unless MAKEFLAGS sets it
            env={**os.environ, "MAKEFLAGS": f"-j{parallel} -l{parallel}"},
        )
    elif package.cmake:
        build = root / BUILD_DIRECTORY / package.name
        if fresh:
            (build / "CMakeCache.txt").unlink(missing_ok=True)
            if (build / "CMakeFiles").exists():
                shutil.rmtree(build / "CMakeFiles")
        cmake = shutil.which("cmake")
        tee(
            [
                cmake,
                "-S",
                str(package.path),
                "-B",
                str(build),
                f"-DCMAKE_INSTALL_PREFIX={prefix}",
                f"-DCMAKE_PREFIX_PATH={cmake_prefix_path(prefix)}",
                f"-DCMAKE_BUILD_TYPE={build_type}",
                *configured,
                *interpreter,
            ],
            log=log,
        )
        # a bare --parallel is an unlimited make -j
        tee([cmake, "--build", str(build), "--parallel", str(build_jobs(jobs))], log=log)
        tee([cmake, "--install", str(build)], log=log)

    if package.python:
        defines = (
            (*configured, *interpreter, f"-DCMAKE_PREFIX_PATH={cmake_prefix_path(prefix)}")
            if package.cmake
            else ()
        )
        pip_install(package.path, log, dev, python, ros and package.cmake, defines)

    # the ref built, not the pin
    marker.parent.mkdir(parents=True, exist_ok=True)
    marker.write_text(f"{state.ref}\n")


def installer(python: Path | None = None) -> list[str]:
    """How to install a Python package into PYTHON's environment, this one by default.

    A virtual environment uv made has no pip in it at all, so `python -m pip` there fails with
    "No module named pip"; uv installs into the interpreter it is pointed at instead.
    """
    interpreter = str(python or sys.executable)
    has_pip = subprocess.run(
        [interpreter, "-m", "pip", "--version"], capture_output=True, check=False
    )
    if has_pip.returncode == 0:
        return [interpreter, "-m", "pip", "install"]
    uv = shutil.which("uv")
    if uv is None:
        raise RuntimeError(
            f"neither pip nor uv is available to install with: {interpreter} has no pip "
            f"module, and no `uv` is on PATH"
        )
    return [uv, "pip", "install", "--python", interpreter]


# PyKDL must share mj_kdl_wrapper's pybind11 to share its types
PYBIND11_REQUIREMENT = "pybind11>=2.13"
EXTENSION_PROBE = (
    "import pybind11, sysconfig; print(pybind11.get_cmake_dir()); "
    "print(sysconfig.get_paths()['platlib'])"
)


def extension_options(python: Path, log: Path | None = None) -> tuple[str, ...]:
    """The interpreter, pybind11 and site-packages a CPython extension is built for.

    Passed to every CMake package: a package with no extension ignores them.
    """
    probe = [str(python), "-c", EXTENSION_PROBE]
    done = subprocess.run(probe, capture_output=True, text=True, check=False)
    if done.returncode:
        tee([*installer(python), PYBIND11_REQUIREMENT], log=log)
        done = subprocess.run(probe, capture_output=True, text=True, check=True)
    cmake_dir, platlib = done.stdout.split("\n")[:2]
    return (
        f"-DPython3_EXECUTABLE={python}",
        f"-Dpybind11_DIR={cmake_dir}",
        f"-DPYTHON_SITE_PACKAGES_INSTALL_DIR={platlib}",
    )


def target_environment(
    root: Path, ros: bool = False, dev: bool = False, log: Path | None = None
) -> Path:
    """The interpreter every Python package goes into: the active venv, else ROOT/.venv.

    A new ROOT/.venv gets motion-spec itself too, so the environment file points at one
    environment that holds the whole toolchain.
    """
    active = os.environ.get("VIRTUAL_ENV")
    if active:
        return Path(active) / "bin" / "python"
    venv = root / VENV_DIRECTORY
    python = venv / "bin" / "python"
    if not python.is_file():
        tee(
            [sys.executable, "-m", "venv", *(["--system-site-packages"] if ros else []), str(venv)],
            log=log,
        )
    has_self = subprocess.run(
        [str(python), "-c", "import motion_spec"], capture_output=True, check=False
    )
    if has_self.returncode:
        if not (CHECKOUT / "pyproject.toml").is_file():
            raise RuntimeError(
                f"no virtual environment is active and motion-spec runs from no checkout to "
                f"install into {venv}; activate the environment motion-spec is installed in"
            )
        pip_install(CHECKOUT, log, dev, python)
    return python


def pip_install(
    source: Path,
    log: Path | None,
    editable: bool,
    python: Path | None = None,
    ros: bool = False,
    defines: tuple[str, ...] = (),
) -> None:
    """Install a checkout; EDITABLE points site-packages back at it.

    ROS is for an extension whose cmake finds ament: ament's scripts import ament_package,
    which reaches the interpreter over PYTHONPATH from the sourced distro, and pip replaces
    PYTHONPATH with its own for an isolated build. Isolation off means pip installs no build
    backend at all, for this checkout or for anything it resolves, so the checkout's own
    requirements go in first and the flag stays off everything that does not need it.
    """
    arguments = ["--editable", str(source)] if editable else [str(source)]
    for define in defines:
        arguments += ["-C", f"cmake.define.{define.removeprefix('-D')}"]
    if ros:
        manifest = source / "pyproject.toml"
        if manifest.is_file():
            import tomllib

            requires = (
                tomllib.loads(manifest.read_text()).get("build-system", {}).get("requires", [])
            )
            if requires:
                tee([*installer(python), *requires], log=log)
        arguments = ["--no-build-isolation", *arguments]
    tee([*installer(python), *arguments], log=log)


def write_environment(root: Path, ros: bool, python: Path | None = None) -> Path:
    """Write ROOT's environment file for $SHELL, pointing at its install prefix.

    A colcon workspace sources ROS and its overlay; any other exports the install's paths.
    """
    from motion_spec.config import configured, shell

    prefix = root / INSTALL_DIRECTORY
    using = shell()
    path = root / f"setup-motion-spec.{using}"
    paths = 'export PATH="$MOTION_SPEC_PREFIX/bin${PATH:+:$PATH}"\n'
    if ros:
        from motion_spec.health import ros_distro

        distro = ros_distro()
        if distro is None:
            raise RuntimeError(
                "no ROS distribution: set [ros] distro, or source one, for a [ros] workspace"
            )
        overlay = shlex.quote(str(prefix / f"setup.{using}"))
        # sourcing an overlay twice repeats its paths
        sourcing = (
            f'[ "${{ROS_DISTRO:-}}" = {distro} ] || . /opt/ros/{distro}/setup.{using}\n'
            f'case ":${{COLCON_PREFIX_PATH:-}}:" in *:{prefix}:*) ;; *)'
            f" [ -f {overlay} ] && . {overlay} ;; esac\n"
        )
    else:
        sourcing = ""
        paths += (
            'export CMAKE_PREFIX_PATH="$MOTION_SPEC_PREFIX${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"\n'
            'export LD_LIBRARY_PATH="$MOTION_SPEC_PREFIX/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"\n'
        )
    venv = python.parent.parent if python else Path(sys.prefix)
    activate = venv / "bin" / "activate"
    activation = (
        f'[ "${{VIRTUAL_ENV:-}}" = {shlex.quote(str(venv))} ] || . {shlex.quote(str(activate))}\n'
        if (venv / "pyvenv.cfg").is_file() and activate.is_file()
        else ""
    )
    declared = configured("workspace.generations", root)
    generations = Path(str(declared)) if declared else root / GENERATION_DIRECTORY
    body = (
        "# Written by `motion-spec setup`. Source it before generating, building or running.\n"
        + activation
        + sourcing
        + f"export {WORKSPACE_VARIABLE}={shlex.quote(str(root))}\n"
        f"export {GENERATION_VARIABLE}={shlex.quote(str(generations))}\n"
        f"export {ENVIRONMENT_VARIABLE}={shlex.quote(str(path))}\n"
        f"export MOTION_SPEC_PREFIX={shlex.quote(str(prefix))}\n" + paths
    )
    root.mkdir(parents=True, exist_ok=True)
    path.write_text(body)
    path.chmod(0o755)
    return path


def find_environment(start: Path | None = None) -> Path | None:
    """The environment file a command should run under, or None to inherit the shell.

    ``$MOTION_SPEC_ENV`` names one outright, then the config file; otherwise the nearest one
    above START and then above the working directory, so a generation under a workspace finds
    the workspace's file without a flag.
    """
    from motion_spec.config import setting, shell

    chosen = setting("workspace.environment")
    if chosen.value is not None:
        path = Path(str(chosen.value)).expanduser()
        if path.is_file():
            return path
        if chosen.source == "environment":
            raise RuntimeError(f"{ENVIRONMENT_VARIABLE} names no file: {path}")
    preferred = f"setup-motion-spec.{shell()}"
    ordered = [preferred] + [name for name in ENVIRONMENT_FILES if name != preferred]
    for root in [start.resolve(), Path.cwd()] if start else [Path.cwd()]:
        for parent in (root, *root.parents):
            for name in ordered:
                if (parent / name).is_file():
                    return parent / name
    return None


def capture_environment(script: Path) -> dict[str, str]:
    """The environment a shell is left with after sourcing SCRIPT."""
    shell = "zsh" if script.suffix == ".zsh" else "bash"
    if shutil.which(shell) is None:
        shell = next((found for found in ("bash", "sh") if shutil.which(found)), None)
        if shell is None:
            raise RuntimeError(f"no shell to source {script} with")
    # env -0: a value may hold newlines
    done = subprocess.run(
        [shell, "-c", f". {shlex.quote(str(script))} >/dev/null && env -0"],
        capture_output=True,
        check=False,
    )
    if done.returncode:
        detail = done.stderr.decode(errors="replace").strip().splitlines()
        reason = detail[-1] if detail else f"it exited {done.returncode} without saying why"
        raise RuntimeError(f"sourcing {script} failed: {reason}")

    entries = (entry.decode(errors="replace").partition("=") for entry in done.stdout.split(b"\0"))
    return {key: value for key, separator, value in entries if separator}
