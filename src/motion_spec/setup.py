# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
# Author: Vamsi Kalagaturu

"""Install what the .repos manifests list, in their order, with the workspace's colcon.meta.

`vcs import` fetches what the workspace has no checkout of; a checkout already there is built
as it stands and never moved. Python packages go into one environment, CMake ones into
install/, and src/thirdparty/ holds what colcon must not build.
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
# The layout `vcs import` produces, so either route gives the same workspace.
SOURCE_DIRECTORY = "src"
BUILD_DIRECTORY = "build"
INSTALL_DIRECTORY = "install"
# colcon's, not motion-spec's: `colcon build` writes it beside build/ and install/.
COLCON_LOG_DIRECTORY = "log"
GENERATION_DIRECTORY = "generations"
GENERATION_VARIABLE = "MOTION_SPEC_GEN"
# For sources colcon has no business building; `setup` marks it with a COLCON_IGNORE.
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


def shipped(filename: str) -> Path:
    """A data file that ships inside the motion_spec package."""
    return Path(__file__).parent / filename


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
            raise ValueError(f"{path}: {key} declares no {', '.join(sorted(missing))}")
        if fields["type"] != "git":
            raise ValueError(f"{path}: {key} is {fields['type']}; only git is supported")
        repositories.append(Repository(str(key), str(fields["url"]), str(fields["version"])))
    return repositories


def manifest_files(repos: tuple[Path, ...] = (), real: bool = False) -> list[Path]:
    """The manifests a setup reads: REPOS in place of the shipped one, then the real layer."""
    files = list(repos) or [shipped(MANIFEST)]
    if real:
        files.append(shipped(REAL_MANIFEST))
    return files


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


def is_thirdparty(repository: Repository) -> bool:
    """Whether colcon must not build it: a Python package or STST, by where it is listed."""
    return repository.path.startswith(f"{THIRDPARTY_DIRECTORY}/")


@dataclass(frozen=True)
class Package:
    """One buildable directory in a checkout, and how it builds: CMake, pip, or both."""

    name: str
    path: Path
    cmake: bool
    python: bool


def _package_at(directory: Path) -> Package | None:
    cmake = (directory / "CMakeLists.txt").is_file()
    python = (directory / "pyproject.toml").is_file() or (directory / "setup.py").is_file()
    if not (cmake or python):
        return None
    return Package(_package_name(directory, cmake, python), directory, cmake, python)


def _package_name(directory: Path, cmake: bool, python: bool) -> str:
    """The name colcon and the markers know it by: package.xml, else project(), else pyproject."""
    import re

    manifest = directory / "package.xml"
    if manifest.is_file():
        found = re.search(r"<name>\s*([^<\s]+)\s*</name>", manifest.read_text())
        if found:
            return found.group(1)
    if cmake:
        found = re.search(
            r"^\s*project\s*\(\s*([A-Za-z0-9_.+-]+)",
            (directory / "CMakeLists.txt").read_text(),
            re.IGNORECASE | re.MULTILINE,
        )
        if found:
            return found.group(1)
    if python and (directory / "pyproject.toml").is_file():
        import tomllib

        declared = tomllib.loads((directory / "pyproject.toml").read_text())
        name = declared.get("project", {}).get("name")
        if name:
            return name
    return directory.name


def _package_dependencies(package: Package) -> set[str]:
    import re

    manifest = package.path / "package.xml"
    if not manifest.is_file():
        return set()
    return set(re.findall(r"<(?:build_)?depend>\s*([^<\s]+)\s*<", manifest.read_text()))


def discover_packages(checkout: Path) -> list[Package]:
    """The packages a checkout holds, as colcon finds them: its root, else each subdirectory.

    Subdirectories come in package.xml dependency order, so python_orocos_kdl follows orocos_kdl.
    """
    at_root = _package_at(checkout)
    if at_root is not None:
        return [at_root]
    found = [
        package
        for directory in sorted(checkout.iterdir())
        if directory.is_dir() and not directory.name.startswith(".")
        if (package := _package_at(directory)) is not None
    ]
    ordered: list[Package] = []
    pending = list(found)
    while pending:
        names = {package.name for package in pending}
        ready = [p for p in pending if not (_package_dependencies(p) & names)] or pending[:1]
        ordered.extend(ready)
        pending = [p for p in pending if p not in ready]
    return ordered


def workspace_colcon_meta(root: Path) -> Path:
    """ROOT's colcon.meta, seeded from the shipped one the first time and edited by hand after."""
    path = root / COLCON_META
    if not path.exists():
        shutil.copyfile(shipped(COLCON_META), path)
    return path


def cmake_arguments(meta: Path, package: str) -> tuple[str, ...]:
    """The cmake-args META holds for PACKAGE: what colcon would pass it, for a plain build too."""
    import json

    names = json.loads(meta.read_text()).get("names", {})
    return tuple(names.get(package, {}).get("cmake-args", []))


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


def source_root(root: Path) -> Path:
    """Where every source is checked out, whatever the mode."""
    return root / SOURCE_DIRECTORY


def thirdparty_directory(root: Path) -> Path:
    """The subtree colcon leaves alone: what it could not build, or must not."""
    return source_root(root) / THIRDPARTY_DIRECTORY


def source_directory(root: Path, repository: str) -> Path:
    """Where a workspace keeps one repository's source, as the manifest spells its path."""
    return source_root(root) / repository


def _ignore_thirdparty(root: Path) -> Path:
    """Create the third-party subtree, marked so `colcon build` does not descend into it."""
    directory = thirdparty_directory(root)
    directory.mkdir(parents=True, exist_ok=True)
    marker = directory / COLCON_IGNORE
    if not marker.exists():
        marker.touch()
    return directory


def build_directory(root: Path, name: str) -> Path:
    """Where a workspace builds one component."""
    return root / BUILD_DIRECTORY / name


def required_profiles(repositories: list[Repository], ros: bool = False) -> tuple[str, ...]:
    """The health profiles REPOSITORIES need before any of them can be installed.

    Scoped, so `setup coord2b` is not refused for want of the Ant that only STST uses.
    """
    profiles = set()
    if any(repository.path == STST_REPOSITORY for repository in repositories):
        profiles.add("codegen")
    if any(not is_thirdparty(repository) for repository in repositories):
        profiles.add("build")
    return (*sorted(profiles), *(("ros",) if ros and profiles else ()))


def missing_prerequisites(
    repositories: list[Repository], ros: bool = False, targets: tuple[str, ...] = ("mujoco",)
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

    profiles = required_profiles(repositories, ros)
    # The ROS checks run even for a selection with no profile at all: without them a
    # Python-only `setup` in a colcon workspace fails on the distro only at the last step.
    packages = (
        apt_packages(check_health(profiles, targets if "build" in profiles else ()))
        if profiles
        else []
    )
    others = []
    if repositories and shutil.which("vcs") is None:
        others.append("vcs: apt install python3-vcstool")
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
    # The commit actually in the tree, which is what the marker records.
    ref: str = ""
    # Set when that commit is not the pinned one, for the caller to report.
    drift: str = ""


def find_stst(path: str | None = None, workspace: str | None = None) -> str | None:
    """Prefer the STST on PATH, falling back to the one the sourced workspace installed.

    PATH and WORKSPACE can describe an environment other than this process's.
    """
    on_path = shutil.which("stst", path=path) if path is not None else shutil.which("stst")
    if on_path:
        return on_path
    # Derived, not a variable of its own: one more to keep in step is one more to disagree.
    if workspace is None and path is None:
        workspace = os.environ.get(WORKSPACE_VARIABLE)
    managed = install_prefix(Path(workspace).expanduser()) / "bin" / "stst" if workspace else None
    return str(managed) if managed and managed.is_file() else None


@dataclass(frozen=True)
class Removed:
    """One thing a clean took: what it was, and where the trash keeps it."""

    what: str
    # None when deleted outright, or trashed where the home trash cannot say.
    to: Path | None
    deleted: bool = False


def _trash_into(path: Path, what: str, removed: list[Removed]) -> None:
    if path.exists() or path.is_symlink():
        removed.append(Removed(what, trash(path)))


def _delete_marker(marker: Path, removed: list[Removed]) -> None:
    if marker.is_file():
        marker.unlink()
        removed.append(Removed(str(marker), None, deleted=True))


def remove_stst(root: Path, prefix: Path | None = None) -> list[Removed]:
    """Remove an STST installation this tool made. Sources are never removed, whoever made them."""
    prefix = prefix or install_prefix(root)
    launcher = prefix / "bin" / "stst"
    marker = install_marker("stst", prefix)
    if not (launcher.exists() or marker.exists()):
        return []
    if not marker.is_file():
        raise RuntimeError(f"refusing to clean an unmanaged STST installation under {prefix}")
    removed: list[Removed] = []
    _trash_into(launcher, str(launcher), removed)
    _delete_marker(marker, removed)
    try:
        (prefix / MANAGED).rmdir()
    except OSError:
        pass
    return removed


def package_installed(name: str, prefix: Path, checkout: Path) -> bool:
    """Whether PREFIX already has package NAME built from CHECKOUT as it stands now.

    Against the checkout's HEAD, so an adopted source at another ref is a no-op until that tree
    moves, rather than rebuilt on every run for not being the pin.
    """
    # Edits are not in any commit, so nothing recorded can prove the install matches.
    if not (checkout / ".git").is_dir() or _dirty(checkout):
        return False
    head = _git(checkout, "rev-parse", "HEAD")
    marker = install_marker(name, prefix)
    return head is not None and marker.is_file() and marker.read_text().split("\n")[0] == head


def stst_installed(root: Path, prefix: Path | None = None) -> bool:
    """Whether PREFIX carries a usable stst: a launcher without its jar is a half-finished one."""
    prefix = prefix or install_prefix(root)
    marker = install_marker("stst", prefix)
    if not (prefix / "bin" / "stst").is_file() or not marker.is_file():
        return False
    if marker.read_text().splitlines()[:1] == ["installing"]:
        return False
    return (source_directory(root, STST_REPOSITORY) / "build" / "jar" / "stst.jar").is_file()


def install_stst(
    root: Path,
    state: SourceState,
    prefix: Path | None = None,
    force: bool = False,
    log: Path | None = None,
) -> Path:
    """Build the pinned STSTv4 from its checkout and install its launcher under PREFIX/bin."""
    prefix = prefix or install_prefix(root)
    launcher = prefix / "bin" / "stst"
    marker = install_marker("stst", prefix)
    if not force and stst_installed(root, prefix):
        return launcher

    missing = [command for command in ("ant", "java") if shutil.which(command) is None]
    if missing:
        raise RuntimeError(
            f"required command{'s' if len(missing) > 1 else ''} missing: {', '.join(missing)}"
        )

    if not state.usable:
        raise RuntimeError(f"{state.path} {state.reason}")
    marker.parent.mkdir(parents=True, exist_ok=True)
    origin = _origin(_recorded_origin(marker), state)
    marker.write_text(f"installing\n{origin}\n")
    # Whatever tree the checkout is in: the launcher must name the jar ant just built.
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
    marker.write_text(f"{state.ref}\n{origin}\n")
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
        (
            line.split("=", 1)[1].strip()
            for line in body.splitlines()
            if line.startswith("STST_HOME=")
        ),
        None,
    )
    if home is None:
        return None
    return other, (Path(shlex.split(home)[0]) / "build" / "jar" / "stst.jar").is_file()


def is_installed(name: str, prefix: Path) -> bool:
    """Whether PREFIX carries an installation of package NAME this tool made."""
    marker = install_marker(name, prefix)
    # The first line only: a half-finished install records the origin under "installing".
    return marker.is_file() and marker.read_text().splitlines()[:1] != ["installing"]


def _manifest_files(manifest: Path, prefix: Path) -> list[Path]:
    """The files a build's install manifest names that are still inside PREFIX."""
    if not manifest.is_file():
        return []
    inside = []
    for line in manifest.read_text().splitlines():
        installed = Path(line.strip())
        if not line.strip() or not (installed.exists() or installed.is_symlink()):
            continue
        try:
            installed.relative_to(prefix)
            installed.resolve().relative_to(prefix.resolve())
        except ValueError:
            # A CMake manifest can name files outside the selected install prefix.
            continue
        inside.append(installed)
    return inside


def installed_files(name: str, root: Path, prefix: Path | None = None) -> list[Path]:
    """What package NAME's build put inside PREFIX, so a prompt can say what removing it takes."""
    prefix = prefix or install_prefix(root)
    manifest = build_directory(root, name) / "install_manifest.txt"
    return _manifest_files(manifest, prefix)


def _trash_installed(manifest: Path, prefix: Path, label: str, removed: list[Removed]) -> None:
    """Trash the files an install left in PREFIX, as one entry rather than hundreds."""
    staging = prefix.parent / f".removed-{label}"
    files = _manifest_files(manifest, prefix)
    for installed in files:
        destination = staging / installed.relative_to(prefix)
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.move(str(installed), str(destination))
    if files:
        _trash_into(staging, f"{len(files)} installed files from {prefix}", removed)


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


def install_marker(name: str, prefix: Path) -> Path:
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


def _dirty(repository: Path) -> bool:
    """Whether tracked files differ from HEAD, so what is there is not any recorded commit."""
    return bool(_git(repository, "status", "--porcelain", "--untracked-files=no"))


def _pinned_commit(repository: Path, ref: str) -> str | None:
    """The commit REF names, preferring the remote: a local branch still points where it did."""
    for candidate in (f"origin/{ref}", ref):
        commit = _git(repository, "rev-parse", "--verify", "--quiet", f"{candidate}^{{commit}}")
        if commit:
            return commit
    return None


def import_sources(
    files: list[Path], listed: list[Repository], root: Path, log: Path | None = None
) -> set[str]:
    """`vcs import --skip-existing` each manifest in FILES into ROOT/src.

    A checkout already there is the operator's, and vcs leaves it as it is. Returns the paths
    that had none before, which are the ones setup cloned.
    """
    if shutil.which("vcs") is None:
        raise RuntimeError("required command missing: vcs (apt install python3-vcstool)")
    missing = {r.path for r in listed if not (source_directory(root, r.path) / ".git").is_dir()}
    target = source_root(root)
    target.mkdir(parents=True, exist_ok=True)
    for path in files:
        tee(["vcs", "import", "--skip-existing", "--input", str(path), str(target)], log=log)
    return missing


def source_state(pinned: Repository, root: Path, imported: bool = False) -> SourceState:
    """What PINNED's checkout holds now, read-only: the commit to record and any drift."""
    repository = source_directory(root, pinned.path)
    if not (repository / ".git").is_dir():
        reason = "is not a git checkout" if repository.exists() else "was not imported"
        return SourceState(repository, False, False, reason)
    if imported:
        # The commit, not the branch it was named by: the marker is compared against HEAD.
        return SourceState(repository, True, True, ref=_git(repository, "rev-parse", "HEAD") or "")
    commit = _pinned_commit(repository, pinned.version)
    head = _git(repository, "rev-parse", "HEAD")
    if head is None:
        return SourceState(repository, False, False, "is a git checkout with no commit")
    if _dirty(repository):
        return SourceState(
            repository,
            False,
            True,
            ref=head,
            drift="has uncommitted changes; building them, and rebuilding on every run",
        )
    # A checkout already here is the operator's answer to which version this workspace wants.
    # It is built as it stands and the ref is recorded, so nothing silently returns it to the
    # pin; drift is said out loud instead, because a fork that merely compiles is the danger.
    if commit is None or head != commit:
        described = _git(repository, "describe", "--all", "--always", "HEAD") or head
        return SourceState(
            repository,
            False,
            True,
            ref=head,
            drift=f"is at {described}, not the pinned {pinned.version}; building it as it stands",
        )
    return SourceState(repository, False, True, ref=head)


def _cmake_prefix_path(prefix: Path) -> str:
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
    prefix: Path,
    python: Path,
    *,
    clear_cache: bool = False,
    build_type: str = BUILD_TYPE,
    log: Path | None = None,
    extra: tuple[str, ...] = (),
    ros: bool = False,
    jobs: int | None = None,
    dev: bool = False,
) -> None:
    """Build PACKAGE from its prepared checkout and install it into PREFIX and PYTHON's env.

    CMake first when it has a CMakeLists, with the workspace colcon.meta's arguments either
    way; then pip when it is a Python package too, editable under --dev.
    """
    marker = install_marker(package.name, prefix)
    origin = _origin(_recorded_origin(marker), state)
    marker.parent.mkdir(parents=True, exist_ok=True)
    # Before the build: a half-built installation is still this tool's.
    marker.write_text(f"installing\n{origin}\n")
    meta = workspace_colcon_meta(root)
    configured = cmake_arguments(meta, package.name)
    interpreter = extension_options(python, log) if package.cmake else ()

    if package.cmake and ros:
        # colcon reads configured from --metas itself; the rest is this machine and this run.
        _colcon_build(
            package.name,
            root,
            prefix,
            build_type,
            log,
            build_jobs(jobs),
            clear_cache,
            (*interpreter, *extra),
        )
    elif package.cmake:
        build = build_directory(root, package.name)
        if clear_cache:
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
                f"-DCMAKE_PREFIX_PATH={_cmake_prefix_path(prefix)}",
                f"-DCMAKE_BUILD_TYPE={build_type}",
                *configured,
                *interpreter,
                *extra,
            ],
            log=log,
        )
        # A bare --parallel is make -j: unlimited, and it overrides the env and MAKEFLAGS too.
        tee([cmake, "--build", str(build), "--parallel", str(build_jobs(jobs))], log=log)
        tee([cmake, "--install", str(build)], log=log)

    if package.python:
        # The same -D options the CMake build got, so an extension links what the build linked.
        defines = (
            (*configured, *interpreter, *extra, f"-DCMAKE_PREFIX_PATH={_cmake_prefix_path(prefix)}")
            if package.cmake
            else ()
        )
        _pip_install(package.path, log, dev, python, ros and package.cmake, defines)

    # What was built, not what was wanted: an adopted checkout at another ref is recorded as
    # that ref, so a rerun compares against the tree rather than the pin it does not match.
    marker.write_text(f"{state.ref}\n{origin}\n")


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


# mj_kdl_wrapper's bindings build with this; PyKDL must share its pybind11 to share its types.
PYBIND11_REQUIREMENT = "pybind11>=2.13"
_EXTENSION_PROBE = (
    "import pybind11, sysconfig; print(pybind11.get_cmake_dir()); "
    "print(sysconfig.get_paths()['platlib'])"
)


def extension_options(python: Path, log: Path | None = None) -> tuple[str, ...]:
    """The interpreter, pybind11 and site-packages a CPython extension is built for.

    Passed to every CMake package: a package with no extension ignores them.
    """
    probe = [str(python), "-c", _EXTENSION_PROBE]
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


def own_source() -> Path | None:
    """motion-spec's own checkout when it runs from one, for a new environment to install."""
    candidate = Path(__file__).resolve().parents[2]
    return candidate if (candidate / "pyproject.toml").is_file() else None


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
        source = own_source()
        if source is None:
            raise RuntimeError(
                f"no virtual environment is active and motion-spec runs from no checkout to "
                f"install into {venv}; activate the environment motion-spec is installed in"
            )
        _pip_install(source, log, dev, python)
    return python


def _build_requirements(source: Path) -> list[str]:
    """What PEP 518 says this checkout needs to build, since nothing else will install them."""
    manifest = source / "pyproject.toml"
    if not manifest.is_file():
        return []
    import tomllib

    return tomllib.loads(manifest.read_text()).get("build-system", {}).get("requires", [])


def _pip_install(
    source: Path,
    log: Path | None,
    editable: bool,
    python: Path | None = None,
    ros: bool = False,
    defines: tuple[str, ...] = (),
) -> None:
    """Install a checkout. Editable points site-packages back at it, so `--clean` would orphan
    the installation along with the source it removes.

    ROS is for an extension whose cmake finds ament: ament's scripts import ament_package,
    which reaches the interpreter over PYTHONPATH from the sourced distro, and pip replaces
    PYTHONPATH with its own for an isolated build. Isolation off means pip installs no build
    backend at all, for this checkout or for anything it resolves, so the checkout's own
    requirements go in first and the flag stays off everything that does not need it.
    """
    arguments = ["--editable", str(source)] if editable else [str(source)]
    # The same -D options the CMake build got, so the extension links what the build linked.
    for define in defines:
        arguments += ["-C", f"cmake.define.{define.removeprefix('-D')}"]
    if ros:
        requires = _build_requirements(source)
        if requires:
            tee([*installer(python), *requires], log=log)
        arguments = ["--no-build-isolation", *arguments]
    tee([*installer(python), *arguments], log=log)


def remove_package(name: str, root: Path, prefix: Path | None = None) -> list[Removed]:
    """Remove package NAME's installation this tool made: its installed files, build and marker.

    Never the source. A checkout is the operator's, whoever cloned it.
    """
    prefix = prefix or install_prefix(root)
    marker = install_marker(name, prefix)
    build = build_directory(root, name)
    if not (build.exists() or marker.exists()):
        return []
    if not marker.is_file():
        raise RuntimeError(f"refusing to clean an unmanaged {name} installation under {prefix}")
    removed: list[Removed] = []
    # The only record of what landed in PREFIX; without it find_package keeps finding it.
    _trash_installed(build / "install_manifest.txt", prefix, f"{name}-install", removed)
    _trash_into(build, str(build), removed)
    _delete_marker(marker, removed)
    for directory in (prefix / MANAGED, root / BUILD_DIRECTORY):
        try:
            directory.rmdir()
        except OSError:
            pass
    return removed


def _activation(python: Path | None = None) -> str:
    """Activate PYTHON's environment, this one by default, unless the shell is already in it.

    Sourcing the file is the one step between a fresh shell and a working workspace, and a
    `motion-spec: command not found` right after it helps nobody.
    """
    venv = python.parent.parent if python else Path(sys.prefix)
    activate = venv / "bin" / "activate"
    if not (venv / "pyvenv.cfg").is_file() or not activate.is_file():
        return ""
    quoted = shlex.quote(str(activate))
    return f'[ "${{VIRTUAL_ENV:-}}" = {shlex.quote(str(venv))} ] || . {quoted}\n'


def write_environment(
    root: Path, prefix: Path | None = None, ros: bool | None = None, python: Path | None = None
) -> Path:
    """Write ROOT's environment file, for the shell in force, pointing at its install prefix."""
    from motion_spec.config import settings, shell

    prefix = prefix or install_prefix(root)
    configured, _ = settings(root)
    using = shell(configured)
    path = root / f"setup-motion-spec.{using}"
    if configured.get("ros", {}).get("workspace") if ros is None else ros:
        return _write_ros_environment(root, prefix, configured, using, path, python)
    body = (
        "# Written by `motion-spec setup`. Source it before generating, building or running.\n"
        + _activation(python)
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
    root: Path, prefix: Path, configured: dict, using: str, path: Path, python: Path | None = None
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
        + _activation(python)
        # Each sourcing is skipped when this shell has already done it: sourcing a distro
        # twice is noise, and sourcing an overlay twice repeats it on every path it sets.
        + f'[ "${{ROS_DISTRO:-}}" = {distro} ] || . /opt/ros/{distro}/setup.{using}\n'
        + f'case ":${{COLCON_PREFIX_PATH:-}}:" in *:{prefix}:*) ;; *)'
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
    name: str,
    root: Path,
    prefix: Path,
    build_type: str,
    log: Path | None,
    jobs: int,
    clear_cache: bool = False,
    options: tuple[str, ...] = (),
) -> None:
    """Build one package with colcon, in the manifest's order, which colcon cannot derive.

    One package per call rather than one `colcon build`: coord2b and robif2b carry no
    package.xml, so colcon has no dependency to order them by. The workspace colcon.meta holds
    the configured arguments; OPTIONS, this machine's interpreter and this run's --cmake-arg,
    go on the command line.
    """
    if shutil.which("colcon") is None:
        raise RuntimeError("required command missing: colcon (this workspace is [ros] workspace)")
    tee(
        [
            "colcon",
            "build",
            "--packages-select",
            name,
            "--base-paths",
            str(source_root(root)),
            # Named, not left to colcon's cwd defaults: --prefix must reach the same place the
            # markers and the environment file describe.
            "--build-base",
            str(root / BUILD_DIRECTORY),
            "--install-base",
            str(prefix),
            "--metas",
            str(root / "colcon.meta"),
            *(["--cmake-clean-cache"] if clear_cache else []),
            "--cmake-args",
            f"-DCMAKE_BUILD_TYPE={build_type}",
            *options,
        ],
        log=log,
        cwd=root,
        # colcon derives -j from the core count unless MAKEFLAGS already names one.
        env={**os.environ, "MAKEFLAGS": f"-j{jobs} -l{jobs}"},
    )


def workspace_outputs(root: Path, prefix: Path | None = None) -> list[Path]:
    """Everything a workspace's builds wrote, for `--clean --all` to offer one by one.

    Wider than any component: it takes what setup did not install too, which is the point of
    asking for all of it. Sources, generations and .motion-spec/ are not outputs and never here.
    """
    prefix = prefix or install_prefix(root)
    candidates = [
        root / BUILD_DIRECTORY,
        prefix,
        root / COLCON_LOG_DIRECTORY,
        *(root / name for name in ENVIRONMENT_FILES),
    ]
    seen: dict[Path, None] = {}
    for path in candidates:
        if path.exists():
            seen.setdefault(path)
    return list(seen)


def remove_environment(root: Path) -> list[Removed]:
    """Trash ROOT's environment files, which describe an installation being cleaned away."""
    removed: list[Removed] = []
    for name in ENVIRONMENT_FILES:
        _trash_into(root / name, str(root / name), removed)
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
        capture_output=True,
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
