# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
# Author: Vamsi Kalagaturu

"""A workspace's own settings, so its shape is not retyped into every shell.

A command-line option beats an environment variable, which beats this file, which beats the
built-in default. Every value says which of the four it came from, and `motion-spec config`
prints that.
"""

from __future__ import annotations

import os
import sys
import tomllib
from dataclasses import dataclass
from pathlib import Path

CONFIG_FILE = "motion-spec.config.toml"
SECTIONS = {
    "workspace": {"root", "generations", "environment", "shell"},
    "setup": {"prefix", "build_type", "components", "cmake_args", "editable", "jobs", "dev"},
    "ros": {"workspace", "distro"},
}
TOP_LEVEL = {"version"}
SHELLS = ("bash", "zsh")
# Paths are written relative to the file, and resolved against its directory.
_PATH_KEYS = {("workspace", "root"), ("workspace", "generations"), ("workspace", "environment")}


@dataclass(frozen=True)
class Setting:
    """One resolved value, and which of the four sources decided it."""

    value: object
    source: str


def detect_shell() -> str:
    """The shell this user runs, from $SHELL; bash when it is neither one we write files for."""
    name = Path(os.environ.get("SHELL", "")).name
    return name if name in SHELLS else "bash"


def shell(configured: dict | None = None) -> str:
    """The shell a workspace's environment file is written for and sourced with."""
    declared = (configured or {}).get("workspace", {}).get("shell")
    return declared if declared in SHELLS else detect_shell()


def find_config(start: Path | None = None) -> Path | None:
    """The nearest motion-spec.config.toml at or above START, else None."""
    root = (start or Path.cwd()).resolve()
    for directory in (root, *root.parents):
        candidate = directory / CONFIG_FILE
        if candidate.is_file():
            return candidate
    return None


def load(path: Path) -> dict:
    """The settings PATH declares, with paths resolved and unknown keys refused."""
    try:
        parsed = tomllib.loads(path.read_text())
    except tomllib.TOMLDecodeError as error:
        raise ValueError(f"{path}: {error}") from error

    from motion_spec.formats import FORMATS, check

    warning = check("config", parsed.get("version", FORMATS["config"].oldest), path)
    if warning:
        print(f"warning: {warning}", file=sys.stderr)
    for section, keys in parsed.items():
        if section in TOP_LEVEL:
            continue
        if section not in SECTIONS:
            raise ValueError(f"{path}: unknown section [{section}]")
        if not isinstance(keys, dict):
            raise ValueError(f"{path}: [{section}] is not a table")
        for key in keys:
            if key not in SECTIONS[section]:
                raise ValueError(f"{path}: unknown key {key} in [{section}]")
    for section, key in _PATH_KEYS:
        value = parsed.get(section, {}).get(key)
        if value is not None:
            parsed[section][key] = str((path.parent / str(value)).resolve())
    return parsed


def settings(start: Path | None = None) -> tuple[dict, Path | None]:
    """The settings in force, and the file they came from."""
    path = find_config(start)
    return (load(path) if path else {}), path


def resolve(flag, variable: str | None, configured, default, environ) -> Setting:
    """The value in force, by the precedence this module exists to state once."""
    if flag is not None:
        return Setting(flag, "flag")
    named = environ.get(variable) if variable else None
    if named:
        return Setting(named, "environment")
    if configured is not None:
        return Setting(configured, "file")
    return Setting(default, "default")


def sample(
    components: dict[str, tuple[str, ...]],
    default_components: tuple[str, ...],
    root: Path | None = None,
    ros: bool = False,
) -> str:
    """Every key this file understands, at its default, commented out for someone to edit."""
    listed = ", ".join(f'"{name}"' for name in default_components)
    on_request = [name for name in components if name not in default_components]
    width = max(len(name) for name in components)
    args = []
    for name, built_in in components.items():
        listed_options = ", ".join(f'"{option}"' for option in built_in)
        # Commented like its entry in `components`: not built unless it is named.
        prefix = "" if name in default_components else "# "
        args.append(f"{prefix}{name:<{width}} = [{listed_options}]")
    from motion_spec.formats import FORMATS

    from motion_spec.health import _ros_distro
    from motion_spec.setup import build_jobs

    # Left commented: a number that fits this machine's memory can take a smaller one down.
    jobs_key = f"# jobs = {build_jobs()}"

    newline = "\n"
    using = shell()
    version = FORMATS["config"].current
    found = _ros_distro()
    distro_key = f'distro = "{found}"' if found else '# distro = "jazzy"'
    ros_key = f"workspace = {'true' if ros else 'false'}"
    workspace_keys = newline.join(
        f"{code:<42} # {note}"
        for code, note in (
            (f'root = "{root}"' if root else '# root = "."', "$MOTION_SPEC_WS"),
            ('generations = "generations"', "$MOTION_SPEC_GEN"),
            (f'environment = "setup-motion-spec.{using}"', "$MOTION_SPEC_ENV"),
            (f'shell = "{using}"', "written, and sourced with"),
        )
    )
    return f"""\
# motion-spec workspace settings.
#
# This file makes the directory it sits in a workspace: `setup` installs into it, generations
# are written to it, and a command run anywhere below it finds it without any variable set.
# It is read by setup, build, run, health and the dashboard.
#
# Flag > environment variable > this file > default. Paths are relative to this file.
version = {version}

[workspace]
{workspace_keys}

[ros]
{ros_key:<42} # true: setup writes colcon.meta so colcon builds these
{distro_key:<42} # which /opt/ros to build against; $ROS_DISTRO overrides

[setup]
prefix = "install"
build_type = "RelWithDebInfo"
dev = false                                # check the Python components out into src/
editable = false                           # pip install -e that checkout; --dev implies it
{jobs_key:<42} # compilers at once; -j and the cmake variable win
components = [{listed}]
#   only when named: {", ".join(on_request)}

# Each list is the whole list cmake is passed.
[setup.cmake_args]
{newline.join(args)}
"""


def write_sample(
    root: Path,
    components: dict[str, tuple[str, ...]],
    default: tuple[str, ...],
    declared: bool = False,
    ros: bool = False,
) -> Path | None:
    """Write the sample at ROOT, unless a config is already there.

    DECLARED records ROOT in the file, for a workspace named on the command line rather than
    found by the file's own location.
    """
    path = root / CONFIG_FILE
    if path.exists():
        return None
    path.write_text(sample(components, default, root if declared else None, ros))
    return path
