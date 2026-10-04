# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
# Author: Vamsi Kalagaturu

"""A workspace's own settings, so its shape is not retyped into every shell.

A command-line option beats an environment variable, which beats this file, which beats the
built-in default; `setting` is the one place that order is applied.
"""

from __future__ import annotations

import os
import sys
import tomllib
from dataclasses import dataclass
from pathlib import Path

CONFIG_FILE = "motion-spec.config.toml"
SHELLS = ("bash", "zsh")


@dataclass(frozen=True)
class Key:
    """Where a setting sits in the file, the variable that overrides it, and whether it is a path."""

    section: str
    key: str
    variable: str | None
    path: bool = False


KEYS = {
    "workspace.root": Key("workspace", "root", "MOTION_SPEC_WS", path=True),
    "workspace.generations": Key("workspace", "generations", "MOTION_SPEC_GEN", path=True),
    "workspace.environment": Key("workspace", "environment", "MOTION_SPEC_ENV", path=True),
    "ros.workspace": Key("ros", "workspace", None),
    "ros.distro": Key("ros", "distro", "ROS_DISTRO"),
}


@dataclass(frozen=True)
class Setting:
    """One resolved value, and which of the four sources decided it."""

    value: object
    source: str


def shell() -> str:
    """The shell environment files are written for: $SHELL's, else bash."""
    name = Path(os.environ.get("SHELL", "")).name
    return name if name in SHELLS else "bash"


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

    warning = check("config", parsed.pop("version", FORMATS["config"].oldest), path)
    if warning:
        print(f"warning: {warning}", file=sys.stderr)
    known = {(key.section, key.key): key for key in KEYS.values()}
    for section, keys in parsed.items():
        if not isinstance(keys, dict):
            raise ValueError(f"{path}: [{section}] is not a table")  # noqa: TRY004 -- file content
        for name, value in keys.items():
            key = known.get((section, name))
            if key is None:
                raise ValueError(f"{path}: unknown key {name} in [{section}]")
            if key.path:
                keys[name] = str((path.parent / str(value)).resolve())
    return parsed


def configured(name: str, start: Path | None = None) -> object | None:
    """What the nearest config file says for NAME, ignoring any environment variable."""
    path = find_config(start)
    if path is None:
        return None
    key = KEYS[name]
    return load(path).get(key.section, {}).get(key.key)


def setting(name: str, flag: object = None, start: Path | None = None) -> Setting:
    """NAME's value in force: the flag, else its variable, else the file, else None."""
    if flag is not None:
        return Setting(flag, "flag")
    variable = KEYS[name].variable
    named = os.environ.get(variable) if variable else None
    if named:
        return Setting(named, "environment")
    value = configured(name, start)
    return Setting(value, "file") if value is not None else Setting(None, "default")


def sample(root: Path | None = None, ros: bool = False) -> str:
    """Every key this file understands, at the value the run that wrote it used."""
    from motion_spec.formats import FORMATS
    from motion_spec.health import ros_distro

    newline = "\n"
    version = FORMATS["config"].current
    found = ros_distro()
    distro_key = f'distro = "{found}"' if found else '# distro = "jazzy"'
    ros_key = f"workspace = {'true' if ros else 'false'}"
    workspace_keys = newline.join(
        f"{code:<42} # {note}"
        for code, note in (
            (f'root = "{root}"' if root else '# root = "."', "$MOTION_SPEC_WS"),
            ('generations = "generations"', "$MOTION_SPEC_GEN"),
            (f'environment = "setup-motion-spec.{shell()}"', "$MOTION_SPEC_ENV"),
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
{ros_key:<42} # true: CMake packages build with colcon
{distro_key:<42} # which /opt/ros to build against; $ROS_DISTRO overrides
"""


def write_sample(root: Path, declared: bool = False, ros: bool = False) -> Path | None:
    """Write the sample at ROOT, unless a config is already there.

    DECLARED records ROOT in the file, for a workspace named on the command line rather than
    found by the file's own location.
    """
    path = root / CONFIG_FILE
    if path.exists():
        return None
    path.write_text(sample(root if declared else None, ros))
    return path
