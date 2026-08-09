# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)

"""Shared guards for tests that need more than this repository provides."""

from __future__ import annotations

from pathlib import Path

import pytest


def requires_interfaces(*type_names: str):
    """Skip a module whose cases read real ROS message shapes.

    Lowering a publish or an action resolves its type through rosidl, so these cases need a
    sourced distribution and the workspace's own interface packages on AMENT_PREFIX_PATH.
    """
    try:
        from rosidl_runtime_py.utilities import get_action, get_message
    except ImportError:
        return pytest.mark.skip(reason="no rosidl_runtime_py; source the ROS distribution")
    for type_name in type_names:
        resolve = get_action if "/action/" in type_name else get_message
        try:
            resolve(type_name)
        except Exception:
            return pytest.mark.skip(reason=f"'{type_name}' does not resolve on AMENT_PREFIX_PATH")

    return pytest.mark.skipif(False, reason="")


def requires_workspace(*paths: Path):
    """Skip a module whose cases read files from a sibling checkout.

    A model, a scene and the metamodels live in repositories beside this one, so a bare checkout
    of motion-spec alone cannot run these.
    """
    missing = [str(path) for path in paths if not path.exists()]
    if missing:
        return pytest.mark.skip(reason=f"not in this checkout: {', '.join(missing)}")

    return pytest.mark.skipif(False, reason="")


def requires_stst():
    """Skip a module whose cases render a template through the StringTemplate runner."""
    from motion_spec.setup import find_stst

    if find_stst() is None:
        return pytest.mark.skip(reason="no stst; run `motion-spec setup`")

    return pytest.mark.skipif(False, reason="")
