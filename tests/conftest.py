# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)

"""Shared guards for tests that need more than this repository provides."""

from __future__ import annotations

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
