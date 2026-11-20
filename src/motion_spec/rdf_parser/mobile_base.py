# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""The castor drive units of a mobile platform, read off the scene's kinematic tree.

hddc2b describes a platform by numbers the tree already states: where each pivot axis stands on
the platform, how far the two hub axles sit either side of it, and how far the axle trails the
pivot. Deriving them here keeps the deployment config free of geometry the model owns, and keeps
a backend module holding only what the target adds -- joint names and indices.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
from rdf_utils.constraints import ConstraintViolation

from motion_spec.classes.geometry import Axis, Subspace
from motion_spec.rdf_parser.model import local_name
from motion_spec.rdf_parser.views import views_by_subobject


@dataclass
class DriveUnit:
    """One castor: the pivot it steers about and the two hub joints that drive it.

    hddc2b describes a drive by a frame whose x is the rolling direction and whose y is x turned
    a quarter turn about the pivot; the wheel at -y is the "right" channel, the one at +y "left"
    (`OFFSET_RIGHT`/`OFFSET_LEFT` in hddc2b's config.c). The two matter together: swapping the
    channels leaves the drive term untouched but reverses the steering term, so a platform built
    on a mismatched pair drives in a straight line and cannot turn.

    Both are read off the tree here rather than assumed, so nothing depends on what the wheels
    happen to be called.
    """

    # local names: a `map tree` matches by the asset's own name
    pivot_joint: str
    wheels: list[str]
    # (x, y) in the platform root frame [m]
    attachment: list[float]
    # hddc2b's `whl_dst` [m]
    wheel_distance: float
    # [m]
    castor_offset: float
    # joint reading with the drive along platform x [rad]
    pivot_offset: float = 0.0
    # hddc2b hub convention to model joint, per channel; differs per side
    hub_signs: list[float] = field(default_factory=list)
    type: str = field(default="DriveUnit")


def drive_unit(pivot: dict, wheels: list[dict]) -> DriveUnit:
    """The drive unit a pivot segment forms with its two wheel segments.

    Raises:
        ConstraintViolation: the wheels share one point, or the axle sits on the pivot axis.
    """
    origin = np.asarray(pivot["joint"]["origin"], dtype=float)
    axis = np.asarray(pivot["joint"]["axis"], dtype=float)

    first, second = (np.asarray(w["joint"]["origin"], dtype=float) for w in wheels)
    separation = float(np.linalg.norm(first - second))
    if separation == 0.0:
        raise ConstraintViolation(
            "kinematics",
            f"drive unit '{pivot['joint']['name']}' has its two wheel joints at one "
            "point, so it has no differential drive",
        )

    pivot_axis = axis / np.linalg.norm(axis)
    # a castor trails: the drive rolls away from its axle
    midpoint = 0.5 * (first + second)
    trail = midpoint - (midpoint @ pivot_axis) * pivot_axis
    castor_offset = float(np.linalg.norm(trail))
    if castor_offset == 0.0:
        raise ConstraintViolation(
            "kinematics",
            f"drive unit '{pivot['joint']['name']}' puts its axle on the pivot axis, "
            "so it has no castor to trail and no rolling direction to steer",
        )
    longitudinal = -trail / castor_offset
    transverse = np.cross(pivot_axis, longitudinal)

    right, left = sorted(
        wheels, key=lambda w: float(np.asarray(w["joint"]["origin"], dtype=float) @ transverse)
    )

    hub_signs = [
        -float(np.sign(np.asarray(right["joint"]["axis"], dtype=float) @ transverse)),
        float(np.sign(np.asarray(left["joint"]["axis"], dtype=float) @ transverse)),
    ]

    return DriveUnit(
        pivot_joint=local_name(pivot["joint"]["iri"]),
        wheels=[local_name(right["joint"]["iri"]), local_name(left["joint"]["iri"])],
        attachment=[float(origin[0]), float(origin[1])],
        wheel_distance=separation / 2.0,
        castor_offset=castor_offset,
        pivot_offset=float(np.arctan2(longitudinal[1], longitudinal[0])),
        hub_signs=hub_signs,
    )


def platform_drives(world_trees: list[dict]) -> list[DriveUnit]:
    """The drive units of the one platform in the scene.

    The platform's own root is the first body under the scene tree's root that holds castors --
    a base_footprint-style ground frame sits above it and holds none.

    Raises:
        ConstraintViolation: no tree in the scene has castor drive units.
    """
    for tree in world_trees:
        children: dict[str, list[dict]] = {}
        for segment in tree["segments"]:
            if segment["joint"] and segment["joint"]["axis"] is not None:
                children.setdefault(segment["parent"], []).append(segment)
        for body in [tree["root"], *(segment["name"] for segment in tree["segments"])]:
            units = [
                drive_unit(pivot, children[pivot["name"]])
                for pivot in children.get(body, ())
                if len(children.get(pivot["name"], ())) == 2
            ]
            if units:
                return units
    raise ConstraintViolation(
        "kinematics",
        "the scene states no castor drive unit: a mobile-platform solver needs a body held by a "
        "revolute pivot that carries exactly two revolute-jointed wheels",
    )


COORDINATE_INDEX_BY_COMPONENT = {
    (Subspace.Linear, Axis.X): 0,
    (Subspace.Linear, Axis.Y): 1,
    (Subspace.Angular, Axis.Z): 2,
}


# hddc2b's input order
PLATFORM_WRENCH_COMPONENTS = (("force", "x"), ("force", "y"), ("torque", "z"))


def commanded_wrenches(force_solvers) -> dict:
    """The wrench each force-distribution controller commands, with the world keys of its frames.

    `local` names the rotated copy a wrench needs when its frame is not the platform's.
    """
    wrench_by_controller = {}
    for solver in force_solvers:
        platform_frame = solver.force.as_seen_by
        platform_key = solver.world_keys.get(
            platform_frame.uri if platform_frame is not None else None
        )
        for spec in solver.forces:
            if not spec.force.id:
                continue
            frame = spec.force.as_seen_by
            frame_key = solver.world_keys.get(frame.uri if frame is not None else None)
            rotated = frame_key and platform_key and frame_key != platform_key
            wrench_by_controller[spec.controller] = {
                "wrench": spec.force.id,
                "local": f"pltf_{spec.force.id}" if rotated else None,
                "frame_index": frame_key,
                "platform_index": platform_key,
            }
    return wrench_by_controller


def platform_coordinate(controller, by_subobject: dict, platform_ids: set) -> int | None:
    """The platform wrench coordinate a controller closes on, or None when it closes on none.

    Raises:
        ConstraintViolation: the controller's measured quantity is read by several views.
    """
    measured = by_subobject.get(controller.measured_signal, ())
    if len(measured) > 1:
        raise ConstraintViolation(
            "control",
            f"controller '{controller.id}' measures a quantity {len(measured)} views read",
        )
    if not measured or measured[0].superobject.id not in platform_ids:
        return None
    return COORDINATE_INDEX_BY_COMPONENT.get((measured[0].subspace, measured[0].axis))


def wrench_terms_by_motion(motions, views, velocity_solvers, force_solvers=()) -> list[dict]:
    """Per motion, the controller outputs that make up the platform wrench, by coordinate.

    Two kinds of term reach the platform. A controller closing on one of the platform's own
    components names that component directly, and contributes its scalar. A controller holding
    something with no platform component of its own -- a distance between two points, say --
    contributes through the wrench built from its magnitude and its direction, and that wrench
    gives all three coordinates at once. sc1's alignment is the second kind throughout: nothing
    commands the base a component, the arms' reach errors become forces along the shoulder-to-hand
    line and the platform is handed the result.

    Only the running motion's controllers may contribute: an inactive handler's control signal
    keeps whatever it last wrote, and summing it would drive the platform from a state it left.
    """
    platform_ids = {solver.velocity.id for solver in velocity_solvers} | {
        solver.force.id for solver in force_solvers
    }
    by_subobject = views_by_subobject(views)
    wrench_by_controller = commanded_wrenches(force_solvers)

    rows = []
    for motion in motions:
        terms = []
        for controller in motion.controllers:
            signal = controller.control_signal.id
            if not signal:
                continue
            commanded = wrench_by_controller.get(controller.id)
            if commanded is not None:
                terms.extend(
                    {**commanded, "subspace": subspace, "axis": axis, "coordinate": index}
                    for index, (subspace, axis) in enumerate(PLATFORM_WRENCH_COMPONENTS)
                )
                continue
            coordinate = platform_coordinate(controller, by_subobject, platform_ids)
            if coordinate is not None:
                terms.append({"signal": signal, "coordinate": coordinate})
        if terms:
            rotations = list({term["local"]: term for term in terms if term.get("local")}.values())
            rows.append({"index": motion.index, "terms": terms, "rotations": rotations})
    return rows
