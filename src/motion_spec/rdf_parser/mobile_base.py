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

from motion_spec.rdf_parser.model import local_name


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

    # The joint names the compiled scene knows, which are the tree's local names: a `map tree`
    # matches an instantiated body or joint to the asset by its own local name, not by the
    # instance-qualified one the model refers to it through.
    pivot_joint: str
    wheels: list[str]
    # Pivot axis on the platform, (x, y) in the platform root's frame [m].
    attachment: list[float]
    # Half the separation of the unit's two axles [m]; hddc2b's `whl_dst`.
    wheel_distance: float
    # Perpendicular distance from the pivot axis to the axle line [m].
    castor_offset: float
    # What the joint reads when the drive points along the platform's x, in radians. hddc2b's
    # pivot angle is the heading of the rolling direction in the platform frame, and a joint's
    # zero is wherever the model put it, so the two differ by this constant.
    pivot_offset: float = 0.0
    # Per channel, what turns hddc2b's hub convention into this model's joint. Two parts: hddc2b
    # runs a unit's channels against each other as the real drive's mirrored motors do, and the
    # model gives each wheel joint whatever spin axis it likes. A unit mounted half a turn round
    # has its axis against the drive frame, so the signs are not the same on both sides -- assume
    # they are and a forward push comes out as a couple, and the platform spins instead of driving.
    hub_signs: list[float] = field(default_factory=list)
    type: str = field(default="DriveUnit")


def _revolute(segment) -> bool:
    return bool(segment.get("joint")) and segment["joint"].get("axis") is not None


def drive_units(tree: dict, root: str) -> list[DriveUnit]:
    """Every castor drive unit hanging off `root` in `tree`, in the tree's own order.

    A drive unit is a body the platform holds by one revolute joint that itself carries exactly
    two revolute-jointed bodies. That is a structural statement, not a naming heuristic: nothing
    else in a castor platform has that shape.

    Raises:
        ConstraintViolation: a unit whose two axles are coincident, which leaves hddc2b's
            differential drive undefined.
    """
    children: dict[str, list[dict]] = {}
    for segment in tree.get("segments", ()):
        if _revolute(segment):
            children.setdefault(segment["parent"], []).append(segment)

    units = []
    for pivot in children.get(root, ()):
        wheels = children.get(pivot["name"], ())
        if len(wheels) != 2:
            continue

        origin = np.asarray(pivot["joint"]["origin"], dtype=float)
        axis = np.asarray(pivot["joint"]["axis"], dtype=float)

        first, second = (np.asarray(w["joint"]["origin"], dtype=float) for w in wheels)
        separation = float(np.linalg.norm(first - second))
        if separation == 0.0:
            raise ConstraintViolation(
                "kinematics",
                f"drive unit '{pivot['joint']['name']}' has its two wheel joints at one point, "
                "so it has no differential drive",
            )

        pivot_axis = axis / np.linalg.norm(axis)
        # The axle sits off the pivot axis by the castor offset, and a castor trails: the drive
        # rolls along the direction that carries the pivot away from its own axle. That fixes the
        # drive frame, and with it both the pivot angle and which wheel is which.
        midpoint = 0.5 * (first + second)
        trail = midpoint - (midpoint @ pivot_axis) * pivot_axis
        castor_offset = float(np.linalg.norm(trail))
        if castor_offset == 0.0:
            raise ConstraintViolation(
                "kinematics",
                f"drive unit '{pivot['joint']['name']}' puts its axle on the pivot axis, so it "
                "has no castor to trail and no rolling direction to steer",
            )
        longitudinal = -trail / castor_offset
        transverse = np.cross(pivot_axis, longitudinal)

        # hddc2b's right channel is the wheel at -y of that frame; getting this backwards leaves
        # the drive term intact and reverses the steering term.
        right, left = sorted(
            wheels,
            key=lambda w: float(np.asarray(w["joint"]["origin"], dtype=float) @ transverse),
        )

        # hddc2b's channels counter-rotate; the model's spin axis may run either way against the
        # drive frame, and on a unit mounted half a turn round it runs the other way.
        hub_signs = [
            mirrored * float(np.sign(np.asarray(w["joint"]["axis"], dtype=float) @ transverse))
            for mirrored, w in ((-1.0, right), (1.0, left))
        ]

        units.append(
            DriveUnit(
                pivot_joint=local_name(pivot["joint"]["iri"]),
                wheels=[local_name(right["joint"]["iri"]), local_name(left["joint"]["iri"])],
                attachment=[float(origin[0]), float(origin[1])],
                wheel_distance=separation / 2.0,
                castor_offset=castor_offset,
                pivot_offset=float(np.arctan2(longitudinal[1], longitudinal[0])),
                hub_signs=hub_signs,
            )
        )
    return units


def platform_drives(world_trees: list[dict]) -> list[DriveUnit]:
    """The drive units of the one platform in the scene.

    The platform's own root is the first body under the scene tree's root that holds castors --
    a base_footprint-style ground frame sits above it and holds none.

    Raises:
        ConstraintViolation: no tree in the scene has castor drive units.
    """
    for tree in world_trees:
        bodies = [tree["root"], *(segment["name"] for segment in tree.get("segments", ()))]
        for body in bodies:
            units = drive_units(tree, body)
            if units:
                return units
    raise ConstraintViolation(
        "kinematics",
        "the scene states no castor drive unit: a mobile-platform solver needs a body held by a "
        "revolute pivot that carries exactly two revolute-jointed wheels",
    )


# Which platform coordinate a controller drives, read off the component it regulates. The
# platform's three coordinates are longitudinal, transverse and yaw, so a controller closing on the
# twist's x velocity and one commanding the wrench's x force both produce the same coordinate --
# the first by regulating it, the second open loop.
_COORDINATE_BY_COMPONENT = {
    "linvel_x": 0,
    "linvel_y": 1,
    "angvel_z": 2,
    "force_x": 0,
    "force_y": 1,
    "torque_z": 2,
}


# The wrench components the platform can be pushed with, in the order hddc2b takes them.
_PLATFORM_WRENCH_COMPONENTS = (("force", "x"), ("force", "y"), ("torque", "z"))


def wrench_terms_by_motion(motions, velocity_solvers, force_solvers=()) -> list[dict]:
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
    platform_ids = {
        solver.velocity.id for solver in velocity_solvers if getattr(solver, "velocity", None)
    } | {solver.force.id for solver in force_solvers if getattr(solver, "force", None)}
    # Every wrench the force distributions are fed, by the controller that commands it.
    wrench_by_controller = {}
    for solver in force_solvers:
        platform_key = getattr(getattr(solver, "force", None), "as_seen_by", None)
        platform_key = getattr(platform_key, "world_key", None)
        for spec in getattr(solver, "forces", ()) or ():
            wrench = getattr(getattr(spec, "force", None), "id", None)
            if wrench:
                wrench_by_controller[spec.id] = (
                    wrench,
                    getattr(getattr(spec.force, "as_seen_by", None), "world_key", None),
                    platform_key,
                )

    rows = []
    for motion in motions:
        terms = []
        for controller in getattr(motion, "controllers", ()):
            signal = getattr(getattr(controller, "control_signal", None), "id", None)
            if not signal:
                continue
            commanded = wrench_by_controller.get(f"spec_{controller.id}")
            if commanded is not None:
                wrench, frame_key, platform_key = commanded
                # The wrench is built in the frame its constraint measures in; the platform sums
                # in its own. Rotated once per wrench, under a local name the terms then read --
                # unless the two are one frame, which a platform without a chain has no world to ask.
                local = (
                    f"pltf_{wrench}"
                    if frame_key and platform_key and frame_key != platform_key
                    else None
                )
                terms.extend(
                    {
                        "wrench": wrench,
                        "local": local,
                        "frame_index": frame_key,
                        "platform_index": platform_key,
                        "subspace": subspace,
                        "axis": axis,
                        "coordinate": index,
                    }
                    for index, (subspace, axis) in enumerate(_PLATFORM_WRENCH_COMPONENTS)
                )
                continue
            measured = getattr(controller, "measured_signal", None)
            if not measured:
                continue
            owner = next((pid for pid in platform_ids if measured.startswith(f"{pid}_")), None)
            if owner is None:
                continue
            coordinate = _COORDINATE_BY_COMPONENT.get(measured[len(owner) + 1 :])
            if coordinate is None:
                continue
            terms.append({"signal": signal, "coordinate": coordinate})
        if terms:
            rotations = list(
                {
                    term["local"]: term
                    for term in terms
                    if term.get("local")
                }.values()
            )
            rows.append(
                {
                    "index": motion.index,
                    "terms": sorted(terms, key=lambda t: t["coordinate"]),
                    "rotations": rotations,
                }
            )
    return rows
