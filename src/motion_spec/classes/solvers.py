# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""Solver families, the per-axis driver record they derive, and the solvers built on them.

The term -> family table lives in `rdf_parser/constraint_handler.py`: `classes/` is RDF-free.
"""

from __future__ import annotations

from dataclasses import dataclass, field

from rdf_utils.namespace import NS_MM_QUDT_QTY, NS_MM_QUDT_UNIT

from motion_spec.classes.base import INTERNAL
from motion_spec.classes.bindings import (
    ChainBinding,
    DeviceBinding,
    HardwareBinding,
    RuntimeBinding,
    SensorBinding,
)
from motion_spec.classes.dynamics import Saturation
from motion_spec.classes.geometry import (
    Axis,
    Direction,
    Frame,
    SimplicialComplex,
    Subspace,
    VelocityTwist,
    Wrench,
)
from motion_spec.classes.qudt import Quantity, QuantityKind, Unit


@dataclass
class AccelerationConstraint:
    """An acceleration constraint (axis- or direction-aligned) on a solver.

    One driver record for every driven family: `acceleration_energy` and `saturation` are set
    only by ACHD, `acceleration` only by RNE, `velocity` only by IVK. Absent means another family
    built this record -- no sentinel, the family that built it is the fact.
    """

    id: str
    subspace: Subspace
    axis: Axis | None
    acceleration_energy: Quantity | None = None
    acceleration: Quantity | None = None
    velocity: Quantity | None = None
    as_seen_by: Frame | None = None
    base_aligned: bool = True
    direction: Direction | None = None
    moment_direction: Direction | None = None
    saturation: Saturation | None = None
    type: str = field(default="AccelerationConstraint")


@dataclass(frozen=True, eq=False)
class SolverFamily:
    """What a solver algorithm accepts: the slot its acceleration drivers fill and their payload
    per subspace, its axis limits, whether it is only read or only forwards commands, and what
    it commands the joints with.
    """

    name: str = ""
    driver_field: str | None = None
    payload_field: str | None = None
    payload_kinds: dict = field(default_factory=dict)
    max_axes: int | None = None
    axes_must_be_distinct: bool = False
    read_only: bool = False
    forwards_commands: bool = False
    command_mode: str = "torque"

    def payload(self, id: str, subspace: Subspace) -> Quantity:
        """The quantity one axis's driver carries, in this family's kind and unit."""
        kind, unit = self.payload_kinds[subspace]
        return Quantity(
            id,
            QuantityKind(kind, str(NS_MM_QUDT_QTY[kind])),
            Unit(unit.replace("-", "_"), str(NS_MM_QUDT_UNIT[unit])),
            None,
            False,
        )


@dataclass
class CartesianForceSpecification:
    """A Cartesian force applied to a body, and the controller commanding it."""

    id: str
    force: Wrench
    attached_to: SimplicialComplex
    controller: str
    type: str = field(default="CartesianForceSpecification")


@dataclass
class JointForceSpecification:
    """A joint-space force for a named joint."""

    id: str
    force_id: str
    joint_name: str
    joint_index: int | None = None
    type: str = field(default="JointForceSpecification")


@dataclass
class PoseTarget:
    """What inverse position kinematics solves for: the chain tip's measured pose and the pose it
    is held equal to, both in one frame, and the joint impedance acting toward the solution.

    Without gains the solution is commanded as joint positions.
    """

    id: str
    measured: str
    reference: str
    stiffness: list[float] = field(default_factory=list)
    damping: list[float] = field(default_factory=list)
    type: str = field(default="PoseTarget")


@dataclass
class MotionDrivers:
    """The physically distinct inputs accepted by a solver pipeline, for the handler whose
    controllers state them."""

    id: str
    acceleration_constraint: list[AccelerationConstraint]
    cartesian_force: list[CartesianForceSpecification]
    handler: str
    cartesian_acceleration: list[AccelerationConstraint] = field(default_factory=list)
    cartesian_velocity: list[AccelerationConstraint] = field(default_factory=list)
    pose_target: PoseTarget | None = None
    joint_force: list[JointForceSpecification] = field(default_factory=list)
    has_cartesian_force: bool = False
    type: str = field(default="MotionDrivers")


@dataclass
class SolverWithInputAndOutput:
    """A full arm solver: chain, algorithm, drivers and outputs."""

    id: str
    motion_drivers: list[MotionDrivers] = field(metadata=INTERNAL)
    output: list
    chain: ChainBinding
    hardware: HardwareBinding
    runtime: RuntimeBinding
    algorithm: SolverFamily = field(metadata=INTERNAL)
    algorithm_name: str | None = None
    sensors: list[SensorBinding] = field(default_factory=list)
    devices: list[DeviceBinding] = field(default_factory=list)
    gravity: list[float] | None = None
    # opposite of the root acceleration ACHD takes
    gravity_compensation: list[float] | None = None
    derived_root_acceleration: list[float] | None = None
    gravity_source: str | None = field(default=None, metadata=INTERNAL)
    torque_saturation: Saturation | None = None
    # widest motion's rows; narrower motions zero the rest
    constraint_rows: int = 0
    # world outputs run every tick; state outputs only while a motion drives the chain
    world_output: list = field(default_factory=list)
    state_output: list = field(default_factory=list)
    joint_space_samples: list = field(default_factory=list)
    joint_space_cmd_samples: list = field(default_factory=list)
    kind: str = field(default="serial_chain")
    type: str = field(default="SolverWithInputAndOutput")


@dataclass
class VelocityCompositionSolver:
    """A base velocity-composition solver."""

    id: str
    configuration: str
    velocity: VelocityTwist
    kind: str = field(default="mobile_base")
    type: str = field(default="VelocityCompositionSolver")


@dataclass
class ForceDistributionSolver:
    """A base force-distribution solver."""

    id: str
    configuration: str
    force: Wrench
    forces: tuple = field(default=())
    world_keys: dict = field(default_factory=dict, metadata=INTERNAL)
    kind: str = field(default="mobile_base")
    type: str = field(default="ForceDistributionSolver")
