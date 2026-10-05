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

    One driver record for both driven families: `acceleration_energy` and `saturation` are set
    only by ACHD, `acceleration` only by RNE. Absent means the other family built this record --
    no sentinel, the family that built it is the fact.
    """

    id: str
    subspace: Subspace
    # Axis-aligned constraints carry an axis; derived constraint forms may leave it unset.
    axis: Axis | None
    acceleration_energy: Quantity | None = None
    acceleration: Quantity | None = None
    as_seen_by: Frame | None = None
    base_aligned: bool = True
    direction: Direction | None = None
    # The other subspace's half of the same solver column: a scalar of a body-fixed primitive
    # changes with both halves of the twist, so one row carries two direction vectors.
    moment_direction: Direction | None = None
    saturation: Saturation | None = None
    type: str = field(default="AccelerationConstraint")


@dataclass(frozen=True, eq=False)
class SolverFamily:
    """What a solver algorithm accepts: the slot its acceleration drivers fill and their payload
    per subspace, its axis limits, and whether it is only read or only forwards commands.
    """

    name: str = ""
    driver_field: str | None = None
    payload_field: str | None = None
    # Subspace -> the payload's QUDT quantity kind and unit local names.
    payload_kinds: dict = field(default_factory=dict)
    max_axes: int | None = None
    axes_must_be_distinct: bool = False
    read_only: bool = False
    forwards_commands: bool = False

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
    # Where the joint sits in the chain's joint array, resolved while generating.
    joint_index: int | None = None
    type: str = field(default="JointForceSpecification")


@dataclass
class MotionDrivers:
    """The physically distinct inputs accepted by a dynamics solver pipeline, for the handler
    whose controllers state them."""

    id: str
    acceleration_constraint: list[AccelerationConstraint]
    cartesian_force: list[CartesianForceSpecification]
    handler: str
    cartesian_acceleration: list[AccelerationConstraint] = field(default_factory=list)
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
    # What drives this chain, resolved once from the algorithm the model names; the runtime and
    # the templates dispatch on the name, the lowering reads the record.
    algorithm: SolverFamily = field(metadata=INTERNAL)
    algorithm_name: str | None = None
    # What the scene mounts on this chain, and what hardware is bound to drive it.
    sensors: list[SensorBinding] = field(default_factory=list)
    devices: list[DeviceBinding] = field(default_factory=list)
    gravity: list[float] | None = None
    # The gravity field a simulated ACHD run's compensation pass is built with: the opposite of
    # the root acceleration ACHD itself takes. Nothing else derives a sign from the author.
    gravity_compensation: list[float] | None = None
    derived_root_acceleration: list[float] | None = None
    # The D-block the gravity vector was read from; templates take the numbers above, this
    # says which recorded constant they came from.
    gravity_source: str | None = field(default=None, metadata=INTERNAL)
    torque_saturation: Saturation | None = None
    # The widest motion's row count; the motions share one state and zero the rows they leave.
    constraint_rows: int = 0
    # Frame-log mirrors of this runtime's joint-space signals; the two lists render at
    # two different hook sites -- the run block and the command-stage block.
    # `output` split by what each observation reads. A world output is answered by the one world
    # model, so the loop computes it every tick whether or not a motion that wants it is running;
    # a state output is read off this solver's synchronized joint mirror, which only the motion
    # driving the chain fills.
    world_output: list = field(default_factory=list)
    state_output: list = field(default_factory=list)
    joint_space_samples: list = field(default_factory=list)
    joint_space_cmd_samples: list = field(default_factory=list)
    # Which resource this solver commands; `resources.robots` is filtered on it.
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
    # The commanded wrenches routed to this solver, one per force controller: what the platform
    # is asked to push with, before it is distributed over the drives. A controller holding a
    # scalar -- a distance, say -- contributes through the wrench built from it and its
    # direction, which is the only form a distribution can take.
    forces: tuple = field(default=())
    # The world-model read this solver makes per frame IRI: its own, since another solver may
    # read the same frame under its own key.
    world_keys: dict = field(default_factory=dict, metadata=INTERNAL)
    kind: str = field(default="mobile_base")
    type: str = field(default="ForceDistributionSolver")
