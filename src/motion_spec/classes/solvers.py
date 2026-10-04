# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""The dynamics-solver family hierarchy, the one per-axis driver record both families derive,
and the solvers built on top of them.

`DynamicsSolverFamily` and its subclasses are never instantiated: they carry a family's
behaviour as class-level attributes and a `payload()` factory, dispatched on with
`issubclass()` or as keys of the family tables rather than an isinstance check on a built
object. The
term -> family map stays in `rdf_parser/constraint_handler.py` -- `classes/` is RDF-free.
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

    One driver record for both solver families: `acceleration_energy` and `saturation`
    are set only by `AccelerationEnergyDriven` (ACHD), `acceleration` only by
    `CartesianAccelerationDriven` (RNE). Absent means the other family built this record --
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


class DynamicsSolverFamily:
    """Base of the solver-algorithm family hierarchy. Class attributes only; never instantiated,
    never published.
    """

    codegen_name: str = ""
    driver: type | None = None
    driver_field: str | None = None
    payload_field: str | None = None
    max_axes: int | None = None
    axes_must_be_distinct: bool = False


class AccelerationEnergyDriven(DynamicsSolverFamily):
    """slv:AccelerationConstrainedHybridDynamicsAlgorithm.

    Vereshchagin's acceleration-constrained hybrid dynamics is posed as a constrained
    optimisation over Gauss's principle, so each constrained axis is driven by an acceleration
    energy (N-m2/s2).
    """

    codegen_name = "ACHD"
    driver = AccelerationConstraint
    driver_field = "acceleration_constraint"
    payload_field = "acceleration_energy"
    max_axes = 6
    axes_must_be_distinct = True

    @staticmethod
    def payload(id: str, axis: Subspace) -> Quantity:
        return Quantity(
            id,
            QuantityKind("AccelerationEnergy", str(NS_MM_QUDT_QTY["AccelerationEnergy"])),
            Unit("N_M2_PER_SEC2", str(NS_MM_QUDT_UNIT["N-M2-PER-SEC2"])),
            None,
            False,
        )


class CartesianAccelerationDriven(DynamicsSolverFamily):
    """slv:RecursiveNewtonEulerAlgorithm. Driven by the Cartesian acceleration itself, linear or
    angular depending on which half of the subspace the axis is in.
    """

    codegen_name = "RNE"
    driver = AccelerationConstraint
    driver_field = "cartesian_acceleration"
    payload_field = "acceleration"

    @staticmethod
    def payload(id: str, axis: Subspace) -> Quantity:
        kind, unit = (
            ("LinearAcceleration", "M-PER-SEC2")
            if axis == Subspace.Linear
            else ("AngularAcceleration", "RAD-PER-SEC2")
        )
        return Quantity(
            id,
            QuantityKind(kind, str(NS_MM_QUDT_QTY[kind])),
            Unit(unit.replace("-", "_"), str(NS_MM_QUDT_UNIT[unit])),
            None,
            False,
        )


class CommandForwarding(DynamicsSolverFamily):
    """slv-ext:CommandForwardingSolver. Accepts no acceleration driver; a controller's output
    forwards straight to a joint.
    """

    codegen_name = ""


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
    # the templates dispatch on the name, the lowering reads the record. None for forward
    # kinematics: nothing drives the chain, it is only read.
    algorithm: type[DynamicsSolverFamily] | None = field(default=None, metadata=INTERNAL)
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
