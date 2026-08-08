# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""The dynamics-solver family hierarchy, the one per-axis driver record both families derive,
and the solvers built on top of them.

`DynamicsSolverFamily` and its subclasses are never instantiated: they carry a family's
behaviour as class-level attributes and a `payload()` factory, dispatched on with
`issubclass()`/`hasattr()` rather than an isinstance check on a built object. The
term -> family map stays in `rdf_parser/constraint_handler.py` -- `classes/` is RDF-free.
"""

from __future__ import annotations

from dataclasses import dataclass, field

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
    saturation: Saturation | None = None
    type: str = field(default="AccelerationConstraint")


class DynamicsSolverFamily:
    """Base of the solver-algorithm family hierarchy. Class attributes only; never instantiated,
    never published.
    """

    codegen_name: str = ""
    signal_prefix: str | None = None
    driver: type | None = None
    driver_field: str | None = None
    payload_field: str | None = None
    id_tags: tuple[str, ...] = ()
    max_axes: int | None = None
    axes_must_be_distinct: bool = False


class AccelerationEnergyDriven(DynamicsSolverFamily):
    """slv:AccelerationConstrainedHybridDynamicsAlgorithm.

    Vereshchagin's acceleration-constrained hybrid dynamics is posed as a constrained
    optimisation over Gauss's principle, so each constrained axis is driven by an acceleration
    energy (N-m2/s2).
    """

    codegen_name = "ACHD"
    signal_prefix = "eacc"
    driver = AccelerationConstraint
    driver_field = "acceleration_constraint"
    payload_field = "acceleration_energy"
    id_tags = ("acc-cstr", "eacc")
    max_axes = 6
    axes_must_be_distinct = True

    @staticmethod
    def payload(id: str, axis: Subspace) -> Quantity:
        return Quantity(id, QuantityKind("AccelerationEnergy"), Unit("N_M2_PER_SEC2"), None, False)


class CartesianAccelerationDriven(DynamicsSolverFamily):
    """slv:RecursiveNewtonEulerAlgorithm. Driven by the Cartesian acceleration itself, linear or
    angular depending on which half of the subspace the axis is in.
    """

    codegen_name = "RNE"
    signal_prefix = "acc"
    driver = AccelerationConstraint
    driver_field = "cartesian_acceleration"
    payload_field = "acceleration"
    id_tags = ("cart-acc", "acc")

    @staticmethod
    def payload(id: str, axis: Subspace) -> Quantity:
        kind, unit = (
            ("LinearAcceleration", "M_PER_SEC2")
            if axis == Subspace.Linear
            else ("AngularAcceleration", "RAD_PER_SEC2")
        )
        return Quantity(id, QuantityKind(kind), Unit(unit), None, False)


class CommandForwarding(DynamicsSolverFamily):
    """slv-ext:CommandForwardingSolver. Accepts no acceleration driver; a controller's output
    forwards straight to a joint.
    """

    codegen_name = ""


class Unsolved(DynamicsSolverFamily):
    """Names no algorithm: nothing drives this solver, it is read for state and watched by
    monitors.
    """


@dataclass
class CartesianForceSpecification:
    """A Cartesian force applied to a body."""

    id: str
    force: Wrench
    attached_to: SimplicialComplex
    type: str = field(default="CartesianForceSpecification")


@dataclass
class JointForceSpecification:
    """A joint-space force for a named joint."""

    id: str
    force_id: str
    joint_name: str
    type: str = field(default="JointForceSpecification")


@dataclass
class MotionDrivers:
    """The physically distinct inputs accepted by a dynamics solver pipeline."""

    id: str
    acceleration_constraint: list[AccelerationConstraint]
    cartesian_force: list[CartesianForceSpecification]
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
    algorithm: type[DynamicsSolverFamily] | None = field(default=None, metadata=INTERNAL)
    algorithm_name: str = ""
    # What the scene mounts on this chain, and what hardware is bound to drive it.
    sensors: list[SensorBinding] = field(default_factory=list)
    devices: list[DeviceBinding] = field(default_factory=list)
    gravity: list[float] | None = None
    derived_root_acceleration: list[float] | None = None
    torque_saturation: Saturation | None = None
    # Frame-log mirrors of this runtime's joint-space signals (plan 012); the two lists render at
    # two different hook sites -- the run block and the command-stage block.
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
    kind: str = field(default="mobile_base")
    type: str = field(default="ForceDistributionSolver")
