# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
# Author: Vamsi Kalagaturu
"""Pure solver semantics derived from authored controller facts."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class AccelerationAxis:
    """One ordered linear or angular acceleration axis."""

    subspace: str
    axis: str

    @property
    def suffix(self) -> str:
        """Return the compatibility suffix used by generated IR identifiers."""
        prefix = "lin" if self.subspace == "linear-acceleration" else "ang"
        return f"{prefix}_{self.axis}"


@dataclass(frozen=True)
class SolverIdFactory:
    """Build compatibility IDs solely from authored RDF resource IDs."""

    controller: str
    motion: str

    def component_controller(self, axis: AccelerationAxis) -> str:
        return f"{self.controller}_{axis.suffix}"

    def component_error(self, axis: AccelerationAxis) -> str:
        return f"{self.controller}_err_{axis.suffix}"

    def component_energy(self, axis: AccelerationAxis) -> str:
        return f"eacc_{self.controller}_{axis.suffix}"

    def component_constraint(self, axis: AccelerationAxis) -> str:
        return f"acc_cstr_{self.controller}_{axis.suffix}"

    def pose_evaluator(self) -> str:
        return f"eval_pose_diff_{self.controller}"

    def pose_difference(self) -> str:
        return f"pose_diff_{self.controller}"


LINEAR_AXES = tuple(AccelerationAxis("linear-acceleration", axis) for axis in "xyz")
ANGULAR_AXES = tuple(AccelerationAxis("angular-acceleration", axis) for axis in "xyz")
POSE_AXES = (*LINEAR_AXES, *ANGULAR_AXES)


def acceleration_axes(
    *,
    controller_type: str,
    subspace: str | None,
    axis: str | None,
    command_type: str | None,
    relation: str,
    quantity_kind: str | None,
) -> tuple[AccelerationAxis, ...]:
    """Return the ordered ACHD/RNE axes implied by an authored controller."""
    if controller_type == "ImpedanceController":
        command_type = "Force"
    if command_type == "Force" or subspace == "force":
        return ()
    if command_type == "Torque" and quantity_kind == "JointPosition":
        return ()
    if quantity_kind == "Pose" and subspace in {None, "pose"} and relation == "EqualityConstraint":
        return POSE_AXES
    if subspace in {"position", "linvel"}:
        return (AccelerationAxis("linear-acceleration", axis),) if axis else LINEAR_AXES
    if subspace in {"orientation", "angvel"}:
        return (AccelerationAxis("angular-acceleration", axis),) if axis else ANGULAR_AXES
    if subspace == "distance" and axis is not None:
        return (AccelerationAxis("linear-acceleration", axis),)
    if subspace == "rotation" and axis is not None:
        return (AccelerationAxis("angular-acceleration", axis),)
    if subspace == "distance" and axis is None:
        return (AccelerationAxis("linear-acceleration", "distance"),)
    return ()
