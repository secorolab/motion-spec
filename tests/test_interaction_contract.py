# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
# Author: Vamsi Kalagaturu
"""Invariants for the interaction path: force sensing, velocity control, per-motion ownership.

Each check here corresponds to a defect that a passing headless run did not reveal -- the FSM
completed start-to-end while the arm was uncommanded on an axis, blind to external force, or
recomputing another motion's state.
"""

from __future__ import annotations

import shutil
from pathlib import Path

import pytest
from support import EXAMPLES

from motion_spec.generation.pipeline import load_model
from motion_spec.rdf_parser.ir import generate_ir

ARC = EXAMPLES["arc_tracing_with_admittance"] / "arc_tracing_with_admittance.robmot"
TABLE_EDGE_REACH = Path(__file__).parent / "fixtures" / "table_edge_reach" / "table_edge_reach.robmot"


@pytest.fixture
def interaction_ir(tmp_path: Path) -> dict:
    """The force-interaction model: admittance, arc re-entry, until groups."""
    loaded = load_model(ARC, tmp_path)
    return generate_ir(loaded.model, loaded.fsm)


@pytest.fixture
def geometric_operators_ir(tmp_path: Path) -> dict:
    """A table-edge reach over Table II of Borghesan et al., "Introducing Geometric Constraint
    Expressions Into Robot Constrained Motion Specification and Control", IEEE RA-L 1(2), 2016."""
    loaded = load_model(TABLE_EDGE_REACH, tmp_path)
    return generate_ir(loaded.model, loaded.fsm)


@pytest.mark.parametrize(
    ("motion_id", "rows"),
    [
        ("motion_touchdown", {("Linear", "Z")}),
        ("motion_compliance", {("Linear", "X"), ("Linear", "Y"), ("Linear", "Z")}),
    ],
)
def test_velocity_constraints_get_a_solver_row(interaction_ir: dict, motion_id, rows) -> None:
    """Axis derivation matched stale subspace tokens, so a velocity constraint produced no solver
    row: the controller computed a signal codegen dropped, leaving the axis uncommanded."""
    motion = next(m for m in interaction_ir["coordination"]["motions"] if m.motion_id == motion_id)
    commanded = {
        (row.subspace, row.axis)
        for solver in motion.serial_chain_solvers
        for row in (
            *solver.motion_driver.acceleration_constraint,
            *solver.motion_driver.cartesian_acceleration,
        )
    }
    assert rows <= commanded, commanded


_ESTIMATED_WRENCH = """        wrench ext-force-est {
            ref-point:      <ft_tree.wrist_ft_body.wrist_ft_site>,
            as-seen-by:     <kinova.base_link.base_link_origin>,
            estimated-from: <agents.arm1> { gain: 30.0 Hz, filter: 0.5 },
            re-tare-on:     { <fsm.E_RUN_STARTED> }
        },
        wrench ext-force {"""


@pytest.mark.parametrize(
    ("old", "new"),
    [
        (
            "            ref-point:  <ft_tree.wrist_ft_body.wrist_ft_site>,\n"
            "            as-seen-by: <kinova.base_link.base_link_origin>,\n",
            "",
        ),
        ("        wrench ext-force {", _ESTIMATED_WRENCH),
    ],
    ids=["sensor-frame-default", "estimated"],
)
def test_a_wrench_reaches_the_ir_in_the_frames_its_source_states(
    tmp_path: Path, old: str, new: str
) -> None:
    """A measured wrench with no frames stated defaults to its physical sensor frame; one from the
    momentum observer carries the observer's tuning and none of the sensor path."""
    # A copy of the examples, so the edited model still finds the scene its example shares.
    models = shutil.copytree(ARC.parents[1], tmp_path / "models")
    edited = models / ARC.relative_to(ARC.parents[1])
    source = edited.read_text()
    assert old in source
    edited.write_text(source.replace(old, new, 1))
    loaded = load_model(edited, tmp_path / "model")
    ir = generate_ir(loaded.model, loaded.fsm)
    outputs = [
        output
        for solver in ir["resources"]["by_kind"]["serial_chain"]
        for output in solver.output
        if output.type == "Wrench"
    ]
    if new:
        estimated = next(output for output in outputs if output.estimator is not None)
        assert estimated.sensor_name == ""
        assert estimated.estimator.estimation_gain_hz == 30.0
        assert estimated.estimator.filter_constant == 0.5
        assert estimated.estimator.agent.endswith("arm1")
        assert (estimated.reference_point.id, estimated.as_seen_by.id) == (
            "wrist_ft_site",
            "base_link",
        )
    else:
        assert {
            (output.sensor_frame.id, output.reference_point.id, output.as_seen_by.id)
            for output in outputs
            if output.sensor_name
        } == {("wrist_ft_site", "wrist_ft_site", "wrist_ft_site")}


def test_admittance_reference_is_produced_before_it_is_consumed(interaction_ir: dict) -> None:
    """Grouped evaluators were excluded from the schedule walk, so the admittance filter was
    generated but never called and the compliant axes were a stiff zero-velocity regulator."""
    compliance = next(
        m for m in interaction_ir["coordination"]["motions"] if m.motion_id == "motion_compliance"
    )
    closures = interaction_ir["computation"]["closures"]
    admit = {name for name, closure in closures.items() if closure["type"] == "Admittance"}
    assert admit, "model declares admittance references"
    assert admit <= set(compliance.while_pre_schedule), sorted(
        admit - set(compliance.while_pre_schedule)
    )


def test_no_motion_captures_or_schedules_another_motions_state(interaction_ir: dict) -> None:
    """Snapshots write shared slots, so a motion re-capturing another's retargets it; and the
    backward schedule walk must not advance another motion's path while it is not running."""
    owners: dict[str, set[str]] = {}
    for motion in interaction_ir["coordination"]["motions"]:
        for snapshot in motion.snapshots:
            owners.setdefault(snapshot.target_id, set()).add(motion.id)
    assert not {t: m for t, m in owners.items() if len(m) > 1}
    for motion in interaction_ir["coordination"]["motions"]:
        if motion.motion_id == "motion_arc_motion":
            continue
        scheduled = set(motion.while_schedule) | set(motion.while_pre_schedule)
        assert "arc_eval_arc_path" not in scheduled, f"{motion.id} schedules the arc path"


# The scalar each Table II operator writes, and the half of the twist a solver row driving that
# scalar has to sit in: a distance moves along its gradient, an angle turns about it.
GEOMETRIC_SCALARS = {
    "PointPlaneToLinearDistance": ("distance", "Linear"),
    "PointLineToLinearDistance": ("distance", "Linear"),
    "PointOnLineProjection": ("distance", "Linear"),
    "LineLineToLinearDistance": ("distance", "Linear"),
    "LineOnLineProjection": ("distance", "Linear"),
    "DirectionPlaneToAngularDistance": ("angle", "Angular"),
    "PlanarAngleFromDirections": ("angle", "Angular"),
}

# Every field a Table II operator's call needs to be complete: drop one and the generated call
# reads an empty shared slot instead of failing to compile.
OPERATOR_FIELDS = {
    "PointPlaneToLinearDistance": ("in1", "in2", "direction", "distance", "gradient"),
    "PointLineToLinearDistance": ("in1", "in2", "direction", "distance", "gradient"),
    "PointOnLineProjection": ("in1", "in2", "direction", "distance", "gradient"),
    "LineLineToLinearDistance": ("in1", "in2", "pose", "distance", "gradient"),
    "LineOnLineProjection": ("in1", "in2", "pose", "distance", "gradient"),
    "DirectionPlaneToAngularDistance": ("in1", "in2", "angle", "gradient"),
    "PlanarAngleFromDirections": ("from_directions", "angle"),
    "RotationVectorFromDirections": ("in1", "in2", "out"),
    "AngleGradientFromDirections": ("in1", "in2", "gradient"),
}


def test_every_driven_geometric_operator_gets_its_gradient_row(geometric_operators_ir: dict) -> None:
    """Nothing downstream of the controller checks this: with no row to carry its output the axis
    is uncommanded while the FSM still reaches S_DONE. A direction pair held off zero failed here.
    An angle between two directions writes no gradient of its own: the
    `AngleGradientFromDirections` over the same pair does, and the row has to find it there."""
    closures = geometric_operators_ir["computation"]["closures"]
    operators = {
        closure[GEOMETRIC_SCALARS[closure["type"]][0]]: closure
        for closure in closures.values()
        if closure.get("type") in GEOMETRIC_SCALARS
    }
    evaluators = {
        closure["error"]: closure
        for closure in closures.values()
        if closure.get("type") == "ErrorEvaluator"
    }
    driven = {}
    for controller in closures.values():
        if controller.get("controller_type") != "ProportionalIntegralDerivative":
            continue
        evaluator = evaluators.get(controller["error_signal"])
        if evaluator is not None and evaluator["quantity"] in operators:
            driven[controller["control_signal"]] = operators[evaluator["quantity"]]
    assert driven

    rows = {
        row.acceleration_energy.id: row
        for motion in geometric_operators_ir["coordination"]["motions"]
        for solver in motion.serial_chain_solvers
        for row in solver.motion_driver.acceleration_constraint
        if row.acceleration_energy is not None
    }
    for signal, operator in driven.items():
        row = rows.get(signal)
        assert row is not None, f"{operator['id']} is driven but commands no solver row"
        assert row.subspace.name == GEOMETRIC_SCALARS[operator["type"]][1], operator["id"]
        assert row.direction is not None, f"{operator['id']} row names an axis, not its gradient"
        gradient = operator.get("gradient") or next(
            closure["gradient"]
            for closure in closures.values()
            if closure.get("type") == "AngleGradientFromDirections"
            and {closure["in1"], closure["in2"]} == set(operator["from_directions"])
        )
        assert row.direction.id == gradient


def test_a_row_reads_only_a_gradient_its_own_motion_computes(geometric_operators_ir: dict) -> None:
    """An unscheduled gradient leaves the row a zero vector every tick: the constraint is inert
    and the solver takes a degenerate row, both silently."""
    writers: dict[str, set[str]] = {}
    for closure in geometric_operators_ir["computation"]["closures"].values():
        if closure.get("gradient"):
            writers.setdefault(closure["gradient"], set()).add(closure["id"])
    for motion in geometric_operators_ir["coordination"]["motions"]:
        scheduled = {*motion.while_pre_schedule, *motion.while_schedule, *motion.until_schedule}
        for solver in motion.serial_chain_solvers:
            for row in solver.motion_driver.acceleration_constraint:
                if row.direction is not None:
                    assert writers.get(row.direction.id, set()) & scheduled, (
                        f"{motion.motion_id} commands along {row.direction.id}, "
                        "which no closure it schedules writes"
                    )


def test_every_geometric_operator_is_complete_and_called(geometric_operators_ir: dict) -> None:
    """The fixture covers every Table II operator, and none is computed into the void.

    Monitored-only operators have no controller to pull them into the schedule, so they are the
    ones that go missing: the monitor then reads a slot nobody ever wrote.
    """
    by_type: dict[str, list] = {}
    for closure in geometric_operators_ir["computation"]["closures"].values():
        by_type.setdefault(closure.get("type"), []).append(closure)
    assert set(OPERATOR_FIELDS) <= set(by_type), set(OPERATOR_FIELDS) - set(by_type)
    for type_, fields in OPERATOR_FIELDS.items():
        for closure in by_type[type_]:
            missing = [field for field in fields if not closure.get(field)]
            assert not missing, f"{closure['id']} ({type_}) is missing {missing}"

    scheduled = {
        step
        for motion in geometric_operators_ir["coordination"]["motions"]
        for step in (*motion.while_pre_schedule, *motion.while_schedule, *motion.until_schedule)
    }
    uncalled = [
        closure["id"]
        for type_ in OPERATOR_FIELDS
        for closure in by_type[type_]
        if closure["id"] not in scheduled
    ]
    assert not uncalled, f"operators generated but never called: {uncalled}"
