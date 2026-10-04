# SPDX-License-Identifier: MPL-2.0
"""The dataflow contract: who writes each shared value, when, and where it is therefore stored."""

from __future__ import annotations

import dataclasses
import json
from pathlib import Path

import pytest

from motion_spec.classes.base import DataclassJSONEncoder
from motion_spec.classes.bindings import ChainBinding, HardwareBinding, RuntimeBinding
from motion_spec.classes.geometry import Direction, Pose, Position, Wrench, WrenchEstimator
from motion_spec.classes.motion import BlackboardValue, MotionSolverSlice, MotionUnit
from motion_spec.classes.qudt import Quantity, QuantityKind, Unit
from motion_spec.classes.solvers import MotionDrivers, SolverWithInputAndOutput
from motion_spec.generation.artifacts import (
    build_frame_log_header_record,
    build_schema,
    build_telemetry_model,
    field_names_and_format,
)
from motion_spec.rdf_parser.quantities import annotate_dataflow
from motion_spec.runs.archive import ArchiveError
from motion_spec.telemetry import frame_log_pb

# The storage table, restated so the test pins the contract rather than the implementation's constant.
STORAGE_BY_CADENCE = {"never": "absent", "init": "record", "tick": "log"}


@pytest.fixture
def two_motions() -> tuple[dict, list, dict, list, list, dict]:
    """`annotate_dataflow`'s inputs for two motions sharing one FK output, each with its own error.

    `arc_only_error` is written by a closure only motion_arc schedules; `pose_ee` by a solver both
    motions instantiate; `stiffness` is authored; `pose_ee_position_rel` is a comp-rob2b relation
    that nothing ever writes.
    """
    pose_ee = Pose("pose_ee", None, None, [], None, [], None)
    shared_data = [
        pose_ee,
        Quantity("arc_only_error", QuantityKind("Distance"), Unit("M"), None, False),
        Quantity("home_only_error", QuantityKind("Distance"), Unit("M"), None, False),
        Quantity("stiffness", QuantityKind("Distance"), Unit("M"), 800.0, False),
        Direction("path_normal", [], None, [], [0.0, 0.0, 1.0]),
        Position("pose_ee_position_rel", None, None, QuantityKind("Length"), None, Unit("M"), None),
    ]
    closures = {
        "eval_arc": {"id": "eval_arc", "type": "ErrorEvaluator", "error": "arc_only_error"},
        "eval_home": {"id": "eval_home", "type": "ErrorEvaluator", "error": "home_only_error"},
    }
    solver = SolverWithInputAndOutput(
        id="arm_solver",
        motion_drivers=[
            MotionDrivers(
                id="arm_solver_drivers",
                acceleration_constraint=[],
                cartesian_force=[],
                handler="move",
            )
        ],
        output=[pose_ee],
        chain=ChainBinding(
            root="base", end="ee", tip="", tree="", namespace="", name="", joints=[]
        ),
        hardware=HardwareBinding(urdf="", model="arm", tool_body="", tcp_frame=""),
        runtime=RuntimeBinding(
            id="arm_solver", owner=True, prefix="", owned_trees=[], config_key=""
        ),
    )
    motions = [
        MotionUnit(
            id=mid,
            motion_id="",
            name=mid,
            description=[],
            when_evaluators=[],
            while_evaluators=[],
            until_evaluators=[],
            controllers=[],
            when_monitors=[],
            while_monitors=[],
            until_monitors=[],
            when_schedule=[],
            while_schedule=[evaluator],
            until_schedule=[],
            index=index,
            fsm_state=state,
            serial_chain_solvers=[
                MotionSolverSlice(
                    id="arm_solver", solver_id="arm_solver", output=[pose_ee], motion_driver=None
                )
            ],
        )
        for mid, index, state, evaluator in (
            ("motion_home", 0, "S_HOME", "eval_home"),
            ("motion_arc", 1, "S_ARC", "eval_arc"),
        )
    ]
    introspection = {
        "quantities": [],
        "controllers": [],
        "monitors": [],
        "quantity_samples": [
            {"id": item.id, "source_id": item.id, "source_type": item.type, "sample_desc": desc}
            for item, desc in zip(
                shared_data[1:],
                (
                    {"kind": "shared", "id": "arc_only_error"},
                    {"kind": "shared", "id": "home_only_error"},
                    {"kind": "shared", "id": "stiffness"},
                    {"kind": "vec", "id": "path_normal", "axis": 2},
                    {"kind": "vec", "id": "pose_ee_position_rel", "axis": 0},
                ),
            )
        ],
        "spatial_samples": {"poses": [{"id": "pose_ee", "index": 0}], "twists": [], "wrenches": []},
        "control_period_ns": 1_000_000,
    }
    return introspection, shared_data, closures, motions, [solver], {}


@pytest.fixture
def schema(two_motions) -> dict:
    """The generation schema of the two motions, built from the IR they publish."""
    introspection, shared_data, closures, motions, solvers, views = two_motions
    annotate_dataflow(introspection, shared_data, closures, motions, solvers, views)
    ir = {
        "configuration": {"platform": {"name": "MuJoCo", "simulated": True, "backend": "mj_kdl"}},
        "communication": {"telemetry": introspection},
        # build_schema reads the published IR, so the motions cross as the JSON they serialize to.
        "coordination": {"motions": json.loads(json.dumps(motions, cls=DataclassJSONEncoder))},
        "computation": {"shared_data": [], "closures": {}},
    }
    fsm_ir = {"states": ["S_HOME", "S_ARC"], "events": [], "start_state": "S_HOME"}
    return build_schema(ir, ir_path=Path("ir.json"), output_dir=Path("."), fsm_ir=fsm_ir)


def test_every_member_has_a_producer_and_a_cadence_its_storage_follows(two_motions) -> None:
    introspection, shared_data, closures, motions, solvers, views = two_motions
    annotate_dataflow(introspection, shared_data, closures, motions, solvers, views)
    for member_id, entry in introspection["dataflow"].items():
        assert entry["producer"]["kind"] not in ("", "unknown"), member_id
        cadence = entry["cadence"]
        assert cadence is not None, member_id
        expected = "log" if isinstance(cadence, dict) else STORAGE_BY_CADENCE[cadence]
        assert entry["storage"] == expected, member_id


def test_read_but_never_written_members_are_reported_not_dropped(two_motions) -> None:
    introspection, shared_data, closures, motions, solvers, views = two_motions
    introspection["monitors"] = [{"id": "mon_hold", "error_signal": "pose_ee_position_rel"}]
    with pytest.raises(RuntimeError, match="pose_ee_position_rel.*mon_hold"):
        annotate_dataflow(introspection, shared_data, closures, motions, solvers, views)


def test_a_wrench_reading_is_written_only_by_the_solver_that_reads_or_estimates_it(
    two_motions,
) -> None:
    """The solver block reads the sensor, tares it and writes the reading, or runs the observer
    for an estimate: on every platform it is the one producer, so nothing may overwrite it."""
    _introspection, _shared_data, _closures, motions, solvers, _views = two_motions
    reading, estimate, command = (
        Wrench(
            id=id_,
            quantity_kind=[],
            reference_point=None,
            as_seen_by=None,
            unit=[],
            sensor_name=sensor_name,
            estimator=estimator,
        )
        for id_, sensor_name, estimator in (
            ("ext_force", "wrist_ft", None),
            ("ext_force_est", "", WrenchEstimator("arm", 30.0, 0.5)),
            ("cmd_wrench", "", None),
        )
    )
    shared_data = [
        reading,
        BlackboardValue(id="ext_force_ft_bias", type="Wrench"),
        estimate,
        BlackboardValue(id="ext_force_est_est_payload", type="Wrench"),
        command,
    ]
    closures = {
        "push": {
            "id": "push",
            "type": "WrenchFromPositionDirectionAndMagnitude",
            "wrench": "cmd_wrench",
        }
    }
    solver = dataclasses.replace(solvers[0], output=[reading, estimate])
    arc = dataclasses.replace(
        motions[1],
        index=0,
        while_schedule=["push"],
        serial_chain_solvers=[
            MotionSolverSlice(
                id="arm_solver",
                solver_id="arm_solver",
                output=[reading, estimate],
                motion_driver=None,
            )
        ],
    )
    introspection: dict = {}
    annotate_dataflow(introspection, shared_data, closures, [arc], [solver], {})
    dataflow = introspection["dataflow"]
    assert dataflow["ext_force"]["producer"] == {"kind": "sensor", "id": "arm_solver"}
    assert dataflow["ext_force_ft_bias"]["producer"]["kind"] == "sensor"
    assert dataflow["ext_force_est"]["producer"] == {"kind": "solver", "id": "arm_solver"}
    assert dataflow["ext_force_est_est_payload"]["producer"] == {"kind": "solver", "id": "arm_solver"}
    assert dataflow["cmd_wrench"]["producer"]["kind"] == "closure"


def test_a_drawn_member_is_written_at_init_but_is_no_header_constant(two_motions) -> None:
    """The run draws it before the loop, so it is init-cadence, and only the run that drew it
    knows the number, so it is neither logged per tick nor recorded in the generation's header."""
    introspection, shared_data, closures, motions, solvers, views = two_motions
    drawn = Quantity("place_near_y", QuantityKind("Distance"), Unit("M"), None, False)
    drawn.sampled = True
    shared_data.append(drawn)
    introspection["quantity_samples"].append(
        {
            "id": "place_near_y",
            "source_id": "place_near_y",
            "source_type": "Quantity",
            "sample_desc": {"kind": "shared", "id": "place_near_y"},
        }
    )
    annotate_dataflow(introspection, shared_data, closures, motions, solvers, views)

    assert introspection["dataflow"]["place_near_y"] == {
        "producer": {"kind": "sampled", "id": "place_near_y"},
        "cadence": "init",
        "storage": "record",
    }
    assert drawn in shared_data
    assert "place_near_y" not in {row["id"] for row in introspection["constants"]}
    assert "place_near_y" not in {row["id"] for row in introspection["quantity_samples"]}


@pytest.mark.parametrize(
    ("devices", "values", "decoded"),
    [
        (
            [],
            {"fsm_state": 1, "active_motion": 1, "arc_only_error": 0.0, "home_only_error": 7.5},
            {"quantities": {"arc_only_error": 0.0}},
        ),
        (
            [{"index": 0, "id": "arm1.wrist_ft", "required_by_motion": [1]}],
            {"device0.seq": 42, "device0.success": 0},
            {"devices": {"arm1.wrist_ft": {"seq": 42, "success": False}}},
        ),
    ],
    ids=["gated-slot", "ft-communication-failure"],
)
def test_a_frame_survives_the_log_round_trip(
    tmp_path: Path, schema: dict, devices: list, values: dict, decoded: dict
) -> None:
    """A genuine 0.0 in the writing motion survives while the other motion's gated slot is absent,
    not zero; an F/T read that failed stays a failure."""
    if devices:
        schema["devices"] = devices
        schema["pools"]["devices"] = len(devices)
        schema["schema_hash"] += "-ft-health"
    index_of = {q["source_id"]: q["index"] for q in schema["quantities"]}
    flat = {name: 0 for name in field_names_and_format(schema["pools"])[1]}
    flat.update({f"q{index_of[key]}" if key in index_of else key: v for key, v in values.items()})
    log = tmp_path / "frame_log.pb"
    with log.open("wb") as fh:
        frame_log_pb.write_delimited(fh, build_frame_log_header_record(schema))
        frame_log_pb.write_delimited(fh, frame_log_pb.frame_record(flat, schema))
    frame = next(iter(frame_log_pb.frame_records(log)))
    assert {key: frame[key] for key in decoded} == decoded


def test_a_gated_slot_belongs_to_its_own_motions_case(schema: dict) -> None:
    index_of = {q["source_id"]: q["index"] for q in schema["quantities"]}
    assert schema["by_motion"]["motion_arc"]["quantities"] == [index_of["arc_only_error"]]
    assert schema["by_motion"]["motion_home"]["quantities"] == [index_of["home_only_error"]]

    model = build_telemetry_model(schema, {"computation": {"shared_data": []}})
    # Gated slots move out of the unconditional block into their motion's case.
    assert model["quantities"] == []
    by_index = {
        case["index"]: [slot["index"] for slot in case["quantities"]] for case in model["motions"]
    }
    assert by_index == {0: [index_of["home_only_error"]], 1: [index_of["arc_only_error"]]}


def test_a_log_without_an_embedded_descriptor_is_rejected(tmp_path: Path, schema: dict) -> None:
    stale = frame_log_pb.record_class()()
    stale.header.schema_hash = schema["schema_hash"]  # a header carrying identity only
    log = tmp_path / "frame_log.pb"
    with log.open("wb") as fh:
        frame_log_pb.write_delimited(fh, stale.SerializeToString())
    with pytest.raises(ArchiveError, match="not a frame-log header carrying its schema"):
        frame_log_pb.read_contract(log)
