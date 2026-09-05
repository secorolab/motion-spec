# SPDX-License-Identifier: MPL-2.0
"""The dataflow contract: who writes each shared value, when, and where it is therefore stored."""

from __future__ import annotations

import json
from pathlib import Path

import pytest
from frame_log_fixture import flat_frame, write_frame_log_proto
from google.protobuf import descriptor_pb2

from motion_spec.classes.base import DataclassJSONEncoder
from motion_spec.classes.bindings import ChainBinding, HardwareBinding, RuntimeBinding
from motion_spec.classes.dynamics import Saturation
from motion_spec.classes.geometry import Direction, Pose, Position, Wrench, WrenchEstimator
from motion_spec.classes.motion import BlackboardValue, MotionSolverSlice, MotionUnit
from motion_spec.classes.qudt import FreeVector, Quantity, QuantityKind, Unit
from motion_spec.classes.solvers import MotionDrivers, SolverWithInputAndOutput
from motion_spec.generation.artifacts import (
    build_frame_log_proto_fields,
    build_introspection_model,
    build_schema,
)
from motion_spec.introspection import frame_log_pb
from motion_spec.rdf_parser.quantities import annotate_dataflow

# The plan's storage table, restated here so the test pins the contract rather than the constant
# the implementation happens to use.
STORAGE_BY_CADENCE = {"never": "absent", "init": "record", "tick": "log"}


def _quantity(id_: str, value: float | None) -> Quantity:
    return Quantity(id_, QuantityKind("Distance"), Unit("M"), value, False)


def _wrench(id_: str, sensor_name: str = "", estimator: WrenchEstimator | None = None) -> Wrench:
    return Wrench(
        id=id_,
        quantity_kind=[],
        reference_point=None,
        as_seen_by=None,
        unit=[],
        sensor_name=sensor_name,
        estimator=estimator,
    )


def _solver(sid: str, output: list) -> SolverWithInputAndOutput:
    """A full chain solver: what `annotate_dataflow` reads writers off."""
    return SolverWithInputAndOutput(
        id=sid,
        motion_drivers=[
            MotionDrivers(id=f"{sid}_drivers", acceleration_constraint=[], cartesian_force=[])
        ],
        output=output,
        chain=ChainBinding(root="base", end="ee", tip="", tree="", name="", joints=[]),
        hardware=HardwareBinding(urdf="", model="arm", tool_body="", tcp_frame=""),
        runtime=RuntimeBinding(id=sid, owner=True, prefix="", owned_trees=[], config_key=""),
    )


def _slice(sid: str, output: list) -> MotionSolverSlice:
    """The per-motion slice of that solver: by reference, no copies (024 §2 F2)."""
    return MotionSolverSlice(id=sid, solver_id=sid, output=output, motion_driver=None)


def _motion(mid: str, index: int, fsm_state: str, schedule: list, solvers: list) -> MotionUnit:
    return MotionUnit(
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
        while_schedule=schedule,
        until_schedule=[],
        index=index,
        fsm_state=fsm_state,
        serial_chain_solvers=solvers,
    )


def _model() -> tuple[dict, list, dict, list, list, dict]:
    """A two-motion model: one shared FK output, one per-motion error, constants and dead relations.

    `arc_only_error` is written by a closure only motion_arc schedules; `pose_ee` by a solver both
    motions instantiate; `stiffness` is authored; `pose_ee_position_rel` is a comp-rob2b relation
    that nothing ever writes.
    """
    shared_data = [
        Pose("pose_ee", None, None, [], None, [], None),
        _quantity("arc_only_error", None),
        _quantity("home_only_error", None),
        _quantity("stiffness", 800.0),
        Direction("path_normal", [], None, [], [0.0, 0.0, 1.0]),
        Position("pose_ee_position_rel", None, None, QuantityKind("Length"), None, Unit("M"), None),
    ]
    closures = {
        "eval_arc": {"id": "eval_arc", "type": "ErrorEvaluator", "error": "arc_only_error"},
        "eval_home": {"id": "eval_home", "type": "ErrorEvaluator", "error": "home_only_error"},
    }
    solver = _solver("arm_solver", output=[shared_data[0]])
    slice_ = _slice("arm_solver", output=[shared_data[0]])
    motions = [
        _motion("motion_home", 0, "S_HOME", ["eval_home"], [slice_]),
        _motion("motion_arc", 1, "S_ARC", ["eval_arc"], [slice_]),
    ]
    introspection = {
        "quantities": [],
        "controllers": [],
        "monitors": [],
        "quantity_samples": [
            {"id": item.id, "source_id": item.id, "source_type": item.type, "sample_desc": desc}
            for item, desc in (
                (shared_data[1], {"kind": "shared", "id": "arc_only_error"}),
                (shared_data[2], {"kind": "shared", "id": "home_only_error"}),
                (shared_data[3], {"kind": "shared", "id": "stiffness"}),
                (shared_data[4], {"kind": "vec", "id": "path_normal", "axis": 2}),
                (shared_data[5], {"kind": "vec", "id": "pose_ee_position_rel", "axis": 0}),
            )
        ],
        "spatial_samples": {"poses": [{"id": "pose_ee", "index": 0}], "twists": [], "wrenches": []},
        "control_period_ns": 1_000_000,
        "provenance": {
            "activities": [
                {
                    "id": "activity:controller_execution",
                    "role": "controller_execution",
                    "wasAssociatedWith": "agent:controller_process",
                }
            ],
            "agents": [
                {"id": "agent:controller_process", "role": "controller_process"},
                {"id": "agent:runtime:mujoco", "role": "runtime_runner"},
            ],
        },
    }
    return introspection, shared_data, closures, motions, [solver], {}


def _annotated() -> tuple[dict, list]:
    introspection, shared_data, closures, motions, solvers, views = _model()
    annotate_dataflow(introspection, shared_data, closures, motions, solvers, views)
    return introspection, shared_data


def _schema(*, closures: dict | None = None, controllers: list | None = None) -> dict:
    introspection, _shared = _annotated()
    # build_schema reads the published IR, so the motions cross as the JSON they serialize to.
    motions = json.loads(json.dumps(_model()[3], cls=DataclassJSONEncoder))
    if controllers is not None:
        motions[1]["controllers"] = controllers  # motion_arc
    ir = {
        "configuration": {"platform": {"name": "MuJoCo", "simulated": True, "backend": "mj_kdl"}},
        "communication": {"introspection": introspection},
        "coordination": {"motions": motions},
        "computation": {"shared_data": [], "closures": closures or {}},
    }
    fsm_ir = {"states": ["S_HOME", "S_ARC"], "events": [], "start_state": "S_HOME"}
    return build_schema(ir, ir_path=Path("ir.json"), output_dir=Path("."), fsm_ir=fsm_ir)


def test_every_member_has_a_producer_and_a_cadence() -> None:
    introspection, _shared = _annotated()
    for member_id, entry in introspection["dataflow"].items():
        assert entry["producer"]["kind"], member_id
        assert entry["producer"]["kind"] != "unknown", member_id
        assert entry["cadence"] is not None, member_id


def test_storage_is_derived_from_cadence_for_every_member() -> None:
    introspection, _shared = _annotated()
    for member_id, entry in introspection["dataflow"].items():
        cadence = entry["cadence"]
        expected = "log" if isinstance(cadence, dict) else STORAGE_BY_CADENCE[cadence]
        assert entry["storage"] == expected, member_id


def test_a_value_written_by_several_motions_is_one_producer_over_all_of_them() -> None:
    introspection, _shared = _annotated()
    pose = introspection["dataflow"]["pose_ee"]
    assert pose["producer"]["kind"] == "solver"
    # One solver declaration, instantiated per motion: the cadence unions them rather than
    # attributing the value to whichever motion happened to be seen last.
    assert pose["cadence"] == {"motions": ["motion_arc", "motion_home"]}


def test_a_sensor_reading_is_produced_by_the_solver_that_reads_it() -> None:
    """The solver block reads the sensor, tares it and writes the reading: on every platform it is
    the one producer, so nothing else may claim to supply it and overwrite what it wrote."""
    reading = _wrench("ext_force", sensor_name="wrist_ft")
    shared_data = [
        reading,
        BlackboardValue(id="ext_force_ft_bias", type="Wrench"),
        BlackboardValue(id="ext_force_ft_settle", type="IntCounter"),
        _wrench("cmd_wrench"),
    ]
    closures = {
        "push": {
            "id": "push",
            "type": "WrenchFromPositionDirectionAndMagnitude",
            "wrench": "cmd_wrench",
        }
    }
    solver = _solver("arm_solver", output=[reading])
    motions = [_motion("motion_arc", 0, "S_ARC", ["push"], [_slice("arm_solver", [reading])])]
    introspection: dict = {}
    annotate_dataflow(introspection, shared_data, closures, motions, [solver], {})
    dataflow = introspection["dataflow"]
    assert dataflow["ext_force"]["producer"] == {"kind": "sensor", "id": "arm_solver"}
    # The tare state is written alongside the reading; cmd_wrench is the program's own output.
    assert dataflow["ext_force_ft_bias"]["producer"]["kind"] == "sensor"
    assert dataflow["cmd_wrench"]["producer"]["kind"] == "closure"


def test_an_estimated_wrench_is_a_plain_solver_write() -> None:
    """Nothing measures it: the solver that runs the observer is its producer, the same as any
    other value the solver block computes, and so is the payload it takes out at a tare."""
    estimate = _wrench("ext_force_est", estimator=WrenchEstimator("arm", 30.0, 0.5))
    payload = BlackboardValue(id="ext_force_est_est_payload", type="Wrench")
    solver = _solver("arm_solver", output=[estimate])
    motions = [_motion("motion_arc", 0, "S_ARC", [], [_slice("arm_solver", [estimate])])]
    introspection: dict = {}
    annotate_dataflow(introspection, [estimate, payload], {}, motions, [solver], {})

    producer = {"kind": "solver", "id": "arm_solver"}
    assert introspection["dataflow"]["ext_force_est"]["producer"] == producer
    assert introspection["dataflow"]["ext_force_est_est_payload"]["producer"] == producer


def test_a_recorded_constant_carries_its_iri_and_who_reads_it() -> None:
    """A constant is a model term someone reads, not just a number: both facts travel with it."""
    stiffness = _quantity("stiffness", 800.0)
    introspection = {
        "quantities": [],
        "controllers": [{"id": "ctrl_push", "setpoint_signal": "stiffness"}],
        "monitors": [],
        "quantity_samples": [
            {
                "id": "stiffness",
                "source_id": "stiffness",
                "source_type": stiffness.type,
                "uri": "https://example.test/stiffness",
                "sample_desc": {"kind": "shared", "id": "stiffness"},
            }
        ],
    }
    annotate_dataflow(introspection, [stiffness], {}, [], [], {})
    (constant,) = introspection["constants"]
    assert constant["uri"] == "https://example.test/stiffness"
    assert constant["consumers"] == [
        {"kind": "controller", "id": "ctrl_push", "role": "setpoint_signal"}
    ]


def _sample(item, desc: dict) -> dict:
    return {"id": item.id, "source_id": item.id, "source_type": item.type, "sample_desc": desc}


def test_a_solver_is_recorded_as_the_reader_of_the_limits_it_is_built_with() -> None:
    """A torque bound and a gravity field are read by the solver the model hung them on. Without
    that the recorded limit says nothing about whose limit it is."""
    limit = _quantity("arm_torque_limit", 39.0)
    gravity = FreeVector(
        "gravity_value_arm", QuantityKind("Acceleration"), Unit("M_PER_SEC2"), [0.0, 0.0, -9.81]
    )
    tau = _quantity("tau_arm", None)
    solver = _solver("arm_solver", output=[])
    solver.torque_saturation = Saturation("sat_torque_arm", tau, tau, limit, None, None)
    solver.gravity_source = gravity.id
    introspection = {
        "quantity_samples": [
            _sample(limit, {"kind": "shared", "id": limit.id}),
            _sample(gravity, {"kind": "vec", "id": gravity.id, "axis": 2}),
        ]
    }
    annotate_dataflow(introspection, [limit, gravity], {}, [], [solver], {})
    by_id = {row["id"]: row for row in introspection["constants"]}
    assert by_id["arm_torque_limit"]["consumers"] == [
        {"kind": "solver", "id": "arm_solver", "role": "torque_saturation.maximum"}
    ]
    assert by_id["gravity_value_arm"]["consumers"] == [
        {"kind": "solver", "id": "arm_solver", "role": "gravity"}
    ]


def test_a_gain_published_as_a_shared_value_is_read_by_its_controller() -> None:
    """Gains live in a sub-dict of the controller closure, so a flat scan of its fields reports
    every gain in the model as read by nobody."""
    kp = _quantity("ctrl_push_kp", 200.0)
    closures = {
        "ctrl_push": {
            "id": "ctrl_push",
            "type": "Controller",
            "control_signal": "cmd_push",
            "gains": {"kp": kp.id, "kd": None},
        }
    }
    introspection = {"quantity_samples": [_sample(kp, {"kind": "shared", "id": kp.id})]}
    annotate_dataflow(introspection, [kp], closures, [], [], {})
    (constant,) = introspection["constants"]
    assert constant["consumers"] == [{"kind": "closure", "id": "ctrl_push", "role": "gains.kp"}]


def test_a_while_constraints_cone_is_read_by_the_evaluator_that_compares_it() -> None:
    """A `while` constraint no controller drives and no monitor watches still gets its evaluator,
    so the cone it is banded by has a recorded reader instead of reaching the program as a
    constant nothing compares."""
    cone = _quantity("align_cone", 0.35)
    error = _quantity("eval_home_while_align_forearm_err", None)
    closures = {
        "eval_home_while_align_forearm": {
            "id": "eval_home_while_align_forearm",
            "type": "ErrorEvaluator",
            "constraint": "BilateralConstraint",
            "quantity": "forearm_alignment",
            "lower_threshold": "align_forearm_lower",
            "upper_threshold": cone.id,
            "error": error.id,
        }
    }
    introspection = {"quantity_samples": [_sample(cone, {"kind": "shared", "id": cone.id})]}
    annotate_dataflow(introspection, [cone, error], closures, [], [], {})
    (constant,) = introspection["constants"]
    assert constant["consumers"] == [
        {"kind": "closure", "id": "eval_home_while_align_forearm", "role": "upper_threshold"}
    ]


def test_a_snapshot_latch_is_read_by_the_motion_that_captures_it() -> None:
    """The guard on a run-scoped capture is a shared value, and the IR names it. A template that
    spelled the name itself would leave the latch with no recorded reader at all."""
    from motion_spec.classes.motion import SnapshotCapture

    latch = BlackboardValue(id="hold_pose_captured", type="Bool", value=False)
    motion = _motion("motion_home", 0, "S_HOME", [], [])
    motion.task_snapshots = [
        SnapshotCapture(
            target_id="hold_pose", source_id="pose_ee", scope="task", captured_id=latch.id
        )
    ]
    introspection = {"quantity_samples": [_sample(latch, {"kind": "bool", "id": latch.id})]}
    annotate_dataflow(introspection, [latch], {}, [motion], [], {})
    (constant,) = introspection["constants"]
    assert constant["consumers"] == [
        {"kind": "motion", "id": "motion_home", "role": "snapshot.captured"}
    ]


def test_a_monitor_debounce_is_read_by_the_monitor_it_gates() -> None:
    """The hold duration is the model's own quantity, read off the blackboard rather than copied
    into the edge test: one source of truth, and a recorded reader."""
    debounce = _quantity("mon_settled_debounce", 0.3)
    introspection = {
        "monitors": [{"id": "mon_settled", "debounce_signal": debounce.id}],
        "quantity_samples": [_sample(debounce, {"kind": "shared", "id": debounce.id})],
    }
    annotate_dataflow(introspection, [debounce], {}, [], [], {})
    (constant,) = introspection["constants"]
    assert constant["consumers"] == [{"kind": "monitor", "id": "mon_settled", "role": "debounce"}]


def test_a_value_no_reader_binds_is_recorded_with_no_consumers() -> None:
    """Scene geometry is baked into a pose while generating; nothing reads it per tick. The
    empty answer is the deriver's, not a gap -- so no reader may be invented to fill it."""
    anchor = Position(
        "anchor_on_table_position_coord",
        None,
        None,
        QuantityKind("Length"),
        None,
        Unit("M"),
        [0.0, 0.0, 0.4],
    )
    introspection = {
        "quantity_samples": [_sample(anchor, {"kind": "vec", "id": anchor.id, "axis": 2})]
    }
    annotate_dataflow(introspection, [anchor], {}, [], [], {})
    (constant,) = introspection["constants"]
    assert constant["value"] == 0.4
    assert "consumers" not in constant


def test_a_shared_quantity_behind_an_axis_less_view_is_still_sampled() -> None:
    """An axis-less MAP view names no field to read, but the value has a shared field of its own."""
    from motion_spec.classes.geometry import Subspace, View
    from motion_spec.rdf_parser.communication import add_quantity_samples

    error = _quantity("err_lin_normal_a", None)
    pose = Pose("pose_ee", None, None, [], None, [], None)
    views = {
        "err_view": View(
            id="err_view",
            superobject=pose,
            subobject=error,
            subspace=Subspace.Linear,
            axis=None,
            direction=Direction("path_normal", [], None, [], [0.0, 0.0, 1.0]),
        )
    }
    introspection: dict = {"quantities": [{"id": error.id, "type": error.type}]}
    add_quantity_samples(introspection, [error, pose], views)
    descs = {row["source_id"]: row["sample_desc"] for row in introspection["quantity_samples"]}
    assert descs["err_lin_normal_a"] == {"kind": "shared", "id": "err_lin_normal_a"}


def test_never_written_members_leave_shared_data_and_the_frame() -> None:
    introspection, shared_data = _annotated()
    assert introspection["dataflow"]["pose_ee_position_rel"]["cadence"] == "never"
    assert "pose_ee_position_rel" not in {item.id for item in shared_data}
    assert "pose_ee_position_rel" not in {
        row["source_id"] for row in introspection["quantity_samples"]
    }


def test_read_but_never_written_members_are_reported_not_dropped() -> None:
    introspection, shared_data, closures, motions, solvers, views = _model()
    introspection["monitors"] = [{"id": "mon_hold", "error_signal": "pose_ee_position_rel"}]
    with pytest.raises(RuntimeError, match="pose_ee_position_rel.*mon_hold"):
        annotate_dataflow(introspection, shared_data, closures, motions, solvers, views)


def test_init_members_become_schema_constants_and_no_per_tick_sample() -> None:
    schema = _schema()
    constants = {entry["id"]: entry["value"] for entry in schema["constants"]}
    # The vector row resolves to its own axis of the authored literal, not to the whole vector.
    assert constants == {"stiffness": 800.0, "path_normal": 1.0}
    assert not [q for q in schema["quantities"] if q["source_id"] in ("stiffness", "path_normal")]


def test_gated_slots_appear_only_in_the_motions_that_write_them() -> None:
    schema = _schema()
    index_of = {q["source_id"]: q["index"] for q in schema["quantities"]}
    assert schema["by_motion"]["motion_arc"]["quantities"] == [index_of["arc_only_error"]]
    assert schema["by_motion"]["motion_home"]["quantities"] == [index_of["home_only_error"]]

    model = build_introspection_model(schema, {"computation": {"shared_data": []}})
    # Gated slots move out of the unconditional block into their motion's case.
    assert model["quantities"] == []
    by_index = {
        case["index"]: [slot["index"] for slot in case["quantities"]] for case in model["motions"]
    }
    assert by_index == {0: [index_of["home_only_error"]], 1: [index_of["arc_only_error"]]}


def test_decoding_yields_only_the_slots_the_frame_s_motion_writes() -> None:
    schema = _schema()
    index_of = {q["source_id"]: q["index"] for q in schema["quantities"]}
    flat = flat_frame(
        schema,
        fsm_state=1,
        active_motion=1,  # motion_arc
        **{f"q{index_of['arc_only_error']}": 0.0, f"q{index_of['home_only_error']}": 7.5},
    )
    # Round-trip through a real log so the gate comes from the header, as in production.
    import tempfile

    from motion_spec.generation.artifacts import build_frame_log_header_record

    with tempfile.TemporaryDirectory() as tmp:
        log = Path(tmp) / "frame_log.pb"
        with log.open("wb") as fh:
            frame_log_pb.write_delimited(fh, build_frame_log_header_record(schema))
            frame_log_pb.write_delimited(fh, frame_log_pb.frame_record(flat, schema))
        decoded = next(iter(frame_log_pb.frame_records(log)))
    # A genuine 0.0 in the writing motion survives; the other motion's slot is absent rather than
    # decoded as zero -- absence never stands in for "inactive" on the wire.
    assert decoded["quantities"] == {"arc_only_error": 0.0}


def test_pose_difference_and_acceleration_twist_rows_survive_dedup() -> None:
    from motion_spec.classes.geometry import AccelerationTwist, PoseDifference
    from motion_spec.rdf_parser.communication import add_quantity_samples

    def spatial(cls, id_):
        return cls(id=id_, quantity_kind=[], reference_point=None, as_seen_by=None, unit=[])

    introspection = {
        "quantities": [
            {"id": "pose_ee", "type": "Pose"},
            {"id": "pose_diff", "type": "PoseDifference"},
            {"id": "acc_ee", "type": "AccelerationTwist"},
        ]
    }
    shared_data = [
        Pose("pose_ee", None, None, [], None, [], None),
        spatial(PoseDifference, "pose_diff"),
        spatial(AccelerationTwist, "acc_ee"),
    ]
    add_quantity_samples(introspection, shared_data, {})
    sampled = {row["source_id"] for row in introspection["quantity_samples"]}
    # pose_ee carries a whole-object PoseSlot, so its scalar rows would be a duplicate; the other
    # two have no slot, so their scalar rows are the only record of them.
    assert sampled == {"pose_diff", "acc_ee"}


def test_run_schema_carries_the_contract_for_every_logged_member(tmp_path: Path) -> None:
    schema = _schema()
    (tmp_path / "schema.json").write_text(json.dumps(schema))
    catalogue = json.loads((tmp_path / "schema.json").read_text())["catalogue"]
    assert not [entry for entry in catalogue if entry["storage"] == "absent"]
    assert not [entry for entry in catalogue if not entry.get("producer")]
    assert {entry["id"] for entry in catalogue} == {
        "pose_ee",
        "arc_only_error",
        "home_only_error",
        "stiffness",
        "path_normal",
    }


# --- the log describes itself (plan 016) ----------------------------------------------------
def _written_log(tmp: Path, schema: dict, flats: list[dict]) -> Path:
    from motion_spec.generation.artifacts import build_frame_log_header_record

    log = tmp / "frame_log.pb"
    with log.open("wb") as fh:
        frame_log_pb.write_delimited(fh, build_frame_log_header_record(schema))
        for flat in flats:
            frame_log_pb.write_delimited(fh, frame_log_pb.frame_record(flat, schema))
    return log


def test_ft_communication_failure_survives_the_log_round_trip(tmp_path: Path) -> None:
    schema = _schema()
    schema["devices"] = [{"index": 0, "id": "arm1.wrist_ft", "required_by_motion": [1]}]
    schema["pools"]["devices"] = 1
    schema["protobuf"] = build_frame_log_proto_fields(schema)
    schema["schema_hash"] += "-ft-health"
    log = _written_log(
        tmp_path, schema, [flat_frame(schema, **{"device0.seq": 42, "device0.success": 0})]
    )
    frame = next(iter(frame_log_pb.frame_records(log)))
    assert frame["devices"] == {"arm1.wrist_ft": {"seq": 42, "success": False}}


def test_a_log_decodes_with_no_companion_artifact(tmp_path: Path) -> None:
    schema = _schema()
    index_of = {q["source_id"]: q["index"] for q in schema["quantities"]}
    log = _written_log(
        tmp_path,
        schema,
        [
            flat_frame(
                schema,
                step=3,
                fsm_state=1,
                active_motion=1,
                **{f"q{index_of['arc_only_error']}": 1.25},
            )
        ],
    )
    # Nothing but the log file is in scope here -- no schema, no proto, no descriptor on disk.
    contract = frame_log_pb.read_contract(log)
    frames = list(frame_log_pb.frame_records(log, contract))
    assert len(frames) == 1
    assert frames[0]["step"] == 3
    assert frames[0]["quantities"] == {"arc_only_error": 1.25}


def test_the_embedded_descriptor_alone_rebuilds_the_frame_message(tmp_path: Path) -> None:
    from google.protobuf import descriptor_pb2, descriptor_pool, message_factory

    schema = _schema()
    contract = frame_log_pb.read_contract(_written_log(tmp_path, schema, []))
    descriptor_set = descriptor_pb2.FileDescriptorSet()
    descriptor_set.ParseFromString(contract.header.descriptor_set)
    pool = descriptor_pool.DescriptorPool()
    for file_proto in descriptor_set.file:
        pool.Add(file_proto)
    frame_cls = message_factory.GetMessageClass(
        pool.FindMessageTypeByName("motion_spec.introspection.log.RuntimeFrame")
    )
    names = {field.name for field in frame_cls.DESCRIPTOR.fields}
    # Slot fields are named from their model id, so the descriptor is readable on its own.
    assert "arc_only_error" in names and "home_only_error" in names


def test_every_slot_carries_its_model_iri(tmp_path: Path) -> None:
    schema = _schema()
    contract = frame_log_pb.read_contract(_written_log(tmp_path, schema, []))
    by_id = {slot.id: slot.iri for slot in contract.header.slots}
    quantity_ids = {q["id"] for q in schema["quantities"]}
    assert quantity_ids <= set(by_id)
    # An id is a lossy projection of its IRI, so the IRI has to travel rather than be recomputed.
    assert all(by_id[q["id"]] == q["uri"] for q in schema["quantities"] if q.get("uri"))


def test_a_slot_carries_what_it_serves_and_a_constant_who_reads_it(tmp_path: Path) -> None:
    """Identity beyond the slot number: the constraint, the phase, the joined quantity ids, the
    gains, and a constant's model term with its readers -- all header-only, none per frame."""
    schema = _schema()
    schema["by_motion"]["motion_arc"]["controllers"] = [
        {
            "index": 0,
            "id": "ctrl_push",
            "uri": "https://example.test/ctrl_push",
            "constraint": "hold",
            "constraint_uri": "https://example.test/hold",
            "gains": {"proportional_gain": 12.0},
            "error_signal": "arc_only_error",
            "output_signal": "cmd_wrench",
            "measured_signal": "pose_ee",
            "setpoint_signal": "stiffness",
        }
    ]
    schema["by_motion"]["motion_arc"]["monitors"] = [
        {
            "index": 0,
            "id": "mon_done",
            "phase": "until",
            "constraint_ids": ["hold", "settled"],
            "constraint_uris": ["https://example.test/hold", "https://example.test/settled"],
            "error_signal": "arc_only_error",
        }
    ]
    (stiffness,) = [row for row in schema["constants"] if row["id"] == "stiffness"]
    stiffness["uri"] = "https://example.test/stiffness"
    stiffness["consumers"] = [{"kind": "controller", "id": "ctrl_push", "role": "setpoint_signal"}]

    header = frame_log_pb.read_contract(_written_log(tmp_path, schema, [])).header
    gate = next(motion for motion in header.motions if motion.id == "motion_arc")
    (controller,) = gate.controllers
    assert controller.constraint_iri == "https://example.test/hold"
    assert controller.constraint_id == "hold"
    assert (controller.measured_id, controller.setpoint_id) == ("pose_ee", "stiffness")
    assert (controller.error_id, controller.output_id) == ("arc_only_error", "cmd_wrench")
    assert [(gain.role, gain.value) for gain in controller.gains] == [("proportional_gain", 12.0)]
    (monitor,) = gate.monitors
    assert monitor.phase == "until"
    # An aggregate watches several constraints, so the scalar stays empty rather than picking one.
    assert list(monitor.constraint_iris) == list(
        schema["by_motion"]["motion_arc"]["monitors"][0]["constraint_uris"]
    )
    assert monitor.constraint_iri == "" and monitor.constraint_id == ""
    logged = next(row for row in header.constants if row.id == "stiffness")
    assert logged.uri == "https://example.test/stiffness"
    assert [(c.kind, c.id, c.role) for c in logged.consumers] == [
        ("controller", "ctrl_push", "setpoint_signal")
    ]


def test_a_slot_names_the_evaluator_and_the_pair_it_compares(tmp_path: Path) -> None:
    """The header says what a constraint's error is computed from: the evaluator closure, the two
    quantities it compares and the difference it writes. A pose pair fills no measured/setpoint,
    so a reader holding only those had nothing to plot."""
    schema = _schema(
        closures={
            "eval_pose": {
                "id": "eval_pose",
                "type": "PoseDiffEvaluator",
                "in1": "pose_ee",
                "in2": "pose_target",
                "out": "pose_diff",
                # Only the difference is computed; the per-axis errors are views onto it.
                "errors": ["arc_only_error"],
            },
            "eval_reach": {
                "id": "eval_reach",
                "type": "ErrorEvaluator",
                "quantity": "home_only_error",
                "reference_value": 0.25,
                "error": "home_only_error",
            },
        },
        controllers=[
            {"id": "ctrl_pose", "error_signal": "arc_only_error"},
            {"id": "ctrl_reach", "error_signal": "home_only_error"},
            {"id": "ctrl_free", "error_signal": "stiffness"},
        ],
    )
    header = frame_log_pb.read_contract(_written_log(tmp_path, schema, [])).header
    gate = next(motion for motion in header.motions if motion.id == "motion_arc")
    pose, reach, free = gate.controllers
    assert list(pose.operand_ids) == ["pose_ee", "pose_target"]
    assert (pose.difference_id, pose.evaluator_id) == ("pose_diff", "eval_pose")
    assert (pose.measured_id, pose.setpoint_id) == ("", "")
    # A numeric reference value is a value, not an id: only the measured quantity is an operand.
    assert list(reach.operand_ids) == ["home_only_error"]
    assert (reach.difference_id, reach.evaluator_id) == ("", "eval_reach")
    # No closure produces this error, so the slot says nothing rather than guessing.
    assert (list(free.operand_ids), free.difference_id, free.evaluator_id) == ([], "", "")


# Proto type words the descriptor builder's scalar types render as in the .proto text.
_PROTO_WORD = {
    getattr(descriptor_pb2.FieldDescriptorProto, f"TYPE_{word.upper()}"): word
    for word in (
        "uint32",
        "uint64",
        "int32",
        "int64",
        "string",
        "bytes",
        "bool",
        "double",
        "sfixed64",
    )
}


def _declared_in_proto_text(text: str) -> dict:
    """{message: {field: (type word, number, repeated)}}, parsed off the rendered .proto."""
    messages: dict = {}
    current, depth = None, 0
    for raw in text.splitlines():
        line = raw.split("//")[0].strip()
        if line.startswith("message "):
            current, depth = line.split()[1], 1
            messages[current] = {}
        elif current is None or not line:
            continue
        elif line.endswith("{"):
            depth += 1
        elif line == "}":
            depth -= 1
            if depth == 0:
                current = None
        elif line.endswith(";"):
            parts = line[:-1].split()
            repeated = parts[0] == "repeated"
            type_word, name, _eq, number = parts[1:] if repeated else parts
            messages[current][name] = (type_word, int(number), repeated)
    return messages


def _declared_in_descriptor(fields: dict) -> dict:
    D = descriptor_pb2.FieldDescriptorProto
    return {
        message.name: {
            field.name: (
                field.type_name.rsplit(".", 1)[-1]
                if field.type == D.TYPE_MESSAGE
                else _PROTO_WORD[field.type],
                field.number,
                field.label == D.LABEL_REPEATED,
            )
            for field in message.field
        }
        for message in frame_log_pb._build_file_descriptor(fields).message_type
    }


def test_the_proto_text_and_the_python_descriptor_declare_the_same_wire(tmp_path: Path) -> None:
    """protoc reads the template's .proto, replay reads the Python descriptor: one contract, so a
    field added to either without the other would decode a log nobody wrote."""
    schema = _schema()
    proto = tmp_path / "frame_log.proto"
    write_frame_log_proto(proto, schema)
    assert _declared_in_proto_text(proto.read_text()) == _declared_in_descriptor(
        frame_log_pb._proto_fields(schema)
    )


def test_a_log_without_an_embedded_descriptor_is_rejected(tmp_path: Path) -> None:
    from motion_spec.introspection.archive import ArchiveError

    schema = _schema()
    record_cls, _ = frame_log_pb._record_class(schema)
    stale = record_cls()
    stale.header.schema_hash = schema["schema_hash"]  # a pre-v3 header: identity only
    log = tmp_path / "frame_log.pb"
    with log.open("wb") as fh:
        frame_log_pb.write_delimited(fh, stale.SerializeToString())
    with pytest.raises(ArchiveError, match="no descriptor set"):
        frame_log_pb.read_contract(log)
