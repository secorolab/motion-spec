# SPDX-License-Identifier: MPL-2.0
"""The dataflow contract: who writes each shared value, when, and where it is therefore stored."""

from __future__ import annotations

import json
from pathlib import Path

import pytest
from frame_log_fixture import flat_frame

from motion_spec.classes.base import DataclassJSONEncoder
from motion_spec.classes.bindings import ChainBinding, HardwareBinding, RuntimeBinding
from motion_spec.classes.geometry import Direction, Pose, Position, Wrench
from motion_spec.classes.motion import BlackboardValue, MotionSolverSlice, MotionUnit
from motion_spec.classes.qudt import Quantity, QuantityKind, Unit
from motion_spec.classes.solvers import MotionDrivers, SolverWithInputAndOutput
from motion_spec.generation.artifacts import build_introspection_model, build_schema
from motion_spec.introspection import frame_log_pb
from motion_spec.rdf_parser.quantities import annotate_dataflow

# The plan's storage table, restated here so the test pins the contract rather than the constant
# the implementation happens to use.
STORAGE_BY_CADENCE = {"never": "absent", "init": "record", "tick": "log"}


def _quantity(id_: str, value: float | None) -> Quantity:
    return Quantity(id_, QuantityKind("Distance"), Unit("M"), value, False)


def _wrench(id_: str, sensor_name: str = "") -> Wrench:
    return Wrench(
        id=id_,
        quantity_kind=[],
        reference_point=None,
        as_seen_by=None,
        unit=[],
        sensor_name=sensor_name,
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
        handler="",
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


def _schema() -> dict:
    introspection, _shared = _annotated()
    # build_schema reads the published IR, so the motions cross as the JSON they serialize to.
    ir = {
        "configuration": {"platform": {"name": "MuJoCo", "simulated": True, "backend": "mj_kdl"}},
        "communication": {"introspection": introspection},
        "coordination": {"motions": json.loads(json.dumps(_model()[3], cls=DataclassJSONEncoder))},
        "computation": {"shared_data": []},
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


def test_externally_measured_is_the_sensor_reading_and_nothing_else() -> None:
    """The projection asks "does the platform supply it?", not "is it a Wrench nobody writes?"."""
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
    values = annotate_dataflow(introspection, shared_data, closures, motions, [solver], {})
    # The tare state is a Wrench written alongside the reading but computed by the program, and
    # cmd_wrench is the program's own output: neither is externally measured.
    assert [item.id for item in values["externally_measured"]] == ["ext_force"]


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
