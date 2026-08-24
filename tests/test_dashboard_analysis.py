# SPDX-License-Identifier: MPL-2.0
"""Statistics over a run: the clock they are measured on, and what each report finds."""

from __future__ import annotations

import math
from types import SimpleNamespace

from motion_spec.dashboard import analysis
from motion_spec.introspection import frame_log_pb

from dashboard_fixture import schema
from frame_log_fixture import flat_frame, write_frame_log_pb
from support import _hash_doc

PERIOD_NS = 1_000_000


def _schema(states=("S_MOVE",)):
    doc = schema()
    doc["control_period_ns"] = PERIOD_NS
    doc["fsm"]["states"] = [
        {"index": index, "id": name, "uri": f"https://example.test/{name}"}
        for index, name in enumerate(states)
    ]
    doc["schema_hash"] = _hash_doc(doc)
    return doc


def _log(tmp_path, doc, frames):
    path = tmp_path / "frame_log.pb"
    write_frame_log_pb(path, doc, [flat_frame(doc, **frame) for frame in frames])
    return path, frame_log_pb.read_contract(path)


def _span(count, state_id="S_MOVE", index=0):
    return {
        "state_index": index,
        "state_id": state_id,
        "entered_step": 0,
        "exited_step": count - 1,
        "first": 0,
        "last": count - 1,
        "ticks": count,
        "seconds": count * 1e-3,
        "motions": [0],
    }


def _sine(hz, count, period_s=1e-3, amplitude=1e-3):
    return [amplitude * math.sin(2 * math.pi * hz * i * period_s) for i in range(count)]


def test_the_clock_is_the_headers_period_and_never_the_frames_t(tmp_path):
    """Two runs of one model disagreed by 2.9x on `t` per step; the period does not drift."""
    doc = _schema()
    _log_path, contract = _log(tmp_path, doc, [{"step": 0, "t": 0.0, "last_event": -1}])
    assert analysis.period_seconds(contract.header) == 1e-3
    assert analysis.run_seconds(1000, contract.header) == 1.0
    # An archive written before the header carried a period: stated, not silently substituted.
    assert analysis.period_seconds(SimpleNamespace(nominal_period_ns=0)) == 1e-3


def test_a_state_re_entered_is_its_own_span(tmp_path):
    doc = _schema(("S_MOVE", "S_HOLD"))
    log, contract = _log(
        tmp_path,
        doc,
        [
            # `t` runs here at a third of the control clock, as a real run's did.
            {"step": step, "t": step * 0.0003, "fsm_state": state, "last_event": -1}
            for step, state in enumerate([0, 0, 0, 1, 1, 0])
        ],
    )
    spans = analysis.state_spans(log, contract)
    assert [(span["state_id"], span["ticks"]) for span in spans] == [
        ("S_MOVE", 3),
        ("S_HOLD", 2),
        ("S_MOVE", 1),
    ]
    assert [round(span["seconds"], 4) for span in spans] == [0.003, 0.002, 0.001]


def test_a_signal_the_run_never_recorded_is_omitted(tmp_path):
    """Omitted, not filled with None: a caller must tell "not recorded" from "recorded zero"."""
    doc = _schema()
    log, contract = _log(tmp_path, doc, [{"step": 0, "q0": 0.5, "last_event": -1}])
    assert analysis.signal_series(log, contract, ["dist", "never_logged"]) == {"dist": [0.5]}


def test_detrending_leaves_nothing_of_a_ramp():
    residual = analysis.detrend([0.01 * i for i in range(1000)])
    assert max(abs(value) for value in residual[200:-200]) < 1e-9


def test_a_synthesised_sine_is_found_at_its_own_frequency():
    hz, amplitude = analysis.dominant_frequency(_sine(2.0, 4000, amplitude=0.5), 1e-3)
    assert hz == 2.0
    assert abs(amplitude - 0.5) / 0.5 < 0.05


def test_a_mode_is_what_several_joints_agree_on():
    joints = {"arm": {"qd": {str(i): f"qd_{i}" for i in range(7)}}}
    series = {f"qd_{i}": _sine(2.0 if i < 6 else 5.0, 10000) for i in range(7)}
    (row,) = analysis.coherent_modes([_span(10000)], series, 1e-3, joints)
    assert (row["mode_hz"], row["agreeing"], row["joints"]) == (2.0, 6, 7)

    scattered = {f"qd_{i}": _sine(2.0 + 0.5 * i, 10000) for i in range(7)}
    (row,) = analysis.coherent_modes([_span(10000)], scattered, 1e-3, joints)
    assert row["mode_hz"] is None


def test_joints_are_partitioned_by_the_solver_that_drives_them():
    """Two arms have no mechanical reason to agree; pooling them would invent a mode."""
    contract = SimpleNamespace(
        quantity_ids=[
            "arm1_solver_hold_qd_joint_1",
            "arm2_solver_hold_qd_joint_1",
            "arm1_solver_hold_tau_cmd_joint_1",
            "arm1_solver_hold_qdd_joint_1",
            "arm1_solver_hold_q_joint_1",
        ]
    )
    assert analysis.joint_signals(contract) == {
        "arm1_solver_hold": {
            "qd": {"1": "arm1_solver_hold_qd_joint_1"},
            "tau_cmd": {"1": "arm1_solver_hold_tau_cmd_joint_1"},
        },
        "arm2_solver_hold": {"qd": {"1": "arm2_solver_hold_qd_joint_1"}},
    }


def test_a_clipped_torque_is_counted_where_it_was_clipped():
    joints = {"arm": {"tau_ctrl": {"2": "want"}, "tau_cmd": {"2": "have"}}}
    series = {"want": [-45.0] * 6 + [-10.0] * 20, "have": [-39.0] * 6 + [-10.0] * 20}
    (row,) = analysis.saturation_report(
        [_span(26, "S_LIFT")], series, joints, [("arm_torque_limit", 39.0)]
    )
    assert (row["ticks"], row["limit_nm"], row["states"]) == (6, 39.0, ["S_LIFT"])
    assert (row["requested_nm"], row["limit_id"]) == (45.0, "arm_torque_limit")


def test_a_model_without_joint_torques_reports_no_saturation():
    joints = {"arm": {"qd": {"1": "qd_1"}}}
    assert analysis.saturation_report([_span(10)], {"qd_1": [0.0] * 10}, joints) == []


def test_a_contact_is_the_wrenchs_rise_and_what_the_body_did_after_it():
    quiet, hit, after = 200, 1, 800
    forces = {"x": "fx", "y": "fy", "z": "fz"}
    velocities = {"x": "vx", "y": "vy", "z": "vz"}
    series = {
        "fx": [0.0] * (quiet + hit + after),
        "fy": [0.0] * (quiet + hit + after),
        "fz": [1.0] * quiet + [50.0] * hit + [8.0] * after,
        "vx": [0.0] * (quiet + hit + after),
        "vy": [0.0] * (quiet + hit + after),
        # driven down, thrown back, then settling inside the band with a small ripple
        "vz": [-0.05] * quiet + [0.12] * hit + [0.005 * (-1) ** i for i in range(after)],
    }
    bands = {0: {"vz": {"band": 0.02, "gate": "settled", "motion": "move"}}}
    (row,) = analysis.contact_events(
        [_span(quiet + hit + after)], series, 1e-3, forces, velocities, bands
    )
    assert (row["axis"], round(row["peak_n"], 1), row["rebound"]) == ("z", 50.0, 0.12)
    assert (row["impact_step"], row["band"], row["gate"]) == (quiet, 0.02, "settled")
    assert round(row["settle_s"], 3) == 0.001 and round(row["ripple"], 3) == 0.01

    # A wrench that never rises is not a contact, whatever level it sits at.
    level = dict(series, fz=[40.0] * (quiet + hit + after))
    assert (
        analysis.contact_events(
            [_span(quiet + hit + after)], level, 1e-3, forces, velocities, bands
        )
        == []
    )
