# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""A saturation bound to the wrong signal silently does nothing."""

from __future__ import annotations

from pathlib import Path

from support import EXAMPLES

from motion_spec.generation.pipeline import load_model
from motion_spec.rdf_parser.ir import generate_ir

ARC = EXAMPLES["arc_tracing_with_admittance"] / "arc_tracing_with_admittance.robmot"


def test_an_output_saturation_limits_the_controllers_own_control_signal(tmp_path: Path) -> None:
    loaded = load_model(ARC, tmp_path / "generated" / "model")
    ir = generate_ir(loaded.model, loaded.fsm)
    saturated = [
        controller
        for motion in ir["coordination"]["motions"]
        for controller in motion.controllers
        if controller.output_saturation is not None
    ]
    assert saturated
    for controller in saturated:
        assert controller.output_saturation.input_signal is controller.control_signal, controller.id
