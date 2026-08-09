# SPDX-License-Identifier: MPL-2.0
"""Two generations of one model must lower to the same IR.

The merged app graph is serialized in rdflib's order, so its bytes differ from run to run;
every list ir_gen publishes has to be ordered by something else, or the event indices and
struct members baked into the generated C++ change between two builds of the same model.
"""

from __future__ import annotations

import shutil
from pathlib import Path

from motion_spec.generation.pipeline import generate_model

MODEL = Path(__file__).parents[2] / "motion-spec-dsl" / "models" / "pick_place_single"

from conftest import requires_workspace

pytestmark = requires_workspace(MODEL)


def test_two_generations_of_one_model_lower_to_the_same_ir(tmp_path):
    # Same generation dir both times: ir.json records the manifest's absolute path, which is
    # the run's identity rather than the model's.
    generation = tmp_path / "generation"
    irs = []
    for _ in range(2):
        shutil.rmtree(generation, ignore_errors=True)
        generation.mkdir()
        generated = generate_model(MODEL / "pick_place_single.robmot", generation, stage="ir")
        irs.append((generated / "model" / "ir.json").read_text())
    assert irs[0] == irs[1]
