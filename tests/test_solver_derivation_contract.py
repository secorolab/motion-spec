# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""Contract for the ACHD/RNE solver structure and ownership emitted by the DSL."""

from __future__ import annotations

from pathlib import Path

import pytest
from motion_spec_dsl.gens import _gen_graph
from motion_spec_dsl.langs import motion_spec_metamodel
from motion_spec_dsl.rdf_parser.vocab import (
    ALGO_EXT,
    CSTR_HDL,
    CSTR_HDL_EXT,
    QUDT_QKIND,
    QUDT_SCHEMA,
    SLV,
    SLV_EXT,
)
from rdflib import Dataset, Graph
from rdflib.namespace import PROV, RDF

from motion_spec.rdf_parser.ir import generate_ir

MODELS = Path(__file__).parents[2] / "motion-spec-dsl" / "models"
METAMODELS = Path(__file__).resolve().parents[2] / "metamodels"

from conftest import requires_workspace

pytestmark = requires_workspace(MODELS, METAMODELS)


@pytest.fixture(scope="module")
def pick_place_single_jsonld(tmp_path_factory: pytest.TempPathFactory) -> Path:
    tmp_path = tmp_path_factory.mktemp("pick_place_single")
    with pytest.MonkeyPatch.context() as mp:
        mp.setenv("METAMODELS_PATH", str(METAMODELS))
        metamodel = motion_spec_metamodel()
        model = metamodel.model_from_file(MODELS / "pick_place_single" / "pick_place_single.robmot")
        _gen_graph(metamodel, model, tmp_path, overwrite=True, debug=False)
    return tmp_path / "pick_place_single.ld.json"


@pytest.fixture
def constraint_graph(pick_place_single_jsonld: Path) -> Graph:
    """Fresh graph per test: parsed from the immutable JSON-LD so no test can leak
    triple mutations into another."""
    dataset = Dataset()
    dataset.parse(str(pick_place_single_jsonld), format="json-ld")
    graph = Graph()
    for quad in dataset.quads((None, None, None, None)):
        graph.add(quad[:3])
    return graph


def test_controllers_reference_their_solver(constraint_graph: Graph) -> None:
    handlers = set(constraint_graph.subjects(RDF.type, CSTR_HDL.ConstraintHandler))
    solvers = set(constraint_graph.subjects(RDF.type, SLV.SolverWithInputAndOutput)) | set(
        constraint_graph.subjects(RDF.type, SLV_EXT.CommandForwardingSolver)
    )
    controllers = {
        controller
        for handler in handlers
        for controller in constraint_graph.objects(handler, CSTR_HDL.controllers)
    }
    assert controllers
    assert all(
        len(set(constraint_graph.objects(controller, CSTR_HDL_EXT.solver))) == 1
        for controller in controllers
    )

    # Derivation provenance belongs to the model's own entities -- a spec pose taken from a
    # scene object carries it -- never to the solver wiring this test is about.
    derived = set(constraint_graph.subjects(PROV.wasDerivedFrom, None))
    assert not derived & (controllers | solvers | handlers)
    assert {
        constraint_graph.value(controller, CSTR_HDL_EXT.solver) for controller in controllers
    } <= solvers

    declared_limit_kinds = {
        kind
        for solver in solvers
        for limit in constraint_graph.objects(solver, ALGO_EXT.limits)
        if (signal := constraint_graph.value(limit, ALGO_EXT["in"])) is not None
        for kind in constraint_graph.objects(signal, QUDT_SCHEMA.hasQuantityKind)
    }
    assert declared_limit_kinds <= {QUDT_QKIND.Torque}


def test_authored_output_limit_binds_to_derived_signal(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setenv("METAMODELS_PATH", str(METAMODELS))
    source = (MODELS / "pick_place_single" / "pick_place_single.robmot").read_text()
    source = source.replace(
        "linear-velocity zero-linvel = 0.0 m/s,",
        "linear-velocity zero-linvel = 0.0 m/s,\n"
        # The signal saturated is the solver's acceleration-energy row, not a velocity.
        "        linear-acceleration output-limit = 10.0 m/s^2,",
    ).replace(
        "constraint: <pick.hold-x>,     Kp:",
        "constraint: <pick.hold-x>, output-saturation: saturation { "
        "max: <shared.spec.output-limit> }, Kp:",
    )
    metamodel = motion_spec_metamodel()
    model = metamodel.model_from_str(
        source, file_name=str(MODELS / "pick_place_single" / "pick_place_single.robmot")
    )
    _gen_graph(metamodel, model, tmp_path, overwrite=True, debug=False)

    graph = Graph().parse(tmp_path / "pick_place_single.ld.json", format="json-ld")
    controller = next(
        node
        for node in graph.subjects(RDF.type, CSTR_HDL.Controller)
        if str(node).endswith("/ctrl-pk-hold-x")
    )
    saturation = next(graph.objects(controller, ALGO_EXT.limits))
    authored_output = graph.value(saturation, ALGO_EXT["in"])
    assert QUDT_QKIND.LinearVelocity in graph[authored_output : QUDT_SCHEMA.hasQuantityKind]

    ir = generate_ir(tmp_path / "pick_place_single-app.ld.json")
    shared_ids = {item.id for item in ir["computation"]["shared_data"]}
    # Handlers are no longer published: a motion carries the controllers derived for it.
    controllers = [
        controller for motion in ir["coordination"]["motions"] for controller in motion.controllers
    ]
    acceleration_constraints = [
        constraint
        for solver in ir["resources"]["by_kind"]["serial_chain"]
        for driver in solver.motion_drivers
        for constraint in driver.acceleration_constraint
    ]
    assert {controller.control_signal.id for controller in controllers} <= shared_ids
    assert {
        constraint.acceleration_energy.id for constraint in acceleration_constraints
    } <= shared_ids
    derived = next(controller for controller in controllers if controller.id == "ctrl_pk_hold_x")
    assert derived.output_saturation.maximum.value == 10.0
    assert derived.output_saturation.input_signal is derived.control_signal
