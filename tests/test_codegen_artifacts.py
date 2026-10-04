# SPDX-License-Identifier: MPL-2.0

from __future__ import annotations

from datetime import UTC, datetime
from pathlib import Path

import pytest
from rdf_utils.constraints import ConstraintViolation
from rdf_utils.resolver import IriToFileResolver, install_resolver
from rdflib import Dataset, Graph, URIRef
from rdflib.namespace import PROV, RDF

from motion_spec.classes.geometry import Axis, Subspace, View
from motion_spec.classes.motion import ComponentRef, PoseComponents
from motion_spec.classes.qudt import Quantity, QuantityKind, Unit
from motion_spec.rdf_parser import quantities
from motion_spec.rdf_parser.model import Model
from motion_spec.runs import provenance


COMPONENT = Quantity("home_pose_position_z", QuantityKind("Length"), Unit("M"), None, False)

# One pose's z read off itself, and the same quantity bound into another pose's z.
COMPONENT_VIEWS = {
    view_id: View(
        view_id,
        Quantity(pose_id, QuantityKind("Length"), Unit("M"), None, False),
        COMPONENT,
        Subspace.Linear,
        Axis.Z,
    )
    for view_id, pose_id in (("read", "home_pose"), ("bind", "target_pose"))
}


def test_component_bound_into_a_pose_is_not_a_reading_of_it() -> None:
    """The binding must not compete with the source pose's own view (both are axis z of a
    pose, so the ambiguity rule would otherwise drop the reading and leave a zero)."""
    components = {
        "target_pose": PoseComponents(
            "quaternion", position_z=ComponentRef(ref="home_pose_position_z")
        )
    }

    index = quantities.views_for_access(COMPONENT_VIEWS, [], [], {}, components)

    assert index["home_pose_position_z"].superobject.id == "home_pose"


def test_a_quantity_no_view_agrees_on_is_rejected() -> None:
    """Two readings that disagree and no direct write: nothing can compute it, and rendering
    it as its own shared field would compile to a zero."""
    with pytest.raises(ConstraintViolation, match="home_pose_position_z"):
        quantities.views_for_access(COMPONENT_VIEWS, [], [], {}, {})


def test_the_generation_document_conforms_to_the_prov_shapes(tmp_path: Path) -> None:
    metamodels = Path(__file__).resolve().parents[2] / "metamodels"
    if not metamodels.exists():
        pytest.skip("metamodels is not in this checkout")
    pyshacl = __import__("pyshacl")
    generated = tmp_path / "generated"
    controller = generated / "controller"
    (controller / "headers").mkdir(parents=True)
    written = [controller / "CMakeLists.txt", controller / "main.cpp", controller / "headers/r.hpp"]
    for path in written:
        path.write_text("// generated\n")
    ir_path = generated / "model" / "ir.json"
    ir_path.parent.mkdir(parents=True)
    ir_path.write_text("{}")
    executable = tmp_path / "build" / "main"
    executable.parent.mkdir()
    executable.write_text("binary")
    now = datetime.now(UTC)
    graph = Graph()
    provenance.record_code_generation(graph, generated, ir_path, written, started=now, ended=now)
    provenance.write_generation_document(
        generated / provenance.GENERATION_DOCUMENT,
        {
            provenance.GRAPH_DSL: Graph(),
            provenance.GRAPH_COORD_DSL: Graph(),
            provenance.GRAPH_MOTION_SPEC: graph,
        },
    )
    provenance.record_build(generated, controller, executable, started=now, ended=now)
    install_resolver(
        IriToFileResolver(
            {"https://secorolab.github.io/metamodels/": str(metamodels)}, download=False
        )
    )
    shapes = Graph()
    for name in ("prov.shacl.ttl", "prov-extension.shacl.ttl"):
        shapes.parse(str(metamodels / name), format="turtle")
    conforms, _, report = pyshacl.validate(
        str(generated / provenance.GENERATION_DOCUMENT),
        shacl_graph=shapes,
        data_graph_format="json-ld",
        inference="rdfs",
    )
    assert conforms, report


def test_a_derived_id_has_one_iri_and_never_shadows_an_authored_one() -> None:
    authored = "https://example.org/m/authored/ctrl-x"
    graph = Dataset(default_union=True)
    graph.add((URIRef(authored), RDF.type, PROV.Entity))
    model = Model(
        graph=graph,
        app_path=Path("model-app.ld.json"),
        namespaces=("https://example.org/m/authored/",),
    )

    assert (
        model.register_derived("ctrl_x", "https://example.org/m/other", "ctrl-x", PROV.wasDerivedFrom)
        == authored
    )
    assert model.derivation_nodes() == []
    model.register_derived("err_x", "https://example.org/m/a", "err", PROV.wasDerivedFrom)
    with pytest.raises(ConstraintViolation, match="minted for two derived entities"):
        model.register_derived("err_x", "https://example.org/m/b", "err", PROV.wasDerivedFrom)
