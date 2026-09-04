# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""Read back the numbers the DSL drew for a randomized model.

The draw happens inside `textx generate` (the `--seed` argument on the `jsonld` target), because
the DSL resolves poses numerically in that same pass. By the time the documents land here every
`distrib:SampledQuantity` already carries plain values; this module only reports them so the
generation's provenance can record what was drawn.
"""

from __future__ import annotations

from pathlib import Path

import rdflib
from motion_spec_dsl.rdf_parser.manifest import install_metamodel_resolver
from motion_spec_dsl.rdf_parser.vocab import GEOM_COORD, QUDT_SCHEMA
from rdf_utils.constraints import ConstraintViolation
from rdf_utils.models.vocab import URI_DISTRIB_PRED_FROM_DISTRIB, URI_DISTRIB_TYPE_SAMPLED_QUANTITY
from rdflib.namespace import RDF


def sampled_draws(model_dir: Path) -> dict[str, dict]:
    """The value the DSL drew for every sampled quantity the generated model documents declare.

    The distribution travels with the numbers: which one a quantity drew from is what a later
    analysis groups runs by.

    Parameters:
        model_dir: the generation's `generated/model` directory

    Returns:
        ``{node_uri: {"values": [...], "distribution": uri}}``, or ``{}`` when the model
        declares no sampled quantity
    """
    install_metamodel_resolver()
    graph = rdflib.Graph()
    for path in sorted(Path(model_dir).glob("*.ld.json")):
        graph.parse(str(path), format="json-ld")

    draws: dict[str, dict] = {}
    for node in sorted(graph.subjects(RDF.type, URI_DISTRIB_TYPE_SAMPLED_QUANTITY), key=str):
        distribution = graph.value(node, URI_DISTRIB_PRED_FROM_DISTRIB)
        draws[str(node)] = {
            "values": _values(graph, node),
            "distribution": str(distribution) if distribution is not None else None,
        }
    return draws


def _values(graph, node) -> list[float]:
    """The drawn numbers of one sampled quantity: a scalar value, or an xyz position."""
    scalar = graph.value(node, QUDT_SCHEMA.value)
    if scalar is not None:
        return [float(scalar)]
    position = [graph.value(node, GEOM_COORD[axis]) for axis in "xyz"]
    if all(value is not None for value in position):
        return [float(value) for value in position]
    raise ConstraintViolation(
        "sampling",
        f"sampled quantity '{node}' carries neither a value nor xyz coordinates, so the DSL drew "
        "nothing for it",
    )


def demo() -> None:
    """A scalar and an xyz draw read back in order; a node carrying neither is rejected."""
    import json
    import tempfile

    base = "http://example.org/demo/"

    def sampled(name, keys):
        return {"@id": f"{base}{name}", "@type": [str(URI_DISTRIB_TYPE_SAMPLED_QUANTITY)], **keys}

    with tempfile.TemporaryDirectory() as directory:
        model_dir = Path(directory)
        (model_dir / "a.ld.json").write_text(
            json.dumps(
                {"@context": {"ex": base}, "@graph": [sampled("t", {str(QUDT_SCHEMA.value): 1.5})]}
            )
        )
        (model_dir / "b.ld.json").write_text(
            json.dumps(
                {
                    "@context": {"ex": base},
                    "@graph": [
                        sampled(
                            "p",
                            {
                                str(URI_DISTRIB_PRED_FROM_DISTRIB): {"@id": f"{base}p-distrib"},
                                **{
                                    str(GEOM_COORD[axis]): v
                                    for axis, v in zip("xyz", [0.25, -1.0, 4.0])
                                },
                            },
                        )
                    ],
                }
            )
        )

        draws = sampled_draws(model_dir)
        assert draws == {
            f"{base}p": {"values": [0.25, -1.0, 4.0], "distribution": f"{base}p-distrib"},
            f"{base}t": {"values": [1.5], "distribution": None},
        }, draws
        assert list(draws) == [f"{base}p", f"{base}t"], draws

        (model_dir / "c.ld.json").write_text(
            json.dumps({"@context": {"ex": base}, "@graph": [sampled("undrawn", {})]})
        )
        try:
            sampled_draws(model_dir)
        except ConstraintViolation as exc:
            assert "undrawn" in str(exc), exc
        else:
            raise AssertionError("a sampled quantity without values must be rejected")

    with tempfile.TemporaryDirectory() as empty:
        assert sampled_draws(Path(empty)) == {}
    print("sampling demo ok")


if __name__ == "__main__":
    demo()
