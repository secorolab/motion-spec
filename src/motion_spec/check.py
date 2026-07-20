# SPDX-License-Identifier: MPL-2.0
"""SHACL validation tool for motion specification models."""

import sys
import argparse
from pathlib import Path

import pyshacl
import rdflib
from rdf_utils.resolver import IriToFileResolver, install_resolver

from motion_spec.manifest import build_url_map, metamodel_url_map
from motion_spec.namespace import APP


def validate_manifest(app_model: str | Path, *, meta_shacl: bool = False) -> tuple[bool, str]:
    """Validate one application manifest and return conformance plus the SHACL report."""
    app_model_path = Path(app_model).resolve()

    # Load top-level, application model
    g = rdflib.Dataset()
    g.parse(app_model, format="json-ld")

    # Metamodel/ontology prefixes resolve through the shared local checkout; the
    # model's own iri-map only declares where its imported graphs live. Merge both
    # (longest prefix first) so neither metamodels nor imports reach the network.
    url_map = {**metamodel_url_map(), **build_url_map(g, app_model_path)}
    install_resolver(
        IriToFileResolver(
            dict(sorted(url_map.items(), key=lambda x: len(x[0]), reverse=True)),
            download=False,
        )
    )

    # Load/import the referenced models
    models = list({o for _, _, o, _ in g.quads((None, APP["import"], None, None))})
    for o in models:
        g.parse(location=o, format="json-ld")

    g_sh = rdflib.Dataset()
    metamodels = sorted(
        str(o) for _, _, o, _ in g.quads((None, APP["constraints"], None, None))
    )
    if not metamodels:
        return (
            False,
            "Validation Report\nConforms: False\n"
            "No SHACL constraint files were listed in the application manifest.",
        )
    for location in metamodels:
        try:
            g_sh.parse(location=location, format="turtle")
        except Exception as exc:
            return (
                False,
                "Validation Report\nConforms: False\n"
                f"Failed to load SHACL constraint graph {location}: {exc}",
            )

    # Validate using Dataset directly
    conforms, _v_graph, v_text = pyshacl.validate(
        data_graph=g, shacl_graph=g_sh, inference="none", meta_shacl=meta_shacl
    )
    return bool(conforms), v_text


def main(argv: list[str] | None = None) -> int:
    """Compatibility entry point; the installed CLI is ``motion-spec check``."""
    parser = argparse.ArgumentParser(prog="motion-spec check")
    parser.add_argument("manifest")
    parser.add_argument("--meta-shacl", action="store_true")
    args = parser.parse_args(argv)

    conforms, report = validate_manifest(args.manifest, meta_shacl=args.meta_shacl)
    print(report)
    return 0 if conforms else 1


if __name__ == "__main__":
    sys.exit(main())
