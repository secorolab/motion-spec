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


def main():
    """Validate motion specification models against SHACL constraints."""
    parser = argparse.ArgumentParser(
        description="Validate motion specification models against SHACL constraints",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s manifest.json                    # Validate model and print result
        """,
    )

    parser.add_argument("manifest", help="Path to the application manifest JSON file")
    parser.add_argument(
        "--meta-shacl",
        action="store_true",
        help="Also validate the SHACL shape graph against SHACL-of-SHACL "
        "(~4x slower; only useful when editing the metamodel shapes themselves).",
    )

    args = parser.parse_args()

    app_model = args.manifest
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
        print("Validation Report")
        print("Conforms: False")
        print("No SHACL constraint files were listed in the application manifest.")
        sys.exit(1)
    for location in metamodels:
        try:
            g_sh.parse(location=location, format="turtle")
        except Exception as exc:
            print("Validation Report")
            print("Conforms: False")
            print(f"Failed to load SHACL constraint graph {location}: {exc}")
            sys.exit(1)

    # Validate using Dataset directly
    conforms, v_graph, v_text = pyshacl.validate(
        data_graph=g, shacl_graph=g_sh, inference="none", meta_shacl=args.meta_shacl
    )

    print(v_text)
    sys.exit(0 if conforms else 1)


if __name__ == "__main__":
    main()
