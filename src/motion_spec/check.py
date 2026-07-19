# SPDX-License-Identifier: MPL-2.0
"""SHACL validation tool for motion specification models."""

import sys
import argparse
from pathlib import Path

import pyshacl
import rdflib
from rdf_utils.resolver import IriToFileResolver, install_resolver

from rdflib.namespace import RDF

from motion_spec.manifest import build_url_map, metamodel_url_map
from motion_spec.namespace import APP, CSTR_HDL, CSTR_HDL_EXT, KC_STAT

SUPPORTED_CONTROL_MODES = {KC_STAT["JointForce"]}


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

    validation_errors = []
    for handler in {s for s, _, _, _ in g.quads((None, RDF.type, CSTR_HDL["ConstraintHandler"], None))}:
        control_mode = next(
            (o for _, _, o, _ in g.quads((handler, CSTR_HDL_EXT["control-mode"], None, None))),
            None,
        )
        if control_mode is None:
            validation_errors.append(f"Constraint handler '{handler}' is missing control-mode.")
            continue
        if control_mode not in SUPPORTED_CONTROL_MODES:
            validation_errors.append(
                f"Constraint handler '{handler}' uses unsupported control mode '{control_mode}'."
            )
    if validation_errors:
        print("Validation Report")
        print("Conforms: False")
        for error in validation_errors:
            print(error)
        sys.exit(1)

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
