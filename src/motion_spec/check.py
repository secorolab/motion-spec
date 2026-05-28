# SPDX-License-Identifier: MPL-2.0
"""SHACL validation tool for motion specification models."""

import sys
import argparse
from pathlib import Path

import pyshacl
import rdflib
from rdf_utils.resolver import IriToFileResolver, install_resolver

from rdflib.namespace import RDF

from motion_spec.namespace import APP, CSTR_HDL

PACKAGE_ROOT = Path(__file__).resolve().parents[2]
SUPPORTED_CONTROL_MODES = {"JointTorque"}


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

    args = parser.parse_args()

    app_model = args.manifest
    app_model_path = Path(app_model).resolve()

    # Load top-level, application model
    g = rdflib.Dataset()
    g.parse(app_model, format="json-ld")

    # Load IRI map and resolve paths relative to the manifest file
    def _quad_objects(predicate):
        """Return distinct objects for a predicate across all named graphs."""
        return list({o for _, _, o, _ in g.quads((None, predicate, None, None))})

    def _quad_value(subject, predicate):
        """Return first object for subject+predicate across all named graphs."""
        return next((o for _, _, o, _ in g.quads((subject, predicate, None, None))), None)

    url_map = {}
    for key in _quad_objects(APP["iri-map"]):
        path_node = _quad_value(key, APP["path"])
        if path_node is None:
            continue
        value = str(path_node)
        if Path(value).is_absolute():
            # Already absolute path, use as-is
            url_map[str(key)] = value
        else:
            # For existing models, the IRI mapping "models/" should resolve to the manifest directory itself
            # For new DSL models, "models/" should resolve to a models/ subdirectory
            if value == "models/":
                # Try manifest directory first (for existing models)
                manifest_dir_path = app_model_path.parent
                # Try models subdirectory (for new DSL models)
                models_subdir_path = app_model_path.parent / "models"

                # Check which one contains the expected files by looking at imports
                imports = _quad_objects(APP["import"])
                if imports:
                    # Take first import URL and extract the path part after the base URL
                    first_import_url = str(imports[0])
                    # The import URLs are like "https://secorolab.github.io/00-common/00-misc.json"
                    # We want to extract "00-common/00-misc.json"
                    import_path = None
                    for base_url in [str(k) for k in _quad_objects(APP["iri-map"])]:
                        if first_import_url.startswith(base_url):
                            import_path = first_import_url[len(base_url) :]
                            break

                    if import_path:
                        if (manifest_dir_path / import_path).exists():
                            url_map[str(key)] = str(manifest_dir_path)
                        elif (models_subdir_path / import_path).exists():
                            url_map[str(key)] = str(models_subdir_path)
                        else:
                            # Fallback to current working directory
                            url_map[str(key)] = str(Path.cwd() / value)
                    else:
                        # Couldn't extract path, use models subdirectory by default
                        url_map[str(key)] = str(models_subdir_path)
                else:
                    # No imports to check, use models subdirectory by default
                    url_map[str(key)] = str(models_subdir_path)
            else:
                # For non-models paths, resolve normally relative to manifest
                absolute_path = app_model_path.parent / value
                if not absolute_path.exists():
                    source_path = PACKAGE_ROOT / value
                    absolute_path = source_path if source_path.exists() else Path.cwd() / value
                url_map[str(key)] = str(absolute_path)

    install_resolver(IriToFileResolver(dict(sorted(url_map.items(), key=lambda x: len(x[0]), reverse=True))))

    # Load/import the referenced models
    models = _quad_objects(APP["import"])
    for o in models:
        g.parse(location=o, format="json-ld")

    validation_errors = []
    for handler in {s for s, _, _, _ in g.quads((None, RDF.type, CSTR_HDL["ConstraintHandler"], None))}:
        control_mode = _quad_value(handler, CSTR_HDL["control-mode"])
        if control_mode is None:
            validation_errors.append(f"Constraint handler '{handler}' is missing control-mode.")
            continue
        mode_name = str(control_mode).rsplit("#", 1)[-1].rsplit("/", 1)[-1]
        if mode_name not in SUPPORTED_CONTROL_MODES:
            validation_errors.append(
                f"Constraint handler '{handler}' uses unsupported control mode '{mode_name}'."
            )
    if validation_errors:
        print("Validation Report")
        print("Conforms: False")
        for error in validation_errors:
            print(error)
        sys.exit(1)

    g_sh = rdflib.Dataset()
    metamodels = sorted(str(o) for o in _quad_objects(APP["constraints"]))
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
        data_graph=g, shacl_graph=g_sh, inference="none", meta_shacl=True
    )

    print(v_text)
    sys.exit(0 if conforms else 1)


if __name__ == "__main__":
    main()
