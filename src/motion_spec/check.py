# SPDX-License-Identifier: MPL-2.0
"""SHACL validation tool for motion specification models."""

import sys
import argparse
from pathlib import Path

import pyshacl
import rdflib
from rdf_utils.resolver import IriToFileResolver, install_resolver

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

    args = parser.parse_args()

    app_model = args.manifest
    app_model_path = Path(app_model).resolve()

    # Load top-level, application model
    g = rdflib.Dataset()
    g.parse(app_model, format="json-ld")

    # Load IRI map and resolve paths relative to the manifest file
    url_map = {}
    for key in g.objects(predicate=APP["iri-map"]):
        value = g.value(key, APP["path"]).value
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
                imports = list(g.objects(predicate=APP["import"]))
                if imports:
                    # Take first import URL and extract the path part after the base URL
                    first_import_url = str(imports[0])
                    # The import URLs are like "https://secorolab.github.io/00-common/00-misc.json"
                    # We want to extract "00-common/00-misc.json"
                    import_path = None
                    for base_url in [str(k) for k in g.objects(predicate=APP["iri-map"])]:
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
                    absolute_path = Path.cwd() / value
                url_map[str(key)] = str(absolute_path)

    install_resolver(IriToFileResolver(url_map))

    # Load/import the referenced models
    models = list(g.objects(predicate=APP["import"]))
    for o in models:
        g.parse(location=o, format="json-ld")

    g_sh = rdflib.Dataset()
    metamodels = list(g.objects(predicate=APP["constraints"]))
    for o in metamodels:
        g_sh.parse(location=str(o), format="turtle")

    # Validate using Dataset directly
    conforms, v_graph, v_text = pyshacl.validate(
        data_graph=g, shacl_graph=g_sh, inference="none", meta_shacl=True
    )

    print(v_text)
    sys.exit(0 if conforms else 1)


if __name__ == "__main__":
    main()
