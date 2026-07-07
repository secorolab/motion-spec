# SPDX-License-Identifier: MPL-2.0
"""Shared IRI-map resolution for application manifests."""

from pathlib import Path

from motion_spec.namespace import APP

PACKAGE_ROOT = Path(__file__).resolve().parents[2]
METAMODELS_URL = "https://secorolab.github.io/metamodels/"
COMP_ROB2B_URL = "https://comp-rob2b.github.io/metamodels/"


def metamodels_root() -> Path | None:
    """Locate the local secorolab metamodels checkout (honours METAMODELS_PATH)."""
    import os

    roots = []
    env_path = os.environ.get("METAMODELS_PATH")
    if env_path:
        roots.append(Path(env_path))
    for start in (PACKAGE_ROOT, Path.cwd()):
        roots.extend([start, *start.parents])
    for root in roots:
        for candidate in (root / "src" / "metamodels", root / "metamodels"):
            if (candidate / "prov.json").exists():
                return candidate
    return None


def metamodel_url_map():
    """Map the metamodel IRI prefixes to their local checkouts for the resolver.

    This is the single source of truth for offline metamodel/ontology resolution;
    callers merge it with a model's own ``build_url_map`` before parsing so that
    neither the metamodel nor the model imports ever hit the network.
    """
    root = metamodels_root()
    if root is None:
        return {}
    url_map = {METAMODELS_URL: str(root)}
    comp_rob2b = root.parent / "comp-rob2b" / "metamodels"
    if comp_rob2b.exists():
        url_map[COMP_ROB2B_URL] = str(comp_rob2b)
    return url_map


def build_url_map(g, manifest_path):
    """Build the IRI->local-path map declared by a manifest's iri-map entries.

    Uses the quad API so it works for both union and non-union rdflib Datasets.
    """
    app_model_path = Path(manifest_path).resolve()

    def _quad_objects(predicate):
        return list({o for _, _, o, _ in g.quads((None, predicate, None, None))})

    def _quad_value(subject, predicate):
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

    return url_map
