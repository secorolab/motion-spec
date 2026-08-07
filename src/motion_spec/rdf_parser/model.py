# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""The loaded model: the graph every other module reads through.

In order: the naming rules, the QUDT-to-SI conversions, dataset loading, ``Model``, and the
reader cache.

``Model`` carries the four things every reader shares -- the merged graph, the one id-minting
rule, the registry of IRIs for entities the model implies but does not author, and the cache the
readers memoize on. Nothing here knows about any concern; every concern reads through it.
"""

from __future__ import annotations

import math
import re
from dataclasses import dataclass, field
from functools import wraps
from pathlib import Path
from typing import NamedTuple
from urllib.parse import urlsplit

import rdflib
from motion_spec_dsl.rdf_parser.manifest import build_url_map, metamodel_url_map
from motion_spec_dsl.rdf_parser.vocab import APP
from rdf_utils.constraints import ConstraintViolation
from rdf_utils.models.common import get_node_types
from rdf_utils.models.vocab import URI_QUDT_UNIT_CM, URI_QUDT_UNIT_M, URI_QUDT_UNIT_MM
from rdf_utils.namespace import NS_MM_QUDT_UNIT
from rdf_utils.resolver import IriToFileResolver, install_resolver
from rdflib import URIRef
from rdflib.namespace import PROV, split_uri

__all__ = [
    "Model",
    "identifier",
    "kebab",
    "length_unit",
    "load_model",
    "local_name",
    "reader",
    "seconds",
    "si",
    "si_all",
    "si_unit",
]


def identifier(name) -> str:
    """A name as a generated identifier: anything but a letter, digit or underscore becomes an
    underscore, and a leading digit is prefixed with one so the result is a legal C++ name.

    Not `rdf_utils.naming.get_valid_var_name`, which *deletes* invalid characters instead of
    replacing them and has no leading-digit rule: every id in the published IR and in the
    generated struct is minted here, so a different sanitizer renames all of them.
    """
    text = re.sub(r"[^0-9A-Za-z_]", "_", str(name))

    return f"_{text}" if text and text[0].isdigit() else text


def kebab(text: str) -> str:
    """An id fragment as an IRI path segment. rdf_utils.naming has no kebab-case rule."""
    return text.replace("_", "-").lower()


def local_name(node) -> str | None:
    """The last segment of a node's URI, or None when there is no node."""
    return None if node is None else split_uri(str(node))[1]


_LENGTH_UNITS = {URI_QUDT_UNIT_M, URI_QUDT_UNIT_CM, URI_QUDT_UNIT_MM}
_TEMPORAL_UNITS = {NS_MM_QUDT_UNIT["SEC"], NS_MM_QUDT_UNIT["MilliSEC"]}
# The DSL records the unit a model was written in and never rescales a value, so putting one on SI
# is the reader's job -- codegen emits metres, radians and seconds. A unit absent here is already
# SI (N, N-M, M-PER-SEC2, KiloGM, UNITLESS, ...).
_SI_EQUIVALENT = {
    NS_MM_QUDT_UNIT["CentiM"]: (NS_MM_QUDT_UNIT["M"], 1e-2),
    NS_MM_QUDT_UNIT["MilliM"]: (NS_MM_QUDT_UNIT["M"], 1e-3),
    NS_MM_QUDT_UNIT["DEG"]: (NS_MM_QUDT_UNIT["RAD"], math.pi / 180.0),
    NS_MM_QUDT_UNIT["CentiM-PER-SEC"]: (NS_MM_QUDT_UNIT["M-PER-SEC"], 1e-2),
    NS_MM_QUDT_UNIT["DEG-PER-SEC"]: (NS_MM_QUDT_UNIT["RAD-PER-SEC"], math.pi / 180.0),
    NS_MM_QUDT_UNIT["DEG-PER-SEC2"]: (NS_MM_QUDT_UNIT["RAD-PER-SEC2"], math.pi / 180.0),
    NS_MM_QUDT_UNIT["MilliSEC"]: (NS_MM_QUDT_UNIT["SEC"], 1e-3),
}


def si_unit(unit):
    """The SI unit `unit` converts to; `unit` itself when it already is SI."""
    return _SI_EQUIVALENT.get(unit, (unit, 1.0))[0]


def length_unit(coordinate):
    """A position coordinate's authored length unit.

    Parameters:
        coordinate: a `PositionCoordModel`, or any model exposing `unit` and `id`

    Returns:
        the QUDT unit the coordinate's values are written in

    Raises:
        ConstraintViolation: the coordinate's unit is not a length.
    """
    if coordinate.unit not in _LENGTH_UNITS:
        raise ConstraintViolation(
            "geometry",
            f"Position coordinate '{coordinate.id}' has an unrecognized length "
            f"unit '{coordinate.unit}'.",
        )

    return coordinate.unit


def si(value: float, unit) -> float:
    """`value`, authored in `unit`, on SI. A unit with no SI equivalent is already SI."""
    return value * _SI_EQUIVALENT.get(unit, (unit, 1.0))[1]


def si_all(values, unit) -> list[float]:
    """Each of `values`, authored in `unit`, on SI."""
    return [si(float(value), unit) for value in values]


def seconds(value: float, unit) -> float:
    """A duration the model authored, in seconds.

    Raises:
        ConstraintViolation: `unit` is not a duration unit, so the value has no reading as one.
    """
    if unit not in _TEMPORAL_UNITS:
        raise ConstraintViolation("units", f"'{unit}' is not a duration this can read")

    return si(value, unit)


# The DSL writes its provenance import under this exact name (motion_spec_dsl.gens), and the
# manifest offers nothing else to tell a provenance import from a model one. A repo-wide path
# convention, ported unchanged from the old loader rather than re-derived.
_PROVENANCE_SUFFIX = "/provenance/dsl.ld.json"


def _import_path(location: str, url_map: dict[str, str]) -> str:
    """An import location as a local file path, through the manifest's url map."""
    for base, root in sorted(url_map.items(), key=lambda item: len(item[0]), reverse=True):
        if location.startswith(base):
            return str((Path(root) / location[len(base) :]).resolve())

    return location


def load_model(manifest_path) -> Model:
    """Load an app manifest and every model it imports into one merged graph.

    Installs the IRI-to-file resolver twice: once with the metamodel map so the manifest itself
    parses, then again with the manifest's own url map, which only exists once it has.

    Parameters:
        manifest_path: path to the `<model>-app.ld.json` the DSL generated

    Returns:
        the `Model` every reader in the package reads through
    """
    app_path = Path(manifest_path).resolve()
    graph = rdflib.Dataset(default_union=True)
    install_resolver(IriToFileResolver(metamodel_url_map(), download=False))
    graph.parse(str(app_path), format="json-ld")

    url_map = build_url_map(graph, app_path)
    install_resolver(IriToFileResolver({**metamodel_url_map(), **url_map}, download=False))

    imported = list(dict.fromkeys(str(model) for model in graph.objects(predicate=APP["import"])))
    provenance = [item for item in imported if item.endswith(_PROVENANCE_SUFFIX)]
    models = [item for item in imported if not item.endswith(_PROVENANCE_SUFFIX)]
    for location in models:
        graph.parse(location=location, format="json-ld")

    return Model(
        graph=graph,
        app_path=app_path,
        imported_models=[_import_path(item, url_map) for item in models],
        imported_provenance=[_import_path(item, url_map) for item in provenance],
    )


class _DerivedIri(NamedTuple):
    """One registry row: the IRI minted for an id, and where it came from."""

    uri: str
    parent: str
    relation: object
    types: list


class _ContextScope(NamedTuple):
    """The three parts of a context quantity's IRI."""

    owner: str
    section: str
    member_path: tuple


class _Index(NamedTuple):
    """The three lookups one walk over the complete graph produces."""

    node_by_id: dict
    id_nodes: list
    authored_iris: dict


@dataclass
class Model:
    """The loaded model: the merged graph, the identity rules every reader shares, the registry of
    derived IRIs, and the cache those readers memoize on.

    The id/IRI index is built on first use rather than at construction, because
    ``operations.normalize`` is still to run and a node it mints has to appear in it.
    """

    graph: rdflib.Dataset
    app_path: Path
    imported_models: list[str]
    imported_provenance: list[str]
    cache: dict = field(default_factory=dict, repr=False)

    def __post_init__(self) -> None:
        self._ambiguous: set[str] | None = None
        self._ids: dict = {}
        self._id_sources: dict[str, set[str]] = {}
        self._index: _Index | None = None
        self._derived: dict[str, _DerivedIri] = {}

    # identity

    def context_scope(self, node) -> _ContextScope | None:
        """The owner, section and member path of a context quantity's IRI, or None when the node
        is not one: a context IRI is ``<app>/<owner>/(spec|world)/<member path>``.
        """
        parts = tuple(part for part in urlsplit(str(node)).path.split("/") if part)
        for index in range(1, len(parts) - 1):
            if parts[index] in ("spec", "world"):
                return _ContextScope(parts[index - 1], parts[index], parts[index + 1 :])

        return None

    def id(self, node) -> str:
        """The stable generated id of a node.

        Its URI's local name, qualified by its owner when that name is shared by more than one
        context quantity -- a context quantity becomes a field on the blackboard, so two of them
        collapsing onto one id would silently merge unrelated values.

        Returns:
            the id, or the node itself when it has no qname to take one from
        """
        cached = self._ids.get(node)
        if cached is not None:
            return cached
        local = self._qname(node)
        if local is None:
            # A literal, a blank node or a URI with no local part has no qname and so no id of
            # its own; it stands for itself wherever a reader passes it on.
            self._ids[node] = node

            return node
        # Only context quantities become shared fields and can merge silently; constraint names,
        # metamodel predicates and aliases legitimately share an id.
        scope = self.context_scope(node)
        if scope:
            owner, _section, member_path = scope
            if local in self._ambiguous_names():
                local = identifier("-".join((owner, *member_path)))
            self._id_sources.setdefault(local, set()).add(str(node))
        self._ids[node] = local

        return local

    def label(self, node) -> str:
        """The node's human-readable local name, or its URI when it has no qname."""
        try:
            return self.graph.compute_qname(node)[2]
        except (TypeError, ValueError):
            return str(node)

    def _qname(self, node) -> str | None:
        """The node's local name as a generated identifier, or None when it has no qname."""
        try:
            return identifier(self.graph.compute_qname(node)[2])
        except (TypeError, ValueError):
            return None

    def motion_suffix(self, motion) -> str:
        """The fragment a motion contributes to the ids derived under it."""
        return self.id(motion).removeprefix("motion_")

    def expect_type(self, node, type_) -> None:
        """Assert a node's rdf:type at the reader that requires it.

        Raises:
            ConstraintViolation: the node does not carry `type_`.
        """
        if type_ not in get_node_types(self.graph, node):
            raise ConstraintViolation(
                "motion-spec", f"Node '{node}' is missing expected rdf:type '{type_}'"
            )

    def _ambiguous_names(self) -> set[str]:
        """Local names more than one context quantity would collapse onto."""
        if self._ambiguous is None:
            sources: dict[str, set[str]] = {}
            for node in set(self.graph.subjects()):
                local = self._qname(node) if self.context_scope(node) is not None else None
                if local is not None:
                    sources.setdefault(local, set()).add(str(node))
            self._ambiguous = {name for name, urls in sources.items() if len(urls) > 1}

        return self._ambiguous

    # the id/IRI index

    @property
    def node_by_id(self) -> dict:
        """The node each generated id names, first occurrence winning."""
        return self._indexes().node_by_id

    @property
    def id_nodes(self) -> list:
        """Every ``(id, node)`` pair in the graph, in node order."""
        return self._indexes().id_nodes

    def _indexes(self) -> _Index:
        node_by_id: dict = {}
        id_nodes: list = []
        if self._index is not None:
            return self._index
        for node in sorted(self.graph.subjects(), key=str):
            id_ = self.id(node)
            id_nodes.append((id_, node))
            node_by_id.setdefault(id_, node)
        authored = {id_: str(node) for id_, node in id_nodes if isinstance(node, URIRef)}
        self._index = _Index(node_by_id, id_nodes, authored)
        self._assert_no_id_collisions()

        return self._index

    def _assert_no_id_collisions(self) -> None:
        """Fail loudly when two distinct context-quantity URIs collapse to one generated id: that
        would silently merge unrelated shared fields, and a genuinely shared quantity has one URI.
        """
        collisions = {i: sorted(u) for i, u in self._id_sources.items() if len(u) > 1}
        if not collisions:
            return
        details = "\n".join(f"  '{i}' <- {', '.join(u)}" for i, u in sorted(collisions.items()))
        raise ValueError(
            "id collision(s): distinct URIs map to one generated id and would be silently "
            "merged (e.g. a context-quantity name reused across motions with different "
            "definitions). Give them distinct names, or extend the motion-qualified id "
            f"scoping in Model.id to cover their URI shape:\n{details}"
        )

    # derived IRIs

    def iri_of(self, id_) -> str | None:
        """The IRI an id resolves to, authored or already derived; None when neither."""
        entry = self._derived.get(id_)

        return entry.uri if entry else self._indexes().authored_iris.get(id_)

    def register_derived(self, id_, parent_iri, suffix, relation, types=()) -> str | None:
        """Mint `<parent_iri>/<suffix>` for an entity the model implies but does not author.

        Every published id has to resolve to something a run graph can make a statement about,
        and an id is a lossy projection of its IRI, so the IRI is recorded here -- at the
        derivation, which is the only place still holding the parent.

        Parameters:
            id_: the generated id the derived entity is published under
            parent_iri: IRI of the node it was derived from
            suffix: the path segment to mint it under, kebab-cased here
            relation: `PROV.specializationOf` when it narrows the parent, else
                `PROV.wasDerivedFrom`
            types: extra RDF types to declare on it in the derivation graph

        Returns:
            the IRI, the authored one when the id already has a node, or None when there is
            nothing to derive from

        Raises:
            RuntimeError: the id was already minted under a different IRI.
        """
        if not id_ or not parent_iri:
            return None
        # An authored node always wins: a derived IRI must never shadow a model's own.
        authored = self._indexes().authored_iris.get(id_)
        if authored is not None:
            return authored
        uri = str(self.child_node(parent_iri, kebab(suffix)))
        existing = self._derived.get(id_)
        if existing is not None:
            if existing.uri != uri:
                raise RuntimeError(
                    f"derived IRI collision: '{id_}' minted as both {existing.uri} and {uri}"
                )

            return uri
        self._derived[id_] = _DerivedIri(uri, parent_iri, relation, list(types))

        return uri

    def derived_node(self, parent, suffix: str) -> URIRef:
        """The IRI of a node ``operations.normalize`` materializes under `parent`.

        The other half of the derived-IRI rule: `register_derived` names an entity that has no
        node, this names one that will have triples of its own, so it needs no registration --
        it becomes a graph subject and so an authored IRI in `uri_rows`. The pattern is
        behaviour: these IRIs surface in the introspection artifact and in the run graph.
        """
        return self._mint(f"{parent}.derived-{suffix}")

    def component_node(self, parent, suffix: str) -> URIRef:
        """The IRI of a part of a node just minted: a frame's origin, a pose's relation."""
        return self._mint(f"{parent}-{suffix}")

    def child_node(self, parent, segment: str) -> URIRef:
        """The IRI one path segment below `parent`, which is how a namespace addresses its own."""
        return self._mint(f"{str(parent).rstrip('/')}/{segment}")

    @staticmethod
    def _mint(text: str) -> URIRef:
        """The one place an IRI becomes a node, so every minting rule reads the same."""
        return URIRef(text)

    def uri_rows(self) -> list[dict]:
        """``[{id, uri}]`` for every authored node then every derived entity.

        Appended, never substituted, and last-wins on a repeated id is deliberate: constraint
        names, metamodel predicates and aliases legitimately share a bare id.
        """
        authored = [
            {"id": id_, "uri": str(node)}
            for id_, node in sorted(self.id_nodes, key=lambda item: (item[0], str(item[1])))
            if isinstance(node, URIRef)
        ]
        derived = [{"id": id_, "uri": entry.uri} for id_, entry in sorted(self._derived.items())]

        return authored + derived

    def derivation_nodes(self) -> list[dict]:
        """Derivation-graph nodes: what each derived entity is, and what it came from."""
        return [
            {
                "id": entry.uri,
                "types": [self.graph.namespace_manager.normalizeUri(PROV.Entity), *entry.types],
                "relation": local_name(entry.relation),
                "parent": entry.parent,
            }
            for _, entry in sorted(self._derived.items())
        ]


def reader(func):
    """Memoize a graph reader on the model, keyed by the function and its arguments.

    Sound because `operations.normalize` is the graph's only writer and runs before any reader,
    so one cache serves the whole pass however many scopes read through it. Keyed by the
    function's qualified name as well as its arguments, since `position(node)` and
    `quantity(node)` would otherwise share a cache entry.

    Parameters:
        func: a reader `f(model, *args)` whose result depends only on the graph
    """

    @wraps(func)
    def call(model, *args):
        key = (func.__qualname__, *args)
        cache = model.cache
        if key not in cache:
            cache[key] = func(model, *args)

        return cache[key]

    return call
