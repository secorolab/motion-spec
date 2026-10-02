# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""The loaded model: the graph every other module reads through.

In order: the naming rules, the QUDT-to-SI conversions, ``Model``, and the reader cache.

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

import rdflib
from rdf_utils.constraints import ConstraintViolation
from rdf_utils.models.vocab import URI_QUDT_UNIT_CM, URI_QUDT_UNIT_M, URI_QUDT_UNIT_MM
from rdf_utils.namespace import NS_MM_QUDT_UNIT
from rdflib import URIRef
from rdflib.namespace import split_uri


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


class _DerivedIri(NamedTuple):
    """One registry row: the IRI minted for an id, and where it came from."""

    uri: str
    parent: str
    relation: object
    types: list


@dataclass
class Model:
    """The loaded model: the merged graph, the identity rules every reader shares, the registry of
    derived IRIs, and the cache those readers memoize on.
    """

    graph: rdflib.Dataset
    app_path: Path
    # The namespaces the application document declares: the nodes under them are the model's own.
    namespaces: tuple = ()
    cache: dict = field(default_factory=dict, repr=False)

    def __post_init__(self) -> None:
        self._ids: dict = {}
        # Each of the model's own ids, and the one node it names.
        self._app_nodes: dict[str, URIRef] = {}
        self._walked = False
        self._derived: dict[str, _DerivedIri] = {}
        # What IR generation adds to the graph, apart from what was authored: the union reads it,
        # and it is what the derivation document states.
        self.derived = self.graph.graph(URIRef(f"{Path(self.app_path).resolve().as_uri()}#derived"))

    # identity

    def id(self, node) -> str:
        """The generated id of a node.

        A node the application declares is named by its IRI below the namespace it is declared
        in -- `align/while/centred` is `align_while_centred` -- so the scope it is written in stays
        in its name. Any other node keeps its own local name: a scene element, an FSM state and a
        metamodel term are named by the tool or vocabulary they belong to.

        Returns:
            the id, or the node itself when it is a literal, a blank node or no node

        Raises:
            ConstraintViolation: two of the model's own nodes fold onto one id.
        """
        cached = self._ids.get(node)
        if cached is not None:
            return cached
        if not isinstance(node, URIRef):
            self._ids[node] = node
            return node
        iri = str(node)
        base = max(
            (ns for ns in self.namespaces if iri.startswith(ns) and iri != ns),
            key=len,
            default=None,
        )
        if base is None:
            local = self._qname(node)
            self._ids[node] = node if local is None else local
            return self._ids[node]
        id_ = identifier(iri[len(base) :])
        claimed = self._app_nodes.setdefault(id_, node)
        if claimed != node:
            raise ConstraintViolation(
                "motion-spec",
                f"'{claimed}' and '{iri}' both name the id '{id_}' in the generated code -- "
                "rename one of them in the model",
            )
        self._ids[node] = id_
        return id_

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

    # the id/IRI index

    @property
    def node_by_id(self) -> dict:
        """The node each of the model's own ids names.

        Every node is named once on first use, so an id is known before any reader asks for it;
        a node minted later is named when a reader first meets it.
        """
        if not self._walked:
            for node in set(self.graph.subjects()):
                self.id(node)
            self._walked = True
        return self._app_nodes

    # derived IRIs

    def iri_of(self, id_) -> str | None:
        """The IRI an id resolves to, authored or already derived; None when neither."""
        entry = self._derived.get(id_)
        if entry is not None:
            return entry.uri
        node = self.node_by_id.get(id_)
        return str(node) if node is not None else None

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
            ConstraintViolation: the id was already minted under a different IRI.
        """
        if not id_ or not parent_iri:
            return None
        # An authored node always wins: a derived IRI must never shadow a model's own.
        authored = self.node_by_id.get(id_)
        if authored is not None:
            return str(authored)
        uri = str(self.child_node(parent_iri, kebab(suffix)))
        existing = self._derived.get(id_)
        if existing is not None:
            if existing.uri != uri:
                raise ConstraintViolation(
                    "motion-spec",
                    f"id '{id_}' is minted for two derived entities:\n"
                    f"    {existing.uri}\n    {uri}\n"
                    "An id names one entity in the generated code, so the two would merge. Their "
                    "parents share a name the id keeps but the IRI does not -- rename one in the "
                    "model, or qualify the id by its parent's owner in Model.id.",
                )

            return uri
        self._derived[id_] = _DerivedIri(uri, parent_iri, relation, list(types))

        return uri

    def derived_node(self, parent, suffix: str) -> URIRef:
        """The IRI of a node ``operations.normalize`` materializes under `parent`.

        The other half of the derived-IRI rule: `register_derived` names an entity that has no
        node, this names one that will have triples of its own, so it needs no registration --
        it becomes a graph subject and so an authored IRI in `uri_rows`. The pattern is
        behaviour: these IRIs surface in the telemetry artifact and in the run graph.
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
        """``[{id, uri}]`` for every one of the model's own nodes, then every derived entity: one
        row per id, since an id names one entity."""
        authored = [{"id": id_, "uri": str(node)} for id_, node in self.node_by_id.items()]
        derived = [{"id": id_, "uri": entry.uri} for id_, entry in self._derived.items()]
        return authored + derived

    def derivation_nodes(self) -> list[dict]:
        """Derivation-graph nodes: what each derived entity is, and what it came from."""
        return [
            {
                "id": id_,
                "uri": entry.uri,
                "types": [str(type_) for type_ in entry.types],
                "relation": local_name(entry.relation),
                "parent": entry.parent,
            }
            for id_, entry in self._derived.items()
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
