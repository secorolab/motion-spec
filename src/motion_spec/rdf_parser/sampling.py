# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""What the run draws at startup: sampled quantities and shape scales.

The graph carries no numbers for these; the generated controller draws them once per run.
"""

from __future__ import annotations

import math
from hashlib import sha256

import numpy as np
from motion_spec_dsl.rdf_parser.vocab import QUDT_SCHEMA
from rdf_utils.constraints import ConstraintViolation
from rdf_utils.models.common import get_node_types
from rdf_utils.models.distribution import DistributionModel
from rdf_utils.models.vocab import (
    URI_DISTRIB_PRED_COV,
    URI_DISTRIB_PRED_FROM_DISTRIB,
    URI_DISTRIB_PRED_LOWER,
    URI_DISTRIB_PRED_MEAN,
    URI_DISTRIB_PRED_STD,
    URI_DISTRIB_PRED_UPPER,
    URI_DISTRIB_TYPE_NORMAL,
    URI_DISTRIB_TYPE_SAMPLED_QUANTITY,
    URI_DISTRIB_TYPE_UNIFORM,
    URI_GEOM_TYPE_VECTOR_XYZ,
)
from rdflib.namespace import RDF
from scene_dsl.rdf.ktree import SHAPES

from motion_spec.classes.sampling import SampledQuantity
from motion_spec.rdf_parser.model import identifier, local_name, seconds, si
from motion_spec.rdf_parser.quantities import is_duration


def unplaced_frames(tree: dict) -> list[dict]:
    """The frames scene-dsl could not place. Absent from the tree of a scene-dsl too old to say."""
    if "unplaced_frames" not in tree:
        raise ConstraintViolation(
            "kinematics",
            f"tree '{tree['name']}' does not report unplaced frames: this scene-dsl is older "
            f"than the one motion-spec needs to draw a frame's position",
        )
    return tree["unplaced_frames"]


def sampled_quantities(
    model, trees: list[dict], placements: set[str] = frozenset()
) -> list[SampledQuantity]:
    """Every sampled quantity and scaled shape, in graph order.

    PLACEMENTS are the drawn positions the scene places an object by: positions too, but on no
    tree, since the simulator places the object.
    """
    frames = {
        frame["position_coord_iri"]: (tree, frame)
        for tree in trees
        for frame in unplaced_frames(tree)
    }
    nodes = list(model.graph.subjects(RDF.type, URI_DISTRIB_TYPE_SAMPLED_QUANTITY))
    nodes.extend(
        shape for shape in model.graph.subjects(RDF.type, SHAPES.Scale)
        if model.graph.value(shape, URI_DISTRIB_PRED_FROM_DISTRIB) is not None
        and shape not in nodes
    )
    sampled = {str(node) for node in nodes}
    for coord in (coord for coord in frames if coord not in sampled):
        _, frame = frames[coord]
        raise ConstraintViolation(
            "sampling",
            f"frame '{frame['iri']}' has no position, and '{coord}' is not a sampled quantity, "
            f"so nothing places it",
        )
    result = []
    for node in nodes:
        distribution = model.graph.value(node, URI_DISTRIB_PRED_FROM_DISTRIB)
        dist, components = _components(DistributionModel(distribution, model.graph))
        unit = model.graph.value(node, QUDT_SCHEMA.unit)
        scale = seconds(1.0, unit) if is_duration(model, node) else si(1.0, unit)
        tree, frame = frames.get(str(node), (None, None))
        position = frame is not None or str(node) in placements
        shape_size = (
            next(model.graph.subjects(SHAPES["has-dimension"], node), None) is not None
            or (node, RDF.type, SHAPES.Scale) in model.graph
        )
        if shape_size and (
            dist != "uniform_real"
            or any(
                not math.isfinite(c["a"])
                or not math.isfinite(c["b"])
                or c["a"] <= 0
                or c["b"] <= c["a"]
                for c in components
            )
        ):
            raise ConstraintViolation(
                "sampling", f"Shape size '{node}' needs positive finite uniform bounds."
            )
        shape_vector = shape_size and bool(
            {URI_GEOM_TYPE_VECTOR_XYZ, SHAPES.Scale} & get_node_types(model.graph, node)
        )
        if not position and not shape_size and URI_GEOM_TYPE_VECTOR_XYZ in get_node_types(model.graph, node):
            raise ConstraintViolation(
                "sampling",
                f"'{node}' is a position with coordinates of its own and a distribution to draw "
                f"from: the scene places the frame, so nothing should draw it",
            )
        if len(components) != (3 if position or shape_vector else 1):
            raise ConstraintViolation(
                "sampling",
                f"'{node}' draws {len(components)} numbers from '{distribution}', which is not "
                f"{'a position or size vector' if position or shape_vector else 'a scalar'}",
            )
        result.append(
            SampledQuantity(
                id=shape_sample_id(node) if shape_size else model.id(node),
                uri=str(node),
                dist=dist,
                components=components,
                size=len(components),
                scale=scale,
                data_member=None if position or shape_size else model.id(node),
                segment=frame["name"] if frame else None,
                parent=frame["parent"] if frame else None,
                rotation=frame["rotation_xyzw"] if frame else None,
                tree=tree["cpp_name"] if tree else None,
            )
        )
    return result


def shape_sample_id(node) -> str:
    """A stable code name: scene namespaces need not be declared by the motion app."""
    return f"shape_{identifier(local_name(node))}_{sha256(str(node).encode()).hexdigest()[:12]}"


def _components(distribution: DistributionModel) -> tuple[str, list[dict]]:
    """The C++ distribution and its per-axis parameters: bounds, or mean and deviation."""
    if distribution.distrib_type == URI_DISTRIB_TYPE_UNIFORM:
        lower = distribution.get_attr(URI_DISTRIB_PRED_LOWER)
        upper = distribution.get_attr(URI_DISTRIB_PRED_UPPER)
        return "uniform_real", [{"a": float(a), "b": float(b)} for a, b in zip(lower, upper)]
    if distribution.distrib_type == URI_DISTRIB_TYPE_NORMAL:
        mean = distribution.get_attr(URI_DISTRIB_PRED_MEAN)
        if len(mean) == 1:
            deviations = [float(distribution.get_attr(URI_DISTRIB_PRED_STD))]
        else:
            covariance = np.asarray(distribution.get_attr(URI_DISTRIB_PRED_COV), dtype=float)
            if np.any(covariance != np.diag(np.diag(covariance))):
                raise ConstraintViolation(
                    "sampling",
                    f"'{distribution.id}' has a non-diagonal covariance; correlated components "
                    "are not drawn",
                )
            deviations = [math.sqrt(v) for v in np.diag(covariance)]
        return "normal", [{"a": float(m), "b": s} for m, s in zip(mean, deviations)]
    raise ConstraintViolation(
        "sampling", f"'{distribution.id}': only uniform and normal distributions are drawn"
    )
